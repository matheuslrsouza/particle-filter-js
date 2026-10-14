import { DEFAULT_MPC, MpcFollower, type MpcParams } from '../control/mpc'
import { Rng } from '../math/rng'
import { clamp, type Pose, type Vec2 } from '../math/vec'
import { advancePose, type VehicleParams, type VehicleState } from '../vehicle/bicycle'
import type { CityDef } from './cityDef'
import { buildLaneGraph, type LaneGraph } from './roadNetwork'
import { locate, planRoute, pointAt, snapToLane, type Route } from './routing'

export type CityPhase = 'idle' | 'driving' | 'arrived'

/** A real-size car (the maze car is 1:5.6). */
export const CITY_VEHICLE: VehicleParams = { wheelbase: 2.8, maxSteer: 0.6, maxSteerRate: 0.8 }

export const CITY_MPC: MpcParams = {
  ...DEFAULT_MPC,
  horizon: 25,
  dt: 0.12,
  wheelbase: CITY_VEHICLE.wheelbase,
  maxSteer: CITY_VEHICLE.maxSteer,
  maxSteerRate: CITY_VEHICLE.maxSteerRate,
  accel: 2,
  brake: 4,
  aLat: 2.5,
  // the speed limits of the streets cap this
  forwardSpeed: 20,
  // mapped polylines have small kinks at their nodes: measure curvature over a few meters
  curvatureWindow: 4,
  weights: { ...DEFAULT_MPC.weights, lateral: 20, heading: 15, speed: 2, steerRate: 2 },
}

export interface CityTelemetry {
  t: number
  speed: number
  limit: number
  /** distance to the center of the lane (the route) */
  lateral: number
}

/**
 * Driving in the city: route on the lane graph to a destination, then follow it with the MPC
 * under the speed limits. Localization is the true pose for now (an EKF comes later).
 */
export class CitySim {
  readonly dt = 0.02
  readonly graph: LaneGraph
  readonly rng: Rng
  car!: VehicleState
  prevCar!: Pose
  route: Route | null = null
  follower: MpcFollower | null = null
  destination: Vec2 | null = null
  phase: CityPhase = 'idle'
  time = 0
  routeVersion = 0
  telemetry: CityTelemetry[] = []
  /** index of the closest route point, for the HUD */
  routeIndex = 0
  onMessage: (msg: string) => void = () => {}
  mpc: MpcParams = structuredClone(CITY_MPC)
  private tickCount = 0

  constructor(
    readonly city: CityDef,
    seed = Date.now(),
  ) {
    this.rng = new Rng(seed)
    this.graph = buildLaneGraph(city)
    this.reset()
  }

  /** Puts the car on Castro Street (or the longest lane), 10 m into it. */
  reset(): void {
    const lanes = this.graph.lanes.filter((l) => l.length > 30)
    const castro = lanes.filter((l) => l.name === 'Castro Street').sort((a, b) => b.length - a.length)[0]
    const lane = castro ?? lanes.sort((a, b) => b.length - a.length)[0]
    const p = pointAt(lane.points, 10)
    const q = pointAt(lane.points, 11)
    const pose = { x: p.x, y: p.y, theta: Math.atan2(q.y - p.y, q.x - p.x) }
    this.car = { ...pose, speed: 0, steer: 0, odometer: 0 }
    this.prevCar = { ...pose }
    this.route = null
    this.follower = null
    this.destination = null
    this.telemetry = []
    this.phase = 'idle'
    this.routeVersion++
  }

  /** Routes from the car to the lane point closest to `target`. */
  setDestination(target: Vec2): boolean {
    const start = locate(this.graph, this.car)
    const goal = snapToLane(this.graph, target)
    if (!start || !goal) return false
    const route = planRoute(this.graph, start, goal)
    if (!route || route.path.length < 3) {
      this.onMessage('No route to that point (one-way streets or outside the network).')
      return false
    }
    this.route = route
    this.destination = goal.point
    this.follower = new MpcFollower([{ dir: 1, points: route.path, speedLimit: route.speedLimit }], this.mpc)
    this.routeIndex = 0
    this.phase = 'driving'
    this.routeVersion++
    return true
  }

  /** A reachable destination at least 300 m away. */
  randomDestination(): boolean {
    for (let i = 0; i < 40; i++) {
      const l = this.graph.lanes[this.rng.int(this.graph.lanes.length)]
      if (l.length < 20) continue
      const p = pointAt(l.points, l.length / 2)
      if (Math.hypot(p.x - this.car.x, p.y - this.car.y) < 300) continue
      if (this.setDestination(p)) return true
    }
    return false
  }

  tick(): void {
    const dt = this.dt
    this.prevCar = { x: this.car.x, y: this.car.y, theta: this.car.theta }
    this.time += dt
    this.tickCount++
    const car = this.car
    let a = car.speed > 0 ? -this.mpc.brake : 0
    let w = -Math.sign(car.steer) * Math.min(Math.abs(car.steer) / dt, this.mpc.maxSteerRate)
    const f = this.follower
    if (this.phase === 'driving' && f) {
      ;[a, w] = f.control(car, car.speed, car.steer, dt)
      this.routeIndex = f.index
      if (f.segmentDone) {
        this.phase = 'arrived'
        this.onMessage('Destination reached!')
      }
    }
    car.steer = clamp(car.steer + w * dt, -CITY_VEHICLE.maxSteer, CITY_VEHICLE.maxSteer)
    car.speed = Math.max(0, car.speed + a * dt)
    advancePose(car, car.speed, car.steer, dt, CITY_VEHICLE.wheelbase)
    car.odometer += car.speed * dt

    if (this.tickCount % 5 === 0) {
      const driving = this.phase === 'driving' && f && this.route
      this.telemetry.push({
        t: this.time,
        speed: car.speed,
        limit: driving ? this.route!.speedLimit[Math.min(this.routeIndex, this.route!.speedLimit.length - 1)] : 0,
        lateral: driving ? f!.cte : 0,
      })
      if (this.telemetry.length > 300) this.telemetry.shift()
    }
  }

  /** Street the car is on (from the route), or ''. */
  get street(): string {
    if (!this.route) return ''
    return this.route.names[Math.min(this.routeIndex, this.route.names.length - 1)] ?? ''
  }

  /** Remaining distance along the route (m). */
  get remaining(): number {
    if (!this.route || !this.follower) return 0
    return Math.max(0, this.follower.distanceToSegmentEnd(this.car))
  }
}
