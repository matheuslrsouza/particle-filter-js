import { DEFAULT_MPC, MpcFollower, type MpcParams } from '../control/mpc'
import { DEFAULT_EKF, Ekf } from '../localization/ekf'
import { wrapAngle } from '../math/angles'
import { DEFAULT_ODOMETRY, GPS_FILTER_STD, GPS_GRADES, Gps, readOdometry, type GpsGrade, type OdometryParams } from '../sensors/gps'
import { Rng } from '../math/rng'
import { clamp, type Pose, type Vec2 } from '../math/vec'
import { advancePose, type VehicleParams, type VehicleState } from '../vehicle/bicycle'
import type { CityDef } from './cityDef'
import { buildLaneGraph, type LaneGraph } from './roadNetwork'
import { locate, planRoute, pointAt, snapToLane, type Route } from './routing'

export type CityPhase = 'idle' | 'driving' | 'arrived'

/** What the car believes its pose is: the EKF fusion, one sensor alone, or the truth. */
export type Localization = 'ekf' | 'gps' | 'odometry' | 'truth'

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
  /** distance of the real car to the center of its lane (the route) */
  lateral: number
  /** distance between the estimated and the real pose */
  locError: number
}

export interface GpsMark {
  x: number
  y: number
  /** a multipath jump (simulated) */
  outlier: boolean
  /** rejected by the EKF gate */
  rejected: boolean
}

/**
 * Driving in the city: route on the lane graph to a destination, then follow it with the MPC
 * under the speed limits. The controller uses the pose estimated by an EKF that fuses a
 * consumer GPS with wheel odometry (or a single sensor, to compare).
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
  localization: Localization = 'ekf'
  odometry: OdometryParams = { ...DEFAULT_ODOMETRY }
  /** distance between the estimate and the real pose at the last tick */
  locError = 0
  private grade: GpsGrade = 'dual'
  readonly ekf = new Ekf({ ...DEFAULT_EKF })
  /** the pose the car believes, used by the controller */
  estimate!: Pose
  /** recent GPS fixes, for drawing */
  gpsMarks: GpsMark[] = []
  private gps!: Gps
  private deadReckoning!: Pose
  private gpsPose!: Pose
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
    this.gps = new Gps({ ...GPS_GRADES[this.grade] }, this.rng)
    this.ekf.p.gpsStd = GPS_FILTER_STD[this.grade]
    this.ekf.init(pose, 2, 0.05)
    this.estimate = { ...pose }
    this.deadReckoning = { ...pose }
    this.gpsPose = { ...pose }
    this.gpsMarks = []
    this.route = null
    this.follower = null
    this.destination = null
    this.telemetry = []
    this.phase = 'idle'
    this.routeVersion++
  }

  get gpsGrade(): GpsGrade {
    return this.grade
  }

  /** Switches the GPS receiver (and what the EKF assumes about it). */
  set gpsGrade(g: GpsGrade) {
    this.grade = g
    this.gps.p = { ...GPS_GRADES[g] }
    this.ekf.p.gpsStd = GPS_FILTER_STD[g]
  }

  /** Routes from the car to the lane point closest to `target`. */
  setDestination(target: Vec2): boolean {
    const start = locate(this.graph, this.estimate)
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
      if (Math.hypot(p.x - this.estimate.x, p.y - this.estimate.y) < 300) continue
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
    // the car's own sensors
    const odo = readOdometry(car.speed, car.steer, this.odometry, this.rng)
    this.localize(odo, dt)
    const f = this.follower
    if (this.phase === 'driving' && f) {
      ;[a, w] = f.control(this.estimate, odo.speed, odo.steer, dt)
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
        lateral: driving ? this.trueLateral() : 0,
        locError: this.locError,
      })
      if (this.telemetry.length > 300) this.telemetry.shift()
    }
  }

  private localize(odo: { speed: number; steer: number }, dt: number): void {
    const L = CITY_VEHICLE.wheelbase
    this.ekf.predict(odo.speed, odo.steer, dt)
    advancePose(this.deadReckoning, odo.speed, odo.steer, dt, L)
    // between fixes, GPS-only dead-reckons too (it has no better idea)
    advancePose(this.gpsPose, odo.speed, odo.steer, dt, L)
    const fix = this.gps.step(this.car, dt)
    if (fix) {
      const accepted = this.ekf.updateGps(fix)
      const moved = Math.hypot(fix.x - this.gpsPose.x, fix.y - this.gpsPose.y)
      const theta = moved > 0.8 && this.car.speed > 1 ? Math.atan2(fix.y - this.gpsPose.y, fix.x - this.gpsPose.x) : this.gpsPose.theta
      this.gpsPose = { x: fix.x, y: fix.y, theta: this.gpsPose.theta + wrapAngle(theta - this.gpsPose.theta) * 0.3 }
      this.gpsMarks.push({ x: fix.x, y: fix.y, outlier: fix.outlier, rejected: !accepted })
      if (this.gpsMarks.length > 60) this.gpsMarks.shift()
    }
    const m = this.localization
    this.estimate = m === 'ekf' ? this.ekf.pose : m === 'gps' ? { ...this.gpsPose } : m === 'odometry' ? { ...this.deadReckoning } : { x: this.car.x, y: this.car.y, theta: this.car.theta }
    this.locError = Math.hypot(this.estimate.x - this.car.x, this.estimate.y - this.car.y)
  }

  /** Signed distance of the real car to the route (the center of its lane). */
  private trueLateral(): number {
    const path = this.route!.path
    const i = Math.min(this.routeIndex, path.length - 2)
    let best = Infinity
    let signed = 0
    for (let k = Math.max(0, i - 20); k < Math.min(path.length - 1, i + 20); k++) {
      const a = path[k]
      const b = path[k + 1]
      const dx = b.x - a.x
      const dy = b.y - a.y
      const len2 = dx * dx + dy * dy || 1
      const t = Math.max(0, Math.min(1, ((this.car.x - a.x) * dx + (this.car.y - a.y) * dy) / len2))
      const px = a.x + dx * t
      const py = a.y + dy * t
      const d = Math.hypot(this.car.x - px, this.car.y - py)
      if (d < best) {
        best = d
        signed = Math.sign((this.car.y - a.y) * dx - (this.car.x - a.x) * dy) * d
      }
    }
    return signed
  }

  /** Street the car is on (from the route), or ''. */
  get street(): string {
    if (!this.route) return ''
    return this.route.names[Math.min(this.routeIndex, this.route.names.length - 1)] ?? ''
  }

  /** Remaining distance along the route (m). */
  get remaining(): number {
    if (!this.route || !this.follower) return 0
    return Math.max(0, this.follower.distanceToSegmentEnd(this.estimate))
  }
}
