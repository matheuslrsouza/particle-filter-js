import { splitByGear } from '../control/gearFollower'
import { DEFAULT_MPC, MpcFollower, type MpcParams } from '../control/mpc'
import { DEFAULT_EKF, Ekf } from '../localization/ekf'
import { wrapAngle } from '../math/angles'
import { DEFAULT_ODOMETRY, GPS_FILTER_STD, GPS_GRADES, Gps, readOdometry, type GpsGrade, type OdometryParams } from '../sensors/gps'
import { Rng } from '../math/rng'
import { clamp, type Pose, type Vec2 } from '../math/vec'
import { DEFAULT_HYBRID, hybridAStar, type HybridParams, type HybridResult, type PathPose } from '../planning/hybridAStar'
import { castRay } from '../sensors/lidar'
import { advancePose, type VehicleParams, type VehicleState } from '../vehicle/bicycle'
import type { Segment as WallSegment } from '../world/map'
import type { CityDef } from './cityDef'
import { drivableGrid } from './drivable'
import { buildLaneGraph, type LaneGraph } from './roadNetwork'
import { DEFAULT_ROUTING, locate, planRoute, pointAt, snapToLane, type LanePosition, type Route } from './routing'

export type CityPhase = 'idle' | 'driving' | 'stopping' | 'uturn' | 'arrived' | 'stuck'

/** A barricade across a whole road. */
export interface Barrier {
  segment: number
  /** footprint polygon */
  poly: Vec2[]
  /** faces seen by the lidar */
  faces: WallSegment[]
}

/** Hybrid A* for a real-size car: the footprint is four circles along the axis (rear axle at 0). */
export const CITY_HYBRID: HybridParams = {
  ...DEFAULT_HYBRID,
  wheelbase: 2.8,
  // maneuvers are planned at 85% of the steering limit, leaving the controller room to correct
  maxSteer: 0.6 * 0.85,
  step: 1,
  circleOffsets: [0.47, 1.32, 2.16, 3.01],
  radius: 1.25,
  reverseCost: 1.5,
  switchCost: 3,
  comfort: 1.6,
  wallCost: 0.5,
  goalTolerance: 0.7,
  maxExpansions: 30_000,
}

const LIDAR_ANGLES = Array.from({ length: 31 }, (_, i) => ((i - 15) / 15) * ((35 * Math.PI) / 180))
const LIDAR_RANGE = 50
/** the lidar sits on the front of the car */
const LIDAR_OFFSET = 3.5

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
  /** waypoints still to visit (a destination, or the remaining mission checkpoints) */
  targets: Vec2[] = []
  mission: { checkpoints: Vec2[]; reached: number } | null = null
  barriers: Barrier[] = []
  barrierVersion = 0
  /** lanes the router must avoid (blocked roads) */
  blockedLanes = new Set<number>()
  /** last forward lidar scan, in world coordinates (from the real car) */
  lidarRays: { from: Vec2; to: Vec2; hit: boolean }[] = []
  /** the last U-turn maneuver (world coordinates) */
  maneuver: { result: HybridResult; path: PathPose[]; origin: Vec2 } | null = null
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
  private stillTime = 0

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
    this.targets = []
    this.mission = null
    this.barriers = []
    this.blockedLanes.clear()
    this.maneuver = null
    this.lidarRays = []
    this.barrierVersion++
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
    const goal = snapToLane(this.graph, target)
    if (!goal) return false
    this.mission = null
    this.targets = [goal.point]
    if (this.planToTargets()) return true
    this.onMessage('No route to that point (one-way streets or outside the network).')
    return false
  }

  /**
   * A mission like the DARPA Urban Challenge MDF: checkpoints to visit in order, spread over
   * the city. The route goes through all of them without stopping.
   */
  startMission(count = 4): boolean {
    for (let attempt = 0; attempt < 20; attempt++) {
      const checkpoints: Vec2[] = []
      let from: Vec2 = this.estimate
      for (let tries = 0; checkpoints.length < count && tries < 200; tries++) {
        const l = this.graph.lanes[this.rng.int(this.graph.lanes.length)]
        if (l.length < 40) continue
        const p = pointAt(l.points, l.length / 2)
        if (Math.hypot(p.x - from.x, p.y - from.y) < 250) continue
        checkpoints.push(p)
        from = p
      }
      this.targets = [...checkpoints]
      this.mission = { checkpoints, reached: 0 }
      if (checkpoints.length === count && this.planToTargets()) {
        this.onMessage(`Mission: ${count} checkpoints`)
        return true
      }
    }
    this.mission = null
    return false
  }

  /**
   * Puts a barricade across a two-way road on the route, at least `ahead` meters in front of the
   * car (like the Urban Challenge blocked roads: the car must turn around and find another way).
   */
  placeBarrierAhead(ahead = 70): boolean {
    if (!this.route || this.phase !== 'driving') return false
    const path = this.route.path
    for (let k = 0; k < 40; k++) {
      const idx = this.routeIndex + Math.round((ahead + k * 10) / 0.5)
      if (idx >= path.length - 20) break
      const p = path[idx]
      const pos = snapToLane(this.graph, p, p.theta)
      if (!pos || Math.hypot(pos.point.x - p.x, pos.point.y - p.y) > 0.8) continue
      const lane = this.graph.lanes[pos.lane]
      const seg = this.graph.segments[lane.segment]
      // a two-way street (a U-turn across a median is impossible), with room before the barrier
      // to stop and turn around on the same street
      if (!seg.forward.length || !seg.backward.length) continue
      if (pos.s < 40 || pos.s > lane.length - 8) continue
      // barricade across the whole roadway (and a bit of sidewalk), facing the segment direction
      const c = projectOnPolyline(seg.center, p)
      const n = { x: -Math.sin(c.heading), y: Math.cos(c.heading) }
      const t = { x: Math.cos(c.heading), y: Math.sin(c.heading) }
      const r = seg.rightWidth + 0.5
      const l = seg.leftWidth + 0.5
      const corner = (side: number, along: number) => ({ x: c.point.x + n.x * side + t.x * along, y: c.point.y + n.y * side + t.y * along })
      const poly = [corner(r, -0.3), corner(r, 0.3), corner(-l, 0.3), corner(-l, -0.3)]
      this.barriers.push({ segment: seg.id, poly, faces: [{ a: poly[0], b: poly[3] }, { a: poly[1], b: poly[2] }] })
      for (const id of [...seg.forward, ...seg.backward]) this.blockedLanes.add(id)
      this.barrierVersion++
      this.onMessage('A road ahead was blocked (the car does not know yet).')
      return true
    }
    return false
  }

  /** Chains routes through the remaining targets, avoiding blocked lanes. */
  private planToTargets(): boolean {
    let start: LanePosition | null = locate(this.graph, this.estimate)
    if (!start || !this.targets.length) return false
    const path: PathPose[] = []
    const speedLimit: number[] = []
    const names: string[] = []
    for (const target of this.targets) {
      const goal = snapToLane(this.graph, target)
      if (!goal) return false
      const leg = planRoute(this.graph, start!, goal, DEFAULT_ROUTING, this.blockedLanes)
      if (!leg || leg.path.length < 2) return false
      const skip = path.length ? 1 : 0
      path.push(...leg.path.slice(skip))
      speedLimit.push(...leg.speedLimit.slice(skip))
      names.push(...leg.names.slice(skip))
      start = goal
    }
    if (path.length < 3) return false
    this.route = { steps: [], path, speedLimit, names, length: path.length * 0.5 }
    this.destination = this.targets[this.targets.length - 1]
    this.follower = new MpcFollower([{ dir: 1, points: path, speedLimit }], this.mpc)
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
    // default: brake to a stop and straighten the wheels
    let a = -Math.sign(car.speed) * Math.min(this.mpc.brake, Math.abs(car.speed) / dt)
    let w = -Math.sign(car.steer) * Math.min(Math.abs(car.steer) / dt, this.mpc.maxSteerRate)
    // the car's own sensors
    const odo = readOdometry(car.speed, car.steer, this.odometry, this.rng)
    this.localize(odo, dt)
    if (this.tickCount % 5 === 0) this.scanAhead()
    const f = this.follower
    if ((this.phase === 'driving' || this.phase === 'stopping' || this.phase === 'uturn') && f) {
      ;[a, w] = f.control(this.estimate, odo.speed, odo.steer, dt)
      if (this.phase === 'driving') {
        this.routeIndex = f.index
        this.checkpointProgress()
      }
      // a maneuver segment the car stopped close to (but not exactly at) counts as done
      if (this.phase === 'uturn') {
        this.stillTime = Math.abs(odo.speed) < 0.05 ? this.stillTime + dt : 0
        const end = f.gear.points[f.gear.points.length - 1]
        if (this.stillTime > 1.5 && Math.hypot(end.x - this.estimate.x, end.y - this.estimate.y) < 2) f.segmentDone = true
      }
      if (f.segmentDone) {
        this.stillTime = 0
        if (this.phase === 'driving') {
          this.phase = 'arrived'
          if (this.mission) this.mission.reached = this.mission.checkpoints.length
          this.onMessage(this.mission ? 'Mission complete!' : 'Destination reached!')
        } else if (this.phase === 'stopping') {
          this.planUturn()
        } else if (!f.done) {
          // U-turn: next forward/reverse segment
          f.next()
        } else {
          this.maneuver = null
          this.routeVersion++
          if (this.planToTargets()) this.onMessage('Rerouting around the blocked road.')
          else {
            this.phase = 'stuck'
            this.onMessage('No route avoiding the blocked road.')
          }
        }
      }
    }
    car.steer = clamp(car.steer + w * dt, -CITY_VEHICLE.maxSteer, CITY_VEHICLE.maxSteer)
    car.speed = car.speed + a * dt
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

  /** Marks mission checkpoints as reached when the car passes them. */
  private checkpointProgress(): void {
    if (!this.mission || this.targets.length < 2) return
    const t = this.targets[0]
    if (Math.hypot(t.x - this.estimate.x, t.y - this.estimate.y) < 6) {
      this.targets.shift()
      this.mission.reached++
      this.onMessage(`Checkpoint ${this.mission.reached}/${this.mission.checkpoints.length}`)
    }
  }

  /**
   * Forward lidar against the barricades. A hit on the route ahead makes the car stop before it;
   * the sensor is on the real car, the decision uses the estimated pose.
   */
  private scanAhead(): void {
    const car = this.car
    const faces = this.barriers.flatMap((b) => b.faces)
    const ox = car.x + Math.cos(car.theta) * LIDAR_OFFSET
    const oy = car.y + Math.sin(car.theta) * LIDAR_OFFSET
    this.lidarRays = []
    let nearest = Infinity
    let bearing = 0
    for (const ang of LIDAR_ANGLES) {
      const a = car.theta + ang
      const d = faces.length ? castRay(ox, oy, Math.cos(a), Math.sin(a), faces, LIDAR_RANGE) : LIDAR_RANGE
      this.lidarRays.push({ from: { x: ox, y: oy }, to: { x: ox + Math.cos(a) * d, y: oy + Math.sin(a) * d }, hit: d < LIDAR_RANGE })
      if (d < nearest) {
        nearest = d
        bearing = ang
      }
    }
    if (this.phase !== 'driving' || !this.route || nearest > 45) return
    // the obstacle in the frame of the estimated pose
    const e = this.estimate
    const hx = e.x + Math.cos(e.theta) * LIDAR_OFFSET + Math.cos(e.theta + bearing) * nearest
    const hy = e.y + Math.sin(e.theta) * LIDAR_OFFSET + Math.sin(e.theta + bearing) * nearest
    const path = this.route.path
    let hitIdx = -1
    let best = Infinity
    for (let i = this.routeIndex; i < Math.min(path.length, this.routeIndex + 140); i++) {
      const d = Math.hypot(path[i].x - hx, path[i].y - hy)
      if (d < best) {
        best = d
        hitIdx = i
      }
    }
    // only obstacles on the road ahead count
    if (hitIdx < 0 || best > 4) return
    const stopIdx = Math.max(this.routeIndex + 2, hitIdx - 16)
    this.follower = new MpcFollower(
      [{ dir: 1, points: path.slice(this.routeIndex, stopIdx + 1), speedLimit: this.route.speedLimit.slice(this.routeIndex, stopIdx + 1) }],
      this.mpc,
    )
    this.phase = 'stopping'
    this.onMessage(`Road blocked ${nearest.toFixed(0)} m ahead! Stopping.`)
  }

  /** U-turn with Hybrid A* on the local drivable surface, ending in the opposite lane. */
  private planUturn(): void {
    const e = this.estimate
    const opposite = snapToLane(this.graph, e, e.theta + Math.PI)
    if (!opposite) {
      this.phase = 'stuck'
      this.onMessage('No opposite lane to turn around into.')
      return
    }
    const grid = drivableGrid(this.graph, e, 60, 0.5, this.barriers.map((b) => b.poly))
    const o = grid.origin
    const lane = this.graph.lanes[opposite.lane]
    for (const extra of [8, 14, 20]) {
      const s = Math.min(lane.length - 1, opposite.s + extra)
      const p = pointAt(lane.points, s)
      const q = pointAt(lane.points, Math.min(lane.length, s + 1))
      const goal = { x: p.x - o.x, y: p.y - o.y, theta: Math.atan2(q.y - p.y, q.x - p.x), useHeading: true }
      const result = hybridAStar(grid.map, { x: e.x - o.x, y: e.y - o.y, theta: e.theta }, goal, CITY_HYBRID)
      if (!result) continue
      const path = result.path.map((q) => ({ ...q, x: q.x + o.x, y: q.y + o.y }))
      this.maneuver = { result, path, origin: o }
      this.follower = new MpcFollower(splitByGear(path), {
        ...this.mpc,
        forwardSpeed: 1.8,
        reverseSpeed: 1.2,
        aLat: 1.5,
        curvatureWindow: 0,
      })
      this.phase = 'uturn'
      this.routeVersion++
      this.onMessage('Turning around (Hybrid A*).')
      return
    }
    this.phase = 'stuck'
    this.onMessage('Could not find a U-turn here.')
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

/** Closest point of a polyline and the heading of the polyline there. */
function projectOnPolyline(pts: Vec2[], p: Vec2): { point: Vec2; heading: number } {
  let best = Infinity
  let out = { point: pts[0], heading: 0 }
  for (let i = 1; i < pts.length; i++) {
    const a = pts[i - 1]
    const b = pts[i]
    const dx = b.x - a.x
    const dy = b.y - a.y
    const len2 = dx * dx + dy * dy
    if (len2 === 0) continue
    const t = Math.max(0, Math.min(1, ((p.x - a.x) * dx + (p.y - a.y) * dy) / len2))
    const q = { x: a.x + dx * t, y: a.y + dy * t }
    const d = Math.hypot(p.x - q.x, p.y - q.y)
    if (d < best) {
      best = d
      out = { point: q, heading: Math.atan2(dy, dx) }
    }
  }
  return out
}
