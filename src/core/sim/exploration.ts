import { GearFollower, splitByGear } from '../control/gearFollower'
import { DEFAULT_MPC, MpcFollower, type MpcParams } from '../control/mpc'
import type { PidGains } from '../control/pid'
import { deg2rad, wrapAngle } from '../math/angles'
import { Rng } from '../math/rng'
import { clamp, type Pose, type Vec2 } from '../math/vec'
import { DEFAULT_HYBRID, footprintClearance, hybridAStar, poseIsFree, type HybridParams, type HybridResult, type PathPose } from '../planning/hybridAStar'
import { DEFAULT_SMOOTH, PLAN_VALIDITY_RATIO, smoothHybridPath, type HybridSmoothOptions } from '../planning/smoothHybrid'
import { evenlySpacedBeams, measure, type LidarConfig } from '../sensors/lidar'
import { advancePose, steerToward, type VehicleState } from '../vehicle/bicycle'
import { OccupancyGrid, WALL } from '../world/grid'
import { KnownMap, type MappingMode } from '../world/knownMap'
import { loadWorld, type MapDef, type Segment, type World } from '../world/map'
import { VEHICLE } from './simulation'

const CAR_HALF_WIDTH = 0.22

/** Shortest distance from a point to any wall segment. */
function distanceToWalls(x: number, y: number, walls: Segment[]): number {
  let best = Infinity
  for (const w of walls) {
    const ex = w.b.x - w.a.x
    const ey = w.b.y - w.a.y
    const len2 = ex * ex + ey * ey
    const t = len2 > 0 ? Math.max(0, Math.min(1, ((x - w.a.x) * ex + (y - w.a.y) * ey) / len2)) : 0
    best = Math.min(best, Math.hypot(x - (w.a.x + ex * t), y - (w.a.y + ey * t)))
  }
  return best
}

export type ExplorePhase = 'idle' | 'driving' | 'switching' | 'recovering' | 'arrived' | 'blocked'

export interface ExploreGoal extends Pose {
  useHeading: boolean
}

export type Controller = 'mpc' | 'pid'

export interface ExploreParams {
  /** MPC (iLQR) with a speed profile, or the PID of the course with constant speed */
  controller: Controller
  forwardSpeed: number
  reverseSpeed: number
  gains: PidGains
  lookahead: number
  lidarRange: number
  /** std of the GPS position (m) and heading (rad) */
  gpsNoise: number
  gpsHeadingNoise: number
  sensorHz: number
  /** occupancy grid: probabilistic (log-odds) or one hit = wall forever (binary) */
  mapping: MappingMode
  /** probability of a bogus short lidar reading per beam */
  lidarSpurious: number
}

export const DEFAULT_EXPLORE: ExploreParams = {
  controller: 'mpc',
  forwardSpeed: 1.2,
  reverseSpeed: 0.6,
  gains: { kp: 2.5, ki: 0.05, kd: 0.6 },
  lookahead: 0.5,
  lidarRange: 3,
  gpsNoise: 0.02,
  gpsHeadingNoise: deg2rad(0.5),
  sensorHz: 10,
  mapping: 'logodds',
  lidarSpurious: 0.01,
}

export interface ExplorePlan {
  result: HybridResult
  /** the path the car follows (smoothed) */
  path: PathPose[]
  /** the raw Hybrid A* path, made of discrete arcs */
  raw: PathPose[]
}

/**
 * Exploration of an unknown map, like the "A* in action" video of the Udacity course:
 * the car only knows its goal. Unknown space is assumed free, the lidar discovers walls,
 * and Hybrid A* replans whenever the current plan runs into one.
 *
 * Localization is a high-precision GPS (true pose + small noise), dead-reckoned between fixes.
 * The particle filter can't be used here because it needs the map the car doesn't have yet.
 */
export class ExplorationSim {
  readonly dt = 0.02
  readonly world: World
  /** true occupancy, only used to detect real collisions */
  readonly truth: OccupancyGrid
  readonly known: KnownMap
  readonly rng: Rng
  lidar: LidarConfig
  hybrid: HybridParams = { ...DEFAULT_HYBRID }
  smoothing: HybridSmoothOptions & { enabled: boolean } = { ...DEFAULT_SMOOTH, enabled: true }
  mpc: MpcParams = structuredClone(DEFAULT_MPC)

  car!: VehicleState
  prevCar!: Pose
  /** pose estimated from GPS fixes + dead reckoning */
  estimate!: Pose
  goal: ExploreGoal | null = null
  plan: ExplorePlan | null = null
  previousPlan: PathPose[] | null = null
  follower: GearFollower | MpcFollower | null = null
  phase: ExplorePhase = 'idle'
  phaseTime = 0
  time = 0
  replans = 0
  lastScan: Float64Array | null = null
  lastScanPose: Pose | null = null
  /** newly discovered wall cells since the renderer last drained them */
  discovered: number[] = []
  /** wall cells that evidence turned back into free space (log-odds), drained by the renderer */
  removedWalls: number[] = []
  planVersion = 0
  /** car poses stamped every 12 cm, drained by the renderer (the green footprint trail) */
  footprints: Pose[] = []
  private lastFootprint: Pose | null = null
  /** speed the controller is aiming for (reference), for the chart */
  refSpeed = 0
  /** bumps on every lidar scan */
  scanVersion = 0
  telemetry: { t: number; speed: number; refSpeed: number; cte: number }[] = []
  onMessage: (msg: string) => void = () => {}

  private tickCount = 0
  private escape: { dir: number; steer: number; driven: number } | null = null
  private triedEscapes: { dir: number; steer: number }[] = []
  private lastReplan = -Infinity

  constructor(
    mapDef: MapDef,
    public params: ExploreParams = structuredClone(DEFAULT_EXPLORE),
    seed = Date.now(),
  ) {
    this.rng = new Rng(seed)
    this.world = loadWorld(mapDef)
    this.truth = new OccupancyGrid(this.world, 0.25, 0)
    this.known = new KnownMap(this.world.width, this.world.height, 0.25)
    this.lidar = { angles: evenlySpacedBeams(72), maxRange: this.params.lidarRange, noise: 0.02 }
    this.reset()
  }

  reset(): void {
    const s = this.world.start
    this.car = { ...s, speed: 0, steer: 0, odometer: 0 }
    this.prevCar = { ...s }
    this.estimate = { ...s }
    this.known.clear()
    this.removedWalls = []
    this.discovered = []
    this.footprints = []
    this.lastFootprint = null
    this.goal = null
    this.clearPlan()
    this.previousPlan = null
    this.replans = 0
    this.telemetry = []
    this.setPhase('idle')
    this.sense()
  }

  setGoal(goal: ExploreGoal): boolean {
    if (goal.x < 0 || goal.y < 0 || goal.x > this.world.width || goal.y > this.world.height) return false
    this.goal = goal
    this.previousPlan = null
    return this.replan('new goal')
  }

  /** Replans right away (e.g. after switching controllers in the panel). */
  forceReplan(): void {
    if (!this.goal || this.phase === 'arrived') return
    this.lastReplan = -Infinity
    this.replan('forced')
  }

  tick(): void {
    const dt = this.dt
    this.prevCar = { x: this.car.x, y: this.car.y, theta: this.car.theta }
    this.time += dt
    this.phaseTime += dt
    this.tickCount++

    let speed = 0
    let steer = this.car.steer
    /** [acceleration, steering rate] when the MPC drives; otherwise speed/steer commands */
    let mpcCmd: [number, number] | null = null
    const f = this.follower
    // walls discovered later may open a new way: retry now and then
    if (this.phase === 'blocked' && this.phaseTime > 2) this.replan('retry')
    if (this.phase === 'switching' && this.phaseTime > 0.4) this.setPhase('driving')
    if (this.phase === 'recovering') {
      // recovery behavior after a bump: set the wheels for the chosen escape maneuver, drive it
      // until there is room again, then plan from that safer pose
      if (!this.escape) this.escape = this.chooseEscape()
      const e = this.escape
      steer = e.steer
      const ready = Math.abs(this.car.steer - e.steer) < 0.05
      speed = ready ? e.dir * Math.min(0.5, this.params.reverseSpeed) : 0
      if (ready) e.driven += Math.abs(speed) * dt
      const roomy = footprintClearance(this.known, this.estimate, this.hybrid) >= this.hybrid.radius
      if ((roomy && e.driven > 0.1) || e.driven >= 0.6 || this.phaseTime > 3) {
        speed = 0
        this.escape = null
        this.triedEscapes = []
        this.lastReplan = -Infinity
        this.replan('after bump')
      }
    }
    if (this.phase === 'driving' && f instanceof MpcFollower) {
      mpcCmd = f.control(this.estimate, this.car.speed, this.car.steer, dt)
      this.refSpeed = f.refSpeed
      if (f.segmentDone) {
        mpcCmd = null
        if (f.done) {
          this.arriveOrReplan()
        } else {
          f.next()
          this.setPhase('switching')
        }
      } else if (Math.abs(f.cte) > 0.7) {
        this.replan('drifted off the path')
      }
    } else if (this.phase === 'driving' && f instanceof GearFollower) {
      steer = f.steer(this.estimate, dt)
      const gearSpeed = f.gear.dir > 0 ? this.params.forwardSpeed : this.params.reverseSpeed
      // slow down before stopping to switch gears or at the goal
      const slow = clamp(f.distanceToSegmentEnd(this.estimate) / 0.8, 0.3, 1)
      speed = f.gear.dir * gearSpeed * slow
      this.refSpeed = speed
      if (f.segmentDone) {
        speed = 0
        if (f.done) {
          this.arriveOrReplan()
        } else {
          f.next()
          this.setPhase('switching')
        }
      } else if (Math.abs(f.cte) > 0.7) {
        this.replan('drifted off the path')
      }
    }

    const car = this.car
    if (mpcCmd) {
      // the MPC commands acceleration and steering rate, like real actuators
      car.steer = clamp(car.steer + mpcCmd[1] * dt, -VEHICLE.maxSteer, VEHICLE.maxSteer)
      car.speed = clamp(car.speed + mpcCmd[0] * dt, -this.params.reverseSpeed * 1.5, this.params.forwardSpeed * 1.5)
    } else {
      steerToward(car, steer, dt, VEHICLE)
      car.speed = speed
    }
    const v = car.speed ? this.rng.gaussian(car.speed, Math.abs(car.speed) * 0.03) : 0
    advancePose(car, v, car.steer, dt, VEHICLE.wheelbase)
    car.odometer += v * dt
    const contact = this.contactCells(car, this.prevCar)
    if (contact.length) {
      Object.assign(car, this.prevCar, { speed: 0 })
      // bumper sensor: thin wall ends can slip between lidar beams, so walls touched are mapped too
      const fresh = this.known.markWalls(contact)
      this.discovered.push(...fresh)
      if (this.phase === 'driving' || this.phase === 'switching') {
        this.onMessage('Bump! Backing off and replanning.')
        this.escape = null
        this.triedEscapes = []
        this.setPhase('recovering')
      } else if (this.phase === 'recovering' && this.escape) {
        // this escape is blocked too: try the next best one (the bumper just mapped the wall)
        this.triedEscapes.push(this.escape)
        this.escape = null
        if (this.triedEscapes.length >= 6) {
          this.triedEscapes = []
          this.lastReplan = -Infinity
          this.replan('stuck')
        }
      }
    }

    const lf = this.lastFootprint
    if (!lf || Math.hypot(car.x - lf.x, car.y - lf.y) >= 0.12) {
      this.lastFootprint = { x: car.x, y: car.y, theta: car.theta }
      this.footprints.push(this.lastFootprint)
    }

    // dead reckoning between GPS fixes, with the wheel speed and steering angle
    advancePose(this.estimate, car.speed, car.steer, dt, VEHICLE.wheelbase)

    const every = Math.max(1, Math.round(1 / (this.params.sensorHz * dt)))
    if (this.tickCount % every === 0) this.sense()
  }

  private sense(): void {
    const car = this.car
    // GPS fix
    this.estimate = {
      x: this.rng.gaussian(car.x, this.params.gpsNoise),
      y: this.rng.gaussian(car.y, this.params.gpsNoise),
      theta: wrapAngle(this.rng.gaussian(car.theta, this.params.gpsHeadingNoise)),
    }
    this.lidar.maxRange = this.params.lidarRange
    this.lidar.spurious = this.params.lidarSpurious
    this.known.mode = this.params.mapping
    const z = measure(car, this.lidar, this.world.walls, this.rng)
    this.lastScan = z
    this.lastScanPose = { x: car.x, y: car.y, theta: car.theta }
    this.scanVersion++
    const fresh = this.known.integrateScan(this.estimate, z, this.lidar)
    if (this.known.lastRemoved.length) this.removedWalls.push(...this.known.lastRemoved)
    if (fresh.length) {
      this.discovered.push(...fresh)
      if (this.plan && (this.phase === 'driving' || this.phase === 'switching') && !this.planStillValid()) {
        this.replan('new wall on the path')
      }
    }
    const driving = this.phase === 'driving' && this.follower
    this.telemetry.push({ t: this.time, speed: car.speed, refSpeed: driving ? this.refSpeed : 0, cte: driving ? this.follower!.cte : 0 })
    if (this.telemetry.length > 300) this.telemetry.shift()
  }

  /** Checks the remaining part of the plan against the walls discovered so far. */
  private planStillValid(): boolean {
    const f = this.follower
    if (!f || !this.plan) return true
    const relaxed = { ...this.hybrid, radius: this.hybrid.radius * PLAN_VALIDITY_RATIO }
    // skip the poses the car already drove past
    let skip = 0
    for (let s = 0; s < f.segment; s++) skip += f.gears[s].points.length - 1
    skip += f.index
    const path = this.plan.path
    for (let i = skip + 3; i < path.length; i += 2) if (!poseIsFree(this.known, path[i], relaxed)) return false
    return true
  }

  private replan(reason: string): boolean {
    if (!this.goal) return false
    // never replan more than ~5x per second
    if (this.time - this.lastReplan < 0.2 && reason !== 'new goal') return true
    this.lastReplan = this.time
    const result = hybridAStar(this.known, this.estimate, this.goal, this.hybrid)
    if (this.plan) this.previousPlan = this.plan.path
    if (!result) {
      this.clearPlan()
      if (this.phase !== 'blocked') this.onMessage('No path to the goal with what the car knows.')
      this.setPhase('blocked')
      return false
    }
    const path = this.smoothing.enabled ? smoothHybridPath(result.path, this.known, this.hybrid, this.smoothing) : result.path
    this.plan = { result, path, raw: result.path }
    const gears = splitByGear(path)
    if (this.params.controller === 'mpc') {
      Object.assign(this.mpc, { forwardSpeed: this.params.forwardSpeed, reverseSpeed: this.params.reverseSpeed })
      this.follower = new MpcFollower(gears, this.mpc)
    } else {
      this.follower = new GearFollower(gears, this.params.gains, this.params.lookahead)
    }
    this.planVersion++
    if (reason !== 'new goal') this.replans++
    if (this.follower.gears.length === 0) {
      this.setPhase('arrived')
      return true
    }
    // stop briefly if the new plan starts in the opposite direction
    this.setPhase(Math.sign(this.car.speed) * this.follower.gear.dir < 0 ? 'switching' : 'driving')
    return true
  }

  private arriveOrReplan(): void {
    const g = this.goal!
    const close = Math.hypot(g.x - this.estimate.x, g.y - this.estimate.y) < 0.5
    if (close) {
      this.setPhase('arrived')
      this.onMessage('Goal reached!')
    } else {
      this.replan('end of plan')
    }
  }

  /** True wall cells touching the car body (empty when there is no collision). */
  /**
   * Picks the short maneuver (forward/reverse × left/straight/right) that gains the most room
   * according to the map the car knows (lidar + bumper), skipping the ones already tried.
   */
  private chooseEscape(): { dir: number; steer: number; driven: number } {
    let best = { dir: -1, steer: 0, driven: 0 }
    let bestScore = -Infinity
    for (const dir of [-1, 1]) {
      for (const steer of [0, VEHICLE.maxSteer, -VEHICLE.maxSteer]) {
        if (this.triedEscapes.some((t) => t.dir === dir && t.steer === steer)) continue
        const p = { ...this.estimate }
        let worst = Infinity
        for (let d = 0; d < 0.5; d += 0.05) {
          advancePose(p, dir, steer, 0.05, VEHICLE.wheelbase)
          worst = Math.min(worst, footprintClearance(this.known, p, this.hybrid))
        }
        // prefer reverse a little: it's the natural way out of a frontal bump
        const score = worst + footprintClearance(this.known, p, this.hybrid) + (dir < 0 ? 0.02 : 0)
        if (score > bestScore) {
          bestScore = score
          best = { dir, steer, driven: 0 }
        }
      }
    }
    return best
  }

  /** Smallest distance from the car body circles to the real walls. */
  private bodyClearance(p: Pose): number {
    let min = Infinity
    for (const o of [0, 0.31, 0.62]) {
      min = Math.min(min, distanceToWalls(p.x + Math.cos(p.theta) * o, p.y + Math.sin(p.theta) * o, this.world.walls))
    }
    return min
  }

  /**
   * True wall cells touching the car body when the last move pushed it into a wall (empty
   * otherwise). A car already touching a wall can always move away from it.
   */
  private contactCells(p: Pose, prev: Pose): number[] {
    const now = this.bodyClearance(p)
    if (now >= CAR_HALF_WIDTH || now >= this.bodyClearance(prev) - 1e-6) return []
    const g = this.truth
    const cells: number[] = []
    for (const o of [0, 0.31, 0.62]) {
      const x = p.x + Math.cos(p.theta) * o
      const y = p.y + Math.sin(p.theta) * o
      if (distanceToWalls(x, y, this.world.walls) >= CAR_HALF_WIDTH) continue
      const [c, r] = g.worldToCell({ x, y })
      for (let dr = -2; dr <= 2; dr++) {
        for (let dc = -2; dc <= 2; dc++) {
          if (g.inBounds(c + dc, r + dr) && g.cells[g.index(c + dc, r + dr)] === WALL) cells.push(g.index(c + dc, r + dr))
        }
      }
    }
    return cells
  }

  private clearPlan(): void {
    this.plan = null
    this.follower = null
    this.planVersion++
  }

  private setPhase(p: ExplorePhase): void {
    this.phase = p
    this.phaseTime = 0
  }

  /** A random reachable-looking goal far from the car (uses the true map). */
  randomGoal(): void {
    for (let i = 0; i < 200; i++) {
      const c = this.rng.int(this.truth.cols)
      const r = this.rng.int(this.truth.rows)
      const idx = this.truth.index(c, r)
      if (this.truth.clearance[idx] < 3) continue
      const p: Vec2 = this.truth.cellCenter(c, r)
      if (Math.hypot(p.x - this.car.x, p.y - this.car.y) < 8) continue
      this.setGoal({ ...p, theta: 0, useHeading: false })
      return
    }
  }
}
