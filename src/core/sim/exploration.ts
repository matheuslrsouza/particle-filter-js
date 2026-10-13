import { GearFollower, splitByGear } from '../control/gearFollower'
import type { PidGains } from '../control/pid'
import { deg2rad, wrapAngle } from '../math/angles'
import { Rng } from '../math/rng'
import { clamp, type Pose, type Vec2 } from '../math/vec'
import { DEFAULT_HYBRID, hybridAStar, poseIsFree, type HybridParams, type HybridResult, type PathPose } from '../planning/hybridAStar'
import { DEFAULT_SMOOTH, PLAN_VALIDITY_RATIO, smoothHybridPath, type HybridSmoothOptions } from '../planning/smoothHybrid'
import { evenlySpacedBeams, measure, type LidarConfig } from '../sensors/lidar'
import { advancePose, steerToward, type VehicleState } from '../vehicle/bicycle'
import { OccupancyGrid, WALL } from '../world/grid'
import { KnownMap } from '../world/knownMap'
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

export interface ExploreParams {
  forwardSpeed: number
  reverseSpeed: number
  gains: PidGains
  lookahead: number
  lidarRange: number
  /** std of the GPS position (m) and heading (rad) */
  gpsNoise: number
  gpsHeadingNoise: number
  sensorHz: number
}

export const DEFAULT_EXPLORE: ExploreParams = {
  forwardSpeed: 1.2,
  reverseSpeed: 0.6,
  gains: { kp: 2.5, ki: 0.05, kd: 0.6 },
  lookahead: 0.5,
  lidarRange: 3,
  gpsNoise: 0.02,
  gpsHeadingNoise: deg2rad(0.5),
  sensorHz: 10,
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

  car!: VehicleState
  prevCar!: Pose
  /** pose estimated from GPS fixes + dead reckoning */
  estimate!: Pose
  goal: ExploreGoal | null = null
  plan: ExplorePlan | null = null
  previousPlan: PathPose[] | null = null
  follower: GearFollower | null = null
  phase: ExplorePhase = 'idle'
  phaseTime = 0
  time = 0
  replans = 0
  lastScan: Float64Array | null = null
  lastScanPose: Pose | null = null
  /** newly discovered wall cells since the renderer last drained them */
  discovered: number[] = []
  planVersion = 0
  /** car poses stamped every 12 cm, drained by the renderer (the green footprint trail) */
  footprints: Pose[] = []
  private lastFootprint: Pose | null = null
  /** bumps on every lidar scan */
  scanVersion = 0
  telemetry: { t: number; speed: number; cte: number }[] = []
  onMessage: (msg: string) => void = () => {}

  private tickCount = 0
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
    this.known.state.fill(0)
    this.known.dist.fill(Infinity)
    this.known.version++
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

  tick(): void {
    const dt = this.dt
    this.prevCar = { x: this.car.x, y: this.car.y, theta: this.car.theta }
    this.time += dt
    this.phaseTime += dt
    this.tickCount++

    let speed = 0
    let steer = this.car.steer
    const f = this.follower
    // walls discovered later may open a new way: retry now and then
    if (this.phase === 'blocked' && this.phaseTime > 2) this.replan('retry')
    if (this.phase === 'switching' && this.phaseTime > 0.4) this.setPhase('driving')
    if (this.phase === 'recovering') {
      // after a bump: back off in a straight line, then plan again from a safer pose
      speed = -this.params.reverseSpeed
      steer = 0
      if (this.phaseTime > 0.8) {
        speed = 0
        this.lastReplan = -Infinity
        this.replan('after bump')
      }
    }
    if (this.phase === 'driving' && f) {
      steer = f.steer(this.estimate, dt)
      const gearSpeed = f.gear.dir > 0 ? this.params.forwardSpeed : this.params.reverseSpeed
      // slow down before stopping to switch gears or at the goal
      const slow = clamp(f.distanceToSegmentEnd(this.estimate) / 0.8, 0.3, 1)
      speed = f.gear.dir * gearSpeed * slow
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
    steerToward(car, steer, dt, VEHICLE)
    car.speed = speed
    const v = speed ? this.rng.gaussian(speed, Math.abs(speed) * 0.03) : 0
    advancePose(car, v, car.steer, dt, VEHICLE.wheelbase)
    car.odometer += v * dt
    const contact = this.contactCells(car)
    if (contact.length) {
      Object.assign(car, this.prevCar, { speed: 0 })
      // bumper sensor: thin wall ends can slip between lidar beams, so walls touched are mapped too
      const fresh = this.known.markWalls(contact)
      this.discovered.push(...fresh)
      if (this.phase === 'driving' || this.phase === 'switching') {
        this.onMessage('Bump! Backing off and replanning.')
        this.setPhase('recovering')
      } else if (this.phase === 'recovering') {
        // can't back off either: just plan from here
        this.lastReplan = -Infinity
        this.replan('stuck')
      }
    }

    const lf = this.lastFootprint
    if (!lf || Math.hypot(car.x - lf.x, car.y - lf.y) >= 0.12) {
      this.lastFootprint = { x: car.x, y: car.y, theta: car.theta }
      this.footprints.push(this.lastFootprint)
    }

    // dead reckoning between GPS fixes, with what was commanded
    advancePose(this.estimate, speed, car.steer, dt, VEHICLE.wheelbase)

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
    const z = measure(car, this.lidar, this.world.walls, this.rng)
    this.lastScan = z
    this.lastScanPose = { x: car.x, y: car.y, theta: car.theta }
    this.scanVersion++
    const fresh = this.known.integrateScan(this.estimate, z, this.lidar)
    if (fresh.length) {
      this.discovered.push(...fresh)
      if (this.plan && (this.phase === 'driving' || this.phase === 'switching') && !this.planStillValid()) {
        this.replan('new wall on the path')
      }
    }
    this.telemetry.push({ t: this.time, speed: car.speed, cte: this.phase === 'driving' && this.follower ? this.follower.cte : 0 })
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
    this.follower = new GearFollower(splitByGear(path), this.params.gains, this.params.lookahead)
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
  private contactCells(p: Pose): number[] {
    const g = this.truth
    const cells: number[] = []
    for (const o of [0, 0.31, 0.62]) {
      const x = p.x + Math.cos(p.theta) * o
      const y = p.y + Math.sin(p.theta) * o
      // exact geometry: the car body (half width 0.22 m) touches a real wall segment
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
