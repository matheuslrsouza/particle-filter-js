import { PathFollower, type PidGains } from '../control/pid'
import { ParticleFilter, type FilterParams } from '../localization/particleFilter'
import { deg2rad } from '../math/angles'
import { Rng } from '../math/rng'
import { clamp, dist, type Pose, type Vec2 } from '../math/vec'
import { astar } from '../planning/astar'
import { smoothPath } from '../planning/smooth'
import { evenlySpacedBeams, measure, type LidarConfig } from '../sensors/lidar'
import { advancePose, steerToward, type VehicleParams, type VehicleState } from '../vehicle/bicycle'
import { FREE, OccupancyGrid } from '../world/grid'
import { loadWorld, type MapDef, type World } from '../world/map'

export type Phase = 'idle' | 'planning' | 'driving' | 'arrived'

export interface SimParams {
  particleCount: number
  cruiseSpeed: number
  gains: PidGains
  lookahead: number
  sensorHz: number
  /** how long the planning phase lasts, so the A* expansion can be animated */
  planSeconds: number
  /** actuator noise of the real car */
  actuatorSpeedNoise: number
  actuatorSteerNoise: number
  filter: FilterParams
}

export interface Plan {
  goal: Vec2
  raw: Vec2[]
  smooth: Vec2[]
  expanded: number[]
}

export interface Telemetry {
  t: number
  cte: number
  locError: number
}

export const DEFAULT_PARAMS: SimParams = {
  particleCount: 500,
  cruiseSpeed: 1.5,
  gains: { kp: 2.5, ki: 0.05, kd: 0.6 },
  lookahead: 0.5,
  sensorHz: 10,
  planSeconds: 2.5,
  actuatorSpeedNoise: 0.05,
  actuatorSteerNoise: deg2rad(1.5),
  filter: {
    speedNoise: 0.1,
    steerNoise: deg2rad(4),
    poseJitter: 0.03,
    sensorSigma: 0.3,
    allowRecovery: true,
  },
}

export const VEHICLE: VehicleParams = { wheelbase: 0.5, maxSteer: deg2rad(35), maxSteerRate: deg2rad(180) }

/**
 * Whole robot pipeline, independent of rendering:
 * lidar → particle filter → A* + smoothing → PID → bicycle model.
 * Advance it with `tick()` at the fixed `dt`.
 */
export class Simulation {
  readonly dt = 0.02
  readonly world: World
  readonly grid: OccupancyGrid
  readonly lidar: LidarConfig = { angles: evenlySpacedBeams(36), maxRange: 12, noise: 0.05 }
  readonly rng: Rng

  car!: VehicleState
  /** car pose at the previous tick, for render interpolation */
  prevCar!: Pose
  pf!: ParticleFilter
  plan: Plan | null = null
  follower: PathFollower | null = null
  phase: Phase = 'idle'
  phaseTime = 0
  time = 0
  lastScan: Float64Array | null = null
  lastScanPose: Pose | null = null
  telemetry: Telemetry[] = []
  /** bumps every time the plan changes, so the renderer can rebuild its meshes */
  planVersion = 0
  /** bumps every time the particle arrays are replaced */
  filterVersion = 0
  onMessage: (msg: string) => void = () => {}

  private tickCount = 0

  constructor(
    mapDef: MapDef,
    public params: SimParams = structuredClone(DEFAULT_PARAMS),
    seed = Date.now(),
  ) {
    this.rng = new Rng(seed)
    this.world = loadWorld(mapDef)
    this.grid = new OccupancyGrid(this.world, 0.25, 0.4)
    this.reset()
  }

  reset(): void {
    const s = this.world.start
    this.car = { ...s, speed: 0, steer: 0, odometer: 0 }
    this.prevCar = { ...s }
    this.createFilter()
    this.pf.initAround(s, 0.5, 0.2)
    this.clearPlan()
    this.telemetry = []
    this.lastScan = null
    this.setPhase('idle')
  }

  setParticleCount(n: number): void {
    const est = this.pf.estimate
    this.params.particleCount = n
    this.createFilter()
    this.pf.initAround(est, 0.3, 0.1)
  }

  /** Throws the particles uniformly over the map: the robot "forgets" where it is. */
  globalLocalization(): void {
    this.pf.initGlobal()
    this.filterVersion++
  }

  /** Teleports the real car somewhere else without telling the filter. */
  kidnap(): void {
    const pose = this.randomFreePose()
    if (!pose) return
    this.car = { ...pose, speed: 0, steer: 0, odometer: this.car.odometer }
    this.prevCar = { ...pose }
    this.clearPlan()
    this.setPhase('idle')
    this.onMessage('Robot kidnapped! Watch the particles recover.')
  }

  randomGoal(): void {
    for (let i = 0; i < 50; i++) {
      const p = this.randomFreePose()
      if (p && dist(p, this.car) > 6 && this.setGoal(p)) return
    }
  }

  /** Plans from the *estimated* pose to `target`. Returns false if no path exists. */
  setGoal(target: Vec2): boolean {
    const [c, r] = this.grid.worldToCell(target)
    const cell = this.grid.nearestFree(c, r)
    if (!cell) return false
    const goal = this.grid.cellCenter(cell[0], cell[1])
    const start = this.pf.estimate
    const result = astar(this.grid, start, goal, { wallPenalty: 1.5, penaltyRange: 4 })
    if (!result || result.path.length < 2) {
      this.onMessage('No path to that point.')
      return false
    }
    const smooth = smoothPath(result.path, this.grid, { weightData: 0.1, weightSmooth: 0.4, tolerance: 1e-5 })
    this.plan = { goal, raw: result.path, smooth, expanded: result.expanded }
    this.follower = new PathFollower(smooth, this.params.gains, this.params.lookahead)
    this.planVersion++
    this.setPhase('planning')
    return true
  }

  tick(): void {
    const dt = this.dt
    this.prevCar = { x: this.car.x, y: this.car.y, theta: this.car.theta }
    this.time += dt
    this.phaseTime += dt
    this.tickCount++

    if (this.phase === 'planning' && this.phaseTime >= this.params.planSeconds) this.setPhase('driving')

    let speed = 0
    let steer = 0
    if (this.phase === 'driving' && this.plan && this.follower) {
      const est = this.pf.estimate
      this.follower.gains = this.params.gains
      this.follower.lookahead = this.params.lookahead
      steer = this.follower.steer(est, dt)
      const toGoal = dist(est, this.plan.goal)
      speed = this.params.cruiseSpeed * clamp(toGoal / 1.5, 0.3, 1)
      if (toGoal < 0.3 || (this.follower.finished && toGoal < 1)) {
        speed = 0
        this.setPhase('arrived')
      }
    }

    // the real car executes the command with actuator noise
    const car = this.car
    steerToward(car, steer, dt, VEHICLE)
    car.speed = speed
    const v = this.rng.gaussian(speed, Math.abs(speed) * this.params.actuatorSpeedNoise)
    const st = speed ? this.rng.gaussian(car.steer, this.params.actuatorSteerNoise) : car.steer
    advancePose(car, v, st, dt, VEHICLE.wheelbase)
    car.odometer += v * dt
    if (!this.grid.isFree(car)) {
      Object.assign(car, this.prevCar, { speed: 0 })
      if (this.phase === 'driving') {
        this.setPhase('idle')
        this.onMessage('Bump! The car hit a wall.')
      }
    }

    // the filter only knows what was commanded
    this.pf.params = this.params.filter
    this.pf.predict({ speed, steer: car.steer }, dt, VEHICLE.wheelbase)

    const sensorEvery = Math.max(1, Math.round(1 / (this.params.sensorHz * dt)))
    if (this.tickCount % sensorEvery === 0) {
      const z = measure(car, this.lidar, this.world.walls, this.rng)
      this.pf.update(z, this.lidar)
      this.filterVersion++
      this.lastScan = z
      this.lastScanPose = { x: car.x, y: car.y, theta: car.theta }
      this.telemetry.push({
        t: this.time,
        cte: this.phase === 'driving' && this.follower ? this.follower.cte : 0,
        locError: dist(this.pf.estimate, car),
      })
      if (this.telemetry.length > 300) this.telemetry.shift()
    }
  }

  private createFilter(): void {
    this.pf = new ParticleFilter(this.params.particleCount, this.grid, this.rng, this.params.filter)
    this.filterVersion++
  }

  private clearPlan(): void {
    this.plan = null
    this.follower = null
    this.planVersion++
  }

  private setPhase(p: Phase): void {
    this.phase = p
    this.phaseTime = 0
  }

  private randomFreePose(): Pose | null {
    const g = this.grid
    for (let i = 0; i < 500; i++) {
      const c = this.rng.int(g.cols)
      const r = this.rng.int(g.rows)
      const idx = g.index(c, r)
      if (g.cells[idx] === FREE && g.clearance[idx] > 4) {
        return { ...g.cellCenter(c, r), theta: this.rng.range(-Math.PI, Math.PI) }
      }
    }
    return null
  }
}
