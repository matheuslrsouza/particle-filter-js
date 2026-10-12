import { describe, expect, it } from 'vitest'
import mapDef from '../maps/default.json'
import { PathFollower } from './control/pid'
import { evaluateGains } from './control/twiddle'
import { ParticleFilter } from './localization/particleFilter'
import { wrapAngle } from './math/angles'
import { Rng } from './math/rng'
import { astar } from './planning/astar'
import { smoothPath } from './planning/smooth'
import { castRay, evenlySpacedBeams, measure } from './sensors/lidar'
import { DEFAULT_PARAMS, Simulation, VEHICLE } from './sim/simulation'
import { stepBicycle, type VehicleState } from './vehicle/bicycle'
import { OccupancyGrid } from './world/grid'
import { loadWorld, type MapDef } from './world/map'

const def = mapDef as unknown as MapDef
const box: MapDef = { name: 'box', width: 10, height: 10, start: { x: 2, y: 2, theta: 0 }, walls: [[5, 0, 5, 7]] }

describe('math', () => {
  it('wraps angles to [-pi, pi)', () => {
    expect(wrapAngle(3 * Math.PI)).toBeCloseTo(-Math.PI)
    expect(wrapAngle(-Math.PI / 2)).toBeCloseTo(-Math.PI / 2)
    expect(wrapAngle(2 * Math.PI + 0.1)).toBeCloseTo(0.1)
  })
})

describe('lidar', () => {
  const world = loadWorld(box)
  it('hits the closest wall', () => {
    expect(castRay(2, 2, 1, 0, world.walls, 20)).toBeCloseTo(3)
    expect(castRay(2, 2, -1, 0, world.walls, 20)).toBeCloseTo(2)
    expect(castRay(2, 8, 1, 0, world.walls, 20)).toBeCloseTo(8) // passes under the wall end
  })
  it('caps at max range', () => {
    expect(castRay(2, 2, 1, 0, world.walls, 1)).toBe(1)
  })
})

describe('A* + smoothing', () => {
  const world = loadWorld(box)
  const grid = new OccupancyGrid(world, 0.25, 0.3)
  it('goes around the wall and reaches the goal', () => {
    const res = astar(grid, { x: 2, y: 2 }, { x: 8, y: 2 }, { wallPenalty: 0, penaltyRange: 0 })!
    expect(res).not.toBeNull()
    const last = res.path[res.path.length - 1]
    expect(Math.hypot(last.x - 8, last.y - 2)).toBeLessThan(0.25)
    expect(Math.max(...res.path.map((p) => p.y))).toBeGreaterThan(7)
    for (const p of res.path.slice(1)) expect(grid.isFree(p, false)).toBe(true)
  })
  it('returns null when the goal is unreachable or blocked', () => {
    expect(astar(grid, { x: 2, y: 2 }, { x: 5, y: 3 }, { wallPenalty: 0, penaltyRange: 0 })).toBeNull()
  })
  it('smoothing keeps endpoints and stays out of walls', () => {
    const res = astar(grid, { x: 2, y: 2 }, { x: 8, y: 2 }, { wallPenalty: 1, penaltyRange: 3 })!
    const s = smoothPath(res.path, grid, { weightData: 0.1, weightSmooth: 0.4, tolerance: 1e-5 })
    expect(s[0]).toEqual(res.path[0])
    expect(s[s.length - 1]).toEqual(res.path[res.path.length - 1])
    for (const p of s) expect(grid.isFree(p)).toBe(true)
  })
})

describe('PID', () => {
  it('converges onto a straight line', () => {
    const path = [{ x: 0, y: 0 }, { x: 50, y: 0 }]
    const car: VehicleState = { x: 0, y: 1, theta: 0, speed: 0, steer: 0, odometer: 0 }
    const f = new PathFollower(path, DEFAULT_PARAMS.gains, 0.5)
    for (let i = 0; i < 500; i++) stepBicycle(car, { speed: 1.5, steer: f.steer(car, 0.02) }, 0.02, VEHICLE)
    expect(Math.abs(car.y)).toBeLessThan(0.1)
  })
  it('turns around when the path starts behind the car', () => {
    const path = [{ x: 0, y: 0 }, { x: -20, y: 0 }]
    const car: VehicleState = { x: 0, y: 0, theta: 0, speed: 0, steer: 0, odometer: 0 }
    const f = new PathFollower(path, DEFAULT_PARAMS.gains, 0.5)
    for (let i = 0; i < 600; i++) stepBicycle(car, { speed: 1.5, steer: f.steer(car, 0.02) }, 0.02, VEHICLE)
    expect(car.x).toBeLessThan(-5)
    expect(Math.abs(car.y)).toBeLessThan(0.5)
  })
})

describe('particle filter', () => {
  it('converges on a static robot in the real map', () => {
    const world = loadWorld(def)
    const grid = new OccupancyGrid(world, 0.25, 0.4)
    const rng = new Rng(1)
    const lidar = { angles: evenlySpacedBeams(36), maxRange: 12, noise: 0.05 }
    const pf = new ParticleFilter(500, grid, rng, DEFAULT_PARAMS.filter)
    const truth = world.start
    pf.initAround(truth, 1, 0.4)
    for (let i = 0; i < 20; i++) {
      pf.predict({ speed: 0, steer: 0 }, 0.1, VEHICLE.wheelbase)
      pf.update(measure(truth, lidar, world.walls, rng), lidar)
    }
    expect(Math.hypot(pf.estimate.x - truth.x, pf.estimate.y - truth.y)).toBeLessThan(0.2)
    expect(Math.abs(wrapAngle(pf.estimate.theta - truth.theta))).toBeLessThan(0.1)
  })
})

describe('simulation', () => {
  it('drives to several goals using only the estimated pose', () => {
    const sim = new Simulation(def, structuredClone(DEFAULT_PARAMS), 42)
    const goals = [{ x: 26, y: 3 }, { x: 15, y: 18 }, { x: 2, y: 18 }]
    for (const g of goals) {
      expect(sim.setGoal(g)).toBe(true)
      for (let i = 0; i < 6000 && sim.phase !== 'arrived'; i++) sim.tick()
      expect(sim.phase).toBe('arrived')
      expect(Math.hypot(sim.car.x - sim.plan!.goal.x, sim.car.y - sim.plan!.goal.y)).toBeLessThan(0.6)
    }
  })
  it('recovers from a kidnapping (augmented MCL)', () => {
    for (const seed of [1, 3, 5]) {
      const sim = new Simulation(def, structuredClone(DEFAULT_PARAMS), seed)
      for (let i = 0; i < 100; i++) sim.tick()
      sim.kidnap()
      for (let i = 0; i < 50 * 30; i++) sim.tick()
      const e = sim.pf.estimate
      expect(Math.hypot(e.x - sim.car.x, e.y - sim.car.y)).toBeLessThan(0.3)
    }
  }, 30_000)
  it('default gains track a planned path tightly', () => {
    const sim = new Simulation(def, structuredClone(DEFAULT_PARAMS), 7)
    sim.setGoal({ x: 26, y: 3 })
    const mse = evaluateGains(DEFAULT_PARAMS.gains, {
      path: sim.plan!.smooth, vehicle: VEHICLE, speed: 1.5, lookahead: 0.5, dt: 0.02,
    })
    expect(mse).toBeLessThan(0.05)
  })
})
