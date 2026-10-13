import { describe, expect, it } from 'vitest'
import mazeDef from '../maps/maze.json'
import { wrapAngle } from './math/angles'
import { Rng } from './math/rng'
import { DEFAULT_HYBRID, hybridAStar } from './planning/hybridAStar'
import { reedsSheppPaths, sampleReedsShepp } from './planning/reedsShepp'
import { evenlySpacedBeams } from './sensors/lidar'
import { ExplorationSim } from './sim/exploration'
import { KNOWN_WALL, KnownMap, UNKNOWN } from './world/knownMap'
import { loadWorld, type MapDef } from './world/map'

const R = DEFAULT_HYBRID.wheelbase / Math.tan(DEFAULT_HYBRID.maxSteer)
const shortest = (a: { x: number; y: number; theta: number }, b: { x: number; y: number; theta: number }) =>
  reedsSheppPaths(a, b, R).reduce((p, q) => (q.length < p.length ? q : p))

describe('Reeds-Shepp', () => {
  it('a straight line is just the distance', () => {
    expect(shortest({ x: 0, y: 0, theta: 0 }, { x: 5, y: 0, theta: 0 }).length).toBeCloseTo(5)
    expect(shortest({ x: 0, y: 0, theta: 0 }, { x: -3, y: 0, theta: 0 }).length).toBeCloseTo(3) // straight in reverse
  })

  it('a quarter turn at full lock is R·π/2', () => {
    expect(shortest({ x: 0, y: 0, theta: 0 }, { x: R, y: R, theta: Math.PI / 2 }).length).toBeCloseTo((R * Math.PI) / 2)
  })

  it('always ends exactly on the target pose', () => {
    const rng = new Rng(3)
    for (let i = 0; i < 300; i++) {
      const a = { x: rng.range(-6, 6), y: rng.range(-6, 6), theta: rng.range(-Math.PI, Math.PI) }
      const b = { x: rng.range(-6, 6), y: rng.range(-6, 6), theta: rng.range(-Math.PI, Math.PI) }
      const samples = sampleReedsShepp(a, shortest(a, b), R, 0.1)
      const end = samples[samples.length - 1]
      expect(Math.hypot(end.x - b.x, end.y - b.y)).toBeLessThan(1e-6)
      expect(Math.abs(wrapAngle(end.theta - b.theta))).toBeLessThan(1e-6)
    }
  })

  it('lets Hybrid A* reach the goal pose exactly, with fewer expansions', () => {
    const maze = loadWorld(mazeDef as unknown as MapDef)
    const map = new KnownMap(maze.width, maze.height, 0.25)
    map.revealWalls(maze.walls)
    const goal = { ...maze.goal!, useHeading: true }
    const withShot = hybridAStar(map, maze.start, goal)!
    const without = hybridAStar(map, maze.start, goal, { ...DEFAULT_HYBRID, analyticExpansion: false })!
    const end = withShot.path[withShot.path.length - 1]
    expect(Math.hypot(end.x - goal.x, end.y - goal.y)).toBeLessThan(1e-6)
    expect(Math.abs(wrapAngle(end.theta - goal.theta))).toBeLessThan(1e-6)
    expect(withShot.analytic.length).toBeGreaterThan(0)
    expect(withShot.expansions).toBeLessThan(without.expansions)
  })
})

describe('log-odds mapping', () => {
  const lidar = { angles: evenlySpacedBeams(1).map(() => 0), maxRange: 4, noise: 0 }
  const pose = { x: 1, y: 1.1, theta: 0 }
  const wallCell = (map: KnownMap) => map.cellOf(3.02, 1.1)

  it('needs repeated hits before calling a cell a wall', () => {
    const map = new KnownMap(10, 5, 0.25)
    map.integrateScan(pose, new Float64Array([2]), lidar)
    expect(map.state[wallCell(map)]).toBe(UNKNOWN)
    map.integrateScan(pose, new Float64Array([2]), lidar)
    expect(map.state[wallCell(map)]).toBe(KNOWN_WALL)
  })

  it('erases a wall only after sustained evidence that it is free (hysteresis)', () => {
    const map = new KnownMap(10, 5, 0.25)
    for (let i = 0; i < 2; i++) map.integrateScan(pose, new Float64Array([2]), lidar)
    expect(map.state[wallCell(map)]).toBe(KNOWN_WALL)
    for (let i = 0; i < 5; i++) map.integrateScan(pose, new Float64Array([3.5]), lidar)
    expect(map.state[wallCell(map)]).toBe(KNOWN_WALL) // a few grazing beams don't erase it
    for (let i = 0; i < 30; i++) map.integrateScan(pose, new Float64Array([3.5]), lidar)
    expect(map.state[wallCell(map)]).not.toBe(KNOWN_WALL)
  })

  it('never erases walls felt by the bumper', () => {
    const map = new KnownMap(10, 5, 0.25)
    map.markWalls([wallCell(map)])
    for (let i = 0; i < 60; i++) map.integrateScan(pose, new Float64Array([3.5]), lidar)
    expect(map.state[wallCell(map)]).toBe(KNOWN_WALL)
  })

  it('binary mode keeps a single bogus hit as a wall forever', () => {
    const map = new KnownMap(10, 5, 0.25)
    map.mode = 'binary'
    map.integrateScan(pose, new Float64Array([2]), lidar)
    for (let i = 0; i < 8; i++) map.integrateScan(pose, new Float64Array([3.5]), lidar)
    expect(map.state[wallCell(map)]).toBe(KNOWN_WALL)
  })

  it('keeps fewer phantom walls than the binary map with a noisy lidar', () => {
    const count = (mapping: 'logodds' | 'binary') => {
      const sim = new ExplorationSim(mazeDef as unknown as MapDef, undefined, 4)
      sim.params.mapping = mapping
      sim.params.lidarSpurious = 0.03
      sim.setGoal({ ...sim.world.goal!, useHeading: false })
      for (let i = 0; i < 50 * 40; i++) sim.tick()
      // a phantom wall is a known wall cell far from every real wall
      let phantoms = 0
      for (let i = 0; i < sim.known.state.length; i++) {
        if (sim.known.state[i] !== KNOWN_WALL) continue
        if (sim.truth.clearance[i] * sim.truth.cellSize > 0.6) phantoms++
      }
      return phantoms
    }
    const binary = count('binary')
    const logodds = count('logodds')
    expect(binary).toBeGreaterThan(5)
    expect(logodds).toBeLessThan(binary / 3)
  }, 60_000)
})
