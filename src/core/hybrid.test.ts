import { describe, expect, it } from 'vitest'
import mazeDef from '../maps/maze.json'
import { wrapAngle } from './math/angles'
import { hybridAStar, poseIsFree, DEFAULT_HYBRID } from './planning/hybridAStar'
import { KnownMap } from './world/knownMap'
import { loadWorld, type MapDef } from './world/map'

const maze = loadWorld(mazeDef as unknown as MapDef)

describe('hybrid A*', () => {
  it('solves the fully known maze with a collision-free, drivable path', () => {
    const map = new KnownMap(maze.width, maze.height, 0.25)
    map.revealWalls(maze.walls)
    const res = hybridAStar(map, maze.start, { ...maze.goal!, useHeading: true })!
    expect(res).not.toBeNull()
    const last = res.path[res.path.length - 1]
    expect(Math.hypot(last.x - maze.goal!.x, last.y - maze.goal!.y)).toBeLessThan(0.4)
    expect(Math.abs(wrapAngle(last.theta - maze.goal!.theta))).toBeLessThan(0.3)
    for (const p of res.path.slice(5)) expect(poseIsFree(map, p, { ...DEFAULT_HYBRID, radius: DEFAULT_HYBRID.radius - 0.01 })).toBe(true)
    console.log('maze known:', res.expansions, 'expansions', res.millis.toFixed(0), 'ms', res.path.length, 'poses')
  })

  it('backs out of a dead end that is too narrow to turn around in', () => {
    const def: MapDef = {
      name: 'dead end', width: 12, height: 6, start: { x: 7, y: 3, theta: 0 },
      walls: [[3, 2.4, 9, 2.4], [3, 3.6, 9, 3.6], [9, 2.4, 9, 3.6]],
    }
    const w = loadWorld(def)
    const map = new KnownMap(w.width, w.height, 0.25)
    map.revealWalls(w.walls)
    const res = hybridAStar(map, w.start, { x: 1.5, y: 3, theta: Math.PI, useHeading: false })!
    expect(res).not.toBeNull()
    expect(res.path.some((p) => p.dir < 0)).toBe(true)
    expect(res.treeReverse.length).toBeGreaterThan(0)
  })

  it('plans through unknown space as if it were free', () => {
    const map = new KnownMap(maze.width, maze.height, 0.25)
    const res = hybridAStar(map, maze.start, { ...maze.goal!, useHeading: false })!
    expect(res).not.toBeNull()
    expect(res.path.every((p) => p.dir > 0)).toBe(true)
  })

  it('returns null when the goal is walled in', () => {
    const def: MapDef = {
      name: 'box', width: 10, height: 10, start: { x: 2, y: 2, theta: 0 },
      walls: [[6, 6, 9, 6], [9, 6, 9, 9], [9, 9, 6, 9], [6, 9, 6, 6]],
    }
    const w = loadWorld(def)
    const map = new KnownMap(w.width, w.height, 0.25)
    map.revealWalls(w.walls)
    expect(hybridAStar(map, w.start, { x: 7.5, y: 7.5, theta: 0, useHeading: false })).toBeNull()
  })
})

describe('exploration', () => {
  it('finds its way through the unknown maze, discovering walls and replanning', async () => {
    const { ExplorationSim } = await import('./sim/exploration')
    const sim = new ExplorationSim(mazeDef as unknown as MapDef, undefined, 1)
    sim.setGoal({ ...sim.world.goal!, useHeading: false })
    for (let i = 0; i < 50 * 120 && sim.phase !== 'arrived'; i++) sim.tick()
    expect(sim.phase).toBe('arrived')
    expect(sim.replans).toBeGreaterThan(5)
    expect(sim.known.explored()).toBeLessThan(0.8)
    expect(Math.hypot(sim.car.x - sim.world.goal!.x, sim.car.y - sim.world.goal!.y)).toBeLessThan(0.7)
  }, 60_000)

  it('recovers when the bumper hits a wall end the lidar missed', async () => {
    const { ExplorationSim } = await import('./sim/exploration')
    const udacity = (await import('../maps/default.json')).default
    const sim = new ExplorationSim(udacity as unknown as MapDef, undefined, 2)
    sim.setGoal({ x: 15, y: 18, theta: 0, useHeading: false })
    for (let i = 0; i < 50 * 200 && sim.phase !== 'arrived'; i++) sim.tick()
    expect(sim.phase).toBe('arrived')
  }, 60_000)

  it('reports blocked once when the goal does not fit', async () => {
    const { ExplorationSim } = await import('./sim/exploration')
    const udacity = (await import('../maps/default.json')).default
    const sim = new ExplorationSim(udacity as unknown as MapDef, undefined, 1)
    const msgs: string[] = []
    sim.onMessage = (m) => msgs.push(m)
    sim.setGoal({ x: 22, y: 12, theta: 0, useHeading: false })
    for (let i = 0; i < 50 * 120; i++) sim.tick()
    expect(sim.phase).toBe('blocked')
    expect(msgs.filter((m) => m.startsWith('No path'))).toHaveLength(1)
  }, 60_000)
})
