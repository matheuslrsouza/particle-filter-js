import { describe, expect, it } from 'vitest'
import mazeDef from '../maps/maze.json'
import { wrapAngle } from './math/angles'
import { DEFAULT_HYBRID, hybridAStar, poseIsFree, type PathPose } from './planning/hybridAStar'
import { PLAN_VALIDITY_RATIO, smoothHybridPath } from './planning/smoothHybrid'
import { KnownMap } from './world/knownMap'
import { loadWorld, type MapDef } from './world/map'

const maze = loadWorld(mazeDef as unknown as MapDef)

/** Sum of heading changes along the path: lower means less zigzag. */
const totalTurning = (p: PathPose[]) => p.slice(1).reduce((s, q, i) => s + Math.abs(wrapAngle(q.theta - p[i].theta)), 0)

function knownMaze() {
  const map = new KnownMap(maze.width, maze.height, 0.25)
  map.revealWalls(maze.walls)
  return map
}

describe('hybrid A* smoothing', () => {
  const map = knownMaze()
  const raw = hybridAStar(map, maze.start, { ...maze.goal!, useHeading: true })!.path
  const smooth = smoothHybridPath(raw, map)

  it('keeps start and goal poses', () => {
    expect(smooth[0]).toMatchObject({ x: raw[0].x, y: raw[0].y, theta: raw[0].theta })
    const a = smooth[smooth.length - 1]
    const b = raw[raw.length - 1]
    expect(Math.hypot(a.x - b.x, a.y - b.y)).toBeLessThan(1e-9)
    expect(a.theta).toBeCloseTo(b.theta)
  })

  it('stays collision-free and drivable', () => {
    const kappaMax = (Math.tan(DEFAULT_HYBRID.maxSteer) / DEFAULT_HYBRID.wheelbase) * 1.06
    const check = { ...DEFAULT_HYBRID, radius: DEFAULT_HYBRID.radius * PLAN_VALIDITY_RATIO }
    for (let i = 1; i < smooth.length - 1; i++) {
      expect(poseIsFree(map, smooth[i], check)).toBe(true)
      const [a, b, c] = [smooth[i - 1], smooth[i], smooth[i + 1]]
      const ab = Math.hypot(b.x - a.x, b.y - a.y)
      const bc = Math.hypot(c.x - b.x, c.y - b.y)
      const ca = Math.hypot(a.x - c.x, a.y - c.y)
      const area2 = Math.abs((b.x - a.x) * (c.y - a.y) - (b.y - a.y) * (c.x - a.x))
      if (ab * bc * ca > 1e-9) expect((2 * area2) / (ab * bc * ca)).toBeLessThan(kappaMax)
    }
  })

  it('removes the zigzag of the discrete arcs', () => {
    // the total includes the ~90° turns the maze forces, so even a modest drop is most of the zigzag
    expect(totalTurning(smooth)).toBeLessThan(totalTurning(raw) * 0.88)
  })

  it('keeps gear switches where the raw path has them', () => {
    const def: MapDef = {
      name: 'dead end', width: 12, height: 6, start: { x: 7, y: 3, theta: 0 },
      walls: [[3, 2.4, 9, 2.4], [3, 3.6, 9, 3.6], [9, 2.4, 9, 3.6]],
    }
    const w = loadWorld(def)
    const m = new KnownMap(w.width, w.height, 0.25)
    m.revealWalls(w.walls)
    const r = hybridAStar(m, w.start, { x: 1.5, y: 3, theta: Math.PI, useHeading: false })!.path
    const s = smoothHybridPath(r, m)
    const switches = (p: PathPose[]) => p.filter((q, i) => i > 1 && q.dir !== p[i - 1].dir).length
    expect(switches(s)).toBe(switches(r))
    expect(s.some((p) => p.dir < 0)).toBe(true)
  })
})
