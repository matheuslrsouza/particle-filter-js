import { describe, expect, it } from 'vitest'
import cityJson from '../maps/city-mountain-view.json'
import type { CityDef, CityRoad } from './city/cityDef'
import { CitySim } from './city/citySim'
import { rectangle } from './city/parking'
import { buildLaneGraph } from './city/roadNetwork'
import { IDM, idmAccel, NPC_LENGTH, NPC_WIDTH, Traffic } from './city/traffic'
import { Rng } from './math/rng'

const city = cityJson as unknown as CityDef

describe('IDM car following', () => {
  it('accelerates to the desired speed on a free road', () => {
    let v = 0
    for (let t = 0; t < 60; t += 0.1) v += idmAccel(v, 12, Infinity, 0) * 0.1
    expect(v).toBeGreaterThan(11.5)
    expect(v).toBeLessThanOrEqual(12.01)
  })

  it('stops behind a stopped car, at about the minimum gap', () => {
    let v = 12
    let x = 0
    const lead = 120
    for (let t = 0; t < 60; t += 0.05) {
      v = Math.max(0, v + idmAccel(v, 12, lead - x, v) * 0.05)
      x += v * 0.05
    }
    expect(v).toBeLessThan(0.05)
    expect(lead - x).toBeGreaterThan(IDM.s0 * 0.8)
    expect(lead - x).toBeLessThan(IDM.s0 * 2)
  })
})

describe('junction reservations', () => {
  const road = (id: number, name: string, pts: [number, number][], nodes: number[]): CityRoad => ({
    id, name, highway: 'secondary', pts, nodes, lanesForward: 1, lanesBackward: 1, centerTurnLane: false, speed: 11,
  })
  const fixture: CityDef = {
    name: 'x', attribution: '', width: 200, height: 200, buildings: [], parks: [], parking: [],
    roads: [road(1, 'A', [[0, 100], [100, 100], [200, 100]], [1, 2, 3]), road(2, 'B', [[100, 0], [100, 100], [100, 200]], [4, 2, 5])],
  }
  const g = buildLaneGraph(fixture)
  const t = new Traffic(g, new Rng(1), 0, 0)
  const heading = (c: { points: { x: number; y: number }[] }) => Math.atan2(c.points[c.points.length - 1].y - c.points[0].y, c.points[c.points.length - 1].x - c.points[0].x)
  const straight = g.connectors.filter((c) => c.turn === 'straight')
  const east = straight.find((c) => Math.cos(heading(c)) > 0.9)!
  const west = straight.find((c) => Math.cos(heading(c)) < -0.9)!
  const south = straight.find((c) => Math.sin(heading(c)) > 0.9)!

  it('lets opposite straight movements go together, but not crossing ones', () => {
    expect(t.reserve(east.id, 1)).toBe(true)
    expect(t.reserve(west.id, 2)).toBe(true)
    expect(t.reserve(south.id, 3)).toBe(false)
    t.release(east.id, 1)
    t.release(west.id, 2)
    expect(t.reserve(south.id, 3)).toBe(true)
  })
})

describe('driving with traffic', () => {
  type P = { x: number; y: number }
  const overlap = (a: P[], b: P[]) => {
    for (const poly of [a, b]) {
      for (let i = 0; i < 4; i++) {
        const p = poly[i]
        const q = poly[(i + 1) % 4]
        const nx = -(q.y - p.y)
        const ny = q.x - p.x
        const pa = a.map((v) => v.x * nx + v.y * ny)
        const pb = b.map((v) => v.x * nx + v.y * ny)
        if (Math.max(...pa) < Math.min(...pb) || Math.max(...pb) < Math.min(...pa)) return false
      }
    }
    return true
  }

  it('reaches its destination among cars and pedestrians without any collision', () => {
    const reacted = new Set<string>()
    for (const seed of [1, 5]) {
      const sim = new CitySim(city, seed)
      sim.enableTraffic(40, 30)
      expect(sim.randomDestination()).toBe(true)
      const t = sim.traffic!
      let collisions = 0
      for (let i = 0; i < 50 * 400 && sim.phase === 'driving'; i++) {
        sim.tick()
        if (sim.yielding) reacted.add(sim.yielding)
        if (i % 10) continue
        const c = sim.car
        const ego = rectangle({ x: c.x + Math.cos(c.theta) * 1.74, y: c.y + Math.sin(c.theta) * 1.74 }, c.theta, 5.04, 1.95)
        const boxes = t.npcs.map((n) => rectangle(n.pose, n.pose.theta, NPC_LENGTH, NPC_WIDTH))
        if (boxes.some((b) => overlap(ego, b))) collisions++
        for (let a = 0; a < boxes.length; a++) {
          for (let b = a + 1; b < boxes.length; b++) {
            const A = t.npcs[a]
            const B = t.npcs[b]
            // lanes crossing at an angle without a junction are bridges (no heights in the sim)
            const bridge = A.edge.kind === 'lane' && B.edge.kind === 'lane' && Math.abs(Math.cos(A.pose.theta - B.pose.theta)) < 0.5
            if (!bridge && overlap(boxes[a], boxes[b])) collisions++
          }
        }
        for (const p of t.pedestrians) {
          if (p.state !== 'crossing') continue
          for (const box of [ego, ...boxes]) {
            const cx = box.reduce((s, q) => s + q.x, 0) / 4
            const cy = box.reduce((s, q) => s + q.y, 0) / 4
            if (Math.hypot(p.pos.x - cx, p.pos.y - cy) < 1.6) collisions++
          }
        }
      }
      expect(sim.phase).toBe('arrived')
      expect(collisions).toBe(0)
    }
    // on the way it had to follow cars, wait at junctions or stop for pedestrians
    expect(reacted.size).toBeGreaterThan(0)
  }, 180_000)
})
