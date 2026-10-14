import { describe, expect, it } from 'vitest'
import cityJson from '../maps/city-mountain-view.json'
import type { CityDef } from './city/cityDef'
import { CITY_HYBRID, CitySim } from './city/citySim'
import { drivableGrid } from './city/drivable'
import { buildLaneGraph } from './city/roadNetwork'
import { planRoute, pointAt, snapToLane } from './city/routing'
import { hybridAStar } from './planning/hybridAStar'

const city = cityJson as unknown as CityDef

/** True if any of the car's body points is inside the polygon. */
function overlaps(sim: CitySim, poly: { x: number; y: number }[]): boolean {
  const xs = poly.map((p) => p.x)
  const ys = poly.map((p) => p.y)
  return [0, 1.5, 3, 4.2].some((o) => {
    const x = sim.car.x + Math.cos(sim.car.theta) * o
    const y = sim.car.y + Math.sin(sim.car.theta) * o
    let hit = false
    for (let a = 0, c = poly.length - 1; a < poly.length; c = a++) {
      if (ys[a] > y !== ys[c] > y && x < ((xs[c] - xs[a]) * (y - ys[a])) / (ys[c] - ys[a]) + xs[a]) hit = !hit
    }
    return hit
  })
}

describe('blocked roads', () => {
  const g = buildLaneGraph(city)

  it('routes around blocked lanes', () => {
    const sim = new CitySim(city, 1)
    sim.randomDestination()
    const start = snapToLane(g, sim.estimate, sim.estimate.theta)!
    const goal = snapToLane(g, sim.destination!)!
    const free = planRoute(g, start, goal)!
    // block a lane in the middle of the free route
    const mid = free.path[Math.floor(free.path.length / 2)]
    const lane = snapToLane(g, mid, mid.theta)!.lane
    const seg = g.segments[g.lanes[lane].segment]
    const blocked = new Set([...seg.forward, ...seg.backward])
    const detour = planRoute(g, start, goal, undefined, blocked)
    expect(detour).not.toBeNull()
    for (const p of detour!.path) {
      const at = snapToLane(g, p, p.theta)
      if (at && Math.hypot(at.point.x - p.x, at.point.y - p.y) < 0.3) expect(blocked.has(at.lane)).toBe(false)
    }
  })

  it('plans a U-turn on a real two-way street', () => {
    const seg = g.segments.find((s) => s.forward.length === 1 && s.backward.length === 1 && g.lanes[s.forward[0]].length > 60)!
    const L = g.lanes[seg.forward[0]]
    const O = g.lanes[seg.backward[0]]
    const p = pointAt(L.points, L.length / 2)
    const q = pointAt(L.points, L.length / 2 + 1)
    const start = { x: p.x, y: p.y, theta: Math.atan2(q.y - p.y, q.x - p.x) }
    const s = O.length - L.length / 2 + 8
    const gp = pointAt(O.points, s)
    const gq = pointAt(O.points, s + 1)
    const grid = drivableGrid(g, start, 60, 0.5)
    const o = grid.origin
    const res = hybridAStar(
      grid.map,
      { ...start, x: start.x - o.x, y: start.y - o.y },
      { x: gp.x - o.x, y: gp.y - o.y, theta: Math.atan2(gq.y - gp.y, gq.x - gp.x), useHeading: true },
      CITY_HYBRID,
    )
    expect(res).not.toBeNull()
    const end = res!.path[res!.path.length - 1]
    // ends facing the other way
    expect(Math.cos(end.theta - start.theta)).toBeLessThan(-0.9)
  })

  it('stops before a barricade, turns around and reaches the destination another way', () => {
    for (const seed of [1, 3]) {
      const sim = new CitySim(city, seed)
      sim.gpsGrade = 'dual'
      expect(sim.randomDestination()).toBe(true)
      const phases = new Set<string>()
      let placed = false
      let crossed = false
      for (let i = 0; i < 50 * 300 && sim.phase !== 'arrived' && sim.phase !== 'stuck'; i++) {
        sim.tick()
        phases.add(sim.phase)
        if (!placed && sim.time > 6) placed = sim.placeBarrierAhead(70)
        if (sim.barriers.some((b) => overlaps(sim, b.poly))) crossed = true
      }
      expect(placed).toBe(true)
      expect(crossed).toBe(false)
      expect(phases.has('stopping')).toBe(true)
      expect(phases.has('uturn')).toBe(true)
      expect(sim.phase).toBe('arrived')
    }
  }, 120_000)
})

describe('missions', () => {
  it('visits the checkpoints in order', () => {
    const sim = new CitySim(city, 2)
    sim.gpsGrade = 'rtk'
    expect(sim.startMission(3)).toBe(true)
    const order: number[] = []
    sim.onMessage = (m) => {
      const k = m.match(/Checkpoint (\d+)/)
      if (k) order.push(Number(k[1]))
    }
    for (let i = 0; i < 50 * 600 && sim.phase === 'driving'; i++) sim.tick()
    expect(sim.phase).toBe('arrived')
    expect(order).toEqual([1, 2])
    expect(sim.mission!.reached).toBe(3)
  }, 120_000)
})
