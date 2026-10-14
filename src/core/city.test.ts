import { describe, expect, it } from 'vitest'
import cityJson from '../maps/city-mountain-view.json'
import type { CityDef, CityRoad } from './city/cityDef'
import { CitySim } from './city/citySim'
import { buildLaneGraph, largestComponent, type LaneGraph } from './city/roadNetwork'
import { planRoute, snapToLane } from './city/routing'
import type { Vec2 } from './math/vec'

const road = (id: number, name: string, pts: [number, number][], nodes: number[], fwd: number, bwd: number): CityRoad => ({
  id, name, highway: 'secondary', pts, nodes, lanesForward: fwd, lanesBackward: bwd, centerTurnLane: false, speed: 11,
})

// a "+" junction at (100, 100): Main St east-west with 2 lanes per direction,
// Oak St one-way southbound with a single lane
const fixture: CityDef = {
  name: 'fixture', attribution: '', width: 200, height: 200, buildings: [], parks: [],
  roads: [
    road(1, 'Main St', [[0, 100], [100, 100], [200, 100]], [1, 2, 3], 2, 2),
    road(2, 'Oak St', [[100, 0], [100, 100], [100, 200]], [4, 2, 5], 1, 0),
  ],
}

const minRadius = (pts: Vec2[]) => {
  let r = Infinity
  for (let i = 1; i < pts.length - 1; i++) {
    const [a, b, c] = [pts[i - 1], pts[i], pts[i + 1]]
    const ab = Math.hypot(b.x - a.x, b.y - a.y)
    const bc = Math.hypot(c.x - b.x, c.y - b.y)
    const ca = Math.hypot(a.x - c.x, a.y - c.y)
    const area2 = Math.abs((b.x - a.x) * (c.y - a.y) - (b.y - a.y) * (c.x - a.x))
    if (area2 > 1e-9) r = Math.min(r, (ab * bc * ca) / (2 * area2))
  }
  return r
}

describe('road network (fixture)', () => {
  const g: LaneGraph = buildLaneGraph(fixture)
  const lanesOf = (name: string) => g.lanes.filter((l) => l.name === name)
  const heading = (l: { points: Vec2[] }) => Math.atan2(l.points[1].y - l.points[0].y, l.points[1].x - l.points[0].x)
  const eastbound = lanesOf('Main St').filter((l) => Math.cos(heading(l)) > 0.9)
  const westbound = lanesOf('Main St').filter((l) => Math.cos(heading(l)) < -0.9)

  it('creates the lanes of each direction', () => {
    expect(eastbound).toHaveLength(4) // 2 lanes x 2 segments
    expect(westbound).toHaveLength(4)
    expect(lanesOf('Oak St')).toHaveLength(2)
    // the one-way only goes south (+y)
    for (const l of lanesOf('Oak St')) expect(Math.sin(heading(l))).toBeGreaterThan(0.9)
  })

  it('puts lanes on the right of their direction (right-hand traffic)', () => {
    // driving east, the right is south (+y); driving west, the right is north
    for (const l of eastbound) expect(l.points[0].y).toBeGreaterThan(100)
    for (const l of westbound) expect(l.points[0].y).toBeLessThan(100)
    // index 0 is the leftmost lane, closest to the centerline
    const [left, right] = eastbound.filter((l) => l.points[0].x < 100).sort((a, b) => a.index - b.index)
    expect(left.points[0].y).toBeCloseTo(100 + 3.3 / 2)
    expect(right.points[0].y).toBeCloseTo(100 + 3.3 * 1.5)
  })

  it('connects the allowed movements only', () => {
    const turnsFrom = (l: (typeof g.lanes)[number]) => l.next.map((c) => g.connectors[c])
    const ebWest = eastbound.filter((l) => l.points[0].x < 100)
    const ebRight = ebWest.find((l) => l.index === l.count - 1)!
    const ebLeft = ebWest.find((l) => l.index === 0)!
    // right turn into the one-way from the rightmost lane, straight from both lanes
    expect(turnsFrom(ebRight).map((c) => c.turn).sort()).toEqual(['right', 'straight'])
    // no left turn: Oak St is one-way southbound
    expect(turnsFrom(ebLeft).map((c) => c.turn)).toEqual(['straight'])
    // southbound: straight, right (west) into the rightmost lane, left (east) into the leftmost
    const sb = lanesOf('Oak St').find((l) => l.points[0].y < 100)!
    const moves = turnsFrom(sb)
    expect(moves.map((c) => c.turn).sort()).toEqual(['left', 'right', 'straight'])
    const right = moves.find((c) => c.turn === 'right')!
    const left = moves.find((c) => c.turn === 'left')!
    expect(g.lanes[right.to].index).toBe(g.lanes[right.to].count - 1)
    expect(g.lanes[left.to].index).toBe(0)
    // no U-turns
    for (const c of g.connectors) expect(g.lanes[c.from].segment).not.toBe(g.lanes[c.to].segment)
  })

  it('makes continuous, drivable connectors', () => {
    for (const c of g.connectors) {
      const a = g.lanes[c.from].points
      const b = g.lanes[c.to].points
      const end = a[a.length - 1]
      expect(Math.hypot(c.points[0].x - end.x, c.points[0].y - end.y)).toBeLessThan(0.1)
      const last = c.points[c.points.length - 1]
      expect(Math.hypot(last.x - b[0].x, last.y - b[0].y)).toBeLessThan(0.1)
      expect(minRadius(c.points)).toBeGreaterThan(4) // a car turns with R ≥ ~4 m
    }
  })
})

describe('road network (Mountain View)', () => {
  const g = buildLaneGraph(cityJson as unknown as CityDef)

  it('builds a connected network from the OSM data', () => {
    expect(g.lanes.length).toBeGreaterThan(300)
    expect(largestComponent(g) / g.lanes.length).toBeGreaterThan(0.9)
  })

  it('keeps nearly every connector drivable', () => {
    const ok = g.connectors.filter((c) => minRadius(c.points) >= 4).length
    expect(ok / g.connectors.length).toBeGreaterThan(0.95)
  })
})

describe('routing', () => {
  const g = buildLaneGraph(fixture)
  const at = (x: number, y: number, heading?: number) => snapToLane(g, { x, y }, heading)!

  it('turns right into the one-way street', () => {
    const route = planRoute(g, at(20, 103, 0), at(103, 170))!
    expect(route).not.toBeNull()
    const end = route.path[route.path.length - 1]
    expect(end.y).toBeGreaterThan(160)
    expect(route.names[route.names.length - 1]).toBe('Oak St')
  })

  it('respects the one-way: the north part of Oak St cannot be reached', () => {
    expect(planRoute(g, at(20, 103, 0), at(100, 30))).toBeNull()
  })

  it('builds a continuous trajectory with headings and speed limits', () => {
    const route = planRoute(g, at(20, 103, 0), at(180, 103))!
    for (let i = 1; i < route.path.length; i++) {
      const a = route.path[i - 1]
      const b = route.path[i]
      expect(Math.hypot(b.x - a.x, b.y - a.y)).toBeLessThan(0.6)
    }
    expect(route.speedLimit.every((v) => v === 11)).toBe(true)
    // driving east the whole way
    expect(route.path.every((p) => Math.cos(p.theta) > 0.9)).toBe(true)
  })
})

describe('driving in Mountain View', () => {
  it('follows real routes in its lane under the speed limits', () => {
    for (const seed of [1, 2]) {
      const sim = new CitySim(cityJson as unknown as CityDef, seed)
      expect(sim.randomDestination()).toBe(true)
      let maxLat = 0
      let sumLat = 0
      let n = 0
      let overSpeed = 0
      for (let i = 0; i < 50 * 300 && sim.phase === 'driving'; i++) {
        sim.tick()
        const f = sim.follower!
        if (i > 50) {
          maxLat = Math.max(maxLat, Math.abs(f.cte))
          sumLat += Math.abs(f.cte)
          n++
        }
        overSpeed = Math.max(overSpeed, sim.car.speed - sim.route!.speedLimit[Math.min(f.index, sim.route!.speedLimit.length - 1)])
      }
      expect(sim.phase).toBe('arrived')
      expect(sumLat / n).toBeLessThan(0.3)
      expect(maxLat).toBeLessThan(1)
      expect(overSpeed).toBeLessThan(0.6)
      expect(Math.hypot(sim.car.x - sim.destination!.x, sim.car.y - sim.destination!.y)).toBeLessThan(1)
    }
  }, 120_000)
})
