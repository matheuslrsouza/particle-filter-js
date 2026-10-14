import type { Vec2 } from '../math/vec'
import { KnownMap } from '../world/knownMap'
import type { LaneGraph } from './roadNetwork'

/**
 * A local occupancy grid of the drivable surface around a point, for maneuvers off the lane
 * graph (U-turns, parking): road surface and junctions are free, everything else is a wall,
 * plus the given obstacle polygons (barriers, parked cars). Coordinates are local: subtract
 * `origin` from world positions.
 */
export interface DrivableGrid {
  map: KnownMap
  origin: Vec2
}

export function drivableGrid(
  g: LaneGraph,
  center: Vec2,
  size: number,
  cellSize: number,
  obstacles: Vec2[][] = [],
  extraFree: Vec2[][] = [],
): DrivableGrid {
  const origin = { x: center.x - size / 2, y: center.y - size / 2 }
  const map = new KnownMap(size, size, cellSize)
  const free = new Uint8Array(map.cols * map.rows)
  const near = (p: Vec2, margin: number) =>
    p.x > origin.x - margin && p.y > origin.y - margin && p.x < origin.x + size + margin && p.y < origin.y + size + margin

  // road surface: cells whose center lies between the left and right edges of a segment
  for (const s of g.segments) {
    const c = s.center
    if (!c.some((p) => near(p, 40))) continue
    for (let i = 1; i < c.length; i++) {
      const a = c[i - 1]
      const b = c[i]
      const len = Math.hypot(b.x - a.x, b.y - a.y)
      if (len < 1e-6) continue
      const ux = (b.x - a.x) / len
      const uy = (b.y - a.y) / len
      // right normal (x east, y south)
      const nx = -uy
      const ny = ux
      fillPolygon(map, origin, free, [
        { x: a.x + nx * s.rightWidth, y: a.y + ny * s.rightWidth },
        { x: b.x + nx * s.rightWidth, y: b.y + ny * s.rightWidth },
        { x: b.x - nx * s.leftWidth, y: b.y - ny * s.leftWidth },
        { x: a.x - nx * s.leftWidth, y: a.y - ny * s.leftWidth },
      ], 1)
    }
  }
  // junctions: a disc of their radius
  for (const j of g.junctions) {
    if (!j.segments.length || !near(j.pos, j.radius + 5)) continue
    const ring: Vec2[] = []
    for (let k = 0; k < 16; k++) ring.push({ x: j.pos.x + Math.cos((k / 16) * Math.PI * 2) * j.radius, y: j.pos.y + Math.sin((k / 16) * Math.PI * 2) * j.radius })
    fillPolygon(map, origin, free, ring, 1)
  }
  for (const poly of extraFree) fillPolygon(map, origin, free, poly, 1)
  for (const poly of obstacles) fillPolygon(map, origin, free, poly, 0)

  const walls: number[] = []
  for (let i = 0; i < free.length; i++) if (!free[i]) walls.push(i)
  map.markWalls(walls)
  return { map, origin }
}

/** Sets cells whose center is inside the polygon (world coordinates) to `value`. */
function fillPolygon(map: KnownMap, origin: Vec2, out: Uint8Array, poly: Vec2[], value: number) {
  const cs = map.cellSize
  const xs = poly.map((p) => p.x - origin.x)
  const ys = poly.map((p) => p.y - origin.y)
  const c0 = Math.max(0, Math.floor(Math.min(...xs) / cs))
  const c1 = Math.min(map.cols - 1, Math.floor(Math.max(...xs) / cs))
  const r0 = Math.max(0, Math.floor(Math.min(...ys) / cs))
  const r1 = Math.min(map.rows - 1, Math.floor(Math.max(...ys) / cs))
  for (let r = r0; r <= r1; r++) {
    for (let c = c0; c <= c1; c++) {
      const px = (c + 0.5) * cs
      const py = (r + 0.5) * cs
      if (inside(xs, ys, px, py)) out[r * map.cols + c] = value
    }
  }
}

function inside(xs: number[], ys: number[], x: number, y: number): boolean {
  let hit = false
  for (let i = 0, j = xs.length - 1; i < xs.length; j = i++) {
    if (ys[i] > y !== ys[j] > y && x < ((xs[j] - xs[i]) * (y - ys[i])) / (ys[j] - ys[i]) + xs[i]) hit = !hit
  }
  return hit
}
