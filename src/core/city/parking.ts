import type { Pose, Vec2 } from '../math/vec'
import type { CityDef } from './cityDef'

export const SPOT_WIDTH = 3
export const SPOT_DEPTH = 5.5
const AISLE = 7

/** A 90° parking spot. `heading` points into the spot, from the aisle. */
export interface ParkingSpot {
  lot: number
  center: Vec2
  heading: number
  /** row and position in the row, to find neighbors */
  row: number
  index: number
  occupied: boolean
}

export interface ParkingLot {
  id: number
  outline: Vec2[]
  spots: ParkingSpot[]
}

/** deterministic pseudo-random in [0, 1) */
const hash01 = (n: number) => {
  let h = n ^ 0x9e3779b9
  h = Math.imul(h ^ (h >>> 16), 0x85ebca6b)
  h = Math.imul(h ^ (h >>> 13), 0xc2b2ae35)
  return ((h ^ (h >>> 16)) >>> 0) / 4294967296
}

function inside(poly: Vec2[], p: Vec2): boolean {
  let hit = false
  for (let i = 0, j = poly.length - 1; i < poly.length; j = i++) {
    const a = poly[i]
    const b = poly[j]
    if (a.y > p.y !== b.y > p.y && p.x < ((b.x - a.x) * (p.y - a.y)) / (b.y - a.y) + a.x) hit = !hit
  }
  return hit
}

/** Corners of a rectangle centered at `c`, `length` along `heading` and `width` across. */
export function rectangle(c: Vec2, heading: number, length: number, width: number): Vec2[] {
  const ux = Math.cos(heading)
  const uy = Math.sin(heading)
  const corner = (a: number, b: number) => ({ x: c.x + ux * a - uy * b, y: c.y + uy * a + ux * b })
  return [corner(-length / 2, -width / 2), corner(length / 2, -width / 2), corner(length / 2, width / 2), corner(-length / 2, width / 2)]
}

/**
 * Lays out 90° spots in a surface lot: rows parallel to its longest side, alternating
 * [row | aisle | row] modules, keeping the spots that fit entirely; about 70% are taken.
 */
export function layoutLot(id: number, outline: Vec2[], occupancy = 0.7): ParkingLot {
  // orientation: the longest edge
  let best = 0
  let angle = 0
  for (let i = 0; i < outline.length; i++) {
    const a = outline[i]
    const b = outline[(i + 1) % outline.length]
    const len = Math.hypot(b.x - a.x, b.y - a.y)
    if (len > best) {
      best = len
      angle = Math.atan2(b.y - a.y, b.x - a.x)
    }
  }
  const ux = Math.cos(angle)
  const uy = Math.sin(angle)
  const toLocal = (p: Vec2) => ({ u: p.x * ux + p.y * uy, v: -p.x * uy + p.y * ux })
  const toWorld = (u: number, v: number) => ({ x: u * ux - v * uy, y: u * uy + v * ux })
  const loc = outline.map(toLocal)
  const u0 = Math.min(...loc.map((p) => p.u))
  const u1 = Math.max(...loc.map((p) => p.u))
  const v0 = Math.min(...loc.map((p) => p.v))
  const v1 = Math.max(...loc.map((p) => p.v))

  const spots: ParkingSpot[] = []
  let row = 0
  for (let v = v0 + 0.8; v + SPOT_DEPTH * 2 + AISLE <= v1 + 1e-9; v += SPOT_DEPTH * 2 + AISLE) {
    // a row below the aisle (spots face -v) and one above it (spots face +v)
    for (const [vc, facing] of [
      [v + SPOT_DEPTH / 2, -1],
      [v + SPOT_DEPTH + AISLE + SPOT_DEPTH / 2, 1],
    ] as const) {
      let index = 0
      for (let u = u0 + 1 + SPOT_WIDTH / 2; u <= u1 - 1 - SPOT_WIDTH / 2; u += SPOT_WIDTH) {
        const c = toWorld(u, vc)
        // from the aisle into the spot: along -v below the aisle, +v above (v = (-uy, ux) in the world)
        const heading = Math.atan2(facing * ux, -facing * uy)
        // the whole spot plus a curb margin must be inside the lot
        const corners = rectangle(c, heading, SPOT_DEPTH + 0.8, SPOT_WIDTH)
        if (!corners.every((p) => inside(outline, p))) continue
        spots.push({ lot: id, center: c, heading, row, index: index++, occupied: hash01(id * 1000 + spots.length) < occupancy })
      }
      row++
    }
  }
  return { id, outline, spots }
}

export function buildParking(city: CityDef): ParkingLot[] {
  return (city.parking ?? []).map((l) => layoutLot(l.id, l.pts.map(([x, y]) => ({ x, y }))))
}

/** Free spots with a parked car on each side, the classic "park between two cars". */
export function spotsBetweenCars(lot: ParkingLot): ParkingSpot[] {
  const at = (row: number, index: number) => lot.spots.find((s) => s.row === row && s.index === index)
  return lot.spots.filter((s) => !s.occupied && at(s.row, s.index - 1)?.occupied && at(s.row, s.index + 1)?.occupied)
}

/** Rear-axle pose that puts a car (body from -0.78 to 4.26 m along it) in the middle of the spot. */
export function parkedPose(spot: ParkingSpot, frontIn: boolean): Pose {
  // distance from the rear axle to the middle of the body
  const bodyCenter = 5.04 / 2 - 0.78
  const h = frontIn ? spot.heading : spot.heading + Math.PI
  return { x: spot.center.x - Math.cos(h) * bodyCenter, y: spot.center.y - Math.sin(h) * bodyCenter, theta: Math.atan2(Math.sin(h), Math.cos(h)) }
}
