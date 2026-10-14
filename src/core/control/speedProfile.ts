import type { Vec2 } from '../math/vec'

export interface SpeedLimits {
  /** top speed of the segment (m/s, positive) */
  vMax: number
  /** max lateral acceleration in curves (m/s²) */
  aLat: number
  /** max acceleration and braking (m/s², positive) */
  accel: number
  brake: number
  /**
   * distance (m) over which curvature is measured; 0 = adjacent points. A window avoids
   * reading a tiny kink of a mapped polyline as a sharp curve.
   */
  curvatureWindow?: number
}

/** Menger curvature at the middle of three points (1/m). */
export function curvature(a: Vec2, b: Vec2, c: Vec2): number {
  const ab = Math.hypot(b.x - a.x, b.y - a.y)
  const bc = Math.hypot(c.x - b.x, c.y - b.y)
  const ca = Math.hypot(a.x - c.x, a.y - c.y)
  const area2 = Math.abs((b.x - a.x) * (c.y - a.y) - (b.y - a.y) * (c.x - a.x))
  return ab * bc * ca > 1e-9 ? (2 * area2) / (ab * bc * ca) : 0
}

/** Cumulative arc length at every point. */
export function arcLengths(points: Vec2[]): number[] {
  const s = [0]
  for (let i = 1; i < points.length; i++) s.push(s[i - 1] + Math.hypot(points[i].x - points[i - 1].x, points[i].y - points[i - 1].y))
  return s
}

/**
 * Speed limit (unsigned) at every point of a segment, as planners do before handing a
 * trajectory to the controller (optionally capped by a per-point limit, such as street limits):
 *  1. curves: v ≤ √(a_lat / κ), so the lateral acceleration stays bounded;
 *  2. backward pass: the car must be able to brake down to 0 at the end of the segment
 *     (a gear switch or the goal): v_i ≤ √(v_{i+1}² + 2·brake·ds).
 * Accelerating from the current speed is handled when the reference is sampled in time.
 */
export function speedProfile(points: Vec2[], lim: SpeedLimits, pointLimits?: number[]): number[] {
  const n = points.length
  // optional per-point limit (e.g. the speed limit of each street)
  const v = Array.from({ length: n }, (_, i) => Math.min(lim.vMax, pointLimits?.[i] ?? Infinity))
  const s = arcLengths(points)
  const half = (lim.curvatureWindow ?? 0) / 2
  let a = 0
  let c = 0
  for (let i = 1; i < n - 1; i++) {
    // neighbors at least half a window behind and ahead
    while (a < i - 1 && s[i] - s[a + 1] >= half) a++
    if (c <= i) c = i + 1
    while (c < n - 1 && s[c] - s[i] < half) c++
    const k = curvature(points[half > 0 ? a : i - 1], points[i], points[half > 0 ? c : i + 1])
    if (k > 1e-6) v[i] = Math.min(v[i], Math.sqrt(lim.aLat / k))
  }
  v[n - 1] = 0
  for (let i = n - 2; i >= 0; i--) {
    const ds = s[i + 1] - s[i]
    v[i] = Math.min(v[i], Math.sqrt(v[i + 1] * v[i + 1] + 2 * lim.brake * ds))
  }
  return v
}
