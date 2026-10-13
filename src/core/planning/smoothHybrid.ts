import type { Vec2 } from '../math/vec'
import type { KnownMap } from '../world/knownMap'
import { DEFAULT_HYBRID, poseIsFree, type HybridParams, type PathPose } from './hybridAStar'

/** fraction of the planner's safety radius a pose of the final path must keep */
export const PLAN_VALIDITY_RATIO = 0.85

export interface HybridSmoothOptions {
  /** resampling distance (m) */
  spacing: number
  /** pull towards the original Hybrid A* path */
  weightData: number
  /** pull towards the neighbors (removes the zigzag of the discrete arcs) */
  weightSmooth: number
  /** push away from walls closer than `obstacleRange` */
  weightObstacle: number
  obstacleRange: number
  iterations: number
}

export const DEFAULT_SMOOTH: HybridSmoothOptions = {
  spacing: 0.25,
  weightData: 0.05,
  weightSmooth: 0.3,
  weightObstacle: 0.25,
  obstacleRange: 1,
  iterations: 300,
}

/**
 * Post-optimization of a Hybrid A* path, after Dolgov et al.: the discrete arcs make the raw
 * path wiggle (full lock left, straight, full lock right...). Each forward/reverse segment is
 * resampled and moved by gradient steps that balance three terms: stay close to the original,
 * be smooth, and keep away from known walls. The start, the goal and every gear switch stay
 * fixed. Points that end up colliding or turning tighter than the car can are pulled back
 * towards the original path, which is always feasible.
 */
export function smoothHybridPath(
  path: PathPose[],
  map: KnownMap,
  hybrid: HybridParams = DEFAULT_HYBRID,
  o: HybridSmoothOptions = DEFAULT_SMOOTH,
): PathPose[] {
  if (path.length < 3) return path.map((p) => ({ ...p }))
  const kappaMax = (Math.tan(hybrid.maxSteer) / hybrid.wheelbase) * 1.05
  const out: PathPose[] = []
  for (const seg of segmentsByGear(path)) {
    // the first pose is the switching pose, still tagged with the previous gear
    const dir = seg[seg.length - 1].dir
    const raw = resample(seg, o.spacing)
    const pts = optimize(raw, map, o)
    const blend = enforceFeasibility(pts, raw, dir, map, hybrid, kappaMax)
    const poses = withHeadings(pts, raw, blend, dir)
    // the first pose of a segment is the last of the previous one (gear switch)
    out.push(...(out.length ? poses.slice(1) : poses))
  }
  return out
}

/** Splits the path where the gear changes; the switching pose belongs to both segments. */
function segmentsByGear(path: PathPose[]): PathPose[][] {
  const segs: PathPose[][] = [[path[0]]]
  for (let i = 1; i < path.length; i++) {
    // path[0] carries the gear of the first move, so a change can only start at i > 1
    if (i > 1 && path[i].dir !== path[i - 1].dir) segs.push([path[i - 1]])
    segs[segs.length - 1].push(path[i])
  }
  return segs.filter((s) => s.length >= 2)
}

interface Sample extends Vec2 {
  /** heading of the original path at this point */
  theta: number
}

function resample(seg: PathPose[], spacing: number): Sample[] {
  const lerpAngle = (a: number, b: number, t: number) => a + Math.atan2(Math.sin(b - a), Math.cos(b - a)) * t
  const out: Sample[] = [{ x: seg[0].x, y: seg[0].y, theta: seg[0].theta }]
  let carry = 0
  for (let i = 1; i < seg.length; i++) {
    const a = seg[i - 1]
    const b = seg[i]
    const len = Math.hypot(b.x - a.x, b.y - a.y)
    let d = spacing - carry
    while (d < len) {
      const t = d / len
      out.push({ x: a.x + (b.x - a.x) * t, y: a.y + (b.y - a.y) * t, theta: lerpAngle(a.theta, b.theta, t) })
      d += spacing
    }
    carry = len - (d - spacing)
  }
  const last = seg[seg.length - 1]
  const prev = out[out.length - 1]
  // keep the exact end point; drop a sample that would sit right on top of it
  if (Math.hypot(last.x - prev.x, last.y - prev.y) < spacing * 0.4 && out.length > 1) out.pop()
  out.push({ x: last.x, y: last.y, theta: last.theta })
  return out
}

function optimize(raw: Sample[], map: KnownMap, o: HybridSmoothOptions): Vec2[] {
  const pts = raw.map((p) => ({ ...p }))
  if (pts.length < 3) return pts
  const h = map.cellSize * 0.5
  for (let it = 0; it < o.iterations; it++) {
    let change = 0
    for (let i = 1; i < pts.length - 1; i++) {
      const p = pts[i]
      let dx = o.weightData * (raw[i].x - p.x) + o.weightSmooth * (pts[i - 1].x + pts[i + 1].x - 2 * p.x)
      let dy = o.weightData * (raw[i].y - p.y) + o.weightSmooth * (pts[i - 1].y + pts[i + 1].y - 2 * p.y)
      const c = map.clearanceAt(p.x, p.y)
      if (c < o.obstacleRange) {
        // move up the clearance gradient, harder the closer the wall
        const gx = (map.clearanceAt(p.x + h, p.y) - map.clearanceAt(p.x - h, p.y)) / (2 * h)
        const gy = (map.clearanceAt(p.x, p.y + h) - map.clearanceAt(p.x, p.y - h)) / (2 * h)
        const g = Math.hypot(gx, gy)
        if (g > 1e-6) {
          const push = o.weightObstacle * (o.obstacleRange - c) * 0.1
          dx += (push * gx) / g
          dy += (push * gy) / g
        }
      }
      p.x += dx
      p.y += dy
      change += Math.abs(dx) + Math.abs(dy)
    }
    if (change < 1e-4) break
  }
  return pts
}

/** Menger curvature of three consecutive points. */
function curvature(a: Vec2, b: Vec2, c: Vec2): number {
  const ab = Math.hypot(b.x - a.x, b.y - a.y)
  const bc = Math.hypot(c.x - b.x, c.y - b.y)
  const ca = Math.hypot(a.x - c.x, a.y - c.y)
  const area2 = Math.abs((b.x - a.x) * (c.y - a.y) - (b.y - a.y) * (c.x - a.x))
  return ab * bc * ca > 1e-9 ? (2 * area2) / (ab * bc * ca) : 0
}

function tangentHeading(pts: Vec2[], i: number, dir: number): number {
  const a = pts[Math.max(0, i - 1)]
  const b = pts[Math.min(pts.length - 1, i + 1)]
  const t = Math.atan2(b.y - a.y, b.x - a.x)
  return dir > 0 ? t : t + Math.PI
}

/**
 * Pulls infeasible stretches back towards the original path. Each violation raises a blend
 * weight (0 = smoothed, 1 = original) over a window that fades out with distance, so the
 * correction has no kinks; repeated until every point is collision-free and drivable.
 */
function enforceFeasibility(pts: Vec2[], raw: Sample[], dir: number, map: KnownMap, hybrid: HybridParams, kappaMax: number): Float64Array {
  const n = pts.length
  const blend = new Float64Array(n)
  if (n < 3) return blend
  const smooth = pts.map((p) => ({ ...p }))
  // same tolerance the exploration uses to decide that a plan is still valid; the clearance
  // field is per 25 cm cell, so a point between two checked poses can read a bit less margin
  const check = { ...hybrid, radius: hybrid.radius * PLAN_VALIDITY_RATIO }
  const WINDOW = 6
  const apply = () => {
    for (let i = 0; i < n; i++) {
      pts[i].x = smooth[i].x + (raw[i].x - smooth[i].x) * blend[i]
      pts[i].y = smooth[i].y + (raw[i].y - smooth[i].y) * blend[i]
    }
  }
  const violations = () => {
    const bad: number[] = []
    for (let i = 1; i < n - 1; i++) {
      if (curvature(pts[i - 1], pts[i], pts[i + 1]) > kappaMax || !poseIsFree(map, { ...pts[i], theta: heading(pts, raw, blend, i, dir) }, check)) bad.push(i)
    }
    return bad
  }
  for (let pass = 0; pass < 40; pass++) {
    const bad = violations()
    if (!bad.length) return blend
    // after a few gentle passes, fall back harder (the original path is always feasible)
    const step = pass < 10 ? 0.35 : 1
    for (const v of bad) {
      for (let j = Math.max(1, v - WINDOW); j <= Math.min(n - 2, v + WINDOW); j++) {
        const w = 1 - Math.abs(j - v) / (WINDOW + 1)
        blend[j] = Math.min(1, blend[j] + step * w)
      }
    }
    apply()
  }
  // still infeasible somewhere (very rare): use the original path for the whole segment
  blend.fill(1)
  apply()
  return blend
}

/** Points still on the original path keep its exact heading; moved points follow the tangent. */
function heading(pts: Vec2[], raw: Sample[], blend: Float64Array, i: number, dir: number): number {
  return blend[i] >= 1 || i === 0 || i === pts.length - 1 ? raw[i].theta : tangentHeading(pts, i, dir)
}

function withHeadings(pts: Vec2[], raw: Sample[], blend: Float64Array, dir: number): PathPose[] {
  return pts.map((p, i) => ({ x: p.x, y: p.y, theta: heading(pts, raw, blend, i, dir), dir }))
}
