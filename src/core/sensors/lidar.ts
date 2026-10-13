import type { Pose } from '../math/vec'
import type { Rng } from '../math/rng'
import type { Segment } from '../world/map'

export interface LidarConfig {
  /** beam angles relative to the vehicle heading */
  angles: number[]
  maxRange: number
  /** std deviation of range noise (meters) */
  noise: number
  /** probability that a beam returns a bogus short range (reflections, dust...) */
  spurious?: number
}

export function evenlySpacedBeams(count: number): number[] {
  return Array.from({ length: count }, (_, i) => (i / count) * Math.PI * 2 - Math.PI)
}

/**
 * Distance from (ox, oy) along direction (dx, dy) to the closest wall,
 * or `maxRange` when nothing is hit. Ray/segment intersection as in
 * https://en.wikipedia.org/wiki/Line%E2%80%93line_intersection
 */
export function castRay(ox: number, oy: number, dx: number, dy: number, walls: Segment[], maxRange: number): number {
  let best = maxRange
  for (let k = 0; k < walls.length; k++) {
    const w = walls[k]
    const x1 = w.a.x
    const y1 = w.a.y
    const ex = w.b.x - x1
    const ey = w.b.y - y1
    const den = ex * dy - ey * dx
    if (den === 0) continue
    const qx = x1 - ox
    const qy = y1 - oy
    // t: position along the ray, u: position along the wall segment
    const t = (ex * qy - ey * qx) / den
    const u = (dx * qy - dy * qx) / den
    if (t > 0 && t < best && u >= 0 && u <= 1) best = t
  }
  return best
}

/** Expected ranges for every beam from `pose` (no noise). */
export function expectedScan(pose: Pose, cfg: LidarConfig, walls: Segment[], out: Float64Array): Float64Array {
  for (let i = 0; i < cfg.angles.length; i++) {
    const a = pose.theta + cfg.angles[i]
    out[i] = castRay(pose.x, pose.y, Math.cos(a), Math.sin(a), walls, cfg.maxRange)
  }
  return out
}

/** Simulated noisy measurement from the real vehicle. */
export function measure(pose: Pose, cfg: LidarConfig, walls: Segment[], rng: Rng): Float64Array {
  const z = expectedScan(pose, cfg, walls, new Float64Array(cfg.angles.length))
  for (let i = 0; i < z.length; i++) {
    if (z[i] < cfg.maxRange) z[i] = Math.max(0, rng.gaussian(z[i], cfg.noise))
    if (cfg.spurious && rng.next() < cfg.spurious) z[i] = rng.range(0.3, Math.min(z[i], cfg.maxRange))
  }
  return z
}
