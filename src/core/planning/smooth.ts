import type { Vec2 } from '../math/vec'
import type { OccupancyGrid } from '../world/grid'

export interface SmoothOptions {
  /** pull towards the original path */
  weightData: number
  /** pull towards neighbors (smoothness) */
  weightSmooth: number
  tolerance: number
}

/**
 * Gradient-descent path smoothing from the Udacity course. Endpoints stay fixed.
 * After converging, any point that ended up in a wall (or inflated) cell is pulled back
 * towards its original A* cell, which the 2D version did not do.
 */
export function smoothPath(path: Vec2[], grid: OccupancyGrid, opts: SmoothOptions): Vec2[] {
  const out = path.map((p) => ({ ...p }))
  if (path.length < 3) return out
  const { weightData: a, weightSmooth: b, tolerance } = opts

  for (let iter = 0; iter < 10_000; iter++) {
    let change = 0
    for (let i = 1; i < out.length - 1; i++) {
      for (const k of ['x', 'y'] as const) {
        const old = out[i][k]
        out[i][k] += a * (path[i][k] - out[i][k]) + b * (out[i - 1][k] + out[i + 1][k] - 2 * out[i][k])
        change += Math.abs(old - out[i][k])
      }
    }
    if (change < tolerance) break
  }

  const ok = (i: number) => grid.isFree(out[i], false) && segmentFree(grid, out[i - 1], out[i])
  for (let i = 1; i < out.length - 1; i++) {
    for (let t = 0; t < 8 && !ok(i); t++) {
      out[i].x = (out[i].x + path[i].x) / 2
      out[i].y = (out[i].y + path[i].y) / 2
    }
    if (!ok(i)) out[i] = { ...path[i] }
  }
  return out
}

function segmentFree(grid: OccupancyGrid, a: Vec2, b: Vec2): boolean {
  const steps = Math.max(1, Math.ceil(Math.hypot(b.x - a.x, b.y - a.y) / (grid.cellSize * 0.25)))
  for (let s = 0; s <= steps; s++) {
    const t = s / steps
    if (!grid.isFree({ x: a.x + (b.x - a.x) * t, y: a.y + (b.y - a.y) * t })) return false
  }
  return true
}
