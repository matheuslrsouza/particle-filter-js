import type { Pose } from '../math/vec'
import type { LidarConfig } from '../sensors/lidar'
import type { Segment } from './map'
import { gridDistance } from '../planning/gridDistance'

export const UNKNOWN = 0
export const KNOWN_FREE = 1
export const KNOWN_WALL = 2

/**
 * The map the robot builds by itself in exploration mode. Every cell starts unknown;
 * lidar hits mark walls and the cells crossed by a beam become free. The planner treats
 * unknown cells as free (optimistic assumption) and replans when a wall shows up.
 */
export class KnownMap {
  readonly cols: number
  readonly rows: number
  readonly state: Uint8Array
  /** distance (m) from each cell center to the nearest known wall cell center */
  readonly dist: Float32Array
  version = 0

  constructor(
    readonly width: number,
    readonly height: number,
    readonly cellSize: number,
  ) {
    this.cols = Math.ceil(width / cellSize)
    this.rows = Math.ceil(height / cellSize)
    this.state = new Uint8Array(this.cols * this.rows)
    this.dist = new Float32Array(this.cols * this.rows).fill(Infinity)
  }

  cellOf(x: number, y: number): number {
    const c = Math.floor(x / this.cellSize)
    const r = Math.floor(y / this.cellSize)
    if (c < 0 || r < 0 || c >= this.cols || r >= this.rows) return -1
    return r * this.cols + c
  }

  /**
   * Distance (m) from a point to the nearest known wall surface, bilinearly interpolated
   * between cell centers. It must be continuous: with a per-cell value, a point 2 cm away
   * from a checked pose could read a whole cell less margin. 0 outside the map.
   */
  clearanceAt(x: number, y: number): number {
    if (x < 0 || y < 0 || x >= this.width || y >= this.height) return 0
    const cs = this.cellSize
    const fx = x / cs - 0.5
    const fy = y / cs - 0.5
    const c0 = Math.floor(fx)
    const r0 = Math.floor(fy)
    const tx = fx - c0
    const ty = fy - r0
    const d =
      this.distAt(c0, r0) * (1 - tx) * (1 - ty) +
      this.distAt(c0 + 1, r0) * tx * (1 - ty) +
      this.distAt(c0, r0 + 1) * (1 - tx) * ty +
      this.distAt(c0 + 1, r0 + 1) * tx * ty
    return Math.max(0, d - cs * 0.5)
  }

  private distAt(c: number, r: number): number {
    // beyond the border (half a cell at the edges) counts as a wall
    if (c < 0 || r < 0 || c >= this.cols || r >= this.rows) return 0
    const d = this.dist[r * this.cols + c]
    return d === Infinity ? 1e3 : d
  }

  /** Adds a scan taken from `pose`. Returns the indices of newly discovered wall cells. */
  integrateScan(pose: Pose, z: Float64Array, lidar: LidarConfig): number[] {
    const fresh: number[] = []
    const step = this.cellSize * 0.5
    for (let b = 0; b < z.length; b++) {
      const a = pose.theta + lidar.angles[b]
      const dx = Math.cos(a)
      const dy = Math.sin(a)
      const hit = z[b] < lidar.maxRange
      const freeUntil = hit ? z[b] - this.cellSize * 0.6 : z[b]
      for (let d = 0; d < freeUntil; d += step) {
        const i = this.cellOf(pose.x + dx * d, pose.y + dy * d)
        if (i >= 0 && this.state[i] === UNKNOWN) this.state[i] = KNOWN_FREE
      }
      if (hit) {
        // nudge past the surface so the point lands inside the wall's cell
        const i = this.cellOf(pose.x + dx * (z[b] + 0.02), pose.y + dy * (z[b] + 0.02))
        if (i >= 0 && this.state[i] !== KNOWN_WALL) {
          this.state[i] = KNOWN_WALL
          fresh.push(i)
        }
      }
    }
    if (fresh.length) {
      this.recomputeDistances()
      this.version++
    }
    return fresh
  }

  /** Marks specific cells as walls (e.g. felt by the bumper). Returns the ones that were new. */
  markWalls(cells: number[]): number[] {
    const fresh = cells.filter((i) => this.state[i] !== KNOWN_WALL)
    for (const i of fresh) this.state[i] = KNOWN_WALL
    if (fresh.length) {
      this.recomputeDistances()
      this.version++
    }
    return fresh
  }

  /** Marks every wall as known (the map is given instead of explored). */
  revealWalls(walls: Segment[]): void {
    for (const w of walls) {
      const len = Math.hypot(w.b.x - w.a.x, w.b.y - w.a.y)
      const n = Math.max(1, Math.ceil(len / (this.cellSize * 0.25)))
      for (let s = 0; s <= n; s++) {
        const x = Math.min(w.a.x + ((w.b.x - w.a.x) * s) / n, this.width - 1e-6)
        const y = Math.min(w.a.y + ((w.b.y - w.a.y) * s) / n, this.height - 1e-6)
        const i = this.cellOf(x, y)
        if (i >= 0) this.state[i] = KNOWN_WALL
      }
    }
    this.recomputeDistances()
    this.version++
  }

  /** Fraction of cells that are no longer unknown. */
  explored(): number {
    let n = 0
    for (let i = 0; i < this.state.length; i++) if (this.state[i] !== UNKNOWN) n++
    return n / this.state.length
  }

  private recomputeDistances(): void {
    const walls: number[] = []
    for (let i = 0; i < this.state.length; i++) if (this.state[i] === KNOWN_WALL) walls.push(i)
    gridDistance(this.cols, this.rows, walls, this.dist, this.cellSize)
  }
}
