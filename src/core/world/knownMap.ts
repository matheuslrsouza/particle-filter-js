import type { Pose } from '../math/vec'
import type { LidarConfig } from '../sensors/lidar'
import type { Segment } from './map'
import { gridDistance } from '../planning/gridDistance'

export const UNKNOWN = 0
export const KNOWN_FREE = 1
export const KNOWN_WALL = 2

/** 'logodds': probabilistic occupancy grid; 'binary': a single lidar hit makes a wall forever */
export type MappingMode = 'logodds' | 'binary'

// log-odds increments and thresholds: ~2 hits make a wall, a few free observations erase it
const L_HIT = 0.85
const L_MISS = 0.4
/** confirmed walls lose evidence more slowly (hysteresis against grazing beams) */
const L_MISS_WALL = 0.1
const L_MIN = -2
const L_MAX = 3.5
const WALL_AT = 1.2
const FREE_AT = -0.2

/**
 * The map the robot builds by itself in exploration mode. Every cell starts unknown.
 * In log-odds mode (the standard occupancy grid) each cell accumulates evidence: a lidar hit
 * adds, a beam passing through subtracts. A cell becomes a wall only after repeated hits and
 * becomes free again if later beams go through it, so spurious readings don't leave phantom
 * walls. The planner treats unknown cells as free (optimistic) and replans when walls appear.
 */
export class KnownMap {
  readonly cols: number
  readonly rows: number
  readonly state: Uint8Array
  /** distance (m) from each cell center to the nearest known wall cell center */
  readonly dist: Float32Array
  readonly logOdds: Float32Array
  mode: MappingMode = 'logodds'
  /** wall cells that turned free during the last scan */
  lastRemoved: number[] = []
  version = 0
  private touched: Uint32Array
  /** cells felt by the bumper: physical contact is certain, they are never erased */
  private pinned: Uint8Array
  private scanId = 0

  constructor(
    readonly width: number,
    readonly height: number,
    readonly cellSize: number,
  ) {
    this.cols = Math.ceil(width / cellSize)
    this.rows = Math.ceil(height / cellSize)
    this.state = new Uint8Array(this.cols * this.rows)
    this.dist = new Float32Array(this.cols * this.rows).fill(Infinity)
    this.logOdds = new Float32Array(this.cols * this.rows)
    this.touched = new Uint32Array(this.cols * this.rows)
    this.pinned = new Uint8Array(this.cols * this.rows)
  }

  /** Forgets everything. */
  clear(): void {
    this.state.fill(UNKNOWN)
    this.dist.fill(Infinity)
    this.logOdds.fill(0)
    this.pinned.fill(0)
    this.lastRemoved = []
    this.version++
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
    this.lastRemoved = []
    const hits: number[] = []
    const misses: number[] = []
    // each cell is updated at most once per scan, however many beams cross it
    const id = ++this.scanId
    const HIT = 1
    const MISS = 2
    const step = this.cellSize * 0.5
    for (let b = 0; b < z.length; b++) {
      const a = pose.theta + lidar.angles[b]
      const dx = Math.cos(a)
      const dy = Math.sin(a)
      const hit = z[b] < lidar.maxRange
      // inverse sensor model: free up to one cell before the hit; around the hit it's uncertain
      // (walls often lie on a cell boundary, so the hit can land in either neighbor cell)
      const freeUntil = hit ? z[b] - this.cellSize * (this.mode === 'binary' ? 0.6 : 1) : z[b]
      for (let d = 0; d < freeUntil; d += step) {
        const i = this.cellOf(pose.x + dx * d, pose.y + dy * d)
        if (i >= 0 && this.touched[i] !== id * 4 + HIT && this.touched[i] !== id * 4 + MISS) {
          this.touched[i] = id * 4 + MISS
          misses.push(i)
        }
      }
      if (hit) {
        // nudge past the surface so the point lands inside the wall's cell
        const i = this.cellOf(pose.x + dx * (z[b] + 0.02), pose.y + dy * (z[b] + 0.02))
        if (i >= 0 && this.touched[i] !== id * 4 + HIT) {
          this.touched[i] = id * 4 + HIT
          hits.push(i)
        }
      }
    }
    const fresh: number[] = []
    if (this.mode === 'binary') {
      for (const i of misses) if (this.state[i] === UNKNOWN && this.touched[i] === id * 4 + MISS) this.state[i] = KNOWN_FREE
      for (const i of hits) {
        if (this.state[i] !== KNOWN_WALL) {
          this.state[i] = KNOWN_WALL
          fresh.push(i)
        }
      }
    } else {
      for (const i of hits) this.logOdds[i] = Math.min(L_MAX, this.logOdds[i] + L_HIT)
      for (const i of misses) {
        if (this.touched[i] !== id * 4 + MISS || this.pinned[i]) continue
        const miss = this.state[i] === KNOWN_WALL ? L_MISS_WALL : L_MISS
        this.logOdds[i] = Math.max(L_MIN, this.logOdds[i] - miss)
      }
      for (const list of [hits, misses]) {
        for (const i of list) {
          const l = this.logOdds[i]
          const before = this.state[i]
          // hysteresis: a wall stays a wall until the evidence says free
          if (l >= WALL_AT) this.state[i] = KNOWN_WALL
          else if (l <= FREE_AT) this.state[i] = KNOWN_FREE
          if (before !== KNOWN_WALL && this.state[i] === KNOWN_WALL) fresh.push(i)
          if (before === KNOWN_WALL && this.state[i] !== KNOWN_WALL) this.lastRemoved.push(i)
        }
      }
    }
    if (fresh.length || this.lastRemoved.length) {
      this.recomputeDistances()
      this.version++
    }
    return fresh
  }

  /** Marks specific cells as walls (e.g. felt by the bumper). Returns the ones that were new. */
  markWalls(cells: number[]): number[] {
    const fresh = cells.filter((i) => this.state[i] !== KNOWN_WALL)
    for (const i of cells) {
      this.logOdds[i] = L_MAX
      this.pinned[i] = 1
    }
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
        if (i >= 0) {
          this.state[i] = KNOWN_WALL
          this.logOdds[i] = L_MAX
        }
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
