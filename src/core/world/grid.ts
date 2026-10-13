import type { Vec2 } from '../math/vec'
import { gridDistance } from '../planning/gridDistance'
import type { World } from './map'

export const FREE = 0
export const WALL = 1
/** free space, but too close to a wall for the car body */
export const INFLATED = 2

/**
 * Occupancy grid built by rasterizing the world walls.
 * Cells are square and indexed as `row * cols + col`, with col along +x and row along +y.
 */
export class OccupancyGrid {
  readonly cols: number
  readonly rows: number
  readonly cells: Uint8Array
  /** distance (in cells) from each cell to the nearest wall cell */
  readonly clearance: Float32Array

  constructor(
    readonly world: World,
    readonly cellSize: number,
    inflateRadius: number,
  ) {
    this.cols = Math.ceil(world.width / cellSize)
    this.rows = Math.ceil(world.height / cellSize)
    this.cells = new Uint8Array(this.cols * this.rows)
    for (const w of world.walls) this.rasterize(w.a, w.b)
    this.clearance = this.computeClearance()
    const inflateCells = inflateRadius / cellSize
    for (let i = 0; i < this.cells.length; i++) {
      if (this.cells[i] === FREE && this.clearance[i] <= inflateCells) this.cells[i] = INFLATED
    }
  }

  index(col: number, row: number): number {
    return row * this.cols + col
  }

  inBounds(col: number, row: number): boolean {
    return col >= 0 && row >= 0 && col < this.cols && row < this.rows
  }

  worldToCell(p: Vec2): [number, number] {
    return [Math.floor(p.x / this.cellSize), Math.floor(p.y / this.cellSize)]
  }

  cellCenter(col: number, row: number): Vec2 {
    return { x: (col + 0.5) * this.cellSize, y: (row + 0.5) * this.cellSize }
  }

  /** True when the point is inside the map and not on a wall cell. */
  isFree(p: Vec2, allowInflated = true): boolean {
    const [c, r] = this.worldToCell(p)
    if (!this.inBounds(c, r)) return false
    const v = this.cells[this.index(c, r)]
    return v === FREE || (allowInflated && v === INFLATED)
  }

  /** Nearest cell (BFS) that is fully free, or null if none. */
  nearestFree(col: number, row: number): [number, number] | null {
    if (!this.inBounds(col, row)) {
      col = Math.min(this.cols - 1, Math.max(0, col))
      row = Math.min(this.rows - 1, Math.max(0, row))
    }
    const seen = new Uint8Array(this.cells.length)
    const queue = [this.index(col, row)]
    seen[queue[0]] = 1
    for (let head = 0; head < queue.length; head++) {
      const i = queue[head]
      if (this.cells[i] === FREE) return [i % this.cols, Math.floor(i / this.cols)]
      const c = i % this.cols
      const r = Math.floor(i / this.cols)
      for (const [dc, dr] of [[1, 0], [-1, 0], [0, 1], [0, -1]]) {
        const nc = c + dc
        const nr = r + dr
        if (!this.inBounds(nc, nr)) continue
        const j = this.index(nc, nr)
        if (!seen[j]) {
          seen[j] = 1
          queue.push(j)
        }
      }
    }
    return null
  }

  private rasterize(a: Vec2, b: Vec2) {
    const len = Math.hypot(b.x - a.x, b.y - a.y)
    const steps = Math.max(1, Math.ceil(len / (this.cellSize * 0.25)))
    for (let s = 0; s <= steps; s++) {
      const t = s / steps
      const x = Math.min(a.x + (b.x - a.x) * t, this.world.width - 1e-6)
      const y = Math.min(a.y + (b.y - a.y) * t, this.world.height - 1e-6)
      const [c, r] = this.worldToCell({ x, y })
      if (this.inBounds(c, r)) this.cells[this.index(c, r)] = WALL
    }
  }

  /** Distance (in cells) from every cell to the nearest wall cell. */
  private computeClearance(): Float32Array {
    const walls: number[] = []
    for (let i = 0; i < this.cells.length; i++) if (this.cells[i] === WALL) walls.push(i)
    return gridDistance(this.cols, this.rows, walls, new Float32Array(this.cells.length))
  }
}
