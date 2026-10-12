import type { Vec2 } from '../math/vec'
import { FREE, type OccupancyGrid } from '../world/grid'
import { MinHeap } from './heap'

export interface AStarOptions {
  /** extra cost per step for cells close to walls (keeps the car in the middle of corridors) */
  wallPenalty: number
  /** clearance (cells) under which the penalty applies */
  penaltyRange: number
}

export interface AStarResult {
  /** cell centers from start to goal (inclusive) */
  path: Vec2[]
  /** grid indices in the order they were expanded, for animation */
  expanded: number[]
}

const NEIGHBORS: [number, number, number][] = [
  [1, 0, 1], [-1, 0, 1], [0, 1, 1], [0, -1, 1],
  [1, 1, Math.SQRT2], [1, -1, Math.SQRT2], [-1, 1, Math.SQRT2], [-1, -1, Math.SQRT2],
]

/**
 * 8-connected A* over the occupancy grid. The octile heuristic is admissible because the
 * wall penalty only ever adds cost. The start cell may lie in inflated space (the car
 * can be close to a wall), but no other inflated cell is entered.
 */
export function astar(grid: OccupancyGrid, start: Vec2, goal: Vec2, opts: AStarOptions): AStarResult | null {
  const [sc, sr] = grid.worldToCell(start)
  const [gc, gr] = grid.worldToCell(goal)
  if (!grid.inBounds(sc, sr) || !grid.inBounds(gc, gr)) return null
  const startIdx = grid.index(sc, sr)
  const goalIdx = grid.index(gc, gr)
  if (grid.cells[goalIdx] !== FREE) return null

  const n = grid.cells.length
  const g = new Float64Array(n).fill(Infinity)
  const parent = new Int32Array(n).fill(-1)
  const closed = new Uint8Array(n)
  const open = new MinHeap()
  const expanded: number[] = []

  const h = (c: number, r: number) => {
    const dx = Math.abs(c - gc)
    const dy = Math.abs(r - gr)
    return dx + dy + (Math.SQRT2 - 2) * Math.min(dx, dy)
  }
  const passable = (i: number) => grid.cells[i] === FREE || i === startIdx

  g[startIdx] = 0
  open.push(startIdx, h(sc, sr))

  while (open.size) {
    const cur = open.pop()!
    if (closed[cur]) continue
    closed[cur] = 1
    expanded.push(cur)
    if (cur === goalIdx) break

    const c = cur % grid.cols
    const r = Math.floor(cur / grid.cols)
    for (const [dc, dr, cost] of NEIGHBORS) {
      const nc = c + dc
      const nr = r + dr
      if (!grid.inBounds(nc, nr)) continue
      const ni = grid.index(nc, nr)
      if (closed[ni] || !passable(ni)) continue
      // no corner cutting on diagonals
      if (dc && dr && (!passable(grid.index(c + dc, r)) || !passable(grid.index(c, r + dr)))) continue
      const clearance = grid.clearance[ni]
      const penalty = clearance < opts.penaltyRange ? opts.wallPenalty * (opts.penaltyRange - clearance) : 0
      const ng = g[cur] + cost * (1 + penalty)
      if (ng < g[ni]) {
        g[ni] = ng
        parent[ni] = cur
        open.push(ni, ng + h(nc, nr))
      }
    }
  }

  if (!closed[goalIdx]) return null
  const path: Vec2[] = []
  for (let i = goalIdx; i !== -1; i = parent[i]) path.push(grid.cellCenter(i % grid.cols, Math.floor(i / grid.cols)))
  path.reverse()
  return { path, expanded }
}
