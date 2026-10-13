import { wrapAngle } from '../math/angles'
import type { Pose } from '../math/vec'
import { advancePose } from '../vehicle/bicycle'
import type { KnownMap } from '../world/knownMap'
import { gridDistance } from './gridDistance'
import { MinHeap } from './heap'

export interface HybridParams {
  wheelbase: number
  maxSteer: number
  /** length of each motion primitive (m); must exceed a cell diagonal */
  step: number
  headingBins: number
  /** car footprint: circles of `radius` centered at these offsets along the car axis (from the rear axle) */
  circleOffsets: number[]
  radius: number
  reverseCost: number
  switchCost: number
  steerCost: number
  /** extra cost per meter when closer than `comfort` meters to a wall */
  wallCost: number
  comfort: number
  maxExpansions: number
  goalTolerance: number
  goalHeadingTolerance: number
  /** max tree edges kept for visualization */
  maxTreeEdges: number
  /** >1 trades optimality for speed (weighted A*) */
  heuristicWeight: number
}

export const DEFAULT_HYBRID: HybridParams = {
  wheelbase: 0.5,
  maxSteer: (35 * Math.PI) / 180,
  step: 0.5,
  headingBins: 72,
  circleOffsets: [0, 0.31, 0.62],
  radius: 0.34,
  reverseCost: 2.5,
  switchCost: 2,
  steerCost: 0.1,
  wallCost: 1.5,
  comfort: 0.8,
  maxExpansions: 25_000,
  goalTolerance: 0.35,
  goalHeadingTolerance: (15 * Math.PI) / 180,
  maxTreeEdges: 60_000,
  heuristicWeight: 1.5,
}

export interface PathPose extends Pose {
  /** +1 forward, -1 reverse */
  dir: number
}

export interface HybridResult {
  path: PathPose[]
  /** search tree edges [x1, y1, x2, y2, ...], forward and reverse separately */
  treeForward: Float32Array
  treeReverse: Float32Array
  expansions: number
  millis: number
}

interface Goal extends Pose {
  /** when false, any final heading is accepted */
  useHeading: boolean
}

const SUBSTEP = 0.1

/**
 * Hybrid A* (Dolgov, Thrun, Montemerlo, Diebel — the planner of Stanford's Junior).
 *
 * Unlike grid A*, every node keeps a continuous pose (x, y, θ) and its children are arcs
 * the car can really drive: 5 steering angles, forward and in reverse. Nodes are only
 * *discretized* (x, y, θ bins) to detect states that were already explored.
 *
 * The heuristic is the max of the straight-line distance and a 2D Dijkstra distance on the
 * known map ("holonomic with obstacles"), which steers the search around known walls.
 */
export function hybridAStar(map: KnownMap, start: Pose, goal: Goal, p: HybridParams = DEFAULT_HYBRID): HybridResult | null {
  const t0 = performance.now()
  const { cols, rows } = map
  const bins = p.headingBins
  const binOf = (th: number) => ((Math.floor(((th + Math.PI) / (2 * Math.PI)) * bins) % bins) + bins) % bins

  const h2d = dijkstraFromGoal(map, goal, p.radius)
  const goalCell = map.cellOf(goal.x, goal.y)
  if (goalCell < 0 || !isFinite(h2d[goalCell])) return null

  // near the start the car may already be closer to a wall than the comfort radius
  const startClear = footprintClearance(map, start, p)
  const collides = (x: number, y: number, th: number, nearStart: boolean) => {
    const r = nearStart ? Math.min(p.radius, startClear - 0.02) : p.radius
    for (const o of p.circleOffsets) {
      if (map.clearanceAt(x + Math.cos(th) * o, y + Math.sin(th) * o) < r) return true
    }
    return false
  }
  const heuristic = (x: number, y: number) => {
    const i = map.cellOf(x, y)
    if (i < 0) return Infinity
    const euclid = Math.hypot(goal.x - x, goal.y - y)
    // cells right next to the start may be too tight for the 2D search: fall back to euclidean
    if (!isFinite(h2d[i])) return Math.hypot(x - start.x, y - start.y) < 1 ? euclid * p.heuristicWeight : Infinity
    // the 8-connected grid distance overestimates off-axis directions by up to ~8%, which makes
    // the weighted search zigzag between 0° and 45°; scaled down it only matters around walls
    return Math.max(euclid, h2d[i] * 0.92) * p.heuristicWeight
  }

  // node storage
  const nx: number[] = []
  const ny: number[] = []
  const nth: number[] = []
  const ng: number[] = []
  const nparent: number[] = []
  const ndir: number[] = []
  const nsteer: number[] = []
  const add = (x: number, y: number, th: number, g: number, parent: number, dir: number, steer: number) => {
    nx.push(x)
    ny.push(y)
    nth.push(th)
    ng.push(g)
    nparent.push(parent)
    ndir.push(dir)
    nsteer.push(steer)
    return nx.length - 1
  }

  const { bestG, seenGen, closedGen, gen } = searchBuffers(cols * rows * bins)
  // a state's bestG/closed only count when stamped with this search's generation
  const isClosed = (k: number) => closedGen[k] === gen
  const best = (k: number) => (seenGen[k] === gen ? bestG[k] : Infinity)
  const key = (x: number, y: number, th: number) => {
    const i = map.cellOf(x, y)
    return i < 0 ? -1 : i * bins + binOf(th)
  }

  const steers = [-p.maxSteer, -p.maxSteer / 2, 0, p.maxSteer / 2, p.maxSteer]
  const fwd: number[] = []
  const rev: number[] = []
  const open = new MinHeap()
  open.push(add(start.x, start.y, start.theta, 0, -1, 1, 0), heuristic(start.x, start.y))

  const pose: Pose = { x: 0, y: 0, theta: 0 }
  let expansions = 0
  let found = -1

  while (open.size && expansions < p.maxExpansions) {
    const id = open.pop()!
    const k = key(nx[id], ny[id], nth[id])
    if (k < 0 || isClosed(k)) continue
    closedGen[k] = gen
    expansions++

    const dGoal = Math.hypot(goal.x - nx[id], goal.y - ny[id])
    if (dGoal < p.goalTolerance && (!goal.useHeading || Math.abs(wrapAngle(goal.theta - nth[id])) < p.goalHeadingTolerance)) {
      found = id
      break
    }

    const nearStart = Math.hypot(nx[id] - start.x, ny[id] - start.y) < 1
    for (const dir of [1, -1]) {
      for (const steer of steers) {
        pose.x = nx[id]
        pose.y = ny[id]
        pose.theta = nth[id]
        let ok = true
        let midX = 0
        let midY = 0
        let wallPenalty = 0
        const n = Math.round(p.step / SUBSTEP)
        for (let s = 0; s < n; s++) {
          advancePose(pose, dir, steer, SUBSTEP, p.wheelbase)
          if (collides(pose.x, pose.y, pose.theta, nearStart)) {
            ok = false
            break
          }
          const c = map.clearanceAt(pose.x, pose.y)
          if (c < p.comfort) wallPenalty += (p.comfort - c) * SUBSTEP
          if (s === (n >> 1)) {
            midX = pose.x
            midY = pose.y
          }
        }
        if (!ok) continue
        const k2 = key(pose.x, pose.y, pose.theta)
        if (k2 < 0 || isClosed(k2)) continue

        let cost = p.step * (dir < 0 ? p.reverseCost : 1)
        if (nparent[id] >= 0 && dir !== ndir[id]) cost += p.switchCost
        cost += p.steerCost * (Math.abs(steer) / p.maxSteer) * p.step
        cost += p.steerCost * (Math.abs(steer - nsteer[id]) / p.maxSteer)
        cost += p.wallCost * wallPenalty
        const g2 = ng[id] + cost
        if (g2 >= best(k2)) continue
        const h = heuristic(pose.x, pose.y)
        if (!isFinite(h)) continue
        bestG[k2] = g2
        seenGen[k2] = gen
        open.push(add(pose.x, pose.y, pose.theta, g2, id, dir, steer), g2 + h)

        const edges = dir > 0 ? fwd : rev
        if (fwd.length + rev.length < p.maxTreeEdges * 4) edges.push(nx[id], ny[id], midX, midY, midX, midY, pose.x, pose.y)
      }
    }
  }

  const result = (path: PathPose[]): HybridResult => ({
    path,
    treeForward: new Float32Array(fwd),
    treeReverse: new Float32Array(rev),
    expansions,
    millis: performance.now() - t0,
  })
  if (found < 0) return null

  // rebuild a dense path by re-driving every primitive from the start
  const chain: number[] = []
  for (let i = found; i >= 0; i = nparent[i]) chain.push(i)
  chain.reverse()
  const path: PathPose[] = [{ x: start.x, y: start.y, theta: start.theta, dir: chain.length > 1 ? ndir[chain[1]] : 1 }]
  for (let c = 1; c < chain.length; c++) {
    const id = chain[c]
    const parent = nparent[id]
    pose.x = nx[parent]
    pose.y = ny[parent]
    pose.theta = nth[parent]
    const n = Math.round(p.step / SUBSTEP)
    for (let s = 0; s < n; s++) {
      advancePose(pose, ndir[id], nsteer[id], SUBSTEP, p.wheelbase)
      path.push({ x: pose.x, y: pose.y, theta: pose.theta, dir: ndir[id] })
    }
  }
  return result(path)
}

// Search buffers are reused between plans (the car replans several times per second):
// instead of clearing ~1M entries every time, entries are stamped with a generation number.
let buffers: { size: number; bestG: Float32Array; seenGen: Uint32Array; closedGen: Uint32Array; gen: number } | null = null

function searchBuffers(size: number) {
  if (!buffers || buffers.size !== size || buffers.gen >= 0xfffffffe) {
    buffers = { size, bestG: new Float32Array(size), seenGen: new Uint32Array(size), closedGen: new Uint32Array(size), gen: 0 }
  }
  buffers.gen++
  return buffers
}

/** Smallest clearance among the footprint circles. */
export function footprintClearance(map: KnownMap, pose: Pose, p: HybridParams = DEFAULT_HYBRID): number {
  let min = Infinity
  for (const o of p.circleOffsets) {
    min = Math.min(min, map.clearanceAt(pose.x + Math.cos(pose.theta) * o, pose.y + Math.sin(pose.theta) * o))
  }
  return min
}

/** True when the car footprint at `pose` keeps at least `radius` from every known wall. */
export function poseIsFree(map: KnownMap, pose: Pose, p: HybridParams = DEFAULT_HYBRID): boolean {
  return footprintClearance(map, pose, p) >= p.radius
}

/** 2D shortest distance (m) from every cell to the goal, through cells with enough clearance. */
function dijkstraFromGoal(map: KnownMap, goal: Pose, radius: number): Float32Array {
  const d = new Float32Array(map.cols * map.rows).fill(Infinity)
  const g = map.cellOf(goal.x, goal.y)
  if (g < 0) return d
  const minDist = radius * 0.9 + map.cellSize * 0.5
  return gridDistance(map.cols, map.rows, [g], d, map.cellSize, (i) => map.dist[i] >= minDist)
}
