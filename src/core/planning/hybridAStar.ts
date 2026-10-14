import { wrapAngle } from '../math/angles'
import type { Pose, Vec2 } from '../math/vec'
import { advancePose } from '../vehicle/bicycle'
import type { KnownMap } from '../world/knownMap'
import { gridDistance } from './gridDistance'
import { reedsSheppPaths, sampleReedsShepp } from './reedsShepp'
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
  /** try to reach the goal pose exactly with a Reeds-Shepp curve (Dolgov's analytic expansion) */
  analyticExpansion: boolean
  /**
   * optional points of the car body (car frame: x forward from the rear axle, y to the right),
   * e.g. its corners, that must keep `cornerClearance` from walls; circles alone can't cover a
   * rectangle's corners without overshooting its front and back
   */
  corners?: Vec2[]
  cornerClearance?: number
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
  analyticExpansion: true,
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
  /** the Reeds-Shepp shot that closed the search, if any [x1, y1, x2, y2, ...] */
  analytic: Float32Array
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
  const cornerClear = (x: number, y: number, th: number) => {
    let min = Infinity
    const c = Math.cos(th)
    const s = Math.sin(th)
    for (const k of p.corners ?? []) min = Math.min(min, map.clearanceAt(x + c * k.x - s * k.y, y + s * k.x + c * k.y))
    return min
  }
  const startCorners = cornerClear(start.x, start.y, start.theta)
  const collides = (x: number, y: number, th: number, nearStart: boolean) => {
    // near the start the car may be closer than the safety radius, but never get any closer
    const r = nearStart ? Math.min(p.radius, startClear) : p.radius
    for (const o of p.circleOffsets) {
      if (map.clearanceAt(x + Math.cos(th) * o, y + Math.sin(th) * o) < r) return true
    }
    if (p.corners) {
      const need = nearStart ? Math.min(p.cornerClearance ?? 0, startCorners) : (p.cornerClearance ?? 0)
      if (cornerClear(x, y, th) < need) return true
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
  let analytic: PathPose[] | null = null
  const turningRadius = p.wheelbase / Math.tan(p.maxSteer)
  /** Shortest (cost-weighted) collision-free Reeds-Shepp curve from a node to the goal pose. */
  const reedsSheppShot = (x: number, y: number, th: number): PathPose[] | null => {
    // a goal without heading is approached facing it
    const target = goal.useHeading ? goal : { x: goal.x, y: goal.y, theta: Math.atan2(goal.y - y, goal.x - x) }
    const weighted = (segs: { length: number }[]) => {
      let c = 0
      for (let i = 0; i < segs.length; i++) {
        c += Math.abs(segs[i].length) * turningRadius * (segs[i].length < 0 ? p.reverseCost : 1)
        if (i > 0 && Math.sign(segs[i].length) !== Math.sign(segs[i - 1].length)) c += p.switchCost
      }
      return c
    }
    const candidates = reedsSheppPaths({ x, y, theta: th }, target, turningRadius)
      .map((path) => ({ path, cost: weighted(path.segments) }))
      .sort((a, b) => a.cost - b.cost)
      .slice(0, 2)
    for (const { path } of candidates) {
      const samples = sampleReedsShepp({ x, y, theta: th }, path, turningRadius, SUBSTEP)
      if (samples.every((q) => !collides(q.x, q.y, q.theta, false))) return samples
    }
    return null
  }

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

    // analytic expansion: more often as the node gets closer to the goal
    if (p.analyticExpansion && dGoal < 15) {
      const interval = Math.max(1, Math.min(10, Math.floor(dGoal / 1.5)))
      if (expansions % interval === 0) {
        const shot = reedsSheppShot(nx[id], ny[id], nth[id])
        if (shot) {
          found = id
          analytic = shot
          break
        }
      }
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
    analytic: shotEdges(found >= 0 ? { x: nx[found], y: ny[found] } : null, analytic),
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
  if (analytic) {
    // the first move of a path that is only a shot carries the shot's gear
    if (path.length === 1) path[0].dir = analytic[0].dir
    path.push(...analytic)
  }
  return result(path)
}

function shotEdges(from: { x: number; y: number } | null, shot: PathPose[] | null): Float32Array {
  if (!from || !shot) return new Float32Array(0)
  const out: number[] = []
  let prev = from
  for (const q of shot) {
    out.push(prev.x, prev.y, q.x, q.y)
    prev = q
  }
  return new Float32Array(out)
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
