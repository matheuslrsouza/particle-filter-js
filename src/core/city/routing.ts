import type { Pose, Vec2 } from '../math/vec'
import { MinHeap } from '../planning/heap'
import type { PathPose } from '../planning/hybridAStar'
import { polylineLength, type LaneGraph } from './roadNetwork'

/** A position on a lane: lane id and distance along it. */
export interface LanePosition {
  lane: number
  s: number
  point: Vec2
}

export type RouteStep =
  | { kind: 'lane'; lane: number; from: number; to: number }
  | { kind: 'connector'; connector: number }
  /** lane change starting at distance `at` along `from`, over `span` meters */
  | { kind: 'change'; from: number; to: number; at: number; span: number }

export interface Route {
  steps: RouteStep[]
  /** dense trajectory (every ~0.5 m) with heading, a speed limit and the street name per point */
  path: PathPose[]
  speedLimit: number[]
  names: string[]
  length: number
}

export interface RoutingParams {
  /** extra time (s) for each lane change */
  laneChangeCost: number
  /** relative extra time for driving in any lane but the rightmost */
  keepRightCost: number
}

export const DEFAULT_ROUTING: RoutingParams = { laneChangeCost: 6, keepRightCost: 0.15 }

/** Closest point of a polyline to `p`: [distance along it, distance to it]. */
function project(pts: Vec2[], p: Vec2): [number, number, Vec2] {
  let best = Infinity
  let bestS = 0
  let bestPt = pts[0]
  let s = 0
  for (let i = 1; i < pts.length; i++) {
    const a = pts[i - 1]
    const b = pts[i]
    const dx = b.x - a.x
    const dy = b.y - a.y
    const len2 = dx * dx + dy * dy
    const t = len2 > 0 ? Math.max(0, Math.min(1, ((p.x - a.x) * dx + (p.y - a.y) * dy) / len2)) : 0
    const q = { x: a.x + dx * t, y: a.y + dy * t }
    const d = Math.hypot(p.x - q.x, p.y - q.y)
    if (d < best) {
      best = d
      bestS = s + Math.sqrt(len2) * t
      bestPt = q
    }
    s += Math.sqrt(len2)
  }
  return [bestS, best, bestPt]
}

/**
 * Nearest lane position to a point. With a heading, only lanes going roughly the same way are
 * considered (the car must not snap onto the opposite direction).
 */
export function snapToLane(g: LaneGraph, p: Vec2, heading?: number): LanePosition | null {
  let best: LanePosition | null = null
  let bestD = Infinity
  for (const l of g.lanes) {
    if (l.length < 1) continue
    const [s, d, point] = project(l.points, p)
    if (d >= bestD) continue
    if (heading !== undefined) {
      const q = pointAt(l.points, Math.min(l.length - 0.1, s + 0.1))
      const h = Math.atan2(q.y - point.y, q.x - point.x)
      if (Math.cos(h - heading) < 0.5) continue
    }
    bestD = d
    best = { lane: l.id, s, point }
  }
  return best
}

/** Point at a distance along a polyline. */
export function pointAt(pts: Vec2[], s: number): Vec2 {
  let acc = 0
  for (let i = 1; i < pts.length; i++) {
    const a = pts[i - 1]
    const b = pts[i]
    const len = Math.hypot(b.x - a.x, b.y - a.y)
    if (acc + len >= s && len > 0) {
      const t = (s - acc) / len
      return { x: a.x + (b.x - a.x) * t, y: a.y + (b.y - a.y) * t }
    }
    acc += len
  }
  return { ...pts[pts.length - 1] }
}

/** Part of a polyline between two distances. */
function subPolyline(pts: Vec2[], from: number, to: number): Vec2[] {
  const out: Vec2[] = [pointAt(pts, from)]
  let acc = 0
  for (let i = 1; i < pts.length; i++) {
    const len = Math.hypot(pts[i].x - pts[i - 1].x, pts[i].y - pts[i - 1].y)
    acc += len
    if (acc > from && acc < to) out.push({ ...pts[i] })
    if (acc >= to) break
  }
  out.push(pointAt(pts, to))
  return out
}

/**
 * Fastest route on the lane graph (A*, cost = travel time at the speed limits). Lane changes
 * cost a few seconds and lanes other than the rightmost are slightly penalized, so the car keeps
 * right unless it needs the left lane to turn left.
 */
export function planRoute(g: LaneGraph, start: LanePosition, goal: LanePosition, p: RoutingParams = DEFAULT_ROUTING): Route | null {
  const maxSpeed = Math.max(...g.lanes.map((l) => l.speed))
  const keepRight = (lane: number) => (g.lanes[lane].right >= 0 ? 1 + p.keepRightCost : 1)
  const h = (lane: number) => {
    const end = g.lanes[lane].points[g.lanes[lane].points.length - 1]
    return Math.hypot(end.x - goal.point.x, end.y - goal.point.y) / maxSpeed
  }
  // same lane, goal ahead: no search needed
  if (start.lane === goal.lane && goal.s >= start.s) return buildRoute(g, [{ kind: 'lane', lane: start.lane, from: start.s, to: goal.s }])

  // nodes are lanes; reaching a lane means being at its start (or at `start.s` for the first one)
  const n = g.lanes.length
  const cost = new Float64Array(n).fill(Infinity)
  const prev: (RouteStep[] | null)[] = new Array(n).fill(null)
  const prevLane = new Int32Array(n).fill(-1)
  const open = new MinHeap()
  const entryS = new Float64Array(n)
  cost[start.lane] = 0
  entryS[start.lane] = start.s
  open.push(start.lane, h(start.lane))
  const closed = new Uint8Array(n)
  while (open.size) {
    const lane = open.pop()!
    if (closed[lane]) continue
    closed[lane] = 1
    const L = g.lanes[lane]
    if (lane === goal.lane && goal.s >= entryS[lane] - 1e-6 && lane !== start.lane) break
    const remaining = Math.max(0, L.length - entryS[lane])
    const atEnd = cost[lane] + (remaining / L.speed) * keepRight(lane)
    const relax = (to: number, c: number, steps: RouteStep[], s0: number) => {
      if (c < cost[to]) {
        cost[to] = c
        prev[to] = steps
        prevLane[to] = lane
        entryS[to] = s0
        open.push(to, c + h(to))
      }
    }
    for (const cid of L.next) {
      const c = g.connectors[cid]
      relax(
        c.to,
        atEnd + c.length / Math.min(L.speed, g.lanes[c.to].speed),
        [{ kind: 'lane', lane, from: entryS[lane], to: L.length }, { kind: 'connector', connector: cid }],
        0,
      )
    }
    // lane change, starting where the car enters this lane
    for (const side of [L.left, L.right]) {
      if (side < 0) continue
      const at = entryS[lane]
      const span = changeSpan(g.lanes[side].length, at)
      if (span < 5) continue
      relax(side, cost[lane] + p.laneChangeCost + span / L.speed, [{ kind: 'change', from: lane, to: side, at, span }], at + span)
    }
  }
  if (!Number.isFinite(cost[goal.lane]) || goal.s < entryS[goal.lane] - 1e-6) return null

  // walk back from the goal (the start lane is never re-entered, so this ends there)
  const steps: RouteStep[] = [{ kind: 'lane', lane: goal.lane, from: entryS[goal.lane], to: goal.s }]
  for (let lane = goal.lane; lane !== start.lane; lane = prevLane[lane]) steps.unshift(...prev[lane]!)
  return buildRoute(g, steps)
}

/** Length of a lane change onto a lane of length `len`, starting at `at`: ~25 m when there is room. */
const changeSpan = (len: number, at: number) => Math.min(25, len - at)

/** Turns route steps into a dense drivable trajectory. */
function buildRoute(g: LaneGraph, steps: RouteStep[]): Route {
  const pts: Vec2[] = []
  const limits: number[] = []
  const names: string[] = []
  const push = (list: Vec2[], limit: number, name: string) => {
    for (const p of list) {
      const last = pts[pts.length - 1]
      if (last && Math.hypot(p.x - last.x, p.y - last.y) < 0.05) continue
      pts.push(p)
      limits.push(limit)
      names.push(name)
    }
  }
  for (let i = 0; i < steps.length; i++) {
    const st = steps[i]
    if (st.kind === 'lane') {
      const L = g.lanes[st.lane]
      if (st.to - st.from > 0.05) push(subPolyline(L.points, st.from, st.to), L.speed, L.name)
    } else if (st.kind === 'connector') {
      const c = g.connectors[st.connector]
      push(c.points, Math.min(g.lanes[c.from].speed, g.lanes[c.to].speed), g.lanes[c.to].name)
    } else {
      // lane change: blend from the current lane to the neighbor (smoothstep), the lanes are parallel
      const A = g.lanes[st.from]
      const B = g.lanes[st.to]
      const blend: Vec2[] = []
      for (let d = 0; d <= st.span + 1e-9; d += Math.min(1, st.span / 5)) {
        const t = d / st.span
        const k = t * t * (3 - 2 * t)
        const a = pointAt(A.points, Math.min(A.length, st.at + d))
        const b = pointAt(B.points, Math.min(B.length, st.at + d))
        blend.push({ x: a.x + (b.x - a.x) * k, y: a.y + (b.y - a.y) * k })
      }
      push(blend, Math.min(A.speed, B.speed), A.name)
    }
  }
  return densify(pts, limits, names)
}

function densify(pts: Vec2[], limits: number[], names: string[]): Route {
  const path: PathPose[] = []
  const speedLimit: number[] = []
  const outNames: string[] = []
  for (let i = 1; i < pts.length; i++) {
    const a = pts[i - 1]
    const b = pts[i]
    const len = Math.hypot(b.x - a.x, b.y - a.y)
    const n = Math.max(1, Math.ceil(len / 0.5))
    for (let k = i === 1 ? 0 : 1; k <= n; k++) {
      const t = k / n
      path.push({ x: a.x + (b.x - a.x) * t, y: a.y + (b.y - a.y) * t, theta: 0, dir: 1 })
      speedLimit.push(limits[i])
      outNames.push(names[i])
    }
  }
  // heading from the tangent
  for (let i = 0; i < path.length; i++) {
    const a = path[Math.max(0, i - 1)]
    const b = path[Math.min(path.length - 1, i + 1)]
    path[i].theta = Math.atan2(b.y - a.y, b.x - a.x)
  }
  return { steps: [], path, speedLimit, names: outNames, length: polylineLength(path) }
}

/** The lane position of a pose (snaps onto a lane going the same way). */
export function locate(g: LaneGraph, pose: Pose): LanePosition | null {
  return snapToLane(g, pose, pose.theta)
}
