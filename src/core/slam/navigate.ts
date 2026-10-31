import { wrapAngle } from '../math/angles'
import type { Pose, Vec2 } from '../math/vec'
import { MinHeap } from '../planning/heap'
import type { OccupancyGrid } from './hector'

/**
 * Going to a point of the SLAM map: A* on the map (walls inflated by the robot's radius, unknown
 * space allowed but more expensive), the cell staircase cut by line of sight and smoothed (the
 * course's gradient smoothing, kept clear of walls), and a pure-pursuit follower for a
 * differential-drive robot that compensates the loop's latency.
 */

export interface NavParams {
  /** robot radius plus a margin (m): cells closer than this to a wall are not entered */
  inflate: number
  robotRadius: number
  /** extra cost per step in unknown cells */
  unknownCost: number
  /** extra cost per step near walls, fading out at `inflate + comfort` */
  comfort: number
  lookahead: number
  /** seconds between a command and its effect showing in the map (SLAM rate + Wi-Fi + firmware) */
  latency: number
  /** goal reached within this distance */
  arrive: number
  /** forward speed (m/s) and turn rate limit (rad/s) */
  vMax: number
  wMax: number
  /** heading error (rad) above which the robot turns in place, and below which it drives again */
  turnInPlace: number
  turnDone: number
  kHeading: number
  /** gradient smoothing weights (pull toward the cut path / toward the neighbors) */
  weightData: number
  weightSmooth: number
}

export const DEFAULT_NAV: NavParams = {
  inflate: 0.24,
  robotRadius: 0.175,
  unknownCost: 3,
  comfort: 0.15,
  lookahead: 0.45,
  latency: 0.3,
  arrive: 0.12,
  vMax: 0.12,
  wMax: 0.6,
  turnInPlace: 1.2,
  turnDone: 0.35,
  kHeading: 1.2,
  weightData: 0.1,
  weightSmooth: 0.35,
}

/** Cell kinds on the planning grid. */
const FREE = 0
const UNKNOWN = 1
const BLOCKED = 2

/**
 * A* from `start` to `goal` (world coordinates) on a SLAM grid. Returns the path as points every
 * cell, or null when the goal can't be reached. The start may be inside the inflated band (the
 * robot can be close to a wall); the walls themselves are never crossed.
 */
/** A* path (null if unreachable). */
export function planOnMap(map: OccupancyGrid, start: Vec2, goal: Vec2, p: NavParams = DEFAULT_NAV, visited: Vec2[] = []): Vec2[] | null {
  return planRoute(map, start, goal, p, visited)?.path ?? null
}

export interface Route {
  path: Vec2[]
  /** whether the robot's body (radius + 2 cm) can move along a straight segment without touching a wall */
  bodyClear: (a: Vec2, b: Vec2) => boolean
}

/** A* path plus a body-clearance check on the same map, for the follower. */
export function planRoute(map: OccupancyGrid, start: Vec2, goal: Vec2, p: NavParams = DEFAULT_NAV, visited: Vec2[] = []): Route | null {
  const n = map.cells
  const res = map.resolution
  const kind = new Uint8Array(n * n)
  // distance (cells) to the nearest wall, by multi-source BFS (8-connected, chessboard metric)
  const dist = new Float32Array(n * n).fill(Infinity)
  const queue: number[] = []
  // where the robot has been there is no wall, whatever the map says (echoes of things carried on
  // the robot, people walking by): cells under its body along its path are free
  const been = new Uint8Array(n * n)
  const body = Math.ceil(0.15 / res)
  for (const v of visited) {
    const cx = Math.floor((v.x - map.origin) / res)
    const cy = Math.floor((v.y - map.origin) / res)
    for (let dy = -body; dy <= body; dy++) {
      for (let dx = -body; dx <= body; dx++) {
        const x = cx + dx
        const y = cy + dy
        if (x >= 0 && y >= 0 && x < n && y < n && dx * dx + dy * dy <= body * body) been[y * n + x] = 1
      }
    }
  }
  for (let i = 0; i < n * n; i++) {
    const l = map.logOdds[i]
    if (been[i]) {
      kind[i] = FREE
      continue
    }
    if (l > 0.5) {
      dist[i] = 0
      queue.push(i)
    }
    kind[i] = l < -0.2 ? FREE : UNKNOWN
  }
  const reach = Math.ceil((p.inflate + p.comfort) / res)
  for (let head = 0; head < queue.length; head++) {
    const i = queue[head]
    const d = dist[i] + 1
    if (d > reach) continue
    const x = i % n
    const y = (i / n) | 0
    for (let dy = -1; dy <= 1; dy++) {
      for (let dx = -1; dx <= 1; dx++) {
        const xx = x + dx
        const yy = y + dy
        if (xx < 0 || yy < 0 || xx >= n || yy >= n) continue
        const j = yy * n + xx
        if (dist[j] > d) {
          dist[j] = d
          queue.push(j)
        }
      }
    }
  }
  const inflateCells = p.inflate / res
  for (let i = 0; i < n * n; i++) if (dist[i] <= inflateCells && !been[i]) kind[i] = BLOCKED

  const cell = (v: Vec2): [number, number] => [Math.floor((v.x - map.origin) / res), Math.floor((v.y - map.origin) / res)]
  const [sx, sy] = cell(start)
  const [gx, gy] = cell(goal)
  if (sx < 0 || sy < 0 || gx < 0 || gy < 0 || sx >= n || sy >= n || gx >= n || gy >= n) return null
  const s = sy * n + sx
  const g = gy * n + gx
  if (kind[g] === BLOCKED) return null

  const cost = new Float32Array(n * n).fill(Infinity)
  const parent = new Int32Array(n * n).fill(-1)
  const closed = new Uint8Array(n * n)
  const open = new MinHeap()
  const h = (i: number) => {
    const dx = Math.abs((i % n) - gx)
    const dy = Math.abs(((i / n) | 0) - gy)
    return dx + dy + (Math.SQRT2 - 2) * Math.min(dx, dy)
  }
  cost[s] = 0
  open.push(s, h(s))
  while (open.size) {
    const i = open.pop()!
    if (closed[i]) continue
    closed[i] = 1
    if (i === g) break
    const x = i % n
    const y = (i / n) | 0
    // leaving the start's inflated band is allowed, entering any other inflated cell is not
    const startBand = kind[i] === BLOCKED && dist[i] > 0
    for (let dy = -1; dy <= 1; dy++) {
      for (let dx = -1; dx <= 1; dx++) {
        if (!dx && !dy) continue
        const xx = x + dx
        const yy = y + dy
        if (xx < 0 || yy < 0 || xx >= n || yy >= n) continue
        const j = yy * n + xx
        if (closed[j] || dist[j] === 0) continue
        if (kind[j] === BLOCKED && !(startBand && dist[j] >= dist[i])) continue
        let step = dx && dy ? Math.SQRT2 : 1
        if (kind[j] === UNKNOWN) step *= p.unknownCost
        // prefer the middle of free space
        if (dist[j] < reach) step += (reach - dist[j]) / reach
        const c = cost[i] + step
        if (c < cost[j]) {
          cost[j] = c
          parent[j] = i
          open.push(j, c + h(j))
        }
      }
    }
  }
  if (parent[g] < 0 && g !== s) return null
  const cells: Vec2[] = []
  for (let i = g; i >= 0; i = parent[i]) {
    cells.push({ x: map.origin + ((i % n) + 0.5) * res, y: map.origin + (((i / n) | 0) + 0.5) * res })
    if (i === s) break
  }
  cells.reverse()

  // a point is clear when it's not in the inflated band, except near the start (the robot may be
  // starting close to a wall) and where the robot has been
  const clear = (v: Vec2) => {
    const x = Math.floor((v.x - map.origin) / res)
    const y = Math.floor((v.y - map.origin) / res)
    if (x < 0 || y < 0 || x >= n || y >= n) return false
    const i = y * n + x
    if (dist[i] === 0) return false
    return kind[i] !== BLOCKED || been[i] === 1 || Math.hypot(v.x - start.x, v.y - start.y) < p.inflate
  }
  const segmentClear = (a: Vec2, b: Vec2) => {
    const steps = Math.max(1, Math.ceil(Math.hypot(b.x - a.x, b.y - a.y) / (res * 0.5)))
    for (let k = 0; k <= steps; k++) if (!clear({ x: a.x + ((b.x - a.x) * k) / steps, y: a.y + ((b.y - a.y) * k) / steps })) return false
    return true
  }
  // the body itself: walls at least the robot's radius (+2 cm) away; the inflated band beyond
  // the body is only a planning margin
  const bodyCells = (p.robotRadius + 0.02) / res
  const bodyClear = (a: Vec2, b: Vec2) => {
    const steps = Math.max(1, Math.ceil(Math.hypot(b.x - a.x, b.y - a.y) / (res * 0.5)))
    for (let k = 0; k <= steps; k++) {
      const x = Math.floor((a.x + ((b.x - a.x) * k) / steps - map.origin) / res)
      const y = Math.floor((a.y + ((b.y - a.y) * k) / steps - map.origin) / res)
      if (x < 0 || y < 0 || x >= n || y >= n) return false
      const i = y * n + x
      // the first centimeters from where the robot is are skipped: if it's already close to a wall,
      // it must still be able to move away from it
      if (dist[i] < bodyCells && !been[i] && (k * Math.hypot(b.x - a.x, b.y - a.y)) / steps > 0.12) return false
    }
    return true
  }
  return { path: smoothPath(shortcut(cells, segmentClear), clear, p), bodyClear }
}

/** Line of sight: from each kept point, jump to the farthest point it can reach in a straight line. */
function shortcut(path: Vec2[], segmentClear: (a: Vec2, b: Vec2) => boolean): Vec2[] {
  if (path.length < 3) return path
  const out = [path[0]]
  let i = 0
  while (i < path.length - 1) {
    let j = path.length - 1
    while (j > i + 1 && !segmentClear(path[i], path[j])) j--
    out.push(path[j])
    i = j
  }
  return out
}

/**
 * Resampled every 5 cm, then the gradient smoothing of the course (endpoints fixed); points that
 * would end up in the inflated band are pulled back toward the straight path.
 */
function smoothPath(corners: Vec2[], clear: (v: Vec2) => boolean, p: NavParams): Vec2[] {
  const pts: Vec2[] = []
  for (let k = 0; k < corners.length - 1; k++) {
    const a = corners[k]
    const b = corners[k + 1]
    const steps = Math.max(1, Math.ceil(Math.hypot(b.x - a.x, b.y - a.y) / 0.05))
    for (let t = 0; t < steps; t++) pts.push({ x: a.x + ((b.x - a.x) * t) / steps, y: a.y + ((b.y - a.y) * t) / steps })
  }
  pts.push(corners[corners.length - 1])
  if (pts.length < 3) return pts
  const out = pts.map((q) => ({ ...q }))
  for (let iter = 0; iter < 500; iter++) {
    let change = 0
    for (let i = 1; i < out.length - 1; i++) {
      const old = { ...out[i] }
      for (const k of ['x', 'y'] as const) {
        out[i][k] += p.weightData * (pts[i][k] - out[i][k]) + p.weightSmooth * (out[i - 1][k] + out[i + 1][k] - 2 * out[i][k])
      }
      if (!clear(out[i])) out[i] = old
      change += Math.abs(old.x - out[i].x) + Math.abs(old.y - out[i].y)
    }
    if (change < 1e-4) break
  }
  return out
}

export interface Command {
  /** m/s forward, rad/s counter-clockwise */
  v: number
  w: number
  done: boolean
  /** index of the path point nearest to the robot */
  target: number
}

/**
 * Pure pursuit for a robot that can turn in place, on a pose predicted `latency` seconds ahead
 * with the last command (the map lags the robot: steering on the stale pose overshoots and
 * zigzags). Turns in place above `turnInPlace` of heading error until under `turnDone`.
 */
export class PathFollower {
  private last = { v: 0, w: 0 }
  private turning = false
  private near = 0

  constructor(
    public path: Vec2[],
    public p: NavParams = DEFAULT_NAV,
    /** straight-line clearance of the robot's body: aim only at points it can drive straight to */
    public bodyClear: ((a: Vec2, b: Vec2) => boolean) | null = null,
  ) {}

  setPath(path: Vec2[], bodyClear: ((a: Vec2, b: Vec2) => boolean) | null = this.bodyClear): void {
    this.path = path
    this.bodyClear = bodyClear
    this.near = 0
  }

  step(pose: Pose): Command {
    const { path, p } = this
    const last = path[path.length - 1]
    if (!last || Math.hypot(last.x - pose.x, last.y - pose.y) < p.arrive) {
      this.last = { v: 0, w: 0 }
      return { v: 0, w: 0, done: true, target: path.length - 1 }
    }
    // where the robot will be when this command takes effect
    const L = p.latency
    const th = pose.theta + this.last.w * L
    const pred = {
      x: pose.x + Math.cos(pose.theta + (this.last.w * L) / 2) * this.last.v * L,
      y: pose.y + Math.sin(pose.theta + (this.last.w * L) / 2) * this.last.v * L,
      theta: th,
    }
    const d = (i: number) => Math.hypot(path[i].x - pred.x, path[i].y - pred.y)
    for (let i = this.near; i < Math.min(path.length, this.near + 60); i++) if (d(i) < d(this.near)) this.near = i
    const toGoal = Math.hypot(last.x - pred.x, last.y - pred.y)
    const look = Math.max(0.15, Math.min(p.lookahead, toGoal))
    let target = this.near
    while (target < path.length - 1 && d(target) < look) target++
    // pure pursuit cuts corners by up to the look-ahead: near a corner, aim at a closer point that
    // the body can reach in a straight line
    if (this.bodyClear) while (target > this.near + 1 && !this.bodyClear(pred, path[target])) target--
    const t = path[target]
    const alpha = wrapAngle(Math.atan2(t.y - pred.y, t.x - pred.x) - pred.theta)
    // hysteresis between turning in place and driving
    if (this.turning ? Math.abs(alpha) > p.turnDone : Math.abs(alpha) > p.turnInPlace) {
      this.turning = true
      const w = Math.sign(alpha) * Math.min(p.wMax, p.kHeading * Math.abs(alpha) + 0.15)
      this.last = { v: 0, w }
      return { v: 0, w, done: false, target: this.near }
    }
    this.turning = false
    // pure pursuit: the arc through the target point has curvature 2·sin(α)/distance
    const dist = Math.max(d(target), 1e-3)
    const kappa = (2 * Math.sin(alpha)) / dist
    let v = p.vMax * Math.min(1, toGoal / 0.3 + 0.35)
    // slow down so the turn rate stays within its limit
    if (Math.abs(v * kappa) > p.wMax) v = p.wMax / Math.abs(kappa)
    const w = v * kappa
    this.last = { v, w }
    return { v, w, done: false, target: this.near }
  }
}

/** One step of a fresh follower (kept for simple callers). */
export function follow(path: Vec2[], pose: Pose, p: NavParams = DEFAULT_NAV): Command {
  return new PathFollower(path, p).step(pose)
}
