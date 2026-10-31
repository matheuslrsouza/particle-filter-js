import { wrapAngle } from '../math/angles'
import type { Pose, Vec2 } from '../math/vec'
import { MinHeap } from '../planning/heap'
import type { OccupancyGrid } from './hector'

/**
 * Going to a point of the SLAM map: A* on the map (walls inflated by the robot's radius, unknown
 * space allowed but more expensive) and a follower for a differential-drive robot (turn in place
 * when facing away, otherwise drive toward a look-ahead point while steering).
 */

export interface NavParams {
  /** robot radius plus a margin (m): cells closer than this to a wall are not entered */
  inflate: number
  /** extra cost per step in unknown cells */
  unknownCost: number
  /** extra cost per step near walls, fading out at `inflate + comfort` */
  comfort: number
  lookahead: number
  /** goal reached within this distance */
  arrive: number
  /** forward speed (m/s) and turn rate limit (rad/s) */
  vMax: number
  wMax: number
  /** heading error (rad) above which the robot turns in place */
  turnInPlace: number
  kHeading: number
}

export const DEFAULT_NAV: NavParams = {
  inflate: 0.24,
  unknownCost: 3,
  comfort: 0.15,
  lookahead: 0.3,
  arrive: 0.12,
  vMax: 0.12,
  wMax: 0.6,
  turnInPlace: 0.7,
  kHeading: 1.5,
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
export function planOnMap(map: OccupancyGrid, start: Vec2, goal: Vec2, p: NavParams = DEFAULT_NAV): Vec2[] | null {
  const n = map.cells
  const res = map.resolution
  const kind = new Uint8Array(n * n)
  // distance (cells) to the nearest wall, by multi-source BFS (8-connected, chessboard metric)
  const dist = new Float32Array(n * n).fill(Infinity)
  const queue: number[] = []
  for (let i = 0; i < n * n; i++) {
    const l = map.logOdds[i]
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
  for (let i = 0; i < n * n; i++) if (dist[i] <= inflateCells) kind[i] = BLOCKED

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
  const path: Vec2[] = []
  for (let i = g; i >= 0; i = parent[i]) {
    path.push({ x: map.origin + ((i % n) + 0.5) * res, y: map.origin + (((i / n) | 0) + 0.5) * res })
    if (i === s) break
  }
  return path.reverse()
}

export interface Command {
  /** m/s forward, rad/s counter-clockwise */
  v: number
  w: number
  done: boolean
  /** index of the path point being steered at */
  target: number
}

/** Look-ahead follower for a robot that can turn in place. */
export function follow(path: Vec2[], pose: Pose, p: NavParams = DEFAULT_NAV, from = 0): Command {
  const last = path[path.length - 1]
  if (!last || Math.hypot(last.x - pose.x, last.y - pose.y) < p.arrive) return { v: 0, w: 0, done: true, target: path.length - 1 }
  // the nearest path point from where we were, then the first one beyond the look-ahead distance
  let near = from
  for (let i = from; i < Math.min(path.length, from + 40); i++) {
    if (Math.hypot(path[i].x - pose.x, path[i].y - pose.y) < Math.hypot(path[near].x - pose.x, path[near].y - pose.y)) near = i
  }
  let target = near
  while (target < path.length - 1 && Math.hypot(path[target].x - pose.x, path[target].y - pose.y) < p.lookahead) target++
  const t = path[target]
  const err = wrapAngle(Math.atan2(t.y - pose.y, t.x - pose.x) - pose.theta)
  const w = Math.max(-p.wMax, Math.min(p.wMax, p.kHeading * err))
  if (Math.abs(err) > p.turnInPlace) return { v: 0, w, done: false, target: near }
  // slow down when the heading is off and near the goal
  const toGoal = Math.hypot(last.x - pose.x, last.y - pose.y)
  const v = p.vMax * Math.cos(err) * Math.min(1, toGoal / 0.3 + 0.3)
  return { v, w, done: false, target: near }
}
