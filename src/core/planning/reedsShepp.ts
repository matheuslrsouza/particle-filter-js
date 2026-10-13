import type { Pose } from '../math/vec'

/**
 * Reeds-Shepp curves: the shortest paths between two poses for a car with a minimum turning
 * radius that can drive forward and in reverse (Reeds & Shepp, 1990). They are made of arcs at
 * full lock (L, R) and straight lines (S). Hybrid A* uses them for its "analytic expansion":
 * from a node, try to reach the goal pose exactly with one of these curves.
 *
 * Each word below is computed in normalized coordinates (radius 1). Segment lengths are
 * signed: negative means reverse. Words are combined with time-flip and reflection symmetries,
 * and every candidate is integrated and checked against the target before being accepted.
 */

export type SegmentType = 'L' | 'S' | 'R'

export interface RsSegment {
  type: SegmentType
  /** signed length in normalized units (radians for arcs); negative = reverse */
  length: number
}

export interface RsPath {
  segments: RsSegment[]
  /** total length in meters */
  length: number
}

const PI = Math.PI
const HALF_PI = PI / 2

function mod2pi(x: number): number {
  let v = x % (2 * PI)
  if (v < -PI) v += 2 * PI
  else if (v > PI) v -= 2 * PI
  return v
}

const polar = (x: number, y: number): [number, number] => [Math.hypot(x, y), Math.atan2(y, x)]

type Word = (x: number, y: number, phi: number) => [number[], SegmentType[]] | null

// --- CSC -----------------------------------------------------------------------------------
const LSL: Word = (x, y, phi) => {
  const [u, t] = polar(x - Math.sin(phi), y - 1 + Math.cos(phi))
  if (t >= 0 && t <= PI) {
    const v = mod2pi(phi - t)
    if (v >= 0 && v <= PI) return [[t, u, v], ['L', 'S', 'L']]
  }
  return null
}

const LSR: Word = (x, y, phi) => {
  const [u1, t1] = polar(x + Math.sin(phi), y - 1 - Math.cos(phi))
  const u1sq = u1 * u1
  if (u1sq >= 4) {
    const u = Math.sqrt(u1sq - 4)
    const theta = Math.atan2(2, u)
    const t = mod2pi(t1 + theta)
    const v = mod2pi(t - phi)
    if (t >= 0 && v >= 0) return [[t, u, v], ['L', 'S', 'R']]
  }
  return null
}

// --- CCC -----------------------------------------------------------------------------------
const LxRxL: Word = (x, y, phi) => {
  const [u1, theta] = polar(x - Math.sin(phi), y - 1 + Math.cos(phi))
  if (u1 <= 4) {
    const A = Math.acos(0.25 * u1)
    const t = mod2pi(A + theta + HALF_PI)
    const u = mod2pi(PI - 2 * A)
    const v = mod2pi(phi - t - u)
    return [[t, -u, v], ['L', 'R', 'L']]
  }
  return null
}

const LxRL: Word = (x, y, phi) => {
  const [u1, theta] = polar(x - Math.sin(phi), y - 1 + Math.cos(phi))
  if (u1 <= 4) {
    const A = Math.acos(0.25 * u1)
    const t = mod2pi(A + theta + HALF_PI)
    const u = mod2pi(PI - 2 * A)
    const v = mod2pi(-phi + t + u)
    return [[t, -u, -v], ['L', 'R', 'L']]
  }
  return null
}

const LRxL: Word = (x, y, phi) => {
  const [u1, theta] = polar(x - Math.sin(phi), y - 1 + Math.cos(phi))
  if (u1 <= 4 && u1 > 1e-9) {
    const u = Math.acos(1 - u1 * u1 * 0.125)
    const A = Math.asin(Math.max(-1, Math.min(1, (2 * Math.sin(u)) / u1)))
    const t = mod2pi(-A + theta + HALF_PI)
    const v = mod2pi(t - u - phi)
    return [[t, u, -v], ['L', 'R', 'L']]
  }
  return null
}

// --- CCCC ----------------------------------------------------------------------------------
const LRxLR: Word = (x, y, phi) => {
  const [u1, theta] = polar(x + Math.sin(phi), y - 1 - Math.cos(phi))
  if (u1 <= 2) {
    const A = Math.acos((u1 + 2) * 0.25)
    const t = mod2pi(theta + A + HALF_PI)
    const u = mod2pi(A)
    const v = mod2pi(phi - t + 2 * u)
    if (t >= 0 && u >= 0 && v >= 0) return [[t, u, -u, -v], ['L', 'R', 'L', 'R']]
  }
  return null
}

const LxRLxR: Word = (x, y, phi) => {
  const [u1, theta] = polar(x + Math.sin(phi), y - 1 - Math.cos(phi))
  const u2 = (20 - u1 * u1) / 16
  if (u2 >= 0 && u2 <= 1 && u1 > 1e-9) {
    const u = Math.acos(u2)
    const A = Math.asin(Math.max(-1, Math.min(1, (2 * Math.sin(u)) / u1)))
    const t = mod2pi(theta + A + HALF_PI)
    const v = mod2pi(t - phi)
    if (t >= 0 && v >= 0) return [[t, -u, -u, v], ['L', 'R', 'L', 'R']]
  }
  return null
}

// --- CCSC / CSCC -----------------------------------------------------------------------------
const LxR90SL: Word = (x, y, phi) => {
  const [u1, theta] = polar(x - Math.sin(phi), y - 1 + Math.cos(phi))
  if (u1 >= 2) {
    const r = Math.sqrt(u1 * u1 - 4)
    const u = r - 2
    const A = Math.atan2(2, r)
    const t = mod2pi(theta + A + HALF_PI)
    const v = mod2pi(t - phi + HALF_PI)
    if (t >= 0 && v >= 0) return [[t, -HALF_PI, -u, -v], ['L', 'R', 'S', 'L']]
  }
  return null
}

const LSR90xL: Word = (x, y, phi) => {
  const [u1, theta] = polar(x - Math.sin(phi), y - 1 + Math.cos(phi))
  if (u1 >= 2) {
    const r = Math.sqrt(u1 * u1 - 4)
    const u = r - 2
    const A = Math.atan2(r, 2)
    const t = mod2pi(theta - A + HALF_PI)
    const v = mod2pi(t - phi - HALF_PI)
    if (t >= 0 && v >= 0) return [[t, u, HALF_PI, -v], ['L', 'S', 'R', 'L']]
  }
  return null
}

const LxR90SR: Word = (x, y, phi) => {
  const [u1, theta] = polar(x + Math.sin(phi), y - 1 - Math.cos(phi))
  if (u1 >= 2) {
    const t = mod2pi(theta + HALF_PI)
    const u = u1 - 2
    const v = mod2pi(phi - t - HALF_PI)
    if (t >= 0 && v >= 0) return [[t, -HALF_PI, -u, -v], ['L', 'R', 'S', 'R']]
  }
  return null
}

const LSL90xR: Word = (x, y, phi) => {
  const [u1, theta] = polar(x + Math.sin(phi), y - 1 - Math.cos(phi))
  if (u1 >= 2) {
    const t = mod2pi(theta)
    const u = u1 - 2
    const v = mod2pi(phi - t - HALF_PI)
    if (t >= 0 && v >= 0) return [[t, u, HALF_PI, -v], ['L', 'S', 'L', 'R']]
  }
  return null
}

// --- CCSCC ---------------------------------------------------------------------------------
const LxR90SL90xR: Word = (x, y, phi) => {
  const [u1, theta] = polar(x + Math.sin(phi), y - 1 - Math.cos(phi))
  if (u1 >= 4) {
    const r = Math.sqrt(u1 * u1 - 4)
    const u = r - 4
    const A = Math.atan2(2, r)
    const t = mod2pi(theta + A + HALF_PI)
    const v = mod2pi(t - phi)
    if (t >= 0 && v >= 0) return [[t, -HALF_PI, -u, -HALF_PI, v], ['L', 'R', 'S', 'L', 'R']]
  }
  return null
}

export const WORDS: Record<string, Word> = { LSL, LSR, LxRxL, LxRL, LRxL, LRxLR, LxRLxR, LxR90SL, LSR90xL, LxR90SR, LSL90xR, LxR90SL90xR }

const reflect = (t: SegmentType[]): SegmentType[] => t.map((s) => (s === 'L' ? 'R' : s === 'R' ? 'L' : 'S'))

/** Moves a normalized pose along one segment (signed length). */
function advance(p: Pose, seg: RsSegment): Pose {
  const l = seg.length
  if (seg.type === 'S') return { x: p.x + l * Math.cos(p.theta), y: p.y + l * Math.sin(p.theta), theta: p.theta }
  const k = seg.type === 'L' ? 1 : -1
  const t2 = p.theta + k * l
  return { x: p.x + (Math.sin(t2) - Math.sin(p.theta)) / k, y: p.y + (Math.cos(p.theta) - Math.cos(t2)) / k, theta: t2 }
}

/** All valid Reeds-Shepp paths from `start` to `goal` (turning radius `radius`). */
export function reedsSheppPaths(start: Pose, goal: Pose, radius: number): RsPath[] {
  const c = Math.cos(start.theta)
  const s = Math.sin(start.theta)
  const dx = goal.x - start.x
  const dy = goal.y - start.y
  const x = (c * dx + s * dy) / radius
  const y = (-s * dx + c * dy) / radius
  const phi = mod2pi(goal.theta - start.theta)
  const out: RsPath[] = []
  const add = (lengths: number[], types: SegmentType[]) => {
    const segments = lengths.map((length, i) => ({ type: types[i], length })).filter((sg) => Math.abs(sg.length) > 1e-10)
    // verify: integrate the word and check it lands on the target (normalized frame)
    let p: Pose = { x: 0, y: 0, theta: 0 }
    for (const sg of segments) p = advance(p, sg)
    if (Math.hypot(p.x - x, p.y - y) > 1e-6 || Math.abs(mod2pi(p.theta - phi)) > 1e-6) return
    out.push({ segments, length: segments.reduce((sum, sg) => sum + Math.abs(sg.length), 0) * radius })
  }
  for (const word of Object.values(WORDS)) {
    let r = word(x, y, phi)
    if (r) add(r[0], r[1])
    r = word(-x, y, -phi) // time flip
    if (r) add(r[0].map((l) => -l), r[1])
    r = word(x, -y, -phi) // reflection
    if (r) add(r[0], reflect(r[1]))
    r = word(-x, -y, phi) // both
    if (r) add(r[0].map((l) => -l), reflect(r[1]))
  }
  return out
}

/** Samples a Reeds-Shepp path every `step` meters; `dir` is +1 forward, -1 reverse. */
export function sampleReedsShepp(start: Pose, path: RsPath, radius: number, step: number): (Pose & { dir: number })[] {
  const out: (Pose & { dir: number })[] = []
  let p: Pose = { ...start }
  for (const seg of path.segments) {
    const dir = Math.sign(seg.length)
    const meters = Math.abs(seg.length) * radius
    const n = Math.max(1, Math.ceil(meters / step))
    const piece: RsSegment = { type: seg.type, length: seg.length / n }
    for (let i = 0; i < n; i++) {
      // advance in the world frame: scale the normalized motion by the radius
      const local = advance({ x: 0, y: 0, theta: 0 }, piece)
      const c = Math.cos(p.theta)
      const s = Math.sin(p.theta)
      p = { x: p.x + radius * (c * local.x - s * local.y), y: p.y + radius * (s * local.x + c * local.y), theta: p.theta + local.theta }
      out.push({ ...p, dir })
    }
  }
  return out
}
