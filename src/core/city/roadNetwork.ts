import { wrapAngle } from '../math/angles'
import type { Vec2 } from '../math/vec'
import type { CityDef, CityRoad } from './cityDef'

export interface NetworkParams {
  laneWidth: number
  /** extra room around a junction, so turning lanes have a drivable radius */
  curbRadius: number
}

export const DEFAULT_NETWORK: NetworkParams = { laneWidth: 3.3, curbRadius: 4 }

export type Turn = 'straight' | 'left' | 'right'

/** A piece of road between two junctions. */
export interface Segment {
  id: number
  road: CityRoad
  from: number
  to: number
  /** centerline, trimmed so it stops at the edge of both junctions */
  center: Vec2[]
  /** roadway extent to the right / left of the centerline, looking from `from` to `to` */
  rightWidth: number
  leftWidth: number
  /** lane ids per direction, index 0 = leftmost lane of that direction */
  forward: number[]
  backward: number[]
}

export interface Junction {
  id: number
  pos: Vec2
  segments: number[]
  radius: number
}

export interface Lane {
  id: number
  segment: number
  /** +1 along the segment, -1 against it */
  dir: number
  /** 0 = leftmost lane of its direction */
  index: number
  count: number
  points: Vec2[]
  length: number
  speed: number
  name: string
  /** neighbor lanes of the same direction (-1 if none) */
  left: number
  right: number
  startJunction: number
  endJunction: number
  /** connectors leaving the end of this lane */
  next: number[]
}

export interface Connector {
  id: number
  from: number
  to: number
  junction: number
  turn: Turn
  points: Vec2[]
  length: number
}

export interface LaneGraph {
  segments: Segment[]
  junctions: Junction[]
  lanes: Lane[]
  connectors: Connector[]
  params: NetworkParams
}

const dist = (a: Vec2, b: Vec2) => Math.hypot(b.x - a.x, b.y - a.y)
const heading = (a: Vec2, b: Vec2) => Math.atan2(b.y - a.y, b.x - a.x)

export function polylineLength(pts: Vec2[]): number {
  let l = 0
  for (let i = 1; i < pts.length; i++) l += dist(pts[i - 1], pts[i])
  return l
}

/** Cuts `head` meters from the start and `tail` meters from the end of a polyline. */
export function trimPolyline(pts: Vec2[], head: number, tail: number): Vec2[] {
  const total = polylineLength(pts)
  const from = Math.min(head, total)
  const to = Math.max(from, total - tail)
  const out: Vec2[] = []
  let s = 0
  for (let i = 1; i < pts.length; i++) {
    const a = pts[i - 1]
    const b = pts[i]
    const len = dist(a, b)
    const s0 = s
    const s1 = s + len
    const at = (d: number) => ({ x: a.x + ((b.x - a.x) * (d - s0)) / len, y: a.y + ((b.y - a.y) * (d - s0)) / len })
    if (len > 0) {
      if (s1 >= from && s0 <= to) {
        if (!out.length) out.push(at(Math.max(from, s0)))
        if (s1 <= to) out.push({ ...b })
        else out.push(at(to))
      }
    }
    s = s1
    if (s > to) break
  }
  if (out.length === 1) out.push({ ...out[0] })
  return out
}

/**
 * Offsets a polyline `d` meters to the right of its direction of travel (x east, y south:
 * the right of a heading (dx, dy) is (-dy, dx)), with mitered joints.
 */
export function offsetPolyline(pts: Vec2[], d: number): Vec2[] {
  const n = pts.length
  const normals = pts.slice(0, -1).map((p, i) => {
    const q = pts[i + 1]
    const len = dist(p, q) || 1
    return { x: -(q.y - p.y) / len, y: (q.x - p.x) / len }
  })
  return pts.map((p, i) => {
    const a = normals[Math.max(0, i - 1)]
    const b = normals[Math.min(n - 2, i)]
    let mx = a.x + b.x
    let my = a.y + b.y
    const ml = Math.hypot(mx, my) || 1
    mx /= ml
    my /= ml
    // miter: scale so the offset lines stay parallel, limited on sharp corners
    const cos = Math.max(0.5, mx * b.x + my * b.y)
    return { x: p.x + (mx * d) / cos, y: p.y + (my * d) / cos }
  })
}

/** Cubic Hermite curve between two poses, sampled every `step` meters (approximately). */
function hermite(p0: Vec2, h0: number, p1: Vec2, h1: number, step: number): Vec2[] {
  const chord = dist(p0, p1)
  const turn = Math.abs(wrapAngle(h1 - h0))
  // tangent length that makes a 90° turn close to a circular arc
  const L = chord * (1 + 0.17 * Math.min(1, turn / (Math.PI / 2)))
  const t0 = { x: Math.cos(h0) * L, y: Math.sin(h0) * L }
  const t1 = { x: Math.cos(h1) * L, y: Math.sin(h1) * L }
  const n = Math.max(2, Math.ceil(chord / step))
  const out: Vec2[] = []
  for (let i = 0; i <= n; i++) {
    const t = i / n
    const t2 = t * t
    const t3 = t2 * t
    const a = 2 * t3 - 3 * t2 + 1
    const b = t3 - 2 * t2 + t
    const c = -2 * t3 + 3 * t2
    const d = t3 - t2
    out.push({ x: a * p0.x + b * t0.x + c * p1.x + d * t1.x, y: a * p0.y + b * t0.y + c * p1.y + d * t1.y })
  }
  return out
}

/**
 * Builds the lane-level road network from the OSM roads: junctions where roads share nodes,
 * lanes offset to the right of their direction (right-hand traffic), and connectors through
 * each junction for the allowed movements (straight, right turn from the rightmost lane,
 * left turn from the leftmost lane, no U-turns).
 */
export function buildLaneGraph(city: CityDef, params: NetworkParams = DEFAULT_NETWORK): LaneGraph {
  const W = params.laneWidth

  // 1. junction nodes: shared by several roads, or road ends
  const usage = new Map<number, number>()
  for (const r of city.roads) for (const id of r.nodes) usage.set(id, (usage.get(id) ?? 0) + 1)
  const junctionOf = new Map<number, number>()
  const junctions: Junction[] = []
  const junctionAt = (nodeId: number, p: [number, number]) => {
    let j = junctionOf.get(nodeId)
    if (j === undefined) {
      j = junctions.length
      junctions.push({ id: j, pos: { x: p[0], y: p[1] }, segments: [], radius: 0 })
      junctionOf.set(nodeId, j)
    }
    return j
  }

  // 2. split roads into segments between junctions
  const segments: Segment[] = []
  for (const road of city.roads) {
    let start = 0
    for (let i = 1; i < road.nodes.length; i++) {
      const isJunction = (usage.get(road.nodes[i]) ?? 0) > 1 || i === road.nodes.length - 1
      if (!isJunction) continue
      const pts = road.pts.slice(start, i + 1).map(([x, y]) => ({ x, y }))
      if (polylineLength(pts) > 0.5) {
        const from = junctionAt(road.nodes[start], road.pts[start])
        const to = junctionAt(road.nodes[i], road.pts[i])
        if (from !== to) {
          const oneway = road.lanesBackward === 0
          const center = road.centerTurnLane ? W / 2 : 0
          const seg: Segment = {
            id: segments.length,
            road,
            from,
            to,
            center: pts,
            rightWidth: oneway ? (road.lanesForward * W) / 2 : center + road.lanesForward * W,
            leftWidth: oneway ? (road.lanesForward * W) / 2 : center + road.lanesBackward * W,
            forward: [],
            backward: [],
          }
          segments.push(seg)
          junctions[from].segments.push(seg.id)
          junctions[to].segments.push(seg.id)
        }
      }
      start = i
    }
  }

  // 3. junction radius (how far lanes stop from the junction center): the widest road plus a
  //    curb radius, larger for acute corners (setback = R / tan(φ/2)). A node where a road just
  //    continues needs little room, unless the lane layout changes (then a taper is needed).
  for (const j of junctions) {
    const segs = j.segments.map((s) => segments[s])
    const maxW = Math.max(0, ...segs.map((s) => Math.max(s.rightWidth, s.leftWidth)))
    const headings = segs.map((s) => endHeading(s, j.id))
    let corner = Math.PI
    for (let a = 0; a < headings.length; a++) {
      for (let b = a + 1; b < headings.length; b++) corner = Math.min(corner, Math.abs(wrapAngle(headings[a] - headings[b])))
    }
    const setback = params.curbRadius * Math.min(3, Math.max(1, 1 / Math.tan(Math.max(0.2, corner) / 2)))
    if (segs.length >= 3) j.radius = maxW + setback
    else if (segs.length === 2) {
      const bend = Math.PI - corner
      const sameLayout =
        segs[0].road.lanesForward + segs[0].road.lanesBackward === segs[1].road.lanesForward + segs[1].road.lanesBackward &&
        segs[0].road.centerTurnLane === segs[1].road.centerTurnLane
      // a change of lane layout needs a long taper (~40 m), like on real streets
      j.radius = bend > 0.5 ? maxW + setback : sameLayout ? 1 : 20
    }
  }

  // 3b. junctions so close that the road between them would be a stub become one composite
  //     intersection: the stub disappears and the connectors cross the whole cluster
  const parent = junctions.map((j) => j.id)
  const find = (a: number): number => (parent[a] === a ? a : (parent[a] = find(parent[a])))
  for (const s of segments) {
    const a = junctions[s.from]
    const b = junctions[s.to]
    if (a.segments.length >= 3 && b.segments.length >= 3 && polylineLength(s.center) < a.radius + b.radius + 4) {
      parent[find(s.from)] = find(s.to)
    }
  }
  // each segment end is still trimmed by the radius of its own (member) junction
  const trims = new Map<Segment, [number, number]>()
  for (const s of segments) trims.set(s, [junctions[s.from].radius, junctions[s.to].radius])
  const kept = segments.filter((s) => find(s.from) !== find(s.to))
  const members = new Map<number, Junction[]>()
  for (const j of junctions) {
    const r = find(j.id)
    if (!members.has(r)) members.set(r, [])
    members.get(r)!.push(j)
  }
  for (const j of junctions) j.segments = []
  for (const [root, ms] of members) {
    if (ms.length > 1) {
      junctions[root].pos = { x: ms.reduce((a, m) => a + m.pos.x, 0) / ms.length, y: ms.reduce((a, m) => a + m.pos.y, 0) / ms.length }
      junctions[root].radius = Math.max(...ms.map((m) => m.radius))
    }
  }
  segments.length = 0
  for (const s of kept) {
    s.id = segments.length
    s.from = find(s.from)
    s.to = find(s.to)
    segments.push(s)
    junctions[s.from].segments.push(s.id)
    junctions[s.to].segments.push(s.id)
  }

  // 4. trim segments at the junctions (short segments share their length)
  for (const s of segments) {
    const len = polylineLength(s.center)
    let [rf, rt] = trims.get(s)!
    const keep = Math.min(2, len * 0.3)
    if (rf + rt > len - keep) {
      const k = Math.max(0, len - keep) / (rf + rt)
      rf *= k
      rt *= k
    }
    s.center = trimPolyline(s.center, rf, rt)
  }

  // 5. lanes
  const lanes: Lane[] = []
  for (const s of segments) {
    const road = s.road
    const oneway = road.lanesBackward === 0
    const addLanes = (dir: number, count: number, ids: number[]) => {
      const base = dir > 0 ? s.center : [...s.center].reverse()
      // the left edge of this direction: the centerline (plus the turn lane), or half the road on one-ways
      const leftEdge = oneway ? (-count * W) / 2 : road.centerTurnLane ? W / 2 : 0
      for (let k = 0; k < count; k++) {
        const points = offsetPolyline(base, leftEdge + (k + 0.5) * W)
        const id = lanes.length
        ids.push(id)
        lanes.push({
          id,
          segment: s.id,
          dir,
          index: k,
          count,
          points,
          length: polylineLength(points),
          speed: road.speed,
          name: road.name,
          left: -1,
          right: -1,
          startJunction: dir > 0 ? s.from : s.to,
          endJunction: dir > 0 ? s.to : s.from,
          next: [],
        })
      }
      ids.forEach((id, k) => {
        lanes[id].left = k > 0 ? ids[k - 1] : -1
        lanes[id].right = k < ids.length - 1 ? ids[k + 1] : -1
      })
    }
    addLanes(1, road.lanesForward, s.forward)
    addLanes(-1, road.lanesBackward, s.backward)
  }

  // 6. connectors through junctions
  const connectors: Connector[] = []
  const incoming = new Map<number, Lane[]>()
  const outgoing = new Map<number, Lane[]>()
  for (const l of lanes) {
    if (!incoming.has(l.endJunction)) incoming.set(l.endJunction, [])
    if (!outgoing.has(l.startJunction)) outgoing.set(l.startJunction, [])
    incoming.get(l.endJunction)!.push(l)
    outgoing.get(l.startJunction)!.push(l)
  }
  for (const j of junctions) {
    if (j.segments.length < 2) continue
    const passThrough = j.segments.length === 2
    for (const a of incoming.get(j.id) ?? []) {
      const pa = a.points[a.points.length - 1]
      const ha = heading(a.points[a.points.length - 2], pa)
      for (const b of outgoing.get(j.id) ?? []) {
        if (b.segment === a.segment) continue // no U-turns
        const pb = b.points[0]
        const hb = heading(pb, b.points[1])
        const delta = wrapAngle(hb - ha)
        if (Math.abs(delta) > (150 * Math.PI) / 180) continue
        // heading grows clockwise on screen (y south), so a right turn has delta > 0
        const turn: Turn = passThrough || Math.abs(delta) <= (35 * Math.PI) / 180 ? 'straight' : delta > 0 ? 'right' : 'left'
        if (!allowed(a, b, turn)) continue
        const points = hermite(pa, ha, pb, hb, 0.5)
        const c: Connector = { id: connectors.length, from: a.id, to: b.id, junction: j.id, turn, points, length: polylineLength(points) }
        connectors.push(c)
        a.next.push(c.id)
      }
    }
  }

  return { segments, junctions, lanes, connectors, params }
}

/** Heading of a segment's centerline at the given junction, pointing away from it. */
function endHeading(s: Segment, junction: number): number {
  const c = s.center
  return s.from === junction ? heading(c[0], c[1]) : heading(c[c.length - 1], c[c.length - 2])
}

/** Which lane pairs a movement may connect. */
function allowed(a: Lane, b: Lane, turn: Turn): boolean {
  const fromRight = (l: Lane) => l.count - 1 - l.index
  if (turn === 'right') return fromRight(a) === 0 && fromRight(b) === 0
  if (turn === 'left') return a.index === 0 && b.index === 0
  // straight: keep the position counted from the right; extra lanes on the left merge into the leftmost
  return fromRight(b) === Math.min(fromRight(a), b.count - 1)
}

/** Largest group of lanes that can reach each other (for sanity checks). */
export function largestComponent(g: LaneGraph): number {
  const adj = new Map<number, number[]>()
  const link = (a: number, b: number) => {
    if (!adj.has(a)) adj.set(a, [])
    if (!adj.has(b)) adj.set(b, [])
    adj.get(a)!.push(b)
    adj.get(b)!.push(a)
  }
  for (const c of g.connectors) link(c.from, c.to)
  for (const l of g.lanes) {
    if (l.left >= 0) link(l.id, l.left)
  }
  const seen = new Set<number>()
  let best = 0
  for (const l of g.lanes) {
    if (seen.has(l.id)) continue
    let size = 0
    const stack = [l.id]
    seen.add(l.id)
    while (stack.length) {
      const v = stack.pop()!
      size++
      for (const w of adj.get(v) ?? []) {
        if (!seen.has(w)) {
          seen.add(w)
          stack.push(w)
        }
      }
    }
    best = Math.max(best, size)
  }
  return best
}
