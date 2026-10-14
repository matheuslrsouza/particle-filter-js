import type { Rng } from '../math/rng'
import type { Pose, Vec2 } from '../math/vec'
import type { LaneGraph } from './roadNetwork'

/** A piece of the lane graph a vehicle can be on: a lane or a junction connector. */
export interface Edge {
  kind: 'lane' | 'connector'
  id: number
}

export interface Npc {
  id: number
  edge: Edge
  /** distance along the edge (center of the car) */
  s: number
  v: number
  /** desired speed as a fraction of the speed limit (drivers differ) */
  eagerness: number
  /** edges after the current one */
  plan: Edge[]
  /** connector reserved at the next junction (-1 if none) */
  reserved: number
  pose: Pose
  color: number
  /** time stopped, to recycle cars stuck in gridlock */
  stuckTime: number
}

export interface Crosswalk {
  id: number
  /** the two sidewalk ends */
  a: Vec2
  b: Vec2
  /** where it crosses each lane */
  lanes: { lane: number; s: number }[]
}

export interface Pedestrian {
  id: number
  pos: Vec2
  heading: number
  state: 'waiting' | 'crossing'
  crosswalk: number
  /** 0: waiting at end a (going to b), 1: at b */
  side: number
  wait: number
  speed: number
  color: number
}

/** Intelligent Driver Model parameters. */
export const IDM = { aMax: 1.5, bComf: 2, s0: 2, T: 1.4, delta: 4 }

export const NPC_LENGTH = 4.7
export const NPC_WIDTH = 1.85

/** A vehicle seen by the others: position on an edge (for queues) and its pose (for geometry). */
export interface EgoState {
  pose: Pose
  speed: number
  edge: Edge | null
  s: number
  /** junction connector the ego car holds (-1 if none) */
  reserved: number
}

const key = (e: Edge) => (e.kind === 'lane' ? e.id : -1 - e.id)

/**
 * Traffic in the city: cars driving the lane graph with the IDM car-following model and a
 * first-come-first-served reservation at junctions (a car enters only if no conflicting
 * movement is using the junction), and pedestrians crossing at crosswalks.
 */
export class Traffic {
  npcs: Npc[] = []
  pedestrians: Pedestrian[] = []
  crosswalks: Crosswalk[] = []
  /** connector -> vehicle holding it (npc id, or -1 for the ego car) */
  reservations = new Map<number, number>()
  private arcs = new Map<number, number[]>()
  private conflicts: Set<number>[]
  private nextId = 0
  /** lanes cars must not enter (blocked roads) */
  blocked = new Set<number>()
  /** cars replaced because they were stuck or left the map */
  recycled = 0

  constructor(
    readonly g: LaneGraph,
    private rng: Rng,
    cars = 40,
    peds = 30,
  ) {
    this.conflicts = this.computeConflicts()
    this.crosswalks = this.buildCrosswalks()
    for (let i = 0; i < cars; i++) this.spawnCar()
    for (let i = 0; i < peds && this.crosswalks.length; i++) this.spawnPedestrian()
  }

  // ---------------------------------------------------------------- geometry of edges

  points(e: Edge): Vec2[] {
    return e.kind === 'lane' ? this.g.lanes[e.id].points : this.g.connectors[e.id].points
  }

  length(e: Edge): number {
    return e.kind === 'lane' ? this.g.lanes[e.id].length : this.g.connectors[e.id].length
  }

  speedLimit(e: Edge): number {
    if (e.kind === 'lane') return this.g.lanes[e.id].speed
    const c = this.g.connectors[e.id]
    return Math.min(this.g.lanes[c.from].speed, this.g.lanes[c.to].speed) * (c.turn === 'straight' ? 1 : 0.55)
  }

  private arc(e: Edge): number[] {
    const k = key(e)
    let a = this.arcs.get(k)
    if (!a) {
      const pts = this.points(e)
      a = [0]
      for (let i = 1; i < pts.length; i++) a.push(a[i - 1] + Math.hypot(pts[i].x - pts[i - 1].x, pts[i].y - pts[i - 1].y))
      this.arcs.set(k, a)
    }
    return a
  }

  poseAt(e: Edge, s: number): Pose {
    const pts = this.points(e)
    const a = this.arc(e)
    let i = 1
    while (i < a.length - 1 && a[i] < s) i++
    const t = a[i] > a[i - 1] ? Math.min(1, Math.max(0, (s - a[i - 1]) / (a[i] - a[i - 1]))) : 0
    const p = pts[i - 1]
    const q = pts[i]
    return { x: p.x + (q.x - p.x) * t, y: p.y + (q.y - p.y) * t, theta: Math.atan2(q.y - p.y, q.x - p.x) }
  }

  // ---------------------------------------------------------------- junction conflicts

  /** Connectors of the same junction that cross each other or merge into the same lane. */
  private computeConflicts(): Set<number>[] {
    const out = this.g.connectors.map(() => new Set<number>())
    const byJunction = new Map<number, number[]>()
    for (const c of this.g.connectors) {
      if (!byJunction.has(c.junction)) byJunction.set(c.junction, [])
      byJunction.get(c.junction)!.push(c.id)
    }
    for (const ids of byJunction.values()) {
      for (let i = 0; i < ids.length; i++) {
        for (let j = i + 1; j < ids.length; j++) {
          const a = this.g.connectors[ids[i]]
          const b = this.g.connectors[ids[j]]
          if (a.from === b.from) continue // same queue: they follow each other
          if (a.to === b.to || polylinesCross(a.points, b.points)) {
            out[a.id].add(b.id)
            out[b.id].add(a.id)
          }
        }
      }
    }
    return out
  }

  /** Tries to reserve a connector for a vehicle (-1 = ego). */
  reserve(connector: number, vehicle: number): boolean {
    const holder = this.reservations.get(connector)
    if (holder !== undefined && holder !== vehicle) {
      // same movement: only one at a time inside the junction, the next waits at the line
      return false
    }
    for (const other of this.conflicts[connector]) {
      const h = this.reservations.get(other)
      if (h !== undefined && h !== vehicle) return false
    }
    this.reservations.set(connector, vehicle)
    return true
  }

  release(connector: number, vehicle: number): void {
    if (this.reservations.get(connector) === vehicle) this.reservations.delete(connector)
  }

  // ---------------------------------------------------------------- crosswalks & pedestrians

  private buildCrosswalks(): Crosswalk[] {
    const out: Crosswalk[] = []
    for (const seg of this.g.segments) {
      const c = seg.center
      const len = c.reduce((acc, p, i) => (i ? acc + Math.hypot(p.x - c[i - 1].x, p.y - c[i - 1].y) : 0), 0)
      if (len < 30 || !['residential', 'tertiary', 'secondary', 'unclassified'].includes(seg.road.highway)) continue
      for (const [end, junction] of [[0, seg.from], [1, seg.to]] as const) {
        if (this.g.junctions[junction].segments.length < 3) continue
        // 3 m into the street from the junction edge
        const p0 = end === 0 ? c[0] : c[c.length - 1]
        const p1 = end === 0 ? c[1] : c[c.length - 2]
        const d = Math.hypot(p1.x - p0.x, p1.y - p0.y) || 1
        const along = { x: (p1.x - p0.x) / d, y: (p1.y - p0.y) / d }
        const at = { x: p0.x + along.x * 3, y: p0.y + along.y * 3 }
        // normal pointing right of the segment direction
        const sign = end === 0 ? 1 : -1
        const n = { x: -along.y * sign, y: along.x * sign }
        const a = { x: at.x + n.x * (seg.rightWidth + 1), y: at.y + n.y * (seg.rightWidth + 1) }
        const b = { x: at.x - n.x * (seg.leftWidth + 1), y: at.y - n.y * (seg.leftWidth + 1) }
        const lanes = [...seg.forward, ...seg.backward].map((lane) => ({ lane, s: projectS(this.g.lanes[lane].points, at) }))
        out.push({ id: out.length, a, b, lanes })
      }
    }
    return out
  }

  private spawnPedestrian(): void {
    const cw = this.crosswalks[this.rng.int(this.crosswalks.length)]
    const side = this.rng.int(2)
    this.pedestrians.push({
      id: this.pedestrians.length,
      pos: { ...(side ? cw.b : cw.a) },
      heading: 0,
      state: 'waiting',
      crosswalk: cw.id,
      side,
      wait: this.rng.range(2, 15),
      speed: this.rng.range(1.1, 1.5),
      color: [0xe76f51, 0x2a9d8f, 0x264653, 0xf4a261, 0x8338ec, 0x3a86ff][this.rng.int(6)],
    })
  }

  /** Crosswalks with someone on them. */
  activeCrosswalks(): Set<number> {
    return new Set(this.pedestrians.filter((p) => p.state === 'crossing').map((p) => p.crosswalk))
  }

  private stepPedestrians(dt: number, ego: EgoState): void {
    for (const p of this.pedestrians) {
      const cw = this.crosswalks[p.crosswalk]
      const from = p.side ? cw.b : cw.a
      const to = p.side ? cw.a : cw.b
      if (p.state === 'waiting') {
        p.wait -= dt
        p.heading = Math.atan2(to.y - from.y, to.x - from.x)
        // pedestrians have the right of way, but don't step in front of a car about to arrive
        if (p.wait <= 0 && this.safeToCross(cw, ego)) p.state = 'crossing'
        continue
      }
      const dx = to.x - p.pos.x
      const dy = to.y - p.pos.y
      const d = Math.hypot(dx, dy)
      if (d < 0.2) {
        p.state = 'waiting'
        p.side = 1 - p.side
        p.wait = this.rng.range(6, 25)
        continue
      }
      const step = Math.min(d, p.speed * dt)
      p.pos = { x: p.pos.x + (dx / d) * step, y: p.pos.y + (dy / d) * step }
      p.heading = Math.atan2(dy, dx)
    }
  }

  private safeToCross(cw: Crosswalk, ego: EgoState): boolean {
    // cars whose planned route goes over this crosswalk within the next few seconds
    for (const n of this.npcs) {
      let dist = -n.s
      for (const e of [n.edge, ...n.plan]) {
        if (dist > 60) break
        if (e.kind === 'lane') {
          const at = cw.lanes.find((l) => l.lane === e.id)
          if (at && dist + at.s > -NPC_LENGTH) {
            const d = dist + at.s
            if (d < 15 || (n.v > 0.5 && d / n.v < 4)) return false
            break
          }
        }
        dist += this.length(e)
      }
    }
    const mid = { x: (cw.a.x + cw.b.x) / 2, y: (cw.a.y + cw.b.y) / 2 }
    const vehicles: { pose: Pose; v: number }[] = this.npcs.map((n) => ({ pose: n.pose, v: n.v }))
    vehicles.push({ pose: ego.pose, v: ego.speed })
    for (const veh of vehicles) {
      const dx = mid.x - veh.pose.x
      const dy = mid.y - veh.pose.y
      const d = Math.hypot(dx, dy)
      if (d > 45) continue
      // anything close (even turning, pointing elsewhere), or approaching and due within 3 s
      if (d < 12) return false
      const ahead = dx * Math.cos(veh.pose.theta) + dy * Math.sin(veh.pose.theta)
      if (ahead > 0 && veh.v > 0.5 && ahead / veh.v < 3) return false
    }
    return true
  }

  // ---------------------------------------------------------------- cars

  /** where the ego car is, so new cars don't appear next to it */
  egoPose: Pose | null = null

  private spawnCar(): void {
    for (let tries = 0; tries < 50; tries++) {
      const lane = this.g.lanes[this.rng.int(this.g.lanes.length)]
      if (lane.length < 20 || this.blocked.has(lane.id) || !lane.next.length) continue
      const s = this.rng.range(5, lane.length - 5)
      const edge: Edge = { kind: 'lane', id: lane.id }
      const p = this.poseAt(edge, s)
      // not on top of another car, and out of the ego car's surroundings
      if (this.npcs.some((n) => Math.hypot(n.pose.x - p.x, n.pose.y - p.y) < 12)) continue
      if (this.egoPose && Math.hypot(this.egoPose.x - p.x, this.egoPose.y - p.y) < 60) continue
      // nor right before a crosswalk someone is using
      if (this.pedestrians.some((q) => q.state === 'crossing' && Math.hypot(q.pos.x - p.x, q.pos.y - p.y) < 40)) continue
      const npc: Npc = {
        id: this.nextId++,
        edge,
        s,
        v: lane.speed * 0.5,
        eagerness: this.rng.range(0.85, 1.05),
        plan: [],
        reserved: -1,
        pose: this.poseAt(edge, s),
        color: [0xf1faee, 0x457b9d, 0x1d3557, 0xa8dadc, 0x2a9d8f, 0xe9c46a, 0x6c757d, 0x222222, 0xb5838d, 0x8d99ae][this.rng.int(10)],
        stuckTime: 0,
      }
      this.extendPlan(npc)
      this.npcs.push(npc)
      return
    }
  }

  /** Random route ahead: mostly straight, sometimes turning; never into a blocked lane. */
  private extendPlan(n: Npc): void {
    while (n.plan.length < 4) {
      const last = n.plan.length ? n.plan[n.plan.length - 1] : n.edge
      if (last.kind === 'connector') {
        n.plan.push({ kind: 'lane', id: this.g.connectors[last.id].to })
        continue
      }
      const options = this.g.lanes[last.id].next.filter((c) => !this.blocked.has(this.g.connectors[c].to))
      if (!options.length) return // dead end: the car will be recycled there
      const weights = options.map((c) => (this.g.connectors[c].turn === 'straight' ? 3 : 1))
      let r = this.rng.next() * weights.reduce((a, b) => a + b, 0)
      let pick = options[0]
      for (let i = 0; i < options.length; i++) {
        r -= weights[i]
        if (r <= 0) {
          pick = options[i]
          break
        }
      }
      n.plan.push({ kind: 'connector', id: pick })
    }
  }

  /** Cars on each edge, sorted by position, for the car-following queries. */
  private occupancy(ego: EgoState): Map<number, { s: number; v: number; id: number; length: number }[]> {
    const occ = new Map<number, { s: number; v: number; id: number; length: number }[]>()
    const add = (e: Edge, s: number, v: number, id: number, length: number) => {
      const k = key(e)
      if (!occ.has(k)) occ.set(k, [])
      occ.get(k)!.push({ s, v, id, length })
    }
    for (const n of this.npcs) add(n.edge, n.s, n.v, n.id, NPC_LENGTH)
    if (ego.edge) add(ego.edge, ego.s, Math.max(0, ego.speed), -1, 5.04)
    for (const list of occ.values()) list.sort((a, b) => a.s - b.s)
    return occ
  }

  step(dt: number, ego: EgoState): void {
    this.egoPose = ego.pose
    this.stepPedestrians(dt, ego)
    const occ = this.occupancy(ego)
    const crossings = this.activeCrosswalks()
    const recycle: Npc[] = []
    for (const n of this.npcs) {
      // leader: the nearest obstacle ahead along the plan (car, stop line, active crosswalk)
      let gap = Infinity
      let leaderV = 0
      let dist = -n.s
      const edges = [n.edge, ...n.plan]
      for (let k = 0; k < edges.length && dist < 80; k++) {
        const e = edges[k]
        const len = this.length(e)
        const from = k === 0 ? n.s : -Infinity
        for (const o of occ.get(key(e)) ?? []) {
          if (o.id === n.id || o.s <= from) continue
          const g = dist + o.s - (NPC_LENGTH + o.length) / 2
          if (g < gap) {
            gap = g
            leaderV = o.v
          }
          break
        }
        if (gap < Infinity) break
        if (e.kind === 'lane') {
          for (const cw of this.crosswalks) {
            if (!crossings.has(cw.id)) continue
            const at = cw.lanes.find((l) => l.lane === e.id)
            if (at && at.s > from) {
              const g = dist + at.s - NPC_LENGTH / 2 - 1.5
              if (g < gap) {
                gap = g
                leaderV = 0
              }
            }
          }
          // the next edge is a junction connector: it must be reserved to go on
          const next = edges[k + 1]
          if (next && next.kind === 'connector' && n.reserved !== next.id) {
            const toLine = dist + len - NPC_LENGTH / 2 - 0.5
            // ask early enough to stop comfortably if the junction is taken
            const ask = Math.max(30, (n.v * n.v) / (2 * IDM.bComf) + 10)
            if (toLine < ask && !this.reserve(next.id, n.id)) {
              if (toLine < gap) {
                gap = toLine
                leaderV = 0
              }
            } else if (toLine < ask) n.reserved = next.id
          }
        }
        if (gap < Infinity) break
        dist += len
      }
      // geometric check on top of the lane queues: the ego car (it may be maneuvering off the
      // lanes) and cars on other lanes merging into this one without a junction
      const c = Math.cos(n.pose.theta)
      const sn = Math.sin(n.pose.theta)
      const look = (pose: Pose, v: number, theta: number, length: number, anyHeading: boolean) => {
        const ex = pose.x - n.pose.x
        const ey = pose.y - n.pose.y
        const ahead = ex * c + ey * sn
        const lateral = Math.abs(-ex * sn + ey * c)
        // ahead, in our corridor (other cars: only those going our way; the ego car: always)
        if (ahead > 0 && ahead < 35 && lateral < (anyHeading ? 2.2 : 2) && (anyHeading || Math.cos(theta - n.pose.theta) > 0.3)) {
          const g = ahead - (NPC_LENGTH + length) / 2
          if (g < gap) {
            gap = g
            leaderV = Math.max(0, v * Math.cos(theta - n.pose.theta))
          }
        }
      }
      look(ego.pose, ego.speed, ego.pose.theta, 5.04, true)
      for (const o of this.npcs) if (o !== n) look(o.pose, o.v, o.pose.theta, NPC_LENGTH, false)
      // safety net, like any driver: whoever is walking in front of the car
      for (const p of this.pedestrians) {
        if (p.state !== 'crossing') continue
        const px = p.pos.x - n.pose.x
        const py = p.pos.y - n.pose.y
        const ahead = px * c + py * sn
        if (ahead > 0 && ahead < 25 && Math.abs(-px * sn + py * c) < 1.8) {
          const g = ahead - NPC_LENGTH / 2 - 1.5
          if (g < gap) {
            gap = g
            leaderV = 0
          }
        }
      }

      const v0 = this.speedLimit(n.edge) * n.eagerness
      n.v = Math.max(0, n.v + idmAccel(n.v, v0, gap, n.v - leaderV) * dt)
      n.s += n.v * dt
      n.stuckTime = n.v < 0.1 ? n.stuckTime + dt : 0

      // move on to the next edges
      while (n.s > this.length(n.edge)) {
        const next = n.plan.shift()
        if (!next) break
        n.s -= this.length(n.edge)
        if (n.edge.kind === 'connector') this.release(n.edge.id, n.id)
        n.edge = next
        if (next.kind === 'connector') n.reserved = -1
        this.extendPlan(n)
      }
      n.pose = this.poseAt(n.edge, Math.min(n.s, this.length(n.edge)))
      const deadEnd = !n.plan.length && n.s >= this.length(n.edge) - 0.5
      if (deadEnd || n.stuckTime > 30 || (n.edge.kind === 'lane' && this.blocked.has(n.edge.id))) recycle.push(n)
    }
    // cars leaving the map (dead ends), stuck in gridlock or on a road that was just blocked
    // are replaced by new ones elsewhere
    for (const n of recycle) {
      this.recycled++
      for (const [c, h] of this.reservations) if (h === n.id) this.reservations.delete(c)
      this.npcs.splice(this.npcs.indexOf(n), 1)
      this.spawnCar()
    }
  }
}

/** IDM acceleration: free-road term and interaction term. */
export function idmAccel(v: number, v0: number, gap: number, dv: number): number {
  const { aMax, bComf, s0, T, delta } = IDM
  const free = 1 - Math.pow(v / Math.max(0.1, v0), delta)
  if (!Number.isFinite(gap)) return aMax * free
  const sStar = s0 + Math.max(0, v * T + (v * dv) / (2 * Math.sqrt(aMax * bComf)))
  return Math.max(-8, aMax * (free - Math.pow(sStar / Math.max(0.1, gap), 2)))
}

function projectS(pts: Vec2[], p: Vec2): number {
  let best = Infinity
  let bestS = 0
  let s = 0
  for (let i = 1; i < pts.length; i++) {
    const a = pts[i - 1]
    const b = pts[i]
    const dx = b.x - a.x
    const dy = b.y - a.y
    const len2 = dx * dx + dy * dy
    const t = len2 > 0 ? Math.max(0, Math.min(1, ((p.x - a.x) * dx + (p.y - a.y) * dy) / len2)) : 0
    const d = Math.hypot(p.x - (a.x + dx * t), p.y - (a.y + dy * t))
    if (d < best) {
      best = d
      bestS = s + Math.sqrt(len2) * t
    }
    s += Math.sqrt(len2)
  }
  return bestS
}

function segmentsCross(a: Vec2, b: Vec2, c: Vec2, d: Vec2): boolean {
  const o = (p: Vec2, q: Vec2, r: Vec2) => (q.x - p.x) * (r.y - p.y) - (q.y - p.y) * (r.x - p.x)
  return o(a, b, c) * o(a, b, d) < 0 && o(c, d, a) * o(c, d, b) < 0
}

function polylinesCross(p: Vec2[], q: Vec2[]): boolean {
  for (let i = 1; i < p.length; i += 1) for (let j = 1; j < q.length; j += 1) if (segmentsCross(p[i - 1], p[i], q[j - 1], q[j])) return true
  return false
}
