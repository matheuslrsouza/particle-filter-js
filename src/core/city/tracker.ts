import { wrapAngle } from '../math/angles'
import type { Vec2 } from '../math/vec'
import type { Detection } from './perception'

/**
 * Hungarian algorithm (Kuhn-Munkres, O(n³)): minimum-cost assignment of rows to columns.
 * `cost[i][j]` may be Infinity (not allowed). Returns, for each row, its column or -1.
 */
export function hungarian(cost: number[][]): number[] {
  const rows = cost.length
  if (!rows) return []
  const cols = cost[0].length
  const n = Math.max(rows, cols)
  const BIG = 1e9
  // square matrix, forbidden pairs replaced by a large finite cost
  const a = Array.from({ length: n + 1 }, (_, i) =>
    Array.from({ length: n + 1 }, (_, j) => (i && j && i <= rows && j <= cols && Number.isFinite(cost[i - 1][j - 1]) ? cost[i - 1][j - 1] : i && j ? BIG : 0)),
  )
  const u = new Array(n + 1).fill(0)
  const v = new Array(n + 1).fill(0)
  const p = new Array(n + 1).fill(0)
  const way = new Array(n + 1).fill(0)
  for (let i = 1; i <= n; i++) {
    p[0] = i
    let j0 = 0
    const minv = new Array(n + 1).fill(Infinity)
    const used = new Array(n + 1).fill(false)
    do {
      used[j0] = true
      const i0 = p[j0]
      let delta = Infinity
      let j1 = 0
      for (let j = 1; j <= n; j++) {
        if (used[j]) continue
        const cur = a[i0][j] - u[i0] - v[j]
        if (cur < minv[j]) {
          minv[j] = cur
          way[j] = j0
        }
        if (minv[j] < delta) {
          delta = minv[j]
          j1 = j
        }
      }
      for (let j = 0; j <= n; j++) {
        if (used[j]) {
          u[p[j]] += delta
          v[j] -= delta
        } else minv[j] -= delta
      }
      j0 = j1
    } while (p[j0] !== 0)
    do {
      const j1 = way[j0]
      p[j0] = p[j1]
      j0 = j1
    } while (j0)
  }
  const out = new Array(rows).fill(-1)
  for (let j = 1; j <= n; j++) {
    const i = p[j]
    if (i >= 1 && i <= rows && j <= cols && a[i][j] < BIG) out[i - 1] = j - 1
  }
  return out
}

export interface Track {
  id: number
  kind: 'car' | 'pedestrian'
  /** [x, y, vx, vy] */
  x: number[]
  /** 4x4 covariance */
  P: number[][]
  heading: number
  length: number
  width: number
  hits: number
  misses: number
  age: number
  confirmed: boolean
}

export interface TrackerParams {
  /** acceleration noise (m/s²) of the constant-velocity model, per class */
  accelStd: { car: number; pedestrian: number }
  /** detection noise (m) */
  measStd: { car: number; pedestrian: number }
  /** Mahalanobis gate (chi-square, 2 dof) */
  gate: number
  /** M of N confirmation, and misses before deletion */
  confirmHits: number
  confirmWindow: number
  maxMissesTentative: number
  maxMissesConfirmed: number
}

export const DEFAULT_TRACKER: TrackerParams = {
  accelStd: { car: 3, pedestrian: 1.5 },
  measStd: { car: 0.35, pedestrian: 0.15 },
  gate: 13.8,
  confirmHits: 3,
  confirmWindow: 5,
  maxMissesTentative: 2,
  maxMissesConfirmed: 6,
}

/**
 * Multi-object tracker: one constant-velocity Kalman filter per object, global nearest-neighbor
 * association with the Hungarian algorithm on Mahalanobis distances (gated), and track
 * management (tentative → confirmed with M-of-N hits, deleted after consecutive misses).
 */
export class Tracker {
  tracks: Track[] = []
  private nextId = 1

  constructor(public p: TrackerParams = DEFAULT_TRACKER) {}

  step(detections: Detection[], dt: number): void {
    for (const t of this.tracks) this.predict(t, dt)
    // cost: Mahalanobis distance of each detection to each track of the same class
    const cost = this.tracks.map((t) =>
      detections.map((d) => {
        if (d.kind !== t.kind) return Infinity
        const m = this.mahalanobis(t, d.center)
        return m < this.p.gate ? m : Infinity
      }),
    )
    const assignment = hungarian(cost)
    const used = new Set<number>()
    this.tracks.forEach((t, i) => {
      const j = assignment[i]
      t.age++
      if (j >= 0) {
        used.add(j)
        this.update(t, detections[j])
        t.hits++
        t.misses = 0
        if (!t.confirmed && t.hits >= this.p.confirmHits && t.age <= this.p.confirmWindow + 2) t.confirmed = true
      } else t.misses++
    })
    this.tracks = this.tracks.filter((t) => {
      if (!t.confirmed && (t.misses >= this.p.maxMissesTentative || t.age > this.p.confirmWindow + 2)) return false
      return t.misses < this.p.maxMissesConfirmed
    })
    // unassigned detections start tentative tracks
    detections.forEach((d, j) => {
      if (used.has(j)) return
      const r = this.p.measStd[d.kind] ** 2
      this.tracks.push({
        id: this.nextId++,
        kind: d.kind,
        x: [d.center.x, d.center.y, 0, 0],
        // unknown velocity: large initial uncertainty
        P: [
          [r, 0, 0, 0],
          [0, r, 0, 0],
          [0, 0, d.kind === 'car' ? 100 : 4, 0],
          [0, 0, 0, d.kind === 'car' ? 100 : 4],
        ],
        heading: d.heading,
        length: d.length,
        width: d.width,
        hits: 1,
        misses: 0,
        age: 1,
        confirmed: false,
      })
    })
  }

  speed(t: Track): number {
    return Math.hypot(t.x[2], t.x[3])
  }

  private predict(t: Track, dt: number): void {
    const [x, y, vx, vy] = t.x
    t.x = [x + vx * dt, y + vy * dt, vx, vy]
    const q = this.p.accelStd[t.kind] ** 2
    const P = t.P
    // F P Fᵀ with F = [[1,0,dt,0],[0,1,0,dt],[0,0,1,0],[0,0,0,1]]
    const FP = [
      [P[0][0] + dt * P[2][0], P[0][1] + dt * P[2][1], P[0][2] + dt * P[2][2], P[0][3] + dt * P[2][3]],
      [P[1][0] + dt * P[3][0], P[1][1] + dt * P[3][1], P[1][2] + dt * P[3][2], P[1][3] + dt * P[3][3]],
      [...P[2]],
      [...P[3]],
    ]
    const N = FP.map((row) => [row[0] + dt * row[2], row[1] + dt * row[3], row[2], row[3]])
    // white acceleration noise: Q = G Gᵀ q, G = [dt²/2, dt²/2, dt, dt] per axis
    const a = (dt ** 4 / 4) * q
    const b = (dt ** 3 / 2) * q
    const c = dt * dt * q
    N[0][0] += a
    N[1][1] += a
    N[0][2] += b
    N[2][0] += b
    N[1][3] += b
    N[3][1] += b
    N[2][2] += c
    N[3][3] += c
    t.P = N
  }

  private mahalanobis(t: Track, z: Vec2): number {
    const r = this.p.measStd[t.kind] ** 2
    const S = [
      [t.P[0][0] + r, t.P[0][1]],
      [t.P[1][0], t.P[1][1] + r],
    ]
    const det = S[0][0] * S[1][1] - S[0][1] * S[1][0]
    const dx = z.x - t.x[0]
    const dy = z.y - t.x[1]
    return (dx * (S[1][1] * dx - S[0][1] * dy) + dy * (-S[1][0] * dx + S[0][0] * dy)) / det
  }

  private update(t: Track, d: Detection): void {
    const r = this.p.measStd[t.kind] ** 2
    const P = t.P
    const S = [
      [P[0][0] + r, P[0][1]],
      [P[1][0], P[1][1] + r],
    ]
    const det = S[0][0] * S[1][1] - S[0][1] * S[1][0]
    const Si = [
      [S[1][1] / det, -S[0][1] / det],
      [-S[1][0] / det, S[0][0] / det],
    ]
    const K = P.map((row) => [row[0] * Si[0][0] + row[1] * Si[1][0], row[0] * Si[0][1] + row[1] * Si[1][1]])
    const yv = [d.center.x - t.x[0], d.center.y - t.x[1]]
    t.x = t.x.map((v, i) => v + K[i][0] * yv[0] + K[i][1] * yv[1])
    // P = (I - K H) P
    t.P = P.map((row, i) => row.map((v, j) => v - K[i][0] * P[0][j] - K[i][1] * P[1][j]))
    // heading: from the motion when moving, else from the box (resolving its 180° ambiguity)
    const sp = this.speed(t)
    if (sp > 1) t.heading = Math.atan2(t.x[3], t.x[2])
    else {
      const h = Math.abs(wrapAngle(d.heading - t.heading)) > Math.PI / 2 ? d.heading + Math.PI : d.heading
      t.heading = wrapAngle(h)
    }
    t.length = 0.8 * t.length + 0.2 * d.length
    t.width = 0.8 * t.width + 0.2 * d.width
  }
}

/** CLEAR-MOT style metrics against the ground truth. */
export class TrackingMetrics {
  gt = 0
  tp = 0
  misses = 0
  falsePositives = 0
  idSwitches = 0
  posErrSum = 0
  velErrSum = 0
  private lastMatch = new Map<number, number>()

  /**
   * `truth`: objects the sensor saw well this scan (they should be tracked); `others`: every other
   * real object around (a track on one of them, e.g. one hidden for a moment, is not a false positive).
   */
  add(tracks: Track[], truth: { id: number; x: number; y: number; vx: number; vy: number }[], others: Vec2[] = []): void {
    const confirmed = tracks.filter((t) => t.confirmed)
    const cost = truth.map((o) => confirmed.map((t) => {
      const d = Math.hypot(t.x[0] - o.x, t.x[1] - o.y)
      return d < 2.5 ? d : Infinity
    }))
    const assign = truth.length && confirmed.length ? hungarian(cost) : truth.map(() => -1)
    const usedTracks = new Set<number>()
    truth.forEach((o, i) => {
      this.gt++
      const j = assign[i]
      if (j < 0) {
        this.misses++
        return
      }
      const t = confirmed[j]
      usedTracks.add(j)
      this.tp++
      this.posErrSum += Math.hypot(t.x[0] - o.x, t.x[1] - o.y)
      this.velErrSum += Math.hypot(t.x[2] - o.vx, t.x[3] - o.vy)
      const prev = this.lastMatch.get(o.id)
      if (prev !== undefined && prev !== t.id) this.idSwitches++
      this.lastMatch.set(o.id, t.id)
    })
    confirmed.forEach((t, j) => {
      if (usedTracks.has(j)) return
      if (!others.some((o) => Math.hypot(t.x[0] - o.x, t.x[1] - o.y) < 2.5)) this.falsePositives++
    })
  }

  get summary() {
    return {
      mota: this.gt ? 1 - (this.misses + this.falsePositives + this.idSwitches) / this.gt : 1,
      recall: this.gt ? this.tp / this.gt : 1,
      posErr: this.tp ? this.posErrSum / this.tp : 0,
      velErr: this.tp ? this.velErrSum / this.tp : 0,
      idSwitches: this.idSwitches,
      falsePositives: this.falsePositives,
    }
  }
}
