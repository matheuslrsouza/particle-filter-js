import { wrapAngle } from '../math/angles'
import { clamp, type Pose, type Vec2 } from '../math/vec'
import type { Gear } from './gearFollower'
import { arcLengths, speedProfile } from './speedProfile'

export interface MpcWeights {
  /** cross-track error (the CTE of the course) */
  lateral: number
  /** being ahead/behind the time reference */
  longitudinal: number
  /** heading error (the eψ of the course) */
  heading: number
  /** (v - v_ref)² */
  speed: number
  accel: number
  /** steering angle magnitude */
  steer: number
  /** steering rate: the (δ_t+1 - δ_t)² smoothness term of the course */
  steerRate: number
  /** multiplier of the state cost at the end of the horizon */
  terminal: number
}

export interface MpcParams {
  /** number of steps N and step length dt (s): the car looks N·dt seconds ahead */
  horizon: number
  dt: number
  wheelbase: number
  maxSteer: number
  maxSteerRate: number
  accel: number
  brake: number
  /** max lateral acceleration used by the speed profile (m/s²) */
  aLat: number
  forwardSpeed: number
  reverseSpeed: number
  weights: MpcWeights
  iterations: number
  /** time between two solves (s); the first control is held in between */
  solveEvery: number
  /** distance (m) over which the speed profile measures curvature (0 = adjacent points) */
  curvatureWindow?: number
}

export const DEFAULT_MPC: MpcParams = {
  horizon: 20,
  dt: 0.1,
  wheelbase: 0.5,
  maxSteer: (35 * Math.PI) / 180,
  maxSteerRate: Math.PI,
  accel: 1.5,
  brake: 2.5,
  aLat: 1.2,
  forwardSpeed: 1.5,
  reverseSpeed: 0.6,
  weights: { lateral: 30, longitudinal: 3, heading: 10, speed: 4, accel: 0.5, steer: 0.2, steerRate: 0.3, terminal: 3 },
  iterations: 10,
  solveEvery: 0.1,
}

/** One point of the time-indexed reference the MPC tracks. */
export interface MpcRef {
  x: number
  y: number
  theta: number
  /** signed speed (negative in reverse) */
  v: number
}

// state x = [px, py, θ, v, δ], control u = [a, ω] (acceleration, steering rate)
const NX = 5
const NU = 2

/**
 * Nonlinear MPC solved with iLQR (iterative LQR): linearize the kinematic bicycle model
 * around the current trajectory, solve the LQR problem backwards in time, roll the new
 * controls forward with a line search, repeat. It plays the role Ipopt + CppAD had in the
 * Udacity MPC project, without any dependency, in a few milliseconds.
 */
export class Ilqr {
  /** controls of the last solution, used to warm-start the next one */
  private U: Float64Array
  /** states of the last solution */
  readonly X: Float64Array
  lastCost = 0
  lastIterations = 0
  lastMillis = 0

  /** horizon length, fixed at construction (the buffers are sized for it) */
  readonly N: number

  constructor(public p: MpcParams) {
    this.N = p.horizon
    this.U = new Float64Array(this.N * NU)
    this.X = new Float64Array((this.N + 1) * NX)
  }

  reset(): void {
    this.U.fill(0)
  }

  solve(x0: number[], ref: MpcRef[]): [number, number] {
    const t0 = performance.now()
    const N = this.N
    // warm start: shift the previous solution by one step
    this.U.copyWithin(0, NU)
    this.U[(N - 1) * NU] = this.U[(N - 2) * NU]
    this.U[(N - 1) * NU + 1] = 0
    for (let k = 0; k < N; k++) this.clampU(this.U, k)

    let cost = this.rollout(x0, this.U, this.X, ref)
    let mu = 1e-3
    const kff = new Float64Array(N * NU)
    const Kfb = new Float64Array(N * NU * NX)
    const Unew = new Float64Array(N * NU)
    const Xnew = new Float64Array((N + 1) * NX)
    let it = 0
    for (; it < this.p.iterations; it++) {
      if (!this.backward(ref, mu, kff, Kfb)) {
        mu = Math.min(mu * 10, 1e6)
        continue
      }
      let improved = false
      for (const alpha of [1, 0.5, 0.25, 0.1]) {
        const c = this.forward(x0, alpha, kff, Kfb, Unew, Xnew, ref)
        if (c < cost) {
          const gain = cost - c
          cost = c
          this.U.set(Unew)
          this.X.set(Xnew)
          improved = true
          mu = Math.max(mu / 4, 1e-6)
          if (gain < 1e-4 * Math.max(1, cost)) it = this.p.iterations
          break
        }
      }
      if (!improved) mu = Math.min(mu * 10, 1e6)
    }
    this.lastCost = cost
    this.lastIterations = it
    this.lastMillis = performance.now() - t0
    return [this.U[0], this.U[1]]
  }

  /** Predicted poses of the last solution (for drawing). */
  predicted(): Pose[] {
    const out: Pose[] = []
    for (let k = 0; k <= this.N; k++) out.push({ x: this.X[k * NX], y: this.X[k * NX + 1], theta: this.X[k * NX + 2] })
    return out
  }

  private clampU(U: Float64Array, k: number) {
    U[k * NU] = clamp(U[k * NU], -this.p.brake, this.p.accel)
    U[k * NU + 1] = clamp(U[k * NU + 1], -this.p.maxSteerRate, this.p.maxSteerRate)
  }

  private step(X: Float64Array, k: number, a: number, w: number) {
    const { dt, wheelbase, maxSteer } = this.p
    const i = k * NX
    const j = i + NX
    const v = X[i + 3]
    const th = X[i + 2]
    X[j] = X[i] + v * Math.cos(th) * dt
    X[j + 1] = X[i + 1] + v * Math.sin(th) * dt
    X[j + 2] = th + (v / wheelbase) * Math.tan(X[i + 4]) * dt
    X[j + 3] = v + a * dt
    X[j + 4] = clamp(X[i + 4] + w * dt, -maxSteer, maxSteer)
  }

  private rollout(x0: number[], U: Float64Array, X: Float64Array, ref: MpcRef[]): number {
    for (let n = 0; n < NX; n++) X[n] = x0[n]
    let cost = 0
    for (let k = 0; k < this.N; k++) {
      this.step(X, k, U[k * NU], U[k * NU + 1])
      cost += this.controlCost(U[k * NU], U[k * NU + 1]) + this.stateCost(X, k + 1, ref[k + 1], null, null)
    }
    return cost
  }

  private forward(x0: number[], alpha: number, kff: Float64Array, Kfb: Float64Array, U: Float64Array, X: Float64Array, ref: MpcRef[]): number {
    for (let n = 0; n < NX; n++) X[n] = x0[n]
    let cost = 0
    const dx = new Float64Array(NX)
    for (let k = 0; k < this.N; k++) {
      for (let n = 0; n < NX; n++) dx[n] = X[k * NX + n] - this.X[k * NX + n]
      dx[2] = wrapAngle(dx[2])
      for (let m = 0; m < NU; m++) {
        let du = alpha * kff[k * NU + m]
        for (let n = 0; n < NX; n++) du += Kfb[(k * NU + m) * NX + n] * dx[n]
        U[k * NU + m] = this.U[k * NU + m] + du
      }
      this.clampU(U, k)
      this.step(X, k, U[k * NU], U[k * NU + 1])
      cost += this.controlCost(U[k * NU], U[k * NU + 1]) + this.stateCost(X, k + 1, ref[k + 1], null, null)
    }
    return cost
  }

  private controlCost(a: number, w: number): number {
    const W = this.p.weights
    return W.accel * a * a + W.steerRate * w * w
  }

  /** Tracking cost of state k; fills the gradient and Hessian when given. */
  private stateCost(X: Float64Array, k: number, r: MpcRef, lx: Float64Array | null, lxx: Float64Array | null): number {
    const W = this.p.weights
    const scale = k === this.N ? W.terminal : 1
    const i = k * NX
    const c = Math.cos(r.theta)
    const s = Math.sin(r.theta)
    const dx = X[i] - r.x
    const dy = X[i + 1] - r.y
    const lon = dx * c + dy * s
    const lat = -dx * s + dy * c
    const eth = wrapAngle(X[i + 2] - r.theta)
    const ev = X[i + 3] - r.v
    const d = X[i + 4]
    // soft wall near the steering limit (the clamp in the model has no gradient)
    const dLim = 0.9 * this.p.maxSteer
    const over = Math.max(0, Math.abs(d) - dLim)
    const cost = scale * (W.lateral * lat * lat + W.longitudinal * lon * lon + W.heading * eth * eth + W.speed * ev * ev) + W.steer * d * d + 50 * over * over
    if (lx && lxx) {
      lx.fill(0)
      lxx.fill(0)
      const wl = scale * W.lateral
      const wo = scale * W.longitudinal
      lx[0] = 2 * (wl * lat * -s + wo * lon * c)
      lx[1] = 2 * (wl * lat * c + wo * lon * s)
      lx[2] = 2 * scale * W.heading * eth
      lx[3] = 2 * scale * W.speed * ev
      lx[4] = 2 * W.steer * d + 100 * over * Math.sign(d)
      lxx[0] = 2 * (wl * s * s + wo * c * c)
      lxx[1] = lxx[NX] = 2 * (-wl * s * c + wo * c * s)
      lxx[NX + 1] = 2 * (wl * c * c + wo * s * s)
      lxx[2 * NX + 2] = 2 * scale * W.heading
      lxx[3 * NX + 3] = 2 * scale * W.speed
      lxx[4 * NX + 4] = 2 * W.steer + (over > 0 ? 100 : 0)
    }
    return cost
  }

  /** Riccati-like backward pass. Returns false if Quu is not positive definite. */
  private backward(ref: MpcRef[], mu: number, kff: Float64Array, Kfb: Float64Array): boolean {
    const N = this.N
    const { dt, wheelbase: L } = this.p
    const W = this.p.weights
    const Vx = new Float64Array(NX)
    const Vxx = new Float64Array(NX * NX)
    this.stateCost(this.X, N, ref[N], Vx, Vxx)
    const lx = new Float64Array(NX)
    const lxx = new Float64Array(NX * NX)
    const A = new Float64Array(NX * NX)
    const B = new Float64Array(NX * NU)
    const VA = new Float64Array(NX * NX)
    const VB = new Float64Array(NX * NU)
    for (let k = N - 1; k >= 0; k--) {
      const i = k * NX
      const th = this.X[i + 2]
      const v = this.X[i + 3]
      const d = this.X[i + 4]
      // linearized dynamics x_{k+1} ≈ A x_k + B u_k
      A.fill(0)
      for (let n = 0; n < NX; n++) A[n * NX + n] = 1
      A[0 * NX + 2] = -v * Math.sin(th) * dt
      A[0 * NX + 3] = Math.cos(th) * dt
      A[1 * NX + 2] = v * Math.cos(th) * dt
      A[1 * NX + 3] = Math.sin(th) * dt
      A[2 * NX + 3] = (Math.tan(d) / L) * dt
      A[2 * NX + 4] = (v / (L * Math.cos(d) ** 2)) * dt
      B.fill(0)
      B[3 * NU + 0] = dt
      B[4 * NU + 1] = dt
      // stage cost (state cost of x_k is applied when k ≥ 1)
      if (k > 0) this.stateCost(this.X, k, ref[k], lx, lxx)
      else {
        lx.fill(0)
        lxx.fill(0)
      }
      const a = this.U[k * NU]
      const w = this.U[k * NU + 1]
      const lu0 = 2 * W.accel * a
      const lu1 = 2 * W.steerRate * w

      // VA = (Vxx + mu I) A, VB = (Vxx + mu I) B
      for (let r = 0; r < NX; r++) {
        for (let c = 0; c < NX; c++) {
          let sum = 0
          for (let m = 0; m < NX; m++) sum += (Vxx[r * NX + m] + (r === m ? mu : 0)) * A[m * NX + c]
          VA[r * NX + c] = sum
        }
        for (let c = 0; c < NU; c++) {
          let sum = 0
          for (let m = 0; m < NX; m++) sum += (Vxx[r * NX + m] + (r === m ? mu : 0)) * B[m * NU + c]
          VB[r * NU + c] = sum
        }
      }
      // Q terms
      const Qx = new Float64Array(NX)
      const Qxx = new Float64Array(NX * NX)
      for (let r = 0; r < NX; r++) {
        let sum = lx[r]
        for (let m = 0; m < NX; m++) sum += A[m * NX + r] * Vx[m]
        Qx[r] = sum
        for (let c = 0; c < NX; c++) {
          // Aᵀ Vxx A, without the regularization (μ only conditions Quu and Qux)
          let q = lxx[r * NX + c]
          for (let m = 0; m < NX; m++) q += A[m * NX + r] * (VA[m * NX + c] - mu * A[m * NX + c])
          Qxx[r * NX + c] = q
        }
      }
      const Qu = [lu0, lu1]
      for (let c = 0; c < NU; c++) for (let m = 0; m < NX; m++) Qu[c] += B[m * NU + c] * Vx[m]
      const Quu = [2 * W.accel, 0, 0, 2 * W.steerRate]
      for (let r = 0; r < NU; r++) for (let c = 0; c < NU; c++) for (let m = 0; m < NX; m++) Quu[r * NU + c] += B[m * NU + r] * VB[m * NU + c]
      const Qux = new Float64Array(NU * NX)
      for (let r = 0; r < NU; r++) for (let c = 0; c < NX; c++) {
        let sum = 0
        for (let m = 0; m < NX; m++) sum += B[m * NU + r] * VA[m * NX + c]
        Qux[r * NX + c] = sum
      }
      // invert the 2x2 Quu
      const det = Quu[0] * Quu[3] - Quu[1] * Quu[2]
      if (Quu[0] <= 0 || det <= 1e-12) return false
      const inv = [Quu[3] / det, -Quu[1] / det, -Quu[2] / det, Quu[0] / det]
      for (let r = 0; r < NU; r++) {
        kff[k * NU + r] = -(inv[r * NU] * Qu[0] + inv[r * NU + 1] * Qu[1])
        for (let c = 0; c < NX; c++) Kfb[(k * NU + r) * NX + c] = -(inv[r * NU] * Qux[c] + inv[r * NU + 1] * Qux[NX + c])
      }
      // value function update
      for (let r = 0; r < NX; r++) {
        let sum = Qx[r]
        for (let m = 0; m < NU; m++) {
          const Kmr = Kfb[(k * NU + m) * NX + r]
          let quuk = 0
          for (let q = 0; q < NU; q++) quuk += Quu[m * NU + q] * kff[k * NU + q]
          sum += Kmr * quuk + Kmr * Qu[m] + Qux[m * NX + r] * kff[k * NU + m]
        }
        Vx[r] = sum
      }
      for (let r = 0; r < NX; r++) {
        for (let c = 0; c < NX; c++) {
          let sum = Qxx[r * NX + c]
          for (let m = 0; m < NU; m++) {
            const Kmr = Kfb[(k * NU + m) * NX + r]
            let quuK = 0
            for (let q = 0; q < NU; q++) quuK += Quu[m * NU + q] * Kfb[(k * NU + q) * NX + c]
            sum += Kmr * quuK + Kmr * Qux[m * NX + c] + Qux[m * NX + r] * Kfb[(k * NU + m) * NX + c]
          }
          Vxx[r * NX + c] = sum
        }
      }
      for (let r = 0; r < NX; r++) for (let c = r + 1; c < NX; c++) Vxx[r * NX + c] = Vxx[c * NX + r] = 0.5 * (Vxx[r * NX + c] + Vxx[c * NX + r])
    }
    return true
  }
}

/**
 * Tracks a Hybrid A* path (forward and reverse segments) with the MPC. For each segment a
 * speed profile is computed; every solve, a time-indexed reference is sampled along the
 * path by "driving" it ideally from the current speed under the profile and acceleration
 * limits. Same interface as the PID GearFollower, so the exploration can use either.
 */
export class MpcFollower {
  segment = 0
  index = 0
  cte = 0
  segmentDone = false
  /** speed of the reference one step ahead (for the chart) */
  refSpeed = 0
  reference: MpcRef[] = []
  readonly solver: Ilqr
  private profiles: number[][]
  private arcs: number[][]
  private u: [number, number] = [0, 0]
  private clock = Infinity

  constructor(
    readonly gears: Gear[],
    public p: MpcParams = DEFAULT_MPC,
  ) {
    this.solver = new Ilqr(p)
    this.arcs = gears.map((g) => arcLengths(g.points))
    this.profiles = gears.map((g) =>
      speedProfile(
        g.points,
        { vMax: g.dir > 0 ? p.forwardSpeed : p.reverseSpeed, aLat: p.aLat, accel: p.accel, brake: p.brake, curvatureWindow: p.curvatureWindow },
        g.speedLimit,
      ),
    )
  }

  get gear(): Gear {
    return this.gears[this.segment]
  }

  get done(): boolean {
    return this.segment >= this.gears.length - 1 && this.segmentDone
  }

  distanceToSegmentEnd(pose: Vec2): number {
    const s = this.arcs[this.segment]
    return s[s.length - 1] - this.progress(pose)
  }

  next(): void {
    if (this.segment < this.gears.length - 1) {
      this.segment++
      this.index = 0
      this.segmentDone = false
      this.solver.reset()
      this.clock = Infinity
    }
  }

  /** Returns [acceleration, steering rate] for the current tick. */
  control(est: Pose, speed: number, steer: number, dt: number): [number, number] {
    const pts = this.gear.points
    // closest point, never going backwards along the segment
    let best = this.index
    let bestD = Infinity
    for (let i = this.index; i < Math.min(pts.length, this.index + 40); i++) {
      const d = Math.hypot(pts[i].x - est.x, pts[i].y - est.y)
      if (d < bestD) {
        bestD = d
        best = i
      }
    }
    this.index = Math.min(best, pts.length - 2)
    const a = pts[this.index]
    const b = pts[this.index + 1]
    const len = Math.hypot(b.x - a.x, b.y - a.y) || 1
    this.cte = ((est.y - a.y) * (b.x - a.x) - (est.x - a.x) * (b.y - a.y)) / len

    const end = pts[pts.length - 1]
    const toEnd = Math.hypot(end.x - est.x, end.y - est.y)
    this.segmentDone = Math.abs(speed) < 0.1 && (toEnd < 0.2 || (this.index >= pts.length - 3 && toEnd < 0.5))

    this.clock += dt
    if (this.clock >= this.p.solveEvery - 1e-9) {
      this.clock = 0
      this.reference = this.buildReference(est, speed)
      this.refSpeed = this.reference[1].v
      this.u = this.solver.solve([est.x, est.y, est.theta, speed, steer], this.reference)
    }
    return this.u
  }

  predicted(): Pose[] {
    return this.solver.predicted()
  }

  /** Arc length of the projection of `pose` on the current segment. */
  private progress(pose: Vec2): number {
    const pts = this.gear.points
    const s = this.arcs[this.segment]
    const a = pts[this.index]
    const b = pts[Math.min(this.index + 1, pts.length - 1)]
    const len = Math.hypot(b.x - a.x, b.y - a.y)
    const t = len > 0 ? clamp(((pose.x - a.x) * (b.x - a.x) + (pose.y - a.y) * (b.y - a.y)) / (len * len), 0, 1) : 0
    return s[this.index] + t * len
  }

  private buildReference(est: Pose, speed: number): MpcRef[] {
    const { dt, accel } = this.p
    const N = this.solver.N
    const dir = this.gear.dir
    const total = this.arcs[this.segment][this.arcs[this.segment].length - 1]
    let s = this.progress(est)
    // current speed along the gear direction (0 if the car still rolls the other way)
    let v = Math.max(0, speed * dir)
    const ref: MpcRef[] = []
    for (let k = 0; k <= N; k++) {
      const p = this.pointAt(s)
      ref.push({ x: p.x, y: p.y, theta: p.theta, v: dir * v })
      const limit = this.profileAt(s)
      v = Math.min(limit, v + accel * dt)
      s = Math.min(total, s + v * dt)
    }
    return ref
  }

  private locate(s: number): [number, number] {
    const arc = this.arcs[this.segment]
    let i = 0
    while (i < arc.length - 2 && arc[i + 1] < s) i++
    const span = arc[i + 1] - arc[i]
    return [i, span > 0 ? clamp((s - arc[i]) / span, 0, 1) : 0]
  }

  private pointAt(s: number): Pose {
    const pts = this.gear.points
    const [i, t] = this.locate(s)
    const a = pts[i]
    const b = pts[i + 1]
    return { x: a.x + (b.x - a.x) * t, y: a.y + (b.y - a.y) * t, theta: a.theta + wrapAngle(b.theta - a.theta) * t }
  }

  private profileAt(s: number): number {
    const prof = this.profiles[this.segment]
    const [i, t] = this.locate(s)
    return prof[i] + (prof[i + 1] - prof[i]) * t
  }
}
