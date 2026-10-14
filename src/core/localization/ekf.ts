import { wrapAngle } from '../math/angles'
import type { Pose, Vec2 } from '../math/vec'

export interface EkfParams {
  wheelbase: number
  /** odometry noise assumed by the filter */
  speedStd: number
  speedStdRel: number
  steerStd: number
  /** GPS position std assumed by the filter (m) */
  gpsStd: number
  /** Mahalanobis gate (chi-square, 2 dof): fixes beyond it are rejected as outliers */
  gate: number
}

export const DEFAULT_EKF: EkfParams = { wheelbase: 2.8, speedStd: 0.05, speedStdRel: 0.02, steerStd: 0.008, gpsStd: 1.2, gate: 13.8 }

type Mat = number[][]

/**
 * Extended Kalman filter. State: [x, y, θ, steering bias, wheel speed scale error].
 *
 * Predict: the bicycle model driven by the wheel speed and steering angle sensors, corrected by
 * the estimated biases (true speed ≈ measured·(1 − scale), true steer = measured − bias); the
 * sensor noise is propagated through the Jacobian. Update: GPS position fixes, gated by
 * Mahalanobis distance so multipath jumps are rejected.
 *
 * Neither the heading nor the biases are measured: they become observable from the motion
 * (heading drift reveals the steering bias, distance traveled reveals the scale error).
 */
export class Ekf {
  x: number[] = [0, 0, 0, 0, 0]
  P: Mat = diag([25, 25, 0.5, 1e-4, 1e-3])
  lastRejected = false
  accepted = 0
  rejected = 0

  constructor(public p: EkfParams = DEFAULT_EKF) {}

  init(pose: Pose, posStd: number, headingStd: number): void {
    this.x = [pose.x, pose.y, pose.theta, 0, 0]
    // the biases start unknown: up to ~0.6° of steering bias and ~3% of speed scale error
    this.P = diag([posStd ** 2, posStd ** 2, headingStd ** 2, 0.01 ** 2, 0.03 ** 2])
    this.accepted = this.rejected = 0
  }

  get pose(): Pose {
    return { x: this.x[0], y: this.x[1], theta: this.x[2] }
  }

  /** Position covariance [sxx, sxy, syy] (for the uncertainty ellipse). */
  get cov(): [number, number, number] {
    return [this.P[0][0], this.P[0][1], this.P[1][1]]
  }

  get steerBias(): number {
    return this.x[3]
  }

  get speedScale(): number {
    return this.x[4]
  }

  predict(speedMeas: number, steerMeas: number, dt: number): void {
    const [x, y, th, b, s] = this.x
    const L = this.p.wheelbase
    const v = speedMeas * (1 - s)
    const d = steerMeas - b
    const c = Math.cos(th)
    const sn = Math.sin(th)
    const t = Math.tan(d)
    const sec2 = 1 / Math.cos(d) ** 2
    this.x = [x + v * c * dt, y + v * sn * dt, wrapAngle(th + (v / L) * t * dt), b, s]

    // F = ∂f/∂state
    const F = identity(5)
    F[0][2] = -v * sn * dt
    F[0][4] = -speedMeas * c * dt
    F[1][2] = v * c * dt
    F[1][4] = -speedMeas * sn * dt
    F[2][3] = -(v / L) * sec2 * dt
    F[2][4] = -(speedMeas / L) * t * dt
    // G = ∂f/∂[speed, steer] (sensor noise)
    const G: Mat = [
      [(1 - s) * c * dt, 0],
      [(1 - s) * sn * dt, 0],
      [((1 - s) / L) * t * dt, (v / L) * sec2 * dt],
      [0, 0],
      [0, 0],
    ]
    const sv = this.p.speedStd + this.p.speedStdRel * Math.abs(speedMeas)
    const M = diag([sv * sv, this.p.steerStd ** 2])
    // model noise (slip, bumps) and slow drift of the biases
    const Q = diag([0.002 * dt, 0.002 * dt, 0.0001 * dt, 1e-8 * dt, 1e-7 * dt])
    this.P = symmetrize(add(add(mul(mul(F, this.P), transpose(F)), mul(mul(G, M), transpose(G))), Q))
  }

  /** GPS update. Returns false if the fix was rejected by the gate. */
  updateGps(fix: Vec2): boolean {
    const r = this.p.gpsStd ** 2
    const P = this.P
    // H selects x and y: S = H P Hᵀ + R
    const S = [
      [P[0][0] + r, P[0][1]],
      [P[1][0], P[1][1] + r],
    ]
    const det = S[0][0] * S[1][1] - S[0][1] * S[1][0]
    const Si = [
      [S[1][1] / det, -S[0][1] / det],
      [-S[1][0] / det, S[0][0] / det],
    ]
    const yv = [fix.x - this.x[0], fix.y - this.x[1]]
    const d2 = yv[0] * (Si[0][0] * yv[0] + Si[0][1] * yv[1]) + yv[1] * (Si[1][0] * yv[0] + Si[1][1] * yv[1])
    this.lastRejected = d2 > this.p.gate
    if (this.lastRejected) {
      this.rejected++
      return false
    }
    this.accepted++
    // K = P Hᵀ S⁻¹: P Hᵀ is the first two columns of P
    const n = this.x.length
    const K: Mat = []
    for (let i = 0; i < n; i++) K.push([P[i][0] * Si[0][0] + P[i][1] * Si[1][0], P[i][0] * Si[0][1] + P[i][1] * Si[1][1]])
    for (let i = 0; i < n; i++) this.x[i] += K[i][0] * yv[0] + K[i][1] * yv[1]
    this.x[2] = wrapAngle(this.x[2])
    // P = (I - K H) P
    const IKH = identity(n)
    for (let i = 0; i < n; i++) {
      IKH[i][0] -= K[i][0]
      IKH[i][1] -= K[i][1]
    }
    this.P = symmetrize(mul(IKH, P))
    return true
  }
}

function identity(n: number): Mat {
  return Array.from({ length: n }, (_, i) => Array.from({ length: n }, (_, j) => (i === j ? 1 : 0)))
}

function diag(v: number[]): Mat {
  return v.map((d, i) => v.map((_, j) => (i === j ? d : 0)))
}

function mul(a: Mat, b: Mat): Mat {
  const out: Mat = []
  for (let i = 0; i < a.length; i++) {
    const row = new Array(b[0].length).fill(0)
    for (let k = 0; k < b.length; k++) {
      const aik = a[i][k]
      if (aik === 0) continue
      for (let j = 0; j < b[0].length; j++) row[j] += aik * b[k][j]
    }
    out.push(row)
  }
  return out
}

const transpose = (a: Mat): Mat => a[0].map((_, j) => a.map((row) => row[j]))

const add = (a: Mat, b: Mat): Mat => a.map((row, i) => row.map((v, j) => v + b[i][j]))

function symmetrize(P: Mat): Mat {
  for (let i = 0; i < P.length; i++) {
    for (let j = i + 1; j < P.length; j++) P[i][j] = P[j][i] = 0.5 * (P[i][j] + P[j][i])
  }
  return P
}
