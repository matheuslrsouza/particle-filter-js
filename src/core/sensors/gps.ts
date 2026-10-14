import type { Rng } from '../math/rng'
import type { Pose, Vec2 } from '../math/vec'

export interface GpsParams {
  /** fixes per second */
  rate: number
  /** slowly wandering error (Gauss-Markov): std (m) and correlation time (s) */
  biasStd: number
  biasTau: number
  /** white noise on top (m) */
  noiseStd: number
  /** probability of a multipath outlier per fix, and its size (m) */
  outlierRate: number
  outlierSize: number
}

export type GpsGrade = 'rtk' | 'dual' | 'consumer'

/** Receiver grades: RTK (survey / robotaxi development), dual-frequency (modern cars), consumer (phones). */
export const GPS_GRADES: Record<GpsGrade, GpsParams> = {
  rtk: { rate: 10, biasStd: 0.02, biasTau: 30, noiseStd: 0.01, outlierRate: 0.002, outlierSize: 5 },
  dual: { rate: 5, biasStd: 0.35, biasTau: 20, noiseStd: 0.15, outlierRate: 0.01, outlierSize: 8 },
  consumer: { rate: 5, biasStd: 1.0, biasTau: 8, noiseStd: 0.5, outlierRate: 0.02, outlierSize: 12 },
}

/** what the EKF assumes for each grade (the wandering bias is folded into the position std) */
export const GPS_FILTER_STD: Record<GpsGrade, number> = { rtk: 0.03, dual: 0.4, consumer: 1.2 }

export const DEFAULT_GPS: GpsParams = GPS_GRADES.dual

/**
 * A consumer-grade GPS. Its error is mostly a slowly wandering bias (first-order Gauss-Markov
 * process), like real receivers, plus white noise and occasional multipath jumps near buildings.
 */
export class Gps {
  private bias: Vec2 = { x: 0, y: 0 }
  private clock = 0
  lastFix: (Vec2 & { outlier: boolean }) | null = null

  constructor(
    public p: GpsParams,
    private rng: Rng,
  ) {}

  /** Advances time; returns a fix when one is due. */
  step(truth: Pose, dt: number): (Vec2 & { outlier: boolean }) | null {
    // Gauss-Markov: b' = -b/τ + w, discretized so the stationary std is biasStd
    const a = Math.exp(-dt / this.p.biasTau)
    const s = this.p.biasStd * Math.sqrt(1 - a * a)
    this.bias = { x: a * this.bias.x + this.rng.gaussian(0, s), y: a * this.bias.y + this.rng.gaussian(0, s) }
    this.clock += dt
    if (this.clock < 1 / this.p.rate - 1e-9) return null
    this.clock = 0
    const outlier = this.rng.next() < this.p.outlierRate
    const jump = outlier ? this.p.outlierSize * (0.5 + this.rng.next()) : 0
    const dir = this.rng.range(-Math.PI, Math.PI)
    this.lastFix = {
      x: truth.x + this.bias.x + this.rng.gaussian(0, this.p.noiseStd) + Math.cos(dir) * jump,
      y: truth.y + this.bias.y + this.rng.gaussian(0, this.p.noiseStd) + Math.sin(dir) * jump,
      outlier,
    }
    return this.lastFix
  }
}

export interface OdometryParams {
  /** wheel speed scale error (e.g. 0.01 = reads 1% high) and noise (m/s) */
  speedScale: number
  speedNoise: number
  /** steering angle sensor bias and noise (rad) */
  steerBias: number
  steerNoise: number
}

export const DEFAULT_ODOMETRY: OdometryParams = { speedScale: 0.015, speedNoise: 0.05, steerBias: 0.004, steerNoise: 0.004 }

/** Wheel speed and steering angle as the car's own sensors read them. */
export function readOdometry(speed: number, steer: number, p: OdometryParams, rng: Rng): { speed: number; steer: number } {
  return {
    speed: speed * (1 + p.speedScale) + (speed !== 0 ? rng.gaussian(0, p.speedNoise) : 0),
    steer: steer + p.steerBias + rng.gaussian(0, p.steerNoise),
  }
}
