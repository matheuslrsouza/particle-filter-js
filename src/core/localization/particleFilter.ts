import { wrapAngle } from '../math/angles'
import type { Rng } from '../math/rng'
import type { Pose } from '../math/vec'
import { castRay, type LidarConfig } from '../sensors/lidar'
import type { OccupancyGrid } from '../world/grid'
import { advancePose, type Control } from '../vehicle/bicycle'

export interface FilterParams {
  /** std of the speed actually executed, as a fraction of the commanded speed */
  speedNoise: number
  /** std of the executed steering angle (rad) */
  steerNoise: number
  /** extra pose jitter per second while moving or idle (keeps diversity) */
  poseJitter: number
  /** std of the beam range model (m) */
  sensorSigma: number
  /** inject random particles when the likelihood drops suddenly (augmented MCL) */
  allowRecovery: boolean
}

export interface Estimate extends Pose {
  /** 2x2 position covariance [sxx, sxy, syy] */
  cov: [number, number, number]
  /** effective sample size ratio (0..1] */
  neff: number
}

/**
 * Monte Carlo localization with a beam range model.
 *
 * Differences from the 2D version:
 *  - weights are compared range-by-range in each beam's own frame, so the filter
 *    never peeks at the true heading of the robot;
 *  - log-likelihoods + log-sum-exp avoid the underflow of multiplying 36 tiny gaussians;
 *  - systematic resampling only when the effective sample size drops;
 *  - heading is estimated too (circular mean), plus a proper covariance.
 */
export class ParticleFilter {
  x: Float64Array
  y: Float64Array
  theta: Float64Array
  /** normalized weights */
  w: Float64Array
  private logw: Float64Array
  private wSlow = 0
  private wFast = 0
  estimate: Estimate = { x: 0, y: 0, theta: 0, cov: [0, 0, 0], neff: 1 }

  constructor(
    readonly n: number,
    private grid: OccupancyGrid,
    private rng: Rng,
    public params: FilterParams,
  ) {
    this.x = new Float64Array(n)
    this.y = new Float64Array(n)
    this.theta = new Float64Array(n)
    this.w = new Float64Array(n).fill(1 / n)
    this.logw = new Float64Array(n)
  }

  /** Gaussian cloud around a known starting pose. */
  initAround(p: Pose, posStd: number, thetaStd: number): void {
    for (let i = 0; i < this.n; i++) {
      this.x[i] = this.rng.gaussian(p.x, posStd)
      this.y[i] = this.rng.gaussian(p.y, posStd)
      this.theta[i] = wrapAngle(this.rng.gaussian(p.theta, thetaStd))
    }
    this.resetWeights()
    this.computeEstimate()
  }

  /** Uniform over free space: global localization / kidnapped robot. */
  initGlobal(): void {
    for (let i = 0; i < this.n; i++) this.randomize(i)
    this.resetWeights()
    this.computeEstimate()
  }

  predict(u: Control, dt: number, wheelbase: number): void {
    const { speedNoise, steerNoise, poseJitter } = this.params
    const jitter = poseJitter * Math.sqrt(dt)
    const pose: Pose = { x: 0, y: 0, theta: 0 }
    for (let i = 0; i < this.n; i++) {
      pose.x = this.x[i]
      pose.y = this.y[i]
      pose.theta = this.theta[i]
      const v = this.rng.gaussian(u.speed, Math.abs(u.speed) * speedNoise)
      const s = this.rng.gaussian(u.steer, steerNoise)
      advancePose(pose, v, s, dt, wheelbase)
      this.x[i] = pose.x + this.rng.gaussian(0, jitter)
      this.y[i] = pose.y + this.rng.gaussian(0, jitter)
      this.theta[i] = wrapAngle(pose.theta + this.rng.gaussian(0, jitter * 0.5))
    }
  }

  update(z: Float64Array, lidar: LidarConfig): void {
    const { sensorSigma } = this.params
    const inv2s2 = 1 / (2 * sensorSigma * sensorSigma)
    // robust beam model: cap each beam's penalty at ~3 sigma so one bad beam can't kill a good particle
    const maxPenalty = 4.5
    const walls = this.grid.world.walls
    const nBeams = lidar.angles.length
    let sumLik = 0
    for (let i = 0; i < this.n; i++) {
      const px = this.x[i]
      const py = this.y[i]
      if (!this.grid.isFree({ x: px, y: py })) {
        this.logw[i] = -1e9
        continue
      }
      let ll = 0
      for (let b = 0; b < nBeams; b++) {
        const a = this.theta[i] + lidar.angles[b]
        const expected = castRay(px, py, Math.cos(a), Math.sin(a), walls, lidar.maxRange)
        const e = z[b] - expected
        ll -= Math.min(e * e * inv2s2, maxPenalty)
      }
      this.logw[i] = ll + Math.log(this.w[i])
      sumLik += Math.exp(ll / nBeams)
    }
    this.normalize()
    this.computeEstimate()

    // augmented MCL: compare short and long term average likelihoods. The long term one
    // rises quickly and decays slowly, so it remembers what good tracking looks like.
    const wAvg = sumLik / this.n
    this.wSlow = this.wSlow === 0 ? wAvg : this.wSlow + (wAvg > this.wSlow ? 0.1 : 0.005) * (wAvg - this.wSlow)
    this.wFast = this.wFast === 0 ? wAvg : this.wFast + 0.3 * (wAvg - this.wFast)

    // when every particle is equally bad the weights stay flat (high Neff), so a sudden
    // likelihood drop must also trigger a resample, otherwise nothing is ever injected
    const injectProb = this.params.allowRecovery && this.wSlow > 0 ? Math.max(0, 1 - this.wFast / this.wSlow) * 0.3 : 0
    if (this.estimate.neff < 0.5 || injectProb > 0.02) this.resample(injectProb)
  }

  private resample(injectProb: number): void {
    const n = this.n
    const nx = new Float64Array(n)
    const ny = new Float64Array(n)
    const nt = new Float64Array(n)
    // systematic (low variance) resampling
    const step = 1 / n
    let r = this.rng.next() * step
    let c = this.w[0]
    let j = 0
    for (let i = 0; i < n; i++) {
      const target = r + i * step
      while (target > c && j < n - 1) c += this.w[++j]
      nx[i] = this.x[j]
      ny[i] = this.y[j]
      nt[i] = this.theta[j]
    }
    this.x = nx
    this.y = ny
    this.theta = nt
    for (let i = 0; i < n; i++) if (this.rng.next() < injectProb) this.randomize(i)
    this.resetWeights()
  }

  private randomize(i: number): void {
    const g = this.grid
    for (let tries = 0; tries < 100; tries++) {
      const x = this.rng.range(0, g.world.width)
      const y = this.rng.range(0, g.world.height)
      if (g.isFree({ x, y }, false)) {
        this.x[i] = x
        this.y[i] = y
        break
      }
    }
    this.theta[i] = this.rng.range(-Math.PI, Math.PI)
  }

  private resetWeights(): void {
    this.w.fill(1 / this.n)
  }

  private normalize(): void {
    let max = -Infinity
    for (let i = 0; i < this.n; i++) max = Math.max(max, this.logw[i])
    let sum = 0
    for (let i = 0; i < this.n; i++) {
      this.w[i] = Math.exp(this.logw[i] - max)
      sum += this.w[i]
    }
    for (let i = 0; i < this.n; i++) this.w[i] /= sum
  }

  private computeEstimate(): void {
    let mx = 0
    let my = 0
    let sc = 0
    let ss = 0
    let sw2 = 0
    for (let i = 0; i < this.n; i++) {
      const w = this.w[i]
      mx += w * this.x[i]
      my += w * this.y[i]
      sc += w * Math.cos(this.theta[i])
      ss += w * Math.sin(this.theta[i])
      sw2 += w * w
    }
    let sxx = 0
    let sxy = 0
    let syy = 0
    for (let i = 0; i < this.n; i++) {
      const dx = this.x[i] - mx
      const dy = this.y[i] - my
      sxx += this.w[i] * dx * dx
      sxy += this.w[i] * dx * dy
      syy += this.w[i] * dy * dy
    }
    this.estimate = { x: mx, y: my, theta: Math.atan2(ss, sc), cov: [sxx, sxy, syy], neff: 1 / (sw2 * this.n) }
  }
}
