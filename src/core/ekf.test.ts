import { describe, expect, it } from 'vitest'
import cityJson from '../maps/city-mountain-view.json'
import type { CityDef } from './city/cityDef'
import { CitySim } from './city/citySim'
import { Ekf } from './localization/ekf'
import { wrapAngle } from './math/angles'
import { Rng } from './math/rng'
import { Gps, readOdometry, DEFAULT_ODOMETRY } from './sensors/gps'
import { advancePose } from './vehicle/bicycle'

/** Drives a gentle S-curve and feeds the filter; returns the position errors over time. */
function run(opts: { ekfGpsStd: number; gps: ConstructorParameters<typeof Gps>[0]; headingError?: number; seconds?: number; useGps?: boolean }) {
  const rng = new Rng(7)
  const truth = { x: 0, y: 0, theta: 0 }
  const ekf = new Ekf({ wheelbase: 2.8, speedStd: 0.05, speedStdRel: 0.02, steerStd: 0.008, gpsStd: opts.ekfGpsStd, gate: 13.8 })
  ekf.init({ ...truth, theta: truth.theta + (opts.headingError ?? 0) }, 1, 0.5)
  const gps = new Gps(opts.gps, rng)
  const errors: number[] = []
  const headingErrors: number[] = []
  const dt = 0.02
  for (let t = 0; t < (opts.seconds ?? 60); t += dt) {
    const steer = 0.05 * Math.sin(t / 4)
    advancePose(truth, 10, steer, dt, 2.8)
    const odo = readOdometry(10, steer, DEFAULT_ODOMETRY, rng)
    ekf.predict(odo.speed, odo.steer, dt)
    const fix = gps.step(truth, dt)
    if (fix && opts.useGps !== false) ekf.updateGps(fix)
    errors.push(Math.hypot(ekf.x[0] - truth.x, ekf.x[1] - truth.y))
    headingErrors.push(Math.abs(wrapAngle(ekf.x[2] - truth.theta)))
  }
  return { errors, headingErrors, ekf }
}

const rms = (a: number[]) => Math.sqrt(a.reduce((s, v) => s + v * v, 0) / a.length)

describe('EKF (GPS + odometry)', () => {
  it('filters white GPS noise well below the noise itself', () => {
    const { errors } = run({ ekfGpsStd: 1, gps: { rate: 5, biasStd: 0, biasTau: 1, noiseStd: 1, outlierRate: 0, outlierSize: 0 } })
    expect(rms(errors.slice(500))).toBeLessThan(0.5)
  })

  it('learns the biases of the steering and wheel speed sensors', () => {
    const { ekf } = run({ ekfGpsStd: 1, gps: { rate: 5, biasStd: 0, biasTau: 1, noiseStd: 1, outlierRate: 0, outlierSize: 0 }, seconds: 90 })
    expect(ekf.steerBias).toBeCloseTo(DEFAULT_ODOMETRY.steerBias, 3)
    expect(Math.abs(ekf.speedScale - DEFAULT_ODOMETRY.speedScale)).toBeLessThan(0.005)
  })

  it('learns the heading from the motion, which it never measures', () => {
    const { headingErrors } = run({ ekfGpsStd: 0.5, gps: { rate: 5, biasStd: 0, biasTau: 1, noiseStd: 0.5, outlierRate: 0, outlierSize: 0 }, headingError: 0.3, seconds: 20 })
    expect(headingErrors[0]).toBeGreaterThan(0.25)
    expect(headingErrors[headingErrors.length - 1]).toBeLessThan(0.05)
  })

  it('rejects multipath jumps with the Mahalanobis gate', () => {
    const { ekf, errors } = run({ ekfGpsStd: 0.5, gps: { rate: 5, biasStd: 0, biasTau: 1, noiseStd: 0.3, outlierRate: 0.1, outlierSize: 15 } })
    expect(ekf.rejected).toBeGreaterThan(10)
    expect(Math.max(...errors.slice(500))).toBeLessThan(1.5)
  })

  it('without GPS, dead reckoning drifts; with it the error stays bounded', () => {
    const gps = { rate: 5, biasStd: 0.35, biasTau: 20, noiseStd: 0.15, outlierRate: 0, outlierSize: 0 }
    const blind = run({ ekfGpsStd: 0.4, gps, useGps: false, seconds: 90 })
    const fused = run({ ekfGpsStd: 0.4, gps, seconds: 90 })
    expect(blind.errors[blind.errors.length - 1]).toBeGreaterThan(5)
    expect(Math.max(...fused.errors.slice(500))).toBeLessThan(1.5)
  })
})

describe('driving on the EKF estimate', () => {
  it('reaches the destination with an RTK GPS, staying in its lane', () => {
    const sim = new CitySim(cityJson as unknown as CityDef, 1)
    sim.gpsGrade = 'rtk'
    expect(sim.randomDestination()).toBe(true)
    let sum = 0
    let n = 0
    for (let i = 0; i < 50 * 300 && sim.phase === 'driving'; i++) {
      sim.tick()
      const t = sim.telemetry[sim.telemetry.length - 1]
      if (t && i > 100 && i % 5 === 0) {
        sum += Math.abs(t.lateral)
        n++
      }
    }
    expect(sim.phase).toBe('arrived')
    expect(sum / n).toBeLessThan(0.25)
  }, 60_000)
})
