import { describe, expect, it } from 'vitest'
import { DEFAULT_MPC, MpcFollower } from './control/mpc'
import { speedProfile } from './control/speedProfile'
import { advancePose, type VehicleState } from './vehicle/bicycle'
import { clamp } from './math/vec'
import type { Gear } from './control/gearFollower'

const line = (x0: number, y0: number, x1: number, y1: number, dir: number, step = 0.25): Gear => {
  const n = Math.round(Math.hypot(x1 - x0, y1 - y0) / step)
  const heading = Math.atan2(y1 - y0, x1 - x0) + (dir < 0 ? Math.PI : 0)
  return { dir, points: Array.from({ length: n + 1 }, (_, i) => ({ x: x0 + ((x1 - x0) * i) / n, y: y0 + ((y1 - y0) * i) / n, theta: heading, dir })) }
}

/** Drives the follower with the same actuator model as the exploration (steering rate + acceleration). */
function drive(f: MpcFollower, car: VehicleState, seconds: number, onStep?: (car: VehicleState) => void) {
  const dt = 0.02
  for (let t = 0; t < seconds && !f.segmentDone; t += dt) {
    const [a, w] = f.control(car, car.speed, car.steer, dt)
    car.steer = clamp(car.steer + w * dt, -DEFAULT_MPC.maxSteer, DEFAULT_MPC.maxSteer)
    car.speed += a * dt
    advancePose(car, car.speed, car.steer, dt, DEFAULT_MPC.wheelbase)
    onStep?.(car)
  }
}

describe('speed profile', () => {
  it('slows down in curves and stops at the end', () => {
    const pts = [
      ...Array.from({ length: 20 }, (_, i) => ({ x: i * 0.25, y: 0 })),
      ...Array.from({ length: 13 }, (_, i) => ({ x: 5 + Math.sin((i / 12) * Math.PI) * 1, y: 1 - Math.cos((i / 12) * Math.PI) * 1 })),
    ]
    const v = speedProfile(pts, { vMax: 1.5, aLat: 1, accel: 1.5, brake: 2 })
    expect(v[v.length - 1]).toBe(0)
    expect(Math.max(...v.slice(22, 30))).toBeLessThanOrEqual(Math.sqrt(1 / 0.9) + 1e-6) // radius 1 m → ≤ ~1 m/s
    expect(v[5]).toBeCloseTo(1.5)
  })
})

describe('MPC (iLQR)', () => {
  it('converges onto a straight line from 0.5 m off, within the actuator limits', () => {
    const f = new MpcFollower([line(0, 0, 30, 0, 1)])
    const car: VehicleState = { x: 0, y: 0.5, theta: 0, speed: 0, steer: 0, odometer: 0 }
    let maxSteer = 0
    drive(f, car, 8, (c) => (maxSteer = Math.max(maxSteer, Math.abs(c.steer))))
    expect(Math.abs(car.y)).toBeLessThan(0.05)
    expect(car.speed).toBeGreaterThan(1.2)
    expect(maxSteer).toBeLessThanOrEqual(DEFAULT_MPC.maxSteer + 1e-9)
  })

  it('tracks a 2 m circle', () => {
    const R = 2
    const pts = Array.from({ length: 41 }, (_, i) => {
      const a = (i / 40) * Math.PI * 1.5 - Math.PI / 2
      return { x: R * Math.cos(a), y: R + R * Math.sin(a), theta: a + Math.PI / 2, dir: 1 }
    })
    const f = new MpcFollower([{ dir: 1, points: pts }])
    const car: VehicleState = { x: 0, y: 0, theta: 0, speed: 0, steer: 0, odometer: 0 }
    let sum = 0
    let n = 0
    drive(f, car, 12, () => {
      sum += Math.abs(Math.hypot(car.x, car.y - R) - R)
      n++
    })
    expect(sum / n).toBeLessThan(0.05)
  })

  it('reverses along a segment and stops at its end', () => {
    const f = new MpcFollower([line(5, 0, 0, 0, -1)])
    const car: VehicleState = { x: 5, y: 0.2, theta: Math.PI, speed: 0, steer: 0, odometer: 0 }
    let minSpeed = 0
    drive(f, car, 15, (c) => (minSpeed = Math.min(minSpeed, c.speed)))
    expect(minSpeed).toBeLessThan(-0.4)
    expect(f.segmentDone).toBe(true)
    expect(Math.hypot(car.x, car.y)).toBeLessThan(0.3)
  })

  it('solves in a few milliseconds', () => {
    const f = new MpcFollower([line(0, 0, 30, 0, 1)])
    const car: VehicleState = { x: 0, y: 0.5, theta: 0.2, speed: 1, steer: 0, odometer: 0 }
    drive(f, car, 1)
    const t = performance.now()
    for (let i = 0; i < 50; i++) f.solver.solve([car.x, car.y, car.theta, car.speed, car.steer], f.reference)
    expect((performance.now() - t) / 50).toBeLessThan(10)
  })
})
