import type { Vec2 } from '../math/vec'
import { stepBicycle, type VehicleParams, type VehicleState } from '../vehicle/bicycle'
import { PathFollower, type PidGains } from './pid'

export interface TwiddleSetup {
  path: Vec2[]
  vehicle: VehicleParams
  speed: number
  lookahead: number
  dt: number
}

/** Mean squared CTE of a noise-free run along the path (penalized if it never finishes). */
export function evaluateGains(gains: PidGains, s: TwiddleSetup): number {
  const a = s.path[0]
  const b = s.path[Math.min(1, s.path.length - 1)]
  const car: VehicleState = { x: a.x, y: a.y, theta: Math.atan2(b.y - a.y, b.x - a.x), speed: 0, steer: 0, odometer: 0 }
  const follower = new PathFollower(s.path, gains, s.lookahead)
  let length = 0
  for (let i = 1; i < s.path.length; i++) length += Math.hypot(s.path[i].x - s.path[i - 1].x, s.path[i].y - s.path[i - 1].y)
  const maxSteps = Math.ceil((length / s.speed / s.dt) * 1.5) + 50
  let err = 0
  let steps = 0
  for (; steps < maxSteps && !follower.finished; steps++) {
    const steer = follower.steer(car, s.dt)
    stepBicycle(car, { speed: s.speed, steer }, s.dt, s.vehicle)
    err += follower.cte * follower.cte
  }
  const mse = err / Math.max(1, steps)
  return follower.finished ? mse : mse + 10
}

/** Coordinate ascent ("twiddle") from the Udacity course. */
export function twiddle(initial: PidGains, s: TwiddleSetup, maxIter = 60, onIter?: (g: PidGains, err: number) => void): PidGains {
  const keys = ['kp', 'kd', 'ki'] as const
  const p = { ...initial }
  const dp: PidGains = { kp: initial.kp * 0.5 || 0.5, kd: initial.kd * 0.5 || 0.5, ki: initial.ki * 0.5 || 0.01 }
  let best = evaluateGains(p, s)
  for (let it = 0; it < maxIter; it++) {
    for (const k of keys) {
      p[k] += dp[k]
      let err = evaluateGains(p, s)
      if (err < best) {
        best = err
        dp[k] *= 1.1
        continue
      }
      p[k] -= 2 * dp[k]
      err = p[k] >= 0 ? evaluateGains(p, s) : Infinity
      if (err < best) {
        best = err
        dp[k] *= 1.1
      } else {
        p[k] += dp[k]
        dp[k] *= 0.9
      }
    }
    onIter?.({ ...p }, best)
  }
  return p
}
