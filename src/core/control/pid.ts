import { wrapAngle } from '../math/angles'
import { clamp, type Pose, type Vec2 } from '../math/vec'

export interface PidGains {
  kp: number
  ki: number
  kd: number
}

/**
 * PID steering on the cross-track error (CTE) to the current path segment.
 *
 * Fixes from the 2D version: the D and I terms are summed (they were multiplied),
 * derivative and integral are time-based, and the integral has anti-windup.
 * The CTE is measured at a point `lookahead` meters ahead of the rear axle, which
 * damps the oscillation of a pure rear-axle PID. When the car faces away from the
 * segment (e.g. the plan starts behind it) it first turns at full lock towards it.
 */
export class PathFollower {
  index = 0
  cte = 0
  private prevCte = 0
  private intCte = 0
  private started = false
  finished = false

  constructor(
    readonly path: Vec2[],
    public gains: PidGains,
    public lookahead = 0.5,
    private intLimit = 2,
  ) {}

  steer(pose: Pose, dt: number): number {
    const px = pose.x + this.lookahead * Math.cos(pose.theta)
    const py = pose.y + this.lookahead * Math.sin(pose.theta)
    const path = this.path
    let dx = 0
    let dy = 0
    let rx = 0
    let ry = 0
    for (;;) {
      const a = path[this.index]
      const b = path[this.index + 1]
      dx = b.x - a.x
      dy = b.y - a.y
      rx = px - a.x
      ry = py - a.y
      const len2 = dx * dx + dy * dy
      const u = len2 > 0 ? (rx * dx + ry * dy) / len2 : 1
      if (u <= 1) break
      if (this.index >= path.length - 2) {
        this.finished = true
        break
      }
      this.index++
    }
    const len = Math.hypot(dx, dy) || 1
    this.cte = (ry * dx - rx * dy) / len

    const headingError = wrapAngle(Math.atan2(dy, dx) - pose.theta)
    if (Math.abs(headingError) > Math.PI / 2) {
      this.started = false
      this.intCte = 0
      return Math.sign(headingError) * 10
    }

    const diff = this.started ? (this.cte - this.prevCte) / dt : 0
    this.intCte = clamp(this.intCte + this.cte * dt, -this.intLimit, this.intLimit)
    this.prevCte = this.cte
    this.started = true

    const { kp, ki, kd } = this.gains
    return -kp * this.cte - kd * diff - ki * this.intCte
  }
}
