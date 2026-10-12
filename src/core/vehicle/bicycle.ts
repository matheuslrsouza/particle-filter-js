import { wrapAngle } from '../math/angles'
import { clamp, type Pose } from '../math/vec'

export interface VehicleParams {
  /** distance between front and rear axles (m) */
  wheelbase: number
  /** max front wheel steering angle (rad) */
  maxSteer: number
  /** max steering angle change per second (rad/s) */
  maxSteerRate: number
}

export interface VehicleState extends Pose {
  speed: number
  /** current front wheel angle (rad) */
  steer: number
  /** accumulated travelled distance, used to spin the wheels */
  odometer: number
}

export interface Control {
  speed: number
  steer: number
}

/**
 * Kinematic bicycle model (rear axle reference). Replaces the CTRV model of the
 * 2D version, where the steering angle was mistakenly used as a yaw rate.
 */
export function stepBicycle(s: VehicleState, u: Control, dt: number, p: VehicleParams): void {
  steerToward(s, u.steer, dt, p)
  s.speed = u.speed
  advancePose(s, s.speed, s.steer, dt, p.wheelbase)
  s.odometer += s.speed * dt
}

/** Moves the front wheels towards the commanded angle, respecting limits and steering rate. */
export function steerToward(s: VehicleState, steer: number, dt: number, p: VehicleParams): void {
  const target = clamp(steer, -p.maxSteer, p.maxSteer)
  const maxDelta = p.maxSteerRate * dt
  s.steer += clamp(target - s.steer, -maxDelta, maxDelta)
}

/** Integrates a pose exactly along the arc defined by speed and steering angle. */
export function advancePose(pose: Pose, speed: number, steer: number, dt: number, wheelbase: number): void {
  const d = speed * dt
  const yawRate = (speed / wheelbase) * Math.tan(steer)
  if (Math.abs(yawRate) < 1e-6) {
    pose.x += d * Math.cos(pose.theta)
    pose.y += d * Math.sin(pose.theta)
  } else {
    const r = speed / yawRate
    const t2 = pose.theta + yawRate * dt
    pose.x += r * (Math.sin(t2) - Math.sin(pose.theta))
    pose.y += r * (Math.cos(pose.theta) - Math.cos(t2))
    pose.theta = t2
  }
  pose.theta = wrapAngle(pose.theta)
}
