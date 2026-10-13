import { wrapAngle } from '../math/angles'
import type { Pose, Vec2 } from '../math/vec'
import type { PathPose } from '../planning/hybridAStar'
import { PathFollower, type PidGains } from './pid'

export interface Gear {
  dir: number
  /** poses of the segment (the heading is used as reference by the MPC) */
  points: PathPose[]
}

/** Splits a Hybrid A* path wherever the car switches between forward and reverse. */
export function splitByGear(path: PathPose[]): Gear[] {
  const gears: Gear[] = []
  for (let i = 0; i < path.length; i++) {
    const p = path[i]
    const cur = gears[gears.length - 1]
    if (!cur || cur.dir !== p.dir) {
      // the switching pose belongs to both segments
      const prev = path[i - 1]
      gears.push({ dir: p.dir, points: prev ? [{ ...prev, dir: p.dir }] : [] })
    }
    gears[gears.length - 1].points.push({ ...p })
  }
  return gears.filter((g) => g.points.length >= 2)
}

/**
 * Follows a path made of forward and reverse segments with the same PID as the 2D course.
 * In reverse the car is treated as a "virtual car" facing backwards: the PID runs on the
 * pose rotated by 180° and the resulting steering angle is mirrored.
 */
export class GearFollower {
  segment = 0
  private follower: PathFollower

  constructor(
    readonly gears: Gear[],
    private gains: PidGains,
    private lookahead: number,
  ) {
    this.follower = this.make(0)
  }

  get gear(): Gear {
    return this.gears[this.segment]
  }

  get cte(): number {
    return this.follower.cte
  }

  /** index of the current path point inside the current segment */
  get index(): number {
    return this.follower.index
  }

  get done(): boolean {
    return this.segment >= this.gears.length - 1 && this.segmentDone
  }

  segmentDone = false

  /** Remaining distance to the end of the current segment. */
  distanceToSegmentEnd(pose: Vec2): number {
    const pts = this.gear.points
    let d = Math.hypot(pts[this.index + 1].x - pose.x, pts[this.index + 1].y - pose.y)
    for (let i = this.index + 1; i < pts.length - 1; i++) d += Math.hypot(pts[i + 1].x - pts[i].x, pts[i + 1].y - pts[i].y)
    return d
  }

  steer(pose: Pose, dt: number): number {
    const dir = this.gear.dir
    const virtual: Pose = dir > 0 ? pose : { x: pose.x, y: pose.y, theta: wrapAngle(pose.theta + Math.PI) }
    const s = this.follower.steer(virtual, dt)
    const end = this.gear.points[this.gear.points.length - 1]
    this.segmentDone = this.follower.finished || Math.hypot(end.x - pose.x, end.y - pose.y) < 0.12
    return dir > 0 ? s : -s
  }

  /** Moves on to the next segment (after the car has stopped to switch gears). */
  next(): void {
    if (this.segment < this.gears.length - 1) {
      this.segment++
      this.follower = this.make(this.segment)
      this.segmentDone = false
    }
  }

  private make(i: number): PathFollower {
    return new PathFollower(this.gears[i].points, this.gains, this.gears[i].dir > 0 ? this.lookahead : this.lookahead * 0.6)
  }
}
