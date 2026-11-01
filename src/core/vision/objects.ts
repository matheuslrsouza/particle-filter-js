import type { Pose, Vec2 } from '../math/vec'

/**
 * Objects seen by the camera, placed on the SLAM map: a detection's box gives the bearing (the
 * camera's horizontal field of view maps pixels to angles), the lidar points in that bearing give
 * the range, and the robot's pose turns it into a map position. Repeated sightings of the same kind
 * of object near the same place are merged; an object counts once it has been seen a few times.
 */

export interface Detection {
  label: string
  conf: number
  /** [x1, y1, x2, y2] in pixels */
  box: number[]
}

export interface CameraParams {
  /** image width (px) and horizontal field of view (rad): Pi Camera v2 is 62.2° */
  width: number
  hfov: number
  /** where the camera points relative to the robot's front (rad, + to the left) */
  yaw: number
  /** the camera's position in the lidar frame (m, x forward, y left) */
  x: number
  y: number
}

/** on this robot the camera sits on a small tripod 17.5 cm behind the lidar, on the center line */
export const PI_CAMERA_V2: CameraParams = { width: 640, hfov: (62.2 * Math.PI) / 180, yaw: 0, x: -0.175, y: 0 }

/** Bearing (rad, + to the left) of an image column, for a pinhole camera. */
export function columnBearing(px: number, cam: CameraParams): number {
  const f = cam.width / 2 / Math.tan(cam.hfov / 2)
  return cam.yaw + Math.atan2(cam.width / 2 - px, f)
}

/**
 * Where a detection is, in the lidar frame: the lidar points (lidar frame) whose bearing from the
 * camera falls in the middle half of the box, the nearer ones (the object, not the wall behind it).
 * Null when no lidar point is there (the object is above or below the lidar's plane).
 */
export function locate(det: Detection, points: Vec2[], cam: CameraParams): Vec2 | null {
  const [x1, , x2] = det.box
  const mid = (x1 + x2) / 2
  const half = Math.max((x2 - x1) / 4, 2)
  const a = columnBearing(mid + half, cam)
  const b = columnBearing(mid - half, cam)
  const inside = points
    .map((p) => ({ p, ang: Math.atan2(p.y - cam.y, p.x - cam.x), d: Math.hypot(p.x - cam.x, p.y - cam.y) }))
    .filter((q) => q.ang >= Math.min(a, b) && q.ang <= Math.max(a, b))
    .sort((q, r) => q.d - r.d)
  if (!inside.length) return null
  // the nearest third: robust to stray echoes and to the wall seen past the object's edges
  const near = inside.slice(0, Math.max(1, Math.ceil(inside.length / 3)))
  const pick = near[Math.floor(near.length / 2)]
  return { x: pick.p.x, y: pick.p.y }
}

export interface MapObject {
  id: number
  label: string
  x: number
  y: number
  sightings: number
  lastSeen: number
}

export class ObjectMap {
  objects: MapObject[] = []
  private nextId = 1

  constructor(
    /** same label within this distance (m) is the same object */
    public mergeRadius = 0.5,
    /** sightings before an object counts */
    public confirmAfter = 3,
  ) {}

  get confirmed(): MapObject[] {
    return this.objects.filter((o) => o.sightings >= this.confirmAfter)
  }

  /** Adds a sighting at a map position; returns the object and whether it just got confirmed. */
  add(label: string, at: Vec2, time: number): { object: MapObject; confirmedNow: boolean } {
    let best: MapObject | null = null
    let bestD = this.mergeRadius
    for (const o of this.objects) {
      const d = Math.hypot(o.x - at.x, o.y - at.y)
      if (o.label === label && d < bestD) {
        best = o
        bestD = d
      }
    }
    if (!best) {
      best = { id: this.nextId++, label, x: at.x, y: at.y, sightings: 0, lastSeen: time }
      this.objects.push(best)
    }
    // running mean, weighing the first sightings more and settling as they add up
    const k = 1 / Math.min(best.sightings + 1, 20)
    best.x += (at.x - best.x) * k
    best.y += (at.y - best.y) * k
    best.sightings++
    best.lastSeen = time
    return { object: best, confirmedNow: best.sightings === this.confirmAfter }
  }

  /** Nearest confirmed object with this label (or containing it), for "go to the chair". */
  find(label: string, from: Vec2): MapObject | null {
    const l = label.toLowerCase()
    const c = this.confirmed.filter((o) => o.label.toLowerCase().includes(l))
    c.sort((a, b) => Math.hypot(a.x - from.x, a.y - from.y) - Math.hypot(b.x - from.x, b.y - from.y))
    return c[0] ?? null
  }
}

/** A point in the robot (lidar) frame → the map, given the lidar's pose on the map. */
export function toMap(p: Vec2, pose: Pose): Vec2 {
  const c = Math.cos(pose.theta)
  const s = Math.sin(pose.theta)
  return { x: pose.x + c * p.x - s * p.y, y: pose.y + s * p.x + c * p.y }
}
