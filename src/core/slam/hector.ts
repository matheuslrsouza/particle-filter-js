import { wrapAngle } from '../math/angles'
import type { Pose, Vec2 } from '../math/vec'

/**
 * 2D SLAM without odometry, after Hector SLAM (Kohlbrecher et al., "A Flexible and Scalable SLAM
 * System with Full 3D Motion Estimation", 2011): each lidar turn is matched against the map built
 * so far (Gauss-Newton on the bilinearly interpolated occupancy, coarse to fine over several
 * resolutions), then inserted into the map with ray tracing (log-odds).
 */

export interface SlamParams {
  /** finest cell size (m); each coarser level doubles it */
  resolution: number
  /** the map is a square of this side (m), centered on the start */
  size: number
  levels: number
  /** log-odds added for a hit / for a cell crossed by a ray, and the clamp */
  hit: number
  miss: number
  clamp: number
  /** the map is only updated after moving this much: keeps it from smearing while standing still */
  updateDistance: number
  updateAngle: number
  iterations: number
}

export const DEFAULT_SLAM: SlamParams = {
  resolution: 0.03,
  size: 24,
  levels: 3,
  hit: 0.9,
  miss: -0.3,
  clamp: 6,
  updateDistance: 0.04,
  updateAngle: 0.06,
  iterations: 12,
}

export class OccupancyGrid {
  readonly cells: number
  readonly logOdds: Float32Array
  /** world coordinate of the grid's corner (the grid is centered on 0, 0) */
  readonly origin: number

  constructor(
    readonly resolution: number,
    size: number,
  ) {
    this.cells = Math.ceil(size / resolution)
    this.logOdds = new Float32Array(this.cells * this.cells)
    this.origin = -(this.cells * resolution) / 2
  }

  /** occupancy probability of a cell (0.5 when unknown or outside) */
  prob(ix: number, iy: number): number {
    if (ix < 0 || iy < 0 || ix >= this.cells || iy >= this.cells) return 0.5
    return 1 - 1 / (1 + Math.exp(this.logOdds[iy * this.cells + ix]))
  }

  /** Bilinear occupancy at a world point and its gradient (per meter). */
  sample(x: number, y: number): [number, number, number] {
    const gx = (x - this.origin) / this.resolution - 0.5
    const gy = (y - this.origin) / this.resolution - 0.5
    const ix = Math.floor(gx)
    const iy = Math.floor(gy)
    const fx = gx - ix
    const fy = gy - iy
    const p00 = this.prob(ix, iy)
    const p10 = this.prob(ix + 1, iy)
    const p01 = this.prob(ix, iy + 1)
    const p11 = this.prob(ix + 1, iy + 1)
    const m = fy * (fx * p11 + (1 - fx) * p01) + (1 - fy) * (fx * p10 + (1 - fx) * p00)
    const dx = (fy * (p11 - p01) + (1 - fy) * (p10 - p00)) / this.resolution
    const dy = (fx * (p11 - p10) + (1 - fx) * (p01 - p00)) / this.resolution
    return [m, dx, dy]
  }

  /** Ray tracing from the sensor to each hit: crossed cells get `miss`, the hit cell gets `hit`. */
  insert(pose: Pose, points: Vec2[], p: SlamParams): void {
    const c = Math.cos(pose.theta)
    const s = Math.sin(pose.theta)
    const toCell = (v: number) => Math.floor((v - this.origin) / this.resolution)
    const x0 = toCell(pose.x)
    const y0 = toCell(pose.y)
    const add = (ix: number, iy: number, v: number) => {
      if (ix < 0 || iy < 0 || ix >= this.cells || iy >= this.cells) return
      const k = iy * this.cells + ix
      this.logOdds[k] = Math.max(-p.clamp, Math.min(p.clamp, this.logOdds[k] + v))
    }
    for (const q of points) {
      const x1 = toCell(pose.x + c * q.x - s * q.y)
      const y1 = toCell(pose.y + s * q.x + c * q.y)
      // Bresenham, without the last cell
      let x = x0
      let y = y0
      const dx = Math.abs(x1 - x0)
      const dy = -Math.abs(y1 - y0)
      const sx = x0 < x1 ? 1 : -1
      const sy = y0 < y1 ? 1 : -1
      let err = dx + dy
      while (x !== x1 || y !== y1) {
        add(x, y, p.miss)
        const e2 = 2 * err
        if (e2 >= dy) {
          err += dy
          x += sx
        }
        if (e2 <= dx) {
          err += dx
          y += sy
        }
      }
      add(x1, y1, p.hit - p.miss)
    }
  }
}

export class HectorSlam {
  readonly maps: OccupancyGrid[]
  pose: Pose = { x: 0, y: 0, theta: 0 }
  /** poses where the map was updated */
  trajectory: Pose[] = []
  /** share of the last scan's points that landed on occupied cells after matching (0–1) */
  lastFit = 0
  scans = 0
  private prev: Pose = { x: 0, y: 0, theta: 0 }
  private lastInsert: Pose | null = null

  constructor(readonly p: SlamParams = DEFAULT_SLAM) {
    this.maps = Array.from({ length: p.levels }, (_, k) => new OccupancyGrid(p.resolution * 2 ** k, p.size))
  }

  /** The motion over the last turn, in the robot's frame (for `scanToPoints` to undo distortion). */
  get motion(): Pose {
    const dx = this.pose.x - this.prev.x
    const dy = this.pose.y - this.prev.y
    const c = Math.cos(this.prev.theta)
    const s = Math.sin(this.prev.theta)
    return { x: c * dx + s * dy, y: -s * dx + c * dy, theta: wrapAngle(this.pose.theta - this.prev.theta) }
  }

  /**
   * Adds one turn of the lidar, `points` in the robot frame (x forward, y left), in the order they
   * were measured. Returns the new pose.
   */
  addScan(points: Vec2[]): Pose {
    this.scans++
    if (!this.lastInsert) {
      for (const m of this.maps) m.insert(this.pose, points, this.p)
      this.lastInsert = { ...this.pose }
      this.trajectory.push({ ...this.pose })
      return this.pose
    }
    // constant velocity: start from where the robot would be if it kept moving the same way
    let guess: Pose = {
      x: 2 * this.pose.x - this.prev.x,
      y: 2 * this.pose.y - this.prev.y,
      theta: wrapAngle(this.pose.theta + wrapAngle(this.pose.theta - this.prev.theta)),
    }
    for (let level = this.maps.length - 1; level >= 0; level--) guess = this.match(this.maps[level], points, guess)
    this.prev = this.pose
    this.pose = guess
    this.lastFit = this.fit(points)
    const li = this.lastInsert
    if (Math.hypot(guess.x - li.x, guess.y - li.y) > this.p.updateDistance || Math.abs(wrapAngle(guess.theta - li.theta)) > this.p.updateAngle) {
      for (const m of this.maps) m.insert(guess, points, this.p)
      this.lastInsert = { ...guess }
      this.trajectory.push({ ...guess })
    }
    return this.pose
  }

  /** Gauss-Newton: the pose that puts the scan points on the most occupied cells. */
  private match(map: OccupancyGrid, points: Vec2[], start: Pose): Pose {
    let { x, y, theta } = start
    for (let it = 0; it < this.p.iterations; it++) {
      const c = Math.cos(theta)
      const s = Math.sin(theta)
      let h00 = 0, h01 = 0, h02 = 0, h11 = 0, h12 = 0, h22 = 0
      let b0 = 0, b1 = 0, b2 = 0
      for (const q of points) {
        const [m, gx, gy] = map.sample(x + c * q.x - s * q.y, y + s * q.x + c * q.y)
        const r = 1 - m
        // ∂(world point)/∂θ
        const j2 = gx * (-s * q.x - c * q.y) + gy * (c * q.x - s * q.y)
        h00 += gx * gx
        h01 += gx * gy
        h02 += gx * j2
        h11 += gy * gy
        h12 += gy * j2
        h22 += j2 * j2
        b0 += gx * r
        b1 += gy * r
        b2 += j2 * r
      }
      if (h00 + h11 < 1e-9) break
      // a little damping keeps featureless directions (a long corridor) from running away
      const d = 1e-3 * (h00 + h11 + h22) + 1e-9
      h00 += d
      h11 += d
      h22 += d
      const det = h00 * (h11 * h22 - h12 * h12) - h01 * (h01 * h22 - h12 * h02) + h02 * (h01 * h12 - h11 * h02)
      if (Math.abs(det) < 1e-12) break
      const dx = (b0 * (h11 * h22 - h12 * h12) - h01 * (b1 * h22 - h12 * b2) + h02 * (b1 * h12 - h11 * b2)) / det
      const dy = (h00 * (b1 * h22 - h12 * b2) - b0 * (h01 * h22 - h12 * h02) + h02 * (h01 * b2 - b1 * h02)) / det
      const dt = (h00 * (h11 * b2 - b1 * h12) - h01 * (h01 * b2 - b1 * h02) + b0 * (h01 * h12 - h11 * h02)) / det
      // limit the step to a cell (and a few degrees): the linearization only holds nearby
      const step = Math.hypot(dx, dy)
      const k = step > map.resolution ? map.resolution / step : 1
      x += dx * k
      y += dy * k
      theta = wrapAngle(theta + Math.max(-0.1, Math.min(0.1, dt)))
      if (step < map.resolution * 0.01 && Math.abs(dt) < 1e-4) break
    }
    return { x, y, theta }
  }

  private fit(points: Vec2[]): number {
    const m = this.maps[0]
    const c = Math.cos(this.pose.theta)
    const s = Math.sin(this.pose.theta)
    let hits = 0
    for (const q of points) {
      const wx = this.pose.x + c * q.x - s * q.y
      const wy = this.pose.y + s * q.x + c * q.y
      const ix = Math.floor((wx - m.origin) / m.resolution)
      const iy = Math.floor((wy - m.origin) / m.resolution)
      let best = 0
      for (let oy = -1; oy <= 1; oy++) for (let ox = -1; ox <= 1; ox++) best = Math.max(best, m.prob(ix + ox, iy + oy))
      if (best > 0.6) hits++
    }
    return points.length ? hits / points.length : 0
  }
}

/**
 * Lidar samples (angle in degrees, distance in meters, in measuring order) → points in the robot
 * frame, undoing the distortion of a robot that moved during the turn: each point is moved to
 * where it would be seen from the pose at the end of the turn. `motion` is the robot's motion over
 * one turn, in the frame of its start.
 */
export function scanToPoints(
  samples: { angle: number; distance: number; quality: number }[],
  opts: { mirror: boolean; rotation: number; minRange: number; maxRange: number; minQuality: number; motion?: Pose },
): Vec2[] {
  const dir = opts.mirror ? -1 : 1
  const rot = (opts.rotation * Math.PI) / 180
  const valid = samples.filter((s) => !Number.isNaN(s.distance) && s.distance >= opts.minRange && s.distance <= opts.maxRange && s.quality >= opts.minQuality)
  const n = valid.length
  const m = opts.motion
  return valid.map((s, i) => {
    const a = dir * ((s.angle * Math.PI) / 180) + rot
    let x = Math.cos(a) * s.distance
    let y = Math.sin(a) * s.distance
    if (m && n > 1) {
      // the robot still had (1 − f) of the turn's motion to go when this point was measured
      const f = 1 - i / (n - 1)
      const th = -m.theta * f
      const c = Math.cos(th)
      const sn = Math.sin(th)
      const tx = -m.x * f
      const ty = -m.y * f
      ;[x, y] = [c * x - sn * y + tx, sn * x + c * y + ty]
    }
    return { x, y }
  })
}
