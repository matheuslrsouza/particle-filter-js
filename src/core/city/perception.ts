import type { Rng } from '../math/rng'
import type { Pose, Vec2 } from '../math/vec'

/** A wall or object outline the lidar can hit. */
export interface SceneSegment {
  a: Vec2
  b: Vec2
  /** part of the known map (buildings, parked cars, barricades): filtered out before tracking */
  mapped: boolean
  /** ground-truth object id (only used to evaluate the tracker), -1 for static geometry */
  object: number
}

export interface LidarPoint {
  x: number
  y: number
  beam: number
  mapped: boolean
  object: number
}

export interface Detection {
  kind: 'car' | 'pedestrian'
  center: Vec2
  heading: number
  length: number
  width: number
  points: Vec2[]
}

export interface LidarParams {
  beams: number
  range: number
  noise: number
}

export const LIDAR_360: LidarParams = { beams: 360, range: 60, noise: 0.02 }

/** Car size used to complete partial L-shapes (a car seen from one corner shows two sides at most). */
export const CAR_PRIOR = { length: 4.7, width: 1.85 }

/**
 * 2D 360° lidar: one ray per beam against the scene segments near the sensor.
 * Returns the hit points with the hit segment's labels.
 */
export function scan360(sensor: Pose, scene: SceneSegment[], p: LidarParams, rng: Rng): LidarPoint[] {
  // only what can be in range
  const near = scene.filter((s) => {
    const minX = Math.min(s.a.x, s.b.x)
    const maxX = Math.max(s.a.x, s.b.x)
    const minY = Math.min(s.a.y, s.b.y)
    const maxY = Math.max(s.a.y, s.b.y)
    const dx = Math.max(minX - sensor.x, 0, sensor.x - maxX)
    const dy = Math.max(minY - sensor.y, 0, sensor.y - maxY)
    return dx * dx + dy * dy < p.range * p.range
  })
  const out: LidarPoint[] = []
  for (let b = 0; b < p.beams; b++) {
    const ang = sensor.theta + (b / p.beams) * Math.PI * 2
    const dx = Math.cos(ang)
    const dy = Math.sin(ang)
    let best = p.range
    let hit: SceneSegment | null = null
    for (const w of near) {
      const ex = w.b.x - w.a.x
      const ey = w.b.y - w.a.y
      const den = ex * dy - ey * dx
      if (den === 0) continue
      const qx = w.a.x - sensor.x
      const qy = w.a.y - sensor.y
      const t = (ex * qy - ey * qx) / den
      const u = (dx * qy - dy * qx) / den
      if (t > 0.3 && t < best && u >= 0 && u <= 1) {
        best = t
        hit = w
      }
    }
    if (!hit) continue
    const r = best + rng.gaussian(0, p.noise)
    out.push({ x: sensor.x + dx * r, y: sensor.y + dy * r, beam: b, mapped: hit.mapped, object: hit.object })
  }
  return out
}

/**
 * Scan-line segmentation of the points that are not on mapped geometry: consecutive beams whose
 * points are closer than `maxGap` belong to the same object (the scan wraps around).
 */
export function cluster(points: LidarPoint[], beams: number, maxGap = 0.9): LidarPoint[][] {
  const dynamic = points.filter((p) => !p.mapped).sort((a, b) => a.beam - b.beam)
  const clusters: LidarPoint[][] = []
  for (const p of dynamic) {
    const last = clusters[clusters.length - 1]
    const prev = last?.[last.length - 1]
    if (prev && p.beam - prev.beam <= 2 && Math.hypot(p.x - prev.x, p.y - prev.y) < maxGap) last.push(p)
    else clusters.push([p])
  }
  // wrap-around: the first and last clusters may be the same object
  if (clusters.length > 1) {
    const first = clusters[0][0]
    const lastC = clusters[clusters.length - 1]
    const last = lastC[lastC.length - 1]
    if (first.beam + beams - last.beam <= 2 && Math.hypot(first.x - last.x, first.y - last.y) < maxGap) {
      clusters[0] = [...lastC, ...clusters[0]]
      clusters.pop()
    }
  }
  // an object partly hidden behind another one shows up as several pieces: merge pieces that
  // are close in space (not only in beam order)
  for (let i = 0; i < clusters.length; i++) {
    for (let j = i + 1; j < clusters.length; j++) {
      if (clusters[i].some((p) => clusters[j].some((q) => Math.hypot(p.x - q.x, p.y - q.y) < maxGap))) {
        clusters[i].push(...clusters[j])
        clusters.splice(j, 1)
        j = i
      }
    }
  }
  return clusters
}

/**
 * L-shape fitting (Zhang et al., "Efficient L-Shape Fitting for Vehicle Detection Using Laser
 * Scanners", 2017): search the orientation that maximizes the closeness of the points to two
 * perpendicular edges of their bounding rectangle.
 */
export function fitLShape(pts: Vec2[]): { center: Vec2; heading: number; length: number; width: number } {
  let bestScore = -Infinity
  let best = { theta: 0, c1: [0, 0], c2: [0, 0] }
  for (let deg = 0; deg < 90; deg += 1) {
    const th = (deg * Math.PI) / 180
    const ux = Math.cos(th)
    const uy = Math.sin(th)
    const p1 = pts.map((p) => p.x * ux + p.y * uy)
    const p2 = pts.map((p) => -p.x * uy + p.y * ux)
    const min1 = Math.min(...p1)
    const max1 = Math.max(...p1)
    const min2 = Math.min(...p2)
    const max2 = Math.max(...p2)
    let score = 0
    for (let i = 0; i < pts.length; i++) {
      const d1 = Math.min(max1 - p1[i], p1[i] - min1)
      const d2 = Math.min(max2 - p2[i], p2[i] - min2)
      score += 1 / Math.max(Math.min(d1, d2), 0.01)
    }
    if (score > bestScore) {
      bestScore = score
      best = { theta: th, c1: [min1, max1], c2: [min2, max2] }
    }
  }
  let { theta } = best
  let len = best.c1[1] - best.c1[0]
  let wid = best.c2[1] - best.c2[0]
  const m1 = (best.c1[0] + best.c1[1]) / 2
  const m2 = (best.c2[0] + best.c2[1]) / 2
  const center = { x: m1 * Math.cos(theta) - m2 * Math.sin(theta), y: m1 * Math.sin(theta) + m2 * Math.cos(theta) }
  // the longer side is the heading axis
  if (wid > len) {
    ;[len, wid] = [wid, len]
    theta += Math.PI / 2
  }
  return { center, heading: theta, length: len, width: wid }
}

/**
 * From clusters to objects: cars (L-shape completed to a typical car size, growing the box away
 * from the sensor since only the near sides are visible) and pedestrians (small clusters).
 */
export function detect(clusters: LidarPoint[][], sensor: Vec2): Detection[] {
  const out: Detection[] = []
  for (const c of clusters) {
    if (c.length < 2) continue
    const pts = c.map((p) => ({ x: p.x, y: p.y }))
    const box = fitLShape(pts)
    // a person is about 0.5 m across; bigger blobs under ~1.3 m are ambiguous (a car's corner)
    if (box.length < 0.8 && box.width < 0.8) {
      out.push({ kind: 'pedestrian', center: box.center, heading: box.heading, length: 0.5, width: 0.5, points: pts })
      continue
    }
    if (box.length < 1.3 || box.length > 7 || box.width > 3) continue
    // a single visible side ("I-shape"): if it's as short as a car is wide, it's the front or the
    // back, so the car's axis is perpendicular to it
    if (box.width < 0.4 && box.length < 2.6) {
      box.heading += Math.PI / 2
      ;[box.length, box.width] = [box.width, box.length]
    }
    // shape prior: extend the visible box to a car, away from the sensor
    const ux = Math.cos(box.heading)
    const uy = Math.sin(box.heading)
    const toSensor = { x: sensor.x - box.center.x, y: sensor.y - box.center.y }
    const along = toSensor.x * ux + toSensor.y * uy
    const across = -toSensor.x * uy + toSensor.y * ux
    const dl = Math.max(0, CAR_PRIOR.length - box.length) / 2
    const dw = Math.max(0, CAR_PRIOR.width - box.width) / 2
    const sl = along > 0 ? -1 : 1
    const sw = across > 0 ? -1 : 1
    const center = { x: box.center.x + ux * dl * sl - uy * dw * sw, y: box.center.y + uy * dl * sl + ux * dw * sw }
    out.push({ kind: 'car', center, heading: box.heading, length: Math.max(box.length, CAR_PRIOR.length), width: Math.max(box.width, CAR_PRIOR.width), points: pts })
  }
  return out
}
