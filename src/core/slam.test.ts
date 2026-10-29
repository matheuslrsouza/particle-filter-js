import { readFileSync } from 'node:fs'
import { describe, expect, it } from 'vitest'
import { scan360, type SceneSegment } from './city/perception'
import { CamsenseParser, RevolutionAssembler } from './lidar/camsense'
import { wrapAngle } from './math/angles'
import { Rng } from './math/rng'
import { HectorSlam, scanToPoints } from './slam/hector'

/** An L-shaped room (5 × 4 m) with a box and a pillar, as wall segments. */
function room(): SceneSegment[] {
  const polys = [
    [[-2, -2], [3, -2], [3, 2], [0.5, 2], [0.5, 1], [-2, 1]],
    [[1.5, -1.2], [2.1, -1.2], [2.1, -0.6], [1.5, -0.6]],
    [[-1.2, 0.2], [-0.9, 0.2], [-0.9, 0.5], [-1.2, 0.5]],
  ]
  const segs: SceneSegment[] = []
  for (const p of polys) for (let i = 0; i < p.length; i++) {
    const [a, b] = [p[i], p[(i + 1) % p.length]]
    segs.push({ a: { x: a[0], y: a[1] }, b: { x: b[0], y: b[1] }, mapped: true, object: -1 })
  }
  return segs
}

describe('Hector SLAM', () => {
  it('tracks a robot moving and turning in a room from lidar scans alone', () => {
    const rng = new Rng(4)
    const scene = room()
    const slam = new HectorSlam()
    let truth = { x: -1.2, y: -1.2, theta: 0.3 }
    let worst = 0
    for (let k = 0; k < 80; k++) {
      // 5 turns per second: 4 cm and up to 6° per turn, like a cleaning robot
      if (k > 0) {
        const w = 0.1 * Math.sin(k / 9)
        truth = { x: truth.x + Math.cos(truth.theta) * 0.04, y: truth.y + Math.sin(truth.theta) * 0.04, theta: truth.theta + w }
      }
      const hits = scan360(truth, scene, { beams: 390, range: 8, noise: 0.01 }, rng)
      const c = Math.cos(truth.theta)
      const s = Math.sin(truth.theta)
      const pts = hits.map((h) => ({ x: c * (h.x - truth.x) + s * (h.y - truth.y), y: -s * (h.x - truth.x) + c * (h.y - truth.y) }))
      slam.addScan(pts)
      // the SLAM frame starts at the first pose: compare in that frame
      const t0 = { x: -1.2, y: -1.2, theta: 0.3 }
      const dx = truth.x - t0.x
      const dy = truth.y - t0.y
      const rel = { x: Math.cos(t0.theta) * dx + Math.sin(t0.theta) * dy, y: -Math.sin(t0.theta) * dx + Math.cos(t0.theta) * dy }
      worst = Math.max(worst, Math.hypot(slam.pose.x - rel.x, slam.pose.y - rel.y))
      expect(Math.abs(wrapAngle(slam.pose.theta - (truth.theta - t0.theta)))).toBeLessThan(0.05)
    }
    expect(worst).toBeLessThan(0.08)
    expect(slam.lastFit).toBeGreaterThan(0.8)
  })

  it('undoes the distortion of a turn taken while rotating', () => {
    // a wall 1 m ahead, seen while the robot turned 10° during the turn
    const samples = [0, 1, 2].map((i) => ({ angle: 0, distance: 1, quality: 50, i }))
    const plain = scanToPoints(samples, { mirror: false, rotation: 0, minRange: 0.1, maxRange: 8, minQuality: 1 })
    expect(plain[0].x).toBeCloseTo(1)
    const fixed = scanToPoints(samples, { mirror: false, rotation: 0, minRange: 0.1, maxRange: 8, minQuality: 1, motion: { x: 0, y: 0, theta: 0.17 } })
    // the first point was taken 10° before the end: seen from the end pose it is 10° to the right
    expect(Math.atan2(fixed[0].y, fixed[0].x)).toBeCloseTo(-0.17, 2)
    expect(Math.atan2(fixed[2].y, fixed[2].x)).toBeCloseTo(0, 5)
  })

  it('keeps the scans of the real robot consistent with its own map', () => {
    const bytes = new Uint8Array(readFileSync(new URL('../real/sample-capture.bin', import.meta.url)))
    const asm = new RevolutionAssembler()
    const turns = new CamsenseParser().push(bytes).map((p) => asm.add(p)).filter((r) => r !== null)
    const slam = new HectorSlam()
    const fits: number[] = []
    for (const t of turns) {
      slam.addScan(scanToPoints(t.samples, { mirror: false, rotation: 0, minRange: 0.12, maxRange: 8, minQuality: 1, motion: slam.motion }))
      fits.push(slam.lastFit)
    }
    const mean = fits.slice(10).reduce((a, b) => a + b, 0) / (fits.length - 10)
    expect(mean).toBeGreaterThan(0.85)
  })
})
