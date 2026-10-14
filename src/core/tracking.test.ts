import { describe, expect, it } from 'vitest'
import { cluster, detect, fitLShape, scan360, type SceneSegment } from './city/perception'
import { rectangle } from './city/parking'
import { hungarian, Tracker } from './city/tracker'
import { wrapAngle } from './math/angles'
import { Rng } from './math/rng'

const boxSegments = (c: { x: number; y: number }, heading: number, length: number, width: number, object: number): SceneSegment[] => {
  const r = rectangle(c, heading, length, width)
  return r.map((a, i) => ({ a, b: r[(i + 1) % 4], mapped: false, object }))
}

describe('Hungarian algorithm', () => {
  it('finds the minimum-cost assignment', () => {
    expect(hungarian([[4, 1, 3], [2, 0, 5], [3, 2, 2]])).toEqual([1, 0, 2])
  })
  it('handles rectangular matrices and forbidden pairs', () => {
    expect(hungarian([[1, Infinity], [Infinity, Infinity], [2, 1]])).toEqual([0, -1, 1])
  })
})

describe('L-shape fitting', () => {
  it('recovers the orientation of a car seen from one corner', () => {
    const rng = new Rng(2)
    for (const heading of [0.3, 1.2, -0.7]) {
      const scene = boxSegments({ x: 15, y: 6 }, heading, 4.7, 1.85, 1)
      const points = scan360({ x: 0, y: 0, theta: 0 }, scene, { beams: 720, range: 60, noise: 0.02 }, rng)
      const box = fitLShape(points)
      const err = Math.abs(wrapAngle(2 * (box.heading - heading))) / 2 // modulo 180°
      expect(err).toBeLessThan(0.06)
    }
  })

  it('completes a partial car to its full size, away from the sensor', () => {
    const rng = new Rng(3)
    const center = { x: 12, y: -5 }
    const points = scan360({ x: 0, y: 0, theta: 0 }, boxSegments(center, 0.4, 4.7, 1.85, 1), { beams: 720, range: 60, noise: 0.02 }, rng)
    const [d] = detect(cluster(points, 720), { x: 0, y: 0 })
    expect(d.kind).toBe('car')
    expect(Math.hypot(d.center.x - center.x, d.center.y - center.y)).toBeLessThan(0.5)
  })
})

describe('Kalman tracker', () => {
  it('tracks a car at constant velocity and estimates its speed', () => {
    const rng = new Rng(4)
    const tracker = new Tracker()
    let x = 5
    for (let k = 0; k < 40; k++) {
      x += 10 * 0.1
      tracker.step([{ kind: 'car', center: { x: x + rng.gaussian(0, 0.3), y: 3 + rng.gaussian(0, 0.3) }, heading: 0, length: 4.7, width: 1.85, points: [] }], 0.1)
    }
    const t = tracker.tracks.find((tr) => tr.confirmed)!
    expect(tracker.tracks.length).toBe(1)
    expect(Math.abs(t.x[2] - 10)).toBeLessThan(0.6)
    expect(Math.abs(t.x[0] - x)).toBeLessThan(0.4)
  })

  it('keeps identities when two cars pass each other', () => {
    const tracker = new Tracker()
    const ids = new Set<number>()
    for (let k = 0; k < 60; k++) {
      const a = { x: k * 1.0, y: 0 }
      const b = { x: 60 - k * 1.0, y: 3.5 }
      tracker.step(
        [a, b].map((c) => ({ kind: 'car' as const, center: c, heading: 0, length: 4.7, width: 1.85, points: [] })),
        0.1,
      )
      for (const t of tracker.tracks) if (t.confirmed) ids.add(t.id)
    }
    expect(ids.size).toBe(2)
  })
})

describe('perception in the city', () => {
  it('tracks the traffic well and drives on its own tracks without collisions', async () => {
    const cityJson = (await import('../maps/city-mountain-view.json')).default
    const { CitySim } = await import('./city/citySim')
    const { NPC_LENGTH, NPC_WIDTH } = await import('./city/traffic')
    type P = { x: number; y: number }
    const overlap = (a: P[], b: P[]) => {
      for (const poly of [a, b]) {
        for (let i = 0; i < 4; i++) {
          const p = poly[i]
          const q = poly[(i + 1) % 4]
          const nx = -(q.y - p.y)
          const ny = q.x - p.x
          const pa = a.map((v) => v.x * nx + v.y * ny)
          const pb = b.map((v) => v.x * nx + v.y * ny)
          if (Math.max(...pa) < Math.min(...pb) || Math.max(...pb) < Math.min(...pa)) return false
        }
      }
      return true
    }
    for (const seed of [3, 5]) {
      const sim = new CitySim(cityJson as never, seed)
      sim.enableTraffic(40, 30)
      sim.perception = 'tracks'
      expect(sim.randomDestination()).toBe(true)
      let collisions = 0
      for (let i = 0; i < 50 * 400 && sim.phase === 'driving'; i++) {
        sim.tick()
        if (i % 10) continue
        const c = sim.car
        const ego = rectangle({ x: c.x + Math.cos(c.theta) * 1.74, y: c.y + Math.sin(c.theta) * 1.74 }, c.theta, 5.04, 1.95)
        if (sim.traffic!.npcs.some((n) => overlap(ego, rectangle(n.pose, n.pose.theta, NPC_LENGTH, NPC_WIDTH)))) collisions++
        const cx = ego.reduce((s, q) => s + q.x, 0) / 4
        const cy = ego.reduce((s, q) => s + q.y, 0) / 4
        for (const p of sim.traffic!.pedestrians) if (p.state === 'crossing' && Math.hypot(p.pos.x - cx, p.pos.y - cy) < 1.8) collisions++
      }
      expect(sim.phase).toBe('arrived')
      expect(collisions).toBe(0)
      const m = sim.metrics.summary
      expect(m.mota).toBeGreaterThan(0.6)
      expect(m.recall).toBeGreaterThan(0.75)
      expect(m.posErr).toBeLessThan(0.4)
      expect(m.velErr).toBeLessThan(1)
    }
  }, 180_000)
})
