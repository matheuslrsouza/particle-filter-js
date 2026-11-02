import { describe, expect, it } from 'vitest'
import { columnBearing, locate, ObjectMap, PI_CAMERA_V2, toMap } from './vision/objects'

const cam = { ...PI_CAMERA_V2, x: 0, y: 0 }

describe('camera + lidar objects', () => {
  it('maps image columns to bearings: center straight ahead, edges at ±half the field of view', () => {
    expect(columnBearing(320, cam)).toBeCloseTo(0)
    expect(columnBearing(0, cam)).toBeCloseTo(PI_CAMERA_V2.hfov / 2)
    expect(columnBearing(640, cam)).toBeCloseTo(-PI_CAMERA_V2.hfov / 2)
  })

  it('takes the range from the lidar points in the box, the object and not the wall behind', () => {
    // a chair 1 m ahead, 20° to the left, 30 cm wide; a wall 3 m ahead
    const pts = []
    for (let a = -30; a <= 30; a += 1) {
      const r = (a * Math.PI) / 180
      pts.push({ x: Math.cos(r) * 3, y: Math.sin(r) * 3 })
    }
    const chair = (20 * Math.PI) / 180
    for (let k = -3; k <= 3; k++) {
      const r = chair + k * 0.025
      pts.push({ x: Math.cos(r) * 1, y: Math.sin(r) * 1 })
    }
    // its box: columns around the chair's bearing
    const f = 320 / Math.tan(PI_CAMERA_V2.hfov / 2)
    const cx = 320 - f * Math.tan(chair)
    const at = locate({ label: 'chair', conf: 0.9, box: [cx - 40, 100, cx + 40, 400] }, pts, cam)!
    expect(Math.hypot(at.x, at.y)).toBeCloseTo(1, 1)
    expect(Math.atan2(at.y, at.x)).toBeCloseTo(chair, 1)
    expect(locate({ label: 'cup', conf: 0.9, box: [600, 0, 640, 50] }, [], cam)).toBeNull()
  })

  it('merges sightings of the same object and confirms it after a few', () => {
    const m = new ObjectMap(0.5, 3)
    const pose = { x: 1, y: 2, theta: Math.PI / 2 }
    const p = toMap({ x: 1, y: 0 }, pose)
    expect(p.x).toBeCloseTo(1)
    expect(p.y).toBeCloseTo(3)
    m.add('chair', { x: 1, y: 3 }, 0)
    m.add('chair', { x: 1.1, y: 3 }, 1)
    expect(m.confirmed).toHaveLength(0)
    expect(m.add('chair', { x: 1, y: 3.1 }, 2).confirmedNow).toBe(true)
    m.add('chair', { x: 3, y: 3 }, 3)
    m.add('person', { x: 1, y: 3 }, 3)
    expect(m.objects).toHaveLength(3)
    expect(m.find('chair', { x: 0, y: 0 })!.sightings).toBe(3)
  })
})
