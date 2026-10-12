import * as THREE from 'three'
import type { Vec2 } from '../core/math/vec'

/** Polyline of recent positions (ring buffer shifted in place). */
export class Trail3d {
  readonly line: THREE.Line
  private count = 0
  private last: Vec2 | null = null

  constructor(color: number, private max = 1500, private height = 0.02) {
    const geo = new THREE.BufferGeometry()
    geo.setAttribute('position', new THREE.BufferAttribute(new Float32Array(max * 3), 3))
    geo.setDrawRange(0, 0)
    this.line = new THREE.Line(geo, new THREE.LineBasicMaterial({ color, transparent: true, opacity: 0.7 }))
    this.line.frustumCulled = false
  }

  push(p: Vec2): void {
    if (this.last) {
      const d = Math.hypot(p.x - this.last.x, p.y - this.last.y)
      if (d < 0.05) return
      if (d > 1.5) this.clear() // teleport (reset / kidnap)
    }
    const attr = this.line.geometry.attributes.position as THREE.BufferAttribute
    const arr = attr.array as Float32Array
    if (this.count === this.max) {
      arr.copyWithin(0, 3)
      this.count--
    }
    arr.set([p.x, this.height, p.y], this.count * 3)
    this.count++
    this.last = { ...p }
    attr.needsUpdate = true
    this.line.geometry.setDrawRange(0, this.count)
  }

  clear(): void {
    this.count = 0
    this.last = null
    this.line.geometry.setDrawRange(0, 0)
  }
}
