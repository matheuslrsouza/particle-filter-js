import * as THREE from 'three'
import type { Estimate } from '../core/localization/particleFilter'
import { COLORS } from './scene'

/** 2-sigma ellipse of the particle filter's position covariance. */
export class Belief3d {
  readonly mesh: THREE.LineLoop

  constructor() {
    const pts: THREE.Vector3[] = []
    for (let i = 0; i < 64; i++) {
      const a = (i / 64) * Math.PI * 2
      pts.push(new THREE.Vector3(Math.cos(a), 0, Math.sin(a)))
    }
    this.mesh = new THREE.LineLoop(
      new THREE.BufferGeometry().setFromPoints(pts),
      new THREE.LineBasicMaterial({ color: COLORS.belief }),
    )
  }

  update(e: Estimate): void {
    const [a, b, c] = e.cov
    const mean = (a + c) / 2
    const d = Math.sqrt(((a - c) / 2) ** 2 + b * b)
    const major = 2 * Math.sqrt(Math.max(mean + d, 0))
    const minor = 2 * Math.sqrt(Math.max(mean - d, 0))
    this.mesh.position.set(e.x, 0.03, e.y)
    this.mesh.rotation.y = -0.5 * Math.atan2(2 * b, a - c)
    this.mesh.scale.set(Math.max(major, 0.08), 1, Math.max(minor, 0.08))
  }
}
