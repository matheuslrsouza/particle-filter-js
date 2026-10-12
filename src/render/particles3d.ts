import * as THREE from 'three'
import type { ParticleFilter } from '../core/localization/particleFilter'
import { COLORS } from './scene'

const MAX_PARTICLES = 5000

/** Each particle is a tiny arrow; brightness follows its weight. */
export class Particles3d {
  readonly mesh: THREE.InstancedMesh
  private m = new THREE.Matrix4()
  private dim = new THREE.Color(0x2a9d63)
  private bright = new THREE.Color(COLORS.particle)
  private c = new THREE.Color()

  constructor() {
    const geo = new THREE.ConeGeometry(0.035, 0.16, 5).rotateZ(-Math.PI / 2)
    this.mesh = new THREE.InstancedMesh(geo, new THREE.MeshBasicMaterial({ color: 0xffffff }), MAX_PARTICLES)
    this.mesh.instanceMatrix.setUsage(THREE.DynamicDrawUsage)
    this.mesh.frustumCulled = false
    this.mesh.count = 0
  }

  update(pf: ParticleFilter): void {
    const n = Math.min(pf.n, MAX_PARTICLES)
    this.mesh.count = n
    let maxW = 0
    for (let i = 0; i < n; i++) maxW = Math.max(maxW, pf.w[i])
    for (let i = 0; i < n; i++) {
      this.m.makeRotationY(-pf.theta[i]).setPosition(pf.x[i], 0.06, pf.y[i])
      this.mesh.setMatrixAt(i, this.m)
      this.mesh.setColorAt(i, this.c.lerpColors(this.dim, this.bright, maxW > 0 ? Math.sqrt(pf.w[i] / maxW) : 1))
    }
    this.mesh.instanceMatrix.needsUpdate = true
    if (this.mesh.instanceColor) this.mesh.instanceColor.needsUpdate = true
  }
}
