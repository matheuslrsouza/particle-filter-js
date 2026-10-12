import * as THREE from 'three'
import type { Pose } from '../core/math/vec'
import type { LidarConfig } from '../core/sensors/lidar'
import { COLORS } from './scene'

const SENSOR_HEIGHT = 0.4

/** Beams of the last scan from the real car, plus green dots where they hit a wall. */
export class Lidar3d {
  readonly group = new THREE.Group()
  private rays: THREE.LineSegments
  private hits: THREE.Points

  constructor(private cfg: LidarConfig) {
    const n = cfg.angles.length
    const rayGeo = new THREE.BufferGeometry()
    rayGeo.setAttribute('position', new THREE.BufferAttribute(new Float32Array(n * 6), 3))
    this.rays = new THREE.LineSegments(
      rayGeo,
      new THREE.LineBasicMaterial({ color: 0xffffff, transparent: true, opacity: 0.22, depthWrite: false }),
    )
    const hitGeo = new THREE.BufferGeometry()
    hitGeo.setAttribute('position', new THREE.BufferAttribute(new Float32Array(n * 3), 3))
    this.hits = new THREE.Points(hitGeo, new THREE.PointsMaterial({ color: COLORS.lidarHit, size: 0.14 }))
    this.rays.frustumCulled = this.hits.frustumCulled = false
    this.group.add(this.rays, this.hits)
  }

  update(pose: Pose, z: Float64Array): void {
    const rp = this.rays.geometry.attributes.position as THREE.BufferAttribute
    const hp = this.hits.geometry.attributes.position as THREE.BufferAttribute
    for (let i = 0; i < z.length; i++) {
      const a = pose.theta + this.cfg.angles[i]
      const x = pose.x + Math.cos(a) * z[i]
      const y = pose.y + Math.sin(a) * z[i]
      rp.setXYZ(2 * i, pose.x, SENSOR_HEIGHT, pose.y)
      rp.setXYZ(2 * i + 1, x, SENSOR_HEIGHT, y)
      const hit = z[i] < this.cfg.maxRange
      hp.setXYZ(i, x, hit ? SENSOR_HEIGHT : -10, y)
    }
    rp.needsUpdate = hp.needsUpdate = true
  }
}
