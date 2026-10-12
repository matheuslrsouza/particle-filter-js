import * as THREE from 'three'
import type { Pose } from '../core/math/vec'
import { COLORS } from './scene'

const WHEEL_RADIUS = 0.09

/**
 * Low-poly procedural car. Its origin is the rear axle (the bicycle model reference point)
 * and it faces +x, like the simulation. Front wheels steer, all wheels roll with the odometer.
 */
export class Car3d {
  readonly group = new THREE.Group()
  private frontPivots: THREE.Group[] = []
  private wheels: THREE.Mesh[] = []
  private lidar: THREE.Mesh

  constructor(wheelbase: number, ghost = false) {
    const mat = (color: number, extra: THREE.MeshStandardMaterialParameters = {}) =>
      ghost
        ? new THREE.MeshBasicMaterial({ color: COLORS.belief, transparent: true, opacity: 0.18, depthWrite: false })
        : new THREE.MeshStandardMaterial({ color, roughness: 0.45, metalness: 0.2, ...extra })

    const body = new THREE.Mesh(new THREE.BoxGeometry(0.78, 0.14, 0.4), mat(COLORS.car))
    body.position.set(wheelbase / 2, 0.16, 0)
    const nose = new THREE.Mesh(new THREE.BoxGeometry(0.16, 0.08, 0.36), mat(COLORS.car))
    nose.position.set(wheelbase / 2 + 0.43, 0.13, 0)
    const cabin = new THREE.Mesh(new THREE.BoxGeometry(0.34, 0.13, 0.32), mat(0x1d2a44, { roughness: 0.15, metalness: 0.6 }))
    cabin.position.set(wheelbase / 2 - 0.06, 0.295, 0)
    this.lidar = new THREE.Mesh(new THREE.CylinderGeometry(0.055, 0.06, 0.07, 16), mat(0x222831))
    this.lidar.position.set(wheelbase / 2 - 0.06, 0.395, 0)
    const stripe = new THREE.Mesh(new THREE.BoxGeometry(0.06, 0.01, 0.06), mat(COLORS.belief, { emissive: COLORS.belief }))
    stripe.position.set(0.04, 0.04, 0)
    this.lidar.add(stripe)
    this.group.add(body, nose, cabin, this.lidar)

    if (!ghost) {
      for (const z of [-0.12, 0.12]) {
        const light = new THREE.Mesh(
          new THREE.BoxGeometry(0.02, 0.04, 0.07),
          new THREE.MeshStandardMaterial({ color: 0xfff6d5, emissive: 0xfff2b0, emissiveIntensity: 2 }),
        )
        light.position.set(wheelbase / 2 + 0.515, 0.14, z)
        this.group.add(light)
      }
    }

    const wheelGeo = new THREE.CylinderGeometry(WHEEL_RADIUS, WHEEL_RADIUS, 0.07, 18).rotateX(Math.PI / 2)
    const rimGeo = new THREE.BoxGeometry(0.12, 0.03, 0.072)
    for (const [x, front] of [[0, false], [wheelbase, true]] as const) {
      for (const z of [-0.22, 0.22]) {
        const wheel = new THREE.Mesh(wheelGeo, mat(0x15181d, { roughness: 0.9, metalness: 0 }))
        wheel.add(new THREE.Mesh(rimGeo, mat(0x9aa4b1)))
        const pivot = new THREE.Group()
        pivot.position.set(x, WHEEL_RADIUS, z)
        pivot.add(wheel)
        this.group.add(pivot)
        this.wheels.push(wheel)
        if (front) this.frontPivots.push(pivot)
      }
    }

    this.group.traverse((o) => {
      if (o instanceof THREE.Mesh) o.castShadow = !ghost
    })
  }

  update(pose: Pose, steer: number, odometer: number, dt: number): void {
    this.group.position.set(pose.x, 0, pose.y)
    // +theta turns from +x towards +z, which is a negative rotation around three's +y
    this.group.rotation.y = -pose.theta
    for (const p of this.frontPivots) p.rotation.y = -steer
    for (const w of this.wheels) w.rotation.z = -odometer / WHEEL_RADIUS
    this.lidar.rotation.y += dt * 12
  }
}
