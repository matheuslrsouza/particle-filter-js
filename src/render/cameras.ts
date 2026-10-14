import * as THREE from 'three'
import { OrbitControls } from 'three/examples/jsm/controls/OrbitControls.js'
import type { Pose } from '../core/math/vec'
import type { Area } from './scene'

export type CameraMode = 'orbit' | 'chase' | 'top'

export class CameraRig {
  readonly camera: THREE.PerspectiveCamera
  readonly controls: OrbitControls
  mode: CameraMode = 'orbit'
  private center: THREE.Vector3
  private chase: { back: number; up: number; ahead: number }
  private chasePos = new THREE.Vector3()
  private chaseLook = new THREE.Vector3()

  constructor(
    dom: HTMLElement,
    private world: Area,
    opts: {
      near?: number
      far?: number
      maxDistance?: number
      minDistance?: number
      /** chase camera placement relative to the car (m): behind, above, look-ahead */
      chase?: { back: number; up: number; ahead: number }
    } = {},
  ) {
    this.chase = opts.chase ?? { back: 2.4, up: 1.6, ahead: 1.5 }
    this.center = new THREE.Vector3(world.width / 2, 0, world.height / 2)
    this.camera = new THREE.PerspectiveCamera(45, dom.clientWidth / dom.clientHeight, opts.near ?? 0.05, opts.far ?? 200)
    this.controls = new OrbitControls(this.camera, dom)
    this.controls.enableDamping = true
    this.controls.maxPolarAngle = Math.PI / 2 - 0.05
    this.controls.minDistance = opts.minDistance ?? 2
    this.controls.maxDistance = opts.maxDistance ?? 90
    this.setMode('orbit')
  }

  setMode(mode: CameraMode): void {
    this.mode = mode
    const c = this.controls
    c.enabled = mode !== 'chase'
    c.enableRotate = mode === 'orbit'
    if (mode === 'orbit') {
      const size = Math.max(this.world.width, this.world.height)
      this.camera.position.set(this.center.x, size * 0.75, this.center.z + size * 0.8)
      c.target.copy(this.center)
    } else if (mode === 'top') {
      const fit = (Math.max(this.world.height, this.world.width / this.camera.aspect) / 2) * 1.15
      const h = fit / Math.tan(THREE.MathUtils.degToRad(this.camera.fov / 2)) + 2
      this.camera.position.set(this.center.x, h, this.center.z + 0.001)
      c.target.copy(this.center)
    }
    c.update()
  }

  resize(w: number, h: number): void {
    this.camera.aspect = w / h
    this.camera.updateProjectionMatrix()
    if (this.mode === 'top') this.setMode('top')
  }

  update(car: Pose, dt: number): void {
    if (this.mode !== 'chase') {
      this.controls.update()
      return
    }
    const dir = new THREE.Vector3(Math.cos(car.theta), 0, Math.sin(car.theta))
    const { back, up, ahead } = this.chase
    const wantPos = new THREE.Vector3(car.x, up, car.y).addScaledVector(dir, -back)
    const wantLook = new THREE.Vector3(car.x, up * 0.125, car.y).addScaledVector(dir, ahead)
    const k = 1 - Math.exp(-dt * 4)
    // first frame, or the car was teleported (reset / kidnap): jump instead of flying across the map
    if (this.chasePos.lengthSq() === 0 || this.chasePos.distanceTo(wantPos) > back * 2.5) {
      this.chasePos.copy(wantPos)
      this.chaseLook.copy(wantLook)
    }
    this.chasePos.lerp(wantPos, k)
    this.chaseLook.lerp(wantLook, k)
    this.camera.position.copy(this.chasePos)
    this.camera.lookAt(this.chaseLook)
  }
}
