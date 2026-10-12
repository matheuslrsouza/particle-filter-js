import * as THREE from 'three'
import type { Plan } from '../core/sim/simulation'
import type { OccupancyGrid } from '../core/world/grid'
import { COLORS } from './scene'

const smoothstep = (t: number) => t * t * (3 - 2 * t)
const progress = (t: number, from: number, to: number) => smoothstep(Math.min(1, Math.max(0, (t - from) / (to - from))))

/**
 * A* expansion, raw path, smoothed path and goal flag.
 * During the planning phase they are revealed one after the other.
 */
export class Plan3d {
  readonly group = new THREE.Group()
  readonly expanded: THREE.InstancedMesh
  readonly raw = new THREE.Group()
  readonly smooth = new THREE.Group()
  private flag = new THREE.Group()
  private flagCloth: THREE.Mesh
  private ring: THREE.Mesh
  private rawLine: THREE.Line | null = null
  private tube: THREE.Mesh | null = null
  private expandedCount = 0
  private expandedMat: THREE.MeshBasicMaterial

  constructor(private grid: OccupancyGrid) {
    const geo = new THREE.PlaneGeometry(grid.cellSize * 0.86, grid.cellSize * 0.86).rotateX(-Math.PI / 2)
    this.expandedMat = new THREE.MeshBasicMaterial({ color: COLORS.expanded, transparent: true, opacity: 0.5, depthWrite: false })
    this.expanded = new THREE.InstancedMesh(geo, this.expandedMat, grid.cells.length)
    this.expanded.count = 0
    this.expanded.frustumCulled = false

    const pole = new THREE.Mesh(
      new THREE.CylinderGeometry(0.02, 0.02, 1.1, 8),
      new THREE.MeshStandardMaterial({ color: 0xdddddd }),
    )
    pole.position.y = 0.55
    const shape = new THREE.Shape()
    shape.moveTo(0, 0)
    shape.lineTo(0.5, -0.15)
    shape.lineTo(0, -0.3)
    this.flagCloth = new THREE.Mesh(
      new THREE.ShapeGeometry(shape),
      new THREE.MeshStandardMaterial({ color: COLORS.goal, emissive: COLORS.goal, emissiveIntensity: 0.4, side: THREE.DoubleSide }),
    )
    this.flagCloth.position.y = 1.08
    this.ring = new THREE.Mesh(
      new THREE.RingGeometry(0.25, 0.32, 40).rotateX(-Math.PI / 2),
      new THREE.MeshBasicMaterial({ color: COLORS.goal, transparent: true, depthWrite: false }),
    )
    this.ring.position.y = 0.015
    pole.castShadow = this.flagCloth.castShadow = true
    this.flag.add(pole, this.flagCloth, this.ring)
    this.flag.visible = false

    this.group.add(this.expanded, this.raw, this.smooth, this.flag)
  }

  setPlan(plan: Plan | null): void {
    this.raw.clear()
    this.smooth.clear()
    this.rawLine?.geometry.dispose()
    this.tube?.geometry.dispose()
    this.rawLine = this.tube = null
    this.flag.visible = !!plan
    this.expandedCount = 0
    this.expanded.count = 0
    if (!plan) return

    const m = new THREE.Matrix4()
    plan.expanded.forEach((idx, k) => {
      const c = this.grid.cellCenter(idx % this.grid.cols, Math.floor(idx / this.grid.cols))
      this.expanded.setMatrixAt(k, m.makeTranslation(c.x, 0.01, c.y))
    })
    this.expanded.instanceMatrix.needsUpdate = true
    this.expandedCount = plan.expanded.length

    this.rawLine = new THREE.Line(
      new THREE.BufferGeometry().setFromPoints(plan.raw.map((p) => new THREE.Vector3(p.x, 0.025, p.y))),
      new THREE.LineBasicMaterial({ color: COLORS.rawPath }),
    )
    this.raw.add(this.rawLine)

    const curve = new THREE.CatmullRomCurve3(plan.smooth.map((p) => new THREE.Vector3(p.x, 0.05, p.y)))
    this.tube = new THREE.Mesh(
      new THREE.TubeGeometry(curve, Math.max(16, plan.smooth.length * 3), 0.03, 6, false),
      new THREE.MeshStandardMaterial({ color: COLORS.smoothPath, emissive: 0x9fd8ff, emissiveIntensity: 0.6 }),
    )
    this.smooth.add(this.tube)

    this.flag.position.set(plan.goal.x, 0, plan.goal.y)
  }

  /** `t` is the time spent in the current phase; `planning` says if it is the planning phase. */
  animate(planning: boolean, t: number, planSeconds: number, time: number): void {
    const reveal = (from: number, to: number) => (planning ? progress(t / planSeconds, from, to) : 1)
    this.expanded.count = Math.floor(this.expandedCount * reveal(0, 0.55))
    this.expandedMat.opacity = planning ? 0.5 : Math.max(0.12, 0.5 - t * 0.4)
    if (this.rawLine) {
      const n = (this.rawLine.geometry.attributes.position as THREE.BufferAttribute).count
      this.rawLine.geometry.setDrawRange(0, Math.ceil(n * reveal(0.5, 0.75)))
    }
    if (this.tube) {
      const n = this.tube.geometry.index!.count
      this.tube.geometry.setDrawRange(0, Math.floor((n * reveal(0.7, 1)) / 6) * 6)
    }
    this.flagCloth.rotation.y = Math.sin(time * 3) * 0.25
    const pulse = (time * 0.8) % 1
    this.ring.scale.setScalar(1 + pulse * 1.5)
    ;(this.ring.material as THREE.MeshBasicMaterial).opacity = 1 - pulse
  }
}
