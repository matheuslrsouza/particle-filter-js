import * as THREE from 'three'
import type { Pose } from '../core/math/vec'
import type { PathPose } from '../core/planning/hybridAStar'
import type { ExploreGoal } from '../core/sim/exploration'
import { KNOWN_WALL, UNKNOWN, type KnownMap } from '../core/world/knownMap'

export const EXPLORE_COLORS = {
  wall: 0xe63946,
  treeForward: 0xff9f1c,
  treeReverse: 0x3a86ff,
  analytic: 0xc77dff,
  plan: 0x00e5ff,
  rawPlan: 0xffe066,
  mpcPrediction: 0x7cfc00,
  mpcReference: 0xffd60a,
  previousPlan: 0xff2e88,
  footprint: 0x39ff88,
  goal: 0x39ff88,
}

/** Car outline relative to the rear axle (matches Car3d). */
const FOOTPRINT = { back: -0.14, front: 0.76, half: 0.22 }
const RISE_SECONDS = 0.35

/** Wall cells discovered by the lidar, rising from the floor when they appear. */
export class DiscoveredWalls {
  readonly mesh: THREE.InstancedMesh
  private born: { cell: number; t: number; slot: number }[] = []
  private m = new THREE.Matrix4()

  constructor(private map: KnownMap) {
    const cs = map.cellSize
    this.mesh = new THREE.InstancedMesh(
      new THREE.BoxGeometry(cs, 0.5, cs).translate(0, 0.25, 0),
      new THREE.MeshStandardMaterial({ color: EXPLORE_COLORS.wall, emissive: 0x5c0a10, roughness: 0.5 }),
      map.cols * map.rows,
    )
    this.mesh.count = 0
    this.mesh.castShadow = true
    this.mesh.frustumCulled = false
  }

  add(cells: number[], now: number): void {
    for (const cell of cells) {
      const slot = this.mesh.count++
      this.born.push({ cell, t: now, slot })
      this.place(slot, cell, 0.01)
    }
  }

  clear(): void {
    this.mesh.count = 0
    this.born = []
  }

  /** Redraws every wall of the map (after some walls were erased by new evidence). */
  rebuild(): void {
    this.clear()
    for (let i = 0; i < this.map.state.length; i++) {
      if (this.map.state[i] === KNOWN_WALL) this.place(this.mesh.count++, i, 1)
    }
  }

  animate(now: number): void {
    if (!this.born.length) return
    this.born = this.born.filter((b) => {
      const k = Math.min(1, (now - b.t) / RISE_SECONDS)
      this.place(b.slot, b.cell, 1 - (1 - k) ** 3)
      return k < 1
    })
    this.mesh.instanceMatrix.needsUpdate = true
  }

  private place(slot: number, cell: number, height: number) {
    const cs = this.map.cellSize
    const x = ((cell % this.map.cols) + 0.5) * cs
    const z = (Math.floor(cell / this.map.cols) + 0.5) * cs
    this.m.makeScale(1, Math.max(height, 0.01), 1).setPosition(x, 0, z)
    this.mesh.setMatrixAt(slot, this.m)
    this.mesh.instanceMatrix.needsUpdate = true
  }
}

/** Darkens the floor where the car has never seen anything. */
export class Fog {
  readonly mesh: THREE.Mesh
  private tex: THREE.DataTexture
  private data: Uint8Array
  private version = -1

  constructor(private map: KnownMap) {
    this.data = new Uint8Array(map.cols * map.rows * 4)
    this.tex = new THREE.DataTexture(this.data, map.cols, map.rows)
    this.tex.magFilter = THREE.LinearFilter
    this.tex.minFilter = THREE.LinearFilter
    this.mesh = new THREE.Mesh(
      new THREE.PlaneGeometry(map.width, map.height).rotateX(-Math.PI / 2),
      new THREE.MeshBasicMaterial({ map: this.tex, transparent: true, depthWrite: false }),
    )
    this.mesh.position.set(map.width / 2, 0.006, map.height / 2)
    this.mesh.renderOrder = 1
  }

  /** `scanVersion` changes on every lidar scan (free cells can change even without new walls). */
  update(scanVersion: number): void {
    if (scanVersion === this.version) return
    this.version = scanVersion
    const { cols, rows, state } = this.map
    for (let r = 0; r < rows; r++) {
      // texture row 0 is at the far (+z) end of the rotated plane
      const tr = rows - 1 - r
      for (let c = 0; c < cols; c++) {
        const v = state[r * cols + c]
        const o = (tr * cols + c) * 4
        this.data[o] = 6
        this.data[o + 1] = 9
        this.data[o + 2] = 14
        this.data[o + 3] = v === UNKNOWN ? 205 : 0
      }
    }
    this.tex.needsUpdate = true
  }
}

/** Hybrid A* search tree: forward arcs in orange, reverse arcs in blue, Reeds-Shepp shot in purple. */
export class SearchTree {
  readonly group = new THREE.Group()
  private forward: THREE.LineSegments
  private reverse: THREE.LineSegments
  private analytic: THREE.LineSegments
  private shownAt = 0

  constructor() {
    const make = (color: number) => {
      const l = new THREE.LineSegments(
        new THREE.BufferGeometry(),
        new THREE.LineBasicMaterial({ color, transparent: true, opacity: 0.8, depthWrite: false }),
      )
      l.frustumCulled = false
      return l
    }
    this.forward = make(EXPLORE_COLORS.treeForward)
    this.reverse = make(EXPLORE_COLORS.treeReverse)
    this.analytic = make(EXPLORE_COLORS.analytic)
    this.group.add(this.forward, this.reverse, this.analytic)
  }

  /** `analytic` is the Reeds-Shepp shot that reached the goal, drawn in purple. */
  set(forward: Float32Array, reverse: Float32Array, now: number, analytic = new Float32Array(0)): void {
    for (const [line, data] of [[this.forward, forward], [this.reverse, reverse], [this.analytic, analytic]] as const) {
      const pos = new Float32Array((data.length / 2) * 3)
      for (let i = 0, j = 0; i < data.length; i += 2, j += 3) {
        pos[j] = data[i]
        pos[j + 1] = 0.03
        pos[j + 2] = data[i + 1]
      }
      line.geometry.dispose()
      line.geometry = new THREE.BufferGeometry()
      line.geometry.setAttribute('position', new THREE.BufferAttribute(pos, 3))
    }
    this.shownAt = now
  }

  clear(): void {
    this.set(new Float32Array(0), new Float32Array(0), 0)
  }

  /** Grows the tree in over a fraction of a second, in the order nodes were expanded. */
  animate(now: number): void {
    const k = Math.min(1, (now - this.shownAt) / 0.5)
    for (const line of [this.forward, this.reverse, this.analytic]) {
      const n = line.geometry.attributes.position?.count ?? 0
      line.geometry.setDrawRange(0, Math.floor((n * k) / 2) * 2)
    }
  }
}

export class PlanLine {
  readonly line: THREE.Line

  constructor(color: number, height: number, opacity = 1) {
    this.line = new THREE.Line(
      new THREE.BufferGeometry(),
      new THREE.LineBasicMaterial({ color, transparent: opacity < 1, opacity }),
    )
    this.line.position.y = height
    this.line.frustumCulled = false
  }

  set(path: PathPose[] | null): void {
    this.line.geometry.dispose()
    this.line.geometry = new THREE.BufferGeometry().setFromPoints((path ?? []).map((p) => new THREE.Vector3(p.x, 0, p.y)))
  }
}

function footprintCorners(p: Pose): THREE.Vector3[] {
  const c = Math.cos(p.theta)
  const s = Math.sin(p.theta)
  const pt = (lx: number, ly: number) => new THREE.Vector3(p.x + c * lx - s * ly, 0, p.y + s * lx + c * ly)
  const { back, front, half } = FOOTPRINT
  return [pt(back, -half), pt(front, -half), pt(front, half), pt(back, half)]
}

/** The car outline stamped by the simulation every 12 cm: the green "ribbon" of the video. */
export class FootprintTrail {
  readonly lines: THREE.LineSegments
  private count = 0
  private positions: Float32Array

  constructor(private max = 6000) {
    this.positions = new Float32Array(max * 8 * 3)
    const geo = new THREE.BufferGeometry()
    geo.setAttribute('position', new THREE.BufferAttribute(this.positions, 3))
    geo.setDrawRange(0, 0)
    this.lines = new THREE.LineSegments(
      geo,
      new THREE.LineBasicMaterial({ color: EXPLORE_COLORS.footprint, transparent: true, opacity: 0.55 }),
    )
    this.lines.position.y = 0.015
    this.lines.frustumCulled = false
  }

  push(p: Pose): void {
    if (this.count >= this.max) return
    const k = footprintCorners(p)
    const seq = [k[0], k[1], k[1], k[2], k[2], k[3], k[3], k[0]]
    seq.forEach((v, i) => this.positions.set([v.x, 0, v.z], (this.count * 8 + i) * 3))
    this.count++
    this.lines.geometry.attributes.position.needsUpdate = true
    this.lines.geometry.setDrawRange(0, this.count * 8)
  }

  clear(): void {
    this.count = 0
    this.lines.geometry.setDrawRange(0, 0)
  }
}

/** Goal: a parking spot (car outline + arrow) when the heading matters, a pulsing ring otherwise. */
export class GoalMarker {
  readonly group = new THREE.Group()
  private spot: THREE.Group
  private ring: THREE.Mesh

  constructor() {
    const { back, front, half } = FOOTPRINT
    const shape = new THREE.Shape()
    shape.moveTo(back, -half)
    shape.lineTo(front, -half)
    shape.lineTo(front, half)
    shape.lineTo(back, half)
    shape.closePath()
    const fill = new THREE.Mesh(
      new THREE.ShapeGeometry(shape).rotateX(Math.PI / 2),
      new THREE.MeshBasicMaterial({ color: EXPLORE_COLORS.goal, transparent: true, opacity: 0.35, side: THREE.DoubleSide, depthWrite: false }),
    )
    const arrow = new THREE.Line(
      new THREE.BufferGeometry().setFromPoints([
        new THREE.Vector3(0.05, 0, 0), new THREE.Vector3(0.6, 0, 0),
        new THREE.Vector3(0.45, 0, -0.1), new THREE.Vector3(0.6, 0, 0), new THREE.Vector3(0.45, 0, 0.1),
      ]),
      new THREE.LineBasicMaterial({ color: 0xffffff }),
    )
    this.spot = new THREE.Group()
    this.spot.add(fill, arrow)
    this.ring = new THREE.Mesh(
      new THREE.RingGeometry(0.25, 0.32, 40).rotateX(-Math.PI / 2),
      new THREE.MeshBasicMaterial({ color: EXPLORE_COLORS.goal, transparent: true, depthWrite: false }),
    )
    this.group.add(this.spot, this.ring)
    this.group.position.y = 0.02
    this.group.visible = false
  }

  set(goal: ExploreGoal | null): void {
    this.group.visible = !!goal
    if (!goal) return
    this.group.position.set(goal.x, 0.02, goal.y)
    this.spot.visible = goal.useHeading
    this.ring.visible = !goal.useHeading
    this.spot.rotation.y = -goal.theta
  }

  animate(time: number): void {
    const pulse = (time * 0.8) % 1
    this.ring.scale.setScalar(1 + pulse * 1.5)
    ;(this.ring.material as THREE.MeshBasicMaterial).opacity = 1 - pulse
  }
}

/** What the MPC is thinking: its predicted trajectory (green) and the reference it tracks (yellow). */
export class MpcView {
  readonly group = new THREE.Group()
  private line: THREE.Line
  private points: THREE.Points

  constructor() {
    this.line = new THREE.Line(new THREE.BufferGeometry(), new THREE.LineBasicMaterial({ color: EXPLORE_COLORS.mpcPrediction }))
    this.points = new THREE.Points(
      new THREE.BufferGeometry(),
      new THREE.PointsMaterial({ color: EXPLORE_COLORS.mpcReference, size: 0.06 }),
    )
    this.line.position.y = 0.09
    this.points.position.y = 0.08
    this.line.frustumCulled = this.points.frustumCulled = false
    this.group.add(this.line, this.points)
  }

  set(predicted: Pose[], reference: Pose[]): void {
    this.line.geometry.dispose()
    this.points.geometry.dispose()
    this.line.geometry = new THREE.BufferGeometry().setFromPoints(predicted.map((p) => new THREE.Vector3(p.x, 0, p.y)))
    this.points.geometry = new THREE.BufferGeometry().setFromPoints(reference.map((p) => new THREE.Vector3(p.x, 0, p.y)))
  }
}
