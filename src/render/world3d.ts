import * as THREE from 'three'
import { INFLATED, WALL, type OccupancyGrid } from '../core/world/grid'
import type { World } from '../core/world/map'
import { COLORS } from './scene'

const WALL_HEIGHT = 0.6
const WALL_THICKNESS = 0.12

function floorTexture(world: World): THREE.CanvasTexture {
  const ppm = 32 // pixels per meter
  const canvas = document.createElement('canvas')
  canvas.width = world.width * ppm
  canvas.height = world.height * ppm
  const ctx = canvas.getContext('2d')!
  ctx.fillStyle = '#161d29'
  ctx.fillRect(0, 0, canvas.width, canvas.height)
  for (let x = 0; x <= world.width; x++) {
    ctx.strokeStyle = x % 5 === 0 ? 'rgba(140,160,190,0.35)' : 'rgba(140,160,190,0.12)'
    ctx.beginPath()
    ctx.moveTo(x * ppm, 0)
    ctx.lineTo(x * ppm, canvas.height)
    ctx.stroke()
  }
  for (let y = 0; y <= world.height; y++) {
    ctx.strokeStyle = y % 5 === 0 ? 'rgba(140,160,190,0.35)' : 'rgba(140,160,190,0.12)'
    ctx.beginPath()
    ctx.moveTo(0, y * ppm)
    ctx.lineTo(canvas.width, y * ppm)
    ctx.stroke()
  }
  const tex = new THREE.CanvasTexture(canvas)
  tex.colorSpace = THREE.SRGBColorSpace
  tex.anisotropy = 8
  return tex
}

export function createWorld3d(world: World): { group: THREE.Group; floor: THREE.Mesh } {
  const group = new THREE.Group()

  const floor = new THREE.Mesh(
    new THREE.PlaneGeometry(world.width, world.height),
    new THREE.MeshStandardMaterial({ map: floorTexture(world), roughness: 0.9 }),
  )
  floor.rotation.x = -Math.PI / 2
  floor.position.set(world.width / 2, 0, world.height / 2)
  floor.receiveShadow = true
  group.add(floor)

  const ground = new THREE.Mesh(
    new THREE.PlaneGeometry(400, 400),
    new THREE.MeshStandardMaterial({ color: 0x0e131b, roughness: 1 }),
  )
  ground.rotation.x = -Math.PI / 2
  ground.position.set(world.width / 2, -0.01, world.height / 2)
  ground.receiveShadow = true
  group.add(ground)

  const walls = new THREE.InstancedMesh(
    new THREE.BoxGeometry(1, 1, 1),
    new THREE.MeshStandardMaterial({ color: COLORS.wall, roughness: 0.55, metalness: 0.05 }),
    world.walls.length,
  )
  const m = new THREE.Matrix4()
  const q = new THREE.Quaternion()
  const up = new THREE.Vector3(0, 1, 0)
  world.walls.forEach((w, i) => {
    const dx = w.b.x - w.a.x
    const dy = w.b.y - w.a.y
    const len = Math.hypot(dx, dy)
    q.setFromAxisAngle(up, -Math.atan2(dy, dx))
    m.compose(
      new THREE.Vector3((w.a.x + w.b.x) / 2, WALL_HEIGHT / 2, (w.a.y + w.b.y) / 2),
      q,
      new THREE.Vector3(len + WALL_THICKNESS, WALL_HEIGHT, WALL_THICKNESS),
    )
    walls.setMatrixAt(i, m)
  })
  walls.castShadow = true
  walls.receiveShadow = true
  group.add(walls)

  return { group, floor }
}

/** Debug overlay of the occupancy grid used by A*: walls and inflated cells. */
export function createGrid3d(grid: OccupancyGrid): THREE.Group {
  const group = new THREE.Group()
  const geo = new THREE.PlaneGeometry(grid.cellSize * 0.92, grid.cellSize * 0.92).rotateX(-Math.PI / 2)
  const layers: [number, number, number][] = [
    [WALL, 0x39ff88, 0.55],
    [INFLATED, 0x39ff88, 0.14],
  ]
  for (const [value, color, opacity] of layers) {
    const idx: number[] = []
    grid.cells.forEach((v, i) => v === value && idx.push(i))
    const mesh = new THREE.InstancedMesh(
      geo,
      new THREE.MeshBasicMaterial({ color, transparent: true, opacity, depthWrite: false }),
      idx.length,
    )
    const m = new THREE.Matrix4()
    idx.forEach((i, k) => {
      const c = grid.cellCenter(i % grid.cols, Math.floor(i / grid.cols))
      mesh.setMatrixAt(k, m.makeTranslation(c.x, 0.012, c.y))
    })
    group.add(mesh)
  }
  return group
}
