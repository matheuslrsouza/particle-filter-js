import * as THREE from 'three'

export const COLORS = {
  background: 0x0b0f16,
  floor: 0x161d29,
  wall: 0xd9e1ec,
  car: 0xe63946,
  belief: 0x39ff88,
  particle: 0x39ff88,
  expanded: 0x2f6bff,
  rawPath: 0xff2e88,
  smoothPath: 0xf2f7ff,
  lidarHit: 0x39ff88,
  goal: 0xffb703,
}

/** Sim coordinates (x, y) live on the three.js ground plane as (x, 0, y). */
export const toVec3 = (x: number, y: number, h = 0) => new THREE.Vector3(x, h, y)

export function createRenderer(container: HTMLElement): THREE.WebGLRenderer {
  const renderer = new THREE.WebGLRenderer({ antialias: true })
  renderer.setPixelRatio(Math.min(window.devicePixelRatio, 2))
  renderer.setSize(container.clientWidth, container.clientHeight)
  renderer.shadowMap.enabled = true
  renderer.shadowMap.type = THREE.PCFSoftShadowMap
  renderer.toneMapping = THREE.ACESFilmicToneMapping
  renderer.outputColorSpace = THREE.SRGBColorSpace
  container.appendChild(renderer.domElement)
  return renderer
}

/** Anything with a size in meters: a maze, a city... */
export interface Area {
  width: number
  height: number
}

export function createScene(world: Area): THREE.Scene {
  const scene = new THREE.Scene()
  scene.background = new THREE.Color(COLORS.background)
  scene.fog = new THREE.Fog(COLORS.background, 35, 80)

  scene.add(new THREE.HemisphereLight(0xc4d7ff, 0x1a1f29, 1.1))

  const cx = world.width / 2
  const cz = world.height / 2
  const sun = new THREE.DirectionalLight(0xffffff, 2.4)
  sun.position.set(cx + 10, 25, cz - 12)
  sun.target.position.set(cx, 0, cz)
  sun.castShadow = true
  sun.shadow.mapSize.set(2048, 2048)
  const s = Math.max(world.width, world.height) * 0.75
  Object.assign(sun.shadow.camera, { left: -s, right: s, top: s, bottom: -s, near: 1, far: 80 })
  sun.shadow.bias = -0.0005
  scene.add(sun, sun.target)
  return scene
}
