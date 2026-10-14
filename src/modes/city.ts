import GUI from 'lil-gui'
import * as THREE from 'three'
import type { CityDef } from '../core/city/cityDef'
import { buildLaneGraph } from '../core/city/roadNetwork'
import { CameraRig, type CameraMode } from '../render/cameras'
import { createCity3d, laneGraphView } from '../render/city3d'
import { createRenderer, createScene } from '../render/scene'
import { Hud } from '../ui/hud'
import { addModeSelector } from '../ui/mode'

export async function startCity(): Promise<void> {
  // the city data is only downloaded when this mode is opened
  const city = (await import('../maps/city-mountain-view.json')).default as unknown as CityDef
  const graph = buildLaneGraph(city)

  const container = document.getElementById('app')!
  const renderer = createRenderer(container)
  const scene = createScene(city)
  scene.background = new THREE.Color(0xbcd4e6)
  scene.fog = new THREE.Fog(0xbcd4e6, 900, 3200)
  const rig = new CameraRig(renderer.domElement, city, { near: 0.5, far: 5000, maxDistance: 1600, minDistance: 5 })
  const hud = new Hud(`Downtown Mountain View, CA · <b>drag</b> to orbit · <b>scroll</b> to zoom · map data ${city.attribution}`)
  document.getElementById('charts')!.style.display = 'none'

  const city3d = createCity3d(city, graph)
  const lanes = laneGraphView(graph)
  lanes.visible = false
  scene.add(city3d.group, lanes)

  const view = { camera: 'orbit' as CameraMode, buildings: true, trees: true, laneGraph: false }
  const gui = new GUI({ title: 'Controls' })
  if (window.innerWidth < 640) gui.close()
  addModeSelector(gui, 'city')
  gui.add(view, 'camera', { Orbit: 'orbit', 'Top-down': 'top' }).name('Camera').onChange((m: CameraMode) => rig.setMode(m))
  const layers = gui.addFolder('Layers')
  layers.add(view, 'buildings').name('Buildings').onChange((v: boolean) => (city3d.buildings.visible = v))
  layers.add(view, 'trees').name('Trees').onChange((v: boolean) => (city3d.trees.visible = v))
  layers.add(view, 'laneGraph').name('Lane graph (debug)').onChange((v: boolean) => (lanes.visible = v))

  hud.setStatus(
    `${city.name} · ${city.roads.length} OSM roads → ${graph.lanes.length} lanes, ${graph.connectors.length} junction connectors · ${city.buildings.length} buildings`,
  )

  window.addEventListener('resize', () => {
    renderer.setSize(container.clientWidth, container.clientHeight)
    rig.resize(container.clientWidth, container.clientHeight)
  })

  let last = performance.now()
  const frame = (now: number) => {
    const dt = Math.min(0.1, (now - last) / 1000)
    last = now
    rig.update({ x: city.width / 2, y: city.height / 2, theta: 0 }, dt)
    renderer.render(scene, rig.camera)
    requestAnimationFrame(frame)
  }
  requestAnimationFrame(frame)
}
