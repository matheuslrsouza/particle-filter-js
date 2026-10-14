import GUI from 'lil-gui'
import * as THREE from 'three'
import type { CityDef } from '../core/city/cityDef'
import { CITY_VEHICLE, CitySim, type Localization } from '../core/city/citySim'
import type { GpsGrade } from '../core/sensors/gps'
import { wrapAngle } from '../core/math/angles'
import type { Pose } from '../core/math/vec'
import { Belief3d } from '../render/belief3d'
import { CameraRig, type CameraMode } from '../render/cameras'
import { Car3d } from '../render/car3d'
import { createCity3d, destinationPin, laneGraphView, routeRibbon } from '../render/city3d'
import { MpcView } from '../render/explore3d'
import { createRenderer, createScene } from '../render/scene'
import { Hud } from '../ui/hud'
import { addModeSelector } from '../ui/mode'

const KMH = 3.6

export async function startCity(): Promise<void> {
  // the city data is only downloaded when this mode is opened
  const city = (await import('../maps/city-mountain-view.json')).default as unknown as CityDef
  const sim = new CitySim(city)

  const container = document.getElementById('app')!
  const renderer = createRenderer(container)
  const scene = createScene(city)
  scene.background = new THREE.Color(0xbcd4e6)
  scene.fog = new THREE.Fog(0xbcd4e6, 900, 3200)
  const rig = new CameraRig(renderer.domElement, city, {
    near: 0.5,
    far: 5000,
    maxDistance: 1600,
    minDistance: 5,
    chase: { back: 11, up: 4.5, ahead: 8 },
  })
  const hud = new Hud(
    `<b>Click</b> a street to drive there · <b>drag</b> to orbit · map data <a href="https://www.openstreetmap.org/copyright" style="color:inherit">${city.attribution}</a>`,
  )
  sim.onMessage = (m) => hud.message(m)

  // the shadow camera follows the car, so shadows stay sharp in a 1 km city
  const sun = scene.children.find((o): o is THREE.DirectionalLight => o instanceof THREE.DirectionalLight)!
  Object.assign(sun.shadow.camera, { left: -80, right: 80, top: 80, bottom: -80, near: 1, far: 400 })
  sun.shadow.camera.updateProjectionMatrix()
  const sunOffset = new THREE.Vector3(60, 120, -80)

  const city3d = createCity3d(city, sim.graph)
  const lanes = laneGraphView(sim.graph)
  lanes.visible = false
  // the maze car is modeled with a 0.5 m wheelbase: scale it to a real car
  const car = new Car3d(0.5)
  car.group.scale.setScalar(CITY_VEHICLE.wheelbase / 0.5)
  // where the car believes it is (EKF estimate): wireframe ghost + 2σ ellipse + GPS fixes
  const ghost = new Car3d(0.5, true)
  ghost.group.scale.setScalar(CITY_VEHICLE.wheelbase / 0.5)
  const ellipse = new Belief3d()
  const gpsGeo = new THREE.BufferGeometry()
  const gpsPoints = new THREE.Points(
    gpsGeo,
    new THREE.PointsMaterial({ size: 0.6, vertexColors: true, depthTest: false, transparent: true, opacity: 0.9 }),
  )
  gpsPoints.renderOrder = 11
  gpsPoints.frustumCulled = false
  const pin = destinationPin()
  const mpcView = new MpcView()
  mpcView.group.position.y = 0.15
  let route: THREE.Mesh | null = null
  scene.add(city3d.group, lanes, car.group, ghost.group, ellipse.mesh, gpsPoints, pin, mpcView.group)

  const view = {
    camera: 'chase' as CameraMode,
    paused: false,
    timeScale: 1,
    buildings: true,
    trees: true,
    laneGraph: false,
    route: true,
    mpc: true,
    localization: 'ekf' as Localization,
    gps: 'dual' as GpsGrade,
    belief: true,
  }
  rig.setMode('chase')
  const gui = new GUI({ title: 'Controls' })
  if (window.innerWidth < 640) gui.close()
  addModeSelector(gui, 'city')
  const simF = gui.addFolder('Simulation')
  simF.add(view, 'paused').name('Pause')
  simF.add(view, 'timeScale', 0.25, 4, 0.25).name('Speed ×')
  simF.add({ go: () => sim.randomDestination() }, 'go').name('Random destination')
  simF.add({ reset: () => sim.reset() }, 'reset').name('Reset car')
  simF.add(view, 'camera', { Chase: 'chase', Orbit: 'orbit', 'Top-down': 'top' }).name('Camera').onChange((m: CameraMode) => rig.setMode(m))
  const loc = gui.addFolder('Localization')
  loc
    .add(view, 'localization', { 'EKF (GPS + odometry)': 'ekf', 'GPS only': 'gps', 'Odometry only': 'odometry', 'True pose (cheat)': 'truth' })
    .name('Car uses')
    .onChange((m: Localization) => (sim.localization = m))
  loc
    .add(view, 'gps', { 'RTK (~2 cm)': 'rtk', 'Dual-frequency (~0.5 m)': 'dual', 'Consumer (~1.5 m)': 'consumer' })
    .name('GPS receiver')
    .onChange((g: GpsGrade) => (sim.gpsGrade = g))
  loc.add(view, 'belief').name('Show estimate + GPS fixes')
  const layers = gui.addFolder('Layers')
  layers.add(view, 'route').name('Route')
  layers.add(view, 'mpc').name('MPC prediction / reference')
  layers.add(view, 'buildings').name('Buildings').onChange((v: boolean) => (city3d.buildings.visible = v))
  layers.add(view, 'trees').name('Trees').onChange((v: boolean) => (city3d.trees.visible = v))
  layers.add(view, 'laneGraph').name('Lane graph (debug)').onChange((v: boolean) => (lanes.visible = v))

  // --- click a street to drive there ---
  const raycaster = new THREE.Raycaster()
  const ndc = new THREE.Vector2()
  let down: { x: number; y: number } | null = null
  renderer.domElement.addEventListener('pointerdown', (e) => (down = { x: e.clientX, y: e.clientY }))
  renderer.domElement.addEventListener('pointerup', (e) => {
    if (!down || e.button !== 0) return
    const moved = Math.hypot(e.clientX - down.x, e.clientY - down.y)
    down = null
    if (moved > 6) return
    const r = renderer.domElement.getBoundingClientRect()
    ndc.set(((e.clientX - r.left) / r.width) * 2 - 1, -((e.clientY - r.top) / r.height) * 2 + 1)
    raycaster.setFromCamera(ndc, rig.camera)
    const hit = raycaster.intersectObject(city3d.floor)[0]
    if (hit) sim.setDestination({ x: hit.point.x, y: hit.point.z })
  })

  window.addEventListener('resize', () => {
    renderer.setSize(container.clientWidth, container.clientHeight)
    rig.resize(container.clientWidth, container.clientHeight)
  })

  // start with a drive, so the demo moves by itself
  setTimeout(() => sim.randomDestination(), 600)

  let routeVersion = -1
  let acc = 0
  let last = performance.now()
  const interp: Pose = { x: 0, y: 0, theta: 0 }
  const frame = (now: number) => {
    const dt = Math.min(0.1, (now - last) / 1000)
    last = now
    if (!view.paused) {
      acc += dt * view.timeScale
      let steps = 0
      while (acc >= sim.dt && steps++ < 20) {
        sim.tick()
        acc -= sim.dt
      }
      acc = Math.min(acc, sim.dt)
    }
    const alpha = acc / sim.dt
    const a = sim.prevCar
    const b = sim.car
    interp.x = a.x + (b.x - a.x) * alpha
    interp.y = a.y + (b.y - a.y) * alpha
    interp.theta = a.theta + wrapAngle(b.theta - a.theta) * alpha

    if (sim.routeVersion !== routeVersion) {
      routeVersion = sim.routeVersion
      if (route) {
        scene.remove(route)
        route.geometry.dispose()
      }
      route = sim.route ? routeRibbon(sim.route.path) : null
      if (route) scene.add(route)
      pin.visible = !!sim.destination
      if (sim.destination) pin.position.set(sim.destination.x, 0, sim.destination.y)
    }
    if (route) route.visible = view.route && sim.phase === 'driving'
    // the car model's units are the maze car's: its odometer must be scaled back for the wheels
    car.update(interp, sim.car.steer, sim.car.odometer / (CITY_VEHICLE.wheelbase / 0.5), view.paused ? 0 : dt)
    ghost.update(sim.estimate, sim.car.steer, 0, 0)
    ghost.group.visible = ellipse.mesh.visible = gpsPoints.visible = view.belief
    const [sxx, sxy, syy] = sim.ekf.cov
    ellipse.update({ ...sim.ekf.pose, cov: [sxx, sxy, syy], neff: 1 })
    ellipse.mesh.position.y = 0.3
    const pos = new Float32Array(sim.gpsMarks.length * 3)
    const col = new Float32Array(sim.gpsMarks.length * 3)
    sim.gpsMarks.forEach((m, i) => {
      pos.set([m.x, 0.3, m.y], i * 3)
      // accepted fixes in green, rejected (gated out) in red
      col.set(m.rejected ? [1, 0.2, 0.3] : [0.2, 1, 0.5], i * 3)
    })
    gpsGeo.setAttribute('position', new THREE.BufferAttribute(pos, 3))
    gpsGeo.setAttribute('color', new THREE.BufferAttribute(col, 3))
    const f = sim.follower
    const mpcActive = !!f && sim.phase === 'driving'
    if (mpcActive) mpcView.set(f!.predicted(), f!.reference)
    mpcView.group.visible = view.mpc && mpcActive

    sun.position.set(interp.x + sunOffset.x, sunOffset.y, interp.y + sunOffset.z)
    sun.target.position.set(interp.x, 0, interp.y)
    rig.update(interp, dt)

    const t = sim.telemetry[sim.telemetry.length - 1]
    const street = sim.street || 'Mountain View'
    const speed = `${(sim.car.speed * KMH).toFixed(0)} km/h`
    const limit = t?.limit ? ` (limit ${(t.limit * KMH).toFixed(0)})` : ''
    const togo = sim.phase === 'driving' ? ` · ${sim.remaining.toFixed(0)} m to go` : sim.phase === 'arrived' ? ' · arrived' : ''
    const mpc = f ? ` · MPC ${f.solver.lastMillis.toFixed(1)} ms` : ''
    hud.setStatus(`${street} · ${speed}${limit}${togo}${mpc}`)
    hud.chart(
      [
        { label: 'speed', color: '#00e5ff', values: sim.telemetry.map((d) => d.speed * KMH), range: 60 },
        { label: 'limit', color: '#ffd60a', values: sim.telemetry.map((d) => d.limit * KMH), range: 60 },
      ],
      `km/h · lane error ${((t?.lateral ?? 0) * 100).toFixed(0)} cm · loc error ${(sim.locError * 100).toFixed(0)} cm`,
    )
    renderer.render(scene, rig.camera)
    requestAnimationFrame(frame)
  }
  requestAnimationFrame(frame)
}
