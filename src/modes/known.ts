import * as THREE from 'three'
import { wrapAngle } from '../core/math/angles'
import type { Pose } from '../core/math/vec'
import { Simulation, VEHICLE } from '../core/sim/simulation'
import type { MapDef } from '../core/world/map'
import mapDef from '../maps/default.json'
import { Belief3d } from '../render/belief3d'
import { CameraRig } from '../render/cameras'
import { Car3d } from '../render/car3d'
import { Lidar3d } from '../render/lidar3d'
import { Particles3d } from '../render/particles3d'
import { Plan3d } from '../render/plan3d'
import { COLORS, createRenderer, createScene } from '../render/scene'
import { Trail3d } from '../render/trail3d'
import { createGrid3d, createWorld3d } from '../render/world3d'
import { Hud } from '../ui/hud'
import { createPanel, type ViewState } from '../ui/panel'

const PHASE_TEXT = {
  idle: 'Waiting for a goal',
  planning: 'Planning with A*…',
  driving: 'Driving (PID on the estimated pose)',
  arrived: 'Arrived!',
}

export function startKnown(): void {
  const sim = new Simulation(mapDef as unknown as MapDef)
  const container = document.getElementById('app')!
  const renderer = createRenderer(container)
  const scene = createScene(sim.world)
  const rig = new CameraRig(renderer.domElement, sim.world)
  const hud = new Hud()
  sim.onMessage = (m) => hud.message(m)

  const { group: worldGroup, floor } = createWorld3d(sim.world)
  const grid3d = createGrid3d(sim.grid)
  const car = new Car3d(VEHICLE.wheelbase)
  const ghost = new Car3d(VEHICLE.wheelbase, true)
  const particles = new Particles3d()
  const lidar = new Lidar3d(sim.lidar)
  const belief = new Belief3d()
  const plan = new Plan3d(sim.grid)
  const trueTrail = new Trail3d(COLORS.car)
  const estTrail = new Trail3d(COLORS.belief, 1500, 0.03)
  scene.add(worldGroup, grid3d, plan.group, particles.mesh, lidar.group, belief.mesh, ghost.group, car.group, trueTrail.line, estTrail.line)

  const view: ViewState = {
    paused: false,
    timeScale: 1,
    camera: 'orbit',
    layers: {
      particles: true,
      lidar: true,
      belief: true,
      astar: true,
      rawPath: true,
      smoothPath: true,
      grid: false,
      trails: true,
    },
  }
  createPanel(sim, view, { onCamera: (m) => rig.setMode(m), message: (m) => hud.message(m) })

  // --- click on the floor to set a goal ---------------------------------------------------
  const raycaster = new THREE.Raycaster()
  const pointer = new THREE.Vector2()
  const cursor = new THREE.Mesh(
    new THREE.RingGeometry(0.18, 0.24, 32).rotateX(-Math.PI / 2),
    new THREE.MeshBasicMaterial({ color: COLORS.goal, transparent: true, opacity: 0.6, depthWrite: false }),
  )
  cursor.visible = false
  scene.add(cursor)

  function floorHit(e: PointerEvent): THREE.Vector3 | null {
    const r = renderer.domElement.getBoundingClientRect()
    pointer.set(((e.clientX - r.left) / r.width) * 2 - 1, -((e.clientY - r.top) / r.height) * 2 + 1)
    raycaster.setFromCamera(pointer, rig.camera)
    return raycaster.intersectObject(floor)[0]?.point ?? null
  }

  let down: { x: number; y: number } | null = null
  renderer.domElement.addEventListener('pointerdown', (e) => (down = { x: e.clientX, y: e.clientY }))
  renderer.domElement.addEventListener('pointerup', (e) => {
    if (!down || e.button !== 0) return
    const moved = Math.hypot(e.clientX - down.x, e.clientY - down.y)
    down = null
    if (moved > 6) return
    const p = floorHit(e)
    if (p) sim.setGoal({ x: p.x, y: p.z })
  })
  renderer.domElement.addEventListener('pointermove', (e) => {
    if (e.pointerType !== 'mouse') return
    const p = floorHit(e)
    cursor.visible = !!p
    if (p) cursor.position.set(p.x, 0.02, p.z)
  })

  window.addEventListener('resize', () => {
    renderer.setSize(container.clientWidth, container.clientHeight)
    rig.resize(container.clientWidth, container.clientHeight)
  })

  // --- main loop: fixed-timestep simulation, interpolated rendering --------------------------
  let planVersion = -1
  let filterVersion = -1
  let scanRef: Float64Array | null = null
  let acc = 0
  let last = performance.now()
  const interp: Pose = { x: 0, y: 0, theta: 0 }

  function frame(now: number) {
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

    if (sim.planVersion !== planVersion) {
      planVersion = sim.planVersion
      plan.setPlan(sim.plan)
    }
    if (sim.filterVersion !== filterVersion) {
      filterVersion = sim.filterVersion
      particles.update(sim.pf)
    }
    if (sim.lastScan && sim.lastScan !== scanRef && sim.lastScanPose) {
      scanRef = sim.lastScan
      lidar.update(sim.lastScanPose, sim.lastScan)
    }

    const L = view.layers
    car.update(interp, sim.car.steer, sim.car.odometer, view.paused ? 0 : dt)
    const est = sim.pf.estimate
    ghost.update(est, sim.car.steer, sim.car.odometer, 0)
    belief.update(est)
    plan.animate(sim.phase === 'planning', sim.phaseTime, sim.params.planSeconds, now / 1000)
    trueTrail.push(interp)
    estTrail.push(est)

    particles.mesh.visible = L.particles
    lidar.group.visible = L.lidar && !!sim.lastScan
    belief.mesh.visible = ghost.group.visible = L.belief
    plan.expanded.visible = L.astar
    plan.raw.visible = L.rawPath
    plan.smooth.visible = L.smoothPath
    grid3d.visible = L.grid
    trueTrail.line.visible = estTrail.line.visible = L.trails

    rig.update(interp, dt)
    const err = Math.hypot(est.x - sim.car.x, est.y - sim.car.y)
    hud.setStatus(`${PHASE_TEXT[sim.phase]} · localization error ${(err * 100).toFixed(0)} cm · Neff ${(est.neff * 100).toFixed(0)}%`)
    hud.chart(
      [
        { label: 'CTE (m)', color: '#ff2e88', values: sim.telemetry.map((d) => d.cte), range: 1 },
        { label: 'loc. error (m)', color: '#39ff88', values: sim.telemetry.map((d) => d.locError), range: 1 },
      ],
      '±1 m · last 30 s',
    )
    renderer.render(scene, rig.camera)
    requestAnimationFrame(frame)
  }
  requestAnimationFrame(frame)
}
