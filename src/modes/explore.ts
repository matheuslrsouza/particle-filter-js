import * as THREE from 'three'
import { wrapAngle } from '../core/math/angles'
import type { Pose } from '../core/math/vec'
import { MpcFollower } from '../core/control/mpc'
import { ExplorationSim, type ExplorePhase } from '../core/sim/exploration'
import { VEHICLE } from '../core/sim/simulation'
import type { MapDef } from '../core/world/map'
import udacityMap from '../maps/default.json'
import mazeMap from '../maps/maze.json'
import { CameraRig } from '../render/cameras'
import { Car3d } from '../render/car3d'
import {
  DiscoveredWalls,
  EXPLORE_COLORS,
  FootprintTrail,
  Fog,
  GoalMarker,
  MpcView,
  PlanLine,
  SearchTree,
} from '../render/explore3d'
import { Lidar3d } from '../render/lidar3d'
import { createRenderer, createScene } from '../render/scene'
import { createWorld3d } from '../render/world3d'
import { createExplorePanel, type ExploreView } from '../ui/explorePanel'
import { Hud } from '../ui/hud'

const PHASE_TEXT: Record<ExplorePhase, string> = {
  idle: 'Click the floor to set a goal',
  driving: 'Exploring',
  switching: 'Switching gears',
  recovering: 'Bumped! Backing off',
  arrived: 'Goal reached!',
  blocked: 'No path with what the car knows',
}

export function startExplore(): void {
  const mapName = new URLSearchParams(location.search).get('map') === 'udacity' ? 'udacity' : 'maze'
  const mapDef = (mapName === 'udacity' ? udacityMap : mazeMap) as unknown as MapDef
  const sim = new ExplorationSim(mapDef)

  const container = document.getElementById('app')!
  const renderer = createRenderer(container)
  const scene = createScene(sim.world)
  const rig = new CameraRig(renderer.domElement, sim.world)
  const hud = new Hud('<b>Click</b> the floor to set a goal · <b>Shift+drag</b> also sets the heading · <b>drag</b> to orbit')
  sim.onMessage = (m) => hud.message(m)

  // the real walls exist but the car doesn't know them: hidden unless "cheat" is on
  const { group: worldGroup, floor, walls: realWalls } = createWorld3d(sim.world)
  const realWallMat = realWalls.material as THREE.MeshStandardMaterial
  realWallMat.transparent = true
  const setRealWalls = (on: boolean) => {
    realWallMat.opacity = on ? 0.18 : 0
    realWallMat.depthWrite = on
    realWalls.castShadow = false
    realWalls.visible = on
  }

  const car = new Car3d(VEHICLE.wheelbase)
  const lidar = new Lidar3d(sim.lidar)
  const discovered = new DiscoveredWalls(sim.known)
  const fog = new Fog(sim.known)
  const tree = new SearchTree()
  const plan = new PlanLine(EXPLORE_COLORS.plan, 0.06)
  const rawPlan = new PlanLine(EXPLORE_COLORS.rawPlan, 0.045, 0.6)
  const previous = new PlanLine(EXPLORE_COLORS.previousPlan, 0.05, 0.7)
  const footprints = new FootprintTrail()
  const goal = new GoalMarker()
  const mpcView = new MpcView()
  scene.add(worldGroup, fog.mesh, discovered.mesh, footprints.lines, tree.group, previous.line, rawPlan.line, plan.line, goal.group, mpcView.group, lidar.group, car.group)

  const view: ExploreView = {
    paused: false,
    // exploring the whole maze takes a few minutes of simulated time
    timeScale: 2,
    camera: 'orbit',
    map: mapName,
    dragSetsHeading: false,
    layers: { mpc: true, tree: true, rawPlan: true, plan: true, previousPlan: true, footprints: true, lidar: true, fog: true, realWalls: false },
  }
  setRealWalls(false)

  const resetAll = () => {
    sim.reset()
    discovered.clear()
    footprints.clear()
    tree.clear()
    fog.update(-1)
  }
  createExplorePanel(sim, view, {
    onCamera: (m) => rig.setMode(m),
    onReset: resetAll,
    onRealWalls: setRealWalls,
    onController: () => sim.forceReplan(),
  })

  // --- goal input: click = goal anywhere; shift+drag (or the panel toggle) = goal + heading ---
  const raycaster = new THREE.Raycaster()
  const ndc = new THREE.Vector2()
  const floorHit = (e: PointerEvent): THREE.Vector3 | null => {
    const r = renderer.domElement.getBoundingClientRect()
    ndc.set(((e.clientX - r.left) / r.width) * 2 - 1, -((e.clientY - r.top) / r.height) * 2 + 1)
    raycaster.setFromCamera(ndc, rig.camera)
    return raycaster.intersectObject(floor)[0]?.point ?? null
  }
  const preview = new THREE.ArrowHelper(new THREE.Vector3(1, 0, 0), new THREE.Vector3(), 1, EXPLORE_COLORS.goal)
  preview.visible = false
  scene.add(preview)

  let down: { x: number; y: number; floor: THREE.Vector3 | null; heading: boolean } | null = null
  renderer.domElement.addEventListener('pointerdown', (e) => {
    const heading = e.button === 0 && (e.shiftKey || view.dragSetsHeading)
    const p = floorHit(e)
    down = { x: e.clientX, y: e.clientY, floor: p, heading: heading && !!p }
    if (down.heading) {
      rig.controls.enabled = false
      preview.position.set(p!.x, 0.1, p!.z)
      preview.visible = true
      preview.setLength(0.01)
    }
  })
  renderer.domElement.addEventListener('pointermove', (e) => {
    if (!down?.heading || !down.floor) return
    const p = floorHit(e)
    if (!p) return
    const d = new THREE.Vector3(p.x - down.floor.x, 0, p.z - down.floor.z)
    if (d.length() > 0.05) {
      preview.setDirection(d.clone().normalize())
      preview.setLength(Math.max(0.6, d.length()), 0.25, 0.15)
    }
  })
  renderer.domElement.addEventListener('pointerup', (e) => {
    if (!down) return
    const start = down
    down = null
    preview.visible = false
    rig.controls.enabled = view.camera !== 'chase'
    if (start.heading && start.floor) {
      const p = floorHit(e) ?? start.floor
      const dx = p.x - start.floor.x
      const dz = p.z - start.floor.z
      const useHeading = Math.hypot(dx, dz) > 0.3
      sim.setGoal({ x: start.floor.x, y: start.floor.z, theta: Math.atan2(dz, dx), useHeading })
      return
    }
    if (Math.hypot(e.clientX - start.x, e.clientY - start.y) > 6) return
    const p = floorHit(e)
    if (p) sim.setGoal({ x: p.x, y: p.z, theta: 0, useHeading: false })
  })

  window.addEventListener('resize', () => {
    renderer.setSize(container.clientWidth, container.clientHeight)
    rig.resize(container.clientWidth, container.clientHeight)
  })

  // the maze comes with a goal, so the demo starts by itself
  if (sim.world.goal) setTimeout(() => sim.setGoal({ ...sim.world.goal!, useHeading: true }), 800)

  // --- main loop ----------------------------------------------------------------------------
  let planVersion = -1
  let scanRef: Float64Array | null = null
  let acc = 0
  let last = performance.now()
  const interp: Pose = { x: 0, y: 0, theta: 0 }

  function frame(now: number) {
    const dt = Math.min(0.1, (now - last) / 1000)
    last = now
    const t = now / 1000
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

    if (sim.removedWalls.length) {
      // evidence erased some walls (log-odds): redraw from the map, the new ones included
      discovered.rebuild()
      sim.removedWalls = []
      sim.discovered = []
    }
    if (sim.discovered.length) {
      discovered.add(sim.discovered, t)
      sim.discovered = []
    }
    discovered.animate(t)
    fog.update(sim.scanVersion)
    if (sim.planVersion !== planVersion) {
      planVersion = sim.planVersion
      plan.set(sim.plan?.path ?? null)
      rawPlan.set(sim.plan?.raw ?? null)
      previous.set(sim.previousPlan)
      if (sim.plan) tree.set(sim.plan.result.treeForward, sim.plan.result.treeReverse, t, sim.plan.result.analytic)
      goal.set(sim.goal)
    }
    tree.animate(t)
    goal.animate(t)
    if (sim.lastScan && sim.lastScan !== scanRef && sim.lastScanPose) {
      scanRef = sim.lastScan
      lidar.update(sim.lastScanPose, sim.lastScan)
    }
    car.update(interp, sim.car.steer, sim.car.odometer, view.paused ? 0 : dt)
    const f = sim.follower
    const mpcActive = f instanceof MpcFollower && sim.phase === 'driving'
    if (mpcActive) mpcView.set(f.predicted(), f.reference)
    mpcView.group.visible = view.layers.mpc && mpcActive
    for (const f of sim.footprints) footprints.push(f)
    sim.footprints = []

    const L = view.layers
    tree.group.visible = L.tree
    plan.line.visible = L.plan
    rawPlan.line.visible = L.rawPlan
    previous.line.visible = L.previousPlan
    footprints.lines.visible = L.footprints
    lidar.group.visible = L.lidar && !!sim.lastScan
    fog.mesh.visible = L.fog

    rig.update(interp, dt)
    const r = sim.plan?.result
    const planInfo = r ? ` · last plan ${r.millis.toFixed(0)} ms, ${r.expansions} nodes` : ''
    const ctl = sim.follower instanceof MpcFollower ? ` · MPC ${sim.follower.solver.lastMillis.toFixed(1)} ms` : sim.follower ? ' · PID' : ''
    hud.setStatus(`${PHASE_TEXT[sim.phase]}${ctl} · replans ${sim.replans}${planInfo} · explored ${(sim.known.explored() * 100).toFixed(0)}%`)
    hud.chart(
      [
        { label: 'speed', color: '#00e5ff', values: sim.telemetry.map((d) => d.speed), range: 2 },
        { label: 'ref', color: '#ffd60a', values: sim.telemetry.map((d) => d.refSpeed), range: 2 },
      ],
      `m/s · CTE ${(sim.follower?.cte ?? 0).toFixed(2)} m · last 30 s`,
    )
    renderer.render(scene, rig.camera)
    requestAnimationFrame(frame)
  }
  requestAnimationFrame(frame)
}
