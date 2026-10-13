import GUI from 'lil-gui'
import type { ExplorationSim } from '../core/sim/exploration'
import type { CameraMode } from '../render/cameras'
import { addModeSelector } from './mode'

export interface ExploreView {
  paused: boolean
  timeScale: number
  camera: CameraMode
  map: string
  dragSetsHeading: boolean
  layers: {
    mpc: boolean
    tree: boolean
    rawPlan: boolean
    plan: boolean
    previousPlan: boolean
    footprints: boolean
    lidar: boolean
    fog: boolean
    realWalls: boolean
  }
}

export function createExplorePanel(
  sim: ExplorationSim,
  view: ExploreView,
  hooks: { onCamera: (m: CameraMode) => void; onReset: () => void; onRealWalls: (v: boolean) => void; onController: () => void },
): GUI {
  const gui = new GUI({ title: 'Controls' })
  if (window.innerWidth < 640) gui.close()
  addModeSelector(gui, 'explore')
  gui
    .add(view, 'map', { Maze: 'maze', 'Udacity map': 'udacity' })
    .name('Map')
    .onChange((m: string) => {
      const url = new URL(location.href)
      url.searchParams.set('map', m)
      location.href = url.toString()
    })

  const simF = gui.addFolder('Simulation')
  simF.add(view, 'paused').name('Pause')
  simF.add(view, 'timeScale', 0.25, 4, 0.25).name('Speed ×')
  simF.add({ goal: () => sim.randomGoal() }, 'goal').name('Random goal')
  simF.add({ reset: hooks.onReset }, 'reset').name('Reset (forget map)')
  simF.add(view, 'dragSetsHeading').name('Drag sets goal heading')
  simF.add(view, 'camera', { Orbit: 'orbit', Chase: 'chase', 'Top-down': 'top' }).name('Camera').onChange(hooks.onCamera)

  const layers = gui.addFolder('Layers')
  const names: Record<keyof ExploreView['layers'], string> = {
    mpc: 'MPC prediction / reference',
    tree: 'Search tree (orange fwd / blue rev)',
    rawPlan: 'Raw Hybrid A* path',
    plan: 'Current plan (smoothed)',
    previousPlan: 'Previous plan',
    footprints: 'Car footprints',
    lidar: 'Lidar beams',
    fog: 'Unexplored fog',
    realWalls: 'Real walls (cheat)',
  }
  for (const k of Object.keys(names) as (keyof ExploreView['layers'])[]) {
    const c = layers.add(view.layers, k).name(names[k])
    if (k === 'realWalls') c.onChange(hooks.onRealWalls)
  }

  const ctl = gui.addFolder('Controller')
  ctl.add(sim.params, 'controller', { 'MPC (iLQR)': 'mpc', 'PID (course)': 'pid' }).name('Controller').onChange(hooks.onController)
  ctl.add(sim.mpc, 'horizon', 5, 40, 1).name('MPC horizon N').onFinishChange(hooks.onController)
  ctl.add(sim.mpc, 'aLat', 0.3, 3, 0.1).name('Max lateral accel').onFinishChange(hooks.onController)
  ctl.add(sim.mpc.weights, 'lateral', 0, 100, 1).name('Weight: CTE')
  ctl.add(sim.mpc.weights, 'heading', 0, 50, 0.5).name('Weight: heading')
  ctl.add(sim.mpc.weights, 'speed', 0, 20, 0.5).name('Weight: speed')
  ctl.add(sim.mpc.weights, 'steerRate', 0, 5, 0.05).name('Weight: steer rate')
  ctl.add(sim.params.gains, 'kp', 0, 10, 0.01).name('PID Kp')
  ctl.add(sim.params.gains, 'kd', 0, 5, 0.01).name('PID Kd')
  ctl.add(sim.params.gains, 'ki', 0, 2, 0.01).name('PID Ki')
  ctl.close()

  const car = gui.addFolder('Car & sensors')
  car.add(sim.params, 'forwardSpeed', 0.3, 3, 0.1).name('Forward (m/s)')
  car.add(sim.params, 'reverseSpeed', 0.2, 1.5, 0.1).name('Reverse (m/s)')
  car.add(sim.params, 'lidarRange', 1.5, 12, 0.5).name('Lidar range (m)')
  car.add(sim.params, 'gpsNoise', 0, 0.2, 0.01).name('GPS noise (m)')
  car.close()

  const plan = gui.addFolder('Hybrid A*')
  plan.add(sim.smoothing, 'enabled').name('Smooth the path')
  plan.add(sim.smoothing, 'weightSmooth', 0, 0.5, 0.01).name('Smoothing weight')
  plan.add(sim.hybrid, 'heuristicWeight', 1, 3, 0.1).name('Heuristic weight')
  plan.add(sim.hybrid, 'reverseCost', 1, 6, 0.1).name('Reverse cost ×')
  plan.add(sim.hybrid, 'switchCost', 0, 10, 0.5).name('Gear switch cost')
  plan.add(sim.hybrid, 'radius', 0.25, 0.6, 0.01).name('Safety radius (m)')
  plan.close()

  return gui
}
