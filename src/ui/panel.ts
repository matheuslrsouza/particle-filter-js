import GUI from 'lil-gui'
import type { PidGains } from '../core/control/pid'
import { VEHICLE, type Simulation } from '../core/sim/simulation'
import type { CameraMode } from '../render/cameras'
import { addModeSelector } from './mode'

export interface ViewState {
  paused: boolean
  timeScale: number
  camera: CameraMode
  layers: {
    particles: boolean
    lidar: boolean
    belief: boolean
    astar: boolean
    rawPath: boolean
    smoothPath: boolean
    grid: boolean
    trails: boolean
  }
}

export function createPanel(sim: Simulation, view: ViewState, hooks: { onCamera: (m: CameraMode) => void; message: (s: string) => void }): GUI {
  const gui = new GUI({ title: 'Controls' })
  if (window.innerWidth < 640) gui.close()
  const p = sim.params
  addModeSelector(gui, 'known')

  const simF = gui.addFolder('Simulation')
  simF.add(view, 'paused').name('Pause')
  simF.add(view, 'timeScale', 0.25, 4, 0.25).name('Speed ×')
  simF.add({ goal: () => sim.randomGoal() }, 'goal').name('Random goal')
  simF.add({ reset: () => sim.reset() }, 'reset').name('Reset')
  simF.add({ kidnap: () => sim.kidnap() }, 'kidnap').name('Kidnap robot')
  simF.add({ global: () => sim.globalLocalization() }, 'global').name('Forget position')
  simF.add(view, 'camera', { Orbit: 'orbit', Chase: 'chase', 'Top-down': 'top' }).name('Camera').onChange(hooks.onCamera)

  const layers = gui.addFolder('Layers')
  const names: Record<keyof ViewState['layers'], string> = {
    particles: 'Particles',
    lidar: 'Lidar beams',
    belief: 'Belief (ghost + 2σ)',
    astar: 'A* expanded cells',
    rawPath: 'A* path',
    smoothPath: 'Smoothed path',
    grid: 'Occupancy grid',
    trails: 'Trails (real / estimated)',
  }
  for (const k of Object.keys(names) as (keyof ViewState['layers'])[]) layers.add(view.layers, k).name(names[k])

  const pf = gui.addFolder('Particle filter')
  pf.add(p, 'particleCount', 50, 3000, 50).name('Particles').onFinishChange((n: number) => sim.setParticleCount(n))
  pf.add(p.filter, 'sensorSigma', 0.05, 1.5, 0.05).name('Sensor σ (m)')
  pf.add(p.filter, 'speedNoise', 0, 0.5, 0.01).name('Speed noise')
  pf.add(p.filter, 'steerNoise', 0, 0.3, 0.005).name('Steer noise (rad)')
  pf.add(p.filter, 'allowRecovery').name('Recovery (aug. MCL)')
  pf.close()

  const ctl = gui.addFolder('PID controller')
  ctl.add(p, 'cruiseSpeed', 0.3, 4, 0.1).name('Speed (m/s)')
  ctl.add(p.gains, 'kp', 0, 10, 0.01).name('Kp')
  ctl.add(p.gains, 'kd', 0, 5, 0.01).name('Kd')
  ctl.add(p.gains, 'ki', 0, 2, 0.01).name('Ki')
  ctl.add(p, 'lookahead', 0, 1.5, 0.05).name('Lookahead (m)')
  let worker: Worker | null = null
  const tune = {
    twiddle: () => {
      if (!sim.plan) return hooks.message('Set a goal first: twiddle tunes on the current path.')
      if (worker) return
      worker = new Worker(new URL('./twiddle.worker.ts', import.meta.url), { type: 'module' })
      hooks.message('Tuning PID with twiddle…')
      worker.onmessage = (e: MessageEvent<{ type: string; gains: PidGains; err?: number }>) => {
        Object.assign(p.gains, e.data.gains)
        ctl.controllersRecursive().forEach((c) => c.updateDisplay())
        if (e.data.type === 'done') {
          worker?.terminate()
          worker = null
          hooks.message('Twiddle done!')
        }
      }
      worker.postMessage({
        gains: { ...p.gains },
        setup: { path: sim.plan.smooth, vehicle: VEHICLE, speed: p.cruiseSpeed, lookahead: p.lookahead, dt: sim.dt },
      })
    },
  }
  ctl.add(tune, 'twiddle').name('Auto-tune (twiddle)')
  ctl.close()

  return gui
}
