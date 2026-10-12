import type { Simulation } from '../core/sim/simulation'

const PHASE_TEXT = {
  idle: 'Waiting for a goal',
  planning: 'Planning with A*…',
  driving: 'Driving (PID on the estimated pose)',
  arrived: 'Arrived!',
}

export class Hud {
  private status = document.getElementById('status')!
  private toast = document.getElementById('toast')!
  private canvas = document.getElementById('chart') as HTMLCanvasElement
  private ctx = this.canvas.getContext('2d')!
  private toastTimer = 0

  constructor() {
    const dpr = Math.min(window.devicePixelRatio, 2)
    this.canvas.width = 280 * dpr
    this.canvas.height = 140 * dpr
    this.ctx.scale(dpr, dpr)
  }

  message(text: string): void {
    this.toast.textContent = text
    this.toast.classList.add('show')
    clearTimeout(this.toastTimer)
    this.toastTimer = window.setTimeout(() => this.toast.classList.remove('show'), 2600)
  }

  update(sim: Simulation): void {
    const e = sim.pf.estimate
    const err = Math.hypot(e.x - sim.car.x, e.y - sim.car.y)
    this.status.textContent = `${PHASE_TEXT[sim.phase]} · localization error ${(err * 100).toFixed(0)} cm · Neff ${(e.neff * 100).toFixed(0)}%`
    this.drawChart(sim)
  }

  private drawChart(sim: Simulation): void {
    const { ctx } = this
    const w = 280
    const h = 140
    ctx.clearRect(0, 0, w, h)
    const data = sim.telemetry
    const series = [
      { label: 'CTE (m)', color: '#ff2e88', get: (d: (typeof data)[number]) => d.cte, range: 1 },
      { label: 'loc. error (m)', color: '#39ff88', get: (d: (typeof data)[number]) => d.locError, range: 1 },
    ]
    const top = 22
    const plotH = h - top - 6
    const mid = top + plotH / 2
    ctx.strokeStyle = 'rgba(140,160,190,0.25)'
    ctx.beginPath()
    ctx.moveTo(0, mid)
    ctx.lineTo(w, mid)
    ctx.stroke()
    ctx.font = '11px system-ui, sans-serif'
    series.forEach((s, k) => {
      ctx.fillStyle = s.color
      ctx.fillText(`● ${s.label} ${data.length ? s.get(data[data.length - 1]).toFixed(2) : '–'}`, 4 + k * 140, 12)
      if (data.length < 2) return
      ctx.strokeStyle = s.color
      ctx.lineWidth = 1.5
      ctx.beginPath()
      data.forEach((d, i) => {
        const x = (i / (300 - 1)) * w
        const v = Math.max(-s.range, Math.min(s.range, s.get(d)))
        const y = mid - (v / s.range) * (plotH / 2)
        i ? ctx.lineTo(x, y) : ctx.moveTo(x, y)
      })
      ctx.stroke()
    })
    ctx.fillStyle = 'rgba(139,152,169,0.8)'
    ctx.fillText('±1 m · last 30 s', w - 92, h - 8)
  }
}
