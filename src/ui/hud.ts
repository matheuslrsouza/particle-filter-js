export interface ChartSeries {
  label: string
  color: string
  values: number[]
  /** values are clipped to ±range */
  range: number
}

const CHART_POINTS = 300

export class Hud {
  private status = document.getElementById('status')!
  private hint = document.querySelector('header .hint')!
  private toast = document.getElementById('toast')!
  private canvas = document.getElementById('chart') as HTMLCanvasElement
  private ctx = this.canvas.getContext('2d')!
  private toastTimer = 0

  constructor(hintHtml?: string) {
    const dpr = Math.min(window.devicePixelRatio, 2)
    this.canvas.width = 280 * dpr
    this.canvas.height = 140 * dpr
    this.ctx.scale(dpr, dpr)
    if (hintHtml) this.hint.innerHTML = hintHtml
  }

  message(text: string): void {
    this.toast.textContent = text
    this.toast.classList.add('show')
    clearTimeout(this.toastTimer)
    this.toastTimer = window.setTimeout(() => this.toast.classList.remove('show'), 2600)
  }

  setStatus(text: string): void {
    if (this.status.textContent !== text) this.status.textContent = text
  }

  chart(series: ChartSeries[], footer: string): void {
    const { ctx } = this
    const w = 280
    const h = 140
    ctx.clearRect(0, 0, w, h)
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
      const last = s.values[s.values.length - 1]
      ctx.fillStyle = s.color
      ctx.fillText(`● ${s.label} ${last !== undefined ? last.toFixed(2) : '–'}`, 4 + k * 140, 12)
      if (s.values.length < 2) return
      ctx.strokeStyle = s.color
      ctx.lineWidth = 1.5
      ctx.beginPath()
      s.values.forEach((value, i) => {
        const x = (i / (CHART_POINTS - 1)) * w
        const v = Math.max(-s.range, Math.min(s.range, value))
        const y = mid - (v / s.range) * (plotH / 2)
        i ? ctx.lineTo(x, y) : ctx.moveTo(x, y)
      })
      ctx.stroke()
    })
    ctx.fillStyle = 'rgba(139,152,169,0.8)'
    ctx.textAlign = 'right'
    ctx.fillText(footer, w - 4, h - 8)
    ctx.textAlign = 'left'
  }
}
