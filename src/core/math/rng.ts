/** Small seedable PRNG (mulberry32) so simulations and tests are reproducible. */
export class Rng {
  private state: number
  private spare: number | null = null

  constructor(seed = Date.now()) {
    this.state = seed >>> 0
  }

  /** Uniform in [0, 1). */
  next(): number {
    let t = (this.state = (this.state + 0x6d2b79f5) >>> 0)
    t = Math.imul(t ^ (t >>> 15), t | 1)
    t ^= t + Math.imul(t ^ (t >>> 7), t | 61)
    return ((t ^ (t >>> 14)) >>> 0) / 4294967296
  }

  range(lo: number, hi: number): number {
    return lo + (hi - lo) * this.next()
  }

  int(n: number): number {
    return Math.floor(this.next() * n)
  }

  /** Normal distribution via Box-Muller. */
  gaussian(mean = 0, std = 1): number {
    if (this.spare !== null) {
      const s = this.spare
      this.spare = null
      return mean + std * s
    }
    let u = 0
    while (u === 0) u = this.next()
    const v = this.next()
    const mag = Math.sqrt(-2 * Math.log(u))
    this.spare = mag * Math.sin(2 * Math.PI * v)
    return mean + std * mag * Math.cos(2 * Math.PI * v)
  }
}
