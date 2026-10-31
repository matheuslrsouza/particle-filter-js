/** HTTP client of scripts/robot-bridge.mjs, which drives the robot vacuum through its own firmware. */
export class RobotBridge {
  private inFlight = false
  /** when the last moving command was sent (ms, performance.now) */
  lastMoveAt = 0

  constructor(public url: string) {}

  /** Remote-control units: v ≈ 720 per m/s forward, w turn (+ counter-clockwise). Drops calls while one is pending. */
  async drive(v: number, w: number): Promise<void> {
    if (this.inFlight) return
    this.inFlight = true
    if (Math.round(v) || Math.round(w)) this.lastMoveAt = performance.now()
    try {
      await fetch(`${this.url}/drive?v=${Math.round(v)}&w=${Math.round(w)}`)
    } catch {
      /* the bridge's watchdog stops the robot when calls stop arriving */
    } finally {
      this.inFlight = false
    }
  }

  async stop(): Promise<void> {
    this.lastMoveAt = 0
    try {
      await fetch(`${this.url}/stop`)
    } catch {
      /* nothing to stop if the bridge isn't running */
    }
  }

  async status(): Promise<{ battery: number; status: number; charging: number }> {
    const r = await fetch(`${this.url}/status`)
    const j = await r.json()
    if (!j.ok) throw new Error(j.error)
    return j
  }
}
