/**
 * Decoder for the 2D lidar of the Xiaomi S10+ robot vacuum (Dreame "D2M"), read from its TX line
 * at 115200 baud, 8N1, 3.3 V. The format matches the Camsense X1 lidar:
 *
 *   55 AA 03 08 | speed u16 | start angle u16 | 8 × (distance u16, quality u8) | end angle u16 | check u16
 *
 * All little-endian. speed = rpm × 64; angle = (raw − 0xA000) / 64 degrees; distance in mm, with
 * 0x8000 meaning "no echo". The last two bytes are a checksum whose algorithm we haven't identified,
 * so packets are validated by their structure (header, angles in range, a plausible angular span).
 */

export const PACKET_SIZE = 36
const SAMPLES = 8
const INVALID = 0x8000

export interface LidarSample {
  /** degrees, 0–360, in the order the head turns */
  angle: number
  /** meters; NaN when there was no echo */
  distance: number
  quality: number
}

export interface LidarPacket {
  rpm: number
  startAngle: number
  endAngle: number
  samples: LidarSample[]
}

const decodeAngle = (raw: number) => (raw - 0xa000) / 64

/** Decodes one packet at `off`, or returns null if the bytes there don't form a valid packet. */
export function decodePacket(b: Uint8Array, off = 0): LidarPacket | null {
  if (b.length - off < PACKET_SIZE) return null
  if (b[off] !== 0x55 || b[off + 1] !== 0xaa || b[off + 2] !== 0x03 || b[off + 3] !== SAMPLES) return null
  const u16 = (i: number) => b[off + i] | (b[off + i + 1] << 8)
  const startAngle = decodeAngle(u16(6))
  const endAngle = decodeAngle(u16(32))
  if (!(startAngle >= 0 && startAngle < 360 && endAngle >= 0 && endAngle < 360)) return null
  // the head turns ~6.5° per packet; the span wraps at 360°
  const span = (endAngle - startAngle + 360) % 360
  if (span > 30) return null
  const samples: LidarSample[] = []
  for (let k = 0; k < SAMPLES; k++) {
    const d = u16(8 + 3 * k)
    samples.push({
      angle: (startAngle + (span * k) / (SAMPLES - 1)) % 360,
      distance: d >= INVALID ? NaN : d / 1000,
      quality: b[off + 10 + 3 * k],
    })
  }
  return { rpm: u16(4) / 64, startAngle, endAngle, samples }
}

/** Streaming decoder: feed it chunks as they arrive from the serial port, in any size. */
export class CamsenseParser {
  packets = 0
  /** bytes thrown away while looking for a packet start (noise, a partial packet on connect) */
  skipped = 0
  private buf = new Uint8Array(0)

  push(chunk: Uint8Array): LidarPacket[] {
    const b = new Uint8Array(this.buf.length + chunk.length)
    b.set(this.buf)
    b.set(chunk, this.buf.length)
    const out: LidarPacket[] = []
    let i = 0
    while (b.length - i >= PACKET_SIZE) {
      const p = decodePacket(b, i)
      if (p) {
        out.push(p)
        this.packets++
        i += PACKET_SIZE
      } else {
        i++
        this.skipped++
      }
    }
    this.buf = b.slice(i)
    return out
  }
}

export interface Revolution {
  rpm: number
  samples: LidarSample[]
}

/**
 * Groups packets into full turns of the head: a turn ends when the angle wraps past 0°. The turn in
 * progress when the stream starts is dropped, since its beginning was never seen.
 */
export class RevolutionAssembler {
  private current: LidarSample[] = []
  private lastAngle = -1
  private rpm = 0
  private started = false

  add(p: LidarPacket): Revolution | null {
    let done: Revolution | null = null
    if (this.lastAngle >= 0 && p.startAngle < this.lastAngle - 180) {
      if (this.started) done = { rpm: this.rpm, samples: this.current }
      this.started = true
      this.current = []
    }
    for (const s of p.samples) this.current.push(s)
    this.lastAngle = p.startAngle
    this.rpm = p.rpm
    return done
  }
}
