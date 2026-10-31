import { readFileSync } from 'node:fs'
import { describe, expect, it } from 'vitest'
import { CamsenseParser, decodePacket, PACKET_SIZE, RevolutionAssembler } from './lidar/camsense'

/** Builds a packet like the lidar sends. */
function packet(rpm: number, start: number, end: number, mm: number[]): Uint8Array {
  const b = new Uint8Array(PACKET_SIZE)
  const put = (i: number, v: number) => {
    b[i] = v & 0xff
    b[i + 1] = (v >> 8) & 0xff
  }
  b.set([0x55, 0xaa, 0x03, 0x08])
  put(4, Math.round(rpm * 64))
  put(6, Math.round(start * 64) + 0xa000)
  mm.forEach((d, k) => {
    put(8 + 3 * k, d)
    b[10 + 3 * k] = 50
  })
  put(32, Math.round(end * 64) + 0xa000)
  return b
}

describe('lidar decoder', () => {
  it('decodes angles, distances, speed and missing echoes', () => {
    const p = decodePacket(packet(320, 104.25, 110.75, [219, 217, 0x8000, 212, 210, 208, 206, 204]))!
    expect(p.rpm).toBeCloseTo(320, 1)
    expect(p.startAngle).toBeCloseTo(104.25, 2)
    expect(p.samples[0]).toEqual({ angle: 104.25, distance: 0.219, quality: 50 })
    expect(p.samples[7].angle).toBeCloseTo(110.75, 2)
    expect(p.samples[2].distance).toBeNaN()
  })

  it('handles packets crossing 0° and rejects garbage', () => {
    const p = decodePacket(packet(320, 357, 3.5, new Array(8).fill(500)))!
    expect(p.samples[0].angle).toBeCloseTo(357, 2)
    expect(p.samples[7].angle).toBeCloseTo(3.5, 2)
    expect(decodePacket(new Uint8Array(PACKET_SIZE).fill(0x55))).toBeNull()
  })

  it('reassembles packets split across serial chunks, skipping noise', () => {
    const parser = new CamsenseParser()
    const stream = new Uint8Array([1, 2, 3, ...packet(300, 10, 16.5, new Array(8).fill(1000)), ...packet(300, 17, 23.5, new Array(8).fill(1000))])
    const got = [stream.slice(0, 20), stream.slice(20, 41), stream.slice(41)].flatMap((c) => parser.push(c))
    expect(got.map((p) => Math.round(p.startAngle))).toEqual([10, 17])
    expect(parser.skipped).toBe(3)
  })

  it('decodes the capture from the real robot: 380–420 points per turn at ~5 Hz', () => {
    const bytes = new Uint8Array(readFileSync(new URL('../real/sample-capture.bin', import.meta.url)))
    const parser = new CamsenseParser()
    const asm = new RevolutionAssembler()
    const revs = parser.push(bytes).map((p) => asm.add(p)).filter((r) => r !== null)
    expect(parser.packets).toBeGreaterThan(7800)
    expect(parser.skipped).toBeLessThan(100)
    // the capture started mid-turn: that partial turn is not emitted
    const full = revs
    expect(full.length).toBeGreaterThan(150)
    for (const r of full) {
      expect(r.samples.length).toBeGreaterThan(370)
      expect(r.samples.length).toBeLessThan(430)
      expect(r.rpm).toBeGreaterThan(290)
      expect(r.rpm).toBeLessThan(350)
    }
    // the robot was in a box of about half a meter: most echoes are close
    const d = full.flatMap((r) => r.samples.map((s) => s.distance)).filter((x) => !Number.isNaN(x))
    const median = d.sort((a, b) => a - b)[Math.floor(d.length / 2)]
    expect(median).toBeGreaterThan(0.15)
    expect(median).toBeLessThan(0.8)
  })
})
