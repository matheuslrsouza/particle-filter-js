// Runs the SLAM on a recorded capture and writes the map as a PGM image plus a per-turn log:
//   npx vite-node scripts/slam-offline.ts capture.bin out-prefix
import { readFileSync, writeFileSync } from 'node:fs'
import { CamsenseParser, RevolutionAssembler } from '../src/core/lidar/camsense'
import { HectorSlam, scanToPoints } from '../src/core/slam/hector'

const [file, out = 'slam'] = process.argv.slice(2)
const minRange = Number(process.env.MIN_RANGE ?? 0.12)
const bytes = new Uint8Array(readFileSync(file))
const asm = new RevolutionAssembler()
const turns = new CamsenseParser().push(bytes).map((p) => asm.add(p)).filter((r) => r !== null)
const slam = new HectorSlam()
const log: string[] = []
turns.forEach((t, i) => {
  const s = t!.samples.filter((s) => !Number.isNaN(s.distance) && s.distance >= minRange && s.distance <= 8 && s.quality >= 1)
  slam.addScan(scanToPoints(s, { mirror: process.env.MIRROR !== '0', rotation: 0, minRange: 0, maxRange: Infinity, minQuality: 0, motion: slam.motion }))
  const m = slam.motion
  log.push(`${i}\t${(i / 5).toFixed(1)}\t${slam.lastFit.toFixed(2)}\t${((m.theta * 180) / Math.PI).toFixed(1)}\t${slam.pose.x.toFixed(2)}\t${slam.pose.y.toFixed(2)}\t${((slam.pose.theta * 180) / Math.PI).toFixed(0)}\t${s.length}`)
})
writeFileSync(`${out}.tsv`, 'turn\tsec\tfit\tdtheta\tx\ty\ttheta\tpoints\n' + log.join('\n'))
const g = slam.maps[0]
const px = Buffer.alloc(g.cells * g.cells)
for (let iy = 0; iy < g.cells; iy++) for (let ix = 0; ix < g.cells; ix++) {
  const l = g.logOdds[iy * g.cells + ix]
  px[(g.cells - 1 - ix) * g.cells + iy] = l > 0.5 ? 255 : l < -0.2 ? 70 : 20
}
writeFileSync(`${out}.pgm`, Buffer.concat([Buffer.from(`P5 ${g.cells} ${g.cells} 255\n`), px]))
let walls = 0
for (const l of g.logOdds) if (l > 0.5) walls++
console.log(`walls ${walls}, turns ${turns.length}, mean fit ${(log.reduce((a, l) => a + Number(l.split('\t')[2]), 0) / log.length).toFixed(3)}`)
