// Tracking metrics over several seeds and localization setups: npx vite-node scripts/bench-tracking.ts
import cityJson from '../src/maps/city-mountain-view.json'
import { CitySim } from '../src/core/city/citySim'

const setups = (process.env.SETUPS ?? 'truth,ekf-rtk,ekf-dual,ekf-consumer').split(',')
const seeds = (process.env.SEEDS ?? '1,2,3,4,5,6').split(',').map(Number)
for (const setup of setups) {
  const rows: string[] = []
  const acc = { mota: 0, recall: 0, posErr: 0, velErr: 0 }
  for (const seed of seeds) {
    const sim = new CitySim(cityJson as never, seed)
    if (setup === 'truth') sim.localization = 'truth'
    else sim.gpsGrade = setup.split('-')[1] as never
    sim.enableTraffic(40, 30)
    sim.perception = 'tracks'
    sim.randomDestination()
    for (let i = 0; i < 50 * 300 && sim.phase === 'driving'; i++) sim.tick()
    const m = sim.metrics.summary
    for (const k of Object.keys(acc) as (keyof typeof acc)[]) acc[k] += m[k] / seeds.length
    rows.push(`${seed}:${m.mota.toFixed(2)}/${m.posErr.toFixed(2)}/${m.velErr.toFixed(2)}`)
  }
  console.log(setup.padEnd(13), `MOTA ${acc.mota.toFixed(2)} recall ${acc.recall.toFixed(2)} pos ${acc.posErr.toFixed(2)} m vel ${acc.velErr.toFixed(2)} m/s`, '|', rows.join(' '))
}
