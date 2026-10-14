/**
 * Downloads the OpenStreetMap data of the city area through the Overpass API.
 * Usage: npx vite-node scripts/fetch-osm.ts
 * Output: scripts/cache/mountain-view.osm.json (raw, not committed). Then run build-city.ts.
 * Data © OpenStreetMap contributors, ODbL.
 */
import { mkdirSync, writeFileSync } from 'node:fs'
import { BBOX } from './cityArea'

const bbox = `${BBOX.south},${BBOX.west},${BBOX.north},${BBOX.east}`
const query = `
[out:json][timeout:60];
(
  way["highway"~"^(primary|secondary|tertiary|residential|unclassified|living_street|primary_link|secondary_link|tertiary_link)$"](${bbox});
  way["building"](${bbox});
  way["leisure"="park"](${bbox});
  way["landuse"~"^(grass|recreation_ground)$"](${bbox});
);
out body;
>;
out skel qt;
`

const res = await fetch('https://overpass-api.de/api/interpreter', {
  method: 'POST',
  body: new URLSearchParams({ data: query }),
  headers: { 'User-Agent': 'robot-js-3d (educational simulator)' },
})
if (!res.ok) throw new Error(`Overpass error ${res.status}: ${await res.text()}`)
const json = await res.json()
mkdirSync(new URL('./cache/', import.meta.url), { recursive: true })
writeFileSync(new URL('./cache/mountain-view.osm.json', import.meta.url), JSON.stringify(json))
const ways = json.elements.filter((e: { type: string }) => e.type === 'way').length
const nodes = json.elements.filter((e: { type: string }) => e.type === 'node').length
console.log(`saved ${ways} ways, ${nodes} nodes`)
