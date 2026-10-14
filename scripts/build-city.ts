/**
 * Converts the raw Overpass data (scripts/fetch-osm.ts) into src/maps/city-mountain-view.json:
 * local metric coordinates (x east, y south, origin at the area's north-west corner), roads
 * with lane counts per direction and speed limits, buildings with heights, parks.
 * Usage: npx vite-node scripts/build-city.ts
 * Data © OpenStreetMap contributors, ODbL.
 */
import { readFileSync, writeFileSync } from 'node:fs'
import type { CityDef, CityRoad } from '../src/core/city/cityDef'
import { BBOX } from './cityArea'

interface OsmNode { type: 'node'; id: number; lat: number; lon: number }
interface OsmWay { type: 'way'; id: number; nodes: number[]; tags?: Record<string, string> }

const raw = JSON.parse(readFileSync(new URL('./cache/mountain-view.osm.json', import.meta.url), 'utf8'))
const nodes = new Map<number, OsmNode>()
const ways: OsmWay[] = []
for (const e of raw.elements) {
  if (e.type === 'node') nodes.set(e.id, e)
  else if (e.type === 'way') ways.push(e)
}

// equirectangular projection around the area center, shifted so the area starts at (0, 0)
const MARGIN = 60
const lat0 = (BBOX.south + BBOX.north) / 2
const lon0 = (BBOX.west + BBOX.east) / 2
const kx = Math.cos((lat0 * Math.PI) / 180) * 111_320
const ky = 110_574
const project = (lat: number, lon: number): [number, number] => [(lon - lon0) * kx, -(lat - lat0) * ky]
const [minX, minY] = project(BBOX.north, BBOX.west)
const [maxX, maxY] = project(BBOX.south, BBOX.east)
const width = maxX - minX + 2 * MARGIN
const height = maxY - minY + 2 * MARGIN
const local = (n: OsmNode): [number, number] => {
  const [x, y] = project(n.lat, n.lon)
  return [round(x - minX + MARGIN), round(y - minY + MARGIN)]
}
const round = (v: number) => Math.round(v * 100) / 100
const inside = ([x, y]: [number, number]) => x >= 0 && y >= 0 && x <= width && y <= height

const mph = (s: string | undefined): number | null => {
  const m = s?.match(/^(\d+)\s*mph$/)
  return m ? Number(m[1]) * 0.44704 : null
}
const DEFAULT_SPEED: Record<string, number> = { primary: 35, secondary: 35, tertiary: 30, residential: 25, unclassified: 25, living_street: 15 }
const DEFAULT_ONEWAY_LANES: Record<string, number> = { primary: 2, secondary: 2 }

function lanesOf(t: Record<string, string>, oneway: boolean): { forward: number; backward: number; centerTurn: boolean } {
  const total = Number(t.lanes) || 0
  if (oneway) return { forward: total || DEFAULT_ONEWAY_LANES[t.highway.replace('_link', '')] || 1, backward: 0, centerTurn: false }
  const f = Number(t['lanes:forward']) || 0
  const b = Number(t['lanes:backward']) || 0
  if (f && b) return { forward: f, backward: b, centerTurn: total > f + b }
  if (!total) return { forward: 1, backward: 1, centerTurn: false }
  // an odd count on a two-way US street is usually a center turn lane
  const side = Math.max(1, Math.floor(total / 2))
  return { forward: side, backward: side, centerTurn: total % 2 === 1 && total > 2 }
}

const roads: CityRoad[] = []
for (const w of ways) {
  const t = w.tags
  if (!t?.highway) continue
  const oneway = t.oneway === 'yes' || t.oneway === '-1' || t.highway.endsWith('_link') && t.oneway !== 'no'
  const ids = t.oneway === '-1' ? [...w.nodes].reverse() : w.nodes
  const lanes = lanesOf(t, oneway)
  const speed = mph(t.maxspeed) ?? DEFAULT_SPEED[t.highway.replace('_link', '')] * 0.44704
  // keep the runs of the way that are inside the area
  let run: { pts: [number, number][]; nodes: number[] } = { pts: [], nodes: [] }
  const flush = () => {
    if (run.pts.length >= 2) {
      roads.push({
        id: w.id,
        name: t.name ?? '',
        highway: t.highway,
        pts: run.pts,
        nodes: run.nodes,
        lanesForward: lanes.forward,
        lanesBackward: lanes.backward,
        centerTurnLane: lanes.centerTurn,
        speed: round(speed),
      })
    }
    run = { pts: [], nodes: [] }
  }
  for (const id of ids) {
    const n = nodes.get(id)
    if (!n) continue
    const p = local(n)
    if (inside(p)) {
      run.pts.push(p)
      run.nodes.push(id)
    } else flush()
  }
  flush()
}

const polygonArea = (pts: [number, number][]) => {
  let a = 0
  for (let i = 0; i < pts.length; i++) {
    const [x1, y1] = pts[i]
    const [x2, y2] = pts[(i + 1) % pts.length]
    a += x1 * y2 - x2 * y1
  }
  return Math.abs(a) / 2
}

/** deterministic pseudo-random in [0, 1) from an id */
const hash01 = (id: number) => {
  let h = id ^ 0x9e3779b9
  h = Math.imul(h ^ (h >>> 16), 0x85ebca6b)
  h = Math.imul(h ^ (h >>> 13), 0xc2b2ae35)
  return ((h ^ (h >>> 16)) >>> 0) / 4294967296
}

const buildings: CityDef['buildings'] = []
const parks: CityDef['parks'] = []
const parking: CityDef['parking'] = []
for (const w of ways) {
  const t = w.tags
  if (!t || w.nodes.length < 4) continue
  const pts = w.nodes.slice(0, -1).map((id) => nodes.get(id)).filter((n): n is OsmNode => !!n).map(local)
  if (pts.length < 3 || !pts.every(inside)) continue
  if (t.amenity === 'parking') {
    // surface lots only (garages are buildings)
    if (!/underground|multi-storey|rooftop/.test(t.parking ?? '') && !t.building) parking.push({ id: w.id, pts })
  } else if (t.building) {
    const levels = Number(t['building:levels'])
    const area = polygonArea(pts)
    // without data, estimate from the footprint: small houses get one floor, larger blocks more
    const guess = area < 120 ? 1 : area < 400 ? 1 + Math.round(hash01(w.id)) : area < 1500 ? 2 + Math.round(hash01(w.id)) : 3 + Math.round(hash01(w.id) * 2)
    const h = Number(t.height) || (levels || guess) * 3.4 + 0.6
    buildings.push({ id: w.id, pts, h: round(h) })
  } else if (t.leisure === 'park' || t.landuse) {
    parks.push({ pts })
  }
}

const city: CityDef = {
  name: 'Mountain View, CA (downtown)',
  attribution: '© OpenStreetMap contributors',
  width: round(width),
  height: round(height),
  roads,
  buildings,
  parks,
  parking,
}
writeFileSync(new URL('../src/maps/city-mountain-view.json', import.meta.url), JSON.stringify(city))
console.log(`city ${city.width} x ${city.height} m: ${roads.length} roads, ${buildings.length} buildings, ${parks.length} parks, ${parking.length} parking lots`)
