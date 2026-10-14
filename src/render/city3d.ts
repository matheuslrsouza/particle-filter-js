import * as THREE from 'three'
import { mergeGeometries } from 'three/examples/jsm/utils/BufferGeometryUtils.js'
import type { CityDef } from '../core/city/cityDef'
import { offsetPolyline, polylineLength, type LaneGraph, type Segment } from '../core/city/roadNetwork'
import type { Vec2 } from '../core/math/vec'

export const CITY_COLORS = {
  ground: 0x9fb48a,
  park: 0x7fa86a,
  asphalt: 0x3d424b,
  sidewalk: 0xc9c3b6,
  yellow: 0xf2c94c,
  white: 0xf2f2f2,
  trunk: 0x7a5a3a,
  crowns: [0x5f9e4f, 0x6fae58, 0x4f8c45, 0x7cb35f],
}

const Y = { park: 0.02, road: 0.05, junction: 0.06, marking: 0.09, sidewalk: 0.15 }

/** Collects flat triangles at a given height, to merge many quads in a single geometry. */
class FlatBuilder {
  readonly pos: number[] = []

  quad(a: Vec2, b: Vec2, c: Vec2, d: Vec2, y: number) {
    // two triangles, counter-clockwise when seen from above (+y up, z = sim y)
    this.pos.push(a.x, y, a.y, c.x, y, c.y, b.x, y, b.y, a.x, y, a.y, d.x, y, d.y, c.x, y, c.y)
  }

  /** A ribbon between two offsets (to the right of the direction) of a polyline. */
  ribbon(pts: Vec2[], left: number, right: number, y: number) {
    if (pts.length < 2) return
    const l = offsetPolyline(pts, left)
    const r = offsetPolyline(pts, right)
    for (let i = 1; i < pts.length; i++) this.quad(l[i - 1], l[i], r[i], r[i - 1], y)
  }

  /** Same as ribbon, keeping only the stretches of a dash/gap pattern. */
  dashed(pts: Vec2[], center: number, width: number, dash: number, gap: number, y: number) {
    for (const piece of dashes(pts, dash, gap)) this.ribbon(piece, center - width / 2, center + width / 2, y)
  }

  fan(center: Vec2, ring: Vec2[], y: number) {
    for (let i = 0; i < ring.length; i++) {
      const a = ring[i]
      const b = ring[(i + 1) % ring.length]
      this.pos.push(center.x, y, center.y, b.x, y, b.y, a.x, y, a.y)
    }
  }

  /** `layer` orders coplanar ground layers (ground < parks < roads < markings) without z-fighting. */
  mesh(color: number, layer: number): THREE.Mesh {
    const geo = new THREE.BufferGeometry()
    geo.setAttribute('position', new THREE.Float32BufferAttribute(this.pos, 3))
    geo.computeVertexNormals()
    const material = new THREE.MeshStandardMaterial({
      color,
      roughness: 0.95,
      polygonOffset: true,
      polygonOffsetFactor: -layer,
      polygonOffsetUnits: -layer * 4,
    })
    const m = new THREE.Mesh(geo, material)
    m.receiveShadow = true
    return m
  }
}

function dashes(pts: Vec2[], dash: number, gap: number): Vec2[][] {
  const out: Vec2[][] = []
  const total = polylineLength(pts)
  for (let s = gap / 2; s < total; s += dash + gap) out.push(slice(pts, s, Math.min(total, s + dash)))
  return out
}

function slice(pts: Vec2[], from: number, to: number): Vec2[] {
  const out: Vec2[] = []
  let s = 0
  for (let i = 1; i < pts.length; i++) {
    const a = pts[i - 1]
    const b = pts[i]
    const len = Math.hypot(b.x - a.x, b.y - a.y)
    const at = (d: number) => ({ x: a.x + ((b.x - a.x) * (d - s)) / len, y: a.y + ((b.y - a.y) * (d - s)) / len })
    if (len > 0 && s + len >= from && s <= to) {
      if (!out.length) out.push(at(Math.max(from, s)))
      out.push(s + len <= to ? { ...b } : at(to))
    }
    s += len
    if (s > to) break
  }
  return out.length >= 2 ? out : [pts[0], pts[0]]
}

/** deterministic pseudo-random in [0, 1) */
const hash01 = (n: number) => {
  let h = n ^ 0x9e3779b9
  h = Math.imul(h ^ (h >>> 16), 0x85ebca6b)
  h = Math.imul(h ^ (h >>> 13), 0xc2b2ae35)
  return ((h ^ (h >>> 16)) >>> 0) / 4294967296
}

function roads(g: LaneGraph): THREE.Group {
  const W = g.params.laneWidth
  const asphalt = new FlatBuilder()
  const yellow = new FlatBuilder()
  const white = new FlatBuilder()
  const sidewalk = new FlatBuilder()

  for (const s of g.segments) {
    const c = s.center
    if (polylineLength(c) < 0.3) continue
    asphalt.ribbon(c, -s.leftWidth, s.rightWidth, Y.road)
    sidewalk.ribbon(c, s.rightWidth, s.rightWidth + 2, Y.sidewalk)
    sidewalk.ribbon(c, -s.leftWidth - 2, -s.leftWidth, Y.sidewalk)
    // edge lines
    white.ribbon(c, s.rightWidth - 0.35, s.rightWidth - 0.2, Y.marking)
    white.ribbon(c, -s.leftWidth + 0.2, -s.leftWidth + 0.35, Y.marking)
    const road = s.road
    const oneway = road.lanesBackward === 0
    if (!oneway) {
      // double yellow center line (on the edges of the turn lane when there is one)
      const half = road.centerTurnLane ? W / 2 : 0.18
      yellow.ribbon(c, -half - 0.06, -half + 0.06, Y.marking)
      yellow.ribbon(c, half - 0.06, half + 0.06, Y.marking)
    }
    // dashed white between lanes of the same direction
    const dividers = (count: number, leftEdge: number, sign: number) => {
      for (let k = 1; k < count; k++) white.dashed(c, sign * (leftEdge + k * W), 0.12, 3, 6, Y.marking)
    }
    const leftEdge = oneway ? (-road.lanesForward * W) / 2 : road.centerTurnLane ? W / 2 : 0
    dividers(road.lanesForward, leftEdge, 1)
    if (!oneway) dividers(road.lanesBackward, leftEdge, -1)
  }

  // junction surfaces: fan over the corners of the incoming roads
  for (const j of g.junctions) {
    if (!j.segments.length) continue
    const corners: Vec2[] = []
    for (const id of j.segments) {
      const s: Segment = g.segments[id]
      const atStart = s.from === j.id
      const r = offsetPolyline(s.center, s.rightWidth)
      const l = offsetPolyline(s.center, -s.leftWidth)
      corners.push(atStart ? r[0] : r[r.length - 1], atStart ? l[0] : l[l.length - 1])
    }
    const cx = corners.reduce((a, p) => a + p.x, 0) / corners.length
    const cy = corners.reduce((a, p) => a + p.y, 0) / corners.length
    corners.sort((a, b) => Math.atan2(a.y - cy, a.x - cx) - Math.atan2(b.y - cy, b.x - cx))
    asphalt.fan({ x: cx, y: cy }, corners, Y.junction)
  }

  const group = new THREE.Group()
  group.add(asphalt.mesh(CITY_COLORS.asphalt, 2), sidewalk.mesh(CITY_COLORS.sidewalk, 2), yellow.mesh(CITY_COLORS.yellow, 4), white.mesh(CITY_COLORS.white, 4))
  return group
}

/** Window pattern, multiplied by the building color; the corner texel is plain (used by roofs). */
function windowTexture(): THREE.CanvasTexture {
  const c = document.createElement('canvas')
  c.width = c.height = 64
  const ctx = c.getContext('2d')!
  ctx.fillStyle = '#ffffff'
  ctx.fillRect(0, 0, 64, 64)
  ctx.fillStyle = '#5d6f86'
  ctx.fillRect(14, 18, 36, 26)
  ctx.fillStyle = '#8fa3bd'
  ctx.fillRect(14, 18, 36, 6)
  const t = new THREE.CanvasTexture(c)
  t.wrapS = t.wrapT = THREE.RepeatWrapping
  t.colorSpace = THREE.SRGBColorSpace
  // one window per 3.4 m (a floor) along the walls; the ExtrudeGeometry UVs are in meters
  t.repeat.set(1 / 3.4, 1 / 3.4)
  return t
}

const PALETTE = [0xe8d5b7, 0xf2e2c9, 0xd9c3a5, 0xc9d6df, 0xe6c7b1, 0xd8dcc6, 0xf0d9da, 0xcfc4b8, 0xe3e0d4, 0xbfcfd8]

function buildings(city: CityDef): THREE.Mesh {
  const geos: THREE.BufferGeometry[] = []
  const color = new THREE.Color()
  const roof = new THREE.Color()
  for (const b of city.buildings) {
    // shape in (x, -y) so that, after rotating to stand up, the footprint lands on (x, z = y)
    const shape = new THREE.Shape(b.pts.map(([x, y]) => new THREE.Vector2(x, -y)))
    const geo = new THREE.ExtrudeGeometry(shape, { depth: b.h, bevelEnabled: false })
    geo.rotateX(-Math.PI / 2)
    color.setHex(PALETTE[Math.floor(hash01(b.id) * PALETTE.length)])
    roof.copy(color).multiplyScalar(0.72)
    const n = geo.attributes.position.count
    const colors = new Float32Array(n * 3)
    const normal = geo.attributes.normal
    const uv = geo.attributes.uv
    for (let i = 0; i < n; i++) {
      const isRoof = Math.abs(normal.getY(i)) > 0.9
      const c = isRoof ? roof : color
      colors.set([c.r, c.g, c.b], i * 3)
      // roofs read the plain corner of the window texture
      if (isRoof) uv.setXY(i, 0.1, 0.1)
    }
    geo.setAttribute('color', new THREE.BufferAttribute(colors, 3))
    geos.push(geo)
  }
  const merged = mergeGeometries(geos)!
  geos.forEach((g) => g.dispose())
  const mesh = new THREE.Mesh(merged, new THREE.MeshStandardMaterial({ vertexColors: true, map: windowTexture(), roughness: 0.85 }))
  mesh.castShadow = true
  mesh.receiveShadow = true
  return mesh
}

function trees(city: CityDef, g: LaneGraph): THREE.Group {
  // coarse grid of building boxes, to keep trees out of buildings
  const cell = 20
  const grid = new Map<string, { minX: number; minY: number; maxX: number; maxY: number }[]>()
  for (const b of city.buildings) {
    const xs = b.pts.map((p) => p[0])
    const ys = b.pts.map((p) => p[1])
    const box = { minX: Math.min(...xs) - 1, minY: Math.min(...ys) - 1, maxX: Math.max(...xs) + 1, maxY: Math.max(...ys) + 1 }
    for (let cx = Math.floor(box.minX / cell); cx <= Math.floor(box.maxX / cell); cx++) {
      for (let cy = Math.floor(box.minY / cell); cy <= Math.floor(box.maxY / cell); cy++) {
        const k = `${cx},${cy}`
        if (!grid.has(k)) grid.set(k, [])
        grid.get(k)!.push(box)
      }
    }
  }
  const blocked = (p: Vec2) =>
    (grid.get(`${Math.floor(p.x / cell)},${Math.floor(p.y / cell)}`) ?? []).some((b) => p.x > b.minX && p.x < b.maxX && p.y > b.minY && p.y < b.maxY)

  const spots: Vec2[] = []
  for (const s of g.segments) {
    if (!['residential', 'tertiary', 'secondary'].includes(s.road.highway)) continue
    const len = polylineLength(s.center)
    for (const [side, offset] of [[1, s.rightWidth + 3], [-1, -s.leftWidth - 3]] as const) {
      const line = offsetPolyline(s.center, offset)
      const start = 4 + hash01(s.id * 7 + side) * 8
      for (let d = start; d < len - 4; d += 13 + hash01(s.id + d) * 6) {
        const p = slice(line, d, d + 0.01)[0]
        if (!blocked(p)) spots.push(p)
      }
    }
  }

  const trunk = new THREE.InstancedMesh(
    new THREE.CylinderGeometry(0.18, 0.25, 2.4, 6).translate(0, 1.2, 0),
    new THREE.MeshStandardMaterial({ color: CITY_COLORS.trunk, roughness: 1 }),
    spots.length,
  )
  const crown = new THREE.InstancedMesh(
    new THREE.IcosahedronGeometry(1.9, 0).translate(0, 3.6, 0),
    new THREE.MeshStandardMaterial({ color: 0xffffff, roughness: 0.9, flatShading: true }),
    spots.length,
  )
  const m = new THREE.Matrix4()
  const c = new THREE.Color()
  spots.forEach((p, i) => {
    const k = 0.8 + hash01(i * 13) * 0.6
    m.makeScale(k, k, k).setPosition(p.x, 0, p.y)
    trunk.setMatrixAt(i, m)
    crown.setMatrixAt(i, m)
    crown.setColorAt(i, c.setHex(CITY_COLORS.crowns[i % CITY_COLORS.crowns.length]))
  })
  trunk.castShadow = crown.castShadow = true
  const group = new THREE.Group()
  group.add(trunk, crown)
  return group
}

/** Debug view of the lane graph: lane centerlines (cyan) and junction connectors (by turn). */
export function laneGraphView(g: LaneGraph): THREE.Group {
  const group = new THREE.Group()
  const lines = (pts: Vec2[][], color: number) => {
    const pos: number[] = []
    for (const p of pts) for (let i = 1; i < p.length; i++) pos.push(p[i - 1].x, 0.3, p[i - 1].y, p[i].x, 0.3, p[i].y)
    const geo = new THREE.BufferGeometry()
    geo.setAttribute('position', new THREE.Float32BufferAttribute(pos, 3))
    const l = new THREE.LineSegments(geo, new THREE.LineBasicMaterial({ color, depthTest: false }))
    l.renderOrder = 10
    return l
  }
  group.add(lines(g.lanes.map((l) => l.points), 0x00e5ff))
  group.add(lines(g.connectors.filter((c) => c.turn === 'straight').map((c) => c.points), 0xffffff))
  group.add(lines(g.connectors.filter((c) => c.turn === 'left').map((c) => c.points), 0xff9f1c))
  group.add(lines(g.connectors.filter((c) => c.turn === 'right').map((c) => c.points), 0x7cfc00))
  return group
}

export interface City3d {
  group: THREE.Group
  /** the ground plane, for picking destinations */
  floor: THREE.Mesh
  buildings: THREE.Mesh
  trees: THREE.Group
  roads: THREE.Group
}

export function createCity3d(city: CityDef, g: LaneGraph): City3d {
  const group = new THREE.Group()
  const groundGroup = new THREE.Group()
  const plane = new THREE.Mesh(
    new THREE.PlaneGeometry(city.width * 3, city.height * 3).rotateX(-Math.PI / 2),
    new THREE.MeshStandardMaterial({ color: CITY_COLORS.ground, roughness: 1 }),
  )
  plane.position.set(city.width / 2, 0, city.height / 2)
  plane.receiveShadow = true
  const parks = new FlatBuilder()
  for (const p of city.parks) {
    const ring = p.pts.map(([x, y]) => ({ x, y }))
    const cx = ring.reduce((a, q) => a + q.x, 0) / ring.length
    const cy = ring.reduce((a, q) => a + q.y, 0) / ring.length
    parks.fan({ x: cx, y: cy }, ring, Y.park)
  }
  groundGroup.add(plane, parks.mesh(CITY_COLORS.park, 1))
  const roadGroup = roads(g)
  const buildingMesh = buildings(city)
  const treeGroup = trees(city, g)
  group.add(groundGroup, roadGroup, buildingMesh, treeGroup)
  return { group, floor: plane, buildings: buildingMesh, trees: treeGroup, roads: roadGroup }
}

/** The route painted on the road. */
export function routeRibbon(points: Vec2[]): THREE.Mesh {
  const b = new FlatBuilder()
  b.ribbon(points, -0.45, 0.45, Y.marking + 0.02)
  const m = b.mesh(0x00c8ff, 5)
  const mat = m.material as THREE.MeshStandardMaterial
  mat.transparent = true
  mat.opacity = 0.55
  mat.emissive = new THREE.Color(0x0088aa)
  m.receiveShadow = false
  return m
}

/** Destination pin. */
export function destinationPin(): THREE.Group {
  const g = new THREE.Group()
  const mat = new THREE.MeshStandardMaterial({ color: 0xff3b6b, emissive: 0x80102a, roughness: 0.4 })
  const pole = new THREE.Mesh(new THREE.CylinderGeometry(0.12, 0.12, 6, 8).translate(0, 3, 0), mat)
  const head = new THREE.Mesh(new THREE.SphereGeometry(0.9, 16, 12).translate(0, 6.6, 0), mat)
  pole.castShadow = head.castShadow = true
  g.add(pole, head)
  g.visible = false
  return g
}
