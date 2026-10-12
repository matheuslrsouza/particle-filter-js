import type { Pose, Vec2 } from '../math/vec'

export interface Segment {
  a: Vec2
  b: Vec2
}

/** Serialized map format (meters). Each wall is [x1, y1, x2, y2]. */
export interface MapDef {
  name: string
  width: number
  height: number
  start: Pose
  walls: [number, number, number, number][]
}

export interface World {
  name: string
  width: number
  height: number
  start: Pose
  /** interior walls plus the outer border */
  walls: Segment[]
}

export function loadWorld(def: MapDef): World {
  const walls: Segment[] = def.walls.map(([x1, y1, x2, y2]) => ({ a: { x: x1, y: y1 }, b: { x: x2, y: y2 } }))
  const { width: w, height: h } = def
  walls.push(
    { a: { x: 0, y: 0 }, b: { x: w, y: 0 } },
    { a: { x: w, y: 0 }, b: { x: w, y: h } },
    { a: { x: w, y: h }, b: { x: 0, y: h } },
    { a: { x: 0, y: h }, b: { x: 0, y: 0 } },
  )
  return { name: def.name, width: w, height: h, start: { ...def.start }, walls }
}
