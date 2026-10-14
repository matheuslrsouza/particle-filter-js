/** City data generated from OpenStreetMap by scripts/build-city.ts (meters, x east, y south). */
export interface CityRoad {
  /** OSM way id */
  id: number
  name: string
  highway: string
  pts: [number, number][]
  /** OSM node id of every point: shared ids are junctions */
  nodes: number[]
  lanesForward: number
  lanesBackward: number
  /** a two-way left-turn lane in the middle (common on US streets) */
  centerTurnLane: boolean
  /** speed limit (m/s) */
  speed: number
}

export interface CityDef {
  name: string
  attribution: string
  width: number
  height: number
  roads: CityRoad[]
  buildings: { id: number; pts: [number, number][]; h: number }[]
  parks: { pts: [number, number][] }[]
}
