// Writes src/maps/maze.json. Usage: node scripts/generate-maze.mjs [seed] (default 12)
// (scripts/pick-maze.ts searches for a seed that makes the exploration interesting)
import { writeFileSync } from 'node:fs'
import { generateMaze, toJson } from './maze.mjs'

const map = generateMaze({ seed: Number(process.argv[2] ?? 12) })
writeFileSync(new URL('../src/maps/maze.json', import.meta.url), toJson(map))
console.log(`maze: ${map.width}x${map.height} m, ${map.walls.length} wall segments`)
