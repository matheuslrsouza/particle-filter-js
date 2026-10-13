// Generates src/maps/maze.json: a recursive-backtracker maze with a few extra openings (loops).
// Usage: node scripts/generate-maze.mjs [seed]
import { writeFileSync } from 'node:fs'

const COLS = 12
const ROWS = 8
const CELL = 2.5 // meters
const EXTRA_OPENINGS = 0.1
let seed = Number(process.argv[2] ?? 7)
const rand = () => ((seed = (seed * 1664525 + 1013904223) >>> 0) / 4294967296)

// walls between cells: h[r][c] is the wall below cell (c, r); v[r][c] the wall right of it
const h = Array.from({ length: ROWS - 1 }, () => Array(COLS).fill(true))
const v = Array.from({ length: ROWS }, () => Array(COLS - 1).fill(true))
const seen = Array.from({ length: ROWS }, () => Array(COLS).fill(false))
const stack = [[0, 0]]
seen[0][0] = true
while (stack.length) {
  const [c, r] = stack[stack.length - 1]
  const options = [[c + 1, r], [c - 1, r], [c, r + 1], [c, r - 1]].filter(
    ([nc, nr]) => nc >= 0 && nr >= 0 && nc < COLS && nr < ROWS && !seen[nr][nc],
  )
  if (!options.length) {
    stack.pop()
    continue
  }
  const [nc, nr] = options[Math.floor(rand() * options.length)]
  if (nc !== c) v[r][Math.min(c, nc)] = false
  else h[Math.min(r, nr)][c] = false
  seen[nr][nc] = true
  stack.push([nc, nr])
}
for (const row of [...h, ...v]) for (let i = 0; i < row.length; i++) if (row[i] && rand() < EXTRA_OPENINGS) row[i] = false

// merge consecutive wall pieces into long segments
const walls = []
for (let r = 0; r < ROWS - 1; r++) {
  for (let c = 0; c < COLS; ) {
    if (!h[r][c]) { c++; continue }
    const start = c
    while (c < COLS && h[r][c]) c++
    walls.push([start * CELL, (r + 1) * CELL, c * CELL, (r + 1) * CELL])
  }
}
for (let c = 0; c < COLS - 1; c++) {
  for (let r = 0; r < ROWS; ) {
    if (!v[r][c]) { r++; continue }
    const start = r
    while (r < ROWS && v[r][c]) r++
    walls.push([(c + 1) * CELL, start * CELL, (c + 1) * CELL, r * CELL])
  }
}

const map = {
  name: 'Maze',
  width: COLS * CELL,
  height: ROWS * CELL,
  start: { x: CELL / 2 - 0.25, y: CELL / 2, theta: 0 },
  goal: { x: (COLS - 0.5) * CELL, y: (ROWS - 0.5) * CELL, theta: Math.PI / 2 },
  walls,
}
writeFileSync(new URL('../src/maps/maze.json', import.meta.url), JSON.stringify(map).replace(/\],\[/g, '],\n['))
console.log(`maze: ${walls.length} wall segments`)
