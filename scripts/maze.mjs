/**
 * Maze generator: recursive backtracker (a "perfect" maze, full of dead ends), plus a few
 * extra openings (loops) and open "plazas" where the car can turn around.
 */
export function generateMaze({ cols = 16, rows = 11, cell = 2.5, extraOpenings = 0.03, plazas = 3, seed = 1 } = {}) {
  let s = seed >>> 0 || 1
  const rand = () => ((s = (s * 1664525 + 1013904223) >>> 0) / 4294967296)

  // h[r][c]: wall below cell (c, r); v[r][c]: wall right of cell (c, r)
  const h = Array.from({ length: rows - 1 }, () => Array(cols).fill(true))
  const v = Array.from({ length: rows }, () => Array(cols - 1).fill(true))
  const seen = Array.from({ length: rows }, () => Array(cols).fill(false))
  const stack = [[0, 0]]
  seen[0][0] = true
  while (stack.length) {
    const [c, r] = stack[stack.length - 1]
    const options = [[c + 1, r], [c - 1, r], [c, r + 1], [c, r - 1]].filter(
      ([nc, nr]) => nc >= 0 && nr >= 0 && nc < cols && nr < rows && !seen[nr][nc],
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
  for (const row of [...h, ...v]) for (let i = 0; i < row.length; i++) if (row[i] && rand() < extraOpenings) row[i] = false

  // plazas: 2x2 blocks with the inner walls removed
  for (let k = 0; k < plazas; k++) {
    const c = 1 + Math.floor(rand() * (cols - 3))
    const r = 1 + Math.floor(rand() * (rows - 3))
    v[r][c] = v[r + 1][c] = false
    h[r][c] = h[r][c + 1] = false
  }

  // merge consecutive wall pieces into long segments
  const walls = []
  for (let r = 0; r < rows - 1; r++) {
    for (let c = 0; c < cols; ) {
      if (!h[r][c]) { c++; continue }
      const start = c
      while (c < cols && h[r][c]) c++
      walls.push([start * cell, (r + 1) * cell, c * cell, (r + 1) * cell])
    }
  }
  for (let c = 0; c < cols - 1; c++) {
    for (let r = 0; r < rows; ) {
      if (!v[r][c]) { r++; continue }
      const start = r
      while (r < rows && v[r][c]) r++
      walls.push([(c + 1) * cell, start * cell, (c + 1) * cell, r * cell])
    }
  }

  return {
    name: 'Maze',
    width: cols * cell,
    height: rows * cell,
    start: { x: cell / 2 - 0.25, y: cell / 2, theta: 0 },
    goal: { x: (cols - 0.5) * cell, y: (rows - 0.5) * cell, theta: Math.PI / 2 },
    walls,
  }
}

export const toJson = (map) => JSON.stringify(map).replace(/\],\[/g, '],\n[')
