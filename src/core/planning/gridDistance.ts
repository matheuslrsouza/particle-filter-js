const STRAIGHT = 5
const DIAGONAL = 7 // 7/5 = 1.4 ≈ √2
const UNREACHED = 0x7fffffff

/**
 * 8-connected grid distances from a set of source cells, with Dial's algorithm: a bucket
 * queue instead of a heap, since the only step costs are "straight" and "diagonal".
 * Every cell is settled once, which is several times faster than Dijkstra with a heap.
 *
 * `out[i]` receives the distance in `unit`s per cell (Infinity when unreachable).
 */
export function gridDistance(
  cols: number,
  rows: number,
  sources: number[],
  out: Float32Array,
  unit = 1,
  passable?: (i: number) => boolean,
): Float32Array {
  const n = cols * rows
  const dist = new Int32Array(n).fill(UNREACHED)
  const buckets: number[][] = Array.from({ length: DIAGONAL + 1 }, () => [])
  let pending = 0
  for (const s of sources) {
    dist[s] = 0
    buckets[0].push(s)
    pending++
  }
  for (let d = 0; pending > 0; d++) {
    // pushes always land in other buckets (d+5, d+7), never in the one being drained
    const bucket = buckets[d & DIAGONAL]
    while (bucket.length) {
      const i = bucket.pop()!
      pending--
      if (dist[i] !== d) continue
      const c = i % cols
      const r = (i / cols) | 0
      for (let dr = -1; dr <= 1; dr++) {
        const nr = r + dr
        if (nr < 0 || nr >= rows) continue
        for (let dc = -1; dc <= 1; dc++) {
          const nc = c + dc
          if ((!dc && !dr) || nc < 0 || nc >= cols) continue
          const j = nr * cols + nc
          const nd = d + (dc && dr ? DIAGONAL : STRAIGHT)
          if (nd < dist[j] && (!passable || passable(j))) {
            dist[j] = nd
            buckets[nd & DIAGONAL].push(j)
            pending++
          }
        }
      }
    }
  }
  const k = unit / STRAIGHT
  for (let i = 0; i < n; i++) out[i] = dist[i] === UNREACHED ? Infinity : dist[i] * k
  return out
}
