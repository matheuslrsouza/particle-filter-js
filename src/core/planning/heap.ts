/** Binary min-heap of integer ids keyed by a float priority, backed by growable typed arrays. */
export class MinHeap {
  private ids = new Int32Array(1024)
  private keys = new Float64Array(1024)
  private n = 0

  get size(): number {
    return this.n
  }

  push(id: number, key: number): void {
    if (this.n === this.ids.length) this.grow()
    // sift up, moving parents down into the "hole" instead of swapping
    let i = this.n++
    while (i > 0) {
      const p = (i - 1) >> 1
      if (this.keys[p] <= key) break
      this.ids[i] = this.ids[p]
      this.keys[i] = this.keys[p]
      i = p
    }
    this.ids[i] = id
    this.keys[i] = key
  }

  pop(): number | undefined {
    if (this.n === 0) return undefined
    const top = this.ids[0]
    const n = --this.n
    if (n > 0) {
      const id = this.ids[n]
      const key = this.keys[n]
      let i = 0
      for (;;) {
        let c = 2 * i + 1
        if (c >= n) break
        if (c + 1 < n && this.keys[c + 1] < this.keys[c]) c++
        if (this.keys[c] >= key) break
        this.ids[i] = this.ids[c]
        this.keys[i] = this.keys[c]
        i = c
      }
      this.ids[i] = id
      this.keys[i] = key
    }
    return top
  }

  private grow(): void {
    const ids = new Int32Array(this.ids.length * 2)
    const keys = new Float64Array(this.keys.length * 2)
    ids.set(this.ids)
    keys.set(this.keys)
    this.ids = ids
    this.keys = keys
  }
}
