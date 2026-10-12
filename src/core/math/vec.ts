export interface Vec2 {
  x: number
  y: number
}

export interface Pose {
  x: number
  y: number
  /** heading in radians, measured from +x towards +y */
  theta: number
}

export const dist = (a: Vec2, b: Vec2): number => Math.hypot(a.x - b.x, a.y - b.y)

export const lerp = (a: number, b: number, t: number): number => a + (b - a) * t

export const clamp = (v: number, lo: number, hi: number): number => Math.min(hi, Math.max(lo, v))
