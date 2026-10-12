const TWO_PI = Math.PI * 2

/** Wraps an angle to the interval [-PI, PI). */
export function wrapAngle(a: number): number {
  a = (a + Math.PI) % TWO_PI
  if (a < 0) a += TWO_PI
  return a - Math.PI
}

export const deg2rad = (d: number): number => (d * Math.PI) / 180
