// The tests read binary fixtures with node:fs; only that call is declared (no @types/node).
declare module 'node:fs' {
  export function readFileSync(path: URL | string): Uint8Array
}
