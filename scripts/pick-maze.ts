/**
 * Searches maze seeds for an interesting exploration: the car should walk into dead ends,
 * turn back and replan (distance driven well above the optimal path), but still finish.
 * Usage: npx vite-node scripts/pick-maze.ts [fromSeed] [toSeed] [lidarRange]
 */
import { hybridAStar } from '../src/core/planning/hybridAStar'
import { ExplorationSim } from '../src/core/sim/exploration'
import { KnownMap } from '../src/core/world/knownMap'
import { loadWorld, type MapDef } from '../src/core/world/map'
import { generateMaze } from './maze.mjs'

const [from = 1, to = 30, range = 3] = process.argv.slice(2).map(Number)

for (let seed = from; seed <= to; seed++) {
  const def = generateMaze({ seed }) as MapDef
  const world = loadWorld(def)
  const full = new KnownMap(world.width, world.height, 0.25)
  full.revealWalls(world.walls)
  const best = hybridAStar(full, world.start, { ...world.goal!, useHeading: false }, undefined)
  if (!best) {
    console.log(seed, 'no optimal path')
    continue
  }
  let optimal = 0
  for (let i = 1; i < best.path.length; i++) optimal += Math.hypot(best.path[i].x - best.path[i - 1].x, best.path[i].y - best.path[i - 1].y)

  const sim = new ExplorationSim(def, undefined, 1)
  sim.params.lidarRange = range
  let bumps = 0
  sim.onMessage = (m) => m.startsWith('Bump') && bumps++
  sim.setGoal({ ...world.goal!, useHeading: false })
  let driven = 0
  let reverseTime = 0
  let turnbacks = 0
  let prevHeading = sim.car.theta
  let headingAtLastCheck = sim.car.theta
  let i = 0
  for (; i < 50 * 400 && sim.phase !== 'arrived'; i++) {
    sim.tick()
    driven += Math.hypot(sim.car.x - sim.prevCar.x, sim.car.y - sim.prevCar.y)
    if (sim.car.speed < 0) reverseTime += sim.dt
    prevHeading = sim.car.theta
    // a "turnback": the heading flips by more than 150° within 5 seconds
    if (i % 250 === 0) {
      const d = Math.abs(Math.atan2(Math.sin(prevHeading - headingAtLastCheck), Math.cos(prevHeading - headingAtLastCheck)))
      if (d > (150 * Math.PI) / 180) turnbacks++
      headingAtLastCheck = prevHeading
    }
  }
  console.log(
    `seed ${seed}: ${sim.phase} in ${(i * sim.dt).toFixed(0)}s, driven ${driven.toFixed(0)}m vs optimal ${optimal.toFixed(0)}m ` +
      `(x${(driven / optimal).toFixed(2)}), reverse ${reverseTime.toFixed(0)}s, turnbacks ${turnbacks}, replans ${sim.replans}, bumps ${bumps}, explored ${(sim.known.explored() * 100).toFixed(0)}%`,
  )
}
