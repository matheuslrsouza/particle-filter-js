/**
 * Benchmark of the exploration mode, to compare changes to the planner, controller or map.
 * Usage: npx vite-node scripts/bench-explore.ts [label]
 */
import udacity from '../src/maps/default.json'
import maze from '../src/maps/maze.json'
import { ExplorationSim, type ExploreParams } from '../src/core/sim/exploration'
import type { MapDef } from '../src/core/world/map'

interface Case {
  name: string
  map: MapDef
  goal: { x: number; y: number; theta: number; useHeading: boolean }
  seeds: number[]
}

const mazeDef = maze as unknown as MapDef
const udacityDef = udacity as unknown as MapDef
const cases: Case[] = [
  { name: 'maze', map: mazeDef, goal: { ...mazeDef.goal!, useHeading: true }, seeds: [1, 2, 3] },
  { name: 'udacity→(26,18)', map: udacityDef, goal: { x: 26, y: 18, theta: 0, useHeading: false }, seeds: [1, 2] },
  { name: 'udacity→(15,18)', map: udacityDef, goal: { x: 15, y: 18, theta: 0, useHeading: false }, seeds: [1, 2] },
  { name: 'udacity→(26,3)', map: udacityDef, goal: { x: 26, y: 3, theta: 0, useHeading: false }, seeds: [1, 2] },
]

function runCase(c: Case, seed: number, tweak?: (p: ExploreParams, sim: ExplorationSim) => void) {
  const sim = new ExplorationSim(c.map, undefined, seed)
  // BENCH_NO_SMOOTH=1 disables the Hybrid A* path smoothing, for comparisons
  if (process.env.BENCH_NO_SMOOTH) sim.smoothing.enabled = false
  // BENCH_PID=1 uses the PID controller instead of the MPC
  if (process.env.BENCH_PID) sim.params.controller = 'pid'
  tweak?.(sim.params, sim)
  let bumps = 0
  sim.onMessage = (m) => m.startsWith('Bump') && bumps++
  sim.setGoal(c.goal)
  let driven = 0
  let cteSum = 0
  let cteMax = 0
  let cteN = 0
  const planMs: number[] = []
  let lastPlan = -1
  let i = 0
  for (; i < 50 * 400 && sim.phase !== 'arrived'; i++) {
    sim.tick()
    driven += Math.hypot(sim.car.x - sim.prevCar.x, sim.car.y - sim.prevCar.y)
    if (sim.phase === 'driving' && sim.follower) {
      const e = Math.abs(sim.follower.cte)
      cteSum += e
      cteMax = Math.max(cteMax, e)
      cteN++
    }
    if (sim.plan && sim.planVersion !== lastPlan) {
      lastPlan = sim.planVersion
      planMs.push(sim.plan.result.millis)
    }
  }
  planMs.sort((a, b) => a - b)
  const goalErr = Math.hypot(sim.car.x - c.goal.x, sim.car.y - c.goal.y)
  return {
    arrived: sim.phase === 'arrived',
    time: i * sim.dt,
    bumps,
    cteMean: cteN ? cteSum / cteN : 0,
    cteMax,
    planMedian: planMs[planMs.length >> 1] ?? 0,
    planP90: planMs[Math.floor(planMs.length * 0.9)] ?? 0,
    driven,
    goalErr,
  }
}

{
  const label = process.argv[2] ?? ''
  console.log(`# exploration benchmark ${label}`)
  console.log('case | seed | arrived | time s | bumps | |cte| mean | |cte| max | plan ms med/p90 | driven m | goal err m')
  const all: ReturnType<typeof runCase>[] = []
  for (const c of cases) {
    for (const seed of c.seeds) {
      const r = runCase(c, seed)
      all.push(r)
      console.log(
        `${c.name} | ${seed} | ${r.arrived ? 'yes' : 'NO'} | ${r.time.toFixed(0)} | ${r.bumps} | ${r.cteMean.toFixed(3)} | ${r.cteMax.toFixed(2)} | ` +
          `${r.planMedian.toFixed(0)}/${r.planP90.toFixed(0)} | ${r.driven.toFixed(0)} | ${r.goalErr.toFixed(2)}`,
      )
    }
  }
  const avg = (f: (r: (typeof all)[number]) => number) => (all.reduce((s, r) => s + f(r), 0) / all.length).toFixed(3)
  console.log(
    `TOTAL | arrived ${all.filter((r) => r.arrived).length}/${all.length} | bumps ${all.reduce((s, r) => s + r.bumps, 0)} | ` +
      `time ${avg((r) => r.time)} | |cte| mean ${avg((r) => r.cteMean)} | |cte| max ${avg((r) => r.cteMax)} | driven ${avg((r) => r.driven)}`,
  )
}
