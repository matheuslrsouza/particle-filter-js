# Robot JS 3D

A little car drives through a maze using the classic robotics pipeline from Udacity's free course
<a href="https://www.udacity.com/course/artificial-intelligence-for-robotics--cs373">Artificial Intelligence for Robotics</a> (CS373):

```
lidar (raycasting) → particle filter → A* → path smoothing → PID → kinematic bicycle model
```

The car never knows where it really is: it plans and steers using only the pose estimated by the particle filter.

**Demo:** https://matheuslrsouza.github.io/particle-filter-js/

## Using it

- **Click the floor** to set a goal. The A* expansion (blue), the raw path (magenta) and the smoothed path (white) are animated, then the car drives.
- **Drag** to orbit, **scroll** to zoom. Switch to the *Chase* or *Top-down* camera in the panel.
- **Kidnap robot** teleports the car without telling the filter. Watch random particles get injected until it finds the car again (augmented MCL).
- **Forget position** spreads the particles over the whole map (global localization). It works better with more particles.
- **Layers** turns each visualization on or off: particles, lidar, belief (green ghost car + 2σ ellipse), A*, paths, occupancy grid and trails.
- **PID controller → Auto-tune (twiddle)** tunes the gains on the current path in a Web Worker.
- The chart shows the cross-track error (CTE) and the localization error over the last 30 s.

## Exploration mode (Hybrid A\*)

Pick **Mode → Exploration** in the panel, or open `?mode=explore`. This recreates the
["A\* in action"](https://youtu.be/qXZt-B7iUyw) demo of the course, where Stanford's Junior drives through a maze it has never seen:

- The car only knows its goal. Unknown space is assumed free, the lidar (3 m range) discovers walls as the car drives, and **Hybrid A\*** replans whenever the current plan runs into one.
- Hybrid A\* searches over continuous poses (x, y, θ) with arcs the car can actually drive, forward and in reverse. Its search tree is drawn in **orange (forward)** and **blue (reverse)**. Reverse moves show up in dead ends.
- The raw path (**yellow**) is made of discrete arcs, so it zigzags. As in the Junior paper, it is then smoothed by an optimization that balances closeness to the original, smoothness and distance to walls, while keeping the start, the goal and the gear switches fixed. Points that would collide or turn tighter than the car can are pulled back towards the original.
- **Cyan** is the current (smoothed) plan, **magenta** the plan it replaced. The **green outlines** are the car's footprints, and red blocks are the walls discovered so far.
- **Click** sets a goal. **Shift+drag** (or the *Drag sets goal heading* toggle) also sets the heading the car must arrive with, like a parking spot.
- Localization is a high-precision GPS (true pose + small noise). The particle filter can't be used here because it needs the map the car doesn't have yet.
- If the car touches a wall end that slipped between lidar beams, a bumper maps it, backs off and replans.
- Two maps: a generated 40 × 27.5 m maze and the Udacity map. The maze is a "perfect" maze (full of dead ends) with a few open plazas. Its seed was picked with `npx vite-node scripts/pick-maze.ts`, which simulates the exploration for many seeds and compares the distance driven with the optimal path. With seed 12 the car first explores a whole wrong region, then backs out of several dead ends (about 1.7× the optimal distance). Regenerate it with `node scripts/generate-maze.mjs [seed]`.

## Running locally

```bash
npm install
npm run dev      # http://localhost:5173/particle-filter-js/
npm test         # unit tests of the simulation core
npm run build
```

## Project structure

```
src/
  core/            pure TypeScript simulation, no three.js (unit tested)
    math/          angles, seedable RNG, vectors
    world/         map loading, occupancy grid, map built from lidar (exploration)
    sensors/       lidar raycasting
    vehicle/       kinematic bicycle model
    localization/  particle filter (Monte Carlo localization)
    planning/      A* (binary heap, 8-connected), Hybrid A* and path smoothing
    control/       PID path follower (with forward/reverse gears) and twiddle
    sim/           simulations: particle filter (known map) and exploration
  modes/           the two app modes (known map / exploration)
  render/          three.js scene, car, particles, lidar, plans, search tree, cameras
  ui/              HUD, control panel, twiddle worker
  maps/            maps in meters (JSON)
```

## What changed from the 2D version

The original p5.js version (2020) is kept in the [`p5-js-2d`](../../tree/p5-js-2d) branch. Besides the move to 3D, these bugs were fixed:

- **PID**: the D and I terms were multiplied together instead of summed, so only P was really active.
- **Particle filter**: the weights used the robot's *true* heading. The filter now compares the ranges beam by beam. Log-likelihoods avoid the underflow of multiplying 36 small Gaussians, and the heading is now estimated as well.
- **Motion noise**: the heading noise read `std[3]` from a 3-element array, so the configured value was ignored.
- **A\***: the squared Euclidean heuristic was not admissible, and the open list was re-sorted on every iteration.
- **Smoothing**: one of the update terms never ran (`path.len`), and the smoothed path could cut through walls.
- **Motion model**: the steering angle was used as a yaw rate. It is now a kinematic bicycle model with a steering-rate limit.

## Credits

- Course: Sebastian Thrun, Udacity CS373.
- Raycasting was originally based on Daniel Shiffman's <a href="https://www.youtube.com/watch?v=TOEi6T2mtHo">Coding Challenge</a>.
