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
- Near the goal, Hybrid A\* also tries a **Reeds-Shepp curve** (the shortest path for a car that can reverse) from each node straight to the goal pose, Dolgov's "analytic expansion". When the curve is collision-free the search ends, so the car reaches the goal pose exactly and with fewer expansions. The curve is drawn in **purple**.
- The raw path (**yellow**) is made of discrete arcs, so it zigzags. As in the Junior paper, it is then smoothed by an optimization that balances closeness to the original, smoothness and distance to walls, while keeping the start, the goal and the gear switches fixed. Points that would collide or turn tighter than the car can are pulled back towards the original.
- **Cyan** is the current (smoothed) plan, **magenta** the plan it replaced. The **green outlines** are the car's footprints, and red blocks are the walls discovered so far.
- The car is driven by an **MPC** (model predictive control), like current stacks (Autoware, Apollo, openpilot) and the CarND MPC project. Every 0.1 s it optimizes acceleration and steering rate over the next 2 s on the kinematic bicycle model, solved with iLQR (iterative LQR) in under a millisecond, with no dependencies. It tracks a time-indexed reference sampled from the path and a **speed profile** that slows down in curves (bounded lateral acceleration) and brakes to a stop at gear switches and at the goal. The **green line** is the MPC's prediction and the **yellow dots** are the reference. The PID of the course can still be picked in the panel to compare.
- **Click** sets a goal. **Shift+drag** (or the *Drag sets goal heading* toggle) also sets the heading the car must arrive with, like a parking spot.
- Localization is a high-precision GPS (true pose + small noise). The particle filter can't be used here because it needs the map the car doesn't have yet.
- The map is a **log-odds occupancy grid**. Every cell accumulates evidence (a lidar hit adds, a beam passing through subtracts), so it only becomes a wall after repeated hits and can become free again. The lidar returns ~1% bogus readings; with the *Binary* mapping (one hit = wall forever) they leave phantom walls that can block corridors. Both can be compared in the panel.
- If the car touches a wall end that slipped between lidar beams, the bumper maps it. The car then picks the short escape maneuver (forward/reverse × left/straight/right) that gains the most room, and replans.
- Two maps: a generated 40 × 27.5 m maze and the Udacity map. The maze is a "perfect" maze (full of dead ends) with a few open plazas. Its seed was picked with `npx vite-node scripts/pick-maze.ts`, which simulates the exploration for many seeds and compares the distance driven with the optimal path. With seed 12 the car first explores a whole wrong region, then backs out of several dead ends (about 1.7× the optimal distance). Regenerate it with `node scripts/generate-maze.mjs [seed]`.

## City mode (Mountain View, CA)

Pick **Mode → City** in the panel, or open `?mode=city`. Downtown Mountain View (Castro Street and around) is rebuilt from **OpenStreetMap**:

- Roads become a lane-level network for right-hand traffic: lanes per direction (from the `lanes` tags; an odd count on a two-way US street is read as a center turn lane), junctions, and connectors through each junction for the allowed movements (straight, right from the rightmost lane, left from the leftmost lane, no U-turns). Junctions that are too close together are merged into one composite intersection.
- Buildings are extruded from their footprints (height from `building:levels`, otherwise estimated from the footprint area). Parks, sidewalks, lane markings and trees complete the scene.
- The *Lane graph* layer shows the lanes (cyan) and the connectors: straight (white), left (orange), right (green).
- **Click a street** (or *Random destination*) and the car drives there. The route is the fastest one on the lane graph (A\*, travel time at the speed limits), keeping right and changing lanes only when needed, e.g. before a left turn. The car is real size (2.8 m wheelbase) and is driven by the same MPC as the maze, with a speed profile capped by each street's `maxspeed` that slows down for turns. The HUD shows the street name, speed vs limit and the lane error.
- **Localization is an EKF** that fuses a GPS with wheel odometry (wheel speed + steering angle). Its state is [x, y, θ, steering bias, wheel speed scale error], so it also learns the biases of the car's sensors. The heading and the biases are never measured: they become observable from the motion. GPS fixes are gated by Mahalanobis distance, which rejects multipath jumps.
  - The simulated GPS has a slowly wandering error (Gauss-Markov), like real receivers, so no filter can remove it alone. Pick the receiver in the panel: **RTK** (~2 cm, robotaxi development), **dual-frequency** (~0.5 m, default) or **consumer** (~1.5 m). With a consumer GPS the car visibly drifts in its lane. That is the motivation for lane detection with a camera (next step).
  - *Car uses* compares the EKF with a single sensor: GPS only jumps around, odometry only drifts away.
  - The wireframe ghost car is where the car believes it is, with its 2σ ellipse. GPS fixes are green (accepted) or red (rejected).
- **Urban Challenge missions**: *Start mission* picks checkpoints spread over the city (like the DARPA MDF), and the car drives through them in order. Markers turn green as they are reached.
- **Blocked roads**: *Block a road ahead* puts barricades across a two-way street of the route. The car doesn't know about them until its **forward lidar** (31 beams, 50 m) sees them. Then it stops, **turns around with Hybrid A\*** on a local grid of the drivable surface (back-and-forth maneuvers on narrow streets, planned at 85% of the steering limit to leave the controller some room), and reroutes on the lane graph avoiding that road. US residential streets are modeled curb to curb (~11 m with the parking lanes), which is what makes the U-turns possible.
- **Parking** (*Park between two cars*): the surface lots of OpenStreetMap get rows of 90° spots, about 70% of them taken. The car drives to the lot, then parks in a free spot between two parked cars with **Hybrid A\*** (Reeds-Shepp shots, front or back in), around the parked cars. While parking, the car localizes with relative sensing (like the ultrasonic sensors and cameras of real cars), centimeter-accurate, instead of the GPS. The planner checks the real rectangle of the car (circles along its axis plus its corners), and obstacles are rasterized conservatively, so planned maneuvers never touch the parked cars.
- The *Camera* menu's orbit and top-down views center on the car; *Whole city* frames the whole map.
- The data is converted once and committed (`src/maps/city-mountain-view.json`). To regenerate: `npx vite-node scripts/fetch-osm.ts && npx vite-node scripts/build-city.ts`.

Map data © [OpenStreetMap contributors](https://www.openstreetmap.org/copyright), ODbL.

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
    control/       MPC (iLQR) and speed profile, PID path follower (with forward/reverse gears), twiddle
    sim/           simulations: particle filter (known map) and exploration
    city/          OpenStreetMap city: lane-level road network, routing, city driving
  modes/           the app modes (known map / exploration / city)
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
