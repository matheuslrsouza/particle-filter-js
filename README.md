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
- **Traffic** (40 cars and 30 pedestrians by default, adjustable in the panel):
  - Other cars drive the lane graph with random routes and the **IDM** (Intelligent Driver Model) for car following.
  - Junctions use a **first-come-first-served reservation**: a car enters only if no conflicting movement (crossing or merging into the same lane) is using the junction, so compatible movements, like opposite straight ones, go together.
  - Pedestrians wait at **crosswalks** near the junctions and cross when no car is about to arrive. Cars stop while someone is on the crosswalk.
  - Our car follows the car ahead like an adaptive cruise control (the MPC gets a speed limit from the gap: v ≤ √(v_lead² + 2·b·(gap − s₀ − T·v_lead))), reserves junctions before entering them and stops for pedestrians. The HUD shows what it is reacting to.
  - Known limitation: the simulation is flat, so roads crossing on a bridge (Shoreline Boulevard over Evelyn Avenue) can draw cars over each other.
- **Perception and tracking** (panel *Perception & tracking*): by default our car does not see the others at their true positions, only what its own sensors tell it.
  - A 360° roof **lidar** (360 beams, 60 m, 2 cm noise) hits buildings, parked cars, barricades, other cars and pedestrians. Points on known geometry (the map) are filtered out (gray); the rest (cyan) are grouped into clusters (scan-line segmentation plus merging of pieces split by occlusion).
  - Each cluster becomes a detection: small ones are pedestrians; the others get an **L-shape fit** (Zhang et al., 2017) and a car-size prior that grows the box away from the sensor, since only the near sides are visible.
  - A **multi-object tracker** keeps a constant-velocity **Kalman filter** per object: Hungarian association on Mahalanobis distances with a gate, tentative tracks confirmed after 3 hits in 5 scans and dropped after missing scans. Tracks are drawn in a color per id, with an arrow for the velocity (where it will be in 1 s).
  - **Prediction**: tracked cars are projected 3 s ahead along the lane graph, pedestrians at constant velocity. Following, junction yielding and crosswalk stops use the tracks and predictions.
  - The panel shows **CLEAR-MOT** metrics against the ground truth (MOTA, recall, position and velocity errors) and can switch the car back to *True positions* to compare.
- The data is converted once and committed (`src/maps/city-mountain-view.json`). To regenerate: `npx vite-node scripts/fetch-osm.ts && npx vite-node scripts/build-city.ts`.

Map data © [OpenStreetMap contributors](https://www.openstreetmap.org/copyright), ODbL.

## Real lidar mode (robot vacuum)

The first step to run these algorithms on a real robot: a broken Xiaomi Robot Vacuum S10+ (B105, made by Dreame), whose 2D lidar ("D2M") we read directly.

- **Wiring (for now):** the lidar stays connected to the robot's own board, which powers it and spins it while cleaning. Two wires soldered on the lidar board (TX and GND pads, next to its 4-pin connector) go to a USB-serial adapter set to 3.3 V: TX → RXI, GND → GND.
- **Protocol** (decoded from a capture; not documented anywhere we found): 115200 baud, 8N1, 3.3 V, 36-byte packets `55 AA 03 08 | speed | start angle | 8 × (distance mm, quality) | end angle | check`, little-endian. Speed = rpm × 64, angle = (raw − 0xA000) / 64 degrees, distance 0x8000 = no echo. It matches the Camsense X1 lidar format. About 310–330 rpm (~5 turns/s) and ~390 points per turn. The checksum algorithm is still unknown, so packets are validated by their structure.
- **The mode** (`?mode=lidar`, *Real lidar* in the mode menu) shows the point cloud live, with the last turns fading out. *Connect USB-serial* uses Web Serial (Chrome or Edge on a computer); *Play sample capture* replays a 30 s capture of the real robot inside a box, so it works without hardware. *Record raw bytes* saves new captures as `.bin` files, which *Open a capture* plays back. *Mounting* rotates or mirrors the scan until angle 0 matches the robot's front.
- **ESP32 over Wi-Fi** (`firmware/lidar_bridge`): an ESP32 DevKit reads the lidar (TX → GPIO16, GND → GND; the signal is 3.3 V, so no level shifter) and sends the raw bytes over a WebSocket on port 81. In the mode, *Connect ESP32 (Wi-Fi)* with the address `lidar.local` (or the IP the ESP32 prints). The browser blocks `ws://` from an https page, so use the local dev server (`npm run dev`) for this one. To build and flash with [arduino-cli](https://arduino.github.io/arduino-cli/):
  ```
  arduino-cli core install esp32:esp32 && arduino-cli lib install WebSockets   # once
  cp firmware/lidar_bridge/secrets.h.example firmware/lidar_bridge/secrets.h     # then fill in your Wi-Fi
  arduino-cli compile --fqbn esp32:esp32:esp32 firmware/lidar_bridge
  arduino-cli upload -p /dev/ttyUSB0 --fqbn esp32:esp32:esp32 firmware/lidar_bridge
  arduino-cli monitor -p /dev/ttyUSB0 -c baudrate=115200                         # shows its address
  ```
  Without hardware, `node scripts/fake-esp32.mjs` plays the sample capture over the same WebSocket (address `localhost:8081`).
- **Mapping (SLAM)** (*Mapping* folder, on by default): while the robot cleans on its own, each lidar turn is matched against the map built so far and then added to it, so the robot moves on a map of the room instead of the room spinning around it. It's Hector SLAM (Kohlbrecher et al., 2011): no wheel odometry, Gauss-Newton matching on a bilinearly interpolated log-odds grid, coarse to fine over 3 resolutions (3, 6 and 12 cm), the map updated after moving 4 cm or 3.5°. Each turn takes 0.2 s and the robot can turn 20° or more meanwhile, so the points are first corrected for that motion (*Undo motion blur of a turn*). These robots also spin in place at up to ~180°/s (35° between turns), more than Gauss-Newton converges from, so each match starts with a brute-force search of ±40° and ±10 cm on the 6 cm map; scans that still fit poorly are not added to the map. Without that search, a 2-minute recording of a bedroom and corridor came out as several rotated copies of the room; with it, the map matches the one the robot's own app draws. `npx vite-node scripts/slam-offline.ts capture.bin out` replays a recording through the SLAM and writes the map (PGM) and a per-turn log. On the sample capture, 97% of each turn's points land on mapped walls, at ~1 ms per turn. *Save the map (PNG)* exports the map with the robot's path.
- **Driving it (click on the map, it goes there)**, without replacing its firmware: the robot's own firmware accepts remote-control steps over the local network (miIO protocol, MIoT property `remote-state`, siid 4 / piid 15, value `{"spdv": forward, "spdw": turn}`; on this firmware 60 ≈ 8 cm/s forward and a positive turn is counter-clockwise). `scripts/robot-bridge.mjs` speaks that protocol (UDP, AES with the device token) and takes `/drive?v=&w=`, `/stop` and `/status` over HTTP; it repeats the last command every 200 ms and stops the robot if the page goes quiet for 600 ms. The token comes from [Xiaomi-cloud-tokens-extractor](https://github.com/PiotrMachowski/Xiaomi-cloud-tokens-extractor) and stays in `~/.robo-xiaomi` (`IP=…`, `TOKEN=…`), outside the repo. In the lidar mode, a click on the map plans a path with A* on the SLAM map (walls inflated by the robot's radius plus a margin, unknown space allowed but costlier, replanned every second); the cell staircase is cut by line of sight and smoothed with the course's gradient smoothing, kept clear of walls. A pure-pursuit follower turns it into speed and turn commands, steering from the pose predicted 0.3 s ahead with its last command (the map lags the robot: steering on the stale pose zigzags), and turns in place when facing away (with hysteresis). The lidar sits 5.75 cm ahead of the body's center, so the SLAM pose (the lidar's) is shifted back for planning and driving, the body being a 35 cm circle around that center; echoes from inside the body (things carried on the robot) are dropped. The follower only aims at points the robot's body can reach in a straight line (pure pursuit otherwise cuts corners by up to its look-ahead), and it stops when the latest lidar turn shows something within ~30 cm right ahead (an obstacle not yet in the map), until the replan goes around it; since a stopped robot doesn't update the map, that scan is added to the map first. A goal that can't be reached (yet) doesn't stop it: the path goes to the reachable cell closest to the goal, the map grows on the way and the replans may open a way; only at that cell, still without a way, does it give up. Paths also keep away from the edge of unseen space (thin things like door frames show up late), and when the robot is asked to drive but doesn't move for 2 s (snagged on something the map doesn't show), it backs up ~10 cm, adds the scan to the map and replans. The status line says what the navigation is doing (driving, turning, blocked, lost). Clearance, look-ahead and latency are adjustable in the panel. *Drive the robot → Armed* lets commands through; space stops, arrows/WASD drive by hand. The robot also stops when the map match drops under 50% or lidar turns stop arriving. The firmware only spins the lidar while it receives remote-control steps (it starts ~1.5 s after the first and stops ~3 s after the last), so the page sends zero-speed steps to keep it on: while armed, or with *Keep the lidar on* to map while pushing the robot by hand. No cleaning is started, but the firmware runs the suction during remote control, so the bridge sets it to the lowest power while armed (and restores it after). The lidar's 0° is not the robot's front (on this robot it was about 180° off): *Calibrate front* drives ~15 cm straight ahead and sets *Mounting → Rotation* from the direction the map says it moved (remembered in the browser). The lidar's angles grow clockwise, so the scan is mirrored by default to make the map right-handed: a recording of a commanded counter-clockwise turn (≈ 29°/s for turn = 40) checks that the map's heading grows the same way. The original firmware keeps its own bumper and cliff safety.
- **Next:** wheel odometry (the encoders, read like the lidar) for the SLAM in long corridors.

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
    city/          OpenStreetMap city: lane-level road network, routing, city driving, traffic, lidar perception and tracking
    lidar/         decoder for the real robot vacuum's lidar
    slam/          Hector SLAM (scan matching, occupancy mapping) and navigation on its map (A*, path follower)
  modes/           the app modes (known map / exploration / city / real lidar)
  real/            real hardware: lidar data sources (ESP32, Web Serial, captures) and the robot bridge client
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
