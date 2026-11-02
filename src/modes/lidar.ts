import GUI from 'lil-gui'
import * as THREE from 'three'
import { CamsenseParser, RevolutionAssembler, type LidarSample, type Revolution } from '../core/lidar/camsense'
import type { Pose, Vec2 } from '../core/math/vec'
import { HectorSlam, scanToPoints } from '../core/slam/hector'
import { DEFAULT_NAV, PathFollower, planRoute } from '../core/slam/navigate'
import { RobotBridge } from '../real/robot'
import sampleUrl from '../real/sample-capture.bin?url'
import { hasWebSerial, ReplaySource, SerialSource, WebSocketSource, type ByteSource } from '../real/sources'
import { CameraRig } from '../render/cameras'
import { createRenderer, createScene } from '../render/scene'
import { Hud } from '../ui/hud'
import { addModeSelector } from '../ui/mode'
import { FURNITURE, locate, MOVING, ObjectMap, PI_CAMERA_V2, toMap, type Detection } from '../core/vision/objects'

/** the view is a 10 m square with the lidar in the middle */
const AREA = { width: 10, height: 10 }
const CENTER = new THREE.Vector3(AREA.width / 2, 0, AREA.height / 2)
const MAX_TURNS = 30
const MAX_POINTS = 440
/** Xiaomi S10+ body: 35 cm across */
const ROBOT_RADIUS = 0.175
/** the lidar sits 5.75 cm ahead of the body's center (11.75 cm from the front, 23.25 cm from the back) */
const LIDAR_AHEAD = 0.0575
/** echoes inside the body (+3 cm) are things carried on the robot: power bank, ESP32, wires */
const BODY_ECHO = ROBOT_RADIUS + 0.03

/** a turn as drawn: its usable samples, the matching points in the robot frame and where the robot was */
interface Turn {
  rev: Revolution
  samples: LidarSample[]
  points: Vec2[]
  pose: Pose
}

/**
 * Live view of the real lidar from the Xiaomi robot vacuum: bytes from the ESP32 (Wi-Fi), the
 * USB-serial adapter or a recorded capture → packet decoder → full turns → point cloud, with the
 * last turns fading out. With mapping on, each turn is matched against the map built so far
 * (Hector SLAM), so the robot moves on a map of the room instead of the room moving around it.
 *
 * Frames: the SLAM works in x forward / y along the lidar's angle; on screen x_slam → −z (up) and
 * y_slam → +x, which keeps the plain view as it was (angle 0 up).
 */
export function startLidar(): void {
  const container = document.getElementById('app')!
  const renderer = createRenderer(container)
  const scene = createScene(AREA)
  scene.fog = null
  const rig = new CameraRig(renderer.domElement, AREA, { near: 0.02, far: 100, minDistance: 0.5, maxDistance: 25 })
  rig.setMode('top', CENTER2D(), 3)
  const hud = new Hud('Connect the ESP32 or the USB-serial adapter or play the sample capture · <b>right-drag</b> to pan · <b>scroll</b> to zoom')
  const { log, logOnce } = createLogDrawer()
  // every toast goes to the log too
  const toast = hud.message.bind(hud)
  hud.message = (m: string) => {
    toast(m)
    log(m)
  }
  document.getElementById('charts')!.hidden = true

  const world = new THREE.Group()
  world.position.copy(CENTER)
  // the map is right-handed (x forward, y to the left): drawn with x up, y must go to the screen's
  // left, so the world is mirrored on the screen's x axis (three.js flips the face winding itself)
  world.scale.x = -1
  scene.add(world)
  const robot = robotBody()
  const rings = rangeRings()
  world.add(rings, robot)

  // the map: one texel per cell of the finest grid, transparent where unknown
  let slam = new HectorSlam()
  const mapSize = slam.maps[0].cells
  const mapPixels = new Uint8Array(mapSize * mapSize * 4)
  const mapTexture = new THREE.DataTexture(mapPixels, mapSize, mapSize, THREE.RGBAFormat)
  mapTexture.magFilter = THREE.NearestFilter
  const mapExtent = mapSize * slam.maps[0].resolution
  const mapMesh = new THREE.Mesh(
    new THREE.PlaneGeometry(mapExtent, mapExtent),
    new THREE.MeshBasicMaterial({ map: mapTexture, transparent: true, depthWrite: false }),
  )
  mapMesh.rotation.x = -Math.PI / 2
  mapMesh.position.y = 0.002
  world.add(mapMesh)
  const pathGeo = new THREE.BufferGeometry()
  const path = new THREE.Line(pathGeo, new THREE.LineBasicMaterial({ color: 0xff2e88 }))
  path.position.y = 0.01
  path.frustumCulled = false
  world.add(path)
  let mapDirty = false
  let lastMapUpload = 0

  const positions = new Float32Array(MAX_TURNS * MAX_POINTS * 3)
  const colors = new Float32Array(MAX_TURNS * MAX_POINTS * 3)
  const geo = new THREE.BufferGeometry()
  geo.setAttribute('position', new THREE.BufferAttribute(positions, 3))
  geo.setAttribute('color', new THREE.BufferAttribute(colors, 3))
  const material = new THREE.PointsMaterial({ size: 0.04, vertexColors: true, sizeAttenuation: true })
  const cloud = new THREE.Points(geo, material)
  cloud.frustumCulled = false
  world.add(cloud)

  const view = {
    trail: 6,
    pointSize: 4,
    minQuality: 1,
    maxRange: 8,
    /** closer echoes are the lidar's own head; echoes inside the body are dropped separately */
    minRange: 0.12,
    // the lidar's angles grow clockwise: mirrored, the map is right-handed (counter-clockwise turns
    // are positive), as the robot's remote control and the navigation expect
    mirror: true,
    /** where the robot's front is in the lidar's angles (set by "Calibrate front", remembered) */
    rotation: 0,
    colorBy: 'age' as 'age' | 'quality' | 'distance',
    freeze: false,
    mapping: true,
    deskew: true,
    follow: true,
  }

  let source: ByteSource | null = null
  let parser = new CamsenseParser()
  let assembler = new RevolutionAssembler()
  let turns: Turn[] = []
  let recording: Uint8Array[] | null = null
  const rate = { turns: 0, since: performance.now(), hz: 0 }

  const onBytes = (bytes: Uint8Array) => {
    if (recording) recording.push(bytes.slice())
    for (const p of parser.push(bytes)) {
      const rev = assembler.add(p)
      if (!rev) continue
      rate.turns++
      if (view.freeze) continue
      turns.push(process(rev))
      if (turns.length > MAX_TURNS) turns.shift()
      redraw()
      if (view.mapping) navigateStep()
    }
  }

  async function use(next: ByteSource) {
    await stop()
    parser = new CamsenseParser()
    assembler = new RevolutionAssembler()
    resetMap()
    try {
      await next.start(onBytes, (error) => {
        if (source !== next) return
        source = null
        hud.message(error ? `Disconnected: ${error}` : 'Source stopped')
      })
      source = next
      hud.message(`${next.label} connected`)
    } catch (e) {
      // closing the port picker is not an error worth shouting about
      const msg = (e as Error).message
      if (!/No port selected/i.test(msg)) hud.message(`Could not start: ${msg}`)
    }
  }

  async function stop() {
    const s = source
    source = null
    await s?.stop()
  }

  const usable = (s: LidarSample) => !Number.isNaN(s.distance) && s.distance >= view.minRange && s.distance <= view.maxRange && s.quality >= view.minQuality

  /** One turn → points in the robot frame; with mapping on, also matched and added to the map. */
  function process(rev: Revolution): Turn {
    const raw = rev.samples.filter(usable)
    const opts = { mirror: view.mirror, rotation: view.rotation, minRange: 0, maxRange: Infinity, minQuality: 0 }
    const all = scanToPoints(raw, { ...opts, motion: view.mapping && view.deskew ? slam.motion : undefined })
    // drop echoes from inside the robot's own body (the body's center is behind the lidar)
    // judged on the points as measured: the deskew moves echoes of things carried on the robot as if
    // they were fixed in the room, pushing them out of the body at speed (a trail of fake obstacles)
    const measured = view.mapping && view.deskew ? scanToPoints(raw, opts) : all
    const keep = measured.map((q) => Math.hypot(q.x + LIDAR_AHEAD, q.y) > BODY_ECHO)
    // and echoes from people or animals the camera sees (checked with the last pose: close enough)
    if (movingNow.length) {
      const c = Math.cos(slam.pose.theta)
      const sn = Math.sin(slam.pose.theta)
      all.forEach((q, i) => {
        if (!keep[i]) return
        const mx = slam.pose.x + c * q.x - sn * q.y
        const my = slam.pose.y + sn * q.x + c * q.y
        if (movingNow.some((z) => Math.hypot(mx - z.x, my - z.y) < 0.45)) keep[i] = false
      })
    }
    const samples = raw.filter((_, i) => keep[i])
    const points = all.filter((_, i) => keep[i])
    if (!view.mapping) return { rev, samples, points, pose: { x: 0, y: 0, theta: 0 } }
    const before = slam.inserts
    const pose = { ...slam.addScan(points) }
    if (slam.inserts !== before) mapDirty = true
    return { rev, samples, points, pose }
  }

  function resetMap() {
    clearGoalIfAny()
    clearObjectsIfAny()
    slam = new HectorSlam()
    turns = []
    mapPixels.fill(0)
    mapTexture.needsUpdate = true
    setLine(pathGeo, [])
    redraw()
  }

  /** where the camera saw people or animals (map frame), lately: their echoes stay out of the map */
  let movingNow: { x: number; y: number; until: number }[] = []
  // set once the driving and camera sections below exist
  let clearGoalIfAny = () => {}
  let clearObjectsIfAny = () => {}

  /** Map cells → texels: free cells dark blue, walls bright, unknown transparent. */
  function uploadMap() {
    const g = slam.maps[0]
    for (let ix = 0; ix < mapSize; ix++) {
      for (let iy = 0; iy < mapSize; iy++) {
        const l = g.logOdds[iy * mapSize + ix]
        // texture row = ix, column = iy (see the frame note above)
        const k = (ix * mapSize + iy) * 4
        if (l > 0.5) mapPixels.set([225, 240, 255, Math.min(255, 120 + l * 40)], k)
        else if (l < -0.2) mapPixels.set([40, 70, 120, Math.min(150, -l * 60)], k)
        else mapPixels[k + 3] = 0
      }
    }
    mapTexture.needsUpdate = true
    setLine(pathGeo, slam.trajectory.map((p) => new THREE.Vector3(p.y, 0, -p.x)))
  }

  function pointColor(s: LidarSample, age: number, out: THREE.Color) {
    if (view.colorBy === 'quality') out.setHSL(0.66 - Math.min(s.quality / 120, 1) * 0.66, 0.9, 0.55)
    else if (view.colorBy === 'distance') out.setHSL(Math.min(s.distance / 4, 1) * 0.8, 0.9, 0.55)
    else {
      // newest turn bright green, older turns fade to the background
      const k = age === 0 ? 1 : 0.55 * (1 - age / Math.max(view.trail, 1))
      out.setRGB(0.22 * k, 1 * k, 0.53 * k)
    }
  }

  const tmp = new THREE.Color()
  function redraw() {
    let n = 0
    const shown = turns.slice(-Math.max(1, view.trail))
    shown.forEach((t, idx) => {
      const age = shown.length - 1 - idx
      const c = Math.cos(t.pose.theta)
      const s = Math.sin(t.pose.theta)
      t.points.forEach((q, i) => {
        if (n >= positions.length / 3) return
        const wx = t.pose.x + c * q.x - s * q.y
        const wy = t.pose.y + s * q.x + c * q.y
        positions.set([wy, 0.02, -wx], n * 3)
        pointColor(t.samples[i], age, tmp)
        colors.set([tmp.r, tmp.g, tmp.b], n * 3)
        n++
      })
    })
    const p = turns[turns.length - 1]?.pose ?? { x: 0, y: 0, theta: 0 }
    robot.position.set(p.y, 0, -p.x)
    robot.rotation.y = -p.theta
    rings.position.copy(robot.position)
    geo.setDrawRange(0, n)
    geo.attributes.position.needsUpdate = true
    geo.attributes.color.needsUpdate = true
  }

  const gui = new GUI({ title: 'Controls' })
  if (window.innerWidth < 640) gui.close()
  addModeSelector(gui, 'lidar')
  const src = gui.addFolder('Source')
  src
    .add(
      {
        connect: () => (hasWebSerial() ? use(new SerialSource(115200)) : hud.message('Web Serial needs Chrome or Edge on a computer')),
      },
      'connect',
    )
    .name('Connect USB-serial (115200)')
  // the ESP32 bridge over Wi-Fi; the address is remembered in this browser
  const esp = { host: 'lidar.local' }
  try {
    esp.host = localStorage.getItem('lidar.espHost') ?? esp.host
  } catch {
    /* storage blocked: keep the default */
  }
  src
    .add(esp, 'host')
    .name('ESP32 address')
    .onFinishChange((h: string) => {
      try {
        localStorage.setItem('lidar.espHost', h)
      } catch {
        /* storage blocked */
      }
    })
  src.add({ esp: () => {
    // "lidar.local" → port 81 (the firmware); "host:port" for anything else (scripts/fake-esp32.mjs)
    const h = esp.host.trim()
    return use(new WebSocketSource(`ws://${h.includes(':') ? h : `${h}:81`}`))
  } }, 'esp').name('Connect ESP32 (Wi-Fi)')
  src
    .add(
      {
        sample: async () => {
          const bytes = new Uint8Array(await (await fetch(sampleUrl)).arrayBuffer())
          await use(new ReplaySource('Sample capture', bytes))
        },
      },
      'sample',
    )
    .name('Play sample capture')
  const fileInput = Object.assign(document.createElement('input'), { type: 'file', accept: '.bin,application/octet-stream' })
  fileInput.onchange = async () => {
    const f = fileInput.files?.[0]
    if (f) await use(new ReplaySource(f.name, new Uint8Array(await f.arrayBuffer())))
    fileInput.value = ''
  }
  src.add({ open: () => fileInput.click() }, 'open').name('Open a capture (.bin)…')
  src.add({ stop: () => stop().then(() => hud.message('Stopped')) }, 'stop').name('Stop')
  const recCtrl = src
    .add(
      {
        record: () => {
          if (!recording) {
            recording = []
            recCtrl.name('Stop recording and save')
            hud.message('Recording raw bytes…')
            return
          }
          const blob = new Blob(recording as BlobPart[], { type: 'application/octet-stream' })
          recording = null
          recCtrl.name('Record raw bytes')
          const a = Object.assign(document.createElement('a'), {
            href: URL.createObjectURL(blob),
            download: `lidar-${new Date().toISOString().replace(/[:.]/g, '-')}.bin`,
          })
          a.click()
          setTimeout(() => URL.revokeObjectURL(a.href), 1000)
        },
      },
      'record',
    )
    .name('Record raw bytes')

  const look = gui.addFolder('View')
  look.add(view, 'freeze').name('Freeze')
  look.add(view, 'trail', 1, MAX_TURNS, 1).name('Turns shown').onChange(redraw)
  look.add(view, 'colorBy', { 'Age (newest bright)': 'age', Quality: 'quality', Distance: 'distance' }).name('Color by').onChange(redraw)
  look.add(view, 'pointSize', 1, 8, 0.5).name('Point size (cm)').onChange((v: number) => (material.size = v / 100))
  look.add(view, 'minQuality', 0, 100, 1).name('Min quality').onChange(redraw)
  look.add(view, 'maxRange', 0.5, 10, 0.5).name('Max range (m)').onChange(redraw)
  look.add(view, 'minRange', 0.1, 0.5, 0.01).name('Min range (m)')
  look
    .add({ camera: 'top' }, 'camera', { 'Top-down': 'top', Orbit: 'orbit' })
    .name('Camera')
    .onChange((m: 'top' | 'orbit') => rig.setMode(m, CENTER2D(), m === 'top' ? 3 : 2.5))
  const mapF = gui.addFolder('Mapping (SLAM)')
  mapF.add(view, 'mapping').name('Build a map').onChange(resetMap)
  mapF.add(view, 'deskew').name('Undo motion blur of a turn')
  mapF.add(view, 'follow').name('Camera follows the robot')
  mapF.add({ reset: resetMap }, 'reset').name('Clear the map')
  mapF.add({ save: () => saveMapImage() }, 'save').name('Save the map (PNG)')
  try {
    const r = localStorage.getItem('lidar.rotation')
    if (r !== null) view.rotation = Number(r)
  } catch {
    /* storage blocked */
  }
  const cal = gui.addFolder('Mounting')
  const rotCtrl = cal.add(view, 'rotation', -180, 180, 1).name('Rotation (°)').onChange((r: number) => {
    try {
      localStorage.setItem('lidar.rotation', String(r))
    } catch {
      /* storage blocked */
    }
    resetMap()
  })
  cal.add(view, 'mirror').name('Mirror (turns the other way)').onChange(resetMap)
  cal.close()
  material.size = view.pointSize / 100

  // ---- driving the real robot (scripts/robot-bridge.mjs talks to its own firmware) ----
  const drive = { armed: false, lidarOn: false, bridge: 'http://localhost:8090', vScale: 720, wScale: 80, manual: { v: 0, w: 0 } }
  const bridge = new RobotBridge(drive.bridge)
  let goal: Vec2 | null = null
  let plan: Vec2[] | null = null
  const follower = new PathFollower([], DEFAULT_NAV)
  let lastPlanMsg = ''
  let partial = false
  let navState = ''
  const setNav = (s: string) => {
    navState = s
    logOnce(s)
  }
  let stuckSince: { at: number; x: number; y: number } | null = null
  let backUpUntil = 0
  let lastNavTurn = 0
  const planGeo = new THREE.BufferGeometry()
  const planLine = new THREE.Line(planGeo, new THREE.LineBasicMaterial({ color: 0x4cc9f0 }))
  planLine.position.y = 0.015
  planLine.frustumCulled = false
  const goalMarker = new THREE.Mesh(new THREE.RingGeometry(0.06, 0.1, 32), new THREE.MeshBasicMaterial({ color: 0x4cc9f0, side: THREE.DoubleSide }))
  goalMarker.rotation.x = -Math.PI / 2
  goalMarker.visible = false
  world.add(planLine, goalMarker)
  const toScreen = (p: Vec2) => new THREE.Vector3(p.y, 0, -p.x)
  // the SLAM tracks the lidar; planning and driving use the body's center (it turns around it)
  const center = (p: Pose): Pose => ({ x: p.x - Math.cos(p.theta) * LIDAR_AHEAD, y: p.y - Math.sin(p.theta) * LIDAR_AHEAD, theta: p.theta })

  function clearGoal(message?: string) {
    goal = null
    plan = null
    goalMarker.visible = false
    setLine(planGeo, [])
    void bridge.stop()
    if (message) hud.message(message)
  }

  function replan(): boolean {
    if (!goal) return false
    const route = planRoute(slam.maps[0], center(slam.pose), goal, DEFAULT_NAV)
    plan = route?.path ?? null
    const len = plan ? plan.reduce((a, q, i) => (i ? a + Math.hypot(q.x - plan![i - 1].x, q.y - plan![i - 1].y) : 0), 0) : 0
    partial = route?.partial ?? false
    const msg = plan ? `path: ${len.toFixed(2)} m${partial ? ' (goal not reachable yet: going as close as possible)' : ''}` : 'no path'
    if (msg !== lastPlanMsg || !plan) log(msg)
    lastPlanMsg = msg
    // keeps its latency state (last command) across replans
    if (route) follower.setPath(route.path, route.bodyClear)
    setLine(planGeo, plan ? plan.map(toScreen) : [])
    return plan !== null
  }

  /** Called after each lidar turn: one step of the path follower, sent to the robot. */
  function navigateStep() {
    lastNavTurn = performance.now()
    if (!goal || !plan) return
    if (slam.lastFit < 0.5) {
      // lost: better stop than drive blind
      setNav('stopped: lost on the map')
      void bridge.stop()
      return
    }
    // replan every second or so: the map grows and the robot drifts off the line
    if (slam.scans % 5 === 0 && !replan()) return clearGoal('No path to the goal anymore')
    // something right in front (an obstacle that isn't in the map yet): stop and let the replan,
    // which uses the map updated with it, find a way around
    // (turning in place is still allowed)
    const cmd = follower.step(center(slam.pose))
    const latest = turns[turns.length - 1]
    // in the body's frame: up to 12 cm beyond its front edge, as wide as the body
    // the faster it goes, the farther it looks: what it travels during the loop's latency, twice
    const reach = DEFAULT_NAV.robotRadius + 0.12 + 2 * DEFAULT_NAV.vMax * DEFAULT_NAV.latency
    const ahead = (q: Vec2) => q.x + LIDAR_AHEAD > 0 && q.x + LIDAR_AHEAD < reach && Math.abs(q.y) < DEFAULT_NAV.robotRadius + 0.02
    if (cmd.v > 0 && latest?.points.some(ahead)) {
      // no forward motion, but keep turning toward the path: turning is what clears the front
      const w = Math.sign(cmd.w || 1) * Math.max(0.3, Math.abs(cmd.w))
      if (drive.armed) void bridge.drive(0, w * drive.wScale)
      setNav('blocked ahead: turning toward the path')
      // standing still, the map isn't updated: add the obstacle now, then replan around it
      if (slam.scans % 2 === 0) {
        slam.insertNow(latest.points)
        mapDirty = true
        if (!replan()) clearGoal('Blocked: no way around')
      }
      return
    }
    if (cmd.done) {
      // at the end of a partial path, one more replan (the map has grown): a way on, or give up
      if (partial) {
        slam.insertNow(latest!.points)
        if (replan() && !partial) return
        return clearGoal("Can't reach the goal: this is as close as it gets")
      }
      return clearGoal('Arrived')
    }
    // stuck (snagged on something the map doesn't show): asked to drive, but not moving. Back up
    // ~10 cm, add the scan to the map and replan, like the escape of the exploration mode
    const now = performance.now()
    const c = center(slam.pose)
    if (now < backUpUntil) {
      setNav('stuck: backing up')
      if (drive.armed) void bridge.drive(-0.08 * drive.vScale, 0)
      return
    }
    if (cmd.v > 0.03) {
      if (!stuckSince) stuckSince = { at: now, x: c.x, y: c.y }
      else if (Math.hypot(c.x - stuckSince.x, c.y - stuckSince.y) > 0.03) stuckSince = { at: now, x: c.x, y: c.y }
      else if (now - stuckSince.at > 2000) {
        stuckSince = null
        backUpUntil = now + 1300
        log(`stuck at (${c.x.toFixed(2)}, ${c.y.toFixed(2)}): backing up`)
        slam.insertNow(latest!.points)
        mapDirty = true
        setTimeout(() => goal && !replan() && clearGoal('Stuck: no way out found'), 1400)
        return
      }
    } else stuckSince = null
    navState = cmd.v > 0.01 ? `driving ${(cmd.v * 100).toFixed(0)} cm/s` : `turning ${((cmd.w * 180) / Math.PI).toFixed(0)}°/s`
    // the log gets the kind of motion, not every speed change
    logOnce(cmd.v > 0.01 ? 'driving' : cmd.w > 0 ? 'turning left in place' : 'turning right in place')
    if (drive.armed) void bridge.drive(cmd.v * drive.vScale, cmd.w * drive.wScale)
  }

  // a click on the map (not a drag) sets the goal
  const raycaster = new THREE.Raycaster()
  const ground = new THREE.Plane(new THREE.Vector3(0, 1, 0), 0)
  let down: { x: number; y: number } | null = null
  renderer.domElement.addEventListener('pointerdown', (e) => (down = { x: e.clientX, y: e.clientY }))
  renderer.domElement.addEventListener('pointerup', (e) => {
    if (!down || Math.hypot(e.clientX - down.x, e.clientY - down.y) > 5 || e.button !== 0) return
    const r = renderer.domElement.getBoundingClientRect()
    raycaster.setFromCamera(new THREE.Vector2(((e.clientX - r.left) / r.width) * 2 - 1, -((e.clientY - r.top) / r.height) * 2 + 1), rig.camera)
    const hit = new THREE.Vector3()
    if (!raycaster.ray.intersectPlane(ground, hit)) return
    // into the (mirrored) world group's frame
    const local = world.worldToLocal(hit)
    goal = { x: -local.z, y: local.x }
    goalMarker.position.set(local.x, 0.012, local.z)
    goalMarker.visible = true
    log(`goal (${goal.x.toFixed(2)}, ${goal.y.toFixed(2)})`)
    if (!view.mapping || !slam.trajectory.length) return clearGoal('Build a map first (the robot must see the room)')
    if (!replan()) return clearGoal('No path there (a wall, or too close to one)')
    hud.message(drive.armed ? 'Going there' : 'Path planned: arm driving to go')
  })

  // keyboard: arrows / WASD drive by hand, space stops
  const keys = new Set<string>()
  const manualTick = () => {
    const v = (keys.has('ArrowUp') || keys.has('w') ? 1 : 0) - (keys.has('ArrowDown') || keys.has('s') ? 1 : 0)
    const w = (keys.has('ArrowLeft') || keys.has('a') ? 1 : 0) - (keys.has('ArrowRight') || keys.has('d') ? 1 : 0)
    drive.manual = { v, w }
  }
  window.addEventListener('keydown', (e) => {
    if ((e.target as HTMLElement).tagName === 'INPUT') return
    if (e.key === ' ') {
      e.preventDefault()
      clearGoal('Stopped')
      return
    }
    keys.add(e.key.length === 1 ? e.key.toLowerCase() : e.key)
    manualTick()
  })
  window.addEventListener('keyup', (e) => {
    keys.delete(e.key.length === 1 ? e.key.toLowerCase() : e.key)
    manualTick()
  })
  setInterval(() => {
    const { v, w } = drive.manual
    if (drive.armed && (v || w)) {
      if (goal) clearGoal()
      // by hand: the same limits as the navigation
      void bridge.drive(v * DEFAULT_NAV.vMax * drive.vScale, w * DEFAULT_NAV.wMax * drive.wScale)
      return
    }
    // the firmware only spins the lidar while it gets remote-control steps: zero-speed steps keep
    // it on (standing still) whenever the follower isn't sending a moving command, and also stop
    // the robot when lidar turns stop arriving (no map updates, no driving)
    const following = goal && performance.now() - lastNavTurn < 700
    if ((drive.armed || drive.lidarOn) && !(following && performance.now() - bridge.lastMoveAt < 400)) void bridge.drive(0, 0)
  }, 150)

  clearGoalIfAny = () => {
    if (goal) clearGoal()
  }
  // ---- camera: YOLO detections (scripts/yolo_cam.py) placed on the map with the lidar ----
  const camera = {
    // the notebook runs YOLO: from a phone, the same host as this page
    url: `http://${location.hostname}:8001`,
    // the raw video comes straight from the Pi (smooth); YOLO's boxes are drawn over it
    piUrl: 'http://picam.local:8000',
    show: false,
    videoMode: 'raw' as 'raw' | 'yolo',
    light: 'auto' as 'auto' | 'day' | 'night',
    mapObjects: true,
    yaw: 0,
    minConf: 0.5,
  }
  const cam = { ...PI_CAMERA_V2 }
  const objects = new ObjectMap()
  const markers = new Map<number, THREE.Group>()
  const people = new THREE.Group()
  world.add(people)
  const camBox = Object.assign(document.createElement('div'), { id: 'camview', hidden: true })
  const video = Object.assign(document.createElement('img'), { alt: 'camera' })
  const overlay = document.createElement('canvas')
  const camInfo = Object.assign(document.createElement('div'), { className: 'caminfo' })
  camBox.append(video, overlay, camInfo)
  document.getElementById('hud')!.appendChild(camBox)
  let shownDets: Detection[] = []
  /** YOLO's boxes over the raw video, scaled from the detection frame to the shown size */
  const drawBoxes = () => {
    const w = video.clientWidth
    const h = video.clientHeight
    if (overlay.width !== w || overlay.height !== h) Object.assign(overlay, { width: w, height: h })
    const ctx = overlay.getContext('2d')!
    ctx.clearRect(0, 0, w, h)
    if (camera.videoMode !== 'raw') return
    const k = w / (cam.width || 640)
    ctx.font = '600 12px system-ui, sans-serif'
    ctx.lineWidth = 2
    for (const d of shownDets) {
      const [x1, y1, x2, y2] = d.box.map((v) => v * k)
      const color = `#${(OBJECT_COLORS[d.label] ?? 0x39ff88).toString(16).padStart(6, '0')}`
      ctx.strokeStyle = color
      ctx.strokeRect(x1, y1, x2 - x1, y2 - y1)
      const text = `${d.label} ${d.conf.toFixed(2)}`
      ctx.fillStyle = color
      ctx.fillRect(x1, y1 - 16, ctx.measureText(text).width + 8, 16)
      ctx.fillStyle = '#0b0f16'
      ctx.fillText(text, x1 + 4, y1 - 4)
    }
  }
  const setVideoSource = () => {
    video.src = !camera.show ? '' : camera.videoMode === 'raw' ? `${camera.piUrl}/stream.mjpg` : `${camera.url}/stream.mjpg`
    drawBoxes()
  }
  clearObjectsIfAny = () => {
    objects.objects = []
    for (const m of markers.values()) world.remove(m)
    markers.clear()
  }
  let camFail = 0
  let mapTick = 0
  let polling = false
  // detections ~10 times a second (smooth boxes); the map is updated with every other batch
  setInterval(async () => {
    if ((!camera.show && !camera.mapObjects) || polling) return
    polling = true
    let dets: Detection[]
    try {
      const r = await fetch(`${camera.url}/detections`)
      const j = await r.json()
      dets = j.detections
      cam.width = j.size?.[0] || cam.width
      camInfo.textContent = `YOLO ${j.fps} fps · ${j.ms} ms · boxes ${Math.round((j.age ?? 0) * 1000)} ms old${piLight ? ` · ${piLight}` : ''}`
      shownDets = dets
      drawBoxes()
      camFail = 0
    } catch {
      if (camFail++ === 3) log(`camera: no answer from ${camera.url} (is scripts/yolo_cam.py running?)`)
      return
    } finally {
      polling = false
    }
    if (mapTick++ % 2) return
    const latest = turns[turns.length - 1]
    if (!camera.mapObjects || !view.mapping || !latest || slam.lastFit < 0.6) return
    cam.yaw = (camera.yaw * Math.PI) / 180
    people.clear()
    movingNow = movingNow.filter((z) => z.until > performance.now())
    for (const d of dets) {
      if (d.conf < camera.minConf) continue
      const at = locate(d, latest.points, cam)
      if (!at) continue
      const p = toMap(at, slam.pose)
      // people and animals move: shown where they are now, and their lidar echoes kept out of
      // the map for a moment (a dog standing still would become a wall)
      if (MOVING.has(d.label)) {
        people.add(objectMarker(d.label, p))
        movingNow.push({ x: p.x, y: p.y, until: performance.now() + 1500 })
        continue
      }
      // only furniture stays on the map (other classes are often wrong guesses: a "cow" rug)
      if (!FURNITURE.has(d.label)) continue
      const { object, confirmedNow } = objects.add(d.label, p, performance.now())
      if (confirmedNow) log(`new object: ${object.label} at (${object.x.toFixed(2)}, ${object.y.toFixed(2)})`)
      if (object.sightings >= objects.confirmAfter) {
        let m = markers.get(object.id)
        if (!m) {
          m = objectMarker(object.label, object)
          markers.set(object.id, m)
          world.add(m)
        }
        m.position.set(object.y, 0, -object.x)
      }
    }
  }, 100)
  const camF = gui.addFolder('Camera (YOLO)')
  camF.add(camera, 'url').name('YOLO server')
  camF
    .add(camera, 'show')
    .name('Show the video')
    .onChange((on: boolean) => {
      camBox.hidden = !on
      setVideoSource()
    })
  camF
    .add(camera, 'videoMode', { 'Raw video + boxes (smooth)': 'raw', 'YOLO annotated video': 'yolo' })
    .name('Video')
    .onChange(setVideoSource)
  camF.add(camera, 'piUrl').name('Camera (Pi)').onFinishChange(setVideoSource)
  // night mode on the Pi: longer exposure, higher gain, 10 fps, black and white (needs IR light in the dark)
  let piLight = ''
  const lightMode = (set?: string) =>
    fetch(`${camera.piUrl}/mode${set ? `?set=${set}` : ''}`)
      .then((r) => r.json())
      .then((j) => {
        const now = `${j.night ? 'night' : 'day'} (brightness ${j.brightness})`
        if (j.night !== piLight.startsWith('night')) log(`camera: ${j.night ? 'night' : 'day'} mode`)
        piLight = now
      })
      .catch(() => {})
  camF
    .add(camera, 'light', { 'Auto (by brightness)': 'auto', Day: 'day', 'Night (IR)': 'night' })
    .name('Light')
    .onChange((m: string) => lightMode(m))
  setInterval(() => camera.show && lightMode(), 3000)
  camF.add(camera, 'mapObjects').name('Put objects on the map')
  camF.add(camera, 'yaw', -90, 90, 1).name('Camera yaw (°, + left)')
  camF.add(cam, 'x', -0.4, 0.3, 0.005).name('Camera ahead of lidar (m)')
  camF.add(camera, 'minConf', 0.2, 0.9, 0.05).name('Min confidence')
  camF.add({ clear: () => clearObjectsIfAny() }, 'clear').name('Clear the objects')

  const driveF = gui.addFolder('Drive the robot')
  driveF.add(drive, 'bridge').name('Bridge address').onFinishChange((u: string) => (bridge.url = u.trim()))
  driveF
    .add(drive, 'armed')
    .name('Armed (robot moves)')
    .onChange((on: boolean) => {
      if (!on) {
        void bridge.stop()
        void bridge.quiet(false)
        return
      }
      // the firmware runs the suction during remote control: lowest power while armed
      void bridge.quiet(true)
      bridge.status().then(
        (s) => hud.message(`Robot connected · battery ${s.battery}%`),
        () => hud.message('No answer from the bridge: run node scripts/robot-bridge.mjs'),
      )
    })
  driveF.add(drive, 'lidarOn').name('Keep the lidar on (robot still)')
  // the robot's front, measured: drive straight ahead and see which way the map says it went
  driveF
    .add(
      {
        calibrate: async () => {
          if (!drive.armed) return hud.message('Arm driving first (the robot drives ~15 cm forward)')
          if (!slam.trajectory.length || slam.lastFit < 0.6) return hud.message('Wait for the map first')
          clearGoal()
          const p0 = { ...slam.pose }
          hud.message('Calibrating: driving forward…')
          drive.manual = { v: 0.75, w: 0 }
          await new Promise((r) => setTimeout(r, 2500))
          drive.manual = { v: 0, w: 0 }
          await new Promise((r) => setTimeout(r, 1200))
          const dx = slam.pose.x - p0.x
          const dy = slam.pose.y - p0.y
          if (Math.hypot(dx, dy) < 0.05) return hud.message('It barely moved: try again with free space in front')
          // the displacement in the robot frame at the start: its angle is how far off the front is
          const fx = Math.cos(p0.theta) * dx + Math.sin(p0.theta) * dy
          const fy = -Math.sin(p0.theta) * dx + Math.cos(p0.theta) * dy
          const off = (Math.atan2(fy, fx) * 180) / Math.PI
          let r = Math.round(view.rotation - off)
          r = ((((r + 180) % 360) + 360) % 360) - 180
          rotCtrl.setValue(r)
          hud.message(`Front calibrated: the lidar was ${Math.round(off)}° off, rotation set to ${r}°`)
        },
      },
      'calibrate',
    )
    .name('Calibrate front (drives 15 cm)')
  driveF.add({ stop: () => clearGoal('Stopped') }, 'stop').name('STOP (space)')
  // speeds in friendly units, stored in m/s and rad/s, remembered in the browser
  const speeds = { maxSpeed: DEFAULT_NAV.vMax * 100, maxTurn: (DEFAULT_NAV.wMax * 180) / Math.PI }
  try {
    const saved = JSON.parse(localStorage.getItem('lidar.speeds') ?? 'null')
    if (saved) Object.assign(speeds, saved)
  } catch {
    /* storage blocked */
  }
  const applySpeeds = () => {
    DEFAULT_NAV.vMax = speeds.maxSpeed / 100
    DEFAULT_NAV.wMax = (speeds.maxTurn * Math.PI) / 180
    try {
      localStorage.setItem('lidar.speeds', JSON.stringify(speeds))
    } catch {
      /* storage blocked */
    }
  }
  applySpeeds()
  driveF.add(speeds, 'maxSpeed', 4, 40, 1).name('Max speed (cm/s)').onChange(applySpeeds)
  driveF.add(speeds, 'maxTurn', 15, 120, 1).name('Max turn (°/s)').onChange(applySpeeds)
  driveF.add(DEFAULT_NAV, 'inflate', 0.18, 0.4, 0.01).name('Wall clearance (m)')
  driveF.add(DEFAULT_NAV, 'lookahead', 0.2, 0.8, 0.05).name('Look-ahead (m)')
  driveF.add(DEFAULT_NAV, 'latency', 0, 0.8, 0.05).name('Latency (s)')
  driveF.add(drive, 'vScale', 200, 1500, 10).name('Speed units per m/s')
  driveF.add(drive, 'wScale', 10, 150, 1).name('Turn units per rad/s')

  window.addEventListener('resize', () => {
    renderer.setSize(container.clientWidth, container.clientHeight)
    rig.resize(container.clientWidth, container.clientHeight)
  })

  /** The map as a PNG, walls white on dark, with the robot's path. */
  function saveMapImage() {
    if (mapDirty || slam.trajectory.length) uploadMap()
    const canvas = Object.assign(document.createElement('canvas'), { width: mapSize, height: mapSize })
    const ctx = canvas.getContext('2d')!
    ctx.fillStyle = '#0b0f16'
    ctx.fillRect(0, 0, mapSize, mapSize)
    const img = ctx.createImageData(mapSize, mapSize)
    // canvas rows go down the screen (+z), texture rows go up: flip
    for (let r = 0; r < mapSize; r++) img.data.set(mapPixels.subarray(r * mapSize * 4, (r + 1) * mapSize * 4), (mapSize - 1 - r) * mapSize * 4)
    const layer = Object.assign(document.createElement('canvas'), { width: mapSize, height: mapSize })
    layer.getContext('2d')!.putImageData(img, 0, 0)
    // mirrored like the screen view (the map's y axis points left)
    ctx.translate(mapSize, 0)
    ctx.scale(-1, 1)
    ctx.drawImage(layer, 0, 0)
    const g = slam.maps[0]
    ctx.strokeStyle = '#ff2e88'
    ctx.lineWidth = 2
    ctx.beginPath()
    slam.trajectory.forEach((p, i) => {
      const cx = (p.y - g.origin) / g.resolution
      // same flip as the map rows above
      const cy = mapSize - 1 - (p.x - g.origin) / g.resolution
      if (i) ctx.lineTo(cx, cy)
      else ctx.moveTo(cx, cy)
    })
    ctx.stroke()
    canvas.toBlob((blob) => {
      if (!blob) return
      const a = Object.assign(document.createElement('a'), { href: URL.createObjectURL(blob), download: `map-${new Date().toISOString().replace(/[:.]/g, '-')}.png` })
      a.click()
      setTimeout(() => URL.revokeObjectURL(a.href), 1000)
    })
  }

  // where the robot was on the screen last frame (world coordinates, starting at the view center)
  world.updateMatrixWorld(true)
  const followed = robot.getWorldPosition(new THREE.Vector3())
  let last = performance.now()
  renderer.setAnimationLoop((now) => {
    const dt = (now - last) / 1000
    last = now
    if (now - rate.since > 1000) {
      rate.hz = (rate.turns * 1000) / (now - rate.since)
      rate.turns = 0
      rate.since = now
    }
    const t = turns[turns.length - 1]?.rev
    if (!source) hud.setStatus('No source · connect the adapter or play the sample capture')
    else if (!t) hud.setStatus(`${source.label} · waiting for data… (${parser.skipped} bytes skipped)`)
    else {
      const valid = t.samples.filter((s) => !Number.isNaN(s.distance)).length
      hud.setStatus(
        `${source.label} · ${t.rpm.toFixed(0)} rpm · ${rate.hz.toFixed(1)} turns/s · ${t.samples.length} points/turn · ${Math.round((100 * valid) / t.samples.length)}% echoes · ${parser.packets} packets${view.mapping ? ` · map: fit ${Math.round(slam.lastFit * 100)}%, ${slam.trajectory.length} poses` : ''}${goal ? ` · goal ${Math.hypot(goal.x - slam.pose.x, goal.y - slam.pose.y).toFixed(2)} m${drive.armed ? ` · ${navState}` : ' (not armed)'}` : ''}${recording ? ' · ● REC' : ''}`,
      )
    }
    if (mapDirty && now - lastMapUpload > 400) {
      uploadMap()
      mapDirty = false
      lastMapUpload = now
    }
    // follow: shift the camera by however much the robot moved, keeping the user's zoom and pan
    const at = robot.getWorldPosition(new THREE.Vector3())
    if (view.follow) {
      const d = at.clone().sub(followed)
      rig.camera.position.add(d)
      rig.controls.target.add(d)
    }
    followed.copy(at)
    rig.update({ x: CENTER.x, y: CENTER.z, theta: 0 }, dt)
    renderer.render(scene, rig.camera)
  })
}

const CENTER2D = () => ({ x: CENTER.x, y: CENTER.z })

/** Replaces a line's points (BufferGeometry.setFromPoints never grows an existing attribute). */
function setLine(geo: THREE.BufferGeometry, points: THREE.Vector3[]) {
  const a = new Float32Array(points.length * 3)
  points.forEach((p, i) => a.set([p.x, p.y, p.z], i * 3))
  geo.setAttribute('position', new THREE.BufferAttribute(a, 3))
  geo.computeBoundingSphere()
}

/** Rings every 0.5 m (brighter every meter) and the axes. */
function rangeRings(): THREE.Group {
  const g = new THREE.Group()
  for (let r = 0.5; r <= 8; r += 0.5) {
    const pts: THREE.Vector3[] = []
    for (let k = 0; k <= 128; k++) pts.push(new THREE.Vector3(Math.cos((k / 128) * Math.PI * 2) * r, 0.005, Math.sin((k / 128) * Math.PI * 2) * r))
    const major = Number.isInteger(r)
    g.add(new THREE.Line(new THREE.BufferGeometry().setFromPoints(pts), new THREE.LineBasicMaterial({ color: major ? 0x3a4a63 : 0x222c3b })))
  }
  const axes = new THREE.BufferGeometry().setFromPoints([
    new THREE.Vector3(-8, 0.005, 0),
    new THREE.Vector3(8, 0.005, 0),
    new THREE.Vector3(0, 0.005, -8),
    new THREE.Vector3(0, 0.005, 8),
  ])
  g.add(new THREE.LineSegments(axes, new THREE.LineBasicMaterial({ color: 0x222c3b })))
  return g
}

/** The robot as a disc with a notch toward its front (angle 0 of the lidar, until calibrated). */
function robotBody(): THREE.Group {
  const g = new THREE.Group()
  const body = new THREE.Mesh(
    new THREE.CylinderGeometry(ROBOT_RADIUS, ROBOT_RADIUS, 0.09, 48),
    new THREE.MeshStandardMaterial({ color: 0xe9edf2, roughness: 0.6, transparent: true, opacity: 0.35 }),
  )
  // the group sits at the lidar; the body's center is behind it (+z is backward)
  body.position.set(0, 0.045, LIDAR_AHEAD)
  const tower = new THREE.Mesh(new THREE.CylinderGeometry(0.04, 0.04, 0.03, 24), new THREE.MeshStandardMaterial({ color: 0x39ff88 }))
  tower.position.y = 0.105
  const front = new THREE.Mesh(new THREE.ConeGeometry(0.03, 0.07, 3), new THREE.MeshStandardMaterial({ color: 0xff2e88 }))
  front.rotation.x = -Math.PI / 2
  front.position.set(0, 0.1, LIDAR_AHEAD - ROBOT_RADIUS + 0.03)
  g.add(body, tower, front)
  return g
}

/**
 * A drawer on the left with what the navigation did, time-stamped: goals, paths, motion kinds,
 * blocks, recoveries, toasts. "Copy" puts it on the clipboard (to paste in a bug report).
 */
function createLogDrawer() {
  const lines: string[] = []
  const box = document.createElement('div')
  box.id = 'navlog'
  box.innerHTML = '<div class="bar"><button type="button" class="toggle">Log ▸</button><button type="button" class="copy">Copy</button><button type="button" class="clear">Clear</button></div><pre></pre>'
  document.getElementById('hud')!.appendChild(box)
  const pre = box.querySelector('pre')!
  const toggle = box.querySelector<HTMLButtonElement>('.toggle')!
  const open = () => box.classList.contains('open')
  toggle.onclick = () => {
    box.classList.toggle('open')
    toggle.textContent = open() ? 'Log ◂' : 'Log ▸'
    pre.scrollTop = pre.scrollHeight
  }
  box.querySelector<HTMLButtonElement>('.copy')!.onclick = () => {
    navigator.clipboard.writeText(lines.join('\n')).catch(() => {
      // no clipboard: select the text so it can be copied by hand
      const r = document.createRange()
      r.selectNodeContents(pre)
      getSelection()?.removeAllRanges()
      getSelection()?.addRange(r)
    })
  }
  box.querySelector<HTMLButtonElement>('.clear')!.onclick = () => {
    lines.length = 0
    pre.textContent = ''
  }
  let last = ''
  const log = (msg: string) => {
    const t = new Date()
    const stamp = `${t.toLocaleTimeString('en-GB')}.${String(Math.floor(t.getMilliseconds() / 100))}`
    lines.push(`${stamp}  ${msg}`)
    if (lines.length > 500) lines.shift()
    const atBottom = pre.scrollTop + pre.clientHeight >= pre.scrollHeight - 4
    pre.textContent = lines.join('\n')
    if (atBottom) pre.scrollTop = pre.scrollHeight
    last = msg
  }
  /** logs only when the message differs from the last one (states, not every step) */
  const logOnce = (msg: string) => {
    if (msg !== last) log(msg)
  }
  return { log, logOnce }
}

const OBJECT_COLORS: Record<string, number> = { person: 0xff2e88, dog: 0xf4a261, cat: 0xf4a261, chair: 0xffb703, couch: 0x4cc9f0, bed: 0x9b5de5, 'dining table': 0x06d6a0, tv: 0x8ecae6 }

/**
 * A map marker: a short colored post and the label floating above it. The world group is mirrored on
 * the screen's x axis, so the label sprite is mirrored back to read normally.
 */
function objectMarker(label: string, at: { x: number; y: number }): THREE.Group {
  const g = new THREE.Group()
  const color = OBJECT_COLORS[label] ?? 0xe9edf2
  const post = new THREE.Mesh(new THREE.CylinderGeometry(0.06, 0.06, 0.3, 16), new THREE.MeshStandardMaterial({ color, emissive: color, emissiveIntensity: 0.35 }))
  post.position.y = 0.15
  const canvas = document.createElement('canvas')
  canvas.width = 256
  canvas.height = 64
  const ctx = canvas.getContext('2d')!
  ctx.fillStyle = 'rgba(11,15,22,0.85)'
  ctx.fillRect(0, 0, 256, 64)
  ctx.fillStyle = `#${color.toString(16).padStart(6, '0')}`
  ctx.font = 'bold 34px system-ui, sans-serif'
  ctx.textAlign = 'center'
  ctx.textBaseline = 'middle'
  ctx.fillText(label, 128, 34)
  const sprite = new THREE.Sprite(new THREE.SpriteMaterial({ map: new THREE.CanvasTexture(canvas), depthTest: false }))
  sprite.scale.set(-0.5, 0.125, 1)
  sprite.position.y = 0.45
  sprite.renderOrder = 20
  g.add(post, sprite)
  g.position.set(at.y, 0, -at.x)
  return g
}
