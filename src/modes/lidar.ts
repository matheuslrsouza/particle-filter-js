import GUI from 'lil-gui'
import * as THREE from 'three'
import { CamsenseParser, RevolutionAssembler, type LidarSample, type Revolution } from '../core/lidar/camsense'
import type { Pose, Vec2 } from '../core/math/vec'
import { HectorSlam, scanToPoints } from '../core/slam/hector'
import sampleUrl from '../real/sample-capture.bin?url'
import { hasWebSerial, ReplaySource, SerialSource, WebSocketSource, type ByteSource } from '../real/sources'
import { CameraRig } from '../render/cameras'
import { createRenderer, createScene } from '../render/scene'
import { Hud } from '../ui/hud'
import { addModeSelector } from '../ui/mode'

/** the view is a 10 m square with the lidar in the middle */
const AREA = { width: 10, height: 10 }
const CENTER = new THREE.Vector3(AREA.width / 2, 0, AREA.height / 2)
const MAX_TURNS = 30
const MAX_POINTS = 440
/** Xiaomi S10+ body: 35 cm across */
const ROBOT_RADIUS = 0.175

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
  document.getElementById('charts')!.hidden = true

  const world = new THREE.Group()
  world.position.copy(CENTER)
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
    mirror: false,
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

  const usable = (s: LidarSample) => !Number.isNaN(s.distance) && s.distance >= 0.12 && s.distance <= view.maxRange && s.quality >= view.minQuality

  /** One turn → points in the robot frame; with mapping on, also matched and added to the map. */
  function process(rev: Revolution): Turn {
    const samples = rev.samples.filter(usable)
    const opts = { mirror: view.mirror, rotation: view.rotation, minRange: 0, maxRange: Infinity, minQuality: 0 }
    if (!view.mapping) return { rev, samples, points: scanToPoints(samples, opts), pose: { x: 0, y: 0, theta: 0 } }
    const points = scanToPoints(samples, { ...opts, motion: view.deskew ? slam.motion : undefined })
    const before = slam.trajectory.length
    const pose = { ...slam.addScan(points) }
    if (slam.trajectory.length !== before) mapDirty = true
    return { rev, samples, points, pose }
  }

  function resetMap() {
    slam = new HectorSlam()
    turns = []
    mapPixels.fill(0)
    mapTexture.needsUpdate = true
    pathGeo.setFromPoints([])
    redraw()
  }

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
    pathGeo.setFromPoints(slam.trajectory.map((p) => new THREE.Vector3(p.y, 0, -p.x)))
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
  const cal = gui.addFolder('Mounting')
  cal.add(view, 'rotation', -180, 180, 1).name('Rotation (°)').onChange(resetMap)
  cal.add(view, 'mirror').name('Mirror (turns the other way)').onChange(resetMap)
  cal.close()
  material.size = view.pointSize / 100

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

  const followed = new THREE.Vector3()
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
        `${source.label} · ${t.rpm.toFixed(0)} rpm · ${rate.hz.toFixed(1)} turns/s · ${t.samples.length} points/turn · ${Math.round((100 * valid) / t.samples.length)}% echoes · ${parser.packets} packets${view.mapping ? ` · map: fit ${Math.round(slam.lastFit * 100)}%, ${slam.trajectory.length} poses` : ''}${recording ? ' · ● REC' : ''}`,
      )
    }
    if (mapDirty && now - lastMapUpload > 400) {
      uploadMap()
      mapDirty = false
      lastMapUpload = now
    }
    // follow: shift the camera by however much the robot moved, keeping the user's zoom and pan
    const at = robot.position
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
  body.position.y = 0.045
  const tower = new THREE.Mesh(new THREE.CylinderGeometry(0.04, 0.04, 0.03, 24), new THREE.MeshStandardMaterial({ color: 0x39ff88 }))
  tower.position.y = 0.105
  const front = new THREE.Mesh(new THREE.ConeGeometry(0.03, 0.07, 3), new THREE.MeshStandardMaterial({ color: 0xff2e88 }))
  front.rotation.x = -Math.PI / 2
  front.position.set(0, 0.1, -ROBOT_RADIUS + 0.03)
  g.add(body, tower, front)
  return g
}
