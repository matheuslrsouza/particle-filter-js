// Bridge between the browser and the robot vacuum's own firmware, over the local miIO protocol
// (UDP 54321, AES-128-CBC with the device token). The browser can't speak UDP, so it calls this
// over HTTP:
//   GET /drive?v=<int>&w=<int>   remote-control step: v forward (60 ≈ 8 cm/s), w turn (+ = counter-clockwise)
//   GET /stop                    stop now
//   GET /suction?quiet=1|0       lowest suction while driving / restore the previous level
//   GET /status                  battery, status and charging state
// While a drive command is fresh, it is repeated every 200 ms (the robot expects a stream of steps);
// if the browser goes quiet for 600 ms the robot is stopped (watchdog). Zero-speed steps are sent
// too: the firmware only spins the lidar while it gets remote-control steps (it starts ~1.5 s after
// the first one and stops ~3 s after the last), so /drive?v=0&w=0 keeps the lidar on, standing still.
//
// Config in ~/.robo-xiaomi (not in the repo): IP=…, TOKEN=… (from Xiaomi-cloud-tokens-extractor).
//   node scripts/robot-bridge.mjs     → http://localhost:8090
import { createCipheriv, createDecipheriv, createHash, randomInt } from 'node:crypto'
import { createSocket } from 'node:dgram'
import { readFileSync } from 'node:fs'
import { createServer } from 'node:http'
import { homedir } from 'node:os'

const cfg = Object.fromEntries(
  readFileSync(`${homedir()}/.robo-xiaomi`, 'utf8')
    .split('\n')
    .filter((l) => l.includes('='))
    .map((l) => l.trim().split(/=(.*)/s).slice(0, 2)),
)
const token = Buffer.from(cfg.TOKEN, 'hex')
const md5 = (...b) => createHash('md5').update(Buffer.concat(b)).digest()
const key = md5(token)
const iv = md5(key, token)

const sock = createSocket('udp4')
let deviceId = 0
let stampOffset = 0 // device stamp − our clock (s)
let nextId = randomInt(1000, 9000)
const pending = new Map()

sock.on('message', (msg) => {
  if (msg.length < 32 || msg.readUInt16BE(0) !== 0x2131) return
  deviceId = msg.readUInt32BE(8)
  stampOffset = msg.readUInt32BE(12) - Math.floor(Date.now() / 1000)
  if (msg.length === 32) return pending.get('hello')?.(null)
  const d = createDecipheriv('aes-128-cbc', key, iv)
  const json = Buffer.concat([d.update(msg.subarray(32)), d.final()]).toString('utf8').replace(/\0+$/, '')
  const reply = JSON.parse(json)
  pending.get(reply.id)?.(reply)
})

function request(id, packet, ms = 2000) {
  return new Promise((resolve, reject) => {
    const t = setTimeout(() => {
      pending.delete(id)
      reject(new Error('no reply from the robot'))
    }, ms)
    pending.set(id, (r) => {
      clearTimeout(t)
      pending.delete(id)
      resolve(r)
    })
    sock.send(packet, 54321, cfg.IP)
  })
}

async function hello() {
  const p = Buffer.alloc(32, 0xff)
  p.writeUInt16BE(0x2131, 0)
  p.writeUInt16BE(32, 2)
  await request('hello', p)
}

async function call(method, params) {
  if (!deviceId) await hello()
  const id = nextId++
  const c = createCipheriv('aes-128-cbc', key, iv)
  const body = Buffer.concat([c.update(JSON.stringify({ id, method, params }), 'utf8'), c.final()])
  const h = Buffer.alloc(32)
  h.writeUInt16BE(0x2131, 0)
  h.writeUInt16BE(32 + body.length, 2)
  h.writeUInt32BE(0, 4)
  h.writeUInt32BE(deviceId, 8)
  h.writeUInt32BE(Math.floor(Date.now() / 1000) + stampOffset + 1, 12)
  md5(h.subarray(0, 16), token, body).copy(h, 16)
  try {
    return await request(id, Buffer.concat([h, body]))
  } catch (e) {
    deviceId = 0 // handshake again next time
    throw e
  }
}

const step = (v, w) =>
  call('set_properties', [
    { did: 'remote', siid: 4, piid: 15, value: JSON.stringify({ spdv: v, spdw: w, audio: 'false', random: randomInt(65535) }) },
  ])

// the command stream and the watchdog
const clamp = (x, a) => Math.max(-a, Math.min(a, Math.round(Number(x) || 0)))
let cmd = { v: 0, w: 0, at: 0 }
let moving = false
let savedSuction = null
setInterval(async () => {
  const fresh = Date.now() - cmd.at < 600
  try {
    if (fresh) {
      await step(cmd.v, cmd.w)
      moving = true
    } else if (moving) {
      await step(0, 0)
      moving = false
    }
  } catch (e) {
    console.log('robot:', e.message)
  }
}, 200)

createServer(async (req, res) => {
  const url = new URL(req.url, 'http://x')
  res.setHeader('Access-Control-Allow-Origin', '*')
  res.setHeader('Content-Type', 'application/json')
  try {
    if (url.pathname === '/drive') {
      // limits: about 15 cm/s and a slow turn, until the controller is trusted
      cmd = { v: clamp(url.searchParams.get('v'), 120), w: clamp(url.searchParams.get('w'), 80), at: Date.now() }
      res.end(JSON.stringify({ ok: true, ...cmd }))
    } else if (url.pathname === '/stop') {
      cmd = { v: 0, w: 0, at: 0 }
      await step(0, 0)
      moving = false
      res.end(JSON.stringify({ ok: true }))
    } else if (url.pathname === '/suction') {
      // the firmware runs the suction during remote control; quiet=1 sets the lowest power
      // (cleaning-mode 0, siid 4 piid 4) and remembers the user's level, quiet=0 restores it
      const quiet = url.searchParams.get('quiet') === '1'
      if (quiet && savedSuction === null) {
        const r = await call('get_properties', [{ did: 'suction', siid: 4, piid: 4 }])
        savedSuction = r.result[0].value
      }
      const level = quiet ? 0 : savedSuction
      if (level !== null) await call('set_properties', [{ did: 'suction', siid: 4, piid: 4, value: level }])
      if (!quiet) savedSuction = null
      res.end(JSON.stringify({ ok: true, level }))
    } else if (url.pathname === '/status') {
      const r = await call('get_properties', [
        { did: 'battery', siid: 3, piid: 1 },
        { did: 'status', siid: 2, piid: 1 },
        { did: 'charging', siid: 3, piid: 2 },
      ])
      res.end(JSON.stringify({ ok: true, ...Object.fromEntries(r.result.map((p) => [p.did, p.value])) }))
    } else {
      res.statusCode = 404
      res.end(JSON.stringify({ ok: false, error: 'unknown path' }))
    }
  } catch (e) {
    res.statusCode = 502
    res.end(JSON.stringify({ ok: false, error: e.message }))
  }
}).listen(8090, () => console.log(`robot bridge → ${cfg.IP} · http://localhost:8090 (/drive?v=&w=, /stop, /status)`))
