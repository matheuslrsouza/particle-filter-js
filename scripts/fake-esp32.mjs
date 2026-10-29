// Stands in for the ESP32 lidar bridge: serves a recorded capture over a WebSocket, at
// the lidar's data rate, so the Wi-Fi path can be tested without hardware.
//   node scripts/fake-esp32.mjs [capture.bin]   → in the app: ESP32 address "localhost:8081"
import { createHash } from 'node:crypto'
import { readFileSync } from 'node:fs'
import { createServer } from 'node:http'

const bytes = readFileSync(process.argv[2] ?? new URL('../src/real/sample-capture.bin', import.meta.url))
const RATE = 9400 // bytes/s

/** One unmasked binary WebSocket frame (server → browser). */
function frame(payload) {
  const n = payload.length
  const head = n < 126 ? Buffer.from([0x82, n]) : Buffer.from([0x82, 126, n >> 8, n & 0xff])
  return Buffer.concat([head, payload])
}

const server = createServer()
server.on('upgrade', (req, socket) => {
  const accept = createHash('sha1').update(req.headers['sec-websocket-key'] + '258EAFA5-E914-47DA-95CA-C5AB0DC85B11').digest('base64')
  socket.write(`HTTP/1.1 101 Switching Protocols\r\nUpgrade: websocket\r\nConnection: Upgrade\r\nSec-WebSocket-Accept: ${accept}\r\n\r\n`)
  console.log('browser connected')
  let pos = 0
  const timer = setInterval(() => {
    const n = Math.round((RATE * 20) / 1000)
    const end = Math.min(pos + n, bytes.length)
    socket.write(frame(bytes.subarray(pos, end)))
    pos = end >= bytes.length ? 0 : end
  }, 20)
  const bye = () => {
    clearInterval(timer)
    console.log('browser disconnected')
  }
  socket.on('close', bye)
  socket.on('error', bye)
  socket.on('data', (d) => {
    if ((d[0] & 0x0f) === 0x8) socket.end() // close frame
  })
})
// port 81 needs root on Linux: the app takes "localhost:8081" as the ESP32 address
server.listen(8081, () => console.log('fake ESP32 on ws://localhost:8081'))
