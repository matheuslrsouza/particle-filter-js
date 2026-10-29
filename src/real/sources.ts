/**
 * Where the lidar bytes come from: the USB-serial adapter (Web Serial, Chrome/Edge on desktop) or
 * a recorded capture played back at the speed the lidar sends it. The ESP32 over Wi-Fi will be one
 * more source with the same interface.
 */
export interface ByteSource {
  readonly label: string
  /** resolves when the source is running; `onEnd` is called once when it stops by itself */
  start(onData: (bytes: Uint8Array) => void, onEnd: (error?: string) => void): Promise<void>
  stop(): Promise<void>
}

// minimal Web Serial types (not in TypeScript's DOM library yet)
interface SerialPortLike {
  open(o: { baudRate: number }): Promise<void>
  close(): Promise<void>
  readable: ReadableStream<Uint8Array> | null
}
interface SerialLike {
  requestPort(): Promise<SerialPortLike>
}

export const hasWebSerial = () => 'serial' in navigator

/** USB-serial adapter. Must be created from a click (the browser asks which port to use). */
export class SerialSource implements ByteSource {
  readonly label = 'USB-serial'
  private port: SerialPortLike | null = null
  private reader: ReadableStreamDefaultReader<Uint8Array> | null = null
  private stopping = false

  constructor(private baudRate = 115200) {}

  async start(onData: (bytes: Uint8Array) => void, onEnd: (error?: string) => void): Promise<void> {
    const serial = (navigator as Navigator & { serial: SerialLike }).serial
    this.port = await serial.requestPort()
    await this.port.open({ baudRate: this.baudRate })
    this.stopping = false
    void (async () => {
      let error: string | undefined
      try {
        while (this.port?.readable && !this.stopping) {
          this.reader = this.port.readable.getReader()
          try {
            for (;;) {
              const { value, done } = await this.reader.read()
              if (done) break
              if (value) onData(value)
            }
          } finally {
            this.reader.releaseLock()
          }
        }
      } catch (e) {
        if (!this.stopping) error = (e as Error).message
      }
      onEnd(error)
    })()
  }

  async stop(): Promise<void> {
    this.stopping = true
    await this.reader?.cancel().catch(() => {})
    await this.port?.close().catch(() => {})
    this.port = null
  }
}

/** A recorded capture (raw bytes from the lidar), played in a loop at the lidar's data rate. */
export class ReplaySource implements ByteSource {
  private timer = 0
  private pos = 0

  /** ~9.4 kB/s: what the lidar sent in the sample capture (≈260 packets of 36 bytes per second) */
  constructor(
    readonly label: string,
    private bytes: Uint8Array,
    private bytesPerSecond = 9400,
  ) {}

  async start(onData: (bytes: Uint8Array) => void): Promise<void> {
    let last = performance.now()
    this.timer = window.setInterval(() => {
      const now = performance.now()
      let n = Math.round(((now - last) / 1000) * this.bytesPerSecond)
      last = now
      while (n > 0) {
        const end = Math.min(this.pos + n, this.bytes.length)
        onData(this.bytes.subarray(this.pos, end))
        n -= end - this.pos
        this.pos = end >= this.bytes.length ? 0 : end
      }
    }, 20)
  }

  async stop(): Promise<void> {
    clearInterval(this.timer)
  }
}

/** The ESP32 bridge (firmware/lidar_bridge): raw lidar bytes over a WebSocket. */
export class WebSocketSource implements ByteSource {
  readonly label: string
  private ws: WebSocket | null = null
  private stopping = false

  constructor(private url: string) {
    this.label = `ESP32 ${url.replace(/^ws:\/\//, '')}`
  }

  start(onData: (bytes: Uint8Array) => void, onEnd: (error?: string) => void): Promise<void> {
    // an https page (GitHub Pages) may not open ws:// connections to the local network
    if (location.protocol === 'https:') return Promise.reject(new Error('the ESP32 needs the app opened over http (npm run dev)'))
    this.stopping = false
    return new Promise((resolve, reject) => {
      const ws = new WebSocket(this.url)
      ws.binaryType = 'arraybuffer'
      let open = false
      ws.onopen = () => {
        open = true
        resolve()
      }
      ws.onmessage = (e) => onData(new Uint8Array(e.data as ArrayBuffer))
      ws.onerror = () => {
        if (!open) reject(new Error(`no answer from ${this.url}`))
      }
      ws.onclose = () => {
        if (open) onEnd(this.stopping ? undefined : 'connection lost')
      }
      this.ws = ws
    })
  }

  async stop(): Promise<void> {
    this.stopping = true
    this.ws?.close()
    this.ws = null
  }
}
