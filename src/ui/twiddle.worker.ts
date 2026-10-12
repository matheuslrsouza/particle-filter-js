import { twiddle, type TwiddleSetup } from '../core/control/twiddle'
import type { PidGains } from '../core/control/pid'

self.onmessage = (e: MessageEvent<{ gains: PidGains; setup: TwiddleSetup }>) => {
  const { gains, setup } = e.data
  const result = twiddle(gains, setup, 40, (g, err) => self.postMessage({ type: 'progress', gains: g, err }))
  self.postMessage({ type: 'done', gains: result })
}
