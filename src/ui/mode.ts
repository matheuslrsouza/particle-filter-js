import type GUI from 'lil-gui'

export type AppMode = 'known' | 'explore'

/** Adds the mode selector; switching reloads the page with the new `?mode=`. */
export function addModeSelector(gui: GUI, current: AppMode): void {
  gui
    .add({ mode: current }, 'mode', { 'Particle filter (known map)': 'known', 'Exploration (Hybrid A*)': 'explore' })
    .name('Mode')
    .onChange((m: AppMode) => {
      const url = new URL(location.href)
      url.search = m === 'known' ? '' : `?mode=${m}`
      location.href = url.toString()
    })
}
