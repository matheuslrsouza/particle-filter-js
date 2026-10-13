import './style.css'

// ?mode=explore → unknown maze with Hybrid A*; default → particle filter on a known map
const mode = new URLSearchParams(location.search).get('mode')
if (mode === 'explore') import('./modes/explore').then((m) => m.startExplore())
else import('./modes/known').then((m) => m.startKnown())
