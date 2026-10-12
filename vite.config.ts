/// <reference types="vitest" />
import { defineConfig } from 'vite'

export default defineConfig({
  base: '/particle-filter-js/',
  build: { chunkSizeWarningLimit: 800 },
  worker: { format: 'es' },
  test: {
    include: ['src/**/*.test.ts'],
  },
})
