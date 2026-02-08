import { defineConfig } from 'vite'
import react from '@vitejs/plugin-react'

export default defineConfig({
  plugins: [react()],
  server: {
    proxy: {
      '/api/sim': {
        target: 'http://localhost:8001',
        rewrite: (path) => path.replace(/^\/api\/sim/, ''),
      },
      '/api/infer': {
        target: 'http://localhost:8002',
        rewrite: (path) => path.replace(/^\/api\/infer/, ''),
      },
    },
  },
})
