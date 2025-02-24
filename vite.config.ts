import { defineConfig } from 'vite'
import vue from '@vitejs/plugin-vue'
import { VitePWA } from 'vite-plugin-pwa'
// https://vite.dev/config/
export default defineConfig({
  plugins: [
    vue(),
    VitePWA({
      registerType: 'autoUpdate',
      devOptions: {
        enabled: true
      },
      manifest: {
        name: 'PyLoT Controller',
        short_name: 'PyLoT Joy',
        theme_color: '#000000',       // テーマカラーを黒に設定
        background_color: '#000000',  // スプラッシュスクリーンの背景を黒に設定
        display: 'standalone'
      },
    })
  ],
})
