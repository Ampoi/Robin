import { createApp } from 'vue'
import './style.css'
import App from './App.vue'

import { createRouter, createWebHistory } from 'vue-router'
import Controller from './pages/controller.vue'
import Face from './pages/face.vue'

const router = createRouter({
  history: createWebHistory(),
  routes: [
    { path: '/', component: Controller, name: "controller" },
    { path: '/face', component: Face, name: "face" },
  ],
})

createApp(App)
  .use(router)
  .mount('#app')
