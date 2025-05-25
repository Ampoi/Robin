import { createApp } from 'vue'
import './style.css'
import App from './App.vue'

import { createRouter, createMemoryHistory } from 'vue-router'
import Controller from './pages/controller.vue'
//import Face from './pages/face.vue'

const routes = [
  { path: '/', component: Controller },
//  { path: '/face', component: Face },
]

const router = createRouter({
  history: createMemoryHistory(),
  routes,
})

createApp(App)
  .use(router)
  .mount('#app')
