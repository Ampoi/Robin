<template>
  <Video
    :ros/>
</template>
<script setup lang="ts">
import { createRos } from "./api/ros.ts"
import Video from "./components/video.vue"

const { ros } = createRos()
</script>