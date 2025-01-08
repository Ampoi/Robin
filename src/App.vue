<template>
  <main class="flex flex-row gap-4 p-4 bg-black w-screen h-screen">
    <Video
      :ros
      :videoTopicName
      class="grow"/>
    <div class="min-w-max flex flex-col gap-4">
      <TopicNameSelector
        v-model="videoTopicName"
        :ros/>
      <div>

      </div>
    </div>
  </main>
</template>
<script setup lang="ts">
import { ref } from "vue";
import { createRos } from "./api/ros.ts"
import TopicNameSelector from "./components/topicNameSelector.vue";
import Video from "./components/video.vue"

const { ros } = createRos()
const videoTopicName = ref("")
</script>