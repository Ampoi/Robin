<template>
  <main class="flex flex-row gap-4 p-4 bg-black w-screen h-screen">
    <div class="grow flex flex-col gap-4 items-center">
      <TopicNameSelector
        v-model="videoTopicName"
        :ros/>
      <Video
        :ros
        :videoTopicName
        class="grow"/>
    </div>
    <div class="min-w-max flex flex-col gap-4">
      <TopicNameSelector
        v-model="debugConsoleTopicName"
        :ros/>
      <DebugConsole
        :ros
        :name="debugConsoleTopicName"/>
    </div>
  </main>
</template>
<script setup lang="ts">
import { ref } from "vue";
import { createRos } from "./api/ros.ts"
import TopicNameSelector from "./components/topicNameSelector.vue";
import Video from "./components/video.vue"
import DebugConsole from "./components/debgConsole.vue";
const { ros } = createRos()
const videoTopicName = ref("")
const debugConsoleTopicName = ref("")
</script>