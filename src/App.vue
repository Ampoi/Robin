<template>
  <main class="flex flex-row gap-4 p-4 bg-black w-screen h-screen">
    <div class="flex flex-col gap-4 items-center basis-32">
      <div class="p-2 flex flex-col gap-4 w-full">
        <TriggerButton
          name="LT"
          class="w-full"/>
        <TriggerButton
          name="LB"
          class="w-full -mt-2"/>
        <div class="relative w-full aspect-square">
          <CircleButton
            name="↑"
            class="absolute top-0 left-1/2 -translate-x-1/2"/>
          <CircleButton
            name="←"
            class="absolute top-1/2 -translate-y-1/2 left-0"/>
          <CircleButton
            name="→"
            class="absolute top-1/2 -translate-y-1/2 right-0"/>
          <CircleButton
            name="↓"
            class="absolute bottom-0 left-1/2 -translate-x-1/2"/>
        </div>
      </div>
      <Stick
        v-model:position="leftStickPosition"/>
    </div>
    <div class="grow flex flex-col gap-2 items-center">
      <TopicNameSelector
        v-model="videoTopicName"
        :ros/>
      <Video
        :ros
        :videoTopicName
        class="grow"/>
    </div>
    <div class="flex flex-col gap-4 items-center basis-32">
      <div class="p-2 flex flex-col gap-4 w-full">
        <TriggerButton
          name="RT"
          class="w-full"/>
        <TriggerButton
          name="RB"
          class="w-full -mt-2"/>
        <div class="relative w-full aspect-square">
          <CircleButton
            name="Y"
            class="absolute top-0 left-1/2 -translate-x-1/2"/>
          <CircleButton
            name="X"
            class="absolute top-1/2 -translate-y-1/2 left-0"/>
          <CircleButton
            name="B"
            class="absolute top-1/2 -translate-y-1/2 right-0"/>
          <CircleButton
            name="A"
            class="absolute bottom-0 left-1/2 -translate-x-1/2"/>
        </div>
      </div>
      <Stick
        v-model:position="rightStickPosition"/>
    </div>
  </main>
</template>
<script setup lang="ts">
import CircleButton from "./components/circleButton.vue";
import TriggerButton from "./components/triggerButton.vue";
import Stick from "./components/stick.vue";

import { reactive, ref, watch } from "vue";
import { createRos } from "./api/ros.ts"
import TopicNameSelector from "./components/topicNameSelector.vue";
import Video from "./components/video.vue"

const { ros } = createRos()
const videoTopicName = ref("")
const debugConsoleTopicName = ref("")

const leftStickPosition = reactive({
  x: 0,
  y: 0
})

const rightStickPosition = reactive({
  x: 0,
  y: 0
})

function buttonPressed( name: string ){
  console.log(name)
}
</script>