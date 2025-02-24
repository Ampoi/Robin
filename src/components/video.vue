<template>
  <main
    class="bg-black overflow-hidden flex items-center justify-center w-full">
    <canvas
      ref="imageCanvas"
      class="w-full h-full object-contain border" />
  </main>
</template>
<script setup lang="ts">
import { ref, onMounted, watch, computed } from "vue";
import RosLib from "roslib";
import testBase64 from '../assets/testImage.txt?raw'

const imageCanvas = ref<HTMLCanvasElement>();

interface Message {
  data: string;
  encoding: "rgb8";
  header: {
    frame_id: string;
    stamp: {
      sec: number;
      nanosec: number;
    };
  };
  height: number;
  width: number;
  step: number;
  is_bigendian: number;
}

const { ros, videoTopicName } = defineProps<{
  ros: RosLib.Ros;
  videoTopicName: string
}>();

const imageTopic = computed(() => {
  return new RosLib.Topic({
    ros: ros,
    name: videoTopicName,
    messageType: "sensor_msgs/msg/Image",
  })
})

function drawCameraImage(_message: RosLib.Message){
  const message = _message as Message;
  const { width, height } = message;
  if (!imageCanvas.value) throw new Error("imageCanvas is undefined");
  imageCanvas.value.width = width;
  imageCanvas.value.height = height;
  const ctx = imageCanvas.value.getContext("2d");
  if (!ctx) throw new Error("ctx is null");
  const imageData = ctx.createImageData(width, height);
  
  const binaryString = atob(testBase64);
  const pixelColorBrightnesses = Array.from(binaryString).map(char => char.charCodeAt(0));
  const totalPixels = width * height;
  for (let i = 0, j = 0; i < totalPixels * 3; i += 3, j += 4) {
    imageData.data[j]     = pixelColorBrightnesses[i];
    imageData.data[j + 1] = pixelColorBrightnesses[i+1];
    imageData.data[j + 2] = pixelColorBrightnesses[i+2];
    imageData.data[j + 3] = 255;
  }
  
  ctx.putImageData(imageData, 0, 0);
}

onMounted(() => {
  const testMessage: Message = {
    width: 640,
    height: 360,
    header: {
      frame_id: "camera",
      stamp: {
        sec: 0,
        nanosec: 0,
      },
    },
    data: testBase64,
    encoding: "rgb8",
    step: 0,
    is_bigendian: 0
  }
  console.time("testDraw")
  drawCameraImage(testMessage)
  console.timeEnd("testDraw")
  watch(imageTopic, (topic) => {
    topic.subscribe(drawCameraImage);
  }, { immediate: true })
});
</script>
