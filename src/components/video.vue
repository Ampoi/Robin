<template>
  <main
    class="bg-black overflow-hidden flex items-center justify-center"
  >
    <canvas
      ref="imageCanvas"
      class="w-full h-full object-contain border " />
  </main>
</template>
<script setup lang="ts">
import { ref, onMounted, watch, computed } from "vue";
import RosLib from "roslib";

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

onMounted(() => {
  watch(imageTopic, (topic) => {
    topic.subscribe(async (_message) => {
      if (!imageCanvas.value) throw new Error("imageCanvas is undefined");
      const message = _message as Message;
      const { width, height } = message;
      imageCanvas.value.width = width;
      imageCanvas.value.height = height;
      const ctx = imageCanvas.value.getContext("2d");
      if (!ctx) throw new Error("ctx is null");
      const imageData = ctx.createImageData(width, height);

      const binaryString = atob(message.data);
      const pixelColorBrightnesses = Array.from(binaryString).map(char => char.charCodeAt(0));
      const totalPixels = width * height;
      for (let i = 0, j = 0; i < totalPixels * 3; i += 3, j += 4) {
        imageData.data[j]     = pixelColorBrightnesses[i];
        imageData.data[j + 1] = pixelColorBrightnesses[i+1];
        imageData.data[j + 2] = pixelColorBrightnesses[i+2];
        imageData.data[j + 3] = 255;
      }

      ctx.putImageData(imageData, 0, 0);
    });
  }, { immediate: true })
});
</script>
