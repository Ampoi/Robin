<template>
  <main
    class="bg-black overflow-hidden flex items-center justify-center"
  >
    <canvas ref="imageCanvas" class="w-full h-full object-contain" />
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

function* base64ToBitesGenerator(base64: string) {
  const binaryString = atob(base64);
  const len = binaryString.length;
  for (let i = 0; i < len; i++) {
    yield binaryString.charCodeAt(i);
  }
}

const { ros, videoTopicName } = defineProps<{
  ros: RosLib.Ros;
  videoTopicName: string
}>();

const imageTopic =computed(() => {
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
      const generator = base64ToBitesGenerator(message.data);
      const ctx = imageCanvas.value.getContext("2d");
      if (!ctx) throw new Error("ctx is null");
      const imageData = ctx.createImageData(width, height);
  
      let i = 0;
      function putBitColor(a: number) {
        const { value, done } = generator.next();
        if (done) return true;
        imageData.data[a] = value;
        return false;
      }
  
      while (true) {
        const done1 = putBitColor(i);
        const done2 = putBitColor(i + 1);
        const done3 = putBitColor(i + 2);
        imageData.data[i + 3] = 255;
        if (done1 || done2 || done3) break;
        i += 4;
      }
  
      ctx.putImageData(imageData, 0, 0);
    });
  }, { immediate: true })
});
</script>
