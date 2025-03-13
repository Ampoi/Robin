<template>
  <main
    class="bg-black overflow-hidden flex items-center justify-center w-full rounded-md border border-white">
    <canvas
      ref="imageCanvas"
      class="w-full h-full object-contain" />
  </main>
</template>
<script setup lang="ts">
import { ref, onMounted, watch } from "vue";
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

async function getTopic(){
  const messageType = await new Promise<string>((resolve) => ros.getTopicType(videoTopicName, resolve))
  return new RosLib.Topic({
    ros: ros,
    name: videoTopicName,
    messageType
  })
}

const imageTopic = ref<RosLib.Topic>()
watch([
  () => ros, 
  () => videoTopicName
], async () => {
  imageTopic.value = await getTopic()
})

function drawCameraImage(_message: RosLib.Message){
  const message = _message as Message;
  let image_data = message.data
  
  let _width = '';
  let _height = '';
  
  let colonCount = 0;
  let index = 0;

  // 最初の2つのコロンを見つけるまで処理
  while (index < image_data.length) {
    let char = image_data[index];
    
    if (char === ':') {
        colonCount++;
        if (colonCount === 1) {
            _width = image_data.substring(0, index);
        } else if (colonCount === 2) {
            _height = image_data.substring(_width.length + 1, index);
            break;
        }
    }
    index++;
  }
  const width = Number(_width)
  const height = Number(_height)
  const image_base64 = image_data.substring(_width.length + _height.length + 2);

  if (!imageCanvas.value) throw new Error("imageCanvas is undefined");
  imageCanvas.value.width = width;
  imageCanvas.value.height = height;
  
  const ctx = imageCanvas.value.getContext("2d");
  if (!ctx) throw new Error("ctx is null");

  const img = new Image();
  img.src = `data:image/jpeg;base64,${image_base64}`;
  img.onload = () => {
    if (!imageCanvas.value) throw new Error("imageCanvas is undefined");
    console.log("draw!")
    console.log(image_base64)
    ctx.drawImage(img, 0, 0, imageCanvas.value.width, imageCanvas.value.height)
  }
}

onMounted(async () => {
  watch(imageTopic, (topic) => {
    if( !topic ) return
    topic.subscribe(drawCameraImage);
  })
});
</script>
