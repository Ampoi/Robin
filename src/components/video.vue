<template>
  <main
    class="bg-black overflow-hidden flex items-center justify-center w-full rounded-md border border-white">
    <canvas
      ref="imageCanvas"
      class="w-full h-full object-contain" />
  </main>
</template>
<script setup lang="ts">
import { ref, onMounted, watch, computed } from "vue";
import RosLib from "roslib";
import testBase64 from '../assets/testImage.txt?raw'
import init, { convert } from "../api/fast-image-converter/fast_image_converter";

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

//何個に1個だけ処理するかの数値、自然数
const PIXEL_MODULO = 1; //縦横割り切れる値で・そんなに速度改善されない
const FRAME_MODULO = 1;

let frameCount = -1

function drawCameraImage(_message: RosLib.Message){
  frameCount += 1
  if(frameCount % FRAME_MODULO != 0) return
  frameCount = 0

  const message = _message as Message;
  const { width: _width, height: _height } = message;
  const width = _width / PIXEL_MODULO
  const height = _height / PIXEL_MODULO

  if (!imageCanvas.value) throw new Error("imageCanvas is undefined");
  imageCanvas.value.width = width;
  imageCanvas.value.height = height;
  
  const ctx = imageCanvas.value.getContext("2d");
  if (!ctx) throw new Error("ctx is null");
  const binaryString = atob(testBase64);
  const convertedArray = convert(binaryString, width, height, PIXEL_MODULO)
  const clampedArray = new Uint8ClampedArray(convertedArray);
  const imageData = new ImageData(clampedArray, width, height)
  ctx.putImageData(imageData, 0, 0);
}

onMounted(async () => {
  await init()
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
    if( !topic ) return
    topic.subscribe(drawCameraImage);
  })
});
</script>
