<template>
  <main class="bg-black w-screen h-screen overflow-hidden flex items-center justify-center">
    <canvas
      ref="faceCanvas"
      class="w-full h-full object-contain"/>
  </main>
</template>
<script setup lang="ts">
import RosLib from "roslib"
import { onMounted, ref } from "vue"

const RosWebsocketUrl = "ws://localhost:9090"
const faceImageTopicName = "/camera/color/image_raw"

const ros = new RosLib.Ros({
  url: RosWebsocketUrl
})

ros.on("connection", () => {
  console.log('🙌 Connected to WebSocket');
});

ros.on('error', (error) => {
  console.log('⚠ Error occurred in WebSocket Connection: ', error);
});

ros.on('close', () => {
  console.log('👋 WebSocket Closed!');
});

const faceImageTopic = new RosLib.Topic({
  ros,
  name: faceImageTopicName,
  messageType: "sensor_msgs/msg/Image"
})

const faceCanvas = ref<HTMLCanvasElement>()

interface Message {
  data: string
  encoding: "rgb8"
  header: {
    frame_id: string
    stamp: {
      sec: number
      nanosec: number
    }
  },
  height: number
  width: number
  step: number
  is_bigendian: number
}

function* base64ToBitesGenerator(base64: string) {
  const binaryString = atob(base64);
  const len = binaryString.length;
  for (let i = 0; i < len; i++) {
    yield binaryString.charCodeAt(i);
  }
}

onMounted(() => {
  let a = false
  faceImageTopic.subscribe(async (_message) => {
    if( a ) return
  //a = true
    if( !faceCanvas.value ) throw new Error("faceImage is undefined")
    const message = _message as Message
    const { width, height } = message
    faceCanvas.value.width = width
    faceCanvas.value.height = height
    const generator = base64ToBitesGenerator(message.data)
    const ctx = faceCanvas.value.getContext("2d")
    if( !ctx ) throw new Error("ctx is null")
    const imageData = ctx.createImageData(width, height)

    let i = 0
    function putBitColor(a: number){
      const { value, done } = generator.next()
      if( done ) return true
      imageData.data[a] = value
      return false
    }

    while( true ){
      const done1 = putBitColor(i)
      const done2 = putBitColor(i+1)
      const done3 = putBitColor(i+2)
      imageData.data[i+3] = 255
      if( done1 || done2 || done3 ) break
      i += 4
    }

    ctx.putImageData(imageData, 0, 0)
  })
})
</script>