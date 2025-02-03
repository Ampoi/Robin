<template>
  <div class="pt-3">
    <div class="relative size-20">
      <div
        ref="stickElement"
        @mousedown="(e) => mouseHandler(onControlStart)(e)"
        @mouseup="onControlEnd"
        @mousemove="(e) => mouseHandler(onControl)(e)"
        @touchstart.prevent="(e) => touchHandler(onControlStart)(e)"
        @touchend="onControlEnd"
        @touchmove="(e) => touchHandler(onControl)(e)"
        @dragstart.prevent
        class="absolute -top-3 h-full w-full rounded-full border border-white bg-black p-1.5">
        <div class="h-full w-full rounded-full border border-white border-dotted -translate-y-0.5"/>
      </div>
      <div class="h-full w-full p-2">
        <div class="h-full w-full rounded-full border border-white border-dotted">

        </div>
      </div>
    </div>
  </div>
</template>
<script setup lang="ts">
import { onMounted, ref } from "vue";

let isDragging = false;
let offsetX = 0;
let offsetY = 0;

const originalPosition = {
  top: 0,
  left: 0
};

onMounted(() => {
  if( !stickElement.value ) throw new Error("stick element not found");
  originalPosition.top = stickElement.value.offsetTop
  originalPosition.left = stickElement.value.offsetLeft
})

const stickElement = ref<HTMLDivElement>();

function onControlStart(clientX: number, clientY: number){
  isDragging = true;
  console.log(isDragging)
  if( !stickElement.value ) throw new Error("stick element not found");
  const dragElement = stickElement.value;
  const rect = dragElement.getBoundingClientRect();
  offsetX = clientX - rect.left;
  offsetY = clientY - rect.top;
  dragElement.style.transition = 'none';
}

function onControl(clientX: number, clientY: number){
  if (!isDragging) return;
  
  if( !stickElement.value ) throw new Error("stick element not found");
  const dragElement = stickElement.value;
  if( !dragElement.parentElement ) throw new Error("stick element parent not found");
  const containerRect = dragElement.parentElement.getBoundingClientRect();

  const newLeft = clientX - containerRect.left - offsetX;
  const newTop = clientY - containerRect.top - offsetY;
  
  dragElement.style.left = newLeft + 'px';
  dragElement.style.top = newTop + 'px';
}

function onControlEnd(){
  if (!isDragging) return;
  isDragging = false;
  
  if( !stickElement.value ) throw new Error("stick element not found");
  const dragElement = stickElement.value;
  dragElement.style.transition = 'left 0.3s ease, top 0.3s ease';
  dragElement.style.left = originalPosition.left + 'px';
  dragElement.style.top = originalPosition.top + 'px';
}

function mouseHandler(handler: (clientX: number, clientY: number) => void){
  return (e: MouseEvent) => {
    handler(e.clientX, e.clientY);
  }
}

function touchHandler(handler: (clientX: number, clientY: number) => void){
  return (e: TouchEvent) => {
    handler(e.touches[0].clientX, e.touches[0].clientY);
  }
}
</script>