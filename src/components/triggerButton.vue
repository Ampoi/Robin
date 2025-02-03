<template>
  <div>
    <div class="relative w-full h-8">
      <div class="absolute -top-1 h-full w-full rounded-lg border border-white bg-black grid place-content-center">
        <span class="font-mono text-white text-sm">{{ name }}</span>
      </div>
      <div class="h-full w-full rounded-lg border border-white border-dotted">
        
      </div>
    </div>
  </div>
</template>
<script setup lang="ts">
defineProps<{
  name: string
}>()
</script>