<template>
  <div>
    <div class="relative size-8">
      <div class="absolute -top-1 h-full w-full rounded-full border border-white bg-black grid place-content-center">
        <Icon
          v-if="icon"
          :icon
          class="text-white text-sm"/>
        <span
          v-else
          class="font-mono text-white text-sm">{{ name }}</span>
      </div>
      <div class="h-full w-full rounded-full border border-white border-dotted"/>
    </div>
  </div>
</template>
<script setup lang="ts">
import { Icon } from '@iconify/vue';
import { computed } from 'vue';
          
const { name } = defineProps<{
  name: string
}>()

const icon = computed(() => {
  return ({
    '↑': 'fa6-solid:chevron-up',
    '←': 'fa6-solid:chevron-left',
    '↓': 'fa6-solid:chevron-down',
    '→': 'fa6-solid:chevron-right'
  })[name]
})
</script>