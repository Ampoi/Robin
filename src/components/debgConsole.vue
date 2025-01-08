<template>
    <div class="grow border border-white rounded-xl max-h-min overflow-y-auto p-4">
        <div
            v-for="log of logs"
            class="border border-white ronded-sm text-white">
            {{ log }}
        </div>
    </div>
</template>
<script setup lang="ts">
import type { Ros } from 'roslib';
import RosLib from 'roslib';
import { computed, onMounted, reactive, watch } from 'vue';

const { ros, name } = defineProps<{
    name: string
    ros: Ros
}>()

const topic = computed(() => {
    return new RosLib.Topic({
        name,
        ros,
        messageType: "*"
    })
})

const logs = reactive<string[]>([])
const maxLogAmount = 100
watch(logs, () => {
    if(logs.length > maxLogAmount){
        logs.splice(0, logs.length - maxLogAmount)
    }
})

onMounted(() => {
    watch(topic, (topicSubscriber) => {
        topicSubscriber.subscribe((message) => {
            logs.push(JSON.stringify(message))
        })
    }, { immediate: true })
})
</script>