<template>
  <div class="flex flex-row justify-center items-center">
    <div class="rounded-xl overflow-hidden border border-white flex flex-row">
      <select
        v-model="selectedTopic"
        placeholder="映像のTopic名"
        class="text-white outline-none bg-transparent px-3 py-2">
        <option
          v-for="topic in topics"
          :value="topic"
          class="text-black">
          {{topic}}
        </option>
      </select>
      <button @click="updateTopics" class="bg-white px-3 text-black">
        <Icon icon="fa6-solid:arrows-rotate"/>
      </button>
    </div>
  </div>
</template>
<script setup lang="ts">
import { Icon } from "@iconify/vue"
import type { Ros } from "roslib";
import { useTopics } from "../hooks/useTopics";

const { ros } = defineProps<{
  ros: Ros;
}>();

const { topics, updateTopics } = useTopics(ros);

const selectedTopic = defineModel<string>({ required: true });
</script>
