<template>
    <v-select
        v-model="selected"
        :items="items"
        item-title="title"
        item-value="value"
        label="Mode"
        variant="outlined"
        density="comfortable"
        hide-details="auto"
        @update:model-value="onSelect"
    />
</template>

<script>
export default {
    data () {
        return {
            selected: null,
            items: [
                { title: 'Off',    value: 0 },
                { title: 'Low',    value: 25 },
                { title: 'Medium', value: 50 },
                { title: 'High',   value: 100 }
            ]
        }
    },
    mounted () {
        this.send({ topic: 'load' })
    },
    watch: {
        msg (msg) {
            if (!msg) return
            // flow sets the current value: {topic: 'data', payload: 50}
            if (msg.topic === 'data' && typeof msg.payload === 'number') {
                this.selected = msg.payload
            }
            // flow replaces the list: {topic: 'items', payload: [{title, value}, ...]}
            else if (msg.topic === 'items' && Array.isArray(msg.payload)) {
                this.items = msg.payload
            }
        }
    },
    methods: {
        onSelect (value) {
            // value is the item's numeric value, not its label
            this.send({ topic: 'value', payload: value })
        }
    }
}
</script>
