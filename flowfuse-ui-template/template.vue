<template>
    <v-card class="pa-4">
        <v-card-title>Stored settings</v-card-title>
        <v-text-field v-model="form.name" label="Name" :loading="loading" />
        <v-slider v-model="form.level" label="Level" min="0" max="100" thumb-label />
        <v-switch v-model="form.enabled" label="Enabled" color="primary" />
        <v-btn color="primary" @click="save">Save</v-btn>
    </v-card>
</template>

<script>
export default {
    data () {
        return {
            loading: true,
            form: { name: '', level: 50, enabled: false }
        }
    },
    mounted () {
        // Ask the flow for the stored data as soon as the component is mounted
        this.send({ topic: 'load' })
    },
    watch: {
        // `msg` is updated whenever the node receives a message from the flow
        msg (msg) {
            if (msg && msg.topic === 'data') {
                this.form = Object.assign({}, this.form, msg.payload)
                this.loading = false
            }
        }
    },
    methods: {
        save () {
            this.send({ topic: 'save', payload: { ...this.form } })
        }
    }
}
</script>
