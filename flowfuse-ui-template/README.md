# ui-template: store data, retrieve on `mounted()`

Dashboard 2.0 `ui-template` that loads persisted data when it mounts.

1. `mounted()` sends `{topic: 'load'}` to the flow.
2. The `load / save` function node answers `{topic: 'data', payload: {...}}` from flow context (file store).
3. The template's `msg` watcher fills the form. `Save` sends `{topic: 'save', payload}`, which is written back to context.

`resendOnRefresh` is on, so the last `data` msg is also replayed to reconnecting clients.

## Setup
Persistent storage needs a `file` context store in Node-RED `settings.js`:

```js
contextStorage: {
    default: { module: 'memory' },
    file: { module: 'localfilesystem' }
}
```

Import `flow.json` (Node-RED > Import), deploy, open `/dashboard/settings`.
`template.vue` is the same template as in the flow, for easier editing.
