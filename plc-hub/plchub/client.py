"""Client side of the hub, used by every simulation: ``HubLink``.

The simulation keeps its simple cycle:  ``await node.read_value()`` returns the last value the PLC wrote (cached, no network),
``await node.write_value(DataValue)`` only queues the value.  A background task does the OPC UA traffic (batched writes of
changed values, a subscription for the PLC's tags, a heartbeat) so a slow network never blocks the simulation cycle.
If the hub is not running it is started; if it goes away the link reconnects (and starts it again) and rewrites all values.
"""
from __future__ import annotations

import asyncio
import logging

from asyncua import Client, ua

from . import config, launcher
from .manifest import MANIFEST

log = logging.getLogger("hub.link")

VT = {"Boolean": ua.VariantType.Boolean, "Int16": ua.VariantType.Int16, "UInt32": ua.VariantType.UInt32}
_MISSING = object()


def coerce(vt_name: str, v):
    if vt_name == "Boolean":
        return bool(v)
    if vt_name == "Int16":
        return max(-32768, min(32767, int(v)))
    return int(v) & 0xFFFFFFFF


class HubNode:
    """Stand-in for an asyncua node, with the two calls the simulations use."""

    def __init__(self, link: "HubLink", tag: str):
        self.link, self.tag = link, tag

    async def read_value(self):
        return self.link.values[self.tag]

    async def write_value(self, value, varianttype=None):
        if isinstance(value, ua.DataValue):
            value = value.Value.Value
        self.link.put(self.tag, value)


class _Handler:
    def __init__(self, link: "HubLink"):
        self.link = link

    def datachange_notification(self, node, val, data):
        ident = node.nodeid.Identifier
        if ident == "PlcClients":
            self.link.plc_clients = int(val)
        elif ident in self.link.values and ident in self.link.plc_tags:
            self.link.values[ident] = coerce(self.link.types_name[ident], val)


class HubLink:
    def __init__(self, sim: str, cfg: dict | None = None, io_interval_s: float = 0.01, heartbeat_s: float = 0.3,
                 spawn: bool = True):
        self.sim = sim
        self.cfg = config.merge(cfg)
        spec = MANIFEST[sim]
        self.plc_tags = set(spec["plc_to_sim"])
        self.sim_tags = set(spec["sim_to_plc"])
        self.types_name = {t: v[0] for d in ("plc_to_sim", "sim_to_plc") for t, v in spec[d].items()}
        self.vt = {t: VT[n] for t, n in self.types_name.items()}
        self.values = {t: coerce(n, 0) for t, n in self.types_name.items()}
        self.nodes = {t: HubNode(self, t) for t in self.types_name}
        self.io_interval = io_interval_s
        self.heartbeat_s = heartbeat_s
        self.spawn = spawn
        self.connected = False
        self.plc_clients = 0
        self._pending: dict[str, object] = {}
        self._sent: dict[str, object] = {}
        self._written: set[str] = set()
        self._running = False
        self._task: asyncio.Task | None = None
        self._ready = asyncio.Event()
        self._warned = False

    # ----------------------------------------------------------------- simulation side
    def plc_connected(self) -> bool:
        return self.connected and self.plc_clients > 0

    def put(self, tag: str, value):
        if tag not in self.sim_tags:
            raise KeyError("%s is not a sim -> PLC tag of %s" % (tag, self.sim))
        v = coerce(self.types_name[tag], value)
        self.values[tag] = v
        self._written.add(tag)
        if self._sent.get(tag, _MISSING) != v:
            self._pending[tag] = v

    async def start(self, wait_s: float = 25.0):
        self._running = True
        self._task = asyncio.create_task(self._supervise())
        try:
            await asyncio.wait_for(self._ready.wait(), wait_s)
        except asyncio.TimeoutError:
            log.warning("hub not reachable yet; the simulation runs on and keeps trying")

    async def stop(self):
        self._running = False
        if self._task:
            self._task.cancel()
            try:
                await self._task
            except (asyncio.CancelledError, Exception):
                pass
        self.connected = False

    # ----------------------------------------------------------------------- network
    async def _supervise(self):
        while self._running:
            client = None
            try:
                if self.spawn:
                    await launcher.ensure_hub(self.cfg)
                client = Client(config.url(self.cfg), timeout=4)
                client.description = config.SIM_MARK + self.sim
                await client.connect()
                await self._session(client)
            except asyncio.CancelledError:
                raise
            except Exception as e:
                if self.connected or not self._warned:
                    log.warning("hub connection (%s): %s: %s - retrying", self.sim, type(e).__name__, e)
                    self._warned = True
            finally:
                self.connected = False
                self.plc_clients = 0
                if client is not None:
                    try:
                        await asyncio.wait_for(client.disconnect(), 2)
                    except Exception:
                        pass
            if self._running:
                await asyncio.sleep(1.0)

    async def _session(self, client: Client):
        idx = await client.get_namespace_index(self.cfg["namespace_uri"])
        didx = await client.get_namespace_index(self.cfg["diag_namespace_uri"])
        node = {t: client.get_node(ua.NodeId(t, idx)) for t in self.types_name}
        hb_node = client.get_node(ua.NodeId(self.sim + ".Heartbeat", didx))
        plc_node = client.get_node(ua.NodeId("PlcClients", didx))
        plc_nodes = [node[t] for t in sorted(self.plc_tags)]
        if plc_nodes:
            for t, v in zip(sorted(self.plc_tags), await client.read_values(plc_nodes)):
                self.values[t] = coerce(self.types_name[t], v)
        self.plc_clients = int(await plc_node.read_value())
        sub = await client.create_subscription(max(10, int(self.io_interval * 1000)), _Handler(self))
        await sub.subscribe_data_change(plc_nodes + [plc_node], queuesize=1)
        # (re)connected: resend everything the simulation has written so far
        self._sent.clear()
        self._pending = {t: self.values[t] for t in self._written}
        self.connected = True
        self._warned = False
        self._ready.set()
        log.info("simulation %s connected to the hub (%d tags, PLC clients %d)", self.sim, len(self.types_name), self.plc_clients)
        loop = asyncio.get_running_loop()
        hb = int(loop.time() * 1000) & 0xFFFFFFFF          # distinct start value: a restarted simulation never repeats the last counter
        last_hb = 0.0
        while self._running:
            now = loop.time()
            writes = list(self._pending.items())
            self._pending.clear()
            if now - last_hb >= self.heartbeat_s:
                hb = (hb + 1) & 0xFFFFFFFF
                last_hb = now
                writes_hb = True
            else:
                writes_hb = False
            if writes or writes_hb:
                nodes = [node[t] for t, _ in writes]
                values = [ua.DataValue(ua.Variant(v, self.vt[t])) for t, v in writes]
                if writes_hb:
                    nodes.append(hb_node)
                    values.append(ua.DataValue(ua.Variant(hb, ua.VariantType.UInt32)))
                res = await client.write_values(nodes, values, raise_on_partial_error=False)
                for (t, v), sc in zip(writes, res):
                    if sc.is_good():
                        self._sent[t] = v
                    elif not self._warned:
                        log.warning("write of %s refused: %s", t, sc)
            await asyncio.sleep(self.io_interval)
