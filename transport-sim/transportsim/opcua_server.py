"""OPC UA server exposing the transport IO (see io.py for the list)."""
from __future__ import annotations

import asyncio
import logging

from asyncua import Server, ua

from . import subscription_tweaks
from .io import DESCRIPTIONS, PLC_BOOL, PLC_INT, SIM_BOOL, SIM_DWORD, SIM_INT

log = logging.getLogger("opcua")

FOLDER = "Transport"
TYPES = ([(n, ua.VariantType.Boolean, False) for n in PLC_BOOL + SIM_BOOL] +
         [(n, ua.VariantType.Int16, 0) for n in PLC_INT + SIM_INT] +
         [(n, ua.VariantType.UInt32, 0) for n in SIM_DWORD])
PLC_TO_SIM = set(PLC_BOOL + PLC_INT)


class _Users:
    def __init__(self, user, password):
        self.user, self.password = user, password

    def get_user(self, iserver, username=None, password=None, certificate=None):
        from asyncua.crypto.permission_rules import User, UserRole
        if username == self.user and password == self.password:
            return User(role=UserRole.Admin, name=username)
        return None


class SimServer:
    def __init__(self, cfg: dict):
        o = cfg["opcua"]
        self.endpoint = o["endpoint"]
        self.uri = o["namespace_uri"]
        self.cap_ms = float(o.get("publishing_interval_cap_ms", 0) or 0)
        self._auth = (o.get("username"), o.get("password")) if o.get("username") else None
        self.mode = o.get("mode", "hub")                  # "hub": common OPC UA server (plc-hub), "standalone": own server
        self.hub_cfg = cfg.get("hub")
        self.link = None
        self.server: Server | None = None
        self.nodes: dict[str, object] = {}
        self.vtype = {n: t for n, t, _ in TYPES}
        self._last: dict[str, object] = {}

    async def start(self):
        if self.mode == "hub":
            from plchub.client import HubLink              # the tags live on the common hub; nodes are cached stand-ins
            self.link = HubLink("transport", self.hub_cfg)
            await self.link.start()
            self.nodes = self.link.nodes
            return
        subscription_tweaks.apply(self.cap_ms)
        s = Server(user_manager=_Users(*self._auth)) if self._auth else Server()
        await s.init()
        s.set_endpoint(self.endpoint)
        s.set_server_name("Transport simulator")
        idx = await s.register_namespace(self.uri)
        folder = await s.nodes.objects.add_folder(idx, FOLDER)
        for name, vt, init in TYPES:
            n = await folder.add_variable(ua.NodeId(name, idx), name, init, varianttype=vt)
            n.description = DESCRIPTIONS.get(name, "")
            if name in PLC_TO_SIM:
                await n.set_writable()
            self.nodes[name] = n
        await s.start()
        self.server = s
        self.ns_index = idx
        log.info("OPC UA server on %s, namespace %d (%s), %d variables", self.endpoint, idx, self.uri, len(self.nodes))

    async def stop(self):
        if self.link:
            await self.link.stop()
            return
        if self.server:
            await self.server.stop()

    def client_count(self) -> int:
        if self.link:
            return 1 if self.link.plc_connected() else 0      # the hub knows whether a PLC is connected
        b = self.server.bserver if self.server else None
        return len(b.clients) if b else 0

    async def read_commands(self) -> dict:
        names = PLC_BOOL + PLC_INT
        vals = await asyncio.gather(*(self.nodes[n].read_value() for n in names))
        return dict(zip(names, vals))

    async def write_outputs(self, words: dict):
        """Write only changed values (a subscription reports changes anyway)."""
        for name, val in words.items():
            vt = self.vtype[name]
            val = bool(val) if vt == ua.VariantType.Boolean else int(val)
            if self._last.get(name) != val:
                await self.nodes[name].write_value(ua.DataValue(ua.Variant(val, vt)))
                self._last[name] = val
