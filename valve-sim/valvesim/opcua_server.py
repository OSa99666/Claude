"""OPC UA server exposing the valve IO (all Bool, folder ``Valve``, string node ids).

    written by the PLC (client -> simulator)   V_YH  command High (active/open)
                                               V_YL  command Low  (passive/closed)
    written by the simulator (-> PLC)          V_XGH valve is High
                                               V_XGL valve is Low
"""
from __future__ import annotations

import logging

from asyncua import Server, ua

from . import subscription_tweaks

log = logging.getLogger("opcua")

FOLDER = "Valve"
PLC_TO_SIM = {"V_YH": "Valve command high", "V_YL": "Valve command low"}
SIM_TO_PLC = {"V_XGH": "Valve is high", "V_XGL": "Valve is low"}


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
        self._last: dict[str, bool] = {}

    async def start(self):
        if self.mode == "hub":
            from plchub.client import HubLink              # the tags live on the common hub; nodes are cached stand-ins
            self.link = HubLink("valve", self.hub_cfg)
            await self.link.start()
            self.nodes = self.link.nodes
            return
        subscription_tweaks.apply(self.cap_ms)
        s = Server(user_manager=_Users(*self._auth)) if self._auth else Server()
        await s.init()
        s.set_endpoint(self.endpoint)
        s.set_server_name("Valve simulator")
        idx = await s.register_namespace(self.uri)
        folder = await s.nodes.objects.add_folder(idx, FOLDER)
        for name, desc in {**PLC_TO_SIM, **SIM_TO_PLC}.items():
            n = await folder.add_variable(ua.NodeId(name, idx), name, False,
                                          varianttype=ua.VariantType.Boolean)
            n.description = desc
            if name in PLC_TO_SIM:
                await n.set_writable()
            self.nodes[name] = n
        await s.start()
        self.server = s
        self.ns_index = idx
        log.info("OPC UA server on %s, namespace %d (%s)", self.endpoint, idx, self.uri)

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

    async def read_commands(self) -> tuple[bool, bool]:
        h = await self.nodes["V_YH"].read_value()
        l = await self.nodes["V_YL"].read_value()
        return bool(h), bool(l)

    async def write_feedbacks(self, words: dict[str, bool]):
        """Write only changed values (a subscription reports changes anyway)."""
        for name, val in words.items():
            if self._last.get(name) is not val:
                await self.nodes[name].write_value(ua.DataValue(ua.Variant(bool(val), ua.VariantType.Boolean)))
                self._last[name] = val
