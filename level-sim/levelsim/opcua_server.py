"""OPC UA server exposing the level process IO (folder ``Level``, string node ids).

    written by the PLC (client -> simulator)   T_FCV10  Int16  valve command, 0..raw_full = 0..100 %
    written by the simulator (-> PLC)          T_LT10   Int16  level transmitter, 0..raw_full = 0..100 %
                                               T_LSH10  Bool   high level switch (level reached the high mark)
                                               T_LSL10  Bool   low level switch  (level reached the low mark)
                                               T_LSHH10 Bool   high-high level switch (level reached the high-high mark, default 95 %)
"""
from __future__ import annotations

import logging

from asyncua import Server, ua

from . import subscription_tweaks

log = logging.getLogger("opcua")

FOLDER = "Level"
PLC_TO_SIM = {"T_FCV10": ("Vessel system input: inlet valve command", ua.VariantType.Int16)}
SIM_TO_PLC = {
    "T_LT10": ("Vessel system output: level transmitter", ua.VariantType.Int16),
    "T_LSH10": ("Vessel high level switch", ua.VariantType.Boolean),
    "T_LSL10": ("Vessel low level switch", ua.VariantType.Boolean),
    "T_LSHH10": ("Vessel high-high level switch", ua.VariantType.Boolean),
}


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
        self._last: dict[str, object] = {}

    async def start(self):
        if self.mode == "hub":
            from plchub.client import HubLink              # the tags live on the common hub; nodes are cached stand-ins
            self.link = HubLink("level", self.hub_cfg)
            await self.link.start()
            self.nodes = self.link.nodes
            return
        subscription_tweaks.apply(self.cap_ms)
        s = Server(user_manager=_Users(*self._auth)) if self._auth else Server()
        await s.init()
        s.set_endpoint(self.endpoint)
        s.set_server_name("Level simulator")
        idx = await s.register_namespace(self.uri)
        folder = await s.nodes.objects.add_folder(idx, FOLDER)
        for name, (desc, vt) in {**PLC_TO_SIM, **SIM_TO_PLC}.items():
            n = await folder.add_variable(ua.NodeId(name, idx), name, False if vt == ua.VariantType.Boolean else 0,
                                          varianttype=vt)
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

    async def read_valve_raw(self) -> int:
        return int(await self.nodes["T_FCV10"].read_value())

    async def write_feedbacks(self, lt_raw: int, lsh: bool, lsl: bool, lshh: bool = False):
        """Write only changed values (a subscription reports changes anyway)."""
        vals = {"T_LT10": (int(lt_raw), ua.VariantType.Int16),
                "T_LSH10": (bool(lsh), ua.VariantType.Boolean),
                "T_LSL10": (bool(lsl), ua.VariantType.Boolean),
                "T_LSHH10": (bool(lshh), ua.VariantType.Boolean)}
        for name, (val, vt) in vals.items():
            if self._last.get(name) != val:
                await self.nodes[name].write_value(ua.DataValue(ua.Variant(val, vt)))
                self._last[name] = val
