"""OPC UA IO of the jacketed tank (all Int16, node id = tag name, 0..32767 raw):

    written by the PLC (client -> simulator)   M_YR01  control value: heater power   (0..32767 = 0..100 %)
                                               M_YR02  control value: inflow / level (0..32767 = 0..1.0)
    written by the simulator (-> PLC)          M_XG01  process value: temperature    (0..32767 = 20..120 degC)
                                               M_XG02  process value: level          (0..32767 = 0.1..0.9 m)

Default mode "hub": the tags live on the common OPC UA server (plc-hub).  Mode "standalone": own server.
"""
from __future__ import annotations

import logging

from asyncua import Server, ua

from . import subscription_tweaks

log = logging.getLogger("opcua")

FOLDER = "JacketTank"
PLC_TO_SIM = {"M_YR01": "Control value: heater power (0-32767 = 0-100 %)",
              "M_YR02": "Control value: level / inflow (0-32767 = 0-1.0)"}
SIM_TO_PLC = {"M_XG01": "Process value: temperature (0-32767 = 20-120 degC)",
              "M_XG02": "Process value: level (0-32767 = 0.1-0.9 m)"}


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
        self.mode = o.get("mode", "hub")                  # "hub": common OPC UA server (plc-hub), "standalone": own server
        self.hub_cfg = cfg.get("hub")
        self.link = None
        self.cap_ms = float(o.get("publishing_interval_cap_ms", 0) or 0)
        self._auth = (o.get("username"), o.get("password")) if o.get("username") else None
        self.server: Server | None = None
        self.nodes: dict[str, object] = {}
        self._last: dict[str, int] = {}

    async def start(self):
        if self.mode == "hub":
            from plchub.client import HubLink              # the tags live on the common hub; nodes are cached stand-ins
            self.link = HubLink("jacket", self.hub_cfg)
            await self.link.start()
            self.nodes = self.link.nodes
            return
        subscription_tweaks.apply(self.cap_ms)
        s = Server(user_manager=_Users(*self._auth)) if self._auth else Server()
        await s.init()
        s.set_endpoint(self.endpoint)
        s.set_server_name("Jacket tank simulator")
        idx = await s.register_namespace(self.uri)
        folder = await s.nodes.objects.add_folder(idx, FOLDER)
        for name, desc in {**PLC_TO_SIM, **SIM_TO_PLC}.items():
            n = await folder.add_variable(ua.NodeId(name, idx), name, 0, varianttype=ua.VariantType.Int16)
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

    async def read_controls(self) -> tuple[int, int]:
        a = await self.nodes["M_YR01"].read_value()
        b = await self.nodes["M_YR02"].read_value()
        return int(a), int(b)

    async def write_outputs(self, raw1: int, raw2: int):
        """Write only changed values (a subscription reports changes anyway)."""
        for name, val in (("M_XG01", int(raw1)), ("M_XG02", int(raw2))):
            if self._last.get(name) != val:
                await self.nodes[name].write_value(ua.DataValue(ua.Variant(val, ua.VariantType.Int16)))
                self._last[name] = val
