"""OPC UA server exposing the rolling-table IO words.

Variables (all Int16, node id = string identifier in the simulation namespace):

    written by the PLC (client -> simulator)      F_YR10  tilt X
                                                  F_YR20  tilt Y
    written by the simulator (simulator -> PLC)   F_XG10  acceleration X
                                                  F_XG11  velocity X
                                                  F_XG12  distance X
                                                  F_XG20  acceleration Y
                                                  F_XG21  velocity Y
                                                  F_XG22  distance Y
"""
from __future__ import annotations

import logging

from asyncua import Server, ua

from . import subscription_tweaks

log = logging.getLogger("opcua")

FOLDER = "Ballplate"
PLC_TO_SIM = {"F_YR10": "Tilt X direction", "F_YR20": "Tilt Y direction"}
SIM_TO_PLC = {
    "F_XG10": "Acceleration X direction", "F_XG11": "Velocity X direction",
    "F_XG12": "Distance X direction",
    "F_XG20": "Acceleration Y direction", "F_XG21": "Velocity Y direction",
    "F_XG22": "Distance Y direction",
}


class _Users:
    """Minimal username/password manager for asyncua."""

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
        self.server: Server | None = None
        self.nodes: dict[str, object] = {}

    async def start(self):
        subscription_tweaks.apply(self.cap_ms)
        s = Server(user_manager=_Users(*self._auth)) if self._auth else Server()
        await s.init()
        s.set_endpoint(self.endpoint)
        s.set_server_name("Rolling table simulator")
        idx = await s.register_namespace(self.uri)
        folder = await s.nodes.objects.add_folder(idx, FOLDER)
        for name, desc in {**PLC_TO_SIM, **SIM_TO_PLC}.items():
            n = await folder.add_variable(ua.NodeId(name, idx), name, 0,
                                          varianttype=ua.VariantType.Int16)
            n.description = desc
            if name in PLC_TO_SIM:
                await n.set_writable()
            self.nodes[name] = n
        await s.start()
        self.server = s
        self.ns_index = idx
        log.info("OPC UA server on %s, namespace %d (%s)", self.endpoint, idx, self.uri)

    async def stop(self):
        if self.server:
            await self.server.stop()

    # ------------------------------------------------------------------ io
    def client_count(self) -> int:
        b = self.server.bserver if self.server else None
        return len(b.clients) if b else 0

    async def read_tilts(self) -> tuple[int, int]:
        x = await self.nodes["F_YR10"].read_value()
        y = await self.nodes["F_YR20"].read_value()
        return int(x), int(y)

    async def write_outputs(self, words: dict[str, int]):
        for name, val in words.items():
            await self.nodes[name].write_value(ua.DataValue(ua.Variant(int(val), ua.VariantType.Int16)))
