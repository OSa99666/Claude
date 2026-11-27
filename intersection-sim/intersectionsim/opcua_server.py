"""OPC UA server exposing the intersection IO (all Bool, folder ``Intersection``, string node ids = the PLC tag names).

PLC -> simulator (12):  I_YTL1R/Y/G, I_YTL2R/Y/G   traffic lights direction 1 / 2 (red, yellow, green)
                        I_YPL1R/G, I_YPL2R/G       pedestrian lights direction 1 / 2 (red, green)
                        I_YPD1A, I_YPD2A           pedestrian request acknowledged (lamp at the push button)
simulator -> PLC (17):  I_XTL1R/Y/G, I_XTL2R/Y/G   traffic light feedback (lamp really lit; a broken lamp reads false)
                        I_XPL1R/G, I_XPL2R/G       pedestrian light feedback
                        I_XTD11, I_XTD12           vehicle detectors direction 1 (arm W, arm E)
                        I_XTD21, I_XTD22           vehicle detectors direction 2 (arm N, arm S)
                        I_XPD1, I_XPD2             pedestrian push buttons
                        I_XF                       intersection fault (collision, latched)
Internal short names (used by the model) are the tag without the ``I_Y`` / ``I_X`` prefix and the lamp/detector part:
``TL1R`` ... ``PL2G``, ``PD1A``/``PD2A`` (outputs), ``XTD11`` ... ``PD1``/``PD2``, ``F`` (inputs).
"""
from __future__ import annotations

import logging

from asyncua import Server, ua

from . import subscription_tweaks
from .model import DETECTORS, LAMPS

log = logging.getLogger("opcua")

FOLDER = "Intersection"
# tag -> internal name
PLC_TO_SIM = {"I_Y" + n: n for n in LAMPS} | {"I_YPD1A": "PD1A", "I_YPD2A": "PD2A"}
SIM_TO_PLC = {"I_X" + n: n for n in LAMPS} | {"I_" + n: n for n in DETECTORS} | {"I_XPD1": "PD1", "I_XPD2": "PD2", "I_XF": "F"}
DESCR = {
    "TL": "traffic light", "PL": "pedestrian light", "PD": "pedestrian detector/button", "TD": "traffic detector", "F": "fault",
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
        self.server: Server | None = None
        self.nodes: dict[str, object] = {}
        self._last: dict[str, bool] = {}

    async def start(self):
        subscription_tweaks.apply(self.cap_ms)
        s = Server(user_manager=_Users(*self._auth)) if self._auth else Server()
        await s.init()
        s.set_endpoint(self.endpoint)
        s.set_server_name("Intersection simulator")
        idx = await s.register_namespace(self.uri)
        folder = await s.nodes.objects.add_folder(idx, FOLDER)
        for name in list(PLC_TO_SIM) + list(SIM_TO_PLC):
            n = await folder.add_variable(ua.NodeId(name, idx), name, False, varianttype=ua.VariantType.Boolean)
            if name in PLC_TO_SIM:
                await n.set_writable()
            self.nodes[name] = n
        await s.start()
        self.server = s
        self.ns_index = idx
        log.info("OPC UA server on %s, namespace %d (%s), %d variables", self.endpoint, idx, self.uri, len(self.nodes))

    async def stop(self):
        if self.server:
            await self.server.stop()

    def client_count(self) -> int:
        b = self.server.bserver if self.server else None
        return len(b.clients) if b else 0

    async def read_commands(self) -> dict[str, bool]:
        return {short: bool(await self.nodes[tag].read_value()) for tag, short in PLC_TO_SIM.items()}

    async def write_inputs(self, vals: dict[str, bool]):
        """vals by internal name; only changed values are written."""
        for tag, short in SIM_TO_PLC.items():
            v = bool(vals.get(short, False))
            if self._last.get(tag) is not v:
                await self.nodes[tag].write_value(ua.DataValue(ua.Variant(v, ua.VariantType.Boolean)))
                self._last[tag] = v
