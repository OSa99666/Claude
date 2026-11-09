"""OPC UA client side: talks to the CODESYS runtime."""
from __future__ import annotations

import logging
from asyncua import Client, ua

from .config import node_id

log = logging.getLogger("plcio")

# name -> default (used until the PLC delivers a value)
FAST_IN = {"TiltX_SP": 0.0, "TiltY_SP": 0.0, "Cmd_Reset": False}
PARAM_IN = {"Reset_X": 0.0, "Reset_Y": 0.0, "Mu_S": None, "Mu_K": None,
            "E_Wall": None, "Mu_Wall": None, "Noise_mm": None, "Delay_ms": None}
OUT = ["Ball_X", "Ball_Y", "TiltX_PV", "TiltY_PV", "Heartbeat"]


def _coerce(value, vt: ua.VariantType):
    if vt == ua.VariantType.Boolean:
        return bool(value)
    if vt in (ua.VariantType.Float, ua.VariantType.Double):
        return float(value)
    return int(value)


class PlcLink:
    def __init__(self, cfg: dict):
        self.cfg = cfg
        o = cfg["opcua"]
        self.client = Client(o["url"], timeout=o.get("timeout_s", 2.0))
        if o.get("username"):
            self.client.set_user(o["username"])
            self.client.set_password(o.get("password", ""))
        self._security = o.get("security") or ""
        self.nodes: dict[str, object] = {}
        self.types: dict[str, ua.VariantType] = {}
        self.connected = False

    async def connect(self):
        if self._security:
            await self.client.set_security_string(self._security)
        await self.client.connect()
        names = list(FAST_IN) + list(PARAM_IN) + OUT
        for n in names:
            node = self.client.get_node(node_id(self.cfg, n))
            self.nodes[n] = node
            # CODESYS rejects writes whose type does not match exactly, so
            # remember what the PLC declared (REAL -> Float, LREAL -> Double...)
            self.types[n] = await node.read_data_type_as_variant_type()
        self.connected = True
        log.info("connected to %s", self.cfg["opcua"]["url"])

    async def disconnect(self):
        self.connected = False
        try:
            await self.client.disconnect()
        except Exception:  # noqa: BLE001 - already dead is fine
            pass

    async def _read(self, names):
        vals = await self.client.read_values([self.nodes[n] for n in names])
        return dict(zip(names, vals))

    async def read_fast(self) -> dict:
        return await self._read(list(FAST_IN))

    async def read_params(self) -> dict:
        return await self._read(list(PARAM_IN))

    async def write_outputs(self, values: dict):
        names = list(values)
        dvs = [ua.DataValue(ua.Variant(_coerce(values[n], self.types[n]), self.types[n]))
               for n in names]
        await self.client.write_values([self.nodes[n] for n in names], dvs)
