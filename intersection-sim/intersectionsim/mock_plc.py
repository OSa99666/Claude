"""Stand-in for the PLC: a fixed-time two-phase signal controller with pedestrian requests.

Cycle: direction 1 green -> yellow -> all red -> direction 2 green -> yellow -> all red.  A pedestrian push button
(I_XPD1/I_XPD2) is latched and acknowledged on I_YPD1A/I_YPD2A; the pedestrian light of a group goes green for the first
``ped_green`` seconds of the vehicle green phase that does not conflict (group 1 walks with direction 1, group 2 with direction 2).
"""
from __future__ import annotations

import argparse
import asyncio
import logging

from asyncua import Client, ua
from plchub import config as hubcfg, mock_support

from . import config as cfgmod
from .opcua_server import PLC_TO_SIM, SIM_TO_PLC

mock_support.tolerate_bad_quality()      # a tag of a simulation that is not running reads 0 instead of raising
log = logging.getLogger("mock_plc")


class FixedTime:
    # (duration, active direction or 0, kind)
    def __init__(self, green=10.0, yellow=3.0, all_red=2.0, ped_green=9.0):
        self.phases = [(green, 1, "G"), (yellow, 1, "Y"), (all_red, 0, "R"),
                       (green, 2, "G"), (yellow, 2, "Y"), (all_red, 0, "R")]
        self.ped_green = ped_green
        self.i = 0
        self.t = 0.0
        self.req = {1: False, 2: False}
        self.ped_allowed = {1: False, 2: False}
        self._btn_prev = {1: False, 2: False}

    def step(self, dt: float, inp: dict) -> dict:
        """inp: sim -> PLC values by internal name ('PD1', 'PD2', ...).  Returns the PLC outputs by internal name."""
        for g in (1, 2):
            if inp.get("PD%d" % g, False) and not self._btn_prev[g]:
                self.req[g] = True
            self._btn_prev[g] = bool(inp.get("PD%d" % g, False))
        dur, d, kind = self.phases[self.i]
        if self.t == 0.0 and kind == "G":                     # phase start: decide the pedestrian light of this group
            self.ped_allowed[d] = self.req[d]
            if self.req[d]:
                self.req[d] = False
        self.t += dt
        out = {n: False for n in PLC_TO_SIM.values()}
        out["TL1R"] = d != 1
        out["TL2R"] = d != 2
        if d:
            out["TL%dR" % d] = False
            out["TL%d%s" % (d, "Y" if kind == "Y" else "G")] = True
        for g in (1, 2):
            walk = d == g and kind == "G" and self.ped_allowed[g] and self.t < self.ped_green
            out["PL%dG" % g] = walk
            out["PL%dR" % g] = not walk
            out["PD%dA" % g] = self.req[g]
        if self.t >= dur:
            self.t = 0.0
            self.i = (self.i + 1) % len(self.phases)
            if self.phases[self.i][2] == "G":
                self.t = 0.0
        return out


async def run(url: str, namespace: str, green: float, duration: float | None = None):
    async with Client(url) as c:
        idx = await c.get_namespace_index(namespace)
        N = lambda n: c.get_node(ua.NodeId(n, idx))
        ctrl = FixedTime(green=green)
        loop = asyncio.get_running_loop()
        t0 = last = loop.time()
        prev = {}
        while duration is None or loop.time() - t0 < duration:
            inp = {short: bool(await N(tag).read_value()) for tag, short in SIM_TO_PLC.items() if short in ("PD1", "PD2")}
            out = ctrl.step(0.02, inp)
            for tag, short in PLC_TO_SIM.items():
                if prev.get(tag) is not out[short]:
                    await N(tag).write_value(ua.DataValue(ua.Variant(bool(out[short]), ua.VariantType.Boolean)))
                    prev[tag] = out[short]
            if loop.time() - last > 5:
                last = loop.time()
                log.info("phase %d (%s)", ctrl.i, ctrl.phases[ctrl.i][2])
            await asyncio.sleep(0.02)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--url", default=None)
    ap.add_argument("--green", type=float, default=10.0, help="vehicle green time per direction in s")
    a = ap.parse_args()
    logging.basicConfig(level=logging.INFO, format="%(asctime)s %(name)s %(levelname)s %(message)s")
    logging.getLogger("asyncua").setLevel(logging.WARNING)
    cfg = cfgmod.load()
    url0, ns_uri = hubcfg.plc_target(cfg)
    url = a.url or url0
    try:
        asyncio.run(run(url, ns_uri, a.green))
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()
