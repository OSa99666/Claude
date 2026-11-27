"""Stand-in for the PLC: level control through the OPC UA IO (T_FCV10, T_LT10, T_LSH10, T_LSL10).

  --mode onoff   on/off controller on the two binary level switches (the first exercise):
                 valve open (--open-pct, default 60 %) once the low switch is reached, closed once the high switch is reached,
                 otherwise the last command is kept (the switches are the two thresholds of a hysteresis)
  --mode pi      PI controller on the analogue level T_LT10 to --setpoint (%)
"""
from __future__ import annotations

import argparse
import asyncio
import logging

from asyncua import Client, ua

from . import config as cfgmod

log = logging.getLogger("mock_plc")


class OnOff:
    """State machine on the binary level switches (also used by the tests)."""

    def __init__(self, open_first=True):
        self.open = open_first

    def step(self, lsh: bool, lsl: bool, lsl_inverted=False) -> bool:
        low_reached = (not lsl) if lsl_inverted else lsl
        if lsh:
            self.open = False
        elif low_reached:
            self.open = True
        return self.open


class PI:
    def __init__(self, kp=2.0, ti=60.0, bias=30.0, dt=0.02):
        self.kp, self.ti, self.bias, self.dt = kp, ti, bias, dt
        self.i = 0.0

    def step(self, sp: float, pv: float) -> float:
        e = sp - pv
        u = self.bias + self.kp * (e + self.i)
        if 0.0 <= u <= 100.0 or (u > 100.0 and e < 0) or (u < 0.0 and e > 0):   # anti-windup
            self.i += e * self.dt / self.ti
        return min(100.0, max(0.0, u))


async def run(url: str, namespace: str, mode: str, setpoint: float, raw_full: float, open_pct: float = 60.0, duration: float | None = None):
    async with Client(url) as c:
        idx = await c.get_namespace_index(namespace)
        N = lambda n: c.get_node(ua.NodeId(n, idx))
        nodes = {n: N(n) for n in ("T_FCV10", "T_LT10", "T_LSH10", "T_LSL10")}
        onoff, pi = OnOff(), PI()
        loop = asyncio.get_running_loop()
        t0 = last = loop.time()
        while duration is None or loop.time() - t0 < duration:
            if mode == "onoff":
                lsh = bool(await nodes["T_LSH10"].read_value())
                lsl = bool(await nodes["T_LSL10"].read_value())
                u = open_pct if onoff.step(lsh, lsl) else 0.0
            else:
                pv = int(await nodes["T_LT10"].read_value()) * 100.0 / raw_full
                u = pi.step(setpoint, pv)
            await nodes["T_FCV10"].write_value(ua.DataValue(ua.Variant(int(round(u * raw_full / 100.0)), ua.VariantType.Int16)))
            if loop.time() - last > 5:
                last = loop.time()
                log.info("%s: valve %.0f %%", mode, u)
            await asyncio.sleep(0.02)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--url", default=None)
    ap.add_argument("--mode", choices=("onoff", "pi"), default="onoff")
    ap.add_argument("--setpoint", type=float, default=50.0, help="level setpoint in %% (pi mode)")
    ap.add_argument("--open-pct", type=float, default=60.0, help="valve opening when filling (onoff mode)")
    a = ap.parse_args()
    logging.basicConfig(level=logging.INFO, format="%(asctime)s %(name)s %(levelname)s %(message)s")
    logging.getLogger("asyncua").setLevel(logging.WARNING)
    cfg = cfgmod.load()
    url = a.url or "opc.tcp://127.0.0.1:%d" % int(cfg["opcua"]["endpoint"].rsplit(":", 1)[1])
    try:
        asyncio.run(run(url, cfg["opcua"]["namespace_uri"], a.mode, a.setpoint, float(cfg["scaling"]["raw_full"]), a.open_pct))
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()
