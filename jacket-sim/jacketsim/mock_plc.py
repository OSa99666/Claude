"""Stand-in for the PLC: two PI loops through the OPC UA IO (NOT an MPC - the real MPC runs in the PLC).

  heater  M_YR01 <- PI on the temperature M_XG01      inflow  M_YR02 <- PI on the level M_XG02

Options: --temp-sp / --level-sp setpoints (degC / m), --steps N toggles both setpoints every N seconds (a visible demo).
"""
from __future__ import annotations

import argparse
import asyncio
import logging

from asyncua import Client, ua
from plchub import config as hubcfg, mock_support

from . import config as cfgmod
from .model import RAW_HIGH, Y_HIGH, Y_LOW

mock_support.tolerate_bad_quality()      # a tag of a simulation that is not running reads 0 instead of raising

log = logging.getLogger("mock_plc")


class PI:
    """Velocity-free PI with anti-windup: u = bias + kp * (e + integral(e) / ti), limited to lo..hi."""

    def __init__(self, kp, ti, bias, lo, hi):
        self.kp, self.ti, self.bias, self.lo, self.hi = kp, ti, bias, lo, hi
        self.i = 0.0

    def step(self, e: float, dt: float) -> float:
        u = self.bias + self.kp * (e + self.i)
        if self.lo <= u <= self.hi or (u > self.hi and e < 0) or (u < self.lo and e > 0):
            self.i += e * dt / self.ti
        return min(self.hi, max(self.lo, u))


def raw_to_eng(raw: float, i: int) -> float:
    return Y_LOW[i] + (Y_HIGH[i] - Y_LOW[i]) * raw / RAW_HIGH


class Controller:
    """The two loops (also used by the tests): temperature -> heater %, level -> inflow 0..1."""

    def __init__(self, temp_sp=70.0, level_sp=0.5):
        self.temp_sp, self.level_sp = temp_sp, level_sp
        self.heater = PI(1.5, 15.0, 26.0, 0.0, 100.0)
        self.inflow = PI(2.0, 12.0, 0.25, 0.0, 1.0)

    def step(self, raw_temp: float, raw_level: float, dt: float) -> tuple[int, int]:
        u1 = self.heater.step(self.temp_sp - raw_to_eng(raw_temp, 0), dt)
        u2 = self.inflow.step(self.level_sp - raw_to_eng(raw_level, 1), dt)
        return int(round(u1 / 100.0 * RAW_HIGH)), int(round(u2 * RAW_HIGH))


async def run(url: str, namespace: str, temp_sp: float, level_sp: float, steps_s: float = 0.0, duration: float | None = None):
    async with Client(url) as c:
        idx = await c.get_namespace_index(namespace)
        N = lambda n: c.get_node(ua.NodeId(n, idx))
        ctl = Controller(temp_sp, level_sp)
        loop = asyncio.get_running_loop()
        t0 = last = last_step = loop.time()
        dt = 0.1
        flip = False
        while duration is None or loop.time() - t0 < duration:
            raw_t = float(await N("M_XG01").read_value())
            raw_l = float(await N("M_XG02").read_value())
            r1, r2 = ctl.step(raw_t, raw_l, dt)
            await N("M_YR01").write_value(ua.DataValue(ua.Variant(r1, ua.VariantType.Int16)))
            await N("M_YR02").write_value(ua.DataValue(ua.Variant(r2, ua.VariantType.Int16)))
            if steps_s and loop.time() - last_step > steps_s:
                last_step = loop.time()
                flip = not flip
                ctl.temp_sp, ctl.level_sp = (temp_sp + 10.0, level_sp + 0.1) if flip else (temp_sp, level_sp)
                log.info("setpoints: %.1f degC, %.2f m", ctl.temp_sp, ctl.level_sp)
            if loop.time() - last > 5:
                last = loop.time()
                log.info("T %.1f (sp %.1f)  level %.3f (sp %.2f)  heater %.0f %%  inflow %.2f", raw_to_eng(raw_t, 0), ctl.temp_sp,
                         raw_to_eng(raw_l, 1), ctl.level_sp, r1 / RAW_HIGH * 100, r2 / RAW_HIGH)
            await asyncio.sleep(dt)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--url", default=None)
    ap.add_argument("--temp-sp", type=float, default=70.0, help="temperature setpoint in degC")
    ap.add_argument("--level-sp", type=float, default=0.5, help="level setpoint in m")
    ap.add_argument("--steps", type=float, default=0.0, help="toggle the setpoints (+10 degC / +0.1 m) every N seconds (0 = off)")
    a = ap.parse_args()
    logging.basicConfig(level=logging.INFO, format="%(asctime)s %(name)s %(levelname)s %(message)s")
    logging.getLogger("asyncua").setLevel(logging.WARNING)
    cfg = cfgmod.load()
    url0, ns_uri = hubcfg.plc_target(cfg)
    try:
        asyncio.run(run(a.url or url0, ns_uri, a.temp_sp, a.level_sp, a.steps))
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()
