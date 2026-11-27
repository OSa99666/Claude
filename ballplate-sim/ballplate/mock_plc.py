"""Stand-in for the PLC, for testing the simulator without hardware.

Connects as an OPC UA *client* to the simulator, reads the INT words
F_XG1x/F_XG2x, runs a cascaded PD controller and writes F_YR10/F_YR20 -
exactly what the real PLC does through the CODESYS Data Source Manager.
"""
from __future__ import annotations

import argparse
import asyncio
import logging
import math
import time

from asyncua import Client, ua
from plchub import config as hubcfg, mock_support

from . import config as cfgmod
from .scaling import from_raw, to_raw

mock_support.tolerate_bad_quality()      # a tag of a simulation that is not running reads 0 instead of raising
log = logging.getLogger("mock_plc")


class CascadePD:
    """theta_cmd = (wn^2 (sp - x) - 2 zeta wn (v_hat - sp_vel)) / k   (k = 5/7 g).

    Plant is a double integrator x'' = k*theta, so this places the closed-loop
    poles at natural frequency wn / damping zeta.  v_hat is a filtered
    derivative of the measured position; sp_vel is optional velocity
    feedforward for moving setpoints."""

    def __init__(self, wn=1.5, zeta=0.9, vtau=0.06, ki=0.0, max_deg=10.0):
        self.wn, self.zeta, self.vtau, self.ki, self.max_deg = wn, zeta, vtau, ki, max_deg
        self.xprev = None
        self.v = 0.0
        self.integ = 0.0

    def update(self, sp, x, dt, sp_vel=0.0):
        if self.xprev is None:
            self.xprev = x
        raw = (x - self.xprev) / dt
        self.xprev = x
        a = dt / (self.vtau + dt)
        self.v += a * (raw - self.v)
        e = sp - x
        k = 5.0 / 7.0 * 9.81
        cmd = (self.wn ** 2 * e - 2 * self.zeta * self.wn * (self.v - sp_vel)) / k   # rad
        if self.ki:
            self.integ = max(-0.05, min(0.05, self.integ + self.ki * e * dt))
            cmd += self.integ
        return max(-self.max_deg, min(self.max_deg, math.degrees(cmd)))


async def main_async(a):
    cfg = cfgmod.load(a.config)
    sc = cfg["scaling"]
    rm, d_m, tilt = sc["raw_max"], sc["dist_m"], sc["tilt_deg"]
    url0, ns_uri = hubcfg.plc_target(cfg)
    url = a.url or url0
    ctl = {"x": CascadePD(ki=0.15), "y": CascadePD(ki=0.15)}
    dt = 0.01
    async with Client(url) as c:
        idx = await c.get_namespace_index(ns_uri)
        def node(name):
            return c.get_node(ua.NodeId(name, idx))
        n_tx, n_ty = node("F_YR10"), node("F_YR20")
        n_dx, n_dy = node("F_XG12"), node("F_XG22")
        log.info("connected to %s, running pattern '%s'", url, a.pattern)
        t0 = time.monotonic()
        nxt = t0
        while True:
            t = time.monotonic() - t0
            rx, ry = await c.read_values([n_dx, n_dy])
            x, y = from_raw(rx, -d_m, d_m, rm), from_raw(ry, -d_m, d_m, rm)
            svx = svy = 0.0
            if a.pattern == "circle":
                r, w = 0.5, 2 * math.pi / 24
                spx, spy = r * math.cos(w * t), r * math.sin(w * t)
                svx, svy = -r * w * math.sin(w * t), r * w * math.cos(w * t)
            elif a.pattern == "square":
                k = int(t // 10) % 4
                spx, spy = [(0.5, 0.5), (-0.5, 0.5), (-0.5, -0.5), (0.5, -0.5)][k]
            else:
                spx = spy = 0.0
            tx = ctl["x"].update(spx, x, dt, svx)
            ty = ctl["y"].update(spy, y, dt, svy)
            await c.write_values([n_tx, n_ty], [
                ua.DataValue(ua.Variant(to_raw(tx, -tilt, tilt, rm), ua.VariantType.Int16)),
                ua.DataValue(ua.Variant(to_raw(ty, -tilt, tilt, rm), ua.VariantType.Int16))])
            nxt += dt
            await asyncio.sleep(max(0.0, nxt - time.monotonic()))


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("-c", "--config", default=None)
    ap.add_argument("--url", help="simulator endpoint (default opc.tcp://127.0.0.1:4840)")
    ap.add_argument("--pattern", choices=["center", "circle", "square"], default="center")
    a = ap.parse_args()
    logging.basicConfig(level=logging.INFO)
    logging.getLogger("asyncua").setLevel(logging.WARNING)
    try:
        asyncio.run(main_async(a))
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()
