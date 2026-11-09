"""Stand-in for the CODESYS runtime, for testing the simulator without hardware.

Exposes the same GVL_Sim variables (same node-id scheme) as an OPC UA server
and runs the same cascaded PD controller as codesys/FB_BallCtrl.st.
"""
from __future__ import annotations

import argparse
import asyncio
import logging
import math
import time

from asyncua import Server, ua

from . import config as cfgmod

log = logging.getLogger("mock_plc")

F, B, I32 = ua.VariantType.Float, ua.VariantType.Boolean, ua.VariantType.Int32
VARS = {  # name: (type, initial)
    "Ball_X": (F, 0.0), "Ball_Y": (F, 0.0), "TiltX_PV": (F, 0.0), "TiltY_PV": (F, 0.0),
    "Heartbeat": (I32, 0),
    "TiltX_SP": (F, 0.0), "TiltY_SP": (F, 0.0), "Cmd_Reset": (B, False),
    "Reset_X": (F, 0.10), "Reset_Y": (F, -0.08),
    "Mu_S": (F, 0.35), "Mu_K": (F, 0.25), "E_Wall": (F, 0.6), "Mu_Wall": (F, 0.2),
    "Noise_mm": (F, 0.3), "Delay_ms": (F, 20.0),
}


class CascadePD:
    """theta_cmd = (wn^2 (sp - x) - 2 zeta wn (v_hat - sp_vel)) / k   (k = 5/7 g).

    Plant is a double integrator x'' = k*theta, so this places the closed-loop
    poles at natural frequency wn / damping zeta.  v_hat is a filtered
    derivative of the (noisy, delayed) position; sp_vel is optional velocity
    feedforward for moving setpoints."""

    def __init__(self, wn=2.5, zeta=0.9, vtau=0.06, ki=0.0, max_deg=10.0):
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
    o = cfg["opcua"]
    server = Server()
    await server.init()
    server.set_endpoint(a.endpoint)
    while True:   # make namespace index match the config (CODESYS uses 4)
        idx = await server.register_namespace(f"urn:mock:{time.time_ns()}")
        if idx >= o["namespace"]:
            break
    nodes = {}
    for name, (vt, init) in VARS.items():
        nid = ua.NodeId(f"|var|{o['device']}.Application.GVL_Sim.{name}", o["namespace"],
                        ua.NodeIdType.String)
        n = await server.nodes.objects.add_variable(nid, name, init, varianttype=vt)
        await n.set_writable()
        nodes[name] = n
    ctl = {"x": CascadePD(ki=0.15), "y": CascadePD(ki=0.15)}
    dt = 0.01
    t0 = time.monotonic()
    async with server:
        log.info("mock PLC listening on %s (ns=%d)", a.endpoint, o["namespace"])
        # release the ball once a client is attached
        nxt = time.monotonic()
        last_hb, last_hb_t = -1, time.monotonic()
        released = False
        while True:
            t = time.monotonic() - t0
            x = await nodes["Ball_X"].read_value()
            y = await nodes["Ball_Y"].read_value()
            hb = await nodes["Heartbeat"].read_value()
            if hb != last_hb:
                last_hb, last_hb_t = hb, time.monotonic()
                if not released:
                    released = True
                    await nodes["Cmd_Reset"].write_value(ua.DataValue(ua.Variant(True, B)))
            elif time.monotonic() - last_hb_t > 0.5:
                released = False
            svx = svy = 0.0
            if a.pattern == "circle":
                r, w = 0.10, 2 * math.pi / 12
                spx, spy = r * math.cos(w * t), r * math.sin(w * t)
                svx, svy = -r * w * math.sin(w * t), r * w * math.cos(w * t)
            elif a.pattern == "square":
                k = int(t // 6) % 4
                spx, spy = [(0.1, 0.1), (-0.1, 0.1), (-0.1, -0.1), (0.1, -0.1)][k]
                svx = svy = 0.0
            else:
                spx = spy = svx = svy = 0.0
            tx = ctl["x"].update(spx, x, dt, svx)
            ty = ctl["y"].update(spy, y, dt, svy)
            await nodes["TiltX_SP"].write_value(ua.DataValue(ua.Variant(tx, F)))
            await nodes["TiltY_SP"].write_value(ua.DataValue(ua.Variant(ty, F)))
            if await nodes["Cmd_Reset"].read_value():
                await asyncio.sleep(0.05)
                await nodes["Cmd_Reset"].write_value(ua.DataValue(ua.Variant(False, B)))
            nxt += dt
            await asyncio.sleep(max(0.0, nxt - time.monotonic()))


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("-c", "--config", default=None)
    ap.add_argument("--endpoint", default="opc.tcp://0.0.0.0:4840")
    ap.add_argument("--pattern", choices=["center", "circle", "square"], default="center")
    a = ap.parse_args()
    logging.basicConfig(level=logging.INFO)
    try:
        asyncio.run(main_async(a))
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()
