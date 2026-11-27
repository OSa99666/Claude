"""Jacketed tank simulator main loop: state-space model + OPC UA IO + Godot UI link.

    PLC (OPC UA client) --M_YR01, M_YR02--> [this process] --M_XG01, M_XG02--> PLC
                                               ^   |
                       UDP commands (9881)      |   v UDP state (9880)
                                               Godot UI / 3D view
"""
from __future__ import annotations

import argparse
import asyncio
import json
import logging
import math
import socket
import time

from . import config as cfgmod
from .model import JacketTank, Params, X_INIT
from .opcua_server import SimServer
from .ui_link import UiLink

log = logging.getLogger("sim")

FLOAT_KEYS = {"sample_time_s": (0.05, 60.0), "noise_temp": (0.0, 20.0), "noise_level": (0.0, 0.5),
              "bias_temp": (-50.0, 50.0), "bias_level": (-0.5, 0.5), "heater_max": (0.0, 100.0),
              "b_scale_heater": (0.0, 5.0), "b_scale_inflow": (0.0, 5.0), "a_scale": (0.0, 3.0)}


def _num(d: dict, key: str, default: float) -> float:
    try:
        v = float(d.get(key, default))
    except (TypeError, ValueError):
        return default
    return v if math.isfinite(v) else default


class Simulator:
    def __init__(self, cfg: dict):
        self.cfg = cfg
        seed = int(cfg["sim"].get("seed", 0)) or None
        self.model = JacketTank(Params(sample_time_s=cfg["model"]["sample_time_s"]), seed=seed)
        self.server = SimServer(cfg)
        self.ui: UiLink | None = None
        self.cycle = cfg["sim"]["cycle_ms"] / 1000.0
        self.t = 0.0
        self.clients = 0
        self.raw_in = (0, 0)
        self.manual_on = False
        self.manual_heater = 0.0
        self.manual_inflow = 0.0
        self.d = [0.0, 0.0]
        self.udp = None
        vis = cfg["visual"]
        if vis.get("enabled", True):
            self.udp = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            self.udp_addr = (vis["host"], vis["port"])

    def apply_ui(self):
        u = self.ui.latest if self.ui else {}
        p = self.model.p
        if u:
            for k, (lo, hi) in FLOAT_KEYS.items():
                setattr(p, k, min(hi, max(lo, _num(u, k, getattr(p, k)))))
            p.dead_time_samples = int(min(60, max(0, _num(u, "dead_time_samples", p.dead_time_samples))))
            p.freeze_sensors = bool(u.get("freeze_sensors", p.freeze_sensors))
            p.hold = bool(u.get("hold", p.hold))
            self.d = [min(200.0, max(-200.0, _num(u, "d1", self.d[0]))), min(20.0, max(-20.0, _num(u, "d2", self.d[1])))]
            self.manual_on = bool(u.get("manual_on", self.manual_on))
            self.manual_heater = min(100.0, max(0.0, _num(u, "manual_heater", self.manual_heater)))
            self.manual_inflow = min(1.0, max(0.0, _num(u, "manual_inflow", self.manual_inflow)))
        if self.ui:
            if self.ui.take("reset_seq"):
                self.model.reset()
                log.info("states reset to the initial state %s", X_INIT)
            if self.ui.take("set_state_seq"):
                self.model.set_state(_num(u, "set_temp", 70.0), _num(u, "set_level", 0.5))
                log.info("state set: temperature %.1f, level %.2f", self.model.x[0], self.model.x[2])
            if self.ui.take("pulse_seq"):
                self.model.pulse_d2(_num(u, "pulse_amount", 1.0), max(0.1, _num(u, "pulse_s", 10.0)))

    def send_visual(self):
        if not self.udp:
            return
        m = self.model
        p = m.p
        params = {k: getattr(p, k) for k in FLOAT_KEYS}
        params.update(dead_time_samples=p.dead_time_samples, freeze_sensors=p.freeze_sensors, hold=p.hold, d1=self.d[0], d2=self.d[1],
                      manual_on=self.manual_on, manual_heater=self.manual_heater, manual_inflow=self.manual_inflow)
        msg = {"t": round(self.t, 3), "x": m.x, "y": m.y_true, "ym": m.y_meas, "raw_out": m.raw, "raw_in": list(self.raw_in),
               "u": m.u, "d": m.d, "samples": m.samples, "plc": self.clients > 0, "clients": self.clients,
               "manual": self.manual_on, "params": params}
        try:
            self.udp.sendto(json.dumps(msg).encode(), self.udp_addr)
        except OSError:
            pass

    async def step_cycle(self):
        self.apply_ui()
        self.clients = self.server.client_count()
        # nobody connected: both controls 0 (heater off, no inflow), unless the manual override is on
        self.raw_in = await self.server.read_controls() if self.clients > 0 else (0, 0)
        if self.manual_on:
            r1, r2 = self.manual_heater / 100.0 * 32767.0, self.manual_inflow * 32767.0
        else:
            r1, r2 = self.raw_in
        self.model.step(self.cycle, r1, r2, self.d[0], self.d[1])
        self.t += self.cycle
        await self.server.write_outputs(*self.model.raw)
        self.send_visual()

    async def run(self):
        await self.server.start()
        self.ui = await UiLink.start(self.cfg["visual"]["command_port"])
        next_t = time.monotonic()
        last_log = 0.0
        try:
            while True:
                await self.step_cycle()
                now = time.monotonic()
                if now - last_log > 2.0:
                    m = self.model
                    log.info("t=%6.1fs T=%5.1f degC  level=%.3f m  heater=%5.1f %%  inflow=%.3f  raw in=%s out=%s  clients=%d%s", self.t,
                             m.y_true[0], m.y_true[1], m.u[0], m.u[1], self.raw_in, m.raw, self.clients, " (manual)" if self.manual_on else "")
                    last_log = now
                next_t += self.cycle
                delay = next_t - time.monotonic()
                if delay < -5 * self.cycle:
                    next_t, delay = time.monotonic(), 0
                await asyncio.sleep(max(0.0, delay))
        finally:
            await self.server.stop()


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawTextHelpFormatter)
    ap.add_argument("-c", "--config", default=None)
    ap.add_argument("--endpoint", help="standalone mode: override opcua.endpoint")
    ap.add_argument("--publish-cap-ms", type=float, default=None)
    ap.add_argument("-v", "--verbose", action="store_true")
    a = ap.parse_args()
    logging.basicConfig(level=logging.DEBUG if a.verbose else logging.INFO,
                        format="%(asctime)s %(name)s %(levelname)s %(message)s")
    logging.getLogger("asyncua").setLevel(logging.INFO if a.verbose else logging.WARNING)
    cfg = cfgmod.load(a.config)
    if a.endpoint:
        cfg["opcua"]["endpoint"] = a.endpoint
    if a.publish_cap_ms is not None:
        cfg["opcua"]["publishing_interval_cap_ms"] = a.publish_cap_ms
    try:
        asyncio.run(Simulator(cfg).run())
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()
