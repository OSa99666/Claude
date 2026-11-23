"""Valve simulator main loop: valve model + OPC UA server + Godot UI link.

    PLC (OPC UA client) --V_YH/V_YL--> [this process] --V_XGH/V_XGL--> PLC
                                          ^   |
                  UDP commands (port 9873) |   v UDP state (port 9872)
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
from .opcua_server import SimServer
from .ui_link import UiLink
from .valve import Valve, ValveParams

log = logging.getLogger("sim")


def _num(d: dict, key: str, default: float) -> float:
    try:
        v = float(d.get(key, default))
    except (TypeError, ValueError):
        return default
    return v if math.isfinite(v) else default


class Simulator:
    def __init__(self, cfg: dict):
        self.cfg = cfg
        v = cfg["valve"]
        self.valve = Valve(ValveParams(
            t_to_high=v["transit_to_high_s"], t_to_low=v["transit_to_low_s"],
            ad_from_low=v["action_delay_from_low_s"], ad_from_high=v["action_delay_from_high_s"],
            travel_to_end=v.get("travel_to_end", True)), high=v.get("start_high", False))
        self.server = SimServer(cfg)
        self.ui: UiLink | None = None
        self.cycle = cfg["sim"]["cycle_ms"] / 1000.0
        self.t = 0.0
        self.clients = 0
        self.cmd = (False, False)
        self.udp = None
        vis = cfg["visual"]
        if vis.get("enabled", True):
            self.udp = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            self.udp_addr = (vis["host"], vis["port"])

    def apply_ui(self):
        u = self.ui.latest if self.ui else {}
        p = self.valve.p
        if u:
            p.t_to_high = max(0.05, _num(u, "t_to_high", p.t_to_high))
            p.t_to_low = max(0.05, _num(u, "t_to_low", p.t_to_low))
            p.ad_from_low = max(0.0, _num(u, "ad_from_low", p.ad_from_low))
            p.ad_from_high = max(0.0, _num(u, "ad_from_high", p.ad_from_high))
            p.travel_to_end = bool(u.get("travel_to_end", p.travel_to_end))
            p.safeguard_high = bool(u.get("safeguard_high", p.safeguard_high))
            p.safeguard_low = bool(u.get("safeguard_low", p.safeguard_low))
            p.force_true = bool(u.get("force_true", p.force_true))
            p.force_false = bool(u.get("force_false", p.force_false))
        if self.ui:
            if self.ui.take("to_low_seq"):
                self.valve.set_position(False)
                log.info("valve set to Low")
            if self.ui.take("to_high_seq"):
                self.valve.set_position(True)
                log.info("valve set to High")

    def send_visual(self, xgh: bool, xgl: bool):
        if not self.udp:
            return
        p = self.valve.p
        msg = {"t": round(self.t, 3), "pos": self.valve.pos, "xgh": xgh, "xgl": xgl,
               "yh": self.cmd[0], "yl": self.cmd[1], "want": self.valve.want,
               "waiting": self.valve.delay > 0 and (self.valve.at_low or self.valve.at_high),
               "plc": self.clients > 0, "clients": self.clients,
               "params": {"t_to_high": p.t_to_high, "t_to_low": p.t_to_low,
                          "ad_from_low": p.ad_from_low, "ad_from_high": p.ad_from_high,
                          "travel_to_end": p.travel_to_end,
                          "safeguard_high": p.safeguard_high, "safeguard_low": p.safeguard_low,
                          "force_true": p.force_true, "force_false": p.force_false}}
        try:
            self.udp.sendto(json.dumps(msg).encode(), self.udp_addr)
        except OSError:
            pass

    async def step_cycle(self):
        self.apply_ui()
        self.clients = self.server.client_count()
        # nobody connected: no command is applied (bistable valve keeps its position)
        self.cmd = await self.server.read_commands() if self.clients > 0 else (False, False)
        self.valve.step(self.cycle, *self.cmd)
        self.t += self.cycle
        xgh, xgl = self.valve.feedbacks()
        await self.server.write_feedbacks({"V_XGH": xgh, "V_XGL": xgl})
        self.send_visual(xgh, xgl)

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
                    xgh, xgl = self.valve.feedbacks()
                    log.info("t=%6.1fs pos=%.2f XGH=%d XGL=%d YH=%d YL=%d clients=%d", self.t,
                             self.valve.pos, xgh, xgl, *map(int, self.cmd), self.clients)
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
    ap.add_argument("--endpoint", help="override opcua.endpoint, e.g. opc.tcp://0.0.0.0:4841")
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
