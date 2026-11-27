"""Intersection simulator main loop: traffic model + OPC UA server + Godot UI link.

    PLC (OPC UA client) --I_Y*--> [this process] --I_X*--> PLC
                                     ^   |
          UDP commands (port 9879)   |   v UDP state (port 9878)
                                     Godot UI / 2D view
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
from .model import ARMS, DETECTORS, LAMPS, Intersection, Params
from .opcua_server import SimServer
from .ui_link import UiLink

log = logging.getLogger("sim")

BOOL_SETS = {"bk_": ("broken", LAMPS), "dead_": ("det_dead", DETECTORS), "stuck_": ("det_stuck", DETECTORS)}


def _num(d: dict, key: str, default: float) -> float:
    try:
        v = float(d.get(key, default))
    except (TypeError, ValueError):
        return default
    return v if math.isfinite(v) else default


class Simulator:
    def __init__(self, cfg: dict):
        self.cfg = cfg
        t = cfg["traffic"]
        seed = int(cfg["sim"].get("seed", 0)) or None
        self.model = Intersection(Params(
            rate={"W": t["rate_w"], "E": t["rate_e"], "N": t["rate_n"], "S": t["rate_s"]}, v0=t["v0"],
            runner_prob=t["runner_prob"], ped_rate={1: t["ped_rate_1"], 2: t["ped_rate_2"]}, ped_speed=t["ped_speed"]),
            seed=seed)
        self.server = SimServer(cfg)
        self.ui: UiLink | None = None
        self.cycle = cfg["sim"]["cycle_ms"] / 1000.0
        self.t = 0.0
        self.clients = 0
        self.cmd: dict[str, bool] = {}
        self.out: dict[str, bool] = {}
        self.udp = None
        vis = cfg["visual"]
        if vis.get("enabled", True):
            self.udp = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            self.udp_addr = (vis["host"], vis["port"])

    def apply_ui(self):
        u = self.ui.latest if self.ui else {}
        p = self.model.p
        if u:
            for arm in ARMS:
                p.rate[arm] = min(120.0, max(0.0, _num(u, "rate_" + arm.lower(), p.rate[arm])))
            p.v0 = min(30.0, max(2.0, _num(u, "v0", p.v0)))
            p.runner_prob = min(1.0, max(0.0, _num(u, "runner_prob", p.runner_prob * 100.0) / 100.0))
            for g in (1, 2):
                p.ped_rate[g] = min(120.0, max(0.0, _num(u, "ped_rate_%d" % g, p.ped_rate[g])))
            p.ped_speed = min(3.0, max(0.3, _num(u, "ped_speed", p.ped_speed)))
            p.traffic_on = bool(u.get("traffic_on", p.traffic_on))
            p.hw_fault = bool(u.get("hw_fault", p.hw_fault))
            for prefix, (attr, names) in BOOL_SETS.items():
                setattr(p, attr, {n for n in names if u.get(prefix + n, False)})
            p.btn_stuck = {g for g in (1, 2) if u.get("stuck_PD%d" % g, False)}
        if self.ui:
            if self.ui.take("clear_fault_seq"):
                self.model.clear_fault()
                log.info("crash cleared")
            if self.ui.take("reset_stats_seq"):
                self.model.reset_stats()
            if self.ui.take("reseed_seq"):
                self.model.reseed(int(_num(u, "seed", 1)) or None)
                log.info("traffic reseeded")

    def send_visual(self):
        if not self.udp:
            return
        m = self.model
        p = m.p
        params = {"rate_" + a.lower(): p.rate[a] for a in ARMS}
        params.update(v0=p.v0, runner_prob=p.runner_prob * 100.0, ped_rate_1=p.ped_rate[1], ped_rate_2=p.ped_rate[2],
                      ped_speed=p.ped_speed, traffic_on=p.traffic_on, hw_fault=p.hw_fault)
        for prefix, (attr, names) in BOOL_SETS.items():
            for n in names:
                params[prefix + n] = n in getattr(p, attr)
        for g in (1, 2):
            params["stuck_PD%d" % g] = g in p.btn_stuck
        s = m.stats
        npass = max(1, s["cars_passed"])
        npeds = max(1, s["peds_crossed"])
        msg = {"t": round(self.t, 3), "crash": m.crash, "hw_fault": m.hardware_fault(), "plc": self.clients > 0, "clients": self.clients,
               "cmd": self.cmd, "lit": m.lit, "det": m.det, "btn": {"1": m.btn[1], "2": m.btn[2]},
               "cars": [[c.arm, round(c.s, 2), c.crashed, c.cid] for c in m.cars],
               "peds": [[q.cw, round(q.u, 3), q.d, q.state] for q in m.peds],
               "stats": dict(s, car_wait_avg=s["car_wait_total"] / npass, ped_wait_avg=s["ped_wait_total"] / npeds),
               "queue": m.queue_lengths(), "params": params}
        try:
            self.udp.sendto(json.dumps(msg).encode(), self.udp_addr)
        except OSError:
            pass

    async def step_cycle(self):
        self.apply_ui()
        self.clients = self.server.client_count()
        # nobody connected: all lamps dark (drivers treat a dark head as red)
        self.cmd = await self.server.read_commands() if self.clients > 0 else {}
        self.out = self.model.step(self.cycle, self.cmd)
        self.t += self.cycle
        await self.server.write_inputs(self.out)
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
                    lamps = "".join(n[-1] if m.lit[n] else "." for n in ("TL1R", "TL1Y", "TL1G", "TL2R", "TL2Y", "TL2G"))
                    log.info("t=%6.1fs cars=%d peds=%d passed=%d crashes=%d crash=%d I_XF=%d lamps(TL1 TL2)=%s clients=%d", self.t,
                             len(m.cars), len(m.peds), m.stats["cars_passed"], m.stats["crashes"], m.crash, m.hardware_fault(), lamps, self.clients)
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
    ap.add_argument("--endpoint", help="override opcua.endpoint, e.g. opc.tcp://0.0.0.0:4844")
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
