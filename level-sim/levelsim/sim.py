"""Level simulator main loop: process model + OPC UA server + Godot UI link.

    PLC (OPC UA client) --T_FCV10--> [this process] --T_LT10, T_LSH10, T_LSL10, T_LSHH10--> PLC
                                        ^   |
                UDP commands (port 9877) |   v UDP state (port 9876)
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
from .process import LevelProcess, Params
from .ui_link import UiLink

log = logging.getLogger("sim")

# UI keys -> Params fields (floats)
FLOAT_KEYS = {"delay": (0.0, 60.0), "t_act": (0.0, 60.0), "fn": (0.005, 5.0), "zeta": (0.01, 5.0),
              "gain": (-10.0, 10.0), "k_acc": (0.0, 5.0), "outlet": (0.0, 100.0),
              "sys_osc_f": (0.0, 5.0), "sys_osc_gain": (0.0, 100.0),
              "rnd_osc_f": (0.0, 5.0), "rnd_osc_gain": (0.0, 100.0),
              "cons_f": (0.0, 20.0), "cons_gain": (0.0, 100.0),
              "noise_f": (0.0, 50.0), "noise_gain": (0.0, 50.0),
              "h_high": (0.0, 100.0), "h_hh": (0.0, 100.0), "h_low": (0.0, 100.0), "hysteresis": (0.0, 50.0)}


def _num(d: dict, key: str, default: float) -> float:
    try:
        v = float(d.get(key, default))
    except (TypeError, ValueError):
        return default
    return v if math.isfinite(v) else default


class Simulator:
    def __init__(self, cfg: dict):
        self.cfg = cfg
        c = cfg["process"]
        seed = int(cfg["sim"].get("seed", 0)) or None
        self.proc = LevelProcess(Params(
            delay=c["delay_s"], t_act=c["actuator_lag_s"], fn=c["fn_hz"], zeta=c["zeta"], gain=c["gain"],
            k_acc=c["k_acc"], outlet=c["outlet_pct"], h_high=c["h_high"], h_hh=c.get("h_highhigh", 95.0), h_low=c["h_low"],
            hysteresis=c["hysteresis"], lsl_inverted=c.get("lsl_inverted", False)),
            level=c["start_level"], seed=seed)
        self.raw_full = float(cfg["scaling"]["raw_full"])
        self.server = SimServer(cfg)
        self.ui: UiLink | None = None
        self.cycle = cfg["sim"]["cycle_ms"] / 1000.0
        self.t = 0.0
        self.clients = 0
        self.fcv_raw = 0
        self.lt_raw = 0
        self.manual_on = False
        self.manual_u = 0.0
        self.udp = None
        vis = cfg["visual"]
        if vis.get("enabled", True):
            self.udp = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            self.udp_addr = (vis["host"], vis["port"])
        self._struct = None

    # --- scaling
    def raw_to_pct(self, raw: int) -> float:
        return raw * 100.0 / self.raw_full

    def pct_to_raw(self, pct: float) -> int:
        return int(round(max(-32768.0, min(32767.0, pct * self.raw_full / 100.0))))

    def apply_ui(self):
        u = self.ui.latest if self.ui else {}
        p = self.proc.p
        if u:
            for k, (lo, hi) in FLOAT_KEYS.items():
                setattr(p, k, min(hi, max(lo, _num(u, k, getattr(p, k)))))
            p.lsl_inverted = bool(u.get("lsl_inverted", p.lsl_inverted))
            self.manual_on = bool(u.get("manual_on", self.manual_on))
            self.manual_u = min(100.0, max(0.0, _num(u, "manual_u", self.manual_u)))
        if self.ui:
            if self.ui.take("set_level_seq"):
                self.proc.set_level(_num(u, "set_level", self.proc.h))
                log.info("level set to %.1f %%", self.proc.h)
            if self.ui.take("pulse_seq"):
                self.proc.pulse(_num(u, "pulse_amount", 20.0), max(0.1, _num(u, "pulse_s", 5.0)))
                log.info("consumption pulse %.1f %% for %.1f s", self.proc.cons_pulse, self.proc.cons_pulse_left)

    def send_visual(self):
        if not self.udp:
            return
        p = self.proc.p
        pr = self.proc
        params = {k: getattr(p, k) for k in list(FLOAT_KEYS) + ["lsl_inverted"]}
        params.update(manual_on=self.manual_on, manual_u=self.manual_u)
        msg = {"t": round(self.t, 3), "h": pr.h, "y": pr.measured, "u": pr.u, "u_delayed": pr.u_delayed,
               "q_in": pr.q_in, "q_out": pr.q_out, "q_over": pr.q_over, "spilled": pr.spilled,
               "overflow": pr.overflowing, "lsh": pr.lsh, "lsl": pr.lsl, "lshh": pr.lshh,
               "fcv_raw": self.fcv_raw, "lt_raw": self.lt_raw,
               "plc": self.clients > 0, "clients": self.clients, "manual": self.manual_on,
               "order": pr.order(), "tf": pr.transfer_function(),
               "poles": ["%.3f%+.3fj" % (z.real, z.imag) for z in pr.poles()],
               "params": params}
        try:
            self.udp.sendto(json.dumps(msg).encode(), self.udp_addr)
        except OSError:
            pass

    async def step_cycle(self):
        self.apply_ui()
        self.clients = self.server.client_count()
        # nobody connected: the valve is closed (unless the manual override is on)
        self.fcv_raw = await self.server.read_valve_raw() if self.clients > 0 else 0
        u = self.manual_u if self.manual_on else self.raw_to_pct(self.fcv_raw)
        self.proc.step(self.cycle, u)
        self.t += self.cycle
        self.lt_raw = self.pct_to_raw(self.proc.measured)
        await self.server.write_feedbacks(self.lt_raw, self.proc.lsh, self.proc.lsl, self.proc.lshh)
        self.send_visual()

    async def run(self):
        await self.server.start()
        self.ui = await UiLink.start(self.cfg["visual"]["command_port"])
        log.info("process: %s", self.proc.transfer_function())
        log.info("poles: %s", ", ".join("%.3f%+.3fj" % (z.real, z.imag) for z in self.proc.poles()))
        next_t = time.monotonic()
        last_log = 0.0
        try:
            while True:
                await self.step_cycle()
                now = time.monotonic()
                if now - last_log > 2.0:
                    log.info("t=%6.1fs level=%5.1f%% u=%5.1f%% FCV=%d LT=%d LSH=%d LSL=%d LSHH=%d clients=%d%s", self.t,
                             self.proc.h, self.proc.u, self.fcv_raw, self.lt_raw, self.proc.lsh, self.proc.lsl, self.proc.lshh,
                             self.clients, " (manual)" if self.manual_on else "")
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
    ap.add_argument("--endpoint", help="override opcua.endpoint, e.g. opc.tcp://0.0.0.0:4843")
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
