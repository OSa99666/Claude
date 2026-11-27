"""Transport simulator main loop: model + OPC UA server + Godot UI link.

    PLC (OPC UA client) --T_YH../T_YL../T_YR..--> [this process] --T_XGH../T_XCQ../T_BQ..--> PLC
                                                     ^   |
                          UDP commands (port 9875)   |   v UDP state (port 9874)
                                                     Godot UI / 3D view
"""
from __future__ import annotations

import argparse
import asyncio
import json
import logging
import math
import re
import socket
import time

from . import config as cfgmod
from .opcua_server import SimServer
from .transport import COLOURS, Pef, Transport
from .ui_link import UiLink

log = logging.getLogger("sim")


def _num(d, key, default):
    try:
        v = float(d.get(key, default))
    except (TypeError, ValueError):
        return default
    return v if math.isfinite(v) else default


def _parse_seq(text: str) -> list:
    """'1,2,3,4,5' (any separators) -> item types 1..5; other characters are ignored."""
    return [int(x) for x in re.findall(r"[1-5]", text)]


def _flags(d, key, n, default):
    v = d.get(key)
    if isinstance(v, list) and len(v) == n:
        return [bool(x) for x in v]
    return default


class Simulator:
    def __init__(self, cfg: dict):
        self.cfg = cfg
        self.pef = Pef.from_cfg(cfg)
        self.model = Transport(self.pef, cfg["sim"].get("random_seed", 0))
        fd, ft = cfg.get("feed", {}), cfg.get("faults", {})
        self.model.feed.rate_per_min = float(fd.get("rate_per_min", 0.0))
        seq = _parse_seq(str(fd.get("sequence", "")))
        if seq:
            self.model.feed.seq, self.model.feed.seq_on = seq, True
        fa = self.model.fault
        fa.ej_jam_prob = float(ft.get("ejector_jam_probability", 0.0))
        fa.ej_jam_s = float(ft.get("ejector_jam_s", 3.0))
        fa.dropout_per_min = float(ft.get("sensor_dropouts_per_min", 0.0))
        fa.dropout_s = float(ft.get("sensor_dropout_s", 0.4))
        fa.trip_per_min = float(ft.get("conveyor_trips_per_min", 0.0))
        self.server = SimServer(cfg)
        self.ui: UiLink | None = None
        self.cycle = cfg["sim"]["cycle_ms"] / 1000.0
        self.t = 0.0
        self.clients = 0
        self.io: dict = {}
        self.cmd_in: dict = {}
        self.udp = None
        vis = cfg["visual"]
        self.tick_m = vis.get("tick_m", 0.0041667)
        if vis.get("enabled", True):
            self.udp = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            self.udp_addr = (vis["host"], vis["port"])

    # ------------------------------------------------------------ UI -> model
    def apply_ui(self):
        u = self.ui.latest if self.ui else {}
        m, f, p = self.model, self.model.f, self.pef
        if u:
            f.xe = bool(u.get("feed", f.xe))
            f.xe01 = bool(u.get("feed_untreated", f.xe01))
            f.fsl_motor = _flags(u, "force_stop_motor", 5, f.fsl_motor)
            f.fsh_ej = _flags(u, "force_extend", 4, f.fsh_ej)
            f.fsl_ej = _flags(u, "force_retract", 4, f.fsl_ej)
            f.fh_ej = _flags(u, "force_fb_true", 4, f.fh_ej)
            f.fl_ej = _flags(u, "force_fb_false", 4, f.fl_ej)
            f.fh_det = _flags(u, "force_front_true", 4, f.fh_det)
            f.fl_det = _flags(u, "force_front_false", 4, f.fl_det)
            p.motor_start = max(0.0, _num(u, "motor_start", p.motor_start))
            p.motor_stop = max(0.0, _num(u, "motor_stop", p.motor_stop))
            p.ej_extend = max(0.01, _num(u, "ej_extend", p.ej_extend))
            p.ej_retract = max(0.01, _num(u, "ej_retract", p.ej_retract))
            p.ej_delay_high = max(0.0, _num(u, "ej_delay_high", p.ej_delay_high))
            p.ej_delay_low = max(0.0, _num(u, "ej_delay_low", p.ej_delay_low))
            for n in range(5):
                lo = max(0.002, _num(u, "period_min_ms", p.period_min[n] * 1000) / 1000)
                hi = max(lo + 0.001, _num(u, "period_max_ms", p.period_max[n] * 1000) / 1000)
                p.period_min[n], p.period_max[n] = lo, hi
            p.anim_ticks = int(max(1, min(10, _num(u, "anim_ticks", p.anim_ticks))))
            m.apply_params()
            fd, fa = m.feed, m.fault
            fd.seq_on = bool(u.get("feed_pattern_on", fd.seq_on))
            seq = _parse_seq(str(u.get("feed_pattern", "")))
            if seq:
                fd.seq = seq
            fd.rate_per_min = max(0.0, _num(u, "feed_rate", fd.rate_per_min))
            fa.ej_jam_prob = max(0.0, min(1.0, _num(u, "f_jam_pct", fa.ej_jam_prob * 100.0) / 100.0))
            fa.ej_jam_s = max(0.0, _num(u, "f_jam_s", fa.ej_jam_s))
            fa.dropout_per_min = max(0.0, _num(u, "f_drop_per_min", fa.dropout_per_min))
            fa.dropout_s = max(0.0, _num(u, "f_drop_s", fa.dropout_s))
            fa.trip_per_min = max(0.0, _num(u, "f_trip_per_min", fa.trip_per_min))
        if self.ui:
            if self.ui.take("feed_single_seq"):
                m.feed_single()
            if self.ui.take("empty_transport_seq"):
                m.empty_transport()
                log.info("transport emptied")
            for n in range(1, 6):
                if self.ui.take("empty_store%d_seq" % n):
                    m.empty_store(n)
            if self.ui.take("enter_store1_seq"):
                m.enter_store1_item5()
            if self.ui.take("restart_feed_seq"):
                m.reseed(int(_num(u, "feed_seed", m.seed)))
                log.info("item sequence restarted (seed %d)", m.seed)
            if self.ui.take("reset_stats_seq"):
                m.reset_stats()
            for n in range(1, 6):
                if self.ui.take("trip%d_seq" % n):
                    m.trip(n)
            for k in range(1, 5):
                if self.ui.take("jam%d_seq" % k):
                    m.jam_next_stroke(k)
            if self.ui.take("drop_all_seq"):
                m.drop_out_all_sensors()

    def params_for_ui(self) -> dict:
        p, f = self.pef, self.model.f
        return {"feed": f.xe, "feed_untreated": f.xe01, "motor_start": p.motor_start, "motor_stop": p.motor_stop,
                "ej_extend": p.ej_extend, "ej_retract": p.ej_retract, "ej_delay_high": p.ej_delay_high,
                "ej_delay_low": p.ej_delay_low, "period_min_ms": p.period_min[0] * 1000,
                "period_max_ms": p.period_max[0] * 1000, "anim_ticks": p.anim_ticks,
                "force_stop_motor": f.fsl_motor, "force_extend": f.fsh_ej, "force_retract": f.fsl_ej,
                "force_fb_true": f.fh_ej, "force_fb_false": f.fl_ej,
                "force_front_true": f.fh_det, "force_front_false": f.fl_det,
                "feed_pattern_on": self.model.feed.seq_on,
                "feed_pattern": ",".join(str(x) for x in self.model.feed.seq),
                "feed_rate": self.model.feed.rate_per_min, "feed_seed": self.model.seed,
                "f_jam_pct": self.model.fault.ej_jam_prob * 100.0, "f_jam_s": self.model.fault.ej_jam_s,
                "f_drop_per_min": self.model.fault.dropout_per_min, "f_drop_s": self.model.fault.dropout_s,
                "f_trip_per_min": self.model.fault.trip_per_min}

    def send_visual(self):
        if not self.udp:
            return
        m, p = self.model, self.pef
        msg = {"t": round(self.t, 3),
               "geo": {"tpi": p.tpi, "tick_m": self.tick_m, "t1_len": m.t1_max(), "side_len": m.side_max(),
                       "det": m.det_pos(), "ej": [m.ej_pos(k) for k in range(4)]},
               "t1": [[i.id, round(i.pos, 2)] for i in m.t1],
               "side": [[[i.id, round(i.pos, 2)] for i in s] for s in m.side],
               "store": m.store, "store_in": m.store_in, "colours": [0] + [COLOURS[i] for i in range(1, 6)], "run": [mo.at_high for mo in m.motors],
               "vel": [round(v, 2) for v in m.vel],
               "ej_pos": [round(e.pos, 3) for e in m.ejectors],
               "plc": self.clients > 0, "clients": self.clients,
               "stats": m.stats_state(), "faults": m.faults_state(),
               "io": self.io, "cmd": self.cmd_in, "params": self.params_for_ui()}
        try:
            self.udp.sendto(json.dumps(msg).encode(), self.udp_addr)
        except OSError:
            pass

    # ------------------------------------------------------------------ loop
    async def step_cycle(self):
        self.apply_ui()
        m = self.model
        self.clients = self.server.client_count()
        raw = await self.server.read_commands() if self.clients > 0 else {}
        g = lambda n, d=False: raw.get(n, d)
        m.cmd = dict(yh=[bool(g("T_YH1%d" % i)) for i in range(1, 6)],
                     yr=[int(g("T_YR1%d" % i, 0)) for i in range(1, 6)],
                     ej_yh=[bool(g("T_YH2%d" % k)) for k in range(1, 5)],
                     ej_yl=[bool(g("T_YL2%d" % k)) for k in range(1, 5)])
        self.cmd_in = {n: (int(v) if not isinstance(v, bool) else v) for n, v in raw.items()}
        m.step(self.cycle)
        self.t += self.cycle
        out = m.outputs()
        self.io = {"T_" + k: (int(v) if not isinstance(v, bool) else v) for k, v in out.items()}
        await self.server.write_outputs(self.io)
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
                    log.info("t=%6.1fs T1=%d items side=%s stores=%s motors=%s clients=%d", self.t, len(m.t1),
                             [len(s) for s in m.side], m.store, [int(x.at_high) for x in m.motors], self.clients)
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
    ap.add_argument("--endpoint", help="override opcua.endpoint, e.g. opc.tcp://0.0.0.0:4842")
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
