"""Simulator main loop: physics + OPC UA server + Godot UI link.

    PLC (OPC UA client) --F_YR10/F_YR20--> [this process] --F_XG1x/F_XG2x--> PLC
                                              ^   |
                      UDP commands (port 9871) |   v UDP state (port 9870)
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
from .physics import BallPlate, Params
from .scaling import from_raw, to_raw
from .ui_link import UiLink

log = logging.getLogger("sim")


def build_model(cfg) -> BallPlate:
    t, c = cfg["table"], cfg["contact"]
    p = Params(table_size=t["size_m"], ball_radius=t["ball_radius_m"],
               ball_mass=t["ball_mass_kg"], wall_height=t.get("wall_height_m", 0.12),
               tilt_max_deg=t["tilt_max_deg"], tilt_rate_deg_s=t["tilt_rate_deg_s"],
               tilt_tau=t["tilt_tau_s"], mu_s=c["mu_s"], mu_k=c["mu_k"], c_rr=c["c_rr"],
               e_wall=max(0.0, min(1.0, c["e_wall"])), walls_enabled=c["e_wall"] >= 0,
               mu_wall=c["mu_wall"],
               limit_velocity=cfg.get("limits", {}).get("limit_velocity", False),
               v_min=cfg["scaling"]["vel_min"], v_max=cfg["scaling"]["vel_max"])
    return BallPlate(p=p)


def _num(d: dict, key: str, default: float) -> float:
    try:
        v = float(d.get(key, default))
    except (TypeError, ValueError):
        return default
    return v if math.isfinite(v) else default


class Simulator:
    def __init__(self, cfg: dict):
        self.cfg = cfg
        self.model = build_model(cfg)
        self.model.reset(0.3, -0.3)          # start away from the centre
        self.sc = dict(cfg["scaling"])        # mutable: velocity range is editable in the UI
        self.server = SimServer(cfg)
        self.ui: UiLink | None = None
        self.elasticity = cfg["contact"]["e_wall"]
        self.cycle = cfg["sim"]["cycle_ms"] / 1000.0
        dt = cfg["sim"]["physics_dt_ms"] / 1000.0
        self.nsub = max(1, round(self.cycle / dt))
        self.dt = self.cycle / self.nsub
        self.t = 0.0
        self.prev_v = (0.0, 0.0)
        self.clients = 0
        self.raw_tilt = (0, 0)
        self.last_raw = {}
        v = cfg["visual"]
        self.udp = None
        if v.get("enabled", True):
            self.udp = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            self.udp_addr = (v["host"], v["port"])

    # ------------------------------------------------------------ UI -> model
    def apply_ui(self):
        u = self.ui.latest if self.ui else {}
        p = self.model.p
        if u:
            self.elasticity = _num(u, "elasticity", self.elasticity)
            p.walls_enabled = self.elasticity >= 0.0
            p.e_wall = max(0.0, min(1.0, self.elasticity))
            p.mu_s = max(0.0, _num(u, "mu_s", p.mu_s))
            p.mu_k = max(0.0, min(_num(u, "mu_k", p.mu_k), p.mu_s))
            vmin, vmax = _num(u, "vel_min", self.sc["vel_min"]), _num(u, "vel_max", self.sc["vel_max"])
            if vmin < vmax:
                self.sc["vel_min"], self.sc["vel_max"] = vmin, vmax
            p.v_min, p.v_max = self.sc["vel_min"], self.sc["vel_max"]
            p.limit_velocity = bool(u.get("limit_velocity", p.limit_velocity))
            m = self.model
            vsx, vsy = _num(u, "vel_sp_x", 0.0), _num(u, "vel_sp_y", 0.0)
            dsx, dsy = _num(u, "dist_sp_x", 0.0), _num(u, "dist_sp_y", 0.0)
            m.hold_vx = vsx if u.get("hold_vx") else None
            m.hold_vy = vsy if u.get("hold_vy") else None
            m.hold_x = dsx if u.get("hold_x") else None
            m.hold_y = dsy if u.get("hold_y") else None
        if self.ui and self.ui.take_centre():
            self.model.center()
            self.prev_v = (0.0, 0.0)
            log.info("ball centred")

    # ------------------------------------------------------------ model -> PLC
    def output_words(self) -> dict[str, int]:
        s, sc = self.model.s, self.sc
        rm = sc["raw_max"]
        if s.off_table:
            self.prev_v = (0.0, 0.0)
            return {k: 0 for k in ("F_XG10", "F_XG11", "F_XG12", "F_XG20", "F_XG21", "F_XG22")}
        ax = (s.vx - self.prev_v[0]) / self.cycle
        ay = (s.vy - self.prev_v[1]) / self.cycle
        self.prev_v = (s.vx, s.vy)
        a, d = sc["accel_ms2"], sc["dist_m"]
        return {
            "F_XG10": to_raw(ax, -a, a, rm), "F_XG11": to_raw(s.vx, sc["vel_min"], sc["vel_max"], rm),
            "F_XG12": to_raw(s.x, -d, d, rm),
            "F_XG20": to_raw(ay, -a, a, rm), "F_XG21": to_raw(s.vy, sc["vel_min"], sc["vel_max"], rm),
            "F_XG22": to_raw(s.y, -d, d, rm),
        }

    def send_visual(self):
        if not self.udp:
            return
        s, p = self.model.s, self.model.p
        msg = {"t": round(self.t, 3), "L": p.table_size, "R": p.ball_radius, "wh": p.wall_height,
               "walls": p.walls_enabled, "x": s.x, "y": s.y, "z": s.z, "off": s.off_table,
               "tx": s.tilt_x, "ty": s.tilt_y, "q": s.q, "slip": s.slipping,
               "plc": self.clients > 0, "clients": self.clients,
               "raw": self.last_raw, "tilt_raw": list(self.raw_tilt),
               "params": {"elasticity": self.elasticity, "mu_s": p.mu_s, "mu_k": p.mu_k,
                          "vel_min": self.sc["vel_min"], "vel_max": self.sc["vel_max"],
                          "limit_velocity": p.limit_velocity}}
        try:
            self.udp.sendto(json.dumps(msg).encode(), self.udp_addr)
        except OSError:
            pass

    # ------------------------------------------------------------------ loop
    async def step_cycle(self):
        self.apply_ui()
        self.raw_tilt = await self.server.read_tilts()
        self.clients = self.server.client_count()
        sc = self.sc
        if self.clients == 0 and self.cfg["sim"].get("comm_loss_levels_table", True):
            tx = ty = 0.0                       # fail-safe: nobody is controlling
        else:
            tx = from_raw(self.raw_tilt[0], -sc["tilt_deg"], sc["tilt_deg"], sc["raw_max"])
            ty = from_raw(self.raw_tilt[1], -sc["tilt_deg"], sc["tilt_deg"], sc["raw_max"])
        self.model.set_tilt_setpoint_deg(tx, ty)
        for _ in range(self.nsub):
            self.model.step(self.dt)
        self.t += self.cycle
        self.last_raw = self.output_words()
        await self.server.write_outputs(self.last_raw)
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
                    s = self.model.s
                    log.info("t=%6.1fs ball=(%+.3f,%+.3f)m tilt=(%+.1f,%+.1f)deg clients=%d %s",
                             self.t, s.x, s.y, math.degrees(s.tilt_x), math.degrees(s.tilt_y),
                             self.clients, "OFF TABLE" if s.off_table else
                             ("SLIP" if s.slipping else "roll"))
                    last_log = now
                next_t += self.cycle
                delay = next_t - time.monotonic()
                if delay < -5 * self.cycle:     # fell far behind: resync
                    next_t, delay = time.monotonic(), 0
                await asyncio.sleep(max(0.0, delay))
        finally:
            await self.server.stop()


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawTextHelpFormatter)
    ap.add_argument("-c", "--config", default=None)
    ap.add_argument("--endpoint", help="override opcua.endpoint, e.g. opc.tcp://0.0.0.0:4840")
    ap.add_argument("-v", "--verbose", action="store_true")
    a = ap.parse_args()
    logging.basicConfig(level=logging.DEBUG if a.verbose else logging.INFO,
                        format="%(asctime)s %(name)s %(levelname)s %(message)s")
    logging.getLogger("asyncua").setLevel(logging.INFO if a.verbose else logging.WARNING)
    cfg = cfgmod.load(a.config)
    if a.endpoint:
        cfg["opcua"]["endpoint"] = a.endpoint
    try:
        asyncio.run(Simulator(cfg).run())
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()
