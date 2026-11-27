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
import random
import socket
import time

from . import config as cfgmod
from . import subscription_tweaks
from .opcua_server import SimServer
from .physics import BallPlate, Params
from .scaling import from_raw, to_raw
from .testaids import FaultCfg, Faults, Imperfections, ImperfectionPipeline, Metrics
from .ui_link import UiLink

log = logging.getLogger("sim")


def build_model(cfg) -> BallPlate:
    t, c = cfg["table"], cfg["contact"]
    p = Params(table_size=t["size_m"], ball_radius=t["ball_radius_m"],
               ball_mass=t["ball_mass_kg"], wall_height=t.get("wall_height_m", 0.12),
               wall_thickness=t.get("wall_thickness_m", 0.003),
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
        self.stats = self._new_stats()
        self.last_tilt_change = 0.0
        # test aids: imperfections, faults, control-quality metrics (all off / idle by default)
        seed = int(cfg["sim"].get("random_seed", 0))
        im, ft = cfg.get("imperfections", {}), cfg.get("faults", {})
        self.imp = ImperfectionPipeline(Imperfections(
            noise_pos_mm=float(im.get("noise_pos_mm", 0.0)), noise_vel_mms=float(im.get("noise_vel_mms", 0.0)),
            noise_acc=float(im.get("noise_acc_ms2", 0.0)), delay_ms=float(im.get("feedback_delay_ms", 0.0)),
            quant_bits=int(im.get("quantisation_bits", 16)), dead_ms=float(im.get("tilt_dead_time_ms", 0.0)),
            backlash_deg=float(im.get("tilt_backlash_deg", 0.0)), offset_x_deg=float(im.get("tilt_offset_x_deg", 0.0)),
            offset_y_deg=float(im.get("tilt_offset_y_deg", 0.0))), seed)
        self.faults = Faults(FaultCfg(
            kick_per_min=float(ft.get("kicks_per_min", 0.0)), kick_v=float(ft.get("kick_speed_ms", 0.3)),
            freeze_per_min=float(ft.get("feedback_freezes_per_min", 0.0)), freeze_s=float(ft.get("feedback_freeze_s", 1.0)),
            stick_per_min=float(ft.get("actuator_sticks_per_min", 0.0)), stick_s=float(ft.get("actuator_stick_s", 1.0))), seed)
        self.seed = seed
        self.seed_ui = seed
        self.rel = {"rel_x": 0.5, "rel_y": 0.3, "rel_vx": 0.0, "rel_vy": 0.0}
        self.metrics = Metrics()
        self.rel_rng = random.Random(seed or None)
        self.last_tilt_cmd = (0.0, 0.0)
        v = cfg["visual"]
        self.udp = None
        if v.get("enabled", True):
            self.udp = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            self.udp_addr = (v["host"], v["port"])

    @staticmethod
    def _new_stats() -> dict:
        return {"n": 0, "cycle_sum": 0.0, "cycle_max": 0.0, "phys_max": 0.0, "write_max": 0.0, "send_max": 0.0,
                "late": 0, "gap_max": 0.0, "tilt_changes": 0, "tilt_pause_max": 0.0}

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
            self._apply_test_aid_ui(u)
        if self.ui and self.ui.take_centre():
            self.model.center()
            self.prev_v = (0.0, 0.0)
            log.info("ball centred")
        if self.ui:
            self._apply_test_aid_actions()

    def _apply_test_aid_ui(self, u: dict):
        c, f, m = self.imp.c, self.faults.c, self.model
        c.noise_pos_mm = max(0.0, _num(u, "imp_noise_pos", c.noise_pos_mm))
        c.noise_vel_mms = max(0.0, _num(u, "imp_noise_vel", c.noise_vel_mms))
        c.noise_acc = max(0.0, _num(u, "imp_noise_acc", c.noise_acc))
        c.delay_ms = max(0.0, _num(u, "imp_delay_ms", c.delay_ms))
        c.quant_bits = int(max(1, min(16, _num(u, "imp_quant_bits", c.quant_bits))))
        c.dead_ms = max(0.0, _num(u, "imp_dead_ms", c.dead_ms))
        c.backlash_deg = max(0.0, _num(u, "imp_backlash", c.backlash_deg))
        c.offset_x_deg = _num(u, "imp_off_x", c.offset_x_deg)
        c.offset_y_deg = _num(u, "imp_off_y", c.offset_y_deg)
        f.kick_per_min = max(0.0, _num(u, "f_kick_per_min", f.kick_per_min))
        f.kick_v = max(0.0, _num(u, "f_kick_v", f.kick_v))
        f.freeze_per_min = max(0.0, _num(u, "f_freeze_per_min", f.freeze_per_min))
        f.freeze_s = max(0.0, _num(u, "f_freeze_s", f.freeze_s))
        f.stick_per_min = max(0.0, _num(u, "f_stick_per_min", f.stick_per_min))
        f.stick_s = max(0.0, _num(u, "f_stick_s", f.stick_s))
        self.seed_ui = int(_num(u, "f_seed", self.seed_ui))
        m.patch_on = bool(u.get("patch_on", m.patch_on))
        m.patch_x, m.patch_y = _num(u, "patch_x", m.patch_x), _num(u, "patch_y", m.patch_y)
        m.patch_r = max(0.01, _num(u, "patch_r", m.patch_r))
        m.patch_drag = max(0.0, _num(u, "patch_drag", m.patch_drag))
        for k in self.rel:
            self.rel[k] = _num(u, k, self.rel[k])
        s = m.s
        if self.metrics.set_target(_num(u, "tgt_x", self.metrics.tx), _num(u, "tgt_y", self.metrics.ty),
                                   _num(u, "tgt_r", self.metrics.zone)):
            self.metrics.start((s.x, s.y))                      # a new target starts a new measurement

    def _apply_test_aid_actions(self):
        ui, m, s = self.ui, self.model, self.model.s
        if ui.take("release_seq"):
            self.release(self.rel["rel_x"], self.rel["rel_y"], self.rel["rel_vx"], self.rel["rel_vy"])
        if ui.take("release_random_seq"):
            half = 0.5 * m.p.table_size - 2 * m.p.ball_radius
            self.release(self.rel_rng.uniform(-half, half), self.rel_rng.uniform(-half, half),
                         self.rel_rng.uniform(-0.2, 0.2), self.rel_rng.uniform(-0.2, 0.2))
        if ui.take("metrics_start_seq"):
            self.metrics.start((s.x, s.y))
        if ui.take("kick_seq"):
            self.faults.kick_now()
        if ui.take("freeze_seq"):
            self.faults.freeze_now()
        if ui.take("stick_seq"):
            self.faults.stick_now()
        if ui.take("f_restart_seq"):
            self.seed = self.seed_ui
            self.faults.reseed(self.seed)
            self.imp.reseed(self.seed)
            self.rel_rng = random.Random(self.seed or None)
            log.info("random generators restarted (seed %d)", self.seed)

    def release(self, x: float, y: float, vx: float, vy: float):
        """Release the ball at (x, y) with velocity (vx, vy) and start a new measurement."""
        half = 0.5 * self.model.p.table_size - self.model.p.ball_radius
        x, y = max(-half, min(half, x)), max(-half, min(half, y))
        self.model.reset(x, y, vx, vy)
        self.prev_v = (vx, vy)
        self.metrics.start((x, y))
        log.info("ball released at (%+.2f, %+.2f) m with (%+.2f, %+.2f) m/s", x, y, vx, vy)

    def test_params(self) -> dict:
        c, f, m = self.imp.c, self.faults.c, self.model
        return {"imp_noise_pos": c.noise_pos_mm, "imp_noise_vel": c.noise_vel_mms, "imp_noise_acc": c.noise_acc,
                "imp_delay_ms": c.delay_ms, "imp_quant_bits": c.quant_bits, "imp_dead_ms": c.dead_ms,
                "imp_backlash": c.backlash_deg, "imp_off_x": c.offset_x_deg, "imp_off_y": c.offset_y_deg,
                "f_kick_per_min": f.kick_per_min, "f_kick_v": f.kick_v, "f_freeze_per_min": f.freeze_per_min,
                "f_freeze_s": f.freeze_s, "f_stick_per_min": f.stick_per_min, "f_stick_s": f.stick_s,
                "f_seed": self.seed, "patch_on": m.patch_on, "patch_x": m.patch_x, "patch_y": m.patch_y,
                "patch_r": m.patch_r, "patch_drag": m.patch_drag,
                "tgt_x": self.metrics.tx, "tgt_y": self.metrics.ty, "tgt_r": self.metrics.zone, **self.rel}

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
        # sensor imperfections: delay + noise on the physical values, then scaling and quantisation
        px, pvx, pxx, py, pvy, pyy = self.imp.feedback((ax, s.vx, s.x, ay, s.vy, s.y), self.cycle)
        if self.faults.frozen and self.last_raw:                   # stuck sensor / communication drop
            return dict(self.last_raw)
        a, d = sc["accel_ms2"], sc["dist_m"]
        q = lambda v: self.imp.quantise(v, rm)
        return {
            "F_XG10": q(to_raw(px, -a, a, rm)), "F_XG11": q(to_raw(pvx, sc["vel_min"], sc["vel_max"], rm)),
            "F_XG12": q(to_raw(pxx, -d, d, rm)),
            "F_XG20": q(to_raw(py, -a, a, rm)), "F_XG21": q(to_raw(pvy, sc["vel_min"], sc["vel_max"], rm)),
            "F_XG22": q(to_raw(pyy, -d, d, rm)),
        }

    def send_visual(self):
        if not self.udp:
            return
        s, p = self.model.s, self.model.p
        msg = {"t": round(self.t, 3), "L": p.table_size, "R": p.ball_radius, "wh": p.wall_height, "wt": p.wall_thickness,
               "walls": p.walls_enabled, "x": s.x, "y": s.y, "z": s.z, "off": s.off_table,
               "tx": s.tilt_x, "ty": s.tilt_y, "q": s.q, "slip": s.slipping,
               "plc": self.clients > 0, "clients": self.clients,
               "raw": self.last_raw, "tilt_raw": list(self.raw_tilt),
               "metrics": self.metrics.state(), "faults": self.faults.state(),
               "patch": {"on": self.model.patch_on, "x": self.model.patch_x, "y": self.model.patch_y,
                         "r": self.model.patch_r},
               "params": {"elasticity": self.elasticity, "mu_s": p.mu_s, "mu_k": p.mu_k,
                          "vel_min": self.sc["vel_min"], "vel_max": self.sc["vel_max"],
                          "limit_velocity": p.limit_velocity, **self.test_params()}}
        try:
            self.udp.sendto(json.dumps(msg).encode(), self.udp_addr)
        except OSError:
            pass

    # ------------------------------------------------------------------ loop
    async def step_cycle(self):
        t_start = time.perf_counter()
        self.apply_ui()
        new_tilt = await self.server.read_tilts()
        if new_tilt != self.raw_tilt:                     # the PLC wrote a different tilt word
            self.stats["tilt_changes"] += 1
            self.stats["tilt_pause_max"] = max(self.stats["tilt_pause_max"], self.t - self.last_tilt_change)
            self.last_tilt_change = self.t
        self.raw_tilt = new_tilt
        self.clients = self.server.client_count()
        sc = self.sc
        if self.clients == 0 and self.cfg["sim"].get("comm_loss_levels_table", True):
            tx = ty = 0.0                       # fail-safe: nobody is controlling
        else:
            tx = from_raw(self.raw_tilt[0], -sc["tilt_deg"], sc["tilt_deg"], sc["raw_max"])
            ty = from_raw(self.raw_tilt[1], -sc["tilt_deg"], sc["tilt_deg"], sc["raw_max"])
        tx, ty = self.imp.tilt(tx, ty, self.cycle)               # dead time, backlash, calibration offset
        if self.faults.stuck:
            tx, ty = self.last_tilt_cmd                          # actuator stuck: keeps its last command
        else:
            self.last_tilt_cmd = (tx, ty)
        self.model.set_tilt_setpoint_deg(tx, ty)
        self.faults.step(self.cycle)
        kick = self.faults.take_kick()
        if kick:
            self.model.kick(*kick)
        t_phys = time.perf_counter()
        for _ in range(self.nsub):
            self.model.step(self.dt)
        s = self.model.s
        self.metrics.update(self.cycle, s.x, s.y, not s.off_table)
        self.t += self.cycle
        self.last_raw = self.output_words()
        t_io = time.perf_counter()
        await self.server.write_outputs(self.last_raw)
        t_out = time.perf_counter()
        self.send_visual()
        t_end = time.perf_counter()
        st = self.stats
        st["n"] += 1
        st["cycle_sum"] += t_end - t_start
        st["cycle_max"] = max(st["cycle_max"], t_end - t_start)
        st["phys_max"] = max(st["phys_max"], t_io - t_phys)
        st["write_max"] = max(st["write_max"], t_out - t_io)
        st["send_max"] = max(st["send_max"], t_end - t_out)

    async def run(self):
        await self.server.start()
        self.ui = await UiLink.start(self.cfg["visual"]["command_port"])
        next_t = time.monotonic()
        last_log = 0.0
        prev_end = None
        try:
            while True:
                await self.step_cycle()
                now = time.monotonic()
                if prev_end is not None:
                    self.stats["gap_max"] = max(self.stats["gap_max"], now - prev_end)
                prev_end = now
                if now - last_log > 2.0:
                    s = self.model.s
                    st = self.stats
                    log.info("t=%6.1fs ball=(%+.3f,%+.3f)m tilt=(%+.1f,%+.1f)deg clients=%d %s",
                             self.t, s.x, s.y, math.degrees(s.tilt_x), math.degrees(s.tilt_y),
                             self.clients, "OFF TABLE" if s.off_table else
                             ("SLIP" if s.slipping else "roll"))
                    if st["n"]:
                        # timing of the last ~2 s: cycle = whole step, gap = time between two cycles (target 10 ms)
                        log.info("timing: cycle avg %.2f / max %.2f ms (physics max %.2f, OPC UA write max %.2f, "
                                 "UDP send max %.2f), longest gap between cycles %.1f ms, late cycles %d",
                                 1000 * st["cycle_sum"] / st["n"], 1000 * st["cycle_max"], 1000 * st["phys_max"],
                                 1000 * st["write_max"], 1000 * st["send_max"], 1000 * st["gap_max"], st["late"])
                        log.info("PLC tilt words F_YR10/F_YR20: %d value changes in the last 2 s, longest pause between "
                                 "changes %.0f ms (a PLC writing every 20 ms gives ~100 changes while the table is moving)",
                                 st["tilt_changes"], 1000 * st["tilt_pause_max"])
                        for line in subscription_tweaks.snapshot():
                            log.info("OPC UA publish: %s", line)
                        if st["gap_max"] > 0.1:
                            log.warning("simulation loop stalled for %.0f ms in the last 2 s", 1000 * st["gap_max"])
                    self.stats = self._new_stats()
                    last_log = now
                next_t += self.cycle
                delay = next_t - time.monotonic()
                if delay < 0:
                    self.stats["late"] += 1
                if delay < -5 * self.cycle:     # fell far behind: resync
                    next_t, delay = time.monotonic(), 0
                await asyncio.sleep(max(0.0, delay))
        finally:
            await self.server.stop()


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawTextHelpFormatter)
    ap.add_argument("-c", "--config", default=None)
    ap.add_argument("--endpoint", help="override opcua.endpoint, e.g. opc.tcp://0.0.0.0:4840")
    ap.add_argument("--publish-cap-ms", type=float, default=None,
                    help="cap every client's subscription publishing interval (ms), e.g. 20; 0 = off")
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
