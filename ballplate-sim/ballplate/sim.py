"""Simulator main loop: physics + OPC UA client + UDP feed for Godot."""
from __future__ import annotations

import argparse
import asyncio
import collections
import json
import logging
import math
import random
import socket
import time

from . import config as cfgmod
from .physics import BallPlate, Params
from .plcio import PlcLink

log = logging.getLogger("sim")


def build_model(cfg) -> BallPlate:
    t, c = cfg["table"], cfg["contact"]
    p = Params(table_size=t["size_m"], ball_radius=t["ball_radius_m"],
               ball_mass=t["ball_mass_kg"], tilt_max_deg=t["tilt_max_deg"],
               tilt_rate_deg_s=t["tilt_rate_deg_s"], tilt_tau=t["tilt_tau_s"],
               mu_s=c["mu_s"], mu_k=c["mu_k"], c_rr=c["c_rr"],
               e_wall=c["e_wall"], mu_wall=c["mu_wall"])
    return BallPlate(p=p)


class Sensor:
    """Ball position as seen by the PLC: delayed and noisy."""

    def __init__(self, noise_mm, delay_ms):
        self.noise_mm, self.delay_ms = noise_mm, delay_ms
        self.buf = collections.deque()

    def sample(self, t, x, y):
        self.buf.append((t, x, y))
        tt = t - self.delay_ms / 1000.0
        while len(self.buf) > 1 and self.buf[1][0] <= tt:
            self.buf.popleft()
        _, xd, yd = self.buf[0]
        n = self.noise_mm / 1000.0
        return xd + random.gauss(0, n), yd + random.gauss(0, n)


class Simulator:
    def __init__(self, cfg, plc: PlcLink | None):
        self.cfg, self.plc = cfg, plc
        self.model = build_model(cfg)
        self.sensor = Sensor(cfg["sensor"]["noise_mm"], cfg["sensor"]["delay_ms"])
        self.model.reset(0.05, -0.05)
        self.cycle = cfg["sim"]["cycle_ms"] / 1000.0
        dt = cfg["sim"]["physics_dt_ms"] / 1000.0
        self.nsub = max(1, round(self.cycle / dt))
        self.dt = self.cycle / self.nsub
        self.t = 0.0
        self.hb = 0
        self.prev_reset = False
        self.last_plc_ok = None   # monotonic time of last good exchange
        self.reset_xy = (0.0, 0.0)
        self._conn_task = None
        self._retry_at = 0.0
        v = cfg["visual"]
        self.udp = None
        if v.get("enabled", True):
            self.udp = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            self.udp_addr = (v["host"], v["port"])

    # -------------------------------------------------------------- helpers
    def apply_params(self, prm: dict):
        p = self.model.p
        def ok(v): return v is not None and math.isfinite(v)
        if ok(prm.get("Mu_S")):    p.mu_s = max(0.0, prm["Mu_S"])
        if ok(prm.get("Mu_K")):    p.mu_k = max(0.0, min(prm["Mu_K"], p.mu_s))
        if ok(prm.get("E_Wall")):  p.e_wall = max(0.0, min(1.0, prm["E_Wall"]))
        if ok(prm.get("Mu_Wall")): p.mu_wall = max(0.0, prm["Mu_Wall"])
        if ok(prm.get("Noise_mm")): self.sensor.noise_mm = max(0.0, prm["Noise_mm"])
        if ok(prm.get("Delay_ms")): self.sensor.delay_ms = max(0.0, prm["Delay_ms"])
        lim = 0.5 * p.table_size - p.ball_radius
        self.reset_xy = (max(-lim, min(lim, prm.get("Reset_X") or 0.0)),
                         max(-lim, min(lim, prm.get("Reset_Y") or 0.0)))

    def physics_cycle(self):
        for _ in range(self.nsub):
            self.model.step(self.dt)
        self.t += self.cycle

    def send_visual(self, plc_ok: bool):
        if not self.udp:
            return
        s, p = self.model.s, self.model.p
        msg = {"t": round(self.t, 3), "L": p.table_size, "R": p.ball_radius,
               "x": s.x, "y": s.y, "tx": s.tilt_x, "ty": s.tilt_y,
               "q": s.q, "plc": plc_ok, "slip": s.slipping,
               "mu_s": p.mu_s, "e": p.e_wall}
        try:
            self.udp.sendto(json.dumps(msg).encode(), self.udp_addr)
        except OSError:
            pass

    # ----------------------------------------------------------------- loop
    def _ensure_connecting(self, now):
        """Reconnect in the background so a dead PLC never stalls the physics."""
        if self._conn_task is not None and not self._conn_task.done():
            return
        if now < self._retry_at:
            return
        async def attempt():
            try:
                await self.plc.connect()
            except Exception as e:  # noqa: BLE001
                log.warning("cannot connect to PLC (%r); retrying", e)
                await self.plc.disconnect()
                self._retry_at = time.monotonic() + 2.0
        self._conn_task = asyncio.create_task(attempt())

    async def exchange(self):
        """One OPC UA round-trip.  Raises on communication failure."""
        fast = await self.plc.read_fast()
        self.model.set_tilt_setpoint_deg(float(fast["TiltX_SP"]), float(fast["TiltY_SP"]))
        rising = bool(fast["Cmd_Reset"]) and not self.prev_reset
        self.prev_reset = bool(fast["Cmd_Reset"])
        if rising:
            self.model.reset(*self.reset_xy)
            self.sensor.buf.clear()
        mx, my = self.sensor.sample(self.t, self.model.s.x, self.model.s.y)
        self.hb = (self.hb + 1) & 0x7FFFFFFF
        await self.plc.write_outputs({
            "Ball_X": mx, "Ball_Y": my,
            "TiltX_PV": math.degrees(self.model.s.tilt_x),
            "TiltY_PV": math.degrees(self.model.s.tilt_y),
            "Heartbeat": self.hb})

    async def run(self):
        cfg = self.cfg["sim"]
        next_t = time.monotonic()
        next_params = 0.0
        last_log = 0.0
        while True:
            now = time.monotonic()
            plc_ok = False
            if self.plc is not None:
                if not self.plc.connected:
                    self._ensure_connecting(now)
                else:
                    try:
                        if now >= next_params:
                            self.apply_params(await self.plc.read_params())
                            next_params = now + cfg["params_period_s"]
                        await self.exchange()
                        self.last_plc_ok = now
                    except Exception as e:  # noqa: BLE001
                        log.warning("PLC link lost: %r", e)
                        await self.plc.disconnect()
                        next_params = 0.0
                plc_ok = (self.last_plc_ok is not None
                          and now - self.last_plc_ok < cfg["comm_timeout_s"])
                if not plc_ok:   # fail-safe: level the table
                    self.model.set_tilt_setpoint_deg(0.0, 0.0)
            self.physics_cycle()
            self.send_visual(plc_ok)
            if now - last_log > 2.0:
                s = self.model.s
                log.info("t=%6.1fs ball=(%+.3f,%+.3f)m tilt=(%+.1f,%+.1f)deg plc=%s %s",
                         self.t, s.x, s.y, math.degrees(s.tilt_x), math.degrees(s.tilt_y),
                         "ok" if plc_ok else "--", "SLIP" if s.slipping else "roll")
                last_log = now
            next_t += self.cycle
            delay = next_t - time.monotonic()
            if delay < -5 * self.cycle:        # fell far behind: resync
                next_t = time.monotonic()
                delay = 0
            await asyncio.sleep(max(0.0, delay))


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("-c", "--config", default=None)
    ap.add_argument("--url", help="override opcua.url")
    ap.add_argument("--no-plc", action="store_true",
                    help="run without OPC UA (table stays flat; for visual checks)")
    ap.add_argument("-v", "--verbose", action="store_true")
    a = ap.parse_args()
    logging.basicConfig(level=logging.DEBUG if a.verbose else logging.INFO,
                        format="%(asctime)s %(name)s %(levelname)s %(message)s")
    logging.getLogger("asyncua").setLevel(logging.WARNING if not a.verbose else logging.INFO)
    cfg = cfgmod.load(a.config)
    if a.url:
        cfg["opcua"]["url"] = a.url
    plc = None if a.no_plc else PlcLink(cfg)
    try:
        asyncio.run(Simulator(cfg, plc).run())
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()
