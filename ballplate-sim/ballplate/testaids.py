"""Test aids for the PLC: imperfections of sensors/actuators, random faults and control-quality metrics.

* ``Imperfections``: noise, delay and quantisation on the feedback words (F_XG..), dead time, backlash and a
  calibration offset on the tilt command (F_YR..).  Everything is off (0) by default.
* ``Faults``: random/triggered feedback freeze (stuck sensor / communication drop), tilt actuator stuck, pushes.
  Own random generator, so a seed makes a test repeatable.
* ``Metrics``: distance to a target, settling time, overshoot, time in the target zone, RMS error.
"""
from __future__ import annotations

import math
import random
from collections import deque
from dataclasses import dataclass


@dataclass
class Imperfections:
    noise_pos_mm: float = 0.0       # std dev of the distance feedback
    noise_vel_mms: float = 0.0      # std dev of the velocity feedback
    noise_acc: float = 0.0          # std dev of the acceleration feedback [m/s^2]
    delay_ms: float = 0.0           # delay of all feedback words
    quant_bits: int = 16            # effective resolution of the feedback words (16 = full 64000-step range)
    dead_ms: float = 0.0            # dead time of the tilt command
    backlash_deg: float = 0.0       # play in the tilt mechanism
    offset_x_deg: float = 0.0       # calibration offset of the tilt
    offset_y_deg: float = 0.0


class ImperfectionPipeline:
    def __init__(self, cfg: Imperfections | None = None, seed: int = 0):
        self.c = cfg or Imperfections()
        self.reseed(seed)
        self.tilt_buf: deque = deque()
        self.fb_buf: deque = deque()
        self.back = [0.0, 0.0]                     # backlash state: actual tilt of the mechanism (deg)

    def reseed(self, seed: int = 0):
        self.rng = random.Random(seed or None)

    @staticmethod
    def _delayed(buf: deque, value, n: int, initial=None):
        """Value from n cycles ago.  Until the buffer is full the output is `initial` (default: the first value)."""
        if not buf:
            buf.extend([value if initial is None else initial] * n)
        buf.append(value)
        while len(buf) > n + 1:
            buf.popleft()
        while len(buf) < n + 1:                    # delay was increased while running: repeat the oldest value
            buf.appendleft(buf[0])
        return buf[0]

    def tilt(self, tx: float, ty: float, cycle: float) -> tuple[float, float]:
        """Commanded tilt (deg) -> tilt setpoint (deg) of the plate: dead time, backlash, offset."""
        c = self.c
        n = int(round(c.dead_ms / 1000.0 / cycle))
        tx, ty = self._delayed(self.tilt_buf, (tx, ty), n, initial=(0.0, 0.0))     # the plate starts level
        out = []
        for i, v in enumerate((tx, ty)):
            half = 0.5 * c.backlash_deg
            if v > self.back[i] + half:
                self.back[i] = v - half
            elif v < self.back[i] - half:
                self.back[i] = v + half
            out.append(self.back[i])
        return out[0] + c.offset_x_deg, out[1] + c.offset_y_deg

    def feedback(self, phys: tuple, cycle: float) -> tuple:
        """Physical values (ax, vx, x, ay, vy, y) -> what the sensors report: delay, noise."""
        c = self.c
        n = int(round(c.delay_ms / 1000.0 / cycle))
        v = self._delayed(self.fb_buf, tuple(phys), n)
        sig = (c.noise_acc, c.noise_vel_mms / 1000.0, c.noise_pos_mm / 1000.0) * 2
        return tuple(x + (self.rng.gauss(0.0, s) if s > 0 else 0.0) for x, s in zip(v, sig))

    def quantise(self, raw: int, raw_max: int) -> int:
        bits = max(1, min(16, int(self.c.quant_bits)))
        if bits >= 16:
            return raw
        step = 2 ** (16 - bits)
        return max(-raw_max, min(raw_max, int(round(raw / step)) * step))


@dataclass
class FaultCfg:
    kick_per_min: float = 0.0       # random pushes
    kick_v: float = 0.3             # push size [m/s]
    freeze_per_min: float = 0.0     # feedback freezes (stuck sensor / communication drop)
    freeze_s: float = 1.0
    stick_per_min: float = 0.0      # tilt actuator sticks (holds its last command)
    stick_s: float = 1.0


class Faults:
    def __init__(self, cfg: FaultCfg | None = None, seed: int = 0):
        self.c = cfg or FaultCfg()
        self.time = 0.0
        self.freeze_until = 0.0
        self.stick_until = 0.0
        self.kick_pending = 0
        self.reseed(seed)

    def reseed(self, seed: int = 0):
        self.rng = random.Random((seed + 4242) if seed else None)

    def freeze_now(self):
        self.freeze_until = self.time + self.c.freeze_s

    def stick_now(self):
        self.stick_until = self.time + self.c.stick_s

    def kick_now(self):
        self.kick_pending += 1

    def step(self, dt: float):
        self.time += dt
        c = self.c
        if c.freeze_per_min > 0 and self.rng.random() < c.freeze_per_min / 60.0 * dt:
            self.freeze_now()
        if c.stick_per_min > 0 and self.rng.random() < c.stick_per_min / 60.0 * dt:
            self.stick_now()
        if c.kick_per_min > 0 and self.rng.random() < c.kick_per_min / 60.0 * dt:
            self.kick_now()

    @property
    def frozen(self) -> bool:
        return self.freeze_until > self.time

    @property
    def stuck(self) -> bool:
        return self.stick_until > self.time

    def take_kick(self) -> tuple[float, float] | None:
        """A pending push as a velocity change (random direction), or None."""
        if self.kick_pending <= 0:
            return None
        self.kick_pending -= 1
        a = self.rng.uniform(0.0, 2.0 * math.pi)
        return self.c.kick_v * math.cos(a), self.c.kick_v * math.sin(a)

    def state(self) -> dict:
        return {"freeze": round(max(0.0, self.freeze_until - self.time), 1),
                "stick": round(max(0.0, self.stick_until - self.time), 1)}


class Metrics:
    """Control quality relative to a target position, measured from the start of a run."""

    HOLD_S = 1.0                    # the ball counts as settled after this long inside the zone

    def __init__(self):
        self.tx = self.ty = 0.0
        self.zone = 0.05
        self.running = False
        self.start((0.0, 0.0))
        self.running = False

    def set_target(self, x: float, y: float, zone: float) -> bool:
        """True if the target changed (a new run should start)."""
        changed = (abs(x - self.tx) > 1e-9 or abs(y - self.ty) > 1e-9)
        self.tx, self.ty, self.zone = x, y, max(0.001, zone)
        return changed

    def start(self, pos: tuple[float, float]):
        dx, dy = self.tx - pos[0], self.ty - pos[1]
        self.d0 = math.hypot(dx, dy)
        self.u = (dx / self.d0, dy / self.d0) if self.d0 > 1e-9 else (0.0, 0.0)
        self.t = 0.0
        self.time_in = 0.0
        self.sq = 0.0
        self.max_past = 0.0
        self.inside_since: float | None = None
        self.err = self.d0
        self.running = True

    def update(self, dt: float, x: float, y: float, on_table: bool = True):
        if not self.running:
            self.err = math.hypot(x - self.tx, y - self.ty)
            return
        self.t += dt
        self.err = math.hypot(x - self.tx, y - self.ty) if on_table else float("inf")
        if self.err <= self.zone:
            self.time_in += dt
            if self.inside_since is None:
                self.inside_since = self.t
        else:
            self.inside_since = None
        if on_table:
            self.sq += self.err * self.err * dt
            past = (x - self.tx) * self.u[0] + (y - self.ty) * self.u[1]
            self.max_past = max(self.max_past, past)

    def state(self) -> dict:
        settled = self.inside_since is not None and self.t - self.inside_since >= self.HOLD_S
        return {"running": self.running, "target": [self.tx, self.ty], "zone": self.zone,
                "err": self.err if math.isfinite(self.err) else -1.0, "t": round(self.t, 2),
                "settled": settled, "settle_time": round(self.inside_since, 2) if settled else None,
                "overshoot_pct": round(100.0 * self.max_past / self.d0, 1) if self.d0 > self.zone else 0.0,
                "overshoot_m": round(self.max_past, 3),
                "in_zone_pct": round(100.0 * self.time_in / self.t, 1) if self.t > 0 else 0.0,
                "rms": round(math.sqrt(self.sq / self.t), 4) if self.t > 0 else 0.0}
