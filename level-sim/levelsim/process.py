"""Level process: a vessel filled through a valve, as a chain of dynamic blocks (SISO).

    u (valve 0..100 %) -> dead time -> [valve actuator lag] -> 2nd-order block -> + inflow disturbance
                                                                   |
        level h (0..100 %)  <-  integrator (accumulator)  <-  inflow - outflow
                                                                   |
                           outflow = outlet valve (load) + random consumption

The vessel is OPEN at the top: above 100 % the surplus spills over the rim (``spilled``, ``q_over``,
``overflowing``); the level stays at 100 %.

State order: 2nd-order block (2) + accumulator (1) = 3; the optional actuator lag (``t_act`` > 0) makes it 4.
Dead time is on top of that.  Linear transfer function (valve command -> level, no saturation, no disturbance):

    G(s) = exp(-s*delay) * gain * k_acc * wn^2 / ( s * (s^2 + 2*zeta*wn*s + wn^2) * (t_act*s + 1) )

The level is an integrator: it holds any value when inflow = outflow (the valve position at which this
happens depends on the outlet valve / consumption), so a controller needs integral-free action for the
level itself but is sensitive to the 2nd-order resonance and the dead time.

Test signals (all in %, 0 = off):
  sys_osc    sinusoid at the INPUT of the 2nd-order block (excites its resonance when f = fn)
  rnd_osc    sinusoid with randomly varying frequency/amplitude added to the inflow
  cons       random consumption: band-limited random added to the outflow
  noise      band-limited random added to the measured level only
"""
from __future__ import annotations

import cmath
import math
import random
from collections import deque
from dataclasses import dataclass


@dataclass
class Params:
    # dynamics
    delay: float = 1.5          # s   dead time valve -> flow
    t_act: float = 0.0          # s   valve actuator lag (0 = off -> order 3, > 0 -> order 4)
    fn: float = 0.15            # Hz  2nd-order block natural frequency
    zeta: float = 0.3           #     damping
    gain: float = 1.0           #     static gain of the 2nd-order block (valve % -> inflow %)
    k_acc: float = 0.1          # 1/s accumulator reinforcement: dh/dt = k_acc * (inflow - outflow)
    outlet: float = 30.0        # %   outlet (consumption) valve opening
    # test signals
    sys_osc_f: float = 0.15
    sys_osc_gain: float = 0.0
    rnd_osc_f: float = 0.1
    rnd_osc_gain: float = 0.0
    cons_f: float = 0.2
    cons_gain: float = 0.0
    noise_f: float = 5.0
    noise_gain: float = 0.0
    # level switches (true = level REACHED the mark, unless inverted)
    h_high: float = 80.0
    h_low: float = 20.0
    hysteresis: float = 2.0
    lsl_inverted: bool = False  # True: T_LSL10 is true while the level is ABOVE the low mark (covered)


class BandNoise:
    """Random signal in -1..1: a new target every 1/f s, cosine-interpolated (smooth, band-limited)."""

    def __init__(self, rng: random.Random):
        self.rng = rng
        self.a = rng.uniform(-1, 1)
        self.b = rng.uniform(-1, 1)
        self.ph = 0.0

    def step(self, dt: float, f: float) -> float:
        if f <= 0:
            return 0.0
        self.ph += dt * f
        while self.ph >= 1.0:
            self.ph -= 1.0
            self.a, self.b = self.b, self.rng.uniform(-1, 1)
        w = 0.5 - 0.5 * math.cos(math.pi * self.ph)
        return self.a + (self.b - self.a) * w


class LevelProcess:
    SUB_DT = 0.002

    def __init__(self, p: Params | None = None, level: float = 40.0, seed: int | None = None):
        self.p = p or Params()
        self.rng = random.Random(seed)
        self.h = level                 # true level, %
        self.x1 = 0.0                  # 2nd-order block output (inflow), %
        self.x2 = 0.0                  # its derivative
        self.act = 0.0                 # actuator lag state
        self.buf: deque[float] = deque()
        self.t = 0.0
        self.u = 0.0                   # valve command, %
        self.u_delayed = 0.0
        self.q_in = 0.0
        self.q_out = 0.0
        self.noise_val = 0.0
        self.lsh = False
        self.lsl = False
        self._cons = BandNoise(self.rng)
        self._noise = BandNoise(self.rng)
        self._ro_f = BandNoise(self.rng)
        self._ro_a = BandNoise(self.rng)
        self._ro_phase = 0.0
        self._so_phase = 0.0
        self.cons_pulse = 0.0          # extra outflow for pulse tests (%)
        self.cons_pulse_left = 0.0
        self._u_init = 0.0
        self.spilled = 0.0             # total volume lost over the rim (in level %, 100 = one full vessel)
        self.q_over = 0.0              # current spill flow (%)
        self.overflow_hold = 0.0
        self.overflowing = False
        self._update_switches()

    # --- structure information
    def order(self) -> int:
        return 3 + (1 if self.p.t_act > 0 else 0)

    def poles(self) -> list[complex]:
        p = self.p
        wn = 2 * math.pi * p.fn
        z = p.zeta
        d = wn * cmath.sqrt(complex(z * z - 1.0))
        out = [0j, complex(-z * wn) + d, complex(-z * wn) - d]
        if p.t_act > 0:
            out.append(complex(-1.0 / p.t_act))
        return out

    def transfer_function(self) -> str:
        p = self.p
        wn = 2 * math.pi * p.fn
        k = p.gain * p.k_acc * wn * wn
        den = "s*(s^2 + %.4f*s + %.4f)" % (2 * p.zeta * wn, wn * wn)
        if p.t_act > 0:
            den += "*(%.3f*s + 1)" % p.t_act
        return "G(s) = exp(-%.2f*s) * %.5f / ( %s )   [order %d]" % (p.delay, k, den, self.order())

    # --- control
    def set_level(self, h: float):
        self.h = min(100.0, max(0.0, h))
        self.spilled = 0.0
        self._update_switches()

    def trim(self, u: float):
        """Put the dynamic blocks in steady state for a constant valve command u (no transient)."""
        self._u_init = u
        self.buf.clear()
        self.act = u
        self.x1, self.x2 = self.p.gain * u, 0.0

    def pulse(self, amount: float, seconds: float):
        self.cons_pulse, self.cons_pulse_left = amount, seconds

    # --- simulation
    def _update_switches(self):
        p = self.p
        h = self.h
        hy = p.hysteresis
        # high mark: reached at h >= h_high, released below h_high - hysteresis
        if h >= p.h_high or (self.lsh and h > p.h_high - hy):
            self.lsh = True
        else:
            self.lsh = False
        # low mark: reached (true) at h <= h_low, released above h_low + hysteresis
        reached = getattr(self, "_low_reached", False)
        if h <= p.h_low:
            reached = True
        elif h > p.h_low + hy:
            reached = False
        self._low_reached = reached
        self.lsl = (not reached) if p.lsl_inverted else reached

    def _deriv(self, x1, x2, h, u_in, dist_in, q_in_extra, q_out):
        """State derivatives (x1, x2 = 2nd-order block, h = level)."""
        p = self.p
        wn = 2 * math.pi * p.fn
        dx1 = x2
        dx2 = wn * wn * (p.gain * (u_in + dist_in) - x1) - 2 * p.zeta * wn * x2
        q = x1 + q_in_extra
        dh = p.k_acc * (q - q_out)
        if h <= 0.0 and dh < 0:                 # empty (the top is open: above 100 % the vessel overflows, see _sub)
            dh = 0.0
        return dx1, dx2, dh

    def step(self, dt: float, u_cmd: float):
        """Advance by dt seconds with valve command u_cmd (%, clamped 0..100)."""
        p = self.p
        self.u = min(100.0, max(0.0, u_cmd))
        n = max(1, int(math.ceil(dt / self.SUB_DT)))
        h = dt / n
        for _ in range(n):
            self._sub(h)
        self.t += dt
        self.noise_val = p.noise_gain * self._noise.step(dt, p.noise_f)
        self._update_switches()

    def _sub(self, dt: float):
        p = self.p
        # dead time (resolution = sub step), then optional actuator lag
        n = int(round(p.delay / dt)) if p.delay > 0 else 0
        if not self.buf:
            self.buf.extend([self._u_init] * (n + 1))
        self.buf.append(self.u)
        while len(self.buf) > n + 1:
            self.buf.popleft()
        ud = self.buf[0]            # oldest sample (after a delay increase the buffer grows: the oldest we have)
        self.u_delayed = ud
        if p.t_act > 0:
            self.act += dt / p.t_act * (ud - self.act)
            u_in = self.act
        else:
            self.act = ud
            u_in = ud
        # disturbances
        self._so_phase += 2 * math.pi * p.sys_osc_f * dt
        dist_in = p.sys_osc_gain * math.sin(self._so_phase)
        rf = max(0.0, p.rnd_osc_f * (1.0 + 0.5 * self._ro_f.step(dt, 0.2)))
        self._ro_phase += 2 * math.pi * rf * dt
        ra = 0.5 * (1.0 + self._ro_a.step(dt, 0.3))
        q_extra = p.rnd_osc_gain * ra * math.sin(self._ro_phase)
        cons = p.cons_gain * self._cons.step(dt, p.cons_f)
        if self.cons_pulse_left > 0:
            cons += self.cons_pulse
            self.cons_pulse_left -= dt
        q_out = max(0.0, p.outlet + cons)
        # RK4 on (x1, x2, h)
        s = (self.x1, self.x2, self.h)
        k1 = self._deriv(s[0], s[1], s[2], u_in, dist_in, q_extra, q_out)
        s2 = tuple(a + 0.5 * dt * b for a, b in zip(s, k1))
        k2 = self._deriv(s2[0], s2[1], s2[2], u_in, dist_in, q_extra, q_out)
        s3 = tuple(a + 0.5 * dt * b for a, b in zip(s, k2))
        k3 = self._deriv(s3[0], s3[1], s3[2], u_in, dist_in, q_extra, q_out)
        s4 = tuple(a + dt * b for a, b in zip(s, k3))
        k4 = self._deriv(s4[0], s4[1], s4[2], u_in, dist_in, q_extra, q_out)
        self.x1, self.x2, self.h = (a + dt / 6.0 * (b + 2 * c + 2 * d + e)
                                    for a, b, c, d, e in zip(s, k1, k2, k3, k4))
        if self.h > 100.0:                                  # open top: what does not fit spills over the rim
            self.spilled += self.h - 100.0
            self.q_over = (self.h - 100.0) / dt / p.k_acc if p.k_acc > 0 else 0.0
            self.overflow_hold = 0.2
        else:
            self.overflow_hold = max(0.0, self.overflow_hold - dt)
            self.q_over = 0.0
        self.h = min(100.0, max(0.0, self.h))
        self.overflowing = self.overflow_hold > 0.0
        self.q_in = self.x1 + q_extra
        self.q_out = q_out

    @property
    def measured(self) -> float:
        """Level as the transmitter reports it (true level + noise), %."""
        return self.h + self.noise_val
