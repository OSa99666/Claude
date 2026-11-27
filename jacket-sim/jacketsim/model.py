"""Heated jacketed water tank: MIMO state-space model (level and temperature), a port of the CODESYS block ``fbSsHeatedJacketTank``.

    x(k+1) = clamp( A x(k) + B u(k) + D d(k) ),   y = C x          (one step per sample time PTSDT, default 1 s)

    states   x1 tank water temperature [degC]      inputs  u1 heater power [%]           (raw M_YR01, 0..32767 = 0..100 %)
             x2 heater / jacket wall temp [degC]           u2 inflow valve [0..1]         (raw M_YR02, 0..32767 = 0..1)
             x3 level [m]                           dist.  d1 heating disturbance, d2 level disturbance (LREAL inputs XD01 / XD02)
             x4 inlet / ambient temperature state    outputs y1 = x1 temperature [degC]    (raw M_XG01, 20..120 degC = 0..32767)
                [degC]                                        y2 = x3 level [m]             (raw M_XG02, 0.1..0.9 m   = 0..32767)

The matrices, the state limits (XkMa / XkMi), the initial state (svXi), the scaling ranges and the sample time are the defaults of the
CODESYS block.  NOT ported: the Kalman filter ``fbMpcKalmanFilter`` and ``fcStateSpaceModel`` (their source was not supplied); in the block the
filter is fed with the model's own previous output, so the plant is taken as the plain deterministic model above.  The ST evaluates
y = C x with the new state (the output of the step), and the raw outputs only change at the sample instants (zero-order hold).
Test aids on top (not in the block): measurement noise / bias / freeze, actuator dead time, heater limit, plant-model mismatch.
"""
from __future__ import annotations

import math
import random
from collections import deque
from dataclasses import dataclass, field

A = [[0.92, 0.05, -0.0003, 0.03],
     [0.04, 0.88, 0.0, 0.0],
     [0.0, 0.0, 0.95, 0.0],
     [0.0, 0.0, 0.0, 0.9]]
B = [[0.16, -0.05], [0.01, 0.0], [0.0, 0.1], [0.0, 0.0]]
D = [[-0.02, 0.003], [-0.002, 0.0], [0.0, -0.05], [0.0, 0.0]]
C = [[1.0, 0.0, 0.0, 0.0], [0.0, 0.0, 1.0, 0.0]]
X_MAX = [120.0, 120.0, 0.9, 120.0]
X_MIN = [2.0, 4.0, 0.0, 4.0]
X_INIT = [70.0, 60.0, 0.5, 20.0]
U_HIGH, U_LOW = [100.0, 1.0], [0.0, 0.0]          # engineering range of the two control inputs (PXHR / PXLR)
Y_HIGH, Y_LOW = [120.0, 0.9], [20.0, 0.1]         # engineering range of the two process values (PYHR / PYLR)
RAW_HIGH, RAW_LOW = 32767.0, 0.0                  # raw range (PXHRR / PXLRR, PYHRR / PYLRR)


def raw_to_u(raw: float, i: int) -> float:
    """Raw controller output -> engineering unit of input i (clamped to the input limits), as in the block."""
    v = (U_HIGH[i] - U_LOW[i]) / (RAW_HIGH - RAW_LOW) * (raw - RAW_LOW) + U_LOW[i]
    return min(U_HIGH[i], max(U_LOW[i], v))


def y_to_raw(y: float, i: int) -> int:
    """Process value (degC / m) -> raw integer 0..32767 (clamped, rounded like LREAL_TO_INT)."""
    v = (RAW_HIGH - RAW_LOW) / (Y_HIGH[i] - Y_LOW[i]) * (y - Y_LOW[i]) + RAW_LOW
    v = min(RAW_HIGH, max(RAW_LOW, v))
    return int(math.floor(v + 0.5))


@dataclass
class Params:
    sample_time_s: float = 1.0                 # PTSDT  system discretisation time
    # test aids (all neutral by default)
    noise_temp: float = 0.0                    # degC  measurement noise (uniform +-)
    noise_level: float = 0.0                   # m
    bias_temp: float = 0.0                     # degC  measurement offset
    bias_level: float = 0.0                    # m
    freeze_sensors: bool = False               # measured values stop changing
    dead_time_samples: int = 0                 # actuator dead time in samples
    heater_max: float = 100.0                  # % heater limit (a fault if < 100)
    b_scale_heater: float = 1.0                # plant gain of the heater input vs the model the PLC uses (mismatch test)
    b_scale_inflow: float = 1.0
    a_scale: float = 1.0                       # scales the decay rates: A' = I + a_scale * (A - I)  (plant speed mismatch)
    hold: bool = False                         # freeze the plant


class JacketTank:
    def __init__(self, p: Params | None = None, seed: int | None = None):
        self.p = p or Params()
        self.rng = random.Random(seed)
        self.x = list(X_INIT)
        self.u = [0.0, 0.0]               # inputs as used at the last sample (engineering units)
        self.d = [0.0, 0.0]
        self.acc = 0.0
        self.t = 0.0
        self.samples = 0
        self.y_true = self._y()
        self.y_meas = list(self.y_true)
        self.raw = [y_to_raw(self.y_meas[0], 0), y_to_raw(self.y_meas[1], 1)]
        self._delay: deque[list[float]] = deque()
        self.d2_pulse = 0.0
        self.d2_pulse_left = 0.0

    def _y(self) -> list[float]:
        return [sum(C[i][j] * self.x[j] for j in range(4)) for i in range(2)]

    def reset(self, x: list[float] | None = None):
        self.x = list(x or X_INIT)
        self.acc = 0.0
        self._delay.clear()
        self.y_true = self._y()
        self._measure(force=True)

    def set_state(self, temp: float | None = None, level: float | None = None):
        if temp is not None:
            self.x[0] = min(X_MAX[0], max(X_MIN[0], temp))
        if level is not None:
            self.x[2] = min(X_MAX[2], max(X_MIN[2], level))
        self.y_true = self._y()
        self._measure(force=True)

    def pulse_d2(self, amount: float, seconds: float):
        self.d2_pulse, self.d2_pulse_left = amount, seconds

    def _measure(self, force=False):
        p = self.p
        if p.freeze_sensors and not force:
            return
        n = [p.noise_temp * self.rng.uniform(-1, 1), p.noise_level * self.rng.uniform(-1, 1)]
        self.y_meas = [self.y_true[0] + p.bias_temp + n[0], self.y_true[1] + p.bias_level + n[1]]
        self.raw = [y_to_raw(self.y_meas[0], 0), y_to_raw(self.y_meas[1], 1)]

    def _sample(self, u: list[float], d: list[float]):
        p = self.p
        x = self.x
        nx = []
        for i in range(4):
            v = 0.0
            for j in range(4):
                aij = A[i][j]
                if p.a_scale != 1.0:
                    aij = (1.0 if i == j else 0.0) + p.a_scale * (aij - (1.0 if i == j else 0.0))
                v += aij * x[j]
            v += B[i][0] * u[0] * p.b_scale_heater + B[i][1] * u[1] * p.b_scale_inflow
            v += D[i][0] * d[0] + D[i][1] * d[1]
            nx.append(min(X_MAX[i], max(X_MIN[i], v)))
        self.x = nx
        self.y_true = self._y()
        self._measure()
        self.samples += 1

    def step(self, dt: float, raw_u1: float, raw_u2: float, d1: float = 0.0, d2: float = 0.0):
        """Advance by dt seconds with the controller outputs (raw 0..32767) and the disturbance inputs; the model steps at the sample time."""
        p = self.p
        u = [raw_to_u(raw_u1, 0), raw_to_u(raw_u2, 1)]
        u[0] = min(u[0], p.heater_max)
        self.u = u
        if self.d2_pulse_left > 0:
            d2 += self.d2_pulse
            self.d2_pulse_left -= dt
        self.d = [d1, d2]
        self.t += dt
        if p.hold:
            return
        self.acc += dt
        while self.acc >= p.sample_time_s - 1e-12:
            self.acc -= p.sample_time_s
            uu = list(u)
            if p.dead_time_samples > 0:
                self._delay.append(uu)
                while len(self._delay) > p.dead_time_samples + 1:
                    self._delay.popleft()
                uu = list(self._delay[0]) if len(self._delay) == p.dead_time_samples + 1 else [0.0, 0.0]
            self._sample(uu, self.d)
