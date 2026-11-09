"""Closed loop without OPC UA: physics + sensor (noise, delay) + the PLC controller."""
import math
import random

from ballplate.mock_plc import CascadePD
from ballplate.sim import Sensor, build_model
from ballplate import config


def simulate(pattern, T, ki=0.15, noise=0.3, delay=20, x0=0.10, y0=-0.08):
    random.seed(1)
    cfg = config.load()
    m = build_model(cfg)
    m.reset(x0, y0)
    sen = Sensor(noise, delay)
    cx, cy = CascadePD(ki=ki), CascadePD(ki=ki)
    t, errs, maxabs = 0.0, [], 0.0
    for _ in range(int(T / 0.01)):
        mx, my = sen.sample(t, m.s.x, m.s.y)
        sx, sy, svx, svy = pattern(t)
        m.set_tilt_setpoint_deg(cx.update(sx, mx, 0.01, svx), cy.update(sy, my, 0.01, svy))
        for _ in range(10):
            m.step(0.001)
        t += 0.01
        errs.append(math.hypot(m.s.x - sx, m.s.y - sy))
        maxabs = max(maxabs, abs(m.s.x), abs(m.s.y))
    return errs, maxabs, m


def test_holds_centre_from_offset_start():
    errs, _, _ = simulate(lambda t: (0.0, 0.0, 0.0, 0.0), 20)
    assert max(errs[-500:]) < 0.004       # last 5 s within 4 mm
    assert sum(errs[-500:]) / 500 < 0.002


def test_tracks_circle():
    w = 2 * math.pi / 12
    errs, maxabs, m = simulate(lambda t: (0.1 * math.cos(w * t), 0.1 * math.sin(w * t), -0.1 * w * math.sin(w * t), 0.1 * w * math.cos(w * t)), 40)
    assert sum(errs[-1000:]) / 1000 < 0.01
    assert maxabs < 0.18       # never hit a wall (limit 0.18)


def test_recovers_from_wall_bounce():
    errs, _, m = simulate(lambda t: (0.0, 0.0, 0.0, 0.0), 30, x0=0.17, y0=0.17)
    assert sum(errs[-500:]) / 500 < 0.004
