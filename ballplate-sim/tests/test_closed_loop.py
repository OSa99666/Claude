"""Closed loop without OPC UA: physics + INT quantisation + the PLC controller."""
import math

from ballplate import config
from ballplate.mock_plc import CascadePD
from ballplate.scaling import from_raw, to_raw
from ballplate.sim import build_model


def simulate(pattern, T, x0=0.4, y0=-0.3, ki=0.15):
    cfg = config.load()
    sc = cfg["scaling"]
    d, rm = sc["dist_m"], sc["raw_max"]
    m = build_model(cfg)
    m.reset(x0, y0)
    cx, cy = CascadePD(ki=ki), CascadePD(ki=ki)
    t, errs, maxabs = 0.0, [], 0.0
    for _ in range(int(T / 0.01)):
        # what the PLC sees: distance as a raw INT word
        mx = from_raw(to_raw(m.s.x, -d, d, rm), -d, d, rm)
        my = from_raw(to_raw(m.s.y, -d, d, rm), -d, d, rm)
        sx, sy, svx, svy = pattern(t)
        m.set_tilt_setpoint_deg(cx.update(sx, mx, 0.01, svx), cy.update(sy, my, 0.01, svy))
        for _ in range(10):
            m.step(0.001)
        t += 0.01
        errs.append(math.hypot(m.s.x - sx, m.s.y - sy))
        maxabs = max(maxabs, abs(m.s.x), abs(m.s.y))
    return errs, maxabs, m


def test_holds_centre_from_offset_start():
    errs, _, _ = simulate(lambda t: (0.0, 0.0, 0.0, 0.0), 40)
    assert max(errs[-500:]) < 0.01
    assert sum(errs[-500:]) / 500 < 0.005


def test_tracks_circle():
    w = 2 * math.pi / 24
    r = 0.5
    errs, maxabs, _ = simulate(lambda t: (r * math.cos(w * t), r * math.sin(w * t),
                                          -r * w * math.sin(w * t), r * w * math.cos(w * t)), 80)
    assert sum(errs[-2000:]) / 2000 < 0.03
    assert maxabs < 0.94 - 0.001      # never touched a wall


def test_recovers_from_wall_bounce():
    errs, _, _ = simulate(lambda t: (0.0, 0.0, 0.0, 0.0), 60, x0=0.9, y0=0.9)
    assert sum(errs[-500:]) / 500 < 0.01
