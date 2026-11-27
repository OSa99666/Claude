"""Imperfections, faults, metrics, sticky patch, release/kick and their UI plumbing."""
import json
import math

import pytest

from ballplate import config
from ballplate.physics import BallPlate, Params
from ballplate.sim import Simulator
from ballplate.testaids import (FaultCfg, Faults, Imperfections, ImperfectionPipeline, Metrics)
from ballplate.ui_link import UiLink

CYCLE = 0.01


def make_sim():
    cfg = config.load()
    cfg["visual"].update(enabled=False)
    sim = Simulator(cfg)
    sim.model.reset(0.3, 0.2)
    return sim


# ---------------------------------------------------------------- imperfections
def test_feedback_delay_noise_quantisation():
    pipe = ImperfectionPipeline(Imperfections(delay_ms=50.0), seed=1)
    outs = [pipe.feedback((i, 0, 0, 0, 0, 0), CYCLE)[0] for i in range(20)]
    assert outs[10] == 5 and outs[19] == 14                       # 50 ms = 5 cycles late
    noisy = ImperfectionPipeline(Imperfections(noise_pos_mm=10.0), seed=2)
    xs = [noisy.feedback((0, 0, 0, 0, 0, 0), CYCLE)[2] for _ in range(4000)]
    mean = sum(xs) / len(xs)
    sd = math.sqrt(sum((x - mean) ** 2 for x in xs) / len(xs))
    assert abs(mean) < 0.001 and 0.008 < sd < 0.012               # std dev 10 mm
    q = ImperfectionPipeline(Imperfections(quant_bits=8))
    assert q.quantise(1234, 32000) % 256 == 0 and q.quantise(12, 32000) == 0
    assert ImperfectionPipeline().quantise(1234, 32000) == 1234   # 16 bits = off


def test_tilt_dead_time_backlash_offset():
    pipe = ImperfectionPipeline(Imperfections(dead_ms=30.0, backlash_deg=2.0, offset_x_deg=0.5))
    seq = [pipe.tilt(10.0, 0.0, CYCLE)[0] for _ in range(8)]
    assert seq[0] == pytest.approx(0.5) and seq[2] == pytest.approx(0.5)    # nothing arrives during the 3 dead cycles
    assert seq[-1] == pytest.approx(10.0 - 1.0 + 0.5)             # half the play lost, plus the offset
    up = pipe.tilt(10.0, 0.0, CYCLE)[0]
    for _ in range(5):
        down = pipe.tilt(9.5, 0.0, CYCLE)[0]
    assert down == up                                              # inside the play: output does not move


# ---------------------------------------------------------------- faults
def test_feedback_freeze_and_actuator_stick_and_kick():
    sim = make_sim()
    sim.model.reset(0.3, 0.2, 0.1, 0.0)
    w1 = sim.output_words()
    sim.last_raw = w1
    sim.faults.freeze_now()
    sim.model.reset(-0.5, -0.4, -0.2, 0.0)
    assert sim.output_words() == w1                                # frozen: old words
    sim.faults.time += 2.0                                         # freeze over
    assert sim.output_words() != w1
    f = Faults(FaultCfg(stick_s=1.5, kick_v=0.25), seed=3)
    f.stick_now()
    assert f.stuck
    f.time += 2.0
    assert not f.stuck
    f.kick_now()
    dvx, dvy = f.take_kick()
    assert math.hypot(dvx, dvy) == pytest.approx(0.25) and f.take_kick() is None


def test_random_faults_use_own_seed():
    a, b = Faults(FaultCfg(kick_per_min=600.0), seed=5), Faults(FaultCfg(kick_per_min=600.0), seed=5)
    ka, kb = [], []
    for _ in range(300):
        a.step(CYCLE); b.step(CYCLE)
        ka.append(a.take_kick()); kb.append(b.take_kick())
    assert ka == kb and any(k is not None for k in ka)


# ---------------------------------------------------------------- metrics
def test_metrics_settling_overshoot_zone():
    m = Metrics()
    m.set_target(1.0, 0.0, 0.05)
    m.start((0.0, 0.0))
    x = 0.0
    for i in range(600):                                            # moves to 1.1 m, returns, settles at 1.0
        t = i * CYCLE
        x = 1.0 + 0.1 * math.exp(-t) * math.cos(6.0 * t) - 0.1 * math.exp(-4 * t) * 0.0 if t > 1.0 else t * 1.1
        m.update(CYCLE, x, 0.0)
    s = m.state()
    assert s["running"] and s["overshoot_pct"] > 5.0 and s["in_zone_pct"] > 30.0
    assert s["settled"] and s["settle_time"] > 0.5 and s["rms"] > 0.0
    m.start((0.0, 0.0))
    m.update(CYCLE, 0.0, 0.0)
    assert not m.state()["settled"] and m.state()["in_zone_pct"] == 0.0


# ---------------------------------------------------------------- physics: patch, release, kick
def run_roll(patch: bool, secs=3.0):
    m = BallPlate(Params())
    m.reset(-0.4, 0.0)
    m.patch_on, m.patch_x, m.patch_y, m.patch_r, m.patch_drag = patch, 0.0, 0.0, 0.3, 0.3
    m.set_tilt_setpoint_deg(3.0, 0.0)
    for _ in range(int(secs / 0.001)):
        m.step(0.001)
    return m.s.x


def test_sticky_patch_slows_the_ball():
    assert run_roll(False) > run_roll(True) + 0.05


def test_kick_and_release():
    m = BallPlate(Params())
    m.reset(0.1, 0.1, 0.2, -0.1)
    assert (m.s.vx, m.s.vy) == (0.2, -0.1) and m.s.wy == pytest.approx(0.2 / 0.06)
    m.kick(0.1, 0.1)
    assert (m.s.vx, m.s.vy) == pytest.approx((0.3, 0.0))


# ---------------------------------------------------------------- UI plumbing
def test_ui_values_and_buttons_reach_the_simulator():
    sim = make_sim()
    sim.ui = UiLink()
    base = {"imp_noise_pos": 5.0, "imp_delay_ms": 40.0, "imp_quant_bits": 10, "imp_dead_ms": 20.0, "imp_backlash": 1.0,
            "imp_off_x": 0.3, "f_kick_per_min": 2.0, "f_freeze_s": 2.5, "patch_on": True, "patch_x": -0.3, "patch_r": 0.2,
            "tgt_x": 0.5, "tgt_y": -0.5, "tgt_r": 0.1, "rel_x": -0.6, "rel_y": 0.4, "rel_vx": 0.1, "rel_vy": 0.0,
            "f_seed": 9, "release_seq": 0, "kick_seq": 0, "freeze_seq": 0, "stick_seq": 0, "f_restart_seq": 0}

    def send(**kw):
        sim.ui.datagram_received(json.dumps(dict(base, **kw)).encode(), None)
        sim.apply_ui()

    send()
    c = sim.imp.c
    assert (c.noise_pos_mm, c.delay_ms, c.quant_bits, c.dead_ms, c.backlash_deg, c.offset_x_deg) == (5.0, 40.0, 10, 20.0, 1.0, 0.3)
    assert sim.model.patch_on and sim.model.patch_x == -0.3
    assert (sim.metrics.tx, sim.metrics.ty, sim.metrics.zone) == (0.5, -0.5, 0.1) and sim.metrics.running
    send(release_seq=1, kick_seq=1, freeze_seq=1, stick_seq=1, f_restart_seq=1)
    s = sim.model.s
    assert (round(s.x, 3), round(s.y, 3), s.vx) == (-0.6, 0.4, 0.1)
    assert sim.faults.kick_pending == 1 and sim.faults.frozen and sim.faults.stuck and sim.seed == 9
