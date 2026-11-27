import math

from levelsim.mock_plc import PI, OnOff
from levelsim.process import LevelProcess, Params


def settle(p, u, seconds=60.0, dt=0.01):
    for _ in range(int(seconds / dt)):
        p.step(dt, u)


def test_order_and_poles():
    p = LevelProcess(Params())
    assert p.order() == 3
    poles = p.poles()
    assert len(poles) == 3 and abs(poles[0]) == 0                      # integrator
    wn = 2 * math.pi * p.p.fn
    assert abs(abs(poles[1]) - wn) < 1e-9                              # |pole| = wn
    p.p.t_act = 2.0
    assert p.order() == 4 and len(p.poles()) == 4
    assert "order 4" in p.transfer_function()


def test_balance_holds_level():
    p = LevelProcess(Params(outlet=30.0), level=50.0, seed=1)
    p.trim(30.0)
    settle(p, 30.0, 60)                                                # inflow = outflow: level stays
    assert abs(p.h - 50.0) < 1e-6


def test_integrator_rate_matches_k_acc():
    p = LevelProcess(Params(outlet=0.0, delay=0.0), level=10.0, seed=1)
    p.trim(10.0)
    settle(p, 10.0, 10)
    h0 = p.h
    settle(p, 10.0, 10)
    assert abs((p.h - h0) / 10.0 - p.p.k_acc * 10.0) < 0.01            # dh/dt = k_acc * inflow


def test_dead_time():
    p = LevelProcess(Params(delay=2.0, outlet=0.0), level=10.0, seed=1)
    settle(p, 100.0, 1.9)
    assert p.x1 < 1e-6 and abs(p.h - 10.0) < 1e-6                      # nothing happened yet
    settle(p, 100.0, 0.6)
    assert p.x1 > 1e-3                                                 # response starts after 2 s


def test_resonance_and_saturation():
    p = LevelProcess(Params(delay=0.0, outlet=50.0, zeta=0.1, fn=0.2), level=50.0, seed=1)
    p.trim(50.0)
    xs = []
    for _ in range(int(40 / 0.01)):
        p.step(0.01, 60.0)
        xs.append(p.x1)
    assert max(xs) > 66.0                                         # underdamped overshoot of the 2nd-order block
    settle(p, 100.0, 500)
    assert p.h == 100.0 and p.lsh                                       # full and clamped
    settle(p, 0.0, 600)
    assert p.h >= 0.0


def test_switches_hysteresis_and_inversion():
    p = LevelProcess(Params(h_high=80, h_low=20, hysteresis=2), level=50)
    assert (p.lsh, p.lsl) == (False, False)
    p.set_level(85)
    assert p.lsh and not p.lsl
    p.set_level(79)
    assert p.lsh                                                        # still held inside the hysteresis band
    p.set_level(77)
    assert not p.lsh
    p.set_level(15)
    assert p.lsl
    p.set_level(21)
    assert p.lsl
    p.set_level(30)
    assert not p.lsl
    p.p.lsl_inverted = True
    p.set_level(30)
    assert p.lsl and not p.lsh                                          # covered switch: true above the mark
    p.set_level(10)
    assert not p.lsl


def test_noise_only_on_measurement():
    p = LevelProcess(Params(noise_gain=2.0, noise_f=10.0, outlet=30), level=50, seed=3)
    settle(p, 30.0, 40)
    ys = []
    for _ in range(2000):
        p.step(0.01, 30.0)
        ys.append(p.measured - p.h)
    assert 0.5 < max(abs(v) for v in ys) <= 2.0
    p2 = LevelProcess(Params(noise_gain=0.0), level=50, seed=3)
    settle(p2, 30.0, 40)
    assert p2.measured == p2.h


def test_seed_repeatable():
    a = LevelProcess(Params(cons_gain=10, cons_f=1, rnd_osc_gain=5, sys_osc_gain=2), level=50, seed=5)
    b = LevelProcess(Params(cons_gain=10, cons_f=1, rnd_osc_gain=5, sys_osc_gain=2), level=50, seed=5)
    settle(a, 40, 30)
    settle(b, 40, 30)
    assert a.h == b.h


def test_closed_loop_pi_settles():
    p = LevelProcess(Params(), level=40.0, seed=1)
    c = PI(2.0, 100.0, dt=0.01)
    for _ in range(int(300 / 0.01)):
        p.step(0.01, c.step(60.0, p.h))
    assert abs(p.h - 60.0) < 0.2


def test_closed_loop_p_gain_too_high_oscillates():
    p = LevelProcess(Params(), level=40.0, seed=1)
    hs = []
    for i in range(int(200 / 0.01)):
        p.step(0.01, 30.0 + 12.0 * (60.0 - p.h))                        # far above the critical gain
        if i > int(150 / 0.01):
            hs.append(p.h)
    assert max(hs) - min(hs) > 5.0


def test_onoff_limit_cycle_between_the_switches():
    p = LevelProcess(Params(), level=50.0, seed=1)
    c = OnOff()
    sw, prev, hs = 0, True, []
    for i in range(int(400 / 0.01)):
        o = c.step(p.lsh, p.lsl)
        p.step(0.01, 60.0 if o else 0.0)
        if i > 5000:
            hs.append(p.h)
        if o != prev:
            sw, prev = sw + 1, o
    assert sw >= 8 and 5 < min(hs) < 25 and 75 < max(hs) < 95


def test_open_top_overflow():
    p = LevelProcess(Params(delay=0.0, outlet=10.0), level=95.0, seed=1)
    p.trim(60.0)
    seen = False
    for _ in range(int(20 / 0.01)):
        p.step(0.01, 60.0)
        seen = seen or p.overflowing
    assert seen and p.h == 100.0 and p.lsh
    assert p.spilled > 20.0
    assert abs(p.q_over - 50.0) < 1.0                                  # spill flow = inflow - outflow (60 - 10)
    # volume balance while spilling: 50 %/s * k_acc = 5 level-%/s
    s0 = p.spilled
    for _ in range(int(10 / 0.01)):
        p.step(0.01, 60.0)
    assert abs((p.spilled - s0) / 10.0 - 5.0) < 0.1
    p.step(0.01, 0.0)
    for _ in range(int(30 / 0.01)):
        p.step(0.01, 0.0)
    assert not p.overflowing and p.h < 100.0
    p.set_level(50.0)
    assert p.spilled == 0.0


def test_high_high_switch():
    p = LevelProcess(Params(h_high=80, h_hh=95, hysteresis=2), level=50)
    assert not p.lshh
    p.set_level(90)
    assert p.lsh and not p.lshh
    p.set_level(96)
    assert p.lshh
    p.set_level(94)
    assert p.lshh                                                       # inside the hysteresis band
    p.set_level(92)
    assert not p.lshh and p.lsh
