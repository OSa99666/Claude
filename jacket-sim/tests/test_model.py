import math

from jacketsim.mock_plc import Controller
from jacketsim.model import (A, B, D, RAW_HIGH, X_INIT, JacketTank, Params, raw_to_u, y_to_raw)


def run(m, seconds, raw1, raw2, d1=0.0, d2=0.0, dt=1.0):
    for _ in range(int(round(seconds / dt))):
        m.step(dt, raw1, raw2, d1, d2)


def heater(pct):
    return pct / 100.0 * RAW_HIGH


def solve(u, d=(0.0, 0.0)):
    """Steady state of the model with x4 at its lower limit (4 degC) - independent of the simulation code."""
    x4 = 4.0
    # (I - A) x = B u + D d + A[:,3] x4 for x1..x3 (x4 fixed by the clamp)
    rhs = [sum(B[i][j] * u[j] for j in range(2)) + sum(D[i][j] * d[j] for j in range(2)) + A[i][3] * x4 for i in range(3)]
    M = [[(1.0 if i == j else 0.0) - A[i][j] for j in range(3)] for i in range(3)]
    for c in range(3):                                              # Gaussian elimination
        p = max(range(c, 3), key=lambda r: abs(M[r][c]))
        M[c], M[p], rhs[c], rhs[p] = M[p], M[c], rhs[p], rhs[c]
        for r in range(c + 1, 3):
            f = M[r][c] / M[c][c]
            M[r] = [a - f * b for a, b in zip(M[r], M[c])]
            rhs[r] -= f * rhs[c]
    x = [0.0] * 3
    for r in (2, 1, 0):
        x[r] = (rhs[r] - sum(M[r][c] * x[c] for c in range(r + 1, 3))) / M[r][r]
    return x


def test_scaling_like_the_codesys_block():
    assert raw_to_u(0, 0) == 0 and abs(raw_to_u(32767, 0) - 100.0) < 1e-9 and abs(raw_to_u(32767, 1) - 1.0) < 1e-9
    assert raw_to_u(-500, 0) == 0 and raw_to_u(40000, 1) == 1.0                    # clamped
    assert y_to_raw(20.0, 0) == 0 and y_to_raw(120.0, 0) == 32767 and y_to_raw(70.0, 0) == 16384
    assert y_to_raw(0.1, 1) == 0 and y_to_raw(0.9, 1) == 32767 and y_to_raw(0.5, 1) == 16384
    assert y_to_raw(500.0, 0) == 32767 and y_to_raw(-5.0, 0) == 0                  # clamped


def test_initial_state_and_first_samples_match_the_hand_calculation():
    m = JacketTank()
    assert m.x == X_INIT and m.raw == [16384, 16384] and abs(m.y_true[0] - 70.0) < 1e-12
    m.step(1.0, heater(26.0), 0.25 * RAW_HIGH)
    # x1 = 0.92*70 + 0.05*60 - 0.0003*0.5 + 0.03*20 + 0.16*26 - 0.05*0.25
    assert abs(m.x[0] - (0.92 * 70 + 0.05 * 60 - 0.0003 * 0.5 + 0.03 * 20 + 0.16 * 26 - 0.05 * 0.25)) < 1e-9
    assert abs(m.x[1] - (0.04 * 70 + 0.88 * 60 + 0.01 * 26)) < 1e-9
    assert abs(m.x[2] - (0.95 * 0.5 + 0.1 * 0.25)) < 1e-9
    assert abs(m.x[3] - 18.0) < 1e-9
    m.step(1.0, heater(26.0), 0.25 * RAW_HIGH)
    assert abs(m.x[3] - 16.2) < 1e-9


def test_outputs_only_change_at_the_sample_instants():
    m = JacketTank(Params(sample_time_s=1.0))
    m.step(0.4, heater(50), 8000)
    m.step(0.4, heater(50), 8000)
    assert m.samples == 0 and m.raw == [16384, 16384]
    m.step(0.4, heater(50), 8000)
    assert m.samples == 1 and m.raw[0] != 16384


def test_steady_state_equals_the_linear_solution_and_x4_hits_its_limit():
    m = JacketTank()
    run(m, 1500, heater(26.0), 0.25 * RAW_HIGH)
    x = solve([26.0, 0.25 * RAW_HIGH / RAW_HIGH])
    assert m.x[3] == 4.0
    for a, b in zip(m.x[:3], x):
        assert abs(a - b) < 1e-6
    assert 69.0 < m.x[0] < 69.2 and abs(m.x[2] - 0.5) < 1e-9


def test_state_limits_and_disturbances():
    m = JacketTank()
    run(m, 400, RAW_HIGH, RAW_HIGH)
    assert m.x[0] <= 120.0 and m.x[2] == 0.9                               # level at its limit
    m = JacketTank()
    run(m, 400, 0, 0)
    assert m.x[2] < 1e-6 and m.x[0] >= 2.0                                  # empty, cooled but not below the limit
    m = JacketTank()
    run(m, 400, heater(26), 0.25 * RAW_HIGH, d2=0.1)                       # level disturbance: x3 = 2*u2 - d2
    assert abs(m.x[2] - (0.5 - 0.1)) < 1e-6
    m = JacketTank()
    run(m, 1500, heater(26), 0.25 * RAW_HIGH, d1=10.0)                     # heating disturbance cools via D11 = -0.02
    ref = JacketTank()
    run(ref, 1500, heater(26), 0.25 * RAW_HIGH)
    assert m.x[0] < ref.x[0] - 1.0


def test_aids_dead_time_heater_limit_mismatch_noise_freeze():
    a, b = JacketTank(), JacketTank(Params(dead_time_samples=3))
    for k in range(6):
        a.step(1.0, heater(60), 8000)
        b.step(1.0, heater(60), 8000)
    assert a.x[0] > 100 and b.x[0] < a.x[0] - 10 and b.x[0] < 90          # the delayed plant reacts three samples later
    m = JacketTank(Params(heater_max=20.0))
    run(m, 800, RAW_HIGH, 8192)
    assert abs(m.u[0] - 20.0) < 1e-9 and m.x[0] < 60
    m1, m2 = JacketTank(), JacketTank(Params(b_scale_heater=0.5))
    run(m1, 800, heater(26), 8192)
    run(m2, 800, heater(26), 8192)
    assert m2.x[0] < m1.x[0] - 15
    m = JacketTank(Params(noise_temp=2.0, noise_level=0.02), seed=5)
    raws = []
    for _ in range(60):
        m.step(1.0, heater(26), 8192)
        raws.append(m.raw[0])
    assert max(raws) - min(raws) > 300 and abs(m.y_meas[0] - m.y_true[0]) <= 2.0
    m2 = JacketTank(Params(noise_temp=2.0, noise_level=0.02), seed=5)
    r2 = []
    for _ in range(60):
        m2.step(1.0, heater(26), 8192)
        r2.append(m2.raw[0])
    assert raws == r2                                                       # repeatable with a seed
    m = JacketTank(Params(freeze_sensors=True))
    frozen = list(m.raw)
    run(m, 30, RAW_HIGH, 8192)
    assert m.raw == frozen and m.y_true[0] > 80                             # plant moves, measurement does not


def test_reset_and_set_state_and_pulse():
    m = JacketTank()
    run(m, 50, RAW_HIGH, 0)
    m.reset()
    assert m.x == X_INIT
    m.set_state(temp=90.0, level=0.8)
    assert m.x[0] == 90.0 and m.x[2] == 0.8 and m.raw == [y_to_raw(90.0, 0), y_to_raw(0.8, 1)]
    m = JacketTank()
    m.pulse_d2(0.2, 3.0)
    run(m, 5, heater(26), 0.25 * RAW_HIGH)
    assert m.x[2] < 0.5                                                     # the pulse drained some level
    run(m, 200, heater(26), 0.25 * RAW_HIGH)
    assert abs(m.x[2] - 0.5) < 1e-3                                         # and it is over


def test_two_pi_loops_settle_on_both_setpoints_and_reject_a_disturbance():
    m = JacketTank()
    c = Controller(80.0, 0.6)
    raw1 = raw2 = 0
    for k in range(500):
        raw1, raw2 = c.step(m.raw[0], m.raw[1], 1.0)
        m.step(1.0, raw1, raw2, 0.0, 0.0 if k < 300 else 0.15)           # level disturbance from k = 300
    assert abs(m.y_true[0] - 80.0) < 0.2 and abs(m.y_true[1] - 0.6) < 0.01
