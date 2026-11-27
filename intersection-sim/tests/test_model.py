import math

from intersectionsim.mock_plc import FixedTime
from intersectionsim.model import (DET0, DET1, STOP, Intersection, Params)

DT = 0.01


def quiet(**kw):
    p = Params(rate={"W": 0, "E": 0, "N": 0, "S": 0}, ped_rate={1: 0, 2: 0}, **kw)
    return Intersection(p, seed=1)


def run(m, seconds, cmd, dt=DT):
    out = {}
    for _ in range(int(seconds / dt)):
        out = m.step(dt, cmd)
    return out


GREEN1 = {"TL1G": True, "TL2R": True, "PL1R": True, "PL2R": True}
RED = {"TL1R": True, "TL2R": True, "PL1R": True, "PL2R": True}


def test_car_stops_at_red_and_detector_and_goes_on_green():
    m = quiet()
    c = m.add_car("W", s=-40, v=10.0)
    out = run(m, 15, RED)
    assert c.v < 0.01 and STOP - 1.0 < c.s <= STOP + 0.2          # waiting at the stop line
    assert out["XTD11"] and not out["XTD12"] and not out["XTD21"]
    out = run(m, 12, GREEN1)
    assert c.s > 5 and not m.fault
    run(m, 10, GREEN1)
    assert m.stats["cars_passed"] == 1


def test_dark_head_means_stop_and_queue_builds_with_standstill_gap():
    m = quiet()
    a = m.add_car("N", s=-30, v=8.0)
    b = m.add_car("N", s=-50, v=8.0)
    run(m, 25, {})
    assert a.v < 0.01 and b.v < 0.01
    assert 1.5 < (a.s - a.length) - b.s < 3.5                      # standstill gap behind the leader
    assert m.queue_lengths()["N"] == 2                              # both cars stand in the queue


def test_yellow_far_stops_near_continues():
    m = quiet()
    far = m.add_car("W", s=-40, v=10.0)
    near = m.add_car("E", s=-12, v=10.0)
    run(m, 0.5, {"TL1Y": True, "TL2R": True})                       # yellow for direction 1
    run(m, 8, {"TL1Y": True, "TL2R": True})
    assert far.v < 0.5 and far.s < STOP + 0.5                      # far car stops
    assert near.s > 0                                                # near car cannot stop comfortably: continues


def test_conflicting_greens_crash_and_latch_fault():
    m = quiet()
    m.add_car("W", s=-30, v=10.0)
    m.add_car("N", s=-30, v=10.0)
    both = {"TL1G": True, "TL2G": True, "PL1R": True, "PL2R": True}
    out = run(m, 6, both)
    assert out["F"] and m.stats["crashes"] == 1
    run(m, 5, GREEN1)
    assert m.fault                                                   # stays latched
    m.clear_fault()
    assert not m.fault and not m.cars


def test_runner_ignores_red():
    m = quiet()
    c = m.add_car("S", s=-30, v=10.0, runner=True)
    run(m, 6, RED)
    assert c.s > 0


def test_pedestrian_request_waits_for_green_and_crosses():
    m = quiet()
    q = m.add_ped("N", 1)
    run(m, 1.0, RED)
    assert q.state == "wait"
    m2 = quiet()
    q2 = m2.add_ped("S", -1)
    cmd = {"TL1G": True, "TL2R": True, "PL1G": True, "PL2R": True}
    run(m2, 1.0, cmd)
    assert q2.state == "cross"
    run(m2, 8.0, cmd)
    assert m2.stats["peds_crossed"] == 1 and not m2.peds


def test_pushbutton_pulse_and_acknowledge():
    m = quiet()
    m.add_ped("N", 1)
    seen = False
    for _ in range(int(2 / DT)):
        out = m.step(DT, RED)
        seen = seen or out["PD1"]
    assert seen                                                      # button pressed
    # acknowledged: no re-press while the request lamp is lit
    m.btn_timer[1] = 0
    for _ in range(int(15 / DT)):
        out = m.step(DT, dict(RED, PD1A=True))
        assert not out["PD1"]


def test_car_yields_to_crossing_pedestrian():
    m = quiet()
    q = m.add_ped("N", 1)
    q.state, q.u = "cross", 0.1
    c = m.add_car("N", s=-30, v=10.0)
    run(m, 4, {"TL2G": True, "TL1R": True})
    assert c.s <= STOP + 0.5 and not m.fault                         # green, but the pedestrian is on the crosswalk


def test_broken_lamp_feedback_and_detector_faults():
    m = quiet(broken={"TL1G"})
    out = run(m, 0.1, GREEN1)
    assert out["TL1G"] is False                                      # commanded but not lit
    m.p.det_stuck = {"XTD22"}
    assert run(m, 0.1, RED)["XTD22"]
    m.p.det_stuck = set()
    m.p.det_dead = {"XTD11"}
    m.add_car("W", s=-9.5, v=0.0)
    assert not run(m, 0.1, RED)["XTD11"]


def test_seed_repeatable():
    def trace(seed):
        m = Intersection(Params(), seed=seed)
        ctrl = FixedTime()
        pos = []
        for _ in range(int(60 / DT)):
            out = m.step(DT, ctrl.step(DT, {"PD1": m.btn[1], "PD2": m.btn[2]}))
        return [(c.arm, round(c.s, 3)) for c in m.cars], m.stats["cars_passed"]
    assert trace(3) == trace(3)


def test_fixed_time_controller_no_crash_and_throughput():
    m = Intersection(Params(rate={"W": 10, "E": 10, "N": 10, "S": 10}, ped_rate={1: 6, 2: 6}), seed=2)
    ctrl = FixedTime(green=10.0)
    sw = set()
    for _ in range(int(600 / DT)):
        cmd = ctrl.step(DT, {"PD1": m.btn[1], "PD2": m.btn[2]})
        assert not (cmd["TL1G"] and cmd["TL2G"])
        m.step(DT, cmd)
    assert not m.fault and m.stats["crashes"] == 0
    assert m.stats["cars_passed"] > 60 and m.stats["peds_crossed"] > 10
    assert m.stats["violations"] == 0
