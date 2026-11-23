import pytest

from transportsim import config
from transportsim.mock_plc import Sorter
from transportsim.transport import COLOURS, Pef, Transport


def make(seed=1):
    return Transport(Pef.from_cfg(config.load()), seed)


def run(m, secs, dt=0.005, plc=None):
    for _ in range(round(secs / dt)):
        if plc:
            o = plc.update(m.outputs() | {"T_" + k: v for k, v in m.outputs().items()})
            m.cmd = dict(yh=[o["T_YH1%d" % n] for n in range(1, 6)], yr=[o["T_YR1%d" % n] for n in range(1, 6)],
                         ej_yh=[o["T_YH2%d" % n] for n in range(1, 5)], ej_yl=[o["T_YL2%d" % n] for n in range(1, 5)])
        m.step(dt)


def start_all(m, yr=32767):
    m.cmd = dict(yh=[True] * 5, yr=[yr] * 5, ej_yh=[False] * 4, ej_yl=[False] * 4)


def test_motor_start_delay_and_speed_scaling():
    m = make()
    start_all(m)
    run(m, 0.05)
    assert not m.outputs()["XGH11"]                       # start delay 100 ms
    run(m, 0.1)
    assert m.outputs()["XGH11"]
    assert m.vel[0] == pytest.approx(1 / 0.015)           # full speed: 1 tick per 15 ms
    start_all(m, 0)
    run(m, 0.02)
    assert m.vel[0] == pytest.approx(1 / 0.080)           # YR=0 is the minimum speed, not a stop
    start_all(m, 16384)
    run(m, 0.02)
    assert 1 / 0.080 < m.vel[0] < 1 / 0.015
    # X11 full-scale mapping of the library: 66.7 ticks/s of 666.7 max -> ~10 %
    start_all(m)
    run(m, 0.02)
    assert m.outputs()["X11"] == pytest.approx(3276, abs=30)


def test_no_motion_when_motor_off():
    m = make()
    m.feed_single()
    run(m, 1.0)
    assert m.t1 == []                                    # entry only happens while T1 runs
    start_all(m)
    run(m, 0.3)
    assert len(m.t1) == 1 and m.t1[0].pos > 0
    m.cmd["yh"][0] = False
    run(m, 0.05)
    p = m.t1[0].pos
    run(m, 1.0)
    assert m.t1[0].pos == p


def test_item_runs_to_store1_and_untreated_type():
    m = make()
    m.f.xe = False
    m.f.xe01 = True
    start_all(m)
    m.t1.append(__import__("transportsim.transport", fromlist=["Item"]).Item(5, 0.0))
    run(m, 22.0)                                          # 1140 ticks at 66.7 ticks/s = 17 s
    assert m.t1 == [] and m.store[0] == 5 and m.outputs()["BQ01"]
    m.empty_store(1)
    assert not m.outputs()["BQ01"]


def test_detector_and_front_flags():
    m = make()
    m.f.xe = False
    from transportsim.transport import Item
    start_all(m)
    run(m, 0.2)
    m.t1.append(Item(3, 0.0))
    seen_xcq = seen_det = seen_front = False
    for _ in range(4000):
        m.step(0.005)
        o = m.outputs()
        seen_det |= o["XCQ03"]
        seen_xcq |= o["XCQ"] == COLOURS[3]
        seen_front |= o["XCQ11"]
        assert not (o["XCQ01"] or o["XCQ02"] or o["XCQ04"])
    assert seen_det and seen_xcq and seen_front


def test_ejector_blocks_and_sorts():
    m = make()
    from transportsim.transport import Item
    m.f.xe = False
    start_all(m)
    run(m, 0.2)
    m.cmd["ej_yh"][0] = True                              # extend ejector 1: it acts as a barrier
    run(m, 0.5)
    assert m.ejectors[0].at_high
    m.t1.append(Item(1, 100.0))
    m.t1.append(Item(2, 0.0))
    run(m, 12.0)
    ej = m.ej_pos(0)
    pos = sorted(i.pos for i in m.t1)
    assert pos[1] == pytest.approx(ej - 60, abs=3) and pos[0] == pytest.approx(ej - 120, abs=3)   # stacked 60 apart
    m.cmd["ej_yh"][0] = False
    m.cmd["ej_yl"][0] = True                              # retract: items move on
    run(m, 2.5)
    assert max(i.pos for i in m.t1) > ej


def test_sorter_end_to_end():
    m = make(seed=3)
    m.f.xe = True
    plc = Sorter()
    run(m, 90.0, plc=plc)
    assert sum(plc.sorted) >= 8
    # every item that reached a side store was of the matching type
    for k in range(4):
        for it in m.side[k]:
            assert it.id == k + 1
    for k in range(4):
        if m.store[k + 1]:
            assert m.store[k + 1] == k + 1


def test_no_overlap_when_stack_is_released_early():
    """Regression: items queued behind an ejector must stay one item apart, also after the front one is ejected."""
    m = make(seed=3)
    plc = Sorter()
    for _ in range(round(90 / 0.005)):
        io = {"T_" + k: v for k, v in m.outputs().items()}
        o = plc.update(io)
        m.cmd = dict(yh=[o["T_YH1%d" % n] for n in range(1, 6)], yr=[o["T_YR1%d" % n] for n in range(1, 6)],
                     ej_yh=[o["T_YH2%d" % n] for n in range(1, 5)], ej_yl=[o["T_YL2%d" % n] for n in range(1, 5)])
        m.step(0.005)
        ps = sorted(i.pos for i in m.t1)
        assert all(b - a > 59 for a, b in zip(ps, ps[1:]))
