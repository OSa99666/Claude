from valvesim.valve import Valve, ValveParams


def run(v, secs, h=False, l=False, dt=0.01):
    for _ in range(round(secs / dt)):
        v.step(dt, h, l)


def test_open_timing_and_feedbacks():
    v = Valve(ValveParams(t_to_high=3.0, ad_from_low=0.5))
    assert v.feedbacks() == (False, True)
    run(v, 0.4, h=True)
    assert v.feedbacks() == (False, True) and v.pos == 0.0     # still in action delay
    run(v, 0.3, h=True)
    assert v.feedbacks() == (False, False) and 0.0 < v.pos < 0.1   # travelling, both feedbacks false
    run(v, 2.9, h=True)
    assert v.feedbacks() == (False, False) or v.at_high
    run(v, 0.2, h=True)
    assert v.feedbacks() == (True, False)


def test_bistable_and_travel_to_end():
    v = Valve(ValveParams(t_to_high=2.0, ad_from_low=0.0))
    run(v, 1.0, h=True)
    run(v, 3.0)                                # command removed: stroke completes
    assert v.at_high
    run(v, 5.0)                                # and stays
    assert v.at_high


def test_without_travel_to_end_stops():
    v = Valve(ValveParams(t_to_high=2.0, ad_from_low=0.0, travel_to_end=False))
    run(v, 1.0, h=True)
    p = v.pos
    run(v, 2.0)
    assert 0.4 < p < 0.6 and abs(v.pos - p) < 1e-9


def test_reverse_mid_travel_without_delay():
    v = Valve(ValveParams(t_to_high=2.0, t_to_low=2.0, ad_from_low=0.0, ad_from_high=5.0))
    run(v, 1.0, h=True)
    p = v.pos
    run(v, 0.1, l=True)
    assert v.pos < p
    run(v, 2.0, l=True)
    assert v.at_low


def test_conflict_keeps_previous_command():
    v = Valve(ValveParams(t_to_high=1.0, ad_from_low=0.0))
    run(v, 0.5, h=True)
    run(v, 2.0, h=True, l=True)
    assert v.at_high


def test_safeguard_and_force():
    v = Valve(ValveParams(ad_from_low=0.0, safeguard_low=True))
    run(v, 2.0, h=True)
    assert v.at_low                             # blocked from leaving Low
    v.p.safeguard_low = False
    run(v, 4.0, h=True)
    assert v.at_high
    v.p.force_false = True
    assert v.feedbacks() == (False, False)
    v.p.force_true = True
    assert v.feedbacks() == (False, False)      # force false wins
    v.p.force_false = False
    assert v.feedbacks() == (True, True)
