import math
import pytest
from ballplate.physics import BallPlate, Params, G


def run(bp, t, dt=0.001):
    for _ in range(int(t / dt)):
        bp.step(dt)


def make(**kw):
    p = Params(tilt_tau=1e-6, tilt_rate_deg_s=1e6, c_rr=0.0, **kw)
    return BallPlate(p=p)


def test_rolling_acceleration_is_5_7_g_sin_theta():
    bp = make(table_size=100.0)
    th = math.radians(5)
    bp.set_tilt_setpoint_deg(5, 0)
    run(bp, 0.5)
    a = bp.s.vx / 0.5
    # exact: a = 5/7 * g * tan/(1+tan^2) = 5/7 g sin cos
    assert a == pytest.approx(5 / 7 * G * math.sin(th) * math.cos(th), rel=0.02)
    assert not bp.s.slipping
    # pure rolling: contact point at rest
    assert bp.s.vx == pytest.approx(bp.p.ball_radius * bp.s.wy, rel=1e-3)


def test_slips_when_slope_exceeds_static_friction():
    # tan(theta) > 3.5 mu_s  ->  slip.  mu_s=0.05 -> ~10 deg
    bp = BallPlate(p=Params(table_size=100.0, mu_s=0.05, mu_k=0.04,
                            tilt_max_deg=30, tilt_tau=1e-6, tilt_rate_deg_s=1e6, c_rr=0))
    bp.set_tilt_setpoint_deg(20, 0)
    run(bp, 0.3)
    assert bp.s.slipping
    th = math.radians(20)
    k = 1 + math.tan(th) ** 2
    a_expect = G * math.tan(th) / k - 0.04 * G / math.sqrt(k)
    assert bp.s.vx / 0.3 == pytest.approx(a_expect, rel=0.03)


def test_slides_then_rolls_after_initial_slip():
    # ball launched sliding (no spin) on a flat table must end up rolling
    bp = make(table_size=100.0)
    bp.s.vx = 1.0
    run(bp, 2.0)
    assert not bp.s.slipping
    assert bp.s.vx == pytest.approx(bp.p.ball_radius * bp.s.wy, rel=1e-3)
    # classic result: final speed = 5/7 of initial
    assert bp.s.vx == pytest.approx(5 / 7, rel=0.01)


@pytest.mark.parametrize("e", [1.0, 0.5, 0.0])
def test_normal_restitution(e):
    bp = make(e_wall=e, mu_wall=0.0)
    lim = 0.5 * bp.p.table_size - bp.p.ball_radius
    bp.reset(x=lim - 1e-6, vx=1.0)
    bp.s.wy = 0.0
    bp.step(0.001)  # this step crosses the wall; friction effect over 1 ms is tiny
    assert bp.s.vx == pytest.approx(-e * 1.0, abs=0.02)


def test_elastic_frictionless_wall_conserves_energy():
    bp = make(e_wall=1.0, mu_wall=0.0, mu_s=0.0, mu_k=0.0)  # no table friction
    bp.reset(x=0.0, y=0.0, vx=0.8, vy=0.3)
    e0 = bp.kinetic_energy()
    run(bp, 1.6, dt=0.0005)
    assert bp.last_impact_speed > 0
    assert bp.kinetic_energy() == pytest.approx(e0, rel=0.02)


def test_oblique_wall_hit_with_friction_creates_spin():
    bp = make(e_wall=0.8, mu_wall=0.3)
    lim = 0.5 * bp.p.table_size - bp.p.ball_radius
    bp.s.x, bp.s.vx, bp.s.vy = lim - 1e-4, 1.0, 0.5
    bp.step(0.001)
    assert bp.s.vx < 0
    assert abs(bp.s.wz) > 1.0           # picked up vertical-axis spin
    assert bp.s.vy < 0.5                # tangential speed reduced by friction


def test_ball_stays_on_table():
    bp = BallPlate()
    bp.reset(0.1, 0.1, 2.0, 1.5)
    bp.set_tilt_setpoint_deg(10, -10)
    lim = 0.5 * bp.p.table_size - bp.p.ball_radius
    for _ in range(20000):
        bp.step(0.001)
        assert abs(bp.s.x) <= lim + 1e-9 and abs(bp.s.y) <= lim + 1e-9


def test_flat_table_rolling_resistance_stops_ball():
    bp = BallPlate(p=Params(table_size=100.0))
    bp.reset(0, 0, vx=0.5)
    run(bp, 0.5 / (0.002 * G) + 1.0)
    assert math.hypot(bp.s.vx, bp.s.vy) < 1e-3


def test_actuator_rate_limit():
    bp = BallPlate()
    bp.set_tilt_setpoint_deg(15, 0)
    run(bp, 0.1)
    assert math.degrees(bp.s.tilt_x) <= 0.1 * bp.p.tilt_rate_deg_s + 1e-6


# ----------------------------------------------------------- new in v2
def test_default_geometry_matches_real_table():
    p = Params()
    assert p.table_size == 2.0 and p.ball_radius == 0.06 and p.wall_height == 0.12
    assert p.wall_thickness == 0.003


def test_hold_velocity_forces_rolling_velocity():
    bp = make(table_size=100.0)
    bp.hold_vx, bp.hold_vy = 0.3, -0.1
    run(bp, 0.5)
    assert bp.s.vx == pytest.approx(0.3) and bp.s.vy == pytest.approx(-0.1)
    assert bp.s.x == pytest.approx(0.15, abs=0.01)
    bp.hold_vx = bp.hold_vy = None
    run(bp, 0.5)                       # released: keeps rolling (no friction decel)
    assert bp.s.vx == pytest.approx(0.3, abs=0.01)


def test_hold_position_pins_ball_and_is_clamped_by_walls():
    bp = make()
    bp.set_tilt_setpoint_deg(10, 10)
    bp.hold_x, bp.hold_y = 0.25, -0.4
    run(bp, 0.3)
    assert (bp.s.x, bp.s.y) == pytest.approx((0.25, -0.4))
    bp.hold_x = 5.0                    # beyond the wall: wall clamps it
    run(bp, 0.01)
    assert bp.s.x == pytest.approx(0.5 * bp.p.table_size - bp.p.ball_radius)


def test_walls_removed_ball_falls_off_and_center_restores():
    bp = make(walls_enabled=False)
    bp.reset(0.9, 0.0, vx=0.8)
    run(bp, 0.5)
    assert bp.s.off_table and bp.s.z < -0.5
    z0 = bp.s.z
    run(bp, 5.0)
    assert bp.s.z >= -bp.p.fall_depth - 1e-9 and bp.s.z < z0
    bp.center()
    assert not bp.s.off_table
    assert (bp.s.x, bp.s.y, bp.s.vx, bp.s.vy, bp.s.z) == (0, 0, 0, 0, 0)
    run(bp, 0.1)
    assert abs(bp.s.x) < 1e-9


def test_velocity_limit_clamps_components():
    bp = make(table_size=100.0, limit_velocity=True, v_min=-0.4, v_max=0.4)
    bp.set_tilt_setpoint_deg(15, -15)
    run(bp, 3.0)
    assert bp.s.vx == pytest.approx(0.4) and bp.s.vy == pytest.approx(-0.4)
