"""Rigid-body model of a solid ball on a square tilting table.

Frame: table frame, x/y in the table plane, z = table normal (up).
Tilt convention (chosen so the PLC programmer never has to think about
rotation axes): ``TiltX`` > 0 makes the surface slope *down towards +X*,
so the ball accelerates towards +X.  Same for ``TiltY`` and +Y.

Contact model
-------------
* Ball/table: Coulomb friction with separate static (mu_s) and kinetic
  (mu_k) coefficients.  Every step we compute the friction force that would
  bring the contact-point slip velocity to exactly zero (rolling without
  slipping).  If that force is within the static limit mu_s*N the ball
  sticks (rolls); otherwise it slips with kinetic friction mu_k*N.
* Rolling resistance: constant deceleration c_rr*g while rolling.
* Ball/wall: normal restitution ``e`` (1 = elastic, 0 = fully inelastic)
  and tangential Coulomb friction mu_w limited by the sticking impulse,
  which couples to the vertical-axis spin (omega_z).  So a ball hitting a
  wall at an angle leaves with spin and a non-mirror exit angle.

Extras used by the PLC-facing simulator
---------------------------------------
* ``walls_enabled = False`` removes the walls; a ball whose centre leaves the
  table falls off (ballistic fall, ``off_table`` = True) until ``center()``.
* ``hold_vx/hold_vy`` force the ball velocity (rolling-consistent spin) and
  ``hold_x/hold_y`` pin the ball position while they are not None.
* ``limit_velocity`` clamps each velocity component to [v_min, v_max].
* ``patch_on`` adds a sticky circular patch (centre ``patch_x/patch_y``, radius ``patch_r``): inside it the rolling
  resistance is increased by ``patch_drag`` (in g, i.e. a deceleration of patch_drag * 9.81 m/s^2).
* ``kick(dvx, dvy)`` adds a velocity impulse (a push); ``reset(x, y, vx, vy)`` releases the ball with a velocity.

Solid sphere: I = 2/5 m R^2, so the effective tangential mass at a contact
is 2m/7 (this is where the famous 5/7 g sin(theta) comes from).
"""
from __future__ import annotations

import math
from dataclasses import dataclass, field

G = 9.81


@dataclass
class Params:
    table_size: float = 2.0       # m, square side length
    ball_radius: float = 0.06     # m (12 cm diameter)
    ball_mass: float = 0.5        # kg (cancels out of the motion; kept for N, F)
    wall_height: float = 0.12     # m (visual only)
    wall_thickness: float = 0.003 # m (visual only; the physical wall is the table edge)
    mu_s: float = 0.35            # ball/table static friction
    mu_k: float = 0.25            # ball/table kinetic friction
    c_rr: float = 0.002           # rolling resistance (decel = c_rr * g)
    e_wall: float = 0.6           # wall restitution 0..1
    mu_wall: float = 0.2          # ball/wall friction
    e_rest_speed: float = 0.02    # m/s, impacts slower than this are inelastic
    spin_damping: float = 1.0     # 1/s, decay of vertical-axis spin
    walls_enabled: bool = True
    limit_velocity: bool = False  # clamp each velocity component to [v_min, v_max]
    v_min: float = -0.4           # m/s
    v_max: float = 0.4
    fall_depth: float = 1.5       # m, a ball that left the table stops this far below it
    # actuator
    tilt_max_deg: float = 15.0
    tilt_rate_deg_s: float = 60.0
    tilt_tau: float = 0.05        # s, first-order lag


@dataclass
class State:
    x: float = 0.0
    y: float = 0.0
    vx: float = 0.0
    vy: float = 0.0
    wx: float = 0.0   # angular velocity, rad/s, table frame
    wy: float = 0.0
    wz: float = 0.0
    q: tuple = (1.0, 0.0, 0.0, 0.0)   # ball orientation (w, x, y, z)
    tilt_x: float = 0.0   # rad, actual
    tilt_y: float = 0.0
    slipping: bool = False
    z: float = 0.0        # height above the table plane (only != 0 when off the table)
    vz: float = 0.0
    off_table: bool = False


def _quat_integrate(q, wx, wy, wz, dt):
    """q_dot = 0.5 * (0, w) (x) q   with w expressed in the fixed frame."""
    w, x, y, z = q
    dw = 0.5 * (-wx * x - wy * y - wz * z)
    dx = 0.5 * (wx * w + wy * z - wz * y)
    dy = 0.5 * (wy * w + wz * x - wx * z)
    dz = 0.5 * (wz * w + wx * y - wy * x)
    w += dw * dt
    x += dx * dt
    y += dy * dt
    z += dz * dt
    n = math.sqrt(w * w + x * x + y * y + z * z)
    return (w / n, x / n, y / n, z / n)


@dataclass
class BallPlate:
    p: Params = field(default_factory=Params)
    s: State = field(default_factory=State)
    tilt_sp_x: float = 0.0   # rad, commanded
    tilt_sp_y: float = 0.0
    last_impact_speed: float = 0.0
    hold_vx: float | None = None
    hold_vy: float | None = None
    hold_x: float | None = None
    hold_y: float | None = None
    patch_on: bool = False          # sticky patch on the plate (extra rolling resistance inside a circle)
    patch_x: float = 0.4
    patch_y: float = 0.3
    patch_r: float = 0.25
    patch_drag: float = 0.15

    # ------------------------------------------------------------------ API
    def reset(self, x=0.0, y=0.0, vx=0.0, vy=0.0):
        self.s = State(x=x, y=y, vx=vx, vy=vy,
                       tilt_x=self.s.tilt_x, tilt_y=self.s.tilt_y)
        # start in pure rolling consistent with the given velocity
        R = self.p.ball_radius
        self.s.wy = vx / R
        self.s.wx = -vy / R

    def center(self):
        """Ball to the table centre, at rest, back on the table."""
        self.reset(0.0, 0.0)
        self.s.wx = self.s.wy = self.s.wz = 0.0

    def kick(self, dvx: float, dvy: float):
        """A push: velocity change of the ball (rolling-consistent spin)."""
        s, R = self.s, self.p.ball_radius
        if s.off_table:
            return
        s.vx += dvx
        s.vy += dvy
        s.wy = s.vx / R
        s.wx = -s.vy / R

    def in_patch(self) -> bool:
        s = self.s
        return self.patch_on and math.hypot(s.x - self.patch_x, s.y - self.patch_y) <= self.patch_r

    def set_tilt_setpoint_deg(self, tx_deg: float, ty_deg: float):
        m = self.p.tilt_max_deg
        self.tilt_sp_x = math.radians(max(-m, min(m, tx_deg)))
        self.tilt_sp_y = math.radians(max(-m, min(m, ty_deg)))

    def step(self, dt: float):
        self._step_actuator(dt)
        if self.s.off_table:
            self._step_fall(dt)
            return
        self._step_ball(dt)
        self._apply_limits_and_holds()
        self._step_walls()

    # ------------------------------------------------------------ internals
    def _step_actuator(self, dt):
        p, s = self.p, self.s
        max_step = math.radians(p.tilt_rate_deg_s) * dt
        alpha = 1.0 - math.exp(-dt / p.tilt_tau) if p.tilt_tau > 0 else 1.0
        for attr, sp in (("tilt_x", self.tilt_sp_x), ("tilt_y", self.tilt_sp_y)):
            cur = getattr(s, attr)
            d = max(-max_step, min(max_step, (sp - cur) * alpha))  # lag + rate limit
            setattr(s, attr, cur + d)

    def _step_ball(self, dt):
        p, s = self.p, self.s
        R, m = p.ball_radius, p.ball_mass
        I = 0.4 * m * R * R

        # gravity projected on the (possibly doubly) tilted plane
        tx, ty = math.tan(s.tilt_x), math.tan(s.tilt_y)
        k = 1.0 + tx * tx + ty * ty
        agx, agy = G * tx / k, G * ty / k
        N = m * G / math.sqrt(k)

        # contact-point slip velocity
        ux = s.vx - R * s.wy
        uy = s.vy + R * s.wx

        # Friction force that would zero the slip velocity this step:
        #   du/dt = a_g + (7/2) F/m  ->  F = -(2m/7) (u/dt + a_g)
        fx = -(2.0 * m / 7.0) * (ux / dt + agx)
        fy = -(2.0 * m / 7.0) * (uy / dt + agy)
        fmag = math.hypot(fx, fy)
        if fmag <= p.mu_s * N:
            s.slipping = False
        else:
            s.slipping = True
            us = math.hypot(ux, uy)
            if us > 1e-12:
                fx, fy = -p.mu_k * N * ux / us, -p.mu_k * N * uy / us
            else:  # exactly zero slip but above static limit: slide down-slope
                ag = math.hypot(agx, agy)
                if ag > 1e-12:
                    fx, fy = -p.mu_k * N * agx / ag, -p.mu_k * N * agy / ag
                else:
                    fx = fy = 0.0

        # semi-implicit Euler
        s.vx += (agx + fx / m) * dt
        s.vy += (agy + fy / m) * dt
        s.wx += (R * fy / I) * dt
        s.wy += (-R * fx / I) * dt
        s.wz *= max(0.0, 1.0 - p.spin_damping * dt)

        c_rr = p.c_rr + (self.patch_drag if self.in_patch() else 0.0)
        if not s.slipping and c_rr > 0.0:
            sp = math.hypot(s.vx, s.vy)
            if sp > 0.0:
                f = max(0.0, 1.0 - c_rr * G * dt / sp)
                s.vx *= f; s.vy *= f; s.wx *= f; s.wy *= f

        s.x += s.vx * dt
        s.y += s.vy * dt
        s.q = _quat_integrate(s.q, s.wx, s.wy, s.wz, dt)

    def _apply_limits_and_holds(self):
        p, s = self.p, self.s
        R = p.ball_radius
        if p.limit_velocity:
            s.vx = max(p.v_min, min(p.v_max, s.vx))
            s.vy = max(p.v_min, min(p.v_max, s.vy))
        if self.hold_vx is not None:
            s.vx = self.hold_vx
            s.wy = s.vx / R          # keep rolling-consistent spin
        if self.hold_vy is not None:
            s.vy = self.hold_vy
            s.wx = -s.vy / R
        if self.hold_x is not None:
            s.x = self.hold_x
        if self.hold_y is not None:
            s.y = self.hold_y

    def _step_fall(self, dt):
        """Ball that left the table: free fall, keeps its horizontal velocity."""
        s = self.s
        s.x += s.vx * dt
        s.y += s.vy * dt
        s.vz -= G * dt
        s.z += s.vz * dt
        if s.z < -self.p.fall_depth:
            s.z = -self.p.fall_depth
            s.vx = s.vy = s.vz = 0.0
        s.q = _quat_integrate(s.q, s.wx, s.wy, s.wz, dt)

    def _step_walls(self):
        p, s = self.p, self.s
        half = 0.5 * p.table_size
        if not p.walls_enabled:
            if abs(s.x) > half or abs(s.y) > half:   # centre over the edge: it falls
                s.off_table = True
                s.vz = 0.0
                s.z = 0.0
            return
        lim = half - p.ball_radius
        # (outward normal) for the four walls; x walls first, then y walls
        if s.x > lim:
            s.x = lim;  self._wall_impact(1.0, 0.0)
        elif s.x < -lim:
            s.x = -lim; self._wall_impact(-1.0, 0.0)
        if s.y > lim:
            s.y = lim;  self._wall_impact(0.0, 1.0)
        elif s.y < -lim:
            s.y = -lim; self._wall_impact(0.0, -1.0)

    def _wall_impact(self, nx, ny):
        """Impulse response against a wall with outward normal (nx, ny)."""
        p, s = self.p, self.s
        R, m = p.ball_radius, p.ball_mass
        vn = s.vx * nx + s.vy * ny
        if vn <= 0.0:
            return  # moving away already
        tx, ty = -ny, nx
        vt = s.vx * tx + s.vy * ty
        e = p.e_wall if vn > p.e_rest_speed else 0.0
        jn = (1.0 + e) * m * vn                 # normal impulse magnitude
        # tangential: slip velocity of the contact point on the wall
        ut = vt + R * s.wz
        jt = -(2.0 * m / 7.0) * ut              # impulse that would stick
        jmax = p.mu_wall * jn
        jt = max(-jmax, min(jmax, jt))
        # apply
        dvn = -(1.0 + e) * vn
        s.vx += dvn * nx + (jt / m) * tx
        s.vy += dvn * ny + (jt / m) * ty
        s.wz += R * jt / (0.4 * m * R * R)
        self.last_impact_speed = vn

    # --------------------------------------------------------- diagnostics
    def kinetic_energy(self):
        s, p = self.s, self.p
        I = 0.4 * p.ball_mass * p.ball_radius ** 2
        return (0.5 * p.ball_mass * (s.vx ** 2 + s.vy ** 2)
                + 0.5 * I * (s.wx ** 2 + s.wy ** 2 + s.wz ** 2))
