"""Traffic intersection model: two crossing roads (direction 1 = west-east, direction 2 = north-south),
one lane per travel direction, straight-through traffic only, pedestrian crossings on all four arms.

Coordinates: x east, y south (screen), metres, intersection centre = (0, 0).
Arms (the side a vehicle comes FROM):  W (drives east), E (drives west), N (drives south), S (drives north).
  direction 1 = arms W, E  (traffic light group TL1, detectors XTD11 = W, XTD12 = E)
  direction 2 = arms N, S  (traffic light group TL2, detectors XTD21 = N, XTD22 = S)
Pedestrians: group 1 walks along direction 1 (east-west) over the crosswalks of the north and south arms, i.e. over
the direction-2 road: pedestrian light PL1 may be green while TL1 is green.  Group 2 is the other way round.

A vehicle is described by ``s`` = position of its FRONT along its travel direction (road centre s = 0, spawn at -ROAD_L).
Output ``F`` (tag I_XF) is a hardware fault of the traffic light system (a lamp that does not light); a collision is NOT reported
to the PLC, it only latches ``crash`` (shown in Godot) until ``clear_fault()``.

Cars follow the intelligent-driver model (IDM) with the signal / pedestrian crossing as a standing obstacle at the stop line.
"""
from __future__ import annotations

import math
import random
from dataclasses import dataclass, field

ROAD_L = 60.0           # road half length (spawn / despawn)
HALF = 4.0              # half width of the intersection box (road half width)
LANE = 2.0              # lane centre offset
STOP = -8.0             # stop line (front of the car) before the crosswalk
CW0, CW1 = 4.4, 7.4     # crosswalk spans |s| in [CW0, CW1]
DET0, DET1 = -11.0, -8.0  # detector loop on the stop-line side (car body overlaps it)
CAR_W = 1.9
PED_R = 0.4

ARMS = ("W", "E", "N", "S")
GROUP_OF_ARM = {"W": 1, "E": 1, "N": 2, "S": 2}
DET_OF_ARM = {"W": "XTD11", "E": "XTD12", "N": "XTD21", "S": "XTD22"}
APPROACH_CW = {"W": "W", "E": "E", "N": "N", "S": "S"}      # crosswalk a car meets first
CW_GROUP = {"N": 1, "S": 1, "W": 2, "E": 2}                 # pedestrian group using the crosswalk
LAMPS = ("TL1R", "TL1Y", "TL1G", "TL2R", "TL2Y", "TL2G", "PL1R", "PL1G", "PL2R", "PL2G")
DETECTORS = ("XTD11", "XTD12", "XTD21", "XTD22")


def car_center(arm: str, s: float, length: float) -> tuple[float, float]:
    m = s - length / 2.0
    return {"W": (m, LANE), "E": (-m, -LANE), "N": (-LANE, m), "S": (LANE, -m)}[arm]


def car_half(arm: str, length: float) -> tuple[float, float]:
    return (length / 2.0, CAR_W / 2.0) if arm in ("W", "E") else (CAR_W / 2.0, length / 2.0)


def crosswalk_interval(arm: str, cw: str) -> tuple[float, float] | None:
    """s-interval (front coordinate) in which a car of ``arm`` is on crosswalk ``cw``."""
    approach = {"W": "W", "E": "E", "N": "N", "S": "S"}[arm]
    exit_ = {"W": "E", "E": "W", "N": "S", "S": "N"}[arm]
    if cw == approach:
        return (-CW1, -CW0)
    if cw == exit_:
        return (CW0, CW1)
    return None


def ped_pos(cw: str, u: float, d: int) -> tuple[float, float]:
    a = -HALF + 2 * HALF * (u if d > 0 else 1.0 - u)
    mid = (CW0 + CW1) / 2.0
    return {"N": (a, -mid), "S": (a, mid), "W": (-mid, a), "E": (mid, a)}[cw]


@dataclass
class Params:
    rate: dict = field(default_factory=lambda: {"W": 6.0, "E": 6.0, "N": 6.0, "S": 6.0})   # vehicles / min and arm
    v0: float = 10.0                 # desired speed m/s
    a_max: float = 2.0
    b_comf: float = 3.0              # comfortable braking
    b_max: float = 6.0               # strongest braking a driver tries
    t_gap: float = 1.2               # time headway
    s0: float = 2.0                  # standstill gap
    car_len: float = 4.5
    runner_prob: float = 0.0         # probability that a new car ignores the signal (0..1)
    ped_rate: dict = field(default_factory=lambda: {1: 4.0, 2: 4.0})    # pedestrians / min and group
    ped_speed: float = 1.3
    traffic_on: bool = True
    broken: set = field(default_factory=set)       # lamp names that do not light
    det_dead: set = field(default_factory=set)     # detectors stuck false
    det_stuck: set = field(default_factory=set)    # detectors stuck true
    btn_stuck: set = field(default_factory=set)    # pushbuttons 1/2 held pressed
    hw_fault: bool = False                         # force I_XF (hardware fault) without a broken lamp


@dataclass
class Car:
    cid: int
    arm: str
    s: float
    v: float
    length: float
    runner: bool = False
    go: bool = False                 # committed to pass the stop line
    crashed: bool = False
    crossed: bool = False
    stopped_t: float = 0.0
    t_spawn: float = 0.0


@dataclass
class Ped:
    pid: int
    cw: str
    d: int
    state: str = "wait"              # wait / cross / hit
    u: float = 0.0
    wait_t: float = 0.0
    press_t: float = 0.0


class Intersection:
    def __init__(self, p: Params | None = None, seed: int | None = None):
        self.p = p or Params()
        self.seed = seed
        self.rng = random.Random(seed)
        self.t = 0.0
        self.cars: list[Car] = []
        self.peds: list[Ped] = []
        self.next_id = 1
        self.lit = {n: False for n in LAMPS}          # physically lit (command and not broken)
        self.det = {n: False for n in DETECTORS}
        self.btn = {1: False, 2: False}
        self.btn_timer = {1: 0.0, 2: 0.0}
        self.crash = False
        self.stats = self._new_stats()
        self.manual_feed: list[tuple[float, str]] = []   # (time, arm) scripted arrivals (tests)

    @staticmethod
    def _new_stats():
        return {"cars_passed": 0, "car_wait_total": 0.0, "peds_crossed": 0, "ped_wait_total": 0.0,
                "crashes": 0, "violations": 0, "max_queue": 0}

    def reseed(self, seed: int | None):
        self.seed = seed
        self.rng = random.Random(seed)

    def reset_stats(self):
        self.stats = self._new_stats()

    def hardware_fault(self) -> bool:
        """I_XF: any traffic light hardware error = a broken lamp (or the forced fault from the UI)."""
        return bool(self.p.broken) or self.p.hw_fault

    def clear_fault(self):
        self.cars = [c for c in self.cars if not c.crashed]
        self.peds = [q for q in self.peds if q.state != "hit"]
        self.crash = False

    # ---------------------------------------------------------------- signals
    def signal(self, group: int) -> str:
        l = self.lit
        r, y, g = (l["TL%dR" % group], l["TL%dY" % group], l["TL%dG" % group])
        if r:
            return "R"
        if y:
            return "Y"
        if g:
            return "G"
        return "R"                                   # dark head: drivers stop

    def _ped_on_crosswalk(self, cw: str) -> bool:
        return any(q.cw == cw and q.state == "cross" for q in self.peds)

    # ------------------------------------------------------------------- cars
    def _must_stop(self, c: Car) -> bool:
        """Does the car treat the stop line as an obstacle (and has it not yet committed to pass)?"""
        if c.go or c.crashed or c.s > STOP + 0.2:
            return False
        d = STOP - c.s
        v = c.v
        brake_possible = d > v * v / (2 * self.p.b_max)
        st = self.signal(GROUP_OF_ARM[c.arm])
        stop = False
        if c.runner:
            stop = False
        elif st == "R":
            stop = True
        elif st == "Y":
            stop = d > v * v / (2 * self.p.b_comf)
        if self._ped_on_crosswalk(APPROACH_CW[c.arm]):
            stop = True
        if stop and not brake_possible:
            c.go = True
            if st == "R":
                self.stats["violations"] += 1
            return False
        return stop

    def _idm(self, c: Car, gap: float, dv: float) -> float:
        p = self.p
        gap = max(gap, 0.1)
        s_star = p.s0 + max(0.0, c.v * p.t_gap + c.v * dv / (2 * math.sqrt(p.a_max * p.b_comf)))
        return p.a_max * (1 - (c.v / p.v0) ** 4 - (s_star / gap) ** 2)

    def _step_cars(self, dt: float):
        p = self.p
        for arm in ARMS:
            lane = sorted((c for c in self.cars if c.arm == arm), key=lambda c: -c.s)   # front car first
            for i, c in enumerate(lane):
                if c.crashed:
                    continue
                acc = p.a_max * (1 - (c.v / p.v0) ** 4)
                if i > 0:
                    lead = lane[i - 1]
                    acc = min(acc, self._idm(c, lead.s - lead.length - c.s, c.v - lead.v))
                if self._must_stop(c):
                    acc = min(acc, self._idm(c, STOP - c.s + p.s0 - 0.3, c.v))
                acc = max(acc, -9.0)
                c.v = max(0.0, c.v + acc * dt)
                c.s += c.v * dt
                if c.v < 0.3 and c.s < STOP + 1.0:
                    c.stopped_t += dt
                if not c.crossed and c.s > STOP:
                    c.crossed = True
        # despawn
        keep = []
        for c in self.cars:
            if c.s - c.length > ROAD_L and not c.crashed:
                self.stats["cars_passed"] += 1
                self.stats["car_wait_total"] += c.stopped_t
            else:
                keep.append(c)
        self.cars = keep

    def _spawn(self, dt: float):
        p = self.p
        if not p.traffic_on:
            return
        for arm in ARMS:
            if self.rng.random() < p.rate.get(arm, 0.0) / 60.0 * dt:
                near = [c for c in self.cars if c.arm == arm and c.s - c.length < -ROAD_L + 12.0]
                if near:
                    continue
                self._add_car(arm)

    def _add_car(self, arm: str) -> Car:
        c = Car(self.next_id, arm, -ROAD_L, self.p.v0, self.p.car_len,
                runner=self.rng.random() < self.p.runner_prob, t_spawn=self.t)
        self.next_id += 1
        self.cars.append(c)
        return c

    def add_car(self, arm: str, s: float = -ROAD_L, v: float | None = None, runner=False) -> Car:
        """Insert a car by hand (tests / scenarios)."""
        c = self._add_car(arm)
        c.s = s
        c.v = self.p.v0 if v is None else v
        c.runner = runner
        return c

    # ------------------------------------------------------------ pedestrians
    def _spawn_peds(self, dt: float):
        p = self.p
        for g in (1, 2):
            if self.rng.random() < p.ped_rate.get(g, 0.0) / 60.0 * dt:
                cw = self.rng.choice([k for k, v in CW_GROUP.items() if v == g])
                self.peds.append(Ped(self.next_id, cw, self.rng.choice((-1, 1))))
                self.next_id += 1

    def add_ped(self, cw: str, d: int = 1) -> Ped:
        q = Ped(self.next_id, cw, d)
        self.next_id += 1
        self.peds.append(q)
        return q

    def _safe_to_start(self, cw: str) -> bool:
        for c in self.cars:
            if c.crashed:
                continue
            iv = crosswalk_interval(c.arm, cw)
            if iv is None:
                continue
            if c.v < 0.3 and c.s < iv[0] - 0.5:
                continue                                  # waiting at its line
            if c.s > iv[0] - 12.0 and c.s - c.length < iv[1]:
                return False
        return True

    def _step_peds(self, dt: float, ypd: dict):
        p = self.p
        for q in self.peds:
            g = CW_GROUP[q.cw]
            if q.state == "wait":
                q.wait_t += dt
                if self.lit["PL%dG" % g] and not self.lit["PL%dR" % g] and self._safe_to_start(q.cw):
                    q.state = "cross"
                    self.stats["ped_wait_total"] += q.wait_t
                else:
                    q.press_t -= dt
                    if q.press_t <= 0 and not ypd[g]:
                        self.btn_timer[g] = max(self.btn_timer[g], 0.6)
                        q.press_t = 10.0                  # impatient: presses again after 10 s
                    elif q.press_t <= 0:
                        q.press_t = 2.0
            elif q.state == "cross":
                q.u += p.ped_speed * dt / (2 * HALF)
        done = [q for q in self.peds if q.state == "cross" and q.u >= 1.0]
        self.stats["peds_crossed"] += len(done)
        self.peds = [q for q in self.peds if q not in done]
        for g in (1, 2):
            self.btn_timer[g] = max(0.0, self.btn_timer[g] - dt)
            self.btn[g] = self.btn_timer[g] > 0 or g in p.btn_stuck

    # --------------------------------------------------------------- collision
    def _check_collisions(self):
        # rear-end collisions inside a lane: the follower stops at the rear of the car in front (no overlap), both are crashed
        for arm in ARMS:
            lane = sorted((c for c in self.cars if c.arm == arm), key=lambda c: -c.s)
            for lead, foll in zip(lane, lane[1:]):
                if lead.s - lead.length - foll.s < 0.0 and not (lead.crashed and foll.crashed):
                    foll.s = lead.s - lead.length - 0.02
                    for c in (lead, foll):
                        c.crashed, c.v = True, 0.0
                    self._crash()
        live = [c for c in self.cars if True]
        for i, a in enumerate(live):
            for b in live[i + 1:]:
                if GROUP_OF_ARM[a.arm] == GROUP_OF_ARM[b.arm] or (a.crashed and b.crashed):
                    continue
                ax, ay = car_center(a.arm, a.s, a.length)
                bx, by = car_center(b.arm, b.s, b.length)
                ahx, ahy = car_half(a.arm, a.length)
                bhx, bhy = car_half(b.arm, b.length)
                if abs(ax - bx) < ahx + bhx and abs(ay - by) < ahy + bhy:
                    for c in (a, b):
                        if not c.crashed:
                            c.crashed, c.v = True, 0.0
                    self._crash()
        for q in self.peds:
            if q.state != "cross":
                continue
            px, py = ped_pos(q.cw, q.u, q.d)
            for c in self.cars:
                cx, cy = car_center(c.arm, c.s, c.length)
                hx, hy = car_half(c.arm, c.length)
                if abs(px - cx) < hx + PED_R and abs(py - cy) < hy + PED_R:
                    q.state = "hit"
                    c.crashed, c.v = True, 0.0
                    self._crash()
                    break

    def _crash(self):
        if not self.crash:
            self.stats["crashes"] += 1
        self.crash = True

    # ------------------------------------------------------------------- step
    def queue_lengths(self) -> dict:
        return {arm: sum(1 for c in self.cars if c.arm == arm and c.v < 0.3 and c.s < STOP + 1.0) for arm in ARMS}

    def step(self, dt: float, cmd: dict) -> dict:
        """cmd: PLC outputs by short name ('TL1R', ..., 'PD1A', 'PD2A').  Returns the PLC inputs (short names)."""
        for n in LAMPS:
            self.lit[n] = bool(cmd.get(n, False)) and n not in self.p.broken
        ypd = {1: bool(cmd.get("PD1A", False)), 2: bool(cmd.get("PD2A", False))}
        while self.manual_feed and self.manual_feed[0][0] <= self.t:
            self._add_car(self.manual_feed.pop(0)[1])
        self._spawn(dt)
        self._spawn_peds(dt)
        self._step_cars(dt)
        self._step_peds(dt, ypd)
        self._check_collisions()
        self.t += dt
        q = self.queue_lengths()
        self.stats["max_queue"] = max(self.stats["max_queue"], max(q.values()))
        for arm in ARMS:
            name = DET_OF_ARM[arm]
            on = any(c.s > DET0 and c.s - c.length < DET1 for c in self.cars if c.arm == arm)
            self.det[name] = (name in self.p.det_stuck) or (on and name not in self.p.det_dead)
        out = {}
        for n in LAMPS:
            out[n] = self.lit[n]
        out.update(self.det)
        out["PD1"], out["PD2"] = self.btn[1], self.btn[2]
        out["F"] = self.hardware_fault()                    # I_XF: error in the traffic light hardware, not a collision
        return out
