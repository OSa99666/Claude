"""Transport / sorting model, a port of the library block fbTransportSimulator.

T1 is the main conveyor, T2..T5 the side conveyors fed by ejectors 1..4, stores 1..5 collect the
items that roll off the ends of T1..T5.  Positions are in "ticks" (60 per item): ``pos`` is the
item's trailing edge, so its centre is pos + ticks_per_item/2.

Speed: each conveyor adds ``animation_ticks`` every animation period; the period runs from
``period_min`` (YR = 32767) to ``period_max`` (YR = 0), linearly in YR - a running conveyor never stops
at YR = 0 (as in the PLC library).  A conveyor only moves while its motor emulator reports "running".
"""
from __future__ import annotations

import random
from dataclasses import dataclass, field

from .twostate import Valve, ValveParams

RAW_MAX = 32767
# item identification colours delivered on T_XCQ (ARGB), item id 1..5
COLOURS = {1: 0xFF000065, 2: 0xFF006500, 3: 0xFF650000, 4: 0xFF656500, 5: 0xFFD0D0D0}
MAX_ANIM_TICKS = 10


@dataclass
class Pef:
    period_min: list = field(default_factory=lambda: [0.015] * 5)
    period_max: list = field(default_factory=lambda: [0.080] * 5)
    anim_ticks: int = 1
    tpi: int = 60
    spacing: int = 30
    motor_start: float = 0.1
    motor_stop: float = 0.0
    ej_extend: float = 0.2
    ej_retract: float = 0.2
    ej_delay_high: float = 1.0
    ej_delay_low: float = 0.1
    ej_travel_to_end: bool = True
    t1_offload_slot: int = 19
    t1_max_items: int = 21
    side_slots: int = 5
    detector_slot: int = 2
    ejector_slots: tuple = (7, 10, 13, 16)

    @classmethod
    def from_cfg(cls, cfg: dict) -> "Pef":
        p, l = cfg["pef"], cfg["layout"]
        return cls(period_min=[x / 1000 for x in p["anim_period_min_ms"]],
                   period_max=[x / 1000 for x in p["anim_period_max_ms"]],
                   anim_ticks=p["animation_ticks"], tpi=p["ticks_per_item"], spacing=p["item_spacing_ticks"],
                   motor_start=p["motor_start_delay_ms"] / 1000, motor_stop=p["motor_stop_delay_ms"] / 1000,
                   ej_extend=p["ejector_extend_ms"] / 1000, ej_retract=p["ejector_retract_ms"] / 1000,
                   ej_delay_high=p["ejector_delay_from_high_ms"] / 1000,
                   ej_delay_low=p["ejector_delay_from_low_ms"] / 1000,
                   ej_travel_to_end=p.get("ejector_travel_to_end", True),
                   t1_offload_slot=l["t1_offload_slot"], t1_max_items=l["t1_max_items"],
                   side_slots=l["side_slots"], detector_slot=l["detector_slot"],
                   ejector_slots=tuple(l["ejector_slots"]))


@dataclass
class Item:
    id: int
    pos: float = 0.0


@dataclass
class Forces:
    """UI fault injection / overrides (names as in the library)."""
    fsl_motor: list = field(default_factory=lambda: [False] * 5)   # FSL11..15 force stop
    fsh_ej: list = field(default_factory=lambda: [False] * 4)      # FSH21..24 force extend
    fsl_ej: list = field(default_factory=lambda: [False] * 4)      # FSL21..24 force retract
    fh_ej: list = field(default_factory=lambda: [False] * 4)       # FH21..24 feedbacks true
    fl_ej: list = field(default_factory=lambda: [False] * 4)       # FL21..24 feedbacks false
    fh_det: list = field(default_factory=lambda: [False] * 4)      # FH11..14 item in front true
    fl_det: list = field(default_factory=lambda: [False] * 4)      # FL11..14 item in front false
    xe: bool = True                                                # XE enable item feed
    xe01: bool = False                                             # XE01 untreated items allowed


class Transport:
    def __init__(self, pef: Pef | None = None, seed: int = 0):
        self.pef = pef or Pef()
        self.rng = random.Random(seed or None)
        self.f = Forces()
        self.t1: list[Item] = []
        self.side: list[list[Item]] = [[] for _ in range(4)]       # side[k] = conveyor T(k+2), newest first
        self.store: list[int] = [0] * 5                             # item id in store n (0 = empty)
        self.motors = [Valve() for _ in range(5)]
        self.ejectors = [Valve() for _ in range(4)]
        self.cmd = dict(yh=[False] * 5, yr=[0] * 5, ej_yh=[False] * 4, ej_yl=[False] * 4)
        self.xes = False                                            # feed single item
        self.vel = [0.0] * 5                                        # ticks / s of each conveyor
        # outputs
        self.det = [False] * 4
        self.xcq = 0
        self.front = [False] * 4
        self.apply_params()

    # ------------------------------------------------------------------ geometry
    @property
    def r(self) -> float:
        return self.pef.tpi / 2

    def det_pos(self) -> float:
        return (self.pef.detector_slot - 1) * self.pef.tpi + self.r

    def ej_pos(self, k: int) -> float:
        return (self.pef.ejector_slots[k] - 1) * self.pef.tpi + self.r

    def t1_max(self) -> float:
        return self.pef.tpi * self.pef.t1_offload_slot

    def side_max(self) -> float:
        return self.pef.tpi * self.pef.side_slots

    # ------------------------------------------------------------------ parameters
    def apply_params(self):
        p = self.pef
        for m in self.motors:
            m.p.t_to_high, m.p.t_to_low = p.motor_start, p.motor_stop
            m.p.ad_from_low = m.p.ad_from_high = 0.0
            m.p.travel_to_end = True
        for e in self.ejectors:
            e.p.t_to_high, e.p.t_to_low = p.ej_extend, p.ej_retract
            e.p.ad_from_low, e.p.ad_from_high = p.ej_delay_low, p.ej_delay_high
            e.p.travel_to_end = p.ej_travel_to_end

    def period(self, n: int, ref: int) -> float:
        ref = max(0, min(RAW_MAX, int(ref)))
        inv = RAW_MAX - ref
        p = self.pef
        return (p.period_max[n] - p.period_min[n]) / RAW_MAX * inv + p.period_min[n]

    # ------------------------------------------------------------------ one-shot UI actions
    def feed_single(self):
        self.xes = True

    def empty_transport(self):
        self.t1.clear()
        for s in self.side:
            s.clear()

    def empty_store(self, n: int):
        self.store[n - 1] = 0

    def enter_store1_item5(self):
        self.store[0] = 5

    # ------------------------------------------------------------------ step
    def step(self, dt: float):
        c, f, p = self.cmd, self.f, self.pef
        # equipment emulators (motors: start delay, stop delay; ejectors: stroke + action delays)
        for i, m in enumerate(self.motors):
            m.p.safeguard_low = f.fsl_motor[i]
            m.step(dt, c["yh"][i] and not f.fsl_motor[i], not (c["yh"][i] and not f.fsl_motor[i]))
        for k, e in enumerate(self.ejectors):
            e.p.safeguard_high, e.p.safeguard_low = f.fsh_ej[k], f.fsl_ej[k]
            e.step(dt, c["ej_yh"][k] and not f.fsh_ej[k], c["ej_yl"][k] and not f.fsl_ej[k])
        run = [m.at_high for m in self.motors]
        ticks = max(1, min(MAX_ANIM_TICKS, p.anim_ticks))
        self.vel = [ticks / self.period(n, c["yr"][n]) if run[n] else 0.0 for n in range(5)]

        self._move_t1(dt, run[0], ticks)
        for k in range(4):
            self._move_side(k, dt, run[k + 1])
        self._detectors()
        # sorting: an ejector that just started to move takes the item in front of it
        for k, e in enumerate(self.ejectors):
            if e.left_low:
                self._eject(k)

    # ---- T1
    def _move_t1(self, dt, running, ticks):
        p = self.pef
        if not running:
            return
        step = self.vel[0] * dt                        # ticks travelled this step
        # An extended (or moving) ejector is a barrier: items queue up in front of it one item length
        # apart; an item already past the barrier is not affected.  (The PLC library holds items with a
        # per-item flag; that lets stacked items overlap after the front one has been ejected.)
        last: dict[int, float] = {}                    # ejector index -> position of the last held item
        keep = []
        for it in sorted(self.t1, key=lambda i: -i.pos):
            target = it.pos + step
            limited = []
            for k, e in enumerate(self.ejectors):
                ej = self.ej_pos(k)
                if e.at_low or it.pos > ej:
                    continue
                lim = max((last[k] if k in last else ej) - p.tpi, it.pos)
                if target >= lim:
                    target = lim
                    limited.append(k)
            it.pos = target
            for k in limited:
                last[k] = it.pos
            if it.pos > self.t1_max():                 # rolled off the end -> store 1
                self.store[0] = it.id
                continue
            keep.append(it)
        self.t1 = sorted(keep, key=lambda i: i.pos)    # oldest (front) item last
        # item entry: needs a free slot and enough room behind the last item
        min_sp = self.t1_max()
        for it in self.t1:
            if it.pos <= min_sp:
                min_sp = it.pos
        enable = len(self.t1) < p.t1_max_items and min_sp > p.tpi + p.spacing
        if enable and (self.f.xe or self.xes):
            self.xes = False
            x = self.rng.random()
            if x > 0.80:
                iid = 4
            elif x > 0.60:
                iid = 3
            elif x > 0.40:
                iid = 2
            elif x > 0.20:
                iid = 1
            elif self.f.xe01:
                iid = 5                                # 'untreated item' can be blocked
            else:
                iid = 1
            self.t1.insert(0, Item(iid, 0.0))

    # ---- T2..T5
    def _move_side(self, k, dt, running):
        if not running:
            return
        keep = []
        for it in self.side[k]:
            it.pos += self.vel[k + 1] * dt
            if it.pos > self.side_max():
                self.store[k + 1] = it.id
                continue
            keep.append(it)
        self.side[k] = keep

    # ---- detectors
    def _detectors(self):
        r = self.r
        self.det = [False] * 4
        self.xcq = 0
        for it in self.t1:
            d = abs(it.pos - self.det_pos())
            if d <= r:
                for j in range(4):
                    self.det[j] = it.id == j + 1
                if d <= r / 2:
                    self.xcq = COLOURS[it.id]
        self.front = [False] * 4
        for it in self.t1:
            c = it.pos + r
            for k in range(4):
                if abs(c - self.ej_pos(k)) <= r:
                    self.front[k] = True

    # ---- sorting onto T(k+2)
    def _eject(self, k):
        p, r = self.pef, self.r
        ejpos = self.ej_pos(k)
        side = self.side[k]
        cand = [it for it in self.t1 if abs(it.pos + r - ejpos) <= r]
        if cand:                                                # one item per stroke: the one best centred
            it = min(cand, key=lambda i: abs(i.pos + r - ejpos))
            if len(side) >= p.side_slots:
                self.store[k + 1] = side.pop().id               # oldest item pushed out of the end
            self.t1.remove(it)
            side.insert(0, Item(it.id, 0.0))
            for j in range(1, len(side)):                       # prevent overlapping items
                if abs(side[j].pos - side[j - 1].pos) < p.tpi:
                    side[j].pos = side[j - 1].pos + p.tpi

    # ------------------------------------------------------------------ outputs (names as the PLC IO)
    def outputs(self) -> dict:
        f = self.f
        o: dict = {}
        for i in range(5):
            o["XGH1%d" % (i + 1)] = self.motors[i].at_high
        for k in range(4):
            e = self.ejectors[k]                                 # ejector k+1 -> names 21..24
            nm = "2%d" % (k + 1)
            o["XGH" + nm] = (e.at_high and not f.fl_ej[k]) or f.fh_ej[k]
            o["XGL" + nm] = (e.at_low and not f.fl_ej[k]) or f.fh_ej[k]
        for j in range(4):
            o["XCQ0%d" % (j + 1)] = self.det[j]
            o["XCQ1%d" % (j + 1)] = (self.front[j] and not f.fl_det[j]) or f.fh_det[j]
        o["XCQ"] = self.xcq
        for n in range(5):
            o["BQ0%d" % (n + 1)] = self.store[n] > 0
        o["X11"] = self._t1_velocity_raw()
        return o

    def _t1_velocity_raw(self) -> int:
        """Port of 'Calculate transport 01 velocity' (full scale = 10 ticks per minimum period)."""
        if not self.motors[0].at_high:
            return 0
        p = self.pef
        vmax = MAX_ANIM_TICKS / p.period_min[0]
        vmin = 1.0 / p.period_max[0]
        ticks = max(1, min(MAX_ANIM_TICKS, p.anim_ticks))
        v = ticks / self.period(0, self.cmd["yr"][0])
        min_a = vmin / vmax * RAW_MAX
        raw = (RAW_MAX - min_a) / (vmax - vmin) * (v - vmin) + min_a
        return max(0, min(RAW_MAX, int(raw)))
