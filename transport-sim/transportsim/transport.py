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


@dataclass
class FeedCfg:
    """How items are fed onto T1 (UI)."""
    seq_on: bool = False                                           # fixed repeating item sequence instead of random
    seq: list = field(default_factory=lambda: [1, 2, 3, 4, 5])
    rate_per_min: float = 0.0                                      # 0 = as fast as there is room


@dataclass
class FaultCfg:
    """Random faults (all 0 = off) for testing the PLC's error handling (UI)."""
    ej_jam_prob: float = 0.0          # probability per ejector stroke that the ejector jams
    ej_jam_s: float = 3.0             # jam duration
    dropout_per_min: float = 0.0      # sensor dropouts per minute while a sensor is active
    dropout_s: float = 0.4            # dropout duration
    trip_per_min: float = 0.0         # conveyor motor trips per minute (each conveyor)


class Transport:
    def __init__(self, pef: Pef | None = None, seed: int = 0):
        self.pef = pef or Pef()
        self.f = Forces()
        self.feed = FeedCfg()
        self.fault = FaultCfg()
        self.time = 0.0
        self.seq_idx = 0
        self.last_entry = -1e9
        self.tripped = [False] * 5                                  # motor trip, latched until the PLC drops T_YH1n
        self.jam_until = [0.0] * 4                                  # ejector jammed (frozen) until this time
        self.jam_next = [False] * 4                                 # jam the next stroke of this ejector
        self.drop_until = [0.0] * 5                                 # sensor dropout: detector, front sensors 1..4
        self.reseed(seed)
        self.reset_stats()
        self.t1: list[Item] = []
        self.side: list[list[Item]] = [[] for _ in range(4)]       # side[k] = conveyor T(k+2), newest first
        self.store: list[int] = [0] * 5                             # item id in store n (0 = empty)
        self.store_in: list[int] = [0] * 5                          # items that have arrived in store n (for the view)
        self.motors = [Valve() for _ in range(5)]
        self.ejectors = [Valve() for _ in range(4)]
        self.cmd = dict(yh=[False] * 5, yr=[0] * 5, ej_yh=[False] * 4, ej_yl=[False] * 4)
        self.xes = False                                            # feed single item
        self.vel = [0.0] * 5                                        # ticks / s of each conveyor
        # outputs
        self.det = [False] * 4
        self.xcq = 0
        self.front = [False] * 4
        self.dt_last = 0.01
        self.apply_params()

    # ------------------------------------------------------------------ seed / statistics / fault triggers
    def reseed(self, seed: int = 0):
        """(Re)start the item sequence: fixed seed (0 = random) and the fixed pattern from its beginning.
        Faults use their own generator, so they never change the item sequence."""
        self.seed = int(seed)
        self.rng = random.Random(self.seed or None)
        self.frng = random.Random((self.seed + 7919) if self.seed else None)
        self.seq_idx = 0

    def reset_stats(self):
        self.stats = {"fed": 0, "correct": 0, "wrong": 0, "arrived": [0] * 5, "last_wrong": ""}

    def trip(self, n: int):
        """Trip conveyor motor n (1..5): it stops until the PLC drops and sets its start command again."""
        self.tripped[n - 1] = True

    def jam_next_stroke(self, k: int):
        self.jam_next[k - 1] = True

    def drop_out_all_sensors(self):
        self.drop_until = [self.time + self.fault.dropout_s] * 5

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
        c, f, p, fa = self.cmd, self.f, self.pef, self.fault
        self.time += dt
        self.dt_last = dt
        # equipment emulators (motors: start delay, stop delay; ejectors: stroke + action delays)
        for i, m in enumerate(self.motors):
            if self.tripped[i] and not c["yh"][i]:
                self.tripped[i] = False                              # the PLC dropped the start command: trip reset
            elif (not self.tripped[i] and m.at_high and fa.trip_per_min > 0
                    and self.frng.random() < fa.trip_per_min / 60.0 * dt):
                self.tripped[i] = True                               # random motor trip while running
            start = c["yh"][i] and not f.fsl_motor[i] and not self.tripped[i]
            m.p.safeguard_low = f.fsl_motor[i]
            m.step(dt, start, not start)
        for k, e in enumerate(self.ejectors):
            e.p.safeguard_high, e.p.safeguard_low = f.fsh_ej[k], f.fsl_ej[k]
            if self.jam_until[k] > self.time:                        # jammed: frozen where it is
                e.left_low = False
                continue
            before = e.pos
            e.step(dt, c["ej_yh"][k] and not f.fsh_ej[k], c["ej_yl"][k] and not f.fsl_ej[k])
            started = (before <= 0.0 < e.pos) or (before >= 1.0 > e.pos)       # a stroke has just begun
            if started and (self.jam_next[k] or self.frng.random() < fa.ej_jam_prob):
                self.jam_next[k] = False
                self.jam_until[k] = self.time + fa.ej_jam_s
                e.left_low = False                                   # jammed before it could push the item
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

    def _to_store(self, n: int, item_id: int):
        self.store[n] = item_id
        self.store_in[n] += 1
        st = self.stats
        st["arrived"][n] += 1
        if n == item_id % 5:                                         # type k belongs in store k+1 (type 5 -> store 1)
            st["correct"] += 1
        else:
            st["wrong"] += 1
            st["last_wrong"] = "type %d in store %d" % (item_id, n + 1)

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
                self._to_store(0, it.id)
                continue
            keep.append(it)
        self.t1 = sorted(keep, key=lambda i: i.pos)    # oldest (front) item last
        # item entry: needs a free slot and enough room behind the last item
        min_sp = self.t1_max()
        for it in self.t1:
            if it.pos <= min_sp:
                min_sp = it.pos
        enable = len(self.t1) < p.t1_max_items and min_sp > p.tpi + p.spacing
        rate = self.feed.rate_per_min
        rate_ok = rate <= 0 or self.time - self.last_entry >= 60.0 / rate
        if enable and ((self.f.xe and rate_ok) or self.xes):
            self.xes = False
            if self.feed.seq_on and self.feed.seq:
                iid = self.feed.seq[self.seq_idx % len(self.feed.seq)]
                self.seq_idx += 1
            else:
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
                    iid = 5                            # 'untreated item' can be blocked
                else:
                    iid = 1
            self.last_entry = self.time
            self.stats["fed"] += 1
            self.t1.insert(0, Item(iid, 0.0))

    # ---- T2..T5
    def _move_side(self, k, dt, running):
        if not running:
            return
        keep = []
        for it in self.side[k]:
            it.pos += self.vel[k + 1] * dt
            if it.pos > self.side_max():
                self._to_store(k + 1, it.id)
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
        # random / triggered sensor dropouts: an active sensor reads false for a short time
        fa = self.fault
        active = [any(self.det) or self.xcq != 0] + list(self.front)
        for i in range(5):
            if active[i] and fa.dropout_per_min > 0 and self.frng.random() < fa.dropout_per_min / 60.0 * self.dt_last:
                self.drop_until[i] = self.time + fa.dropout_s
        if self.drop_until[0] > self.time:
            self.det = [False] * 4
            self.xcq = 0
        for k in range(4):
            if self.drop_until[k + 1] > self.time:
                self.front[k] = False

    # ---- sorting onto T(k+2)
    def _eject(self, k):
        p, r = self.pef, self.r
        ejpos = self.ej_pos(k)
        side = self.side[k]
        cand = [it for it in self.t1 if abs(it.pos + r - ejpos) <= r]
        if cand:                                                # one item per stroke: the one best centred
            it = min(cand, key=lambda i: abs(i.pos + r - ejpos))
            if len(side) >= p.side_slots:
                self._to_store(k + 1, side.pop().id)            # oldest item pushed out of the end
            self.t1.remove(it)
            side.insert(0, Item(it.id, 0.0))
            for j in range(1, len(side)):                       # prevent overlapping items
                if abs(side[j].pos - side[j - 1].pos) < p.tpi:
                    side[j].pos = side[j - 1].pos + p.tpi

    def faults_state(self) -> dict:
        t = self.time
        return {"trip": list(self.tripped), "jam": [round(max(0.0, u - t), 1) for u in self.jam_until],
                "drop": [u > t for u in self.drop_until], "jam_next": list(self.jam_next)}

    def stats_state(self) -> dict:
        st = self.stats
        return {"fed": st["fed"], "correct": st["correct"], "wrong": st["wrong"], "arrived": list(st["arrived"]),
                "last_wrong": st["last_wrong"], "in_transit": len(self.t1) + sum(len(x) for x in self.side)}

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
