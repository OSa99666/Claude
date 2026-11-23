"""Bistable (double-solenoid) valve model, after the library block fbValveSimulator.

Position ``pos`` runs 0.0 (Low = passive/closed) .. 1.0 (High = active/open).

    V_YH  command High      -> valve travels to High
    V_YL  command Low       -> valve travels to Low
    V_XGH feedback: valve is at High (pos == 1)
    V_XGL feedback: valve is at Low  (pos == 0)    both false while travelling

Behaviour
* Bistable: the last command is remembered; with neither command set the valve stays where it is.
  With both commands set (conflict) the previous command stays in force.
* Action delay: the valve leaves an end position only after the command has been present for
  ``ad_from_low`` / ``ad_from_high`` seconds.  Reversing in mid-travel has no delay.
* Transit time: ``t_to_high`` / ``t_to_low`` seconds for the full stroke, linear.
* travel_to_end: a started travel is completed even if the command goes away (here the memory
  means the same; with travel_to_end=False the valve stops when the command is removed).
* Safeguards (interpretation): ``safeguard_high`` blocks the valve from leaving High,
  ``safeguard_low`` blocks it from leaving Low.
* Force feedbacks: ``force_true`` makes both feedbacks true, ``force_false`` both false (wins).
"""
from __future__ import annotations

from dataclasses import dataclass


@dataclass
class ValveParams:
    t_to_high: float = 3.0
    t_to_low: float = 3.0
    ad_from_low: float = 0.5
    ad_from_high: float = 0.5
    travel_to_end: bool = True
    safeguard_high: bool = False
    safeguard_low: bool = False
    force_true: bool = False
    force_false: bool = False


class Valve:
    def __init__(self, p: ValveParams | None = None, high: bool = False):
        self.p = p or ValveParams()
        self.pos = 1.0 if high else 0.0
        self.want: int | None = None      # remembered command: 1 = High, 0 = Low
        self.delay = 0.0                  # time the current command has been waiting

    # -- direct control from the UI ------------------------------------------------
    def set_position(self, high: bool):
        self.pos = 1.0 if high else 0.0
        self.want = None
        self.delay = 0.0

    # -- feedbacks -------------------------------------------------------------------
    @property
    def at_high(self) -> bool:
        return self.pos >= 1.0

    @property
    def at_low(self) -> bool:
        return self.pos <= 0.0

    def feedbacks(self) -> tuple[bool, bool]:
        """(V_XGH, V_XGL)"""
        if self.p.force_false:
            return False, False
        if self.p.force_true:
            return True, True
        return self.at_high, self.at_low

    # -- time step -------------------------------------------------------------------
    def step(self, dt: float, cmd_high: bool, cmd_low: bool):
        p = self.p
        if cmd_high and not cmd_low:
            if self.want != 1:
                self.delay = 0.0
            self.want = 1
        elif cmd_low and not cmd_high:
            if self.want != 0:
                self.delay = 0.0
            self.want = 0
        elif not (cmd_high or cmd_low) and not p.travel_to_end and 0.0 < self.pos < 1.0:
            self.want = None                          # command removed: stop where we are
        if self.want is None:
            return
        if (self.want == 1 and self.at_high) or (self.want == 0 and self.at_low):
            self.delay = 0.0
            return
        # leaving an end position
        if self.at_low or self.at_high:
            if (self.at_low and p.safeguard_low) or (self.at_high and p.safeguard_high):
                self.delay = 0.0
                return
            wait = p.ad_from_low if self.at_low else p.ad_from_high
            self.delay += dt
            if self.delay < wait:
                return
        if self.want == 1:
            self.pos = min(1.0, self.pos + dt / max(p.t_to_high, 1e-3))
        else:
            self.pos = max(0.0, self.pos - dt / max(p.t_to_low, 1e-3))
