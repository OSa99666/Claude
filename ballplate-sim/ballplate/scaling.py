"""Raw INT <-> engineering units, same convention as fbLinearScaling in the PLC.

``lo`` maps to -raw_max and ``hi`` maps to +raw_max (default +-32000); values
outside [lo, hi] saturate.
"""
from __future__ import annotations

RAW_MAX = 32000


def to_raw(value: float, lo: float, hi: float, raw_max: int = RAW_MAX) -> int:
    if hi == lo:
        return 0
    f = (value - lo) / (hi - lo)
    raw = round(-raw_max + f * 2 * raw_max)
    return max(-raw_max, min(raw_max, raw))


def from_raw(raw: float, lo: float, hi: float, raw_max: int = RAW_MAX) -> float:
    raw = max(-raw_max, min(raw_max, raw))
    return lo + (raw + raw_max) / (2 * raw_max) * (hi - lo)
