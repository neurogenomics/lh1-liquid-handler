"""Helpers for authoring a wiring table: route lengths, cut lengths, colours, pre-made connector naming."""
from __future__ import annotations

import math

DEFAULT_COLOUR_FALLBACK = ("SAFETY", "SIGNAL", "ENDSTOP", "SENSOR", "MOTOR", "TEC", "HEATER", "CAN", "USB", "STEP", "PE",
                           "MAINS", "0V")


def manhattan(a, b) -> float:
    return sum(abs(x - y) for x, y in zip(a, b))


def route_length(points, chains=()) -> float:
    """Length of a cable through `points` (3-D, Manhattan legs: cables follow frame members).  Each entry of `chains`
    is (fixed_end, moving_end, chain_length): the straight fixed->moving distance, already counted among the legs when
    both ends are waypoints, is replaced by the chain's own length."""
    total = 0.0
    for i in range(len(points) - 1):
        total += manhattan(points[i], points[i + 1])
    for fx, mv, clen in chains:
        total += clen - manhattan(fx, mv)
    return total


def cut_length(raw_mm: float, slack: float = 1.10, terminations_mm: float = 200.0, round_to_mm: int = 50) -> int:
    """raw x slack + terminations, rounded UP to round_to_mm."""
    return int(math.ceil((raw_mm * slack + terminations_mm) / float(round_to_mm)) * round_to_mm)


def colour_of(signal: str, colours: dict, fallback=DEFAULT_COLOUR_FALLBACK, default: str = "grey") -> str:
    """One colour per rail: the longest key of `colours` that prefixes the signal (or its part before ' (') wins;
    otherwise the first `fallback` class that is one of the first two words or prefixes the signal."""
    best = max((k for k in colours if signal.startswith(k) or signal.split(" (")[0].startswith(k)), key=len, default=None)
    if best is None:
        for k in fallback:
            if k in signal.split()[0:2] or signal.startswith(k):
                best = k
                break
    return colours.get(best, default)


def premade(end: str, families, prefix: str = "pre-made ") -> str:
    """Mark a connector end as a bought pre-made lead when its family is in `families`."""
    e = end.strip()
    for fam in families:
        if e.startswith(fam) and not e.startswith(prefix.strip()):
            return prefix + e
    return e
