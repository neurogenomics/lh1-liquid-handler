"""Cut list, cable totals, joint counts and connector/crimp/ferrule counts from a wiring table."""
from __future__ import annotations

import math
from collections import defaultdict

from .table import connector_ends, split_family

# Joint classes per conductor end: first rule whose prefix starts the connector end wins.  "tool:" classes are joints
# made with a tool on the bench; their sum is reported as "tool-made joints, total".  Override with `joint_types` in
# the cable-types file (same shape).
DEFAULT_JOINT_TYPES = [
    {"label": "tool-free: pre-made / moulded / plug-in", "prefixes": ["pre-made", "moulded", "maker's", "board plug-in", "USB"]},
    {"label": "tool-free: lever splice", "prefixes": ["Wago"]},
    {"label": "tool-free: ZIF / flat-flex", "prefixes": ["ZIF"]},
    {"label": "tool: bootlace ferrule", "prefixes": ["ferrule"]},
    {"label": "tool: small-contact crimp", "prefixes": ["JST", "Molex"]},
    {"label": "tool: solder + heat-shrink", "prefixes": ["solder"]},
    {"label": "tool: ring lug / receptacle crimp", "prefixes": ["ring lug", "insulated receptacle"]},
    {"label": "tool: soldered XT30/XT60", "prefixes": ["Amass"]},
]
TOTAL_LABEL = "tool-made joints, total"
DEFAULT_HAND_CRIMP = ("JST-XH", "JST-PH", "Molex Mini-Fit Jr", "Molex Micro-Fit 3.0")
DEFAULT_NO_CABLE_TYPES = ("PLUG_IN", "DEVICE_LEAD", "MOTOR_LEAD", "XT30_LEAD")


def joint_counts(rows, joint_types=None) -> dict:
    """Conductor ends per joint class (each end of each conductor counts once); unmatched ends -> 'other: <end>'."""
    rules = joint_types or DEFAULT_JOINT_TYPES
    c = defaultdict(int)
    for r in rows:
        n = int(r["conductors"])
        for e in connector_ends(r):
            for rule in rules:
                if e.startswith(tuple(rule["prefixes"])):
                    c[rule["label"]] += n
                    break
            else:
                c["other: " + e] += n
    c[TOTAL_LABEL] = sum(v for k, v in c.items() if k.startswith("tool:"))
    return dict(c)


def connector_counts(rows, hand_crimp=DEFAULT_HAND_CRIMP, skip_note="RESERVED"):
    """Housings per (family, pins), crimp contacts per (family, gauge) and ferrules per gauge.  Rows whose notes
    contain `skip_note` (not fitted) are left out."""
    housings, crimps, ferrules = defaultdict(int), defaultdict(int), defaultdict(int)
    for r in rows:
        if skip_note and skip_note in r["notes"]:
            continue
        for end in connector_ends(r):
            fam, pins = split_family(end)
            n = int(r["conductors"])
            if fam.startswith("ferrule"):
                ferrules[float(r["gauge_mm2"])] += n
            elif fam in hand_crimp:
                housings[(fam, pins)] += 1
                crimps[(fam, float(r["gauge_mm2"]))] += n
            elif fam.startswith("insulated receptacle") or fam.startswith("ring lug"):
                housings[(fam, None)] += n
            elif fam == "solder + heat-shrink":
                housings[("heat-shrink sleeve 3:1 adhesive-lined, per joint", None)] += n
    return housings, crimps, ferrules


def cut_list(rows, types, no_cable_types=DEFAULT_NO_CABLE_TYPES, skip_note="RESERVED") -> list[dict]:
    """One line per cable to cut: pieces = conductors for single-core types (each core cut separately), else 1."""
    out = []
    for r in rows:
        if r["cable_type"] in no_cable_types or (skip_note and skip_note in r["notes"]):
            continue
        t = types[r["cable_type"]]
        pieces = int(r["conductors"]) if t.get("single_core") else 1
        length = float(r["length_mm_estimate"])
        out.append({"cable_id": r["cable_id"], "cable_type": r["cable_type"], "pieces": pieces, "length_mm": r["length_mm_estimate"],
                    "total_mm": length * pieces, "from": f"{r['from_device']} {r.get('from_connector', '')}".strip(),
                    "to": f"{r['to_device']} {r.get('to_connector', '')}".strip()})
    return out


def cable_totals(rows, types, no_cable_types=DEFAULT_NO_CABLE_TYPES, skip_note="RESERVED") -> dict:
    """mm of each cable type to buy (before spare), in first-use order."""
    by_type = defaultdict(float)
    for r in rows:
        if r["cable_type"] in no_cable_types or (skip_note and skip_note in r["notes"]):
            continue
        t = types[r["cable_type"]]
        n = int(r["conductors"]) if t.get("single_core") else 1
        by_type[r["cable_type"]] += float(r["length_mm_estimate"]) * n
    return dict(by_type)


def metres_to_buy(mm: float, spare: float = 1.2) -> int:
    return math.ceil(mm * spare / 1000.0)
