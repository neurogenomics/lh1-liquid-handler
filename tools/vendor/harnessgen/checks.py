"""Electrical paper checks on a wiring table.  Each function returns a list of findings {code, cable_id, msg}.

Rules (all were in the originating project's check script; nothing new is invented here):
* CABLE_ID_DUP             cable ids are unique
* UNKNOWN_CABLE_TYPE       every cable_type is in the cable-types file
* CABLE_OVER_AMPACITY      max_current_a <= the cable type's ampacity_a
* UNKNOWN_FUSE / FUSE_DOES_NOT_PROTECT_CABLE
                           a cable whose protected_by names a fuse (fuse_pattern) must have ampacity >= that fuse
* VOLTAGE_DROP             power cables (signal prefix in voltage_classes): drop < max_drop_pct of the nominal voltage;
                           2 x length for a multi-conductor cable (out and back), 1 x for a single conductor
* CONNECTOR_FORMAT         connector_part is 'end A | end B'
* UNKNOWN_CONNECTOR        each end's family is in the connectors table
* CONNECTOR_OVER_RATING    max_current_a <= the family's a_per_contact
* CRIMP_GAUGE              gauge_mm2 <= the family's max_gauge_mm2 (when given)
* CONNECTOR_PINS           '<n>-pin' <= the family's max_pins, and conductors <= n
"""
from __future__ import annotations

import re

from .table import REQUIRED, connector_ends, missing_columns, split_family

DEFAULT_RULES = {
    "fuses": {},                         # fuse name -> rating (A)
    "fuse_pattern": r"^(F\d+)",          # how protected_by names a fuse (group 1 = fuse name)
    "fuse_exempt_types": [],             # cable types not held to their fuse (e.g. board-to-board plug-in)
    "voltage_classes": [],               # [{prefix: "24V", volts: 24}, ...]: first prefix that starts the signal
    "max_drop_pct": 3.0,
    "resistivity_ohm_mm2_per_m": 0.0172,  # copper at 20 C
    "pin_count_exempt_families": [],     # families whose '<n>-pin' does not bound the conductor count
    "pin_count_exempt_ends": [],         # end prefixes exempt from the conductor-count check
}


def rules_with_defaults(rules: dict | None) -> dict:
    r = dict(DEFAULT_RULES)
    r.update(rules or {})
    return r


def _f(code, cid, msg):
    return {"code": code, "cable_id": cid, "msg": msg}


def check_columns(rows) -> list[dict]:
    miss = missing_columns(rows, REQUIRED)
    return [_f("MISSING_COLUMN", None, f"wiring table lacks column(s) {miss}")] if miss else []


def check_ids(rows) -> list[dict]:
    ids = [r["cable_id"] for r in rows]
    if len(ids) != len(set(ids)):
        return [_f("CABLE_ID_DUP", None, str(sorted({i for i in ids if ids.count(i) > 1})))]
    return []


def check_types(rows, types) -> list[dict]:
    return [_f("UNKNOWN_CABLE_TYPE", r["cable_id"], f"{r['cable_id']}: {r['cable_type']}") for r in rows if r["cable_type"] not in types]


def cable_resistance(row, types, rules=None, legs: int | None = None) -> float:
    """Conductor resistance of one cable (ohm): legs x length / gauge x resistivity; legs default 2 for a
    multi-conductor cable (out and back), 1 for a single conductor."""
    rules = rules_with_defaults(rules)
    t = types[row["cable_type"]]
    n = legs or (1 if int(row["conductors"]) == 1 else 2)
    return n * float(row["length_mm_estimate"]) / 1000 * rules["resistivity_ohm_mm2_per_m"] / t["gauge_mm2"]


def check_rows(rows, types, connectors, rules=None) -> tuple[list[dict], dict]:
    """Ampacity, fuse, voltage drop and connector checks, row by row (findings in row order).
    Returns (findings, {cable_id: drop % of nominal} for every power cable checked)."""
    rules = rules_with_defaults(rules)
    fuses = rules["fuses"]
    out, drops = [], {}
    for r in rows:
        cid = r["cable_id"]
        if r["cable_type"] not in types:
            continue                      # reported by check_types
        t = types[r["cable_type"]]
        i = float(r["max_current_a"])
        sig = r["signal"]
        if i > t["ampacity_a"] + 1e-9:
            out.append(_f("CABLE_OVER_AMPACITY", cid, f"{cid}: {i} A on {r['cable_type']} ({t['ampacity_a']} A)"))
        m = re.match(rules["fuse_pattern"], r["protected_by"])
        if m:
            if m.group(1) not in fuses:
                out.append(_f("UNKNOWN_FUSE", cid, f"{cid}: {r['protected_by']}"))
            elif t["ampacity_a"] + 1e-9 < fuses[m.group(1)] and r["cable_type"] not in rules["fuse_exempt_types"]:
                out.append(_f("FUSE_DOES_NOT_PROTECT_CABLE", cid,
                              f"{cid}: {r['cable_type']} {t['ampacity_a']} A < {m.group(1)} {fuses[m.group(1)]} A"))
        vc = next((c for c in rules["voltage_classes"] if sig.startswith(c["prefix"])), None)
        if vc and t["gauge_mm2"] > 0 and i > 0:
            v_nom = float(vc["volts"])
            legs = 1 if int(r["conductors"]) == 1 else 2
            vd = legs * float(r["length_mm_estimate"]) / 1000 * rules["resistivity_ohm_mm2_per_m"] / t["gauge_mm2"] * i
            pct = 100 * vd / v_nom
            drops[cid] = round(pct, 2)
            if pct >= rules["max_drop_pct"]:
                out.append(_f("VOLTAGE_DROP", cid, f"{cid}: {vd:.2f} V = {pct:.1f} % at {i} A over {r['length_mm_estimate']} mm of {t['gauge_mm2']} mm2"))
        ends = connector_ends(r)
        if len(ends) != 2:
            out.append(_f("CONNECTOR_FORMAT", cid, f"{cid}: '{r['connector_part']}' (want 'end A | end B')"))
        for e in ends:
            fam, npins = split_family(e)
            if fam not in connectors:
                out.append(_f("UNKNOWN_CONNECTOR", cid, f"{cid}: {fam}"))
                continue
            cn = connectors[fam]
            if i > cn["a_per_contact"] + 1e-9:
                out.append(_f("CONNECTOR_OVER_RATING", cid, f"{cid}: {i} A > {fam} {cn['a_per_contact']} A per contact"))
            if cn.get("max_gauge_mm2") and float(r["gauge_mm2"]) > cn["max_gauge_mm2"] + 1e-9:
                out.append(_f("CRIMP_GAUGE", cid, f"{cid}: {r['gauge_mm2']} mm2 does not fit a {fam} contact (max {cn['max_gauge_mm2']} mm2)"))
            if npins and npins > cn["max_pins"]:
                out.append(_f("CONNECTOR_PINS", cid, f"{cid}: {fam} has at most {cn['max_pins']} positions"))
            if (npins and fam not in rules["pin_count_exempt_families"] and int(r["conductors"]) > npins
                    and not e.startswith(tuple(rules["pin_count_exempt_ends"]))):
                out.append(_f("CONNECTOR_PINS", cid, f"{cid}: {r['conductors']} conductors into a {npins}-pin {fam}"))
    return out, drops


def check_all(rows, types, connectors, rules=None) -> tuple[list[dict], dict]:
    cols = check_columns(rows)
    if cols:
        return cols, {}
    f = check_ids(rows) + check_types(rows, types)
    fr, drops = check_rows(rows, types, connectors, rules)
    return f + fr, drops
