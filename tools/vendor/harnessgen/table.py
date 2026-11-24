"""The wiring table (one row per cable / conductor group) and the cable-types file."""
from __future__ import annotations

import csv
from pathlib import Path

import yaml

COLUMNS = ["cable_id", "from_device", "from_connector", "from_pins", "to_device", "to_connector", "to_pins", "signal",
           "conductors", "gauge_mm2", "shielded", "max_current_a", "length_mm_estimate", "route", "connector_part", "notes",
           "cable_type", "protected_by", "fw_pins", "wire_no", "colour"]
# columns the checks, counts and drawing read; the rest are carried through
REQUIRED = ["cable_id", "from_device", "to_device", "signal", "conductors", "gauge_mm2", "max_current_a",
            "length_mm_estimate", "connector_part", "notes", "cable_type", "protected_by"]


def read_rows(path) -> list[dict]:
    with open(path, newline="") as f:
        return list(csv.DictReader(f))


def missing_columns(rows: list[dict], required=REQUIRED) -> list[str]:
    have = set(rows[0]) if rows else set()
    return [c for c in required if c not in have]


def write_rows(rows: list[dict], path, columns=COLUMNS) -> None:
    with open(path, "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=columns)
        w.writeheader()
        w.writerows(rows)


def load_types(path) -> dict:
    """Cable-types file: {cable_types: {TYPE: {...}}, connectors: {FAMILY: {...}}, joint_types: [...] (optional)}."""
    d = yaml.safe_load(Path(path).read_text()) or {}
    d.setdefault("cable_types", {})
    d.setdefault("connectors", {})
    return d


def connector_ends(row: dict) -> list[str]:
    """'end A | end B' -> ['end A', 'end B'] (stripped)."""
    return [e.strip() for e in row["connector_part"].split("|")]


def split_family(end: str) -> tuple[str, int | None]:
    """'JST-XH 3-pin' -> ('JST-XH', 3); anything without '<n>-pin' at the end -> (end, None)."""
    import re
    m = re.match(r"(.+?)\s+(\d+)-pin$", end)
    return (m.group(1), int(m.group(2))) if m else (end, None)
