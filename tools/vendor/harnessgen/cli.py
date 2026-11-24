"""harnessgen check | build | draw   (exit 0 pass, 1 findings, 2 usage / input error)

  harnessgen check WIRING.csv --types cable_types.yaml [--rules rules.yaml] [--json]
  harnessgen build WIRING.csv --types cable_types.yaml [--rules rules.yaml] --out DIR [--json]
        writes DIR/cut_list.csv, DIR/cable_totals.csv, DIR/joint_counts.json, DIR/connector_counts.json, DIR/checks.json
  harnessgen draw  WIRING.csv --devices devices.yaml[:key/path] --style style.yaml -o harness.svg
"""
from __future__ import annotations

import argparse
import csv
import json
import sys
from pathlib import Path

import yaml

from .checks import check_all
from .counts import cable_totals, connector_counts, cut_list, joint_counts, metres_to_buy
from .draw import DrawError, draw_svg
from .table import load_types, read_rows


def _load_rules(path):
    return (yaml.safe_load(Path(path).read_text()) or {}) if path else {}


def _load_devices(spec):
    path, _, key = spec.rpartition(":") if ":" in Path(spec).name else (spec, "", "")
    d = yaml.safe_load(Path(path).read_text())
    for k in key.split("/") if key else []:
        d = d[k]
    return d


def _report(findings, as_json, extra=None):
    if as_json:
        print(json.dumps({"passed": not findings, "findings": findings, **(extra or {})}, indent=1))
    else:
        for f in findings:
            print(f"{f['code']}: {f['msg']}")
        print(f"{len(findings)} findings")


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(prog="harnessgen", description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = ap.add_subparsers(dest="cmd", required=True)
    for name in ("check", "build"):
        p = sub.add_parser(name)
        p.add_argument("wiring")
        p.add_argument("--types", required=True, help="cable types + connectors YAML")
        p.add_argument("--rules", help="check rules YAML (fuses, voltage_classes, ...)")
        p.add_argument("--json", action="store_true")
        if name == "build":
            p.add_argument("--out", required=True, help="output directory")
            p.add_argument("--spare", type=float, default=1.2, help="cable spare factor for cable_totals (default 1.2)")
    d = sub.add_parser("draw")
    d.add_argument("wiring")
    d.add_argument("--devices", required=True, help="YAML of {device: {group, kind}}; FILE:key/path selects a sub-mapping")
    d.add_argument("--style", required=True)
    d.add_argument("-o", "--output", required=True)
    args = ap.parse_args(argv)
    try:
        rows = read_rows(args.wiring)
        if args.cmd == "draw":
            svg, info = draw_svg(rows, _load_devices(args.devices), yaml.safe_load(Path(args.style).read_text()))
            Path(args.output).write_text(svg)
            print(f"{Path(args.output).name}: {info['devices']} devices, {info['cables']} cables, {info['width']} x {info['height']}")
            return 0
        t = load_types(args.types)
        findings, drops = check_all(rows, t["cable_types"], t["connectors"], _load_rules(args.rules))
        if args.cmd == "check":
            _report(findings, args.json, {"voltage_drop_pct": drops})
            return 1 if findings else 0
        out = Path(args.out)
        out.mkdir(parents=True, exist_ok=True)
        known = [r for r in rows if r["cable_type"] in t["cable_types"]]
        cl = cut_list(known, t["cable_types"])
        with open(out / "cut_list.csv", "w", newline="") as f:
            w = csv.DictWriter(f, fieldnames=["cable_id", "cable_type", "pieces", "length_mm", "total_mm", "from", "to"])
            w.writeheader()
            w.writerows(cl)
        tot = cable_totals(known, t["cable_types"])
        with open(out / "cable_totals.csv", "w", newline="") as f:
            w = csv.writer(f)
            w.writerow(["cable_type", "total_mm", f"metres_to_buy (x{args.spare})", "description"])
            for ct, mm in sorted(tot.items()):
                w.writerow([ct, round(mm, 1), metres_to_buy(mm, args.spare), t["cable_types"][ct].get("desc", "")])
        jc = joint_counts(rows, t.get("joint_types"))
        (out / "joint_counts.json").write_text(json.dumps(jc, indent=1))
        h, c, fe = connector_counts(rows)
        (out / "connector_counts.json").write_text(json.dumps(
            {"housings": [{"family": k[0], "pins": k[1], "count": v} for k, v in h.items()],
             "crimp_contacts": [{"family": k[0], "gauge_mm2": k[1], "count": v} for k, v in c.items()],
             "ferrules": [{"gauge_mm2": k, "count": v} for k, v in fe.items()]}, indent=1))
        (out / "checks.json").write_text(json.dumps({"passed": not findings, "findings": findings, "voltage_drop_pct": drops}, indent=1))
        _report(findings, args.json, {"cut_list_lines": len(cl), "joint_counts": jc})
        return 1 if findings else 0
    except (OSError, KeyError, DrawError, yaml.YAMLError) as e:
        print(f"error: {e!r}", file=sys.stderr)
        return 2


if __name__ == "__main__":
    sys.exit(main())
