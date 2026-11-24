"""klipper-guardcheck render CASES.yaml | audit TABLE.yaml   (exit 0 pass, 1 findings, 2 usage / input error)"""
from __future__ import annotations

import argparse
import json
import sys

from .audit import AuditTableError, audit_file
from .render import CaseFileError, run_case_file


def _overrides(items):
    out = {}
    for it in items or []:
        if "=" not in it:
            raise SystemExit(f"--config takes NAME=PATH, got {it!r}")
        k, v = it.split("=", 1)
        out[k] = v
    return out


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(prog="klipper-guardcheck", description=__doc__)
    sub = ap.add_subparsers(dest="cmd", required=True)
    r = sub.add_parser("render", help="run behavioural cases against guard macros with a mocked printer")
    r.add_argument("cases", help="case file (YAML)")
    a = sub.add_parser("audit", help="check every reachable built-in command is refused, wrapped or classified")
    a.add_argument("table", help="audit table (YAML)")
    for p in (r, a):
        p.add_argument("--config", action="append", metavar="NAME=PATH", help="override a config path from the file")
        p.add_argument("--json", action="store_true", help="print the full result as JSON")
    args = ap.parse_args(argv)
    try:
        if args.cmd == "render":
            res = run_case_file(args.cases, _overrides(args.config))
            bad = res["failed"]
            if args.json:
                print(json.dumps(res, indent=1))
            else:
                for c in res["cases"]:
                    if not c["ok"]:
                        print(f"FAIL {c['where']} {c['macro']} {c['params']} [{c['state']}]: {'; '.join(c['problems'])}")
                print(f"{res['n']} cases, {res['failed']} failed (jinja2 {res['jinja2']})")
        else:
            res = audit_file(args.table, _overrides(args.config))
            bad = len(res["findings"])
            if args.json:
                print(json.dumps(res, indent=1))
            else:
                for f in res["findings"]:
                    print(f"{f['code']}: {f['msg']}" + (f" ({f['reason']})" if f.get("reason") else ""))
                n = sum(len(v) for v in res["report"].values())
                print(f"{n} refused/wrapped entries checked, {bad} findings")
    except (CaseFileError, AuditTableError, OSError, KeyError) as e:
        print(f"error: {e}", file=sys.stderr)
        return 2
    return 1 if bad else 0


if __name__ == "__main__":
    sys.exit(main())
