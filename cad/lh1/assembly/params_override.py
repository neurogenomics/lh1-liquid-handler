"""LH-1 integration: params override mechanism, RETIRED in revision E.

Revision D applied cad/lh1/assembly/out/params_requests.yaml in memory.  Those values are now in cad/lh1/params.py, so
apply() is a no-op unless LH1_PARAMS_OVERRIDE=1 is set (kept only so old command lines still work).
"""
from __future__ import annotations

import os
from pathlib import Path

import yaml

REQ = Path(__file__).resolve().parent / "out" / "params_requests.yaml"


def apply(P):
    if os.environ.get("LH1_PARAMS_OVERRIDE", "0") != "1":
        P.REV_D_OVERRIDES = {}
        return P
    done = {}
    for r in yaml.safe_load(REQ.read_text())["requests"]:
        if r.get("override"):
            name = r["param"].split()[0]
            done[name] = {"old": getattr(P, name, None), "new": r["new"]}
            setattr(P, name, r["new"])
    P.REV_D_OVERRIDES = done
    return P
