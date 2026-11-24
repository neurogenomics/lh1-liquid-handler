"""Smoke tests: the repository is clean and the paper checks run.  These compare files; they do not test a machine."""
import importlib.util
import json
import subprocess
import sys
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[1]


def run(*args):
    return subprocess.run([sys.executable, *args], cwd=ROOT, capture_output=True, text=True, timeout=900)


def test_no_private_content():
    r = run("tools/check_no_private_content.py")
    assert r.returncode == 0, r.stdout[-3000:]


def test_electrical_check():
    r = run("electronics/lh1/check.py")
    rep = json.loads((ROOT / "electronics/lh1/out/check_report.json").read_text())
    errors = [f for f in rep["findings"] if f["severity"] == "error"]
    assert r.returncode == 0 and not errors, errors[:10]


def test_firmware_guard_cases():
    r = run("electronics/lh1/fw_render_check.py")
    out = json.loads(r.stdout[r.stdout.index("{"):])
    assert r.returncode == 0 and out["failed"] == 0 and out.get("total_failed_all_runs", 0) == 0


@pytest.mark.skipif(importlib.util.find_spec("cadquery") is None, reason="cadquery not installed (CAD regeneration is optional)")
def test_cad_smoke():
    r = run("tests/cad_smoke.py")
    assert r.returncode == 0, (r.stdout + r.stderr)[-3000:]
