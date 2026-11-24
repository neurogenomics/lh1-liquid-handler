"""CAD smoke test: import the size-family build module and regenerate one small output (the starter's tip supply plan),
then compare it with the committed report.  Run from the repository root: python tests/cad_smoke.py"""
import json
import sys
import tempfile
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "cad" / "lh1" / "machine_family"))
import build as B  # noqa: E402  (imports geometry, scene, checks, deck_rules, mechanisms and the library modules they load)

S = B.G.Size("S")
new = S.tips_override
out = Path(tempfile.mkdtemp()) / "tip_supply_S.json"
out.write_text(json.dumps(new, indent=1))
old = json.loads((ROOT / "cad/lh1/machine_family/out/S/report.json").read_text())["tip_supply"]["tips"]
same = {k: (new[k] == old[k]) for k in ("need", "per_type", "towers", "held_total")}
print(json.dumps({"regenerated": str(out.name), "dock_y": S.dock_y, "rows": S.rows, "matches_committed_report": same}))
sys.exit(0 if all(same.values()) else 1)
