"""Plan (top-view) drawings of the LH-1 size family, one PNG per size, from out/size_family.json and out/layouts.yaml.

Run from the project root after build.py:  python3 cad/lh1/size_family/draw.py      (needs matplotlib and PyYAML; the project .venv
has no matplotlib, so this runs on the system Python; it reads only this module's own outputs)
Out: cad/lh1/size_family/out/plan_S.png, plan_M.png, plan_L.png
Status: drawing of CALCULATED layouts; nothing built.
"""
import json
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
from matplotlib.patches import Rectangle  # noqa: E402
import yaml  # noqa: E402

HERE = Path(__file__).resolve().parent
OUT = HERE / "out"
REP = json.loads((OUT / "size_family.json").read_text())
LAY = yaml.safe_load((OUT / "layouts.yaml").read_text())

COL = {  # one colour family per function (light fills, dark text)
    "cold": "#cfe3f7", "warm": "#f7d9cf", "magnet": "#e3d7f2", "tips": "#d9efd5", "waste": "#ececec", "tubes": "#fbefc8",
    "labware": "#f3f3e6", "tc": "#f6c9c9", "shaker": "#f9dcb5", "spare": "#ffffff", "res": "#d6f0f0",
}
CLASS = {"CTB1": "cold", "CTB2": "cold", "CPB1": "cold", "STRIP": "cold", "ATR1": "tubes", "ATR2": "tubes", "TC1": "tc", "TC1LID": "tc",
         "SHAKER": "shaker", "MB1": "magnet", "MAG2": "magnet", "TIPWASTE": "waste", "LW": "waste", "P10": "tips", "P200": "tips",
         "P300": "tips", "P1000R": "tips", "TPK": "tips", "WB200": "tips", "RES1": "res", "RES2": "res", "HAZ": "warm",
         "STACK": "labware", "STACK2": "labware", "LIDPARK2": "labware", "WN1": "labware", "BAL": "labware", "SPARE": "spare"}
SHORT = {"CTB1": "CTB-1 cold\n48 x 1.5 mL", "CTB2": "CTB-2 cold\n5/15/50 mL", "ATR1": "ATR-1\n50/15 mL", "ATR2": "ATR-2\n1.5/5 mL",
         "TC1": "TC-1\nthermocycler", "TC1LID": "TC-1 lid\npark", "SHAKER": "shaker\n(sunk)", "MB1": "magnet\nMB-1", "MAG2": "magnet 2\nlow elution",
         "TIPWASTE": "tip waste\n+ VC-2", "LW": "liquid\nwaste LW-1", "P10": "p10 frame", "P200": "p200 frame", "P300": "p300 frame",
         "P1000R": "p1000 rack", "TPK": "tip parking\nTPK-1", "RES1": "reservoir\n4 x 73 mL", "RES2": "reservoir\n12-col",
         "HAZ": "hazardous liquid\ntrough", "STACK": "plate stack\nx4", "STACK2": "plate stack\nx4", "LIDPARK2": "lid park 2",
         "CPB1": "cold plate\nCPB-1", "STRIP": "cold strip\ncarrier", "WN1": "weighing\nWN-1", "BAL": "balance", "SPARE": "spare"}


def draw(name):
    g = REP["geometry"][name]
    s = LAY["sizes"][name]
    fig, ax = plt.subplots(figsize=(9.5, 9.5 * (g["frame_y"][1] - g["frame_y"][0] + 140) / (g["enclosure_x"][1] - g["enclosure_x"][0] + 260)))
    ex0, ex1 = g["enclosure_x"]
    fy0, fy1 = g["frame_y"]
    ax.add_patch(Rectangle((ex0, fy0 - 10), ex1 - ex0, fy1 - fy0 + 12, fill=False, lw=1.6, ec="#333333"))
    ax.text(ex0, fy0 - 32, f"enclosure {ex1 - ex0 + 4:.0f} wide x {fy1 - fy0 + 12:.0f} deep (door at the bottom edge); "
                           f"envelope W x D x H {g['envelope_W_D_H_mm (frame, panels, feet, drawer; no fan)']} mm, fan on the left side panel",
            fontsize=7, va="top")
    for x in g["Y_rails_x"]:
        ax.plot([x, x], [fy0 + 70, fy1 - 20], color="#888888", lw=3, solid_capstyle="butt")
    dx0, dx1 = g["deck_x"]
    dy0, dy1 = g["deck_y"]
    ax.add_patch(Rectangle((dx0, dy0), dx1 - dx0, dy1 - dy0, fill=True, fc="#fafafa", ec="#999999", lw=0.8))
    for n, p in s["places"].items():
        cx, cy = p["centre_xy"]
        k = p["station"]
        ax.add_patch(Rectangle((cx - 64, cy - 53), 128, 106, fc=COL[CLASS.get(k, "spare")], ec="#555555", lw=0.8))
        ax.text(cx - 60, cy + 46, n, fontsize=7, va="top", weight="bold")
        ax.text(cx, cy - 6, SHORT.get(k, k), fontsize=6.3, ha="center", va="center")
        ax.text(cx + 60, cy - 49, f"{p['height_mm']:g}", fontsize=5.5, ha="right", va="bottom", color="#666666")
    tc = [p["centre_xy"] for p in s["places"].values() if p["station"] == "TC1"][0]
    ax.add_patch(Rectangle((-264.0, tc[1] - 53.0), 51.0, 217.0, fc="#f6c9c9", ec="#a33", lw=0.6, hatch="///"))
    ax.text(-238, tc[1] + 55, "TC-1\nlid\nstrip", fontsize=5.5, ha="center", va="center")
    # shelf
    sx0, sx1 = 335.0, g["Y_rails_x"][1] - 30.0
    ax.add_patch(Rectangle((sx0, dy0 + 15), sx1 - sx0, g["dock_axis_y"] + 60 - dy0 - 15, fill=True, fc="#f4f4f4", ec="#999999", lw=0.6))
    for slot, what in s["shelf"].items():
        r = slot[1:]
        y = g["dock_axis_y"] if r == "K" else s["row_y"][r]
        x = 410.0 if slot[0] == "I" else 540.0
        ax.add_patch(Rectangle((x - 61, y - 40), 122, 80, fc="#ffffff", ec="#aaaaaa", lw=0.5))
        ax.text(x, y, f"{slot}\n{what.split('(')[0].strip()[:22]}", fontsize=5.0, ha="center", va="center")
    # dock
    for i in range(8):
        x = -296.0 + 78.0 * i
        ax.add_patch(Rectangle((x - 36, g["dock_axis_y"] - 45), 72, 90, fc="#e6e6f5", ec="#666699", lw=0.6))
        ax.text(x, g["dock_axis_y"], f"bay\n{i + 1}", fontsize=5.5, ha="center", va="center")
    # head travel (coupling axis reach of carriage B) and person reach
    xc0, xc1 = g["XC"]
    yf0, yf1 = g["YF"]
    ax.add_patch(Rectangle((xc0, yf0 - 102.5), xc1 - xc0, yf1 - yf0, fill=False, ec="#2a7", lw=0.8, ls="--"))
    ax.text(xc0 + 4, yf1 - 102.5 - 4, "carriage B nozzle reach", fontsize=5.5, color="#2a7", va="top")
    reach = fy0 - 10 + 600.0
    ax.plot([ex0, ex1], [reach, reach], color="#c33", lw=0.7, ls=":")
    ax.text(ex1 - 4, reach + 4, "600 mm from the door (ASSUMED comfortable reach)", fontsize=5.5, color="#c33", ha="right")
    rc = REP["rule_checks"][name]
    ci = REP["interference_and_reach"].get(name, {})
    title = (f"LH-1 size {name}: {len(s['places'])} places, {len(s['rows'])} rows, 8 dock bays; "
             f"rule check {'PASS' if rc['pass'] else 'FAIL'}"
             + (f"; machine_l interference {ci.get('interferences')} / reach failures {len(ci.get('reach_failures', []))}" if "interferences" in ci else ""))
    ax.set_title(title + "\nCALCULATED layout, nothing built; deck frame of params.py (x right, y away from the operator), mm", fontsize=8)
    ax.set_xlim(ex0 - 40, ex1 + 40)
    ax.set_ylim(fy0 - 60, fy1 + 40)
    ax.set_aspect("equal")
    ax.tick_params(labelsize=6)
    fig.tight_layout()
    fig.savefig(OUT / f"plan_{name}.png", dpi=170)
    plt.close(fig)


if __name__ == "__main__":
    for nm in ("S", "M", "L"):
        draw(nm)
    print("written", sorted(p.name for p in OUT.glob("plan_*.png")))
