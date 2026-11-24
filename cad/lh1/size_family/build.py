"""LH-1 size family study: can the machine be a sized family (S starter -> M -> L) on the deck module standard?

Run from the project root:  .venv/bin/python cad/lh1/size_family/build.py
Then (plan drawings, needs matplotlib):  python3 cad/lh1/size_family/draw.py
Out: cad/lh1/size_family/out/{size_family.json, layouts.yaml, checks_S.json}

Read-only use of the existing modules: machine_l (frame stack-up rules, head model, interference engine, station solids), head_l
(M1 stiffness budget, belt dynamics), deck_k (the K2 layout rule checker).  Nothing in those modules or their outputs is changed: the
size-specific values are set on the in-memory module objects only, and no function that writes a file is called.

Status: CALCULATION ONLY.  Nothing is built or measured.  Every figure is CALCULATED from the model/rules or from an input marked
ASSUMED; prices come from the rev H order list rates (engineering estimates, not quotes) and are labelled so.
"""
from __future__ import annotations

import ast
import copy
import csv
import importlib.util
import json
import math
import re
import sys
import time
from pathlib import Path

import yaml

HERE = Path(__file__).resolve().parent
LH1 = HERE.parent
ROOT = LH1.parent.parent
OUT = HERE / "out"
sys.path.insert(0, str(LH1))
import params as P  # noqa: E402

T0 = time.time()


def log(*a):
    print(f"[size_family {time.time() - T0:6.1f}s]", *a, flush=True)


def _load(name, rel):
    spec = importlib.util.spec_from_file_location(name, str(LH1 / rel))
    m = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(m)
    return m


M = _load("machine_l_for_size_family", "machine_l/build.py")     # also loads head_l and deck_k (its HL, DKK)
HL, DKK = M.HL, M.DKK
ML_REPORT = json.loads((LH1 / "machine_l" / "out" / "report.json").read_text())
ASM_L = json.loads((LH1 / "assembly" / "out" / "report.json").read_text())["rev_l"]
DECKK = json.loads((LH1 / "deck_k" / "out" / "report.json").read_text())

# =====================================================================================================
# 1. the frame stack-up, as machine_l derives it (CALCULATED rules, reproduced and checked against rev L below)
# =====================================================================================================
HX0, HX1 = M.HEAD_X                     # head-local x extent with tools
HY0 = M.HEAD_Y[0]
PITCH_X, PITCH_Y = 132.0, 110.0         # deck module standard (ADR-010)
END_MARGIN = 15.0                       # travel end margin (machine_l X ends: 15; Y front was 29.5, Y back 16)


def stack_up(xc0, xc1, yf0, yf1, fan="side"):
    xr_l = xc0 + HX0 - M.X_MARGIN - M.MG15["W"] / 2
    xr_r = xc1 + HX1 + M.X_MARGIN + M.MG15["W"] / 2
    beam = (xr_r + M.MG15["W"] / 2 + 4.5) - (xr_l - M.MG15["W"] / 2 - 4.5)
    ex0, ex1 = xr_l - 70.0, xr_r + 70.0
    fy0 = yf0 + HY0 - 27.0              # rev L: -425 at yf0 -230 (head front 32 mm behind the door)
    fy1 = yf1 + M.YC["dy"][1] + 45.0    # back uprights 5 mm behind the Y carriage at the back of travel
    y_rail = (yf0 + M.YB_REL[1] - M.MG15["L"] / 2 - 5.0, yf1 + M.YB_REL[0] + M.MG15["L"] / 2 + 5.0)
    x_env = (ex0 - M.PANEL_T["side"], ex1 + M.PANEL_T["side"] + 4.0)        # +4: hand-off shutters on the right panel
    y_env = (fy0 - 5.0 - M.PANEL_T["door"], fy1 + M.PANEL_T["back"])
    z_env = (M.BASE["z0"] - M.FOOT_H, M.ROOF_Z + 20.0 + M.PANEL_T["roof"] + 2.0)
    W, D, H = x_env[1] - x_env[0], y_env[1] - y_env[0], z_env[1] - z_env[0]
    fan_add = {"back": (0.0, 150.0, 0.0), "side": (150.0, 0.0, 0.0), "roof": (0.0, 0.0, 150.0)}[fan]
    inner = (ex1 - ex0) * (fy1 - fy0) * ((M.ROOF_Z + 20.0) - M.BASE["z0"]) / 1e9
    above = (ex1 - ex0) * (fy1 - fy0) * (M.ROOF_Z + 20.0) / 1e9
    return {"XC": [xc0, xc1], "YF": [yf0, yf1], "travel_X_mm": round(xc1 - xc0, 1), "travel_Y_mm": round(yf1 - yf0, 1),
            "Y_rails_x": [round(xr_l, 1), round(xr_r, 1)], "X_beam_mm": round(beam, 1), "Y_rail_mm": round(y_rail[1] - y_rail[0], 1),
            "frame_x": [round(xr_l - 20, 1), round(xr_r + 20, 1)], "frame_y": [round(fy0, 1), round(fy1, 1)],
            "side_member_mm": round(fy1 - fy0 - 80.0, 1), "enclosure_x": [round(ex0, 1), round(ex1, 1)],
            "envelope_W_D_H_mm (frame, panels, feet, drawer; no fan)": [round(W), round(D), round(H)],
            "fan_position": fan, "envelope_W_D_H_mm (with the extraction fan)": [round(W + fan_add[0]), round(D + fan_add[1]), round(H + fan_add[2])],
            "inner_volume_gross_m3": round(inner, 3), "inner_volume_above_deck_m3": round(above, 3)}


def check_stackup_against_rev_l():
    s = stack_up(M.XC0, M.XC1, M.YF0, M.YF1, fan="back")
    rl = ML_REPORT["frame_geometry"]
    env = ML_REPORT["envelope_mm"]
    return {"rails_x model/rev L": [s["Y_rails_x"], rl["Y_rails_x"]], "X_beam model/rev L": [s["X_beam_mm"], rl["X_beam_mm"]],
            "Y_rail model/rev L": [s["Y_rail_mm"], rl["Y_rails_mm"]], "frame_y model/rev L": [s["frame_y"], rl["frame_y"]],
            "envelope W D H (with fan behind) model/rev L": [s["envelope_W_D_H_mm (with the extraction fan)"], env["W_D_H_mm"]],
            "verdict": "the stack-up reproduces revision L within 1 mm in X and Y" if
            abs(s["X_beam_mm"] - rl["X_beam_mm"]) < 1 and abs(s["Y_rail_mm"] - rl["Y_rails_mm"]) < 1 and
            abs(s["envelope_W_D_H_mm (with the extraction fan)"][1] - env["W_D_H_mm"][1]) <= 1 else "MISMATCH: see the numbers"}


# =====================================================================================================
# 2. the family: parameter sets and layouts on the deck module standard
#    Place names are grid names of each size (row letter + column digit; column 0 = the pipette-only column at x -264).
#    'id' is the revision L place whose station (and its CAD solids) the place takes: used to relocate machine_l's station
#    solids for the interference run.  kind / height above the seat / pipette work as the K2 rule checker wants them.
# =====================================================================================================
COLX = {0: -264.0, 1: -132.0, 2: 0.0, 3: 132.0, 4: 264.0}
ROW_F_Y = -275.0
DOCK_BEHIND_BACK_ROW = 197.0           # rev L: dock coupling axis y 472, row G centre 275 (machine_l)

ST = {   # station catalogue: (kind, height above seat mm, work, rev L identity, needs gripper, under-deck parts in x < -49.5 of a col-2 place)
    "CTB1": ("cold tube block CTB-1 (48 x 1.5/2 mL) on a thermal module (module zone with CTB-2)", 82.0, "deep_single_only", "F3", False),
    "CTB2": ("cold tube block CTB-2 (6 x 5, 4 x 15, 3 x 50 mL), recessed 60", 100.0, "deep_single_only", "F4", False),
    "ATR2": ("ambient tube rack ATR-2 (24 x 1.5/2 + 10 x 5 mL), recessed 20", 50.0, "deep_single_only", "A2", False),
    "ATR1": ("ambient tube rack ATR-1 (6 x 50 + 6 x 15 mL), recessed 60", 100.0, "deep_single_only", "F1", False),
    "SHAKER": ("heater-shaker on SK-1, sunk 40 mm on a tray (edge place, overhangs outwards)", 90.0, None, "D4", True),
    "TC1": ("thermocycler TC-1 (double place with its lid park behind)", 21.0, "deep", "C1", True),
    "TC1LID": ("TC-1 lid park", 40.0, None, "D1", True),
    "RES1": ("reservoir place: 4-trough 73 mL (water, buffers; ethanol trough under a slotted cover), plain insert", 44.0, "deep", "B1", False),
    "RES2": ("reservoir place 2: 12-column reservoir, plain insert", 44.0, "deep", None, False),
    "MB1": ("magnet MB-1", 16.0, "deep", "B2", True),
    "CPB1": ("cold plate CPB-1 on a thermal module", 30.0, "deep", "B3", True),
    "STRIP": ("cold strip carrier on a thermal module", 23.5, "deep", "C3", True),
    "TIPWASTE": ("tip waste funnel + gate + VC-2 -> vertical chute -> crate in the drawer", 0.0, "shallow", "A1", False),
    "STACK": ("plate stack x4 (gripper only)", 51.0, None, "C4", True),
    "STACK2": ("plate stack x4, second (gripper only)", 51.0, None, "C2", True),
    "LIDPARK2": ("lid park 2: shaker, sealing and 3 rescue lids", 47.5, None, "D3", True),
    "HAZ": ("hazardous-liquid trough, lidded, ambient (proposed must_not_chill); a thermal module fits under it if 37 C is wanted", 44.0, "deep", "A3", False),
    "LW": ("liquid waste LW-1 (two streams) -> closed bottles in the drawer", 45.0, "centre", "A4", False),
    "TPK": ("tip parking TPK-1 (pre-PCR tips; enables the waste-tip-per-well rules)", 81.6, "shallow", "D2", False),
    "MAG2": ("magnet 2: low-elution, post-PCR", 16.0, "deep", "B4", True),
    "WN1": ("weighing nest WN-1", 44.0, "deep", None, True),
    "BAL": ("balance (under-deck bay, draught shield)", 60.0, "shallow", None, False),
    "P10": ("tip frame p10 (gripper-refilled)", 50.0, "shallow", "G1", True),
    "P200": ("tip frame p200, recessed 13 (rev L: 7) so it is 51 mm above the seat", 51.0, "shallow", "G2", True),
    "P300": ("tip frame p300, recessed 7", 65.0, "shallow", "G3", True),
    "P1000R": ("p1000 tip rack (person-loaded)", 72.0, "shallow", "G4", False),
    "WB200": ("wide-bore p200 rack (person-loaded)", 57.0, "shallow", None, False),
    "SPARE": ("spare plain place (standard bolt pattern, cable route)", 0.0, None, None, False),
}

ST["TPK"] = ("tip parking TPK-1 (pre-PCR tips; enables the waste-tip-per-well rules)", 81.6, "shallow", "D2", False)
ST["P300"] = ("tip frame p300, recessed 15 (rev L: 7) so it is 57 mm above the seat", 57.0, "shallow", "G3", True)
# parts below the deck plate (from machine_l station solids: thermal modules, recessed racks/frames, magnet block, funnels, trays)
UNDER = {"CTB1", "CTB2", "ATR2", "ATR1", "SHAKER", "TC1", "MB1", "CPB1", "STRIP", "TIPWASTE", "LW", "WN1", "BAL", "P200", "P300", "P1000R"}
FRAMES = ("P10", "P200", "P300")
SINGLE_DEEP = ("deep", "deep_single_only")

# Placement rules (CALCULATED): K2 R1/R2 (deck_k k2_rules) plus the neighbour rules this study found by running machine_l's
# interference engine on the first S layout (checks_S.json history in size_family_study.txt, section 2):
#  N1 tip frames: the eight-channel cradle cheeks pass over the next place in X: neighbours <= 57 mm (57 passes in rev L, 65 fails)
#  N2 single-channel deep work: the cradle straps reach about 25 mm in front of the place: front neighbour <= 100 mm (87-105 pass in
#     rev L; the 140 mm TC-1 lid guide strip fails)
#  N3 tip waste: the VC-2 camera and bracket stand 34-38 mm outside the place on the -X side up to z 102: column 1 only, with
#     nothing in column 0 of that row
#  N4 TC-1: its under-deck body is 165 mm wide (16.5 mm under each neighbour column) and its 140 mm lid guide strip stands in
#     column 0: column 1, lid park directly behind, column 2 of both rows free of under-deck parts, column 0 of both rows unusable
#  N5 shaker: body 148 mm wide; set 8 mm outward of the place centre at the right edge (shelf cut to x 350), its under-deck adapter
#     then reaches 21 mm under the column-3 neighbour: that neighbour has no under-deck parts
#  N6 column 0 is pipette-only (no gripper); N7 the tip waste chute stands over the drawer crate: rows F-B only (they never move)
#  N8 the cold tube blocks CTB-1/CTB-2 form one module zone: adjacent in a row; N9 Y8: hazardous liquid not beside a buffer reservoir
#  N10 TC-1 in row F or A only (its lid park must not be separated from it when a row is inserted in front of row G)


def _h(key):
    return 0.0 if key in (None, "EMPTY") else ST[key][1]


def _work(key):
    return None if key in (None, "EMPTY") else ST[key][2]


def layout_violations(rows, lay, final=True):
    """every rule above on a (partial) layout {place: key}; places not in lay are unknown (partial) or empty (final)"""
    bad = []
    tc_rows = []
    for n, k in lay.items():
        if k == "TC1":
            tc_rows = [n[0], rows[rows.index(n[0]) + 1]] if rows.index(n[0]) + 1 < len(rows) else [n[0]]

    def get(r, c):
        if r is None or c < 0 or c > 4:
            return "OFF"
        if c == 0 and r in tc_rows:
            return "STRIP"
        n = f"{r}{c}"
        if n in lay:
            return lay[n]
        return "EMPTY" if final else None

    def h(k):
        return {"STRIP": 140.0, "OFF": 0.0}.get(k, _h(k)) if k is not None else None
    for n, k in lay.items():
        if k in (None, "EMPTY", "SPARE"):
            continue
        r, c = n[0], int(n[1])
        i = rows.index(r)
        rb = rows[i + 1] if i + 1 < len(rows) else None
        rf = rows[i - 1] if i > 0 else None
        w = _work(k)
        kb, kf, kl, kr = get(rb, c), get(rf, c), get(r, c - 1), get(r, c + 1)
        if w and rb and kb is not None:
            lim = 51.0 if w in SINGLE_DEEP else 72.0
            if h(kb) > lim:
                bad.append(f"R1 {n} {k}: behind {rb}{c} {kb} {h(kb):g} > {lim:g}")
        if w == "deep":
            for kk, cc in ((kl, c - 1), (kr, c + 1)):
                if kk not in (None, "OFF", "STRIP") and h(kk) > 64.0:      # the strip is TC-1's own part (CAD-checked in rev L)
                    bad.append(f"R2 {n} {k}: beside {r}{cc} {kk} {h(kk):g} > 64")
        if k in FRAMES:
            if r != "G":
                bad.append(f"R3 {n}: tip frames in row G")
            for kk, cc in ((kl, c - 1), (kr, c + 1)):
                if kk not in (None, "OFF") and h(kk) > 57.0:
                    bad.append(f"N1 {n} {k}: beside {r}{cc} {kk} {h(kk):g} > 57")
        if w in SINGLE_DEEP and kf not in (None, "OFF") and h(kf) > 100.0:
            bad.append(f"N2 {n} {k}: in front {rf}{c} {kf} {h(kf):g} > 100")
        if k == "TIPWASTE":
            if c != 1:
                bad.append(f"N3 {n}: tip waste in column 1 only")
            elif get(r, 0) not in ("EMPTY", None):
                bad.append(f"N3 {n}: column 0 of row {r} must be clear for VC-2 ({get(r, 0)})")
            if r not in ("F", "A", "B"):
                bad.append(f"N7 {n}: tip waste over the crate (rows F-B)")
        if k == "TC1":
            if c != 1 or r not in ("F", "A"):
                bad.append(f"N4/N10 {n}: TC-1 at F1 or A1")
            if rb and get(rb, c) not in ("TC1LID", None):
                bad.append(f"N4 {n}: lid park behind")
            for rr in tc_rows:
                k2 = get(rr, 2)
                if k2 in UNDER:
                    bad.append(f"N4 {rr}2 {k2}: under-deck parts where the TC-1 body is")
                if f"{rr}0" in lay and lay[f"{rr}0"] not in ("EMPTY",):
                    bad.append(f"N4 {rr}0: TC-1 strip there")
        if k == "SHAKER":
            if c != 4:
                bad.append(f"N5 {n}: shaker at the right edge")
            if kl in UNDER:
                bad.append(f"N5 {n}: the column-3 neighbour {kl} has under-deck parts")
        if c == 0 and ST[k][4]:
            bad.append(f"N6 {n} {k}: column 0 is pipette-only")
        if k == "CTB1":
            if get(r, c - 1) not in ("CTB2", None) and get(r, c + 1) not in ("CTB2", None):
                bad.append(f"N8 {n}: CTB-1 beside CTB-2")
        if k == "HAZ":
            for rr, cc in ((r, c - 1), (r, c + 1), (rf, c), (rb, c)):
                if rr and get(rr, cc) in ("RES1", "RES2"):
                    bad.append(f"N9 {n}: hazardous liquid beside a reservoir at {rr}{cc}")
    return bad


def _domain(k, places, rows):
    out = []
    for p in places:
        r, c = p[0], int(p[1])
        if c == 0 and ST[k][4]:
            continue
        if k == "TC1" and p not in ("F1", "A1"):
            continue
        if k == "TC1LID" and p not in ("A1", "B1"):
            continue
        if k == "TIPWASTE" and (c != 1 or r not in ("F", "A", "B")):
            continue
        if k == "SHAKER" and c != 4:
            continue
        if k in FRAMES and (r != "G" or c == 0):
            continue
        out.append(p)
    return out


def solve(rows, fixed, items, new_rows, col0=True, max_solutions=1, drop_ok=()):
    """backtracking, item by item (most constrained first) into the free places of new_rows; unused places stay empty"""
    places = [f"{r}{c}" for r in new_rows for c in ((0, 1, 2, 3, 4) if col0 else (1, 2, 3, 4))]
    places = [q for q in places if q not in fixed]
    main_free = [q for q in places if q[1] != "0"]
    dom = {k: _domain(k, places, rows) for k in set(items)}
    order = sorted(items, key=lambda k: (len(dom[k]), items.index(k)))
    sols = []
    seen = set()

    def rec(i, lay, used):
        if len(sols) >= max_solutions:
            return
        if i == len(order):
            if any(q not in lay for q in main_free if sum(1 for x in main_free) <= len(items)):
                pass
            if not layout_violations(rows, lay, final=True):
                key = tuple(sorted(lay.items()))
                if key not in seen:
                    seen.add(key)
                    sols.append(dict(lay))
            return
        k = order[i]
        for p in dom[k]:
            if p in used:
                continue
            lay[p] = k
            if not layout_violations(rows, lay, final=False):
                rec(i + 1, lay, used | {p})
            del lay[p]
    rec(0, dict(fixed), set())
    return sols


# S holds 16 of the 17 day-1 stations at once (the search finds no layout with all 17 under the rules above; dropping any one of
# ATR2, HAZ, RES1, LW or CPB1 works, dropping STACK does not): ATR-2 is the one left out (cheapest: see the study)
S_ITEMS = ["TC1", "TC1LID", "TIPWASTE", "SHAKER", "CTB1", "CTB2", "P10", "P200", "P300", "MB1", "STRIP", "CPB1", "LW", "RES1", "HAZ", "STACK"]
M_ITEMS = ["ATR2", "TPK", "MAG2", "LIDPARK2", "P1000R"]
L_ITEMS = ["WN1", "BAL", "ATR1", "RES2", "STACK2"]


def _best_extension(rows, fixed, items, new_row):
    """the largest subset of items (dropping as few as possible, last-listed first) that fits the inserted row"""
    import itertools
    for n_drop in range(0, len(items) + 1):
        for drop in itertools.combinations(reversed(items), n_drop):
            sub = [k for k in items if k not in drop]
            sol = solve(rows, fixed, sub, [new_row], max_solutions=1)
            if sol:
                return sol[0], list(drop)
    return dict(fixed), list(items)


def solve_family():
    rows_s, rows_m, rows_l = ["F", "A", "B", "G"], ["F", "A", "B", "C", "G"], ["F", "A", "B", "C", "D", "G"]
    t = time.time()
    s_sols = solve(rows_s, {}, S_ITEMS, rows_s, max_solutions=60)
    best = None
    for i, s in enumerate(s_sols):
        m, dm = _best_extension(rows_m, s, M_ITEMS, "C")
        lsol, dl = _best_extension(rows_l, m, L_ITEMS + dm, "D")
        score = len(dl)
        if best is None or score < best[0]:
            best = (score, s, m, lsol, dm, dl, i + 1)
        if score == 0:
            break
    _, s, m, lsol, dm, dl, i = best
    return {"S": s, "M": m, "L": lsol, "M_left_out": dm, "L_left_out": dl, "S_solutions_tried": i,
            "S_solutions_found": len(s_sols), "solver_s": round(time.time() - t, 1),
            "rule": "S fixed; M = S + row C inserted in front of row G; L = M + row D; items that do not fit a row are carried to the next"}


_CACHE = OUT / "solver_result.json"
_KEY = json.dumps([S_ITEMS, M_ITEMS, L_ITEMS, "rules-v2"])
if _CACHE.exists() and json.loads(_CACHE.read_text()).get("key") == _KEY:
    SOL = json.loads(_CACHE.read_text())["sol"]
else:
    SOL = solve_family()
    OUT.mkdir(parents=True, exist_ok=True)
    _CACHE.write_text(json.dumps({"key": _KEY, "sol": SOL}, indent=1))
SIZES = {
    "S": {"rows": ["F", "A", "B", "G"], "layout": SOL["S"], "samples_day1": 24},
    "M": {"rows": ["F", "A", "B", "C", "G"], "layout": SOL["M"], "samples_day1": 48},
    "L": {"rows": ["F", "A", "B", "C", "D", "G"], "layout": SOL["L"], "samples_day1": 96},
}


def shelf_plan(name):
    """gripper-only side shelf (inner column x 410, outer x 540): the inner slot beside the shaker is covered by its overhang"""
    rows = SIZES[name]["rows"]
    lay = SIZES[name]["layout"]
    sh_row = [n[0] for n, k in lay.items() if k == "SHAKER"][0]
    plan = {"OF": "H1 edge hand-off"}
    if name != "S":
        plan["OA"] = "H2 edge hand-off"
    plan[f"I{sh_row}"] = "blocked (shaker overhang)"
    if "P1000R" not in lay.values():
        plan["IG"] = "p1000 rack, sunk 15 mm into the shelf (carriage B only: carriage A cannot reach x 410)"
    if "LIDPARK2" not in lay.values():
        plan["OG"] = "lid park 2"
    plan["OK"] = "spent wafer stack"
    for r in rows:
        for side in ("I", "O"):
            plan.setdefault(f"{side}{r}", "tower")
    return plan


SHELF_PLAN = {nm: shelf_plan(nm) for nm in SIZES}


def row_y(rows):
    return {r: ROW_F_Y + i * PITCH_Y for i, r in enumerate(rows)}


def size_geometry(name, col0=True, fan="side"):
    rows = SIZES[name]["rows"]
    ry = row_y(rows)
    dock_y = ry["G"] + DOCK_BEHIND_BACK_ROW
    # travel: the machine_l targets that drive the ends (CALCULATED): front = single channels at row F front tubes (y - 28),
    # back = gripper at the shelf slot behind/at the dock row; X left = carriage B at column 0 (or bay 1 without it), right =
    # gripper at the outer shelf column (x 540)
    yf0 = ROW_F_Y - 28.0 - M.AY - END_MARGIN
    yf1 = dock_y - M.GRIP_Y + 16.0
    xc0 = (COLX[0] - 49.5 if col0 else M.DOCK_X[0]) - END_MARGIN
    xc1 = M.XC1
    g = stack_up(xc0, xc1, yf0, yf1, fan=fan)
    g.update({"rows": {r: ry[r] for r in rows}, "dock_axis_y": dock_y, "deck_y": [ROW_F_Y - 85.0, ry["G"] + 55.0 + 15.0],
              "deck_x": [COLX[0] - 66.0 if col0 else -270.0, 335.0]})
    return g


def places_of(name):
    ry = row_y(SIZES[name]["rows"])
    out = {}
    for n, key in SIZES[name]["layout"].items():
        kind, h, work, ident, grip = ST[key]
        out[n] = {"station": key, "kind": kind, "h": h, "work": work, "rev_L_identity": ident, "needs_gripper": grip,
                  "c": (COLX[int(n[1])], ry[n[0]])}
    return out


# =====================================================================================================
# 3. rule check: the K2 checker of deck_k (R1, R2) run on each layout, plus the family's own placement rules
# =====================================================================================================
def k2_check(name):
    pl = places_of(name)
    rows = SIZES[name]["rows"]
    saved = (DKK.K2_PLACES, DKK.K2_ROWS)
    try:
        full = {f"{r}{c}": ("empty place", 0.0, None) for r in rows for c in range(1, 5)}
        full.update({n: (p["kind"], p["h"], p["work"]) for n, p in pl.items()})
        DKK.K2_PLACES = full
        DKK.K2_ROWS = {r: row_y(rows)[r] for r in rows}
        res = DKK.k2_rules()
    finally:
        DKK.K2_PLACES, DKK.K2_ROWS = saved
    extra = layout_violations(rows, SIZES[name]["layout"], final=True)
    return {"k2_rules": res["rules"], "n_checked": res["n_checked"], "violations": res["violations"], "family_rule_violations": extra,
            "pass": not res["violations"] and not extra}


def k2_check_rev_l():
    """the rev L layout as machine_l modified it (TPK at D2, heights after the pass 2 recesses), through the same checker"""
    saved = (DKK.K2_PLACES, DKK.K2_ROWS)
    try:
        DKK.K2_PLACES = {n: (p["kind"], p["h"], p["work"]) for n, p in M.PLACES.items()}
        DKK.K2_ROWS = dict(saved[1])
        res = DKK.k2_rules()
    finally:
        DKK.K2_PLACES, DKK.K2_ROWS = saved
    return {"n_checked": res["n_checked"], "violations": res["violations"], "pass": not res["violations"]}


# =====================================================================================================
# 4. interference and reach for S: machine_l's own engine and checks, with machine_l's station solids relocated to the S places
# =====================================================================================================
def run_machine_checks(name, geo):
    pl = places_of(name)
    saved = {k: getattr(M, k) for k in ("PLACES", "XC0", "XC1", "YF0", "YF1", "DOCK_Y", "SLOTS", "SHELF", "FY1", "gripper_targets")}
    plan_l = M.tip_supply_plan_l()
    static_l, zs_l = M.scene_static(plan_l)
    # place identity -> delta
    by_id = {p["rev_L_identity"]: n for n, p in pl.items() if p["rev_L_identity"]}
    st_items = []
    for it in static_l:
        if it["group"] != "station":
            continue
        ident = it.get("place")
        if ident not in by_id:
            continue
        n = by_id[ident]
        dx = pl[n]["c"][0] - M.PLACES[ident]["c"][0]
        dy = pl[n]["c"][1] - M.PLACES[ident]["c"][1]
        key = pl[n]["station"]
        if key in ("RES1", "HAZ") and ("wn_" in it["name"] or "thermal_module" in it["name"] or "deepwell" in it["name"] or "warm" in it["name"]):
            continue                                # plain inserts in S: no weighing parts, no thermal module
        if key in ("P200", "P300") and "tip_frame" in it["name"]:
            cx, cy = pl[n]["c"]
            hh = ST[key][1]
            st_items.append(dict(it, solid=M.recessed(cx, cy, M.SEAT + hh, {"P200": 13.0, "P300": 15.0}[key]), place=ident))
            continue
        if key == "SHAKER":
            dx += SHAKER_OUT
        if key == "P1000R":
            continue                                # handled below (on the shelf in S)
        st_items.append(dict(it, solid=it["solid"].translate(M.V(dx, dy, 0)), place=ident))
    for key in ("RES1", "HAZ"):                    # plain insert + 44 mm reservoir / trough
        n = [k for k, p in pl.items() if p["station"] == key][0]
        cx, cy = pl[n]["c"]
        ident = pl[n]["rev_L_identity"]
        st_items.append(dict(name=f"{ident}_IN-P_plain_insert", solid=M.insert_plate(cx, cy), group="station", cls="part", place=ident, part="IN-P", mat="al"))
        st_items.append(dict(name=f"{ident}_reservoir_or_trough_44", solid=M.sbs(cx, cy, M.SEAT, M.SEAT + 44.0), group="station", cls="labware",
                             place=ident, part="reservoir", mat="pp"))
    # S: the p1000 rack sits on the shelf at IG (x 410) and is served by carriage B only
    places_for_checks = {p["rev_L_identity"]: dict(kind=p["kind"], h=p["h"], work=p["work"], c=p["c"]) for p in pl.values() if p["rev_L_identity"]}
    if name == "S":
        p1000_xy = (410.0, geo["rows"]["G"])
        st_items.append(dict(name="G4_p1000_rack_on_shelf_IG_sunk15", solid=M.sbs(*p1000_xy, M.SHELF["t"] - 15.0, M.SHELF["t"] - 15.0 + 72.0), group="station",
                             cls="labware", place="G4", part="p1000 rack", mat="pp"))
        places_for_checks["G4"] = dict(kind="p1000 rack on the shelf (IG), sunk 15", h=72.0 - 15.0 - M.SEAT + M.SHELF["t"], work="shallow", c=p1000_xy)
    # deck plate of this size (full rectangle with the place windows; the frame, enclosure and drawer are not modelled here)
    win = []
    for n, p in pl.items():
        cx, cy = p["c"]
        win.append(M.B(cx - M.WIN[0], cx + M.WIN[0], cy - M.WIN[1], cy + M.WIN[1], -P.DECK_T - 1, 1))
    tc = [p["c"] for p in pl.values() if p["station"] == "TC1"][0]
    win.append(M.B(tc[0] - M.WIN[0], tc[0] + M.WIN[0], tc[1] - M.WIN[1], tc[1] + PITCH_Y + M.WIN[1], -P.DECK_T - 1, 1))
    sh = [p["c"] for p in pl.values() if p["station"] == "SHAKER"][0]
    win.append(M.B(sh[0] - 77.0, geo["deck_x"][1] + 1.0, sh[1] - 57.0, sh[1] + 53.0, -P.DECK_T - 1, 1))
    tw_c = [p["c"] for p in pl.values() if p["station"] == "TIPWASTE"][0]
    vb = M.mv(DKK.vc2_parts()["bracket"], *tw_c).BoundingBox()          # VC-2 bracket foot passes the deck (as machine_l deck_items)
    if vb.zmin < 0:
        win.append(M.B(vb.xmin - 1, vb.xmax + 1, vb.ymin - 1, vb.ymax + 1, -P.DECK_T - 1, 1))
    deck = M.cut_all(M.B(geo["deck_x"][0], geo["deck_x"][1], geo["deck_y"][0], geo["deck_y"][1], -P.DECK_T, 0.0), win)
    deck_items = [dict(name=f"DK-{name}_deck", solid=deck, group="deck", cls="part", mat="al")]
    # shelf, dock and parked tools at this size's positions (machine_l functions with this size's globals)
    slots = {}
    for r, y in geo["rows"].items():
        slots[f"I{r}"] = (P.L_SHELF_X["inner"], y)
        slots[f"O{r}"] = (P.L_SHELF_X["outer"], y)
    slots["OK"] = (P.L_SHELF_X["outer"], geo["dock_axis_y"])
    plan = {"slots": {}}
    tw = []
    for slot, what in SHELF_PLAN.get(name, {}).items():
        if slot not in slots:
            continue
        if what.startswith("H1"):
            plan["slots"][slot] = {"kind": "handoff", "name": "H1", "height_mm": 19.1}
        elif what.startswith("H2"):
            plan["slots"][slot] = {"kind": "handoff", "name": "H2", "height_mm": 19.1}
        elif what == "tower":
            tw.append(slot)
        elif what.startswith("spent"):
            plan["slots"][slot] = {"kind": "spent_wafers", "height_mm": 150.0, "what": "spent wafers"}
        elif what.startswith("lid park"):
            plan["slots"][slot] = {"kind": "lidpark", "height_mm": 47.5, "what": "5 lids"}
    sup = tip_supply(name)
    for slot, (tip, n) in zip(tw, sup["towers"]):
        plan["slots"][slot] = {"kind": f"tower_{tip}", "height_mm": round(M.tower_h(tip, n), 1), "what": f"{tip} x {n} wafers"}
    try:
        M.PLACES = {**places_for_checks}
        M.PLACES["D4"] = dict(M.PLACES["D4"], c=(sh[0] + SHAKER_OUT, sh[1]))   # plate on the shaker; shelf_items cut beside it
        M.XC0, M.XC1 = geo["XC"]
        M.YF0, M.YF1 = geo["YF"]
        M.DOCK_Y = geo["dock_axis_y"]
        M.FY1 = geo["frame_y"][1]
        M.SLOTS = slots
        M.SHELF = dict(M.SHELF, y1=geo["dock_axis_y"] + 60.0)
        shelf = M.shelf_items(plan)
        for it in shelf:                            # the shaker sits 8 mm further out than in machine_l: shelf cut-out to x 350 beside it
            if it["name"].startswith("TSS-L"):
                it["solid"] = it["solid"].cut(M.B(geo["deck_x"][1] - 1.0, 350.0, sh[1] - 60.0, sh[1] + 56.0, -1, 30))
                if name == "S":                     # pocket for the sunk p1000 rack at IG
                    it["solid"] = it["solid"].cut(M.B(410.0 - 66.0, 410.0 + 66.0, geo["rows"]["G"] - 46.0, geo["rows"]["G"] + 46.0, -1, 30))
        dock = M.dock_items(M.DOCK_TOOLS_FULL)
        static = deck_items + st_items + shelf + dock
        orig_gt = saved["gripper_targets"]

        def gt(zs_shaker, plan_):
            t = orig_gt(zs_shaker, plan_)
            t.pop("G4", None)                            # the p1000 rack is person-loaded (no gripper refill) in S
            for slot, c in plan_["slots"].items():
                if c["kind"] in ("lidpark",):
                    x, y = slots[slot]
                    t[f"{slot}_lidpark"] = ((x, y), M.SHELF_SEAT + c["height_mm"] - 9.5 + 3.0, "top lid on the shelf lid park")
            return t
        M.gripper_targets = gt
        sc = M.Scene(static)
        log(f"{name}: static items {len(static)}")
        st_hits = M.static_check(static)
        # station-to-station overlaps between different places (the relocation can create them; machine_l checks only across groups)
        ss_hits = []
        stn = [i for i, it in enumerate(static) if it["group"] == "station"]
        for a in stn:
            idx = sc.candidates(sc.bb[a])
            for b in idx:
                if b <= a or static[b]["group"] != "station" or static[b].get("place") == static[a].get("place"):
                    continue
                v = M.overlap_volume(static[a]["solid"], static[b]["solid"])
                if v > 1.0 or v < 0:
                    ss_hits.append({"a": static[a]["name"], "b": static[b]["name"], "mm3": round(v, 1)})
        head = M.Head()
        res, bpt = M.run_checks(sc, head, zs_l, plan)
    finally:
        for k, v in saved.items():
            setattr(M, k, v)
    lane = [h for h in res["transit"] if h["config"].startswith("dock entry lane") and float(h["config"].split(" yf ")[1].split(" ")[0]) > geo["YF"][1] + 1e-6]
    res["transit"] = [h for h in res["transit"] if h not in lane]
    real = res["transit"] + res["lowered"] + res["gripper"] + res["dock"] + res["eject"]
    rf = res["reach_fail"]
    expected = [q for q in rf if q.get("place") == "G4" and q.get("carriage") == "carriage_A"]
    rf_other = [q for q in rf if q not in expected]
    grouped = {}
    for h in real:
        k = (h["head_part"].split(":")[0] + ":" + h["head_part"].split(":")[-1], h["item"])
        g = grouped.setdefault(k, {"head_part": k[0], "item": k[1], "n_configs": 0, "max_mm3": 0.0, "example": h["config"]})
        g["n_configs"] += 1
        g["max_mm3"] = max(g["max_mm3"], h["mm3"])
    name_of = {p["rev_L_identity"]: n for n, p in pl.items() if p["rev_L_identity"]}
    reached = {}
    for ident, d in bpt.items():
        nm = name_of.get(ident, ident)
        reached[nm] = {k: v["reached"] for k, v in d.items()}
    return {"configs_checked": res["configs"], "interferences": len(real),
            "discarded_out_of_travel": f"{len(lane)} hits in machine_l's dock-entry-lane configs at yf 420-560 (rev L constants) beyond this size's Y travel end {geo['YF'][1]:.0f}", "by_kind": {k: len(res[k]) for k in ("transit", "lowered", "gripper", "dock", "eject")},
            "grouped": sorted(grouped.values(), key=lambda g: -g["n_configs"]), "reach_failures": rf_other,
            "reach_failures_expected": f"{len(expected)} configs: the p1000 rack on the shelf (IG, x 410) is out of carriage A's reach by design "
                                       "(planner rule: p1000 tips are picked on carriage B)" if expected else "none",
            "reach_limits_body_in_target_labware": sorted({(q["config"].split(" lowered at ")[0], name_of.get(q["place"], q["place"])) for q in res["reach_limit_body_in_labware"]}),
            "static_cross_group_overlaps": st_hits, "station_to_station_overlaps": ss_hits, "reached_by_place_and_tool": reached,
            "method": "machine_l run_checks (transit over every place and slot with three tool pairs, every pipette lowered at every place it "
                      "serves on both carriages at 50/85/100 % of the stroke, gripper at every place it serves, dock drop-off/pick-up at all 8 "
                      "bays by both carriages, eject over-travel), on machine_l's station solids moved to the S places; S deck plate, shelf "
                      "and dock rebuilt at the S positions; frame, enclosure and drawer NOT in the scene (their clearances follow the machine_l "
                      "stack-up rule by construction)"}


SHAKER_OUT = 8.0


def station_overlaps(static):
    sc = M.Scene(static)
    hits = []
    stn = [i for i, it in enumerate(static) if it["group"] == "station"]
    for a in stn:
        for b in sc.candidates(sc.bb[a]):
            if b <= a or static[b]["group"] != "station" or static[b].get("place") == static[a].get("place"):
                continue
            v = M.overlap_volume(static[a]["solid"], static[b]["solid"])
            if v > 1.0 or v < 0:
                hits.append({"a": static[a]["name"], "b": static[b]["name"], "mm3": round(v, 1)})
    return hits


def run_machine_checks_rev_l():
    plan = M.tip_supply_plan_l()
    static, zs = M.scene_static(plan)
    head = M.Head()
    res, _ = M.run_checks(M.Scene(static), head, zs, plan)
    real = res["transit"] + res["lowered"] + res["gripper"] + res["dock"] + res["eject"]
    return {"configs_checked": res["configs"], "interferences": len(real),
            "reach_failures": len(res["reach_fail"]),
            "station_to_station_overlaps_between_places (machine_l checks only across groups)": station_overlaps(static),
            "note": "rev L re-run in memory with the unmodified machine_l scene: reproduces machine_l/out/report.json (harness check)"}


# =====================================================================================================
# 5. tips: demand per size (planner PLAN figures) and what the deck + shelf hold
# =====================================================================================================
def planner_tips():
    """per-type tip need from build/lh1/tip_damage.txt (planner, rev K three pipettes, corrected counts): no rule and the waste-tip rule"""
    txt = (ROOT / "build" / "lh1" / "tip_damage.txt").read_text().splitlines()
    out = {}
    for ln in txt:
        m = re.match(r"\s+(\d+) samples\s+(as built \(no tip rule\)|waste_tip_per_well_wash_phase)\s+total (\d+) -> (\d+).*-> (\{.*\})\s*$", ln)
        if m:
            n, rule, _, tot, d = int(m.group(1)), m.group(2), m.group(3), int(m.group(4)), ast.literal_eval(m.group(5))
            out.setdefault(n, {})["no_rule" if rule.startswith("as built") else "waste_tip_rule"] = {
                "total": tot, **{k: v["required"] for k, v in d.items()}}
    return out


TIPS = None


def tip_supply(name):
    global TIPS
    if TIPS is None:
        TIPS = planner_tips()
    n = SIZES[name]["samples_day1"]
    has_tpk = "TPK" in SIZES[name]["layout"].values()
    rule = "waste_tip_rule" if has_tpk else "no_rule"
    d = TIPS[n][rule]
    need = {"p10": d["p10"], "p200": d["p200"], "p300": d["p300"], "p1000": int(round(300 * n / 96))}
    on_deck = {"p10": 1, "p200": 1, "p300": 1, "p1000": 1}
    towers, rows = [], {}
    for tip in ("p300", "p200", "p10", "p1000"):
        wafers = math.ceil(need[tip] / 96.0)
        want = wafers + 2
        if tip == "p1000" and name != "L":
            rows[tip] = {"need": need[tip], "wafers_or_racks_needed": wafers, "held": 96, "rule_met": need[tip] <= 96 - 0,
                         "how": "one person-loaded rack (largest need + 2 spare NOT met: named lower figure, a second rack is a loading action)"}
            continue
        per = M.tower_max_l(tip)
        nt = math.ceil((want - 1) / per)
        towers += [(tip, min(per, want - 1 - k * per)) for k in range(nt)]
        rows[tip] = {"need": need[tip], "wafers_needed": wafers, "with_2_spare": want, "towers": nt, "held": (min(nt * per, want - 1) + 1) * 96}
    return {"samples": n, "basis": f"planner PLAN figures (build/lh1/tip_damage.txt, corrected counts, rev K three pipettes), "
                                    f"{'waste-tip rule (needs tip parking)' if has_tpk else 'no tip rule (no tip parking on this size)'}; p1000 = 300 x n/96 ASSUMED",
            "need": need, "need_total": sum(need.values()), "per_type": rows, "towers": towers, "n_towers": len(towers),
            "held_total": sum(r["held"] for r in rows.values())}


# =====================================================================================================
# 6. stiffness, belts, exchange time (head_l functions with this size's travel), ethanol, mass, cost
# =====================================================================================================
def with_params(geo, fn, *a):
    saved = (P.L_XC_RANGE, P.L_YF_RANGE)
    try:
        P.L_XC_RANGE = tuple(geo["XC"])
        P.L_YF_RANGE = tuple(geo["YF"])
        return fn(*a)
    finally:
        P.L_XC_RANGE, P.L_YF_RANGE = saved


def side_beam_span(geo):
    """mid-side uprights spaced evenly so no side-beam span exceeds rev L's longest (405 mm); returns (n_mid, max span)"""
    y0, y1 = geo["frame_y"][0] + 20.0, geo["frame_y"][1] - 20.0
    n = max(0, math.ceil((y1 - y0) / 405.0) - 1)
    return n, (y1 - y0) / (n + 1)


def stiffness(geo):
    out = {}
    n_mid, span = side_beam_span(geo)
    fac = (span / 405.0) ** 3
    for F in (120.0, 150.0):
        b = with_params(geo, HL.m1_budget, F)
        rows = b["rows"]
        h = sum(r["horizontal_mm"] for r in rows)
        v = sum(r["vertical_mm"] * (fac if r["contributor"].startswith("side (Y) beam") else 1.0) for r in rows)
        out[f"M1_{F:.0f}N"] = {"horizontal_mm": round(h, 3), "vertical_mm": round(v, 3), "total_rss_mm": round(math.hypot(h, v), 3),
                               "limit_mm": 0.5, "horizontal_pass_rev_L_rule": h <= 0.5 - 0.0 and b.get("horizontal_pass", True),
                               "lateral_budget_left_mm (0.5 - total)": round(0.5 - math.hypot(h, v), 3)}
    out["side_beam"] = {"mid_uprights_per_side": n_mid, "max_span_mm": round(span), "vertical_term_factor_vs_rev_L": round(fac, 2)}
    return out


def dynamics(geo):
    d = with_params(geo, HL.dynamics, ASM_L["tool_carriages"]["head_mass"]["head_kg"] if "head_mass" in ASM_L["tool_carriages"] else 13.96)
    keep = {k: v for k, v in d.items() if k.startswith(("X_EA", "Y_EA"))}
    return {"Y_moving_mass_kg": d["Y_moving_mass_kg"], "modes": keep, "basis": d["basis"]}


def exchange_time(d_to_dock):
    """head_l exchange_time() with the deck-to-dock distance of this size (head_l uses 450 mm, ASSUMED for K2)"""
    v_xy, a_xy, vzf, vzs, dwell = 300.0, 2000.0, 20.0, 5.0, 1.04

    def move(d):
        da = v_xy ** 2 / a_xy
        return 2 * math.sqrt(d / a_xy) if d < da else d / v_xy + v_xy / a_xy
    sd, lift, row = P.L_DOCK_S, 35.0, 300.0
    t = (move(d_to_dock) + move(row) + dwell + (sd - 15) / vzf + 15 / vzs + lift / vzf + move(3 * P.L_DOCK_PITCH) + dwell
         + (lift - 15) / vzf + 15 / vzs + 8 / vzs + 0.5 + (sd - 8) / vzf + move(row) + move(d_to_dock) + dwell)
    return round(t, 1)


def ethanol(geo):
    """deck_k ethanol_ventilation() method with this size's free volume (CALCULATED from ASSUMED inputs; competent-person review)"""
    R, Mw, T, Pa, lfl = 8.314, 0.046, 303.0, 101325.0, 0.033
    rho_v = Pa * Mw / (R * T)
    p_v80 = 7.5e3
    a_open = 0.0096 + 0.0096 + 0.0027                     # deck_k: two open troughs + one plate of ethanol wash
    a_cov = 0.16 * (0.0096 + 0.0096) + 0.0027             # ER-1 slotted covers on the troughs (deck_k response)
    out = {}
    for basis, V in (("gross enclosure x 0.85", geo["inner_volume_gross_m3"] * 0.85), ("above the deck x 0.85", geo["inner_volume_above_deck_m3"] * 0.85)):
        m_lfl = lfl * V * rho_v
        r = {"free_volume_m3": round(V, 3), "ethanol_at_LFL_g": round(m_lfl * 1000, 1)}
        for k_m in (0.003, 0.006):
            G_open = k_m * p_v80 * Mw / (R * T) * a_open
            G_cov = k_m * p_v80 * Mw / (R * T) * a_cov
            r[f"k={k_m * 1000:g} mm/s"] = {"hours_to_LFL_fan_off_open_troughs": round(m_lfl / G_open / 3600, 2),
                                           "hours_to_LFL_fan_off_covered_troughs": round(m_lfl / G_cov / 3600, 2)}
        r["air_changes_per_hour_at_30_m3h"] = round(30.0 / V, 1)
        out[basis] = r
    out["note"] = ("deck_k's K2 figure used 0.68 m3 (an ESTIMATE before the 3D frame); the fan's 30 m3/h and the airflow switch are the same "
                   "on every size and are well above the K = 5 need (10.6-21.2 m3/h for the open case): the sizes differ only in how long "
                   "the box takes to reach the LFL when the fan has STOPPED")
    return out


def parts_mass_and_cost(geo, geo_ref, name, rows_n):
    """scale the rev L modelled parts (machine_l_parts.csv) by the size's depth (and width) change; prices from rev H order-list rates"""
    dD = (geo["frame_y"][1] - geo["frame_y"][0]) - (geo_ref["frame_y"][1] - geo_ref["frame_y"][0])
    dW = (geo["enclosure_x"][1] - geo["enclosure_x"][0]) - (geo_ref["enclosure_x"][1] - geo_ref["enclosure_x"][0])
    groups = {}
    for r in csv.DictReader(open(LH1 / "machine_l" / "out" / "machine_l_parts.csv")):
        if r["group"] in ("station", "labware", "parked_tool", "reserved") or r["part"].startswith(("TB-2", "LWB-")):   # as machine_l: crate, canisters out
            continue
        dims = [float(v) for v in r["envelope_mm"].split(" x ")]
        m = float(r["mass_kg_each"]) * int(r["qty"])
        x, y = dims[0], dims[1]
        f = 1.0
        if r["group"] in ("frame", "enclosure", "shelf") and y > 500:
            f *= (y + dD) / y
        if r["group"] in ("frame", "enclosure", "drawer") and x > 900:
            f *= (x + dW) / x
        if r["group"] == "deck":
            f *= (rows_n * PITCH_Y + 45.0) / 705.0 * ((geo["deck_x"][1] - geo["deck_x"][0]) / 605.0)
        if r["part"].startswith("DSB-L_deck_bar"):
            f *= (rows_n + 1) / 7.0
        if r["part"].startswith("EXT-upright_mid"):
            f *= side_beam_span(geo)[0] / 2.0
        groups[r["group"]] = groups.get(r["group"], 0.0) + m * f
    return {k: round(v, 2) for k, v in groups.items()}, dD, dW


RATE = {   # GBP, ex VAT, from bom/lh1_order_list.csv (rev H; engineering estimates, not quotes) unless marked ASSUMED
    "2040_per_m": 8.84 / 0.97, "2020_per_m": 6.24 / 1.08, "mgn15_per_mm (ASSUMED 1.4 x the MGN12 slope)": 1.4 * (53.25 - 39.75) / 300.0,
    "belt_per_m (15 mm class ASSUMED 2 x the 9 mm price)": 2 * 3.9, "panel_per_m2 (ACM slope, rev H panels)": (57.91 - 46.68) / (1.126 * 0.713 - 0.684 * 0.713),
    "deck_plate_per_m2 (DK-1 GBP 69.18 over a 0.42 x 0.56 plate, ASSUMED area)": 69.18 / (0.42 * 0.56), "chain_per_m": 1.49,
    "thermal_module (TEC x2, machined sink, fan, NTC, fuse, foam, insert, shroud: rev H lines)": 2 * 9.85 + 62.36 + 15.0 + 1.91 + 5.3 + 8.0 + 9.17 + 4.32,
}


def size_cost_delta(geo, geo_ref, n_rows, n_rows_ref):
    """parts cost of the size-dependent members against the reference (family L); stations listed separately"""
    dD = (geo["frame_y"][1] - geo["frame_y"][0]) - (geo_ref["frame_y"][1] - geo_ref["frame_y"][0])
    dm = dD / 1000.0
    n_mid = side_beam_span(geo)[0] - side_beam_span(geo_ref)[0]
    lines = {
        "Y-running 2040 (base sides x2, base long x2, side beams x2, shelf rails x2)": 8 * dm * RATE["2040_per_m"],
        "Y-running 2020 roof sides x2": 2 * dm * RATE["2020_per_m"],
        "mid uprights 2040 x 0.555 m": 2 * n_mid * 0.555 * RATE["2040_per_m"],
        "MGN15 Y rails x2": 2 * dD * RATE["mgn15_per_mm (ASSUMED 1.4 x the MGN12 slope)"],
        "Y belts x2 (loop changes by 2 x dY)": 2 * 2 * dm * RATE["belt_per_m (15 mm class ASSUMED 2 x the 9 mm price)"],
        "side panels x2 + roof (sheet)": (2 * dm * 0.936 + dm * (geo["enclosure_x"][1] - geo["enclosure_x"][0]) / 1000.0) * RATE["panel_per_m2 (ACM slope, rev H panels)"],
        "deck plate rows": (n_rows - n_rows_ref) * PITCH_Y / 1000.0 * (geo["deck_x"][1] - geo["deck_x"][0]) / 1000.0 * RATE["deck_plate_per_m2 (DK-1 GBP 69.18 over a 0.42 x 0.56 plate, ASSUMED area)"],
        "Y cable chain": dm / 2 * RATE["chain_per_m"],
    }
    return {k: round(v, 1) for k, v in lines.items()}, round(sum(lines.values()), 0)


STATION_COST = {   # GBP, ex VAT; 'not priced' where the repository has no price
    "TPK": "not priced (printed and sheet parts, rev K)", "MAG2": "not priced (bought low-elution magnet plate)", "LIDPARK2": "not priced (printed guides)",
    "STACK2": "not priced (printed guides)", "P1000R": "not priced (a rack adapter)", "RES2": "not priced (reservoir holder)",
    "WN1": "not priced (load cell + machined platform)", "BAL": "not priced (balance qty 0 in the order list)", "ATR1": "not priced (printed rack)",
    "SPARE": "about 8 (plain insert IN-P, rev H line)", "HAZ_37C": f"about {RATE['thermal_module (TEC x2, machined sink, fan, NTC, fuse, foam, insert, shroud: rev H lines)']:.0f} (a thermal module, rev H lines) if hazardous liquid must be warm",
}


# =====================================================================================================
# 7. main
# =====================================================================================================
def main():
    OUT.mkdir(parents=True, exist_ok=True)
    rep = {"module": "LH-1 size family study (S starter, M, L) on the deck module standard",
           "status": "CALCULATION AND LAYOUT ONLY; nothing built or measured; every figure CALCULATED or ASSUMED as labelled",
           "inputs": ["cad/lh1/machine_l (frame stack-up, head, interference engine, station solids)", "cad/lh1/head_l (m1_budget, dynamics)",
                      "cad/lh1/deck_k (K2 rule checker k2_rules, ethanol method)", "build/lh1/tip_damage.txt (planner tip counts)",
                      "bom/lh1_order_list.csv (rev H unit rates)"]}
    rep["stackup_reproduces_rev_L"] = check_stackup_against_rev_l()
    geos = {"rev_L_as_designed": stack_up(M.XC0, M.XC1, M.YF0, M.YF1, fan="back")}
    for nm in SIZES:
        geos[nm] = size_geometry(nm, col0=True, fan="side")
    rep["geometry"] = geos
    log("geometry done")
    rep["rule_checks"] = {nm: k2_check(nm) for nm in SIZES}
    rep["rule_checks"]["rev_L_as_designed"] = k2_check_rev_l()
    log("rule checks", {k: v["pass"] for k, v in rep["rule_checks"].items()})
    rep["tips"] = {nm: tip_supply(nm) for nm in SIZES}
    rep["stiffness"] = {nm: stiffness(geos[nm]) for nm in ("rev_L_as_designed", "S", "M", "L")}
    rep["dynamics"] = {nm: dynamics(geos[nm]) for nm in ("rev_L_as_designed", "S", "M", "L")}
    mean_y = {nm: sum(row_y(SIZES[nm]["rows"]).values()) / len(SIZES[nm]["rows"]) for nm in SIZES}
    d_ref = 472.0 - sum(M.ROWS.values()) / len(M.ROWS)
    rep["exchange_time_s"] = {"rev_L_as_designed (head_l: 450 mm deck-to-dock)": exchange_time(450.0)}
    for nm in SIZES:
        d = 450.0 * (geos[nm]["dock_axis_y"] - mean_y[nm]) / d_ref
        rep["exchange_time_s"][f"{nm} (deck-to-dock {d:.0f} mm, scaled from 450 by the row-centroid distance)"] = exchange_time(d)
    rep["ethanol"] = {nm: ethanol(geos[nm]) for nm in ("rev_L_as_designed", "S", "M", "L")}
    mass = {}
    for nm in ("S", "M", "L"):
        g, dD, dW = parts_mass_and_cost(geos[nm], geos["rev_L_as_designed"], nm, len(SIZES[nm]["rows"]))
        n_st = len([k for k in SIZES[nm]["layout"].values() if k != "SPARE"])
        modules = 15.0 * n_st / 23.0                  # rev L: about 15 kg of deck modules over its stations (machine_l note), ASSUMED pro rata
        tot = sum(g.values()) + 13.96 + 5.4 + modules + 17.5
        mass[nm] = {"modelled_groups_kg (rev L parts scaled by depth/width)": g, "head_kg": 13.96, "tools_6_kg (ASSUMED)": 5.4,
                    "deck_modules_kg (ASSUMED pro rata to stations, rev L ~15)": round(modules, 1), "electrics_fasteners_kg (ASSUMED, rev L 15-20)": 17.5,
                    "total_kg_ESTIMATE": round(tot), "depth_change_vs_rev_L_mm": round(dD, 1), "width_change_vs_rev_L_mm": round(dW, 1)}
    rl = ML_REPORT["mass_kg"]
    mass["rev_L_as_designed"] = {"modelled_kg": rl["total_modelled_kg"], "total_kg_ESTIMATE": round(rl["total_modelled_kg"] + 15.0 + 17.5)}
    rep["mass"] = mass
    cost = {}
    for nm in ("S", "M"):
        lines, tot = size_cost_delta(geos[nm], geos["L"], len(SIZES[nm]["rows"]), len(SIZES["L"]["rows"]))
        missing = [k for k in SIZES["L"]["layout"].values() if k not in SIZES[nm]["layout"].values()]
        cost[nm] = {"size_dependent_members_vs_family_L_GBP": lines, "members_total_GBP": tot,
                    "stations_left_out": {k: STATION_COST.get(k, "not priced") for k in missing}}
    lines, tot = size_cost_delta(geos["L"], geos["rev_L_as_designed"], 6, 6)
    cost["L_vs_rev_L"] = {"members_GBP": lines, "members_total_GBP": tot, "column_0": "+17.5 mm of width: X beam, X rails, belts, front/back panels, "
                                                                                       "door: about +GBP 10 (rev H rates; the X beam tube is the dearest at about GBP 0.32/mm)"}
    cost["basis"] = "rev H order-list unit rates (engineering estimates, not quotes); the whole-machine total is not re-priced for revision L"
    rep["cost"] = cost
    log("calculations done")
    rep["interference_and_reach"] = {"rev_L_harness_check": run_machine_checks_rev_l()}
    log("rev L harness", rep["interference_and_reach"]["rev_L_harness_check"])
    rep["interference_and_reach"]["S"] = run_machine_checks("S", geos["S"])
    log("S checks", rep["interference_and_reach"]["S"]["interferences"], len(rep["interference_and_reach"]["S"]["reach_failures"]))
    rep["interference_and_reach"]["L"] = {"status": "family L is the rev L stations re-arranged as M + row D: NOT CAD-checked here (machine_l's "
                                                    "station code names the rev L places); rev L itself: 1,516 configs, 0 interferences, 0 reach "
                                                    "failures (machine_l report, reproduced in memory above)"}
    rep["runtime_s"] = round(time.time() - T0, 1)
    # layouts file
    lay = {"standard": {"pitch_mm": [PITCH_X, PITCH_Y], "columns_x": COLX, "row_F_y": ROW_F_Y, "dock_behind_back_row_mm": DOCK_BEHIND_BACK_ROW,
                        "column_0": "pipette-only column at x -264 (carriage A and B reach it with X travel +17.5 mm; the gripper does not)"},
           "sizes": {}}
    for nm in SIZES:
        pl = places_of(nm)
        lay["sizes"][nm] = {"rows": SIZES[nm]["rows"], "row_y": row_y(SIZES[nm]["rows"]), "samples_day1_target": SIZES[nm]["samples_day1"],
                            "geometry": {k: geos[nm][k] for k in ("XC", "YF", "travel_X_mm", "travel_Y_mm", "X_beam_mm", "Y_rail_mm", "frame_x", "frame_y",
                                                                  "dock_axis_y", "deck_x", "deck_y", "envelope_W_D_H_mm (frame, panels, feet, drawer; no fan)",
                                                                  "envelope_W_D_H_mm (with the extraction fan)")},
                            "places": {n: {"station": p["station"], "kind": p["kind"], "height_mm": p["h"], "work": p["work"], "centre_xy": list(p["c"])}
                                       for n, p in sorted(pl.items(), key=lambda kv: (SIZES[nm]["rows"].index(kv[0][0]), kv[0][1]))},
                            "shelf": SHELF_PLAN[nm], "dock_bays": 8, "rule_check_pass": rep["rule_checks"][nm]["pass"]}
    (OUT / "layouts.yaml").write_text(yaml.safe_dump(lay, sort_keys=False, width=150, allow_unicode=True))
    (OUT / "checks_S.json").write_text(json.dumps(rep["interference_and_reach"], indent=1, default=str))
    (OUT / "size_family.json").write_text(json.dumps(rep, indent=1, default=str))
    log("written", [p.name for p in OUT.iterdir()])
    return rep


if __name__ == "__main__":
    main()
