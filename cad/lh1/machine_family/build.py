"""LH-1 machine family (ADR-011): one size-parametric machine, S (starter, built first), M and L, growing in depth only.

Run from the project root:  .venv/bin/python cad/lh1/machine_family/build.py [S M L] [MVP] [--no-step]   (MVP: the stage-0 build variant, mvp.py)
Out:  cad/lh1/machine_family/out/<size>/report.json            full report of one size (also family_<size>.json, the planner block)
      cad/lh1/machine_family/out/<size>/parts_<size>.csv        parts list of the size (custom and bought, with quantities)
      cad/lh1/machine_family/out/<size>/machine_<size>_assembly.step   (S: renders-ready assembly; M/L with --step-all)
      cad/lh1/machine_family/out/kits.json, kit_S_to_M.csv, kit_M_to_L.csv    extension kits generated from the models
      cad/lh1/machine_family/out/S/proposed_build_parts_S.yaml, parts_S_vs_rev_L.json
      cad/lh1/machine_family/out/rev_L_station_check.json, cr_answers.json, family_summary.json
      cad/lh1/assembly/out/report.json keys family_S, family_M, family_L (same key layout as rev_l, plus "size"; rev_h/rev_k/rev_l untouched)

Why a new module and not machine_l: machine_l is the committed revision L record (the planner binds to its rev_l block and its
place names are written into its station, target and report code); the family needs stations placed by type at any grid position,
per-row deck strips, per-bay panels, a size parameter and the electrical owner's changes.  machine_l, head_l and deck_k are imported
read-only (their engine, part generators and head model are reused), so revision L still reproduces bit for bit
(machine_family/repro.py).  Family L replaces revision L as the large machine.

Status: CAD and calculation only.  Nothing built or measured.  Every figure CALCULATED (from the model, the machine_l/head_l/deck_k
methods, the size-family study) or ASSUMED (marked).
"""
from __future__ import annotations

import csv
import json
import math
import sys
import time
from pathlib import Path

import cadquery as cq
import yaml

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
import geometry as G  # noqa: E402
import scene as SC  # noqa: E402
import checks as CK  # noqa: E402
import deck_rules as DR  # noqa: E402
import mechanisms as MX  # noqa: E402
from geometry import AY, BASE, DKK, HL, M, P, SEAT, V, ZB  # noqa: E402

OUT = HERE / "out"
AOUT = G.LH1 / "assembly" / "out"
T0 = time.time()
SF = G._load("size_family_for_family", "size_family/build.py")     # study helpers (stiffness, dynamics, exchange, ethanol): read-only


def log(*a):
    print(f"[machine_family {time.time() - T0:6.1f}s]", *a, flush=True)


def save(path, obj):
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(obj, indent=1, default=str))


# =====================================================================================================
# per-size calculations
# =====================================================================================================
def stiffness(S):
    out = {}
    fac = (S.max_side_span / 405.0) ** 3
    for F in (120.0, 150.0):
        b = SF.with_params(S.geo_dict(), HL.m1_budget, F)
        rows = b["rows"]
        h = sum(r["horizontal_mm"] for r in rows)
        v = sum(r["vertical_mm"] * (fac if r["contributor"].startswith("side (Y) beam") else 1.0) for r in rows)
        out[f"M1_{F:.0f}N"] = {"horizontal_mm": round(h, 3), "vertical_mm": round(v, 3), "total_rss_mm": round(math.hypot(h, v), 3), "limit_mm": 0.5,
                               "horizontal_pass_line_0.4": h <= 0.4, "margin_to_0.5_mm": round(0.5 - math.hypot(h, v), 3)}
    out["side_beam"] = {"mid_uprights_y": S.mid_uprights, "max_span_mm": round(S.max_side_span, 1), "vertical_term_factor_vs_rev_L_405": round(fac, 3)}
    out["basis"] = ("head_l m1_budget (lever-corrected for beam mid z 545) with this size's travel; the side-beam vertical term scaled by "
                    "(longest side-beam span / 405)^3; the head changes (NEMA14 pusher, loops) do not enter the lateral path (ASSUMED)")
    return out


HEAD_EXTRA_KG = None


def head_extra_mass(head):
    """modelled family additions (Al) plus ASSUMED bought deltas: NEMA14 34 mm about 0.18 kg against NEMA11 34 mm about 0.11 kg"""
    global HEAD_EXTRA_KG
    if HEAD_EXTRA_KG is None:
        al = 0.0
        for ax in ("carriage_A", "carriage_B", "gripper"):
            for k, v in G.head_moving_additions(ax).items():
                if k in ("pusher_bracket", "ffc_arm"):
                    al += v.Volume() * 2.7e-6
        for k, v in G.head_fixed_additions().items():
            if k.startswith("ffc_guide") or k.startswith("x_chain"):
                al += v.Volume() * 2.7e-6
        HEAD_EXTRA_KG = {"aluminium_brackets_guides_kg": round(al, 3), "NEMA14_minus_NEMA11_x2_kg (ASSUMED)": 0.14,
                         "switches_hall_sensors_ffc_kg (ASSUMED)": 0.06, "total_kg": round(al + 0.20, 3)}
    return HEAD_EXTRA_KG


def dynamics(S, head):
    hr = json.loads((G.LH1 / "head_l" / "out" / "report.json").read_text())
    m_head = hr["mass"]["head_kg"] + head_extra_mass(head)["total_kg"]
    d = SF.with_params(S.geo_dict(), HL.dynamics, m_head)
    keep = {k: v for k, v in d.items() if k.startswith(("X_EA", "Y_EA"))}
    return {"head_kg": round(m_head, 2), "Y_moving_mass_kg": d["Y_moving_mass_kg"], "modes": keep, "basis": d["basis"],
            "belt_requirement": "X belt EA >= 6e4 N per run (EA 3e4 fails the 1.04 s dwell in every size); Y belt EA 3e4 passes in S"}


def exchange(S):
    mean_y = sum(S.ry.values()) / len(S.ry)
    d = 450.0 * (S.dock_y - mean_y) / 472.0
    return {"deck_to_dock_mm": round(d), "exchange_s": SF.exchange_time(d),
            "basis": "head_l exchange_time() steps with the deck-to-dock distance scaled from 450 mm by the row-centroid distance (study method)"}


def travel_report(S, pl, plan):
    ax_x = P.L_AXIS_X
    xs, ys = [], []
    for n, key, pts, s_t, tips, what, excl in CK.pipette_targets(S, pl):
        for x, y in pts:
            if n == "IG":
                xs.append(x - ax_x["carriage_B"])
            else:
                xs += [x - ax_x["carriage_A"], x - ax_x["carriage_B"]]
            ys.append(y - AY)
    g = CK.gripper_targets(S, pl, 0.0, plan)
    xs += [x - ax_x["gripper"] for (x, y), zf, w in g.values()]
    ys += [y - G.GRIP_Y for (x, y), zf, w in g.values()]
    xs += [x - ax_x[a] for x in S.DOCK_X for a in ("carriage_A", "carriage_B")]
    ys.append(S.dock_y - AY)
    nx, ny = (min(xs), max(xs)), (min(ys), max(ys))
    return {"X": {"travel_xc": [S.XC0, S.XC1], "travel_mm": round(S.XC1 - S.XC0, 1), "needed": [round(v, 1) for v in nx],
                  "end_margins_mm": [round(nx[0] - S.XC0, 1), round(S.XC1 - nx[1], 1)], "min_end_margin_mm": round(min(nx[0] - S.XC0, S.XC1 - nx[1]), 1),
                  "driving_targets": "left: single channels at column 0 (carriage B); right: gripper at the outer shelf column x 540, carriage A at bay 8",
                  "clearance_at_ends_mm": M.X_MARGIN},
            "Y": {"travel_yf": [S.YF0, S.YF1], "travel_mm": round(S.YF1 - S.YF0, 1), "needed": [round(v, 1) for v in ny],
                  "end_margins_mm": [round(ny[0] - S.YF0, 1), round(S.YF1 - ny[1], 1)], "min_end_margin_mm": round(min(ny[0] - S.YF0, S.YF1 - ny[1]), 1),
                  "driving_targets": "front: single channels at the front tubes of row F (y -303); back: the gripper at the shelf slot OK at the dock row",
                  "clearance_front_mm (head front to the door)": round((S.YF0 + M.HEAD_Y[0]) - (S.FY0 - 5), 1),
                  "front_target": "single channels at the front wells of the deep places in row F (TC-1 at F1, reservoir at F2): y -306.5",
                  "clearance_back_mm (Y carriage to the back cross member)": round((S.FY1 - 40) - (S.YF1 + G.YC["dy"][1]), 1),
                  "front_margin_note": ("15 mm front margin (rev L 29.5) beyond the true front target.  The study set 15 mm from the row F tube "
                                        "target (y -303) and so left only 11.5 mm beyond the deep wells of the TC-1 and reservoir now in row F (y "
                                        "-306.5); here the travel reaches 3.5 mm further forward with the frame unchanged (ADR-011 depth), so the head "
                                        "front stops 28.5 mm behind the door instead of 32.  Required 10 mm for overshoot and the soft limit (ASSUMED; "
                                        "the Y endstops are at the back), 15 = 1.5 x.  What limits it: the row F wells and the door plane; the Y blocks "
                                        "keep 1.5 mm of rail beyond them")},
            "Z": {"travel_mm": P.L_Z_TRAVEL, "eject_over_travel_mm": P.Z_EJECT_TRAVEL, "dock_seat_s": P.L_DOCK_S}}


def deck_positions(S, pl, by_place_tool):
    from deck_rules import STATIONS
    rows = S.rows
    order = sorted(pl, key=lambda q: (pl[q]["key"] in SC.NO_WINDOW, rows.index(q[0]), int(q[1])))   # added half places take the next tag numbers
    tags = {n: k + 1 for k, n in enumerate(order)}
    out = {}
    for n, p in pl.items():
        r, c = n[0], int(n[1])
        i = rows.index(r)
        lim, why = 150.0, "MAX_LABWARE_H"
        if i > 0:
            fk = pl.get(f"{rows[i - 1]}{c}", {}).get("key")
            fw = STATIONS[fk][2] if fk else None
            if fw:
                l1 = 51.0 if fw.startswith("deep") else 72.0
                if l1 < lim:
                    lim, why = l1, f"R1 behind {rows[i - 1]}{c}"
        for cc in (c - 1, c + 1):
            nb = pl.get(f"{r}{cc}")
            if nb and nb["work"] == "deep" and 64.0 < lim:
                lim, why = 64.0, f"R2 beside {r}{cc}"
            if nb and nb["key"] in ("P10", "P200", "P300") and 57.0 < lim:
                lim, why = 57.0, f"N1 beside the frame {r}{cc}"
        if i + 1 < len(rows):
            bk = pl.get(f"{rows[i + 1]}{c}", {}).get("key")
            if bk and STATIONS[bk][2] in ("deep", "deep_single_only") and 100.0 < lim:
                lim, why = 100.0, f"N2 in front of {rows[i + 1]}{c}"
        bt = by_place_tool.get(n, {})
        reach = sorted({k.split("@")[0] for k, v in bt.items() if v["reached"] and v["interferences"] == 0})
        out[n] = {"station": p["key"], "kind": p["kind"], "centre_xy": list(p["c"]), "work": p["work"], "height_mm": p["h"],
                  "height_limit_mm (R1/R2/N1/N2)": lim, "height_limit_rule": why, "within_limit": p["h"] <= lim + 1e-6,
                  "reaching_tools (zero interference)": reach, "tools_with_interference": sorted({k for k, v in bt.items() if v["interferences"]}),
                  "reach_limits": {k: v["reach_limit"] for k, v in bt.items() if v.get("reach_limit")},
                  "weighable": {"WN1": "WN-1 (labware)", "BAL": "balance (reagents by weight)"}.get(p["key"], False), "deck_tag": tags[n],
                  "connector": ("fitted: " + SC.POWERED[p["key"]]) if p["key"] in SC.POWERED else (SC.OWN_LEADS.get(p["key"]) or (
                      "none (half insert on the deck top, no window)" if p["key"] in SC.NO_WINDOW else "cable route (notch) only")),
                  "deck_strip": f"row {r}", "back_section": r == rows[-1]}
    if S.shelf_plan.get("IG", "").startswith("p1000"):
        bt = by_place_tool.get("IG", {})
        out["IG"] = {"station": "P1000R", "kind": "p1000 rack (person-loaded) on the shelf, sunk 15 mm", "centre_xy": [S.IG_RACK_X, S.SLOTS["IG"][1]], "work": "shallow",
                     "height_mm": 72.0 - 15.0 + S.SHELF["t"] - SEAT, "reaching_tools (zero interference)": sorted({k.split("@")[0] + "@" + k.split("@")[1]
                                                                                                                for k, v in bt.items() if v["reached"] and not v["interferences"]}),
                     "note": "carriage B only (carriage A cannot reach x 410): planner rule 'p1000 tips on carriage B'", "deck_tag": None,
                     "connector": "none"}
    out["_rules"] = DR.RULES
    return out


def tube_positions(pl):
    rep = HL.reach_into_tubes()
    racks = {"ATR1": ["15mL (CTB-2 F4, ATR-1 F1; recessed 60)", "50mL (CTB-2 F4, ATR-1 F1; recessed 60)"], "CTB1": ["1.5mL (CTB-1 F3, ATR-2 A2)"],
             "CTB2": ["5mL (CTB-2 F4, ATR-2 A2)", "15mL (CTB-2 F4, ATR-1 F1; recessed 60)", "50mL (CTB-2 F4, ATR-1 F1; recessed 60)"],
             "ATR2": ["1.5mL (CTB-1 F3, ATR-2 A2)", "5mL (CTB-2 F4, ATR-2 A2)"]}
    out = {}
    for n, p in pl.items():
        if p["key"] not in racks:
            continue
        out[n] = {"rack": p["kind"], "station": p["key"], "per_tool": {}}
        for k, t in rep["tools"].items():
            if HL.TOOLS[k]["eight"]:
                continue
            out[n]["per_tool"][k] = {r.split(" (")[0]: t["racks"][r] for r in racks[p["key"]] if r in t["racks"]}
    for n, p in pl.items():
        if p["key"] == "ATR3":
            zb = SEAT + 48.0 - 39.0
            out[n] = {"rack": p["kind"], "station": "ATR3", "per_tool": {k: {"1.5mL": {"tip_end_lowest_z": t["tip_end_lowest_z"], "tube_bottom_z": zb,
                      "bottom_reached": t["tip_end_lowest_z"] <= zb + 0.5}} for k, t in rep["tools"].items() if not HL.TOOLS[k]["eight"]},
                      "note": "tube bottom = rack top 48 - 39 mm tube (ASSUMED class); body entry into the tube is checked in the CAD run (reach_limits)"}
    out["_basis"] = rep["basis"] + " (rack depths as revision L: CTB-2/ATR-1 recessed 60, ATR-2 recessed 20)"
    return out


def pipettes_report(by_place_tool):
    hr = json.loads((G.LH1 / "head_l" / "out" / "report.json").read_text())
    rng = {"M10": (0.5, 10), "M50": (5, 50), "M300": (30, 300), "S10": (0.5, 10), "S100": (10, 100), "S1000": (100, 1000)}
    out = {}
    for k, t in HL.TOOLS.items():
        places = sorted({n for n, d in by_place_tool.items() for kk, v in d.items() if kk.startswith(k + "@") and v["reached"] and v["interferences"] == 0})
        out[k] = {"name": t["name"], "range_uL": rng.get(k), "channels": 8 if t["eight"] else 1, "tip": t["tip"], "Hn_mm": t["Hn"],
                  "nozzle_z_at_top": round(M.nozzle_top(k), 1), "places_reached_clean": places, "mass_kg": hr["tool_masses"][k]["total_kg"],
                  "dock_bay": (M.DOCK_TOOLS.index(k) + 1) if k in M.DOCK_TOOLS else "7 (reserved)" if k == "S100" else None}
    return out


def tips_and_waste(S, plan):
    t = plan["tips"]
    crate_l = SC.CRATE["L"] * SC.CRATE["W"] * SC.CRATE["H"] / 1e6
    vt = {"p300": 2.0, "p200": 1.7, "p10": 0.6, "p1000": 4.5}
    need = t["need"]
    mean = sum(vt[k] * v for k, v in need.items()) / max(1, sum(need.values()))
    holds = int(crate_l * 1000 * 0.7 / mean)
    return {"tips": t, "shelf_towers": {"needed": plan["towers_needed"], "placed": plan["towers_placed"], "fits": plan["fits"]},
            "tip_waste": {"crate_L_gross": round(crate_l, 1), "holds_tips (0.7 fill, run mix)": holds, "run_tips": t["need_total"],
                          "ratio": round(holds / t["need_total"], 2), "rule": "1.5 x", "meets": holds >= 1.5 * t["need_total"]}}


def headroom(S, zc, trav, stiff, plan, dyn, tp=None):
    rows = []

    def add(kind, cap, req, prov, unit, note=""):
        if req is None or prov is None:
            ok, rule = None, "1.5 x (electrical owner)"
        elif kind == "consumable":
            ok, rule = prov >= req + 2, "largest need + 2 spare"
        else:
            ok, rule = prov >= 1.5 * req, "1.5 x"
        rows.append({"kind": kind, "capacity": cap, "required_largest_known": req, "provided": prov, "unit": unit,
                     "ratio": round(prov / req, 2) if req and prov else None, "rule": rule, "meets_rule": ok, "note": note})
    tw = tips_and_waste(S, plan)
    tt = tw["tips"]
    for tip, r in tt["per_type"].items():
        if "need_x1.5" in r:
            n_req = r.get("wafers_needed", r.get("wafers_or_racks_needed"))
            prov = r["held"] // 96
            rows.append({"kind": "consumable (V15: 32 reactions x 1.5)", "capacity": f"{tip} tips held (frames + towers / rack)", "required_largest_known": r["need_x1.5"],
                         "provided": r["held"], "unit": "tips", "ratio": round(r["held"] / max(1, r["need"]), 2), "rule": "held >= 1.5 x planner need at 32",
                         "meets_rule": r["held"] >= r["need_x1.5"], "note": f"need {r['need']} (planner), {n_req} wafers/racks, {prov} held"})
        elif "wafers_needed" in r:
            add("consumable", f"{tip} refill wafers held (deck frame + towers)", r["wafers_needed"], r["held"] // 96, "wafers", f"need {r['need']} tips (planner PLAN figures)")
        else:
            add("consumable", f"{tip} tips (one person-loaded rack)", math.ceil(r["need"] / 96), 1, "racks", r["how"])
    add("fixed", "tip waste crate (tips)", tt["need_total"], tw["tip_waste"]["holds_tips (0.7 fill, run mix)"], "tips", "29 L crate, one run between emptyings")
    add("consumable", "pipette dock bays", 6, 8, "bays", "five pipettes + syringe tool, S100 in bay 7, bay 8 cable bay")
    nh = sum(1 for c in plan["slots"].values() if c["kind"] == "handoff")
    add("consumable", "edge hand-off positions (open)", 2, nh, "places", "LOWER: S opens NO hand-off (H1 and H2 cut and blanked; their "
        "slots OF and OA hold the spent-wafer stack and a p300 tower, V15); the S->M kit opens both" if S.name == "S" else "ADR-010 minimum")
    add("consumable", "shelf tower slots", plan["towers_needed"], plan["towers_placed"], "slots", "LOWER than +2 where the slots are exactly used (named figure)")
    add("fixed", "Z clearance over the tallest deck item", 33.0, zc["deck_clearance_mm"], "mm", f"{zc['tallest_deck_item']}")
    add("fixed", "Z clearance over the shelf towers", 33.0, zc["shelf_clearance_mm"], "mm", "LOWER (as rev L): towers at the 150 mm cap, gripper only")
    add("fixed", "Z clearance over the dock (no tips)", 8.0, zc["dock_clearance_mm"], "mm", "pass 2 dock rule")
    add("fixed", "X travel end margin", 10.0, trav["X"]["min_end_margin_mm"], "mm", "10 mm ASSUMED (overshoot + switch)")
    add("fixed", "Y travel end margin", 10.0, trav["Y"]["min_end_margin_mm"], "mm", "front 15 (was 29.5 in rev L), back 16")
    h150 = stiff["M1_150N"]["horizontal_mm"]
    add("fixed", "lateral load at which M1 horizontal reaches 0.4 mm", 120.0, round(150.0 * 0.4 / h150), "N", f"horizontal {h150} mm at 150 N")
    add("fixed", "pusher thrust (LGA35 at 0.7 A, 5 mm/s; sourcing curve INFERENCE)", 80.0, HL.LT.LGA35["force_N"]["0.7A_5mm_s"], "N",
        "against the largest plunger force 80 N (SYR); at the firmware run current, not rated current (V23)")
    add("fixed", "extraction air (fan running)", round(30.0 / 2.83, 2), 30.0, "m3/h", "need about 10.6 m3/h (K = 5, open troughs); fan 30 m3/h (deck_k method)")
    add("fixed", "24 V and TEC supplies", None, None, "W", "electrical owner: sized for L in every size")
    if tp and tp.get("present"):
        add("fixed", "tip parking (p300 tips parked at once)", 32, tp["capacity_tips"], "tips",
            f"{tp['place']} {tp['station']}: one tip per sample at a time, ADR-013 upper case 32 reactions")
    else:
        add("fixed", "tip parking (p300 tips parked at once)", 32, 0, "tips", "no tip parking place")
    af = airflow()
    add("fixed", "control-compartment airflow", af["needed_CFM"], af["provided_CFM"], "CFM", "136 W at a 10 K rise; 2 x 92 mm fans derated 0.6 (ASSUMED)")
    return {"rule": "ADR-009 as adjusted by ADR-010: fixed capacities >= 1.5 x; consumable places >= largest need + 2 spare; lower figures named",
            "table": rows, "fails_or_lower": [r["capacity"] for r in rows if r["meets_rule"] is False]}


# =====================================================================================================
# tip parking, keep-outs, services interface, syringe (published for the planner and the wiring owner)
# =====================================================================================================
def _bb(sol):
    b = sol.BoundingBox()
    return [round(b.xmin, 1), round(b.xmax, 1), round(b.ymin, 1), round(b.ymax, 1), round(b.zmin, 1), round(b.zmax, 1)]


def _union(bbs):
    return [min(b[0] for b in bbs), max(b[1] for b in bbs), min(b[2] for b in bbs), max(b[3] for b in bbs), min(b[4] for b in bbs), max(b[5] for b in bbs)]


TIP_PARK_NEED = {"reactions_max (ADR-013 upper case, first build)": 32, "parked_per_reaction_at_once": 1, "headroom": 1.5,
                 "basis": "planner build/lh1/small_run_waste.txt: the wash series that parking serves (the wash series: supernatant removal and washes) each re-use ONE removal tip per "
                          "sample, parked between removals; the series run one after another on one plate, so at most one tip per sample is "
                          "parked at a time"}


def dock_clearance(head, items):
    """machine_l's pass 2 rule with the dock lift pads (bar DOWN) in place of the old release posts: the head at s 0 (no tips) clears
    the parked tools and the pads by >= 8 mm while it moves along the dock row"""
    top_tool = max(it["solid"].BoundingBox().zmax for it in items if it["group"] == "parked_tool")
    top_pad = max(it["solid"].BoundingBox().zmax for it in items if "lift_pad" in it["name"] or "lift_bar" in it["name"])
    hp = head.parts(0.0, 0.0, toolA="M300", toolB="S1000", tipsA=False, tipsB=False)
    low_noz, _ = M.lowest_point(head, [h for h in hp if h[0].startswith(("carriage_A:tool", "carriage_B:tool"))])
    low_g, _ = M.lowest_point(head, hp, "gripper")
    return {"pass 2 (this model)": {"parked_tool_top_z": round(top_tool, 1), "lift_pad_top_z (bar DOWN)": round(top_pad, 1),
                                    "head_nozzles_at_s0_z (no tips)": round(low_noz, 1), "gripper_fingertips_at_s0_z": round(low_g, 1),
                                    "clearance_mm": round(min(low_noz, low_g) - max(top_tool, top_pad), 1)},
            "rule": "the idle carriage (tool up, no tips) and the gripper pass over the neighbouring bays: >= 8 mm over parked tools and pads"}


def dock_lift_block(S, items):
    """ADR-014 dock lift DLB-1 as built into this size's dock: where its parts are (machine frame, deck-top z 0), the end the motor is at,
    the bar-open switch, the in-row stroke and its clearance (head report inrow_clearance)"""
    hr = json.loads((G.LH1 / "head_l" / "out" / "report.json").read_text())
    dl = hr["dock_bay"]["dock_lift"]
    parts = {it["name"][5:]: _bb(it["solid"]) for it in items if it["name"].startswith("DOCK_lift")}
    act = parts.get("lift_actuator")
    ic = hr["inrow_clearance"]
    return {"motor_end": "LEFT (left of bay 1, x %.0f..%.0f); the head enters and leaves the dock row at its right end" % (act[0], act[1]) if act else None,
            "parts_xyz (bounding boxes x0 x1 y0 y1 z0 z1)": parts,
            "motor_xy_and_bottom_z": [round((act[0] + act[1]) / 2, 1), round((act[2] + act[3]) / 2, 1), round(act[4], 1)] if act else None,
            "open_height_mm": dl["open_height_mm"], "stroke_mm": dl["stroke_mm"], "run_current_A": dl["run_current_A"], "speed_mm_s": dl["speed_mm_s"],
            "switches": dl["switches"], "thrust_basis": dl["thrust_basis"],
            "in_row_stroke_s": ic["in_row_stroke_s"], "in_row_clearance_mm": ic["min_clearance_over_mm"], "in_row_where": ic["where"],
            "in_row_clearance_at_307_mm": ic["previous_stroke_307"]["min_clearance_over_mm"], "in_row_rule_mm": ic["rule_mm"], "in_row_pass": ic["pass"],
            "in_row_side_gap_mm": ic["min_side_gap_mm (no Y motion during the move)"]}


def shelf_slots(S, plan):
    """V3 / integration 23 Nov 2026: every shelf slot's centre (machine frame), what stands there and the gripper's stroke to it, so the
    firmware and the planner stop deriving slot XY from the slot names"""
    g = CK.gripper_targets(S, {}, 0.0, plan)
    out = {}
    for slot, (x, y) in S.SLOTS.items():
        c = plan["slots"].get(slot)
        key = (c or {}).get("name") if (c or {}).get("kind") == "handoff" else (f"{slot}_lidpark" if (c or {}).get("kind") == "lidpark" else slot)
        tgt = g.get(key)
        out[slot] = {"centre_xy": [round(x, 1), round(y, 1)], "kind": (c or {}).get("kind", "empty"), "what": (c or {}).get("what", (c or {}).get("name")),
                     "height_mm": (c or {}).get("height_mm", 0.0), "handoff": (c or {}).get("name") if (c or {}).get("kind") == "handoff" else None,
                     "gripper_s": round(M.grip_s(tgt[1]), 1) if tgt else None, "printer_xy_gripper": [round(x - P.L_AXIS_X["gripper"], 1), round(y - G.GRIP_Y, 1)]}
    if S.shelf_plan.get("IG", "").startswith("p1000"):
        out["IG"]["p1000_rack_centre_xy"] = [S.IG_RACK_X, S.SLOTS["IG"][1]]
    out["_note"] = "machine frame (deck-top z 0); printer_xy_gripper = slot centre - gripper axis x / - gripper finger dy (the gripper's printer XY)"
    return out


def tip_parking(S, pl, items, head, bpt, rep=None):
    from geometry import TPKS
    n = [q for q, p in pl.items() if p["key"] in ("TPKS", "TPK")]
    if not n:
        return {"present": False}
    n = n[0]
    key = pl[n]["key"]
    need = TIP_PARK_NEED["reactions_max (ADR-013 upper case, first build)"] * TIP_PARK_NEED["parked_per_reaction_at_once"]
    req = math.ceil(need * TIP_PARK_NEED["headroom"])
    reach = {k: {"reached": v["reached"], "interferences": v["interferences"], "s_target": v["s_target"]} for k, v in bpt.get(n, {}).items()}
    if key == "TPK":
        return {"present": True, "place": n, "station": "TPK", "part": "TPK-1 (deck_k, 13 eight-channel sets, p300; LIFO, not changed here)",
                "capacity_tips": 104, "capacity_sets": 13, "required_tips (32 x 1 x 1.5)": req, "meets_1.5x": 104 >= req, "reach": reach,
                "need_basis": TIP_PARK_NEED, "random_access": False}
    cx, cy = pl[n]["c"]
    _, _, geo = SC.tpks_parts()
    spots = [round(cx + v, 1) for v in TPKS["spots_local"]]
    entries = [round(cx + v, 1) for v in TPKS["entries_local"]]
    cap_sets = len(spots)
    vc2 = [it for it in items if it.get("place") == "B1" and it["name"].split("_", 1)[1].startswith("vc2")]
    vb = [_bb(it["solid"]) for it in vc2]
    clear = {}
    for ax in ("carriage_A", "carriage_B"):
        s_t = bpt[n][f"M300@{ax}"]["s_target"]
        xc, yf = spots[-1] - P.L_AXIS_X[ax], cy - AY
        kw = dict(sA=s_t, sB=0.0, toolA="M300", toolB="M300", tipsA=True) if ax == "carriage_A" else dict(sA=0.0, sB=s_t, toolA="M300", toolB="M300", tipsB=True)
        best = (1e9, None)
        for hn, v, off in head.parts(xc, yf, **kw):
            if not hn.startswith(ax + ":tool:"):
                continue
            b = _bb(v.translate(V(*off)))
            for q in vb:
                gaps = [max(q[2 * i] - b[2 * i + 1], b[2 * i] - q[2 * i + 1], 0.0) for i in range(3)]
                d = math.sqrt(sum(g * g for g in gaps))
                if d < best[0]:
                    best = (round(d, 1), hn.split(":")[-1])
        clear[f"M300@{ax}"] = {"min_clearance_to_VC2_mm": best[0], "part": best[1]}
    comb = [it for it in items if it.get("place") == n and it["name"].startswith(n + "_TPK-S")]
    cb = _union([_bb(it["solid"]) for it in comb if "parked" not in it["name"]])
    zl1 = TPKS["zl1"]
    tl = P.TIP_BELOW_NOZZLE["p300"]
    s_seat = M.s_for_tip_end("M300", zl1 - TPKS["pocket_depth"] - tl)
    tol = None
    if rep is not None:
        h120 = rep["stiffness"]["M1_120N"]["horizontal_mm"]
        ys = rep["dynamics"]["modes"].get("Y_EA3e+04", {}).get("stretch_mm", 0.289)
        tol = CK.comb_tolerance(h120, ys)
    groups = [{"group": g + 1, "entry_x": entries[g], "spots_x": [spots[2 * g], spots[2 * g + 1]]} for g in range(TPKS["groups"])]
    return {"present": True, "place": n, "station": "TPKS", "centre_xy": [cx, cy], "random_access": True,
            "part": "TPK-S random access: IN-H half insert 92 x 94 x 5, lower plate 8 mm Al (CNC: slot %.1f per row per group, %.1f x %.1f deep "
                    "pocket at each spot, 0.3 chamfer), upper plate 6 mm Al (CNC: slot %.1f per row per group with 0.5 chamfers, D%.1f entry "
                    "hole at each group entry with a 1.0 chamfer, D%.1f x %.1f counterbore on the underside over each spot), 4 PA12 posts with M3 "
                    "rods, PA12 drip tray; plates cut to the M300 pipette's MEASURED nozzle row (scan its eight nozzle prints at 1200 dpi)" %
                    (TPKS["ws"], TPKS["pocket_d"], TPKS["pocket_depth"], TPKS["wn"], TPKS["entry_d"], TPKS["pocket_d"], TPKS["ucb_depth"]),
            "tip_type": "p300", "tools_admitted": ["M300"], "capacity_per_tool": {"M300": cap_sets, "M50": 0, "others": 0},
            "capacity_note": "only the M300 parks (removal tips of repeated wash series); M50/p200 is no longer "
                             "admitted (V29): firmware tool list and planner rule text = ['M300']",
            "groups": groups, "rows_y": [round(cy + y, 1) for y in geo["rows"]],
            "capacity_sets": cap_sets, "capacity_tips": 8 * cap_sets, "required_tips": req, "meets_1.5x": 8 * cap_sets >= req,
            "need_basis": TIP_PARK_NEED, "comb_bbox": cb, "height_above_seat_mm": TPKS["height"],
            "lower_plate_top_z": zl1, "collar_seat_z_in_pocket": zl1 - TPKS["pocket_depth"], "parked_tip_end_z": round(geo["tip_end_z"], 1),
            "detent": "each spot is a %.1f mm pocket: a parked set cannot slide toward the entry (X) or across (Y) without being lifted 1.0 mm" % TPKS["pocket_depth"],
            "motion (firmware and planner)": {
                "park at spot k (group g)": [
                    "1 X/Y to entry_x of group g, rows_y; Z to s_above = %.1f (tip ends 5 mm above the upper plate)" % (M.s_for_tip_end("M300", geo["zu1"] + 5.0)),
                    "2 Z down at <= 10 mm/s to s_slide = %.1f (collars 0.3 mm above the lower plate)" % (s_seat - TPKS["pocket_depth"] - 0.3),
                    "3 X to the spot (+/-%.1f mm) at <= 10 mm/s, acceleration <= 0.25 m/s2" % TPKS["p"],
                    "4 Z down 1.3 mm to s_seat = %.1f (collars on the pocket floor)" % s_seat,
                    "5 strip: Z up at the force current (1.3 A) at <= 5 mm/s: collars meet the upper plate after 1.6 mm, tips stay; on to s_above",
                    "6 record (tool M300, series, column) as the owner of spot k; tip check EXPECT=0"],
                "re-pick spot k": [
                    "1 X/Y to the spot, rows_y; Z to s_above",
                    "2 force-ended pick-up down to the parked collars (as at a rack, 120 N switch), at most to s_seat + %g = %.1f (checked in the "
                    "model with bare nozzles); refuse if spot k's owner is not this tool and series" % (CK.TPKS_REPICK_OVER, s_seat + CK.TPKS_REPICK_OVER),
                    "3 Z up 1.3 mm (collars out of the pocket), X to the group's entry at <= 10 mm/s and <= 0.25 m/s2",
                    "4 Z up through the entry holes to s_above; tip check EXPECT=8"],
                "any order": "every spot is reached from its own group's entry, which no parked set ever occupies: random access"},
            "s_values": {"s_seat": round(s_seat, 1), "s_slide": round(s_seat - TPKS["pocket_depth"] - 0.3, 1),
                         "s_repick_max": round(s_seat + CK.TPKS_REPICK_OVER, 1)},
            "tolerance_stack": tol,
            "reach (zero interference, both carriages, 50/85/100 % of the stroke at both end spots and the entries)": reach,
            "clearance_to_VC2_at_the_deepest_spot": clear,
            "what_moved": "B0's half place could hold only 4 random-access spots in its 51.6 mm; the X travel's left end, the left Y rail, the "
                          "frame, the X beam and the deck's left edge moved 10 mm left (every size; machine 10 mm wider) so the three groups "
                          "fit (65.6 mm); the deepest spot moved 4 mm right (M300 >= 5 mm from VC-2).  Integration 23 Nov 2026: the same TPK-S "
                          "at B0 in M and L (TPK-1 LIFO at C0 withdrawn, C0 empty): one parking design and one firmware path in every size",
            "footprint": "inside B0 on a 92 x 94 half insert (deck 10 mm wider on the left)"}


def keepouts(S, pl, items):
    """the wiring owner's keep-outs: thermocycler with its lid park, tip-check gate, magnet module (boxes x0 x1 y0 y1 z0 z1, mm, machine frame)"""
    win = SC.windows(pl)
    out = {}
    spec = (("thermocycler_TC-1", "TC1", None), ("TC-1_lid_park", "TC1LID", None), ("tip_check_gate_tip_waste", "TIPWASTE", None),
            ("magnet_module_MB-1", "MB1", None))
    for nm, key, _ in spec:
        ns = SC.find(pl, key)
        if not ns:
            continue
        n = ns[0]
        parts = {it["name"][len(n) + 1:]: _bb(it["solid"]) for it in items if it.get("place") == n}
        if key == "TIPWASTE":
            parts.update({it["name"]: _bb(it["solid"]) for it in items if it["name"].startswith(("TW-L_tip_chute", "TWB-F", f"CON-F_{n}"))})
        w = [k for k in win if n in k.split("+")]
        out[nm] = {"place": n, "station": key, "union": _union(list(parts.values())), "parts": parts,
                   "deck_window": [round(v, 1) for v in win[w[0]]] if w else None}
    tc = SC.find(pl, "TC1")
    lp = SC.find(pl, "TC1LID")
    if tc and lp:
        a = pl[tc[0]]["c"]
        b = pl[lp[0]]["c"]
        out["TC-1_lid_travel_corridor (ASSUMED: lid moved on the guide strip from closed on F1 to parked on A1)"] = {
            "box": [a[0] - 64.0, a[0] + 64.0, min(a[1], b[1]) - 47.0, max(a[1], b[1]) + 47.0, SEAT, SEAT + 60.0]}
    tw = SC.find(pl, "TIPWASTE")
    if tw:
        cx, cy = pl[tw[0]]["c"]
        tg = SC.TIP_GATE
        out["tip_check_gate_beam"] = {"place": tw[0], "axis": "X", "beam_y": round(cy + tg["y_local"], 1), "beam_z": tg["z"],
                                      "x_faces": [round(cx - tg["face_x"], 1), round(cx + tg["face_x_rx"], 1)], "gate_x": round(cx + tg["gate_x_local"], 1),
                                      "union": _union([_bb(it["solid"]) for it in items if it.get("place") == tw[0] and "TGB-F" in it["name"]]),
                                      "parts": {it["name"][len(tw[0]) + 1:]: _bb(it["solid"]) for it in items if it.get("place") == tw[0] and "TGB-F" in it["name"]},
                                      "why_z": tg["why_z"], "gate_s_rule": "stroke with the tip ends at beam_z: s = nozzle_top(tool) - tip length - beam_z"}
    out["_note"] = "same in every size (rows F, A, B never move); z = 0 is the deck top, the labware seat is z 5"
    return out


def airflow():
    a = SC.CBX_AIR
    q_m3s = a["heat_W"] / (1.2 * 1005.0 * a["rise_K"])
    cfm = q_m3s / 0.000471947
    prov = 2 * a["fan_free_air_cfm_each"] * a["system_derate"]
    return {"heat_W": a["heat_W"], "rise_K": a["rise_K"], "needed_m3_h": round(q_m3s * 3600, 1), "needed_CFM": round(cfm, 1),
            "needed_1.5x_CFM": round(1.5 * cfm, 1), "fans": "2 x 92 mm, about %d CFM free air each (ASSUMED class, UNVERIFIED), with guard and "
                                                            "washable filter" % a["fan_free_air_cfm_each"],
            "derate_for_filter_grille_and_compartment (ASSUMED)": a["system_derate"], "provided_CFM": round(prov, 1),
            "ratio": round(prov / cfm, 2), "rise_K_at_provided": round(a["heat_W"] / (1.2 * 1005.0 * prov * 0.000471947), 1),
            "one_fan_failed_CFM": round(prov / 2, 1), "one_fan_failed_rise_K": round(a["heat_W"] / (1.2 * 1005.0 * prov / 2 * 0.000471947), 1),
            "grille_open_area_cm2 (50 % open)": round((a["grille_y"][1] - a["grille_y"][0]) * (a["grille_z"][1] - a["grille_z"][0]) * 0.5 / 100.0),
            "grille_face_velocity_m_s": round(prov * 0.000471947 / ((a["grille_y"][1] - a["grille_y"][0]) * (a["grille_z"][1] - a["grille_z"][0]) * 0.5e-6), 2),
            "basis": "Q = P / (rho cp dT), rho 1.2 kg/m3, cp 1005 J/kgK; the electrical owner's 136 W"}


def services_interface(S, items):
    byname = {it["name"]: it for it in items}

    def bb(prefix):
        return {k: _bb(v["solid"]) for k, v in byname.items() if k.startswith(prefix)}
    c = SC.COMPARTMENT
    return {
        "control_compartment": {"box": [c["x0"], c["x1"], c["y0"], c["y1"], c["z0"], c["z1"]], "wall_t": c["t"],
                                "device_volume": _bb(byname["CBX-F_device_volume_127 (reserved)"]["solid"]),
                                "note": "fixed in every size (never moves on an extension)"},
        "airflow": dict(airflow(), intake="RIGHT wall (x 495): two 92 mm fans blowing in, centres y %s, z %.1f; 88 mm holes" %
                        (list(SC.CBX_AIR["fans_y"]), (c["z0"] + c["z1"]) / 2), exhaust="LEFT wall (x -495): louvred grille, opening y %s, z %s" %
                        (list(SC.CBX_AIR["grille_y"]), list(SC.CBX_AIR["grille_z"])),
                        why_these_walls="the right wall has 125 mm to the right feet and the Y-chain conduit; the left wall has only 8 mm to the "
                                        "left feet, enough for a grille but not a fan; cross-flow sweeps both plates (CB-L left, CBP2 right)",
                        parts=bb("CBX-F_intake_fan") | bb("CBX-F_exhaust_grille")),
        "back_wall_cut_outs (y 345, the compartment's back wall; reached from behind, under the base)": {
            nm: {"centre_xz": [v[0], v[1]], "cut_out_w_h_mm": [v[2], v[3]], "depth_inside_mm": v[4], "flange_w_h_mm": [v[5], v[6]],
                 "envelope": _bb(byname[f"CBX-F_{nm}"]["solid"])} for nm, v in SC.BACK_WALL.items()},
        "back_wall_note": "all in the band z -213.5..-163.5 above the 127 mm device volume; recessed %d mm in front of the frame's back face "
                          "in S (M +110, L +220): reach under the base from behind" % round(S.FY1 - c["y1"]),
        "glands": {"G1_top_signal (floor sheet hole + compartment top, brush grommet)": {"x": list(SC.GLANDS["G1_top_signal"][:2]),
                                                                                          "y": list(SC.GLANDS["G1_top_signal"][2:]),
                                                                                          "carries": "under-deck rail, connector plane, bin-full beam lead, CTB fan lead"},
                   "G2_right_wall_M25": {"y": SC.GLANDS["G2_right_wall_y_chain"][0], "z": SC.GLANDS["G2_right_wall_y_chain"][1],
                                         "carries": "Y-chain leads (head, X motor) via the vertical duct YCD-F, the floor hole and conduit YLC-F",
                                         "conduit": _bb(byname["YLC-F_y_chain_lead_conduit_under_floor"]["solid"])},
                   "G4_floor_mains (floor sheet hole behind the compartment)": {"x": list(SC.GLANDS["G4_floor_mains_TC1_shaker"][:2]),
                                                                                "y": list(SC.GLANDS["G4_floor_mains_TC1_shaker"][2:]),
                                                                                "carries": "TC-1 and shaker mains leads down to their back-wall outlets "
                                                                                           "(mains kept apart from G1)"}},
        "operator_controls": {k: {"envelope": _bb(v["solid"])} for k, v in byname.items() if k.startswith("CTL-F")},
        "operator_controls_note": "e-stop (40 mm mushroom) and reset (22 mm illuminated) on the LEFT side panel %d mm behind the door plane at z "
                                  "300 / 220 (deck top z 0, bench z %d: %d / %d mm above the bench); the extraction fan already sets the left width, "
                                  "so the controls widen nothing; door sensor: coded non-contact safety sensor under the front roof member at x "
                                  "516..604, actuator on the door's inner face" % (SC.CONTROLS["e_stop"]["y_from_front"], BASE["z0"] - G.FOOT_H,
                                                                                    300 - (BASE["z0"] - G.FOOT_H), 220 - (BASE["z0"] - G.FOOT_H)),
        "CTB-1_fan_connector": {"envelope": _bb(byname["CON-F_A3_fan_2way"]["solid"]) if "CON-F_A3_fan_2way" in byname else None,
                                "what": "keyed 2-way panel connector beside the A3 place connector, under the deck: the CTB-1 module's "
                                        "heat-sink fan plugs in here; lead to G1"},
        "tip_bin_full_beam": {"parts": bb("TWB-F"), "what": "moved from the drawer to the fixed chute outlet (15 mm above it): it trips when "
                                                            "the heap under the chute reaches the outlet (crate rim + 30 mm); own 4-core lead up "
                                                            "the chute's +X wall, on the base top to the service loops and G1, not through the "
                                                            "B1 station connector; nothing rides on the drawer"}}


def syringe_tool():
    """the direct-drive syringe tool (head_l syringe_parts, modelled in 3D) and its return spring (head report syringe_return_spring)"""
    hr = json.loads((G.LH1 / "head_l" / "out" / "report.json").read_text())
    sp = hr["syringe_return_spring"]
    st = HL.SYR_T
    return {"syringe": "10 mL luer-lock, bore about %.1f mm (ASSUMED class, UNVERIFIED), 165 mm2" % st["bore_d"],
            "pusher_stroke_mm": 35.0, "contact_window_mm (pusher travel before it meets the plunger)": st["window"], "limit_for_5_mL": 4.0,
            "plunger_travel_mm": st["travel"], "fill_mL": round(st["travel"] * 165.1 / 1000.0, 2), "needed_for_5_mL_mm": round(5000.0 / 165.1, 1),
            "contact_window_set_by": "a lock-nutted M4 stop screw in the pusher pad, set with a 3 mm gauge at the bench",
            "return_spring": {"type": sp["spring"], "installed_length_mm": HL.SYR_SPRING["installed_l"], "rate_N_per_mm": sp["rate_N_per_mm"],
                              "force_plunger_out_N": sp["force_plunger_out_N"], "force_full_stroke_N": sp["force_full_stroke_N"],
                              "force_plunger_out_low_tolerance_N": sp["force_plunger_out_low_tolerance_N"],
                              "solid_length_mm": sp["solid_length_mm"], "length_at_full_stroke_mm": sp["length_at_full_stroke_mm"],
                              "seal_friction_N (ASSUMED, UNVERIFIED)": HL.SYR_SPRING["seal_friction_N"], "check_valve_N (ASSUMED)": HL.SYR_SPRING["check_valve_N"],
                              "needed": sp["need_basis"], "margin": sp["margin"], "margin_low_tolerance": sp["margin_low_tolerance"],
                              "shear_stress_full_stroke_MPa": sp["shear_stress_full_stroke_MPa"],
                              "pusher_cost": "the pusher works against at most %.1f N of spring (pusher about 128 N at 0.7 A and 5 mm/s)" % sp["force_full_stroke_N"],
                              "replaces": sp["replaces"]},
            "model": "head_l syringe_parts: body + flange clamp, barrel, luer tip and blunt needle, plunger rod and thumb rest (the standard "
                     "slider pad 2 mm under the tool-plate top), return spring (coil envelope in the head model; true helix in "
                     "cad/lh1/head_l/out/SRS-L_syringe_return_spring_helix.step and SYR-L_syringe_tool_assembly.step); checked in every "
                     "head configuration and at every deep place in the family runs",
            "status": "designed and checked in the model; nothing built; SYR stays uncommissioned until CT-SYR"}


# =====================================================================================================
# V13, V14, V17 (23 Nov 2026): reservoir at F2, cold places, installation envelope
# =====================================================================================================
RES_LOAD_8 = {"buffer A": 8.692, "clean-up beads": 3.148, "binding buffer": 3.70, "80 % ethanol": 13.30}   # mL at 8 reactions: run sheet loading table


def reservoir_plan(n=32, margin=1.5, col_ml=21.0, trough_ml=73.0):
    """V13: 4-trough 73 mL vs 12-column reservoir for 8..32 reactions; volumes scaled linearly from the 8-reaction run sheet (PLAN figures)"""
    out = {}
    for k in (8, 16, 32):
        v = {r: round(x * k / 8.0 * margin, 1) for r, x in RES_LOAD_8.items()}
        cols = {r: math.ceil(x / col_ml) for r, x in v.items()}
        out[k] = {"mL_x1.5": v, "columns_12col": cols, "columns_used": sum(cols.values()), "fits_12col": sum(cols.values()) <= 12,
                  "fits_4trough": all(x <= trough_ml for x in v.values())}
    return {"choice": "12-column reservoir (RES2) at F2 on S, with a slotted ethanol cover over its ethanol columns",
            "why": "four reagents need four separate wells; the 12-column reservoir holds all four at 8-32 reactions x 1.5 in <= %d columns, "
                   "lets the eight-channel draw a column per reagent, and leaves less in a 21 mL column than in a 73 mL trough at 8 reactions; "
                   "the 4-trough fits too but strands about 3-5 mL per trough at 8 reactions (ADR-013 reagent waste)" % out[32]["columns_used"],
            "by_reactions": out, "column_capacity_mL (ASSUMED class, UNVERIFIED)": col_ml,
            "ethanol_cover": "ECV-12: slotted PP cover over the ethanol columns (Y7 vapour case), 9 mm slots on the 9 mm pitch",
            "planner_items (not CAD)": "bead resuspension before each bead clean-up draw; ethanol loaded just in time (V13)",
            "basis": "reference run loading table (buffer A 8.69, beads 3.15, binding buffer 3.70, ethanol 13.3 mL) scaled by n/8"}


COLD_PLACES = {"cold reagent 1": {"place": "A3", "block": "CTB-1 cold tube block", "position": "row 6, column 7", "setpoint_C": 4},
               "cold reagent 2": {"place": "A3", "block": "CTB-1 cold tube block", "position": "row 6, column 8", "setpoint_C": 4}}


def cold_places(pl):
    """V14: places at their temperature for the two vessels with none; CTB-1 grid 8 x 6 at 13 mm (ASSUMED layout of its 48 positions)"""
    out = {}
    for nm, c in COLD_PLACES.items():
        n = c["place"]
        if n not in pl or pl[n]["key"] != "CTB1":
            out[nm] = {"error": "no CTB-1 at %s" % n}
            continue
        cx, cy = pl[n]["c"]
        r, k = int(c["position"].split()[1].strip(",")), int(c["position"].split()[-1])
        x, y = cx + (k - 4.5) * 13.0, cy + (r - 3.5) * 13.0
        out[nm] = dict(c, xy=[round(x, 1), round(y, 1)], vessel="1.5 mL tube, cap off at use", reach="S10 / S1000 (tube places: single channels only)",
                       note="CTB-1 holds 4 C on its thermal module (A3); the planner places both here and the person loads them cold")
    out["_basis"] = "CTB-1 grid ASSUMED 8 columns x 6 rows at 13 mm (deck_k CTB class); confirm against the CTB-1 drawing before loading rules use it"
    return out


def installation(S, env, mass, items):
    """V17: installation envelope for the person who receives the machine"""
    feet = [it for it in items if it["name"].startswith("EXT-foot")]
    nf = len(feet)
    m = mass["total_kg_ESTIMATE"] + 5.0                               # + liquids, tips, labware loaded (ASSUMED)
    w, d, h = env["with_fan"]
    door_h = (S.ROOF_Z + 20.0) - BASE["z0"]
    try:
        sy = yaml.safe_load((G.ROOT / "electronics" / "lh1" / "system.yaml").read_text())
        mains_txt = next((v for k, v in sy.get("family", {}).items() if k == "mains"), None)
    except Exception:  # noqa: BLE001
        mains_txt = None
    return {"footprint_mm (W x D x H incl. extraction fan, feet on the bench)": [w, d, h],
            "service_clearances_mm": {"front": "900 (door swing %d + a person)" % round(door_h), "left": 150, "right": 100, "back": 100, "above": 300},
            "bench_area_needed_mm (W x D)": [w + 250, d + 100],
            "bench_depth_min_mm": round(d), "bench_note": "a 750 mm lab bench is too shallow (feet would overhang): use a >= %d mm deep bench "
                                                          "or the floor trolley" % round(d),
            "mass_kg": {"machine_estimate": mass["total_kg_ESTIMATE"], "loaded": round(m)},
            "feet": nf, "load_per_foot_N": {"even": round(m * 9.81 / nf), "worst (uneven bench, 2 x even)": round(2 * m * 9.81 / nf)},
            "bench_rated_load_min_kg": round(1.5 * m),
            "height_above_floor_on_a_900_bench_mm": 900 + h, "ceiling_or_shelf_clear_above_bench_mm_min": h + 300,
            "door": "full-width PC door hinged on the front roof member, opening upwards on two gas struts (ASSUMED class); swing radius %d mm: "
                    "keep %d mm clear in front; open, the door stands level with the roof, %d mm above the bench" % (round(door_h), round(door_h), round(h)),
            "lifting": "%d kg: never by hand alone; scissor-lift table or 4 people at the 4 base corner extrusions after removing the enclosure "
                       "panels (about 41 kg) and deck modules (about 12 kg)" % round(mass["total_kg_ESTIMATE"]),
            "mains": "one 230 V C14 inlet (T10A), about 7 A worst coincident (electrical owner)" + (": " + str(mains_txt)[:160] if mains_txt else ""),
            "extraction": "100 mm duct from the inline fan on the left side panel (30 m3/h class) to a building extract point within 2 m; the "
                          "airflow switch must be made before a run (ethanol, Y7)",
            "other": "no water or gas; network to the host PC"}


def published_changes(S, rep):
    """what the wiring owner, the firmware and the planner must take from this CAD (per size; integration 23 Nov 2026 corrections:
    dock lift end, latch sensing, H1, tip parking in M and L, in-row stroke, open-height switch, shelf slots, tip gate)"""
    hr = json.loads((G.LH1 / "head_l" / "out" / "report.json").read_text())
    sen = hr["coupling"]["latch"]["sensing"]
    dl = rep["dock_lift"]
    tp = rep["tip_parking"]
    tg = rep["keepouts"]["tip_check_gate_beam"]
    nh = [c["name"] for c in rep["shelf_plan"].values() if c["kind"] == "handoff"]
    return {
        "wiring (electrical owner)": [
            "dock lift DLB-1: one more stepper (NEMA17 48 + Tr8x2) at the dock row's LEFT end (left of bay 1; the head enters the row at its "
            "right end, where nothing may stand), run 1.3 A, plus its DOWN slot sensor and its OPEN-height switch, both at the left end "
            "(dock_lift.parts_xyz); a spare Octopus socket; leads through the connector plane",
            "pusher motors: Nanotec LGA351S12-A-UIAP-038 (1.2 A winding, 1.85 ohm, 2.46 mH, JST B6B-EH-A on the motor) at 0.7 A; carriage B's "
            "0.5 mm FFC check stays at 0.7 A (the winding is rated 1.2 A: the 'rated <= 1.0 A' purchase condition no longer holds; 0.7 A run only)",
            "pusher RETRACTED switch kept (sealed subminiature), now beside the D12 rod collar under the U standoff, 24.6 mm above the bracket",
            "latch sensing: TWO single-edge sealed switches per carriage at the jaw (LOCK operates at +%.2f mm, EMPTY at %.2f mm; jaw closed "
            "empty at %.2f mm) on ONE carriage-cable way through the firmware's ladder: Rs 1.0 k, R0 22 k, LOCK adds 4.7 k, EMPTY adds 2.0 k "
            "(was 1.5 k: EMPTY now reads with LOCK also made): LOCKED %d ohm, CLOSED-EMPTY %d ohm, NOT CLOSED %d ohm; resistors and switch "
            "leads in lever splices on the switch bracket (no soldering)" % (sen["lock_switch_operates_at_mm"], sen["empty_switch_operates_at_mm"],
                                                                              sen["jaw_closed_empty_mm"], sen["ladder_ohm"]["LOCKED"],
                                                                              sen["ladder_ohm"]["CLOSED-EMPTY"], sen["ladder_ohm"]["NOT CLOSED"]),
            ("edge hand-offs: NONE open on S: H1 and H2 both closed by blank plates (H1's slot OF holds the spent-wafer stack, H2's slot OA a "
             "p300 tower): no shutter switch is fitted in S; the S->M kit fits both shutters and both switches") if S.name == "S" else
            ("edge hand-offs H1 and H2 open, each with its shutter-closed switch (the S->M kit fits both: S has blank plates)"),
            "enclosure 50 mm taller (roof frame z 760 -> 810, it follows the LGA35 pusher top) and 10 mm wider on the left (frame, Y rail, enclosure): lead lengths"],
        "firmware": [
            "LH_TOOL_PARK / LH_TOOL_PICK replaced by the dock-lift sequences (head report safety.sequences) and their guards; Z only positions; "
            "in-row move of the empty carriage at s %.0f (was 307: %.1f mm over the parked studs, below the 8 mm dock rule; now %.1f mm)" %
            (dl["in_row_stroke_s"], dl["in_row_clearance_at_307_mm"], dl["in_row_clearance_mm"]),
            "pusher: rotation_distance 0.6096 mm (328.08 full steps/mm), stroke 38.1 mm, soft limits 0.5 .. 37.5, run 0.7 A, homing <= 0.50 A and "
            "<= 2 mm/s; never into the internal end stop above 0.57 A; force strokes <= 5 mm/s",
            "dock lift bar OPEN-height switch: operates at %.1f mm of bar stroke (roller rise + 2 mm), made to the end of the stroke" % dl["open_height_mm"],
            "tip parking in EVERY size: random access TPK-S at %s, tool list ['M300'] only, spots and entries per family_<size> tip_parking.groups, "
            "s_seat %.1f / s_slide %.1f; re-pick force-ended to s_seat + 1 (CAD reach s_target %.1f, checked); slide <= 10 mm/s and <= 0.25 m/s2; "
            "ownership per spot" % (tp["place"], tp["s_values"]["s_seat"], tp["s_values"]["s_slide"], tp["s_values"]["s_repick_max"]),
            "tip-check gate beam: along X at y %.1f, z %.1f (deck-top frame), emitter / receiver faces x %.1f / %.1f; nozzle x for the scan %.1f "
            "(keepouts tip_check_gate_beam)" % (tg["beam_y"], tg["beam_z"], tg["x_faces"][0], tg["x_faces"][1], tg["gate_x"]),
            "shelf slots: centre XY per slot published (shelf_slots), no longer from the slot names",
            "X travel left end 10 mm further left (travel_xc[0]); dock-row unchanged"],
        "planner": [
            "tip parking: 6 random-access spots (M300 only), any order, in every size (TPK-S at B0; TPK-1 LIFO at C0 withdrawn in M and L)",
            "F2 = 12-column reservoir (RES2) with an ethanol cover in S; cold reagent 1 and enzyme 1:5 at A3 (CTB-1, 4 C) positions in cold_places",
            ("tip supply S: 5 towers (p300 8 + 8 + 6, p200 8, p10 5) + frames, spent wafers at OF; NO edge hand-off on S (H1 and H2 closed)"
             if S.name == "S" else "edge hand-offs open: %s" % ", ".join(nh)),
            "shelf slot XY: shelf_slots (centre, what stands there, gripper stroke)",
            "exchange time: head model %.1f s per exchange (dock lift strokes at 5 mm/s, the 13 mm follow steps, in-row stroke s %.0f); the family "
            "exchange_s figure is the study model and does not include it" % (hr["exchange"]["total_s"], dl["in_row_stroke_s"])]}


# =====================================================================================================
# electrical owner's requests and the open questions (CALCULATED / ASSUMED as marked)
# =====================================================================================================
def _thrust(T_hold, eff=0.35, pullout=0.6, lead=1.0):
    return round(2 * math.pi * eff * T_hold * pullout / (lead / 1000.0))


PUSHER = {"NEMA11_34_present": {"holding_Nm": 0.06, "thrust_N_at_10mm_s": _thrust(0.06), "body_mm": "28 x 28 x 34"},
          "NEMA11_51": {"holding_Nm": 0.12, "thrust_N_at_10mm_s": _thrust(0.12), "body_mm": "28 x 28 x 51 (head +17 mm taller)"},
          "NEMA14_34": {"holding_Nm": 0.14, "thrust_N_at_10mm_s": _thrust(0.14), "body_mm": "35 x 35 x 34 (head height unchanged)"},
          "NEMA14_47": {"holding_Nm": 0.28, "thrust_N_at_10mm_s": _thrust(0.28), "body_mm": "35 x 35 x 47 (head +13 mm, roof +10)"},
          "basis": "thrust = 2 pi eta T / lead; 1 mm lead, eta 0.35 (ASSUMED trapezoidal lead screw in a captive actuator), torque at 600 rpm "
                   "(10 mm/s) = 60 % of holding (ASSUMED for a small hybrid stepper at 24 V on a TMC5160-class driver); holding torques are "
                   "catalogue-class values (ASSUMED); the electrical owner's 66 N for the present motor agrees within 20 %"}


def cr_answers(rep_S):
    yc = SC.y_chain(G.Size("S"))
    lg = {ax: G.loop_geometry(ax) for ax in G.FFC}
    return {
        "CR-L1 cable chains": {
            "X chain": "igus E2 medium class (inner 38 x 18, R 55, bend height 135 + 25) on the X beam top in a U tray XCT-F (x -300..560, front "
                       "wall 35 mm high under the moving-end bracket, back wall to z 735, 25 mm under the roof frame); fixed end at the beam middle "
                       "(x about 138), moving end on bracket XCB-F on the carriage plate top (local x 60..80, behind the beam face); length about "
                       f"{round((793.5) / 2 + math.pi * 55 + 100)} mm",
            "Y chain": f"same chain in an outboard tray YCT-F along the right side beam (x 641..695, floor z 322, outer wall to z 472, inner wall to 455 "
                       f"under the Y blocks); moving end on bracket YCB-F under the right Y carriage (between its two blocks); fixed end at the tray middle "
                       f"(S: y {yc['fixed_y']:.0f}); chain length S {yc['chain_length_mm']} mm (M +55, L +110; links added per kit); the cables go "
                       "down a vertical duct YCD-F inside the right panel to a floor gland and the control compartment",
            "status": "modelled as envelopes and checked against the frame, gantry and head (0 interferences); fill (cross-section +30 % spares) is "
                      "the electrical owner's check against inner 38 x 18"},
        "CR-L2 head loops": {
            "design": "one rolling loop per axis (R 25, fixed clamp at mid-range on a guide plate, moving clamp on an arm; free length "
                      f"{lg['carriage_A']['L']:.0f} mm = stroke/2 + pi R + over-travel): carriage A beside the carriage plate's left end (x -160..-136); "
                      "carriage B in the 13 mm slot between the A and B moving assemblies (x -45..-32); gripper in the 13 mm slot between B and the "
                      "gripper axis (x 45..58).  The pusher-top clamps keep the loop bottom at z >= 357 at full stroke, above the parked tools (307) "
                      "and release posts (340) of the neighbouring bays",
            "change_to_the_request": "carriage A keeps the 20-way 1.0 mm FFC (21 mm wide).  Carriage B and the gripper cannot take a 21 mm FFC: "
                                     "B uses a 20-way 0.5 mm FFC (10.5 mm wide, 1.25 mm each side in its slot) with each pusher-motor phase on two "
                                     "conductors; the gripper a 12-way 0.5 mm FFC (6.5 mm).  The alternative is a 10 mm wider axis pitch A-B, which "
                                     "widens the head and the machine",
            "status": "modelled (stroke-dependent loop envelopes) and checked in every head configuration of the S and L runs: 0 interferences"},
        "CR-L4 rear electronics plate": {
            "design": "a control compartment CBX-F UNDER the base (y 35..345, x -495..495, z -345..-162, behind the drawer, between the feet), fixed in "
                      "every size (it never moves on an extension): the control-bay plate (640 x 310, rev L CB-L) and CBP2 (235 x 295 at x 245..480, "
                      "as requested) side by side, 127 mm device height; sealed from the deck air by the floor sheets (the enclosure's new bottom, "
                      "with glands G1 top, G2 right wall, G4 floor for mains); own air: two 92 mm intake fans with filters on its RIGHT wall, "
                      "louvred exhaust grille on its LEFT wall (replaces the 60 mm fan: see 'compartment airflow')",
            "consequence": "TC_RELAY, TB_RELAY, contactors and safety relays leave the enclosure air altogether; harness lengths change (wiring owner)"},
        "CR-L5 pusher": {
            "retracted_switch": "sealed subminiature switch on each carriage's pusher bracket, made by a collar on the pusher rod at the top of its stroke",
            "motor": PUSHER, "choice": "NEMA14 34 mm captive linear stepper, 1 mm lead: CALCULATED about %d N at 10 mm/s (1.5 x 120 = 180 N needed for "
                                       "the 1.5 x rule: met by %d N); room made by a new pusher bracket (flange under the motor, side walls) and by moving "
                                       "the eject finger line 4 mm to axis x -26 (eject arm, post, pin, bracket hole, tool-plate eject pin and cradle "
                                       "eject plunger): 4.5 mm to the eject arm, as today" % (PUSHER["NEMA14_34"]["thrust_N_at_10mm_s"],
                                                                                              PUSHER["NEMA14_34"]["thrust_N_at_10mm_s"]),
            "NEMA11_51_verdict": f"about {PUSHER['NEMA11_51']['thrust_N_at_10mm_s']} N: 1.3 x 120, below the 1.5 x rule, and the head 17 mm taller"},
        "CR-L6 extraction intake LOW": f"intake grille at z -80 (under the deck level) at the back of the left side panel, inline EC fan outside the left "
                                       f"panel (+150 mm width, no depth); the deck_k top-of-back-panel extract is dropped; with the new floor sheets the enclosure "
                                       f"is closed at the bottom, so the low intake draws from the lowest air",
        "CR-L7 bought sensors": "the Hall board is replaced by four 6.5 mm cylindrical leaded Hall switches pushed into reamed holes through the coupling "
                                "bracket (tips 1 mm above its underside, over the tool-plate magnets) under a clamp plate; latch, force (push-rod), "
                                "pusher-retracted and gripper switches are sealed subminiature types with factory leads (envelope 10.2 x 6.4 x 20, "
                                "ASSUMED class); the dock bay switches keep their envelope (already that class); Z-up switches are on the carriage plate "
                                "(head_l), same class",
        "CR-L8 notches and rail": "every window has the 30 x 14 notch at place-local x +15..+45 (deck strips); keyed connectors fitted at the powered "
                                  "places only (S: tip waste B1, CTB zone A3, strip B2, LW B3, CPB-1 G4, hazardous liquid G0 reserved; TC-1 and the shaker "
                                  "bring their own leads); a 3-way cable duct (40 x 16) under the deck at x -25..15 on the base cross members from "
                                  "row F to the service loops, one 110 mm segment per inserted row",
        "connector plane": "CPL-F: keyed connector plate under the front edge of row G (x -60..60, behind the joint bar), on the back section; 250 mm "
                           "service loops flat in front of it; dock switches, row-G places, Y motors and endstops plug in here, so an extension "
                           "inserts keyed leads",
        "open_question_gripper_switches": "head_l models one 'jaw switch'.  The family head carries both: jaws-open limit and 'gripped' on the "
                                          "sprung finger, sealed subminiature, on the gripper body's +X face (inside the head envelope); checked",
        "open_question_syringe_stroke": {
            "finding": "head_l's SYR envelope says a 2:1 lever turns the 35 mm pusher stroke into 70 mm at the plunger, but the modelled lever has "
                       "its input (pusher rod) and output (plunger link) on the same axis, so it gives no ratio; a real 2:1 lever with a 35 mm input "
                       "and the 28 mm arm the tool has room for would swing about 50 degrees: as drawn, it does not work",
            "redesign": "drive a 10 mL luer-lock syringe DIRECTLY from the pusher: about 14.5 mm bore (ASSUMED), so the same 5 mL per fill needs about "
                        "30 mm of plunger travel, inside the 35 mm pusher stroke; no lever, no custom mechanism; force at the plunger = pusher "
                        "thrust (NEMA14: about %d N, about 0.9 MPa on 165 mm2, far above a jetting need of the order of 0.1-0.2 MPa, ASSUMED); "
                        "resolution about 0.17 uL per full step at 1 mm lead and 200 steps per rev (CALCULATED)" % PUSHER["NEMA14_34"]["thrust_N_at_10mm_s"],
            "fallback": "a rack-and-pinion stroke doubler (pinion on a pusher-driven slider between a fixed and a moving rack): 2:1, bought racks "
                        "and pinion, half the force at the plunger; only if the rig shows the 10 mL syringe cannot jet the small volumes",
            "firmware": "SYR can be re-armed with stroke <= 35 mm of pusher = plunger; contact window as the other tools; to be proved on the rig"},
        "open_question_noz_dy_dock_yf": {
            "noz_dy_real_mm": AY, "gripper_finger_dy_mm": G.GRIP_Y,
            "dock_yf_real": {nm: round(G.Size(nm).dock_y - AY, 1) for nm in G.SIZES},
            "finding": "printer.cfg holds noz_dy -73.5 and dock_yf 545.5.  The coupling axis (= nozzle row centre) is 102.5 mm in front of the beam "
                       "front face (head_l AXY), so noz_dy = -102.5; with -73.5 every place guard and tip-gate position is 29 mm off in Y.  The two "
                       "provisional numbers are consistent with each other (545.5 - 73.5 = 472, the rev L dock row), so the dock was aimed right "
                       "by the firmware's own arithmetic but the real nozzle would come down 29 mm in front of the bay; the real printer Y over the "
                       "bay row is dock_y + 102.5: L 574.5 (as rev L), M 464.5, S 354.5",
            "also_per_size": "dock_zone_yf = dock_yf - 94.5; dock_exit_yf = dock_yf - 124.5 (rev L ratio); travel YF and place list per size "
                             "(family_<size>.travel / deck_positions)"},
        "tip parking on the starter (ADR-013)": {k: rep_S["tip_parking"].get(k) for k in ("place", "station", "part", "capacity_tips", "required_tips",
                                                                                           "positions_x", "height_above_seat_mm", "need_basis")},
        "keep-outs published": "family_<size>.keepouts: TC-1 at F1 with its lid park A1 and the lid travel corridor, the tip-check gate / tip waste "
                               "B1 (funnel, VC-2, chute, beam, connector), magnet MB-1 at B4",
        "compartment airflow": dict(airflow(), answer="2 x 92 mm intake fans on the RIGHT wall, louvred exhaust grille on the LEFT wall; the scene "
                                                      "and this file now agree (the old text said intake left, the scene intake right)"),
        "back wall, glands, controls": "family_<size>.services_interface: mains inlet (C14 fused, switched), outlets TC-1 and shaker (C13), "
                                       "earth stud M6 on the compartment back wall; glands G1 (top, signal), G2 (right wall, Y chain), G4 "
                                       "(floor, mains); e-stop and reset on the left side panel, door sensor under the front roof member",
        "syringe tool": syringe_tool(),
        "CTB-1 fan": "2-way keyed connector CON-F_A3_fan_2way beside the A3 place connector",
        "tip-bin-full beam": "moved to the fixed chute outlet with its own lead to G1 (services_interface.tip_bin_full_beam)",
        "S figures": {"interference": rep_S["interference"]["count_real"], "configs": rep_S["interference"]["head_configs_checked"]},
    }


# =====================================================================================================
# parts lists, kits, build steps
# =====================================================================================================
EXT_KG_M = {"2040": 0.49, "2020": 0.26}
RHO = {"al": 2.70e-6, "steel": 7.85e-6, "pa12": 1.01e-6, "pc": 1.20e-6, "pp": 0.90e-6}
REPLACE = ("EXT-side_beam", "MGN15_Y_rail", "YCT-F_y_chain_tray")
EXTEND_NOTE = {"YCH-F_y_chain_envelope": "chain links added (same chain)"}


def base_name(it):
    nm = it.get("part") or it["name"]
    import re
    nm = re.sub(r"_y[+-]\d+.*$", "", nm)
    nm = re.sub(r"_(front|back)$", lambda m: "_" + m.group(1), nm)
    nm = re.sub(r"^DSB-F_deck_bar_(joint|front|back).*$", r"DSB-F_deck_bar", nm)
    nm = re.sub(r"^SQT-deck_post_split.*$", "SQT-deck_post_split", nm)
    nm = re.sub(r"^DOCK_bay\d_", "DOCK_bay_", nm)
    nm = re.sub(r"^EXT-dock_(rail|post)_.*$", r"EXT-dock_\1", nm)
    nm = re.sub(r"^EXT-base_cross_dock_\d$", "EXT-base_cross_dock", nm)
    nm = re.sub(r"^EXT-foot_.*$", "EXT-foot_post", nm)
    nm = re.sub(r"^EXT-upright_mid_.*$", "EXT-upright_mid", nm)
    nm = re.sub(r"^EXT-upright_corner_.*$", "EXT-upright_corner", nm)
    nm = re.sub(r"^BCB-corner_bracket_base_.*$", "BCB-corner_bracket_base", nm)
    nm = re.sub(r"^JP-(base_side|base_long)_joint_.*$", r"JP-\1_joint_plate", nm)
    nm = re.sub(r"^CON-F_[A-Z]\d_", "CON-F_", nm)
    nm = re.sub(r"^[IO][A-Z]_guides$", "STG-F_tower_guides", nm)
    nm = re.sub(r"^O[A-Z]_H\d_nest$", "HN-F_handoff_nest", nm)
    return nm


def parts_rows(items):
    rows, seen = [], {}
    for it in items:
        if it["cls"] in ("labware", "reserved") or it["group"] == "parked_tool":
            continue
        base = base_name(it)
        bb = it["solid"].BoundingBox()
        dims = sorted([bb.xlen, bb.ylen, bb.zlen])
        key = (base, round(dims[0]), round(dims[1]), round(dims[2]))
        if key in seen:
            seen[key]["qty"] += 1
            continue
        proc = ("bought" if it["cls"] == "bought" else "cut_to_length" if base.startswith(("EXT", "SQT", "DSB", "TUBE")) else
                "sheet_laser" if base.startswith(("DK-F", "ENC", "TSS", "DRW", "CBX", "CBP", "XCT", "YCT", "YCD", "CPL", "SPT", "DMK")) else "cnc_or_print")
        mass = (EXT_KG_M["2040" if dims[1] > 30 else "2020"] * dims[2] / 1000.0) if base.startswith("EXT-") else it["solid"].Volume() * RHO.get(it.get("mat", "al"), RHO["al"])
        r = {"part": base, "group": it["group"], "kind": "bought" if it["cls"] == "bought" else "custom", "process": proc, "material": it.get("mat", "al"),
             "envelope_mm": f"{dims[2]:.0f} x {dims[1]:.0f} x {dims[0]:.0f}", "qty": 1, "mass_kg_each": round(mass, 3)}
        seen[key] = r
        rows.append(r)
    return rows


def write_csv(path, rows):
    path.parent.mkdir(parents=True, exist_ok=True)
    with open(path, "w", newline="") as fh:
        w = csv.DictWriter(fh, fieldnames=list(rows[0]))
        w.writeheader()
        w.writerows(rows)


def kit(rows_a, rows_b, layout_a, layout_b, a, b):
    ka = {(r["part"], r["envelope_mm"]): r for r in rows_a}
    kb = {(r["part"], r["envelope_mm"]): r for r in rows_b}
    names_a = {}
    for (p, e), r in ka.items():
        names_a.setdefault(p, []).append(r)
    out = {"re_used": [], "added": [], "replaced": [], "extended_in_place": [], "moved_back_110": [], "removed": []}
    for (p, e), r in kb.items():
        ra = ka.get((p, e))
        if ra:
            out["re_used"].append({"part": p, "envelope_mm": e, "qty": min(ra["qty"], r["qty"])})
            if r["qty"] > ra["qty"]:
                out["added"].append({"part": p, "envelope_mm": e, "qty": r["qty"] - ra["qty"], "kind": r["kind"], "why": "more of the same (new row / bay)"})
            elif r["qty"] < ra["qty"]:
                out["removed"].append({"part": p, "envelope_mm": e, "qty": ra["qty"] - r["qty"]})
            continue
        if p in names_a and p.startswith(REPLACE):
            out["replaced"].append({"part": p, "old_envelope_mm": [x["envelope_mm"] for x in names_a[p]], "new_envelope_mm": e, "qty": r["qty"], "kind": r["kind"]})
        elif p in names_a and p.startswith("YCH-F"):
            out["extended_in_place"].append({"part": p, "old_envelope_mm": [x["envelope_mm"] for x in names_a[p]], "new_envelope_mm": e, "qty": r["qty"],
                                             "note": EXTEND_NOTE.get(p.split(" (")[0], "longer / re-cut (see the study's table)")})
        else:
            out["added"].append({"part": p, "envelope_mm": e, "qty": r["qty"], "kind": r["kind"],
                                 "why": "extension piece / joint" if ("_ext" in p or p.startswith("JP-")) else "new"})
    for (p, e), r in ka.items():
        if (p, e) not in kb and not any(x["part"] == p for x in out["replaced"] + out["extended_in_place"]):
            out["removed"].append({"part": p, "envelope_mm": e, "qty": r["qty"]})
    out["moved_back_110"] = ["the back section: row G strip and its stations, dock base with 8 bays and tools, dock rails and posts, dock-rail base "
                             "cross members, back base member, back corner uprights and feet, back top member, roof back member, back panel, the "
                             "back sheets of side panels, roof and floor, the connector plane, the Y motors"]
    new_st = {n: k for n, k in layout_b.items() if layout_a.get(n) != k}
    out["stations_added"] = new_st
    out["stations_moved"] = {n: k for n, k in layout_a.items() if n not in layout_b}
    custom_added = sum(x["qty"] for x in out["added"] if x["kind"] == "custom")
    bought_added = sum(x["qty"] for x in out["added"] if x["kind"] == "bought")
    out["counts"] = {"re_used_lines": len(out["re_used"]), "replaced_lines": len(out["replaced"]), "extended_lines": len(out["extended_in_place"]),
                     "added_lines": len(out["added"]), "added_custom_qty": custom_added, "added_bought_qty": bought_added, "removed_lines": len(out["removed"])}
    out["steps"] = [
        "1 Park the head at the front, park the tools in the dock, then take the tools out to a rack; switch off; unplug the back-section connectors at the connector plane CPL-F",
        "2 Remove the back panel, the back sheets of the side panels, roof and floor, and the butt straps at the back-section joint",
        "3 Lift the gantry off (two people; Y moving mass about 22 kg) or slide it to the front on its blocks",
        "4 Unbolt the back section from the base and side members and move it back 110 mm on the extended base members (fit the 110 mm "
        "pieces with their joining plates and the new foot pair under each joint first)",
        "5 Fit the new side beams and Y rails (replaced, never jointed); align with the rail jig RJ and a straightedge",
        "6 Refit the gantry, fit the longer Y belts and tension them (frequency method); fit the new Y chain tray and add chain links",
        f"7 Fit the inserted row: deck strip, joint bar and its two posts, the stations of row {', '.join(sorted({n[0] for n in new_st}))}, "
        "the under-deck rail segment and the extension leads at the connector plane",
        "8 Fit the bay sheets of the side panels, roof and floor with butt straps and gasket tape; refit the back sheets and back panel; extend the shelf",
        "9 Re-commission: Y travel and endstops (size config), input shaping with the accelerometer, the head camera re-measures every deck place "
        "and the dock bay markers, 20 exchanges per carriage, extraction airflow switch re-proved, enclosure leak check at the new laps"]
    out["downtime_ASSUMED"] = "3-4 working days (mechanical 1.5-2 days with two people, commissioning 1 day, one validation run 1 day: study)"
    return out


def build_steps(rows):
    steps = {}
    groups = (("LH-F-S01_frame_and_feet", ("frame",)), ("LH-F-S02_gantry_and_x_chain", ("gantry",)), ("LH-F-S03_deck_strips_and_connectors", ("deck",)),
              ("LH-F-S04_stations", ("station",)), ("LH-F-S05_shelf_and_hand-offs", ("shelf",)), ("LH-F-S06_dock_and_markers", ("dock",)),
              ("LH-F-S07_drawer_crate_waste", ("drawer",)), ("LH-F-S08_services_chains_rail_connector_plane_compartment", ("services",)),
              ("LH-F-S09_enclosure_floor_fan", ("enclosure",)))
    for code, gs in groups:
        steps[code] = [{"part": r["part"], "qty": int(r["qty"]), "kind": r["kind"]} for r in rows if r["group"] in gs]
    steps["LH-F-S10_head (head_l parts + family head changes)"] = [{"part": r["part"], "qty": int(r["qty"])} for r in csv.DictReader(open(G.LH1 / "head_l" / "out" / "head_l_parts.csv"))
                                                                   if not r["part"].startswith(("CBL", "TPL", "PS-L"))] + [
        {"part": "PB-F_pusher_bracket_NEMA14 (replaces PB-L)", "qty": 2}, {"part": "B-captive_linear_stepper_NEMA14_34mm_1mm_lead (replaces NEMA11)", "qty": 2},
        {"part": "B-sealed_subminiature_switch_factory_leads (retracted x2, latch x2, force x2, gripper x2)", "qty": 8},
        {"part": "B-Hall_switch_6.5mm_cylindrical_leaded (replaces the Hall board)", "qty": 8}, {"part": "HCP-F_hall_clamp_plate", "qty": 2},
        {"part": "EJA-F_eject_arm_post_pin moved to x -26", "qty": 2}, {"part": "FGP-F_ffc_guide_plate (A, B, gripper)", "qty": 3},
        {"part": "FFA-F_ffc_moving_arm (A, B, gripper)", "qty": 3}, {"part": "B-FFC_20way_1.0mm (A)", "qty": 1}, {"part": "B-FFC_20way_0.5mm (B)", "qty": 1},
        {"part": "B-FFC_12way_0.5mm (gripper)", "qty": 1}, {"part": "XCB-F_x_chain_end_bracket", "qty": 1}]
    steps["LH-F-S11_tools"] = [{"part": r["part"], "qty": int(r["qty"])} for r in csv.DictReader(open(G.LH1 / "head_l" / "out" / "head_l_parts.csv"))
                               if r["part"].startswith(("CBL", "TPL", "PS-L"))] + [{"part": "SYR: 10 mL luer-lock syringe direct drive (replaces the 2:1 lever)", "qty": 1}]
    return steps


def vs_rev_l(rows):
    rl = list(csv.DictReader(open(G.LH1 / "machine_l" / "out" / "machine_l_parts.csv")))

    def summ(rr, qkey="qty"):
        d = {}
        for r in rr:
            g = r["group"]
            d.setdefault(g, {"custom": 0, "bought": 0, "lines": 0})
            d[g][r["kind"]] += int(r[qkey])
            d[g]["lines"] += 1
        return d
    a, b = summ(rl), summ(rows)
    tot = lambda d, k: sum(v[k] for v in d.values())  # noqa: E731
    return {"by_group_rev_L": a, "by_group_S": b,
            "totals": {"rev_L": {"lines": tot(a, "lines"), "custom_qty": tot(a, "custom"), "bought_qty": tot(a, "bought")},
                       "S": {"lines": tot(b, "lines"), "custom_qty": tot(b, "custom"), "bought_qty": tot(b, "bought")}},
            "note": "machine-level parts only (frame, gantry, deck, stations' custom parts, shelf, dock, drawer, enclosure, services); labware, "
                    "parked tools and the head are counted elsewhere (head_l parts list + the family head changes in proposed_build_parts_S.yaml)"}


# =====================================================================================================
# one size
# =====================================================================================================
def run_size(name, head, step=True):
    S = G.Size(name)
    od = OUT / name
    od.mkdir(parents=True, exist_ok=True)
    items, zs, pl, plan = SC.scene(S)
    log(name, "items", len(items))
    invalid = [it["name"] for it in items if not it["solid"].isValid()]
    rules = DR.report(S.rows, S.layout)
    stat = CK.static_pairs(items)
    log(name, "rules", rules["pass"], "static overlaps", stat["count"])
    g_hits, hg = CK.gantry_checks(S, items, head, SC.gantry_items)
    res, bpt = CK.run_checks(S, M.Scene(items), head, zs, pl, plan, log=lambda *a: log(name, *a))
    real = res["transit"] + res["lowered"] + res["gripper"] + res["dock"] + res["eject"]
    expected_rf = [q for q in res["reach_fail"] if q.get("place") == "IG" and q.get("carriage") == "carriage_A"]
    rf = [q for q in res["reach_fail"] if q not in expected_rf]
    groups = {}
    for h in real:
        k = (h["head_part"].split(":")[0] + ":" + h["head_part"].split(":")[-1], h["item"])
        g = groups.setdefault(k, {"head_part": k[0], "item": k[1], "n_configs": 0, "max_mm3": 0.0, "example": h["config"]})
        g["n_configs"] += 1
        g["max_mm3"] = max(g["max_mm3"], h["mm3"])
    save(od / "checks_partial.json", {"static": stat, "real": len(real), "configs": res["configs"]})
    zc = M.z_clearance(head, items)
    trav = travel_report(S, pl, plan)
    stiff = stiffness(S)
    dyn = dynamics(S, head)
    gm = SC.gantry_items(S, (S.YF0 + S.YF1) / 2)
    masses = M.mass_of([it for it in items if it["group"] not in ("services",) and "envelope" not in it["name"]] + [g for g in gm if "envelope" not in g["name"]])
    masses["services"] = round(sum(it["solid"].Volume() * RHO.get(it["mat"], RHO["al"]) for it in items if it["group"] == "services" and it["cls"] != "reserved"
                                   and "envelope" not in it["name"]), 2)
    masses["cable_chains (0.8 kg/m ASSUMED)"] = round(0.8 * (SC.y_chain(S)["chain_length_mm"] + 800.0) / 1000.0, 2)
    hr = json.loads((G.LH1 / "head_l" / "out" / "report.json").read_text())
    n_st = len([k for k in S.layout.values() if k != "EMPTY"])
    mass = {"by_group_modelled": masses, "head_kg": dyn["head_kg"], "tools_6_kg (ASSUMED 0.9 each)": 5.4,
            "deck_modules_kg (ASSUMED pro rata, rev L about 15 over 23 stations)": round(15.0 * n_st / 23.0, 1),
            "electrics_fasteners_kg (ASSUMED, rev L 15-20)": 17.5}
    mass["total_kg_ESTIMATE"] = round(sum(masses.values()) + dyn["head_kg"] + 5.4 + mass["deck_modules_kg (ASSUMED pro rata, rev L about 15 over 23 stations)"] + 17.5)
    env_items = [it for it in items if it["group"] in ("frame", "enclosure", "drawer")]
    bb = cq.Compound.makeCompound([it["solid"] for it in env_items]).BoundingBox()
    bbn = cq.Compound.makeCompound([it["solid"] for it in env_items if not it["name"].startswith("EXF-F")]).BoundingBox()
    env = {"with_fan": [round(bb.xlen), round(bb.ylen), round(bb.zlen)], "without_fan": [round(bbn.xlen), round(bbn.ylen), round(bbn.zlen)],
           "x": [round(bb.xmin, 1), round(bb.xmax, 1)], "y": [round(bb.ymin, 1), round(bb.ymax, 1)], "z": [round(bb.zmin, 1), round(bb.zmax, 1)],
           "study_W_D_H (no fan)": S.study["envelope_W_D_H_mm (frame, panels, feet, drawer; no fan)"],
           "note": "frame, panels, floor, feet, drawer; the extraction fan on the left side panel adds width only; the control compartment is under the base"}
    rows = parts_rows(items + gm)
    write_csv(od / f"parts_{name}.csv", rows)
    rep = {"module": f"LH-1 machine family, size {name} (cad/lh1/machine_family)", "size": name,
           "status": "CAD and calculation only; nothing built or measured; every figure CALCULATED or ASSUMED as labelled",
           "geometry": S.summary(), "parts_valid": not invalid, "invalid": invalid,
           "deck_rules": rules,
           "interference": {"static_all_pairs": stat, "gantry_vs_static": g_hits, "head_vs_gantry": hg, "head_configs_checked": res["configs"],
                            "count_real": len(real), "by_kind": {k: len(res[k]) for k in ("transit", "lowered", "gripper", "dock", "eject")},
                            "grouped": sorted(groups.values(), key=lambda g: -g["n_configs"]), "all": real[:300],
                            "method": "machine_l engine with the family head: transit over every place and shelf slot with three tool pairs, travel "
                                      "corners, the dock entry lane and the dock row; every pipette lowered at every place it serves on both carriages at "
                                      "50/85/100 % of its working stroke; the gripper at every place and slot it serves (fingers open); drop-off and "
                                      "pick-up at all 8 bays by both carriages with the other 7 bays full; eject over-travel at the tip waste; frame, "
                                      "enclosure, drawer, services and shelf in the scene"},
           "reach_failures": rf, "reach_failures_by_design": expected_rf,
           "reach_limits_body_in_target_labware": sorted({(q["config"].split(" lowered at ")[0], q["place"]) for q in res["reach_limit_body_in_labware"]}),
           "z_clearance": zc, "dock_clearance": dock_clearance(head, items), "travel": trav, "stiffness": stiff, "dynamics": dyn, "exchange_time": exchange(S),
           "ethanol": SF.ethanol(S.geo_dict()), "envelope_mm": env, "mass_kg": mass, "tip_supply": tips_and_waste(S, plan), "shelf_plan": plan["slots"],
           "y_chain": SC.y_chain(S), "head_extra_mass": head_extra_mass(head), "parts_lines": len(rows)}
    rep["deck_positions"] = deck_positions(S, pl, bpt)
    rep["by_place_and_tool"] = bpt
    rep["tube_positions"] = tube_positions(pl)
    rep["pipettes"] = pipettes_report(bpt)
    rep["tip_parking"] = tip_parking(S, pl, items, head, bpt, rep)
    rep["keepouts"] = keepouts(S, pl, items)
    rep["services_interface"] = services_interface(S, items)
    rep["syringe_tool"] = syringe_tool()
    rep["headroom"] = headroom(S, zc, trav, stiff, plan, dyn, rep["tip_parking"])
    rep["reservoir_F2"] = reservoir_plan() if pl.get("F2", {}).get("key") == "RES2" else None
    rep["cold_places"] = cold_places(pl)
    rep["installation"] = installation(S, env, mass, items)
    rep["mechanism_audit"] = MX.audit()
    rep["dock_lift"] = dock_lift_block(S, items)
    rep["shelf_slots"] = shelf_slots(S, plan)
    rep["published_changes_23nov"] = published_changes(S, rep)
    rep["runtime_s"] = round(time.time() - T0, 1)
    save(od / "report.json", rep)
    if step:
        pose = dict(xc=60.0, yf=(S.YF0 + S.YF1) / 2 - 60, sA=0.0, sB=120.0, sG=0.0, toolA="M300", toolB="S10")
        asm = cq.Assembly(name=f"lh1_machine_family_{name}")
        for it in items + gm:
            asm.add(it["solid"], name=it["name"].replace(":", "__").replace(" ", "_").replace("(", "").replace(")", "").replace(",", ""))
        for hn, v, off in head.parts(**pose):
            asm.add(v.translate(V(*off)), name="head__" + hn.replace(":", "__"))
        asm.save(str(od / f"machine_{name}_assembly.step"))
        for it in items:
            if it["name"].startswith(("DK-F-", "ENC-F-SL", "ENC-F-SR", "ENC-F-TOP", "ENC-F-FL", "TSS-F_tip_supply_shelf")):
                f = od / "dxf" / f"{it['name'].split(' (')[0]}.dxf"
                f.parent.mkdir(exist_ok=True)
                face = "<Z" if not it["name"].startswith(("ENC-F-SL", "ENC-F-SR")) else "<X"
                try:
                    cq.exporters.export(cq.Workplane().add(it["solid"]).faces(face), str(f))
                except Exception as e:  # noqa: BLE001
                    log("dxf failed", it["name"], e)
        log(name, "STEP + DXF written")
    return rep, rows, items, pl


def planner_block(rep, rows):
    """the rev_l key layout of assembly/out/report.json, for this size"""
    hr = json.loads((G.LH1 / "head_l" / "out" / "report.json").read_text())
    i = rep["interference"]
    S = G.Size(rep["size"])
    return {
        "size": rep["size"], "status": rep["status"] + "; ADR-011 size family (machine_family)",
        "interference": {"head_configurations_checked": i["head_configs_checked"], "real_interferences": i["count_real"], "by_kind": i["by_kind"],
                         "static_all_pairs (station vs station and vs everything)": i["static_all_pairs"]["count"],
                         "gantry_vs_static": len(i["gantry_vs_static"]), "head_vs_gantry": len(i["head_vs_gantry"]),
                         "reach_failures": len(rep["reach_failures"]), "reach_failures_by_design": rep["reach_failures_by_design"],
                         "reach_limits (tool body meets the target labware before the floor)": rep["reach_limits_body_in_target_labware"], "method": i["method"]},
        "deck_positions": rep["deck_positions"], "tube_positions": rep["tube_positions"], "pipettes": rep["pipettes"],
        "dock": {"bays": [{"bay": k + 1, "x": S.DOCK_X[k], "y": S.dock_y, "first_build_tool": M.DOCK_TOOLS[k],
                           "marker": "DMK-F tag + 2 holes in front of the drip tray (head camera)",
                           "note": "cable bay: reserved route + retractor space" if k == 7 else ("S100 later" if k == 6 else "")} for k in range(8)],
                 "design": hr["dock_bay"], "seat_stroke_s": P.L_DOCK_S, "clearance": rep["dock_clearance"],
                 "printer_y_over_the_bays (dock_yf)": round(S.dock_y - AY, 1), "dock_zone_yf": round(S.dock_y - AY - 94.5, 1),
                 "entry_rule": "enter and leave the dock row at its right end (xc >= 465), no tips on either carriage"},
        "tool_carriages": {"axes": P.L_AXIS_X, "nozzle_dy_from_beam_front_face": AY, "gripper_dy": G.GRIP_Y, "coupling": hr["coupling"]["geometry"],
                           "head_mass_kg": rep["dynamics"]["head_kg"], "family_head_changes": "LGA35 pusher (V6), LTG-1 latch (V1), vee seats + tooling balls (V18), retracted switches, sealed switches, Hall "
                           "switches, both gripper switches, FFC loops, eject line at x -26, X chain end bracket"},
        "exchange_time": rep["exchange_time"], "stiffness": rep["stiffness"], "dynamics": rep["dynamics"], "z_clearance": rep["z_clearance"],
        "travel": rep["travel"], "headroom": rep["headroom"], "frame": rep["geometry"], "envelope_mm": rep["envelope_mm"], "mass_kg": rep["mass_kg"],
        "tip_supply": rep["tip_supply"], "shelf": rep["shelf_plan"], "deck_rules": rep["deck_rules"],
        "tip_parking": rep["tip_parking"], "keepouts": rep["keepouts"], "services_interface": rep["services_interface"],
        "reservoir_F2": rep.get("reservoir_F2"), "cold_places": rep.get("cold_places"), "installation": rep.get("installation"),
        "mechanism_audit": rep.get("mechanism_audit"), "published_changes_23nov": rep.get("published_changes_23nov"),
        "dock_lift": rep.get("dock_lift"), "shelf_slots": rep.get("shelf_slots"),
        "syringe_tool": rep["syringe_tool"],
        "planner_rules": ["no tips on EITHER carriage while the head is over the dock row", "the dock row is entered and left at s 0 on all three axes",
                          "column 0 places are pipette-only (no gripper)", "S/M: p1000 tips from the shelf rack at IG on carriage B only",
                          "single-channel 10-100 uL does not reach the bottom of 15/50 mL tubes (CTB-2): use the 100-1,000 uL tool",
                          "the shaker plate sits 8 mm outward of the place centre (gripper target x +8)"] +
                         (["tip parking at B0 (TPK-S, every size): RANDOM ACCESS, M300 / p300 only, 6 spots in 3 groups (enter at the group's "
                           "entry, slide 8 mm to a spot, drop 1 mm into its pocket, strip); any order; removal tips of repeated wash series only"] if rep["tip_parking"].get("station") == "TPKS" else
                          ["tip parking at %s (TPK-1, 13 p300 sets, 104 tips)" % rep["tip_parking"]["place"]] if rep["tip_parking"].get("present") else []),
        "reports": {"size": f"cad/lh1/machine_family/out/{rep['size']}/report.json", "head": "cad/lh1/head_l/out/report.json"}}


def write_assembly_keys(blocks):
    ar = json.loads((AOUT / "report.json").read_text())
    before = {k: json.dumps(ar.get(k), sort_keys=True, default=str) for k in ("rev_h", "rev_k", "rev_l")}
    for nm, b in blocks.items():
        ar[f"family_{nm}"] = b
    assert all(json.dumps(ar.get(k), sort_keys=True, default=str) == v for k, v in before.items())
    (AOUT / "report.json").write_text(json.dumps(ar, indent=1, default=str))


def main(sizes=("S", "M", "L"), step_all=False):
    OUT.mkdir(parents=True, exist_ok=True)
    head = G.FamilyHead()
    hic = G.head_internal_check(head)
    save(OUT / "head_internal_check.json", hic)
    log("head internal", hic["poses"], len(hic["hits"]))
    rl = CK.rev_l_station_check()
    save(OUT / "rev_L_station_check.json", rl)
    log("rev L all-pairs", rl["count"], rl["by_kind"])
    reps, rowsd, blocks = {}, {}, {}
    for nm in sizes:
        rep, rows, items, pl = run_size(nm, head, step=(nm == "S" or step_all))
        reps[nm], rowsd[nm] = rep, rows
        blocks[nm] = planner_block(rep, rows)
        save(OUT / nm / f"family_{nm}.json", blocks[nm])
        log(nm, "done: real", rep["interference"]["count_real"], "static", rep["interference"]["static_all_pairs"]["count"])
    write_assembly_keys(blocks)
    if "S" in reps:
        save(OUT / "S" / "proposed_build_parts_S.yaml.json", build_steps(rowsd["S"]))
        (OUT / "S" / "proposed_build_parts_S.yaml").write_text(yaml.safe_dump({"source": "generated by cad/lh1/machine_family/build.py from parts_S.csv and head_l_parts.csv",
                                                                               "steps": build_steps(rowsd["S"])}, sort_keys=False, width=150))
        save(OUT / "S" / "parts_S_vs_rev_L.json", vs_rev_l(rowsd["S"]))
        save(OUT / "cr_answers.json", cr_answers(reps["S"]))
    kits = {}
    for a, b in (("S", "M"), ("M", "L")):
        if a in rowsd and b in rowsd:
            k = kit(rowsd[a], rowsd[b], G.Size(a).layout, G.Size(b).layout, a, b)
            kits[f"{a}_to_{b}"] = k
            flat = [dict(action=act, **{kk: (json.dumps(vv) if isinstance(vv, list) else vv) for kk, vv in x.items()})
                    for act in ("added", "replaced", "extended_in_place", "removed") for x in k[act]]
            if flat:
                keys = sorted({kk for r in flat for kk in r})
                write_csv(OUT / f"kit_{a}_to_{b}.csv", [{kk: r.get(kk, "") for kk in keys} for r in flat])
    save(OUT / "kits.json", kits)
    summ = {nm: {"interferences": r["interference"]["count_real"], "configs": r["interference"]["head_configs_checked"],
                 "static_all_pairs_overlaps": r["interference"]["static_all_pairs"]["count"], "rules_pass": r["deck_rules"]["pass"],
                 "reach_failures": len(r["reach_failures"]), "envelope_with_fan": r["envelope_mm"]["with_fan"], "envelope_no_fan": r["envelope_mm"]["without_fan"],
                 "mass_kg_ESTIMATE": r["mass_kg"]["total_kg_ESTIMATE"], "M1_120N": r["stiffness"]["M1_120N"], "M1_150N": r["stiffness"]["M1_150N"],
                 "exchange_s": r["exchange_time"]["exchange_s"], "parts_lines": r["parts_lines"]} for nm, r in reps.items()}
    summ["rev_L_all_pairs_overlaps"] = rl["count"]
    summ["head_internal_hits"] = len(hic["hits"])
    save(OUT / "family_summary.json", summ)
    log("all done")
    return summ


if __name__ == "__main__":
    args = [a for a in sys.argv[1:] if not a.startswith("--")]
    if "MVP" in args:                 # MVP build variant (stage 0 of S): machine_family/mvp.py, writes out/MVP only; S, M, L untouched
        import mvp
        rep_mvp = mvp.run(sys.modules[__name__])
        print(json.dumps({k: rep_mvp[k] for k in ("parts_summary", "envelope_mm", "deck_rules")} | {"interference": rep_mvp["interference"]["count_real"]},
                         indent=1, default=str))
        args = [a for a in args if a != "MVP"]
        if not args:
            sys.exit(0)
    print(json.dumps(main(tuple(args) if args else ("S", "M", "L"), step_all="--step-all" in sys.argv), indent=1, default=str))
