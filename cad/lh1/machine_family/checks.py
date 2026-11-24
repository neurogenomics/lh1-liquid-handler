"""LH-1 machine family: interference and reach checks for any size (machine_l's engine, with the targets keyed by station type instead
of revision L place names), the all-pairs static check (station against station and against frame, shelf, dock, drawer, panels) and
the same static check on the committed revision L scene.

Method as machine_l: bounding-box prefilter, OCC boolean common > 1 mm3; designed contacts named.  CALCULATED; nothing measured.
"""
from __future__ import annotations

import math

import numpy as np

from geometry import AY, DKK, GRIP_Y, HL, M, P, SEAT, TPKS, ZB, V

PIPETTES, EIGHT, SINGLE = M.PIPETTES, M.EIGHT, M.SINGLE
FRAME_TIP = {"P10": "p10", "P200": "p200", "P300": "p300", "P1000R": "p1000"}
PLATE_FLOOR = {"WN1": SEAT + 2.5, "MB1": SEAT + 1.0, "MAG2": SEAT + 1.0, "CPB1": SEAT + 15.0, "STRIP": SEAT + 23.5 - 15.0,
               "TC1": P.TC1_PLATE_TOP_Z - 15.0, "HAZ": SEAT + 3.0, "RES1": SEAT + 3.0, "RES2": SEAT + 3.0,
               "PLATE96": SEAT + 1.0}       # MVP variant only (flat-bottom 96-well plate on a plain insert; well floor ASSUMED 1 mm above the skirt)
TUBE_BOTTOM = {"ATR3": SEAT + 48.0 - 39.0, "CTB1": SEAT + 82.0 - 39.0, "CTB2": SEAT + 100.0 - 115.0, "ATR1": SEAT + 100.0 - 115.0, "ATR2": SEAT + 50.0 - 57.0}
TUBE_TOOLS = {"ATR3": SINGLE, "CTB1": SINGLE, "ATR2": SINGLE, "CTB2": ("S100", "S1000"), "ATR1": ("S100", "S1000")}
TROUGHS = ("HAZ", "RES1", "RES2")


TPKS_REPICK_OVER = 1.0      # mm past s_seat: the force-ended re-pick's over-travel (as at a tip rack)


def pipette_targets(S, pl):
    out = []
    for n, p in pl.items():
        k, (cx, cy), w = p["key"], p["c"], p["work"]
        if w == "deep":
            for t in PIPETTES:
                pts = [(cx - 49.5, cy), (cx + 49.5, cy)] if HL.TOOLS[t]["eight"] else [(cx - 49.5, cy + 31.5), (cx + 49.5, cy - 31.5)]
                out.append((n, t, pts, M.s_for_tip_end(t, PLATE_FLOOR[k] + 0.5), True, "into wells to the floor",
                            ("nozzles", "tips", "pipette") if k in TROUGHS else ("nozzles", "tips")))
            out.append((n, "SYR", [(cx - 49.5, cy), (cx + 49.5, cy)], M.nozzle_top("SYR") - (SEAT + p["h"] + 3.0), False, "jet 3 mm above the labware", ()))
        elif w == "deep_single_only":
            for t in TUBE_TOOLS[k]:
                pts = [(cx - 45.0, cy - 28.0), (cx + 45.0, cy + 28.0)]
                zb = (SEAT + 50.0 - 39.0) if (k == "ATR2" and t == "S10") else TUBE_BOTTOM[k]
                out.append((n, t, pts, M.s_for_tip_end(t, zb + 0.5), True, "into tubes to the bottom", ("nozzles", "tips", "pipette")))
        elif k == "TIPWASTE":
            from scene import TIP_GATE as TG_
            gx, gy = cx + TG_["gate_x_local"], cy + TG_["y_local"]
            scan = [(gx, gy - 40.0), (gx, gy + 40.0)]                 # the row centre's scan across the beam (+/- 31.5 + steps)
            for t in PIPETTES:
                out.append((n, t, scan, M.s_for_tip_end(t, TG_["z"]), True, "tip check: tip ends at the gate beam", ()))
                out.append((n, t, scan, M.nozzle_top(t) - (TG_["z"] + 7.5), False, "nozzle check: bare nozzles 7.5 mm above the beam", ()))
                out.append((n, t, [(cx, cy)], M.s_for_tip_end(t, SEAT + 10.0), True, "tip eject over the funnel (tip end z 15)", ()))
        elif k in FRAME_TIP:
            for t in PIPETTES:
                if HL.TOOLS[t]["tip"] != FRAME_TIP[k]:
                    continue
                top = SEAT + p["h"]
                pts = [(cx - 49.5, cy), (cx + 49.5, cy)] if HL.TOOLS[t]["eight"] else [(cx - 49.5, cy + 31.5), (cx + 49.5, cy + 31.5)]
                out.append((n, t, pts, M.nozzle_top(t) - (top - 8.0), False, "tip pick-up: bare nozzles 8 mm into the collars", ("nozzles", "pipette")))
        elif k == "TPK":
            zl1 = DKK.TPK["z_lower_top"]
            out.append((n, "M300", [(cx + DKK.TPK["x_spots"][0], cy), (cx + DKK.TPK["x_spots"][-1], cy)],
                        M.s_for_tip_end("M300", zl1 - P.TIP_BELOW_NOZZLE["p300"] + 1.0), True, "tip parking: collars into the TPK-1 slots", ("nozzles", "tips")))
        elif k == "TPKS":
            # integration 23 Nov 2026: three descents, the deepest last (by_place_and_tool keeps it): park entry at s_slide with tips,
            # park at s_seat with tips (collars on the pocket floor), and the force-ended RE-PICK with bare nozzles to s_seat + 1 mm (the
            # pick over-travel), which the firmware's depth cap at the parking place uses (was s_seat - 0.5: 0.5 mm above the seat)
            s_seat = M.s_for_tip_end("M300", TPKS["zl1"] - TPKS["pocket_depth"] - P.TIP_BELOW_NOZZLE["p300"])
            ents = [(cx + v, cy) for v in (TPKS["entries_local"][0], TPKS["entries_local"][-1])]
            spots = [(cx + v, cy) for v in (TPKS["spots_local"][0], TPKS["spots_local"][-1])]
            out.append((n, "M300", ents, s_seat - TPKS["pocket_depth"] - 0.3, True, "tip parking: entry, tips on, to s_slide", ("nozzles", "tips")))
            out.append((n, "M300", spots, s_seat, True, "tip parking: park, collars on the pocket floor (s_seat)", ("nozzles", "tips")))
            out.append((n, "M300", spots, s_seat + TPKS_REPICK_OVER, False,
                        "tip parking: re-pick, bare nozzles force-ended to s_seat + %g (M300 only)" % TPKS_REPICK_OVER, ("nozzles", "tips")))
        elif k == "BAL":
            px, py = cx, cy - 12.5
            for t in SINGLE:
                out.append((n, t, [(px, py)], M.s_for_tip_end(t, DKK.BAL_CELL["pan_z"] + 20.0), True, "dispense into a tube on the balance pan", ("nozzles", "tips")))
        elif w == "centre":
            for t in PIPETTES:
                out.append((n, t, [(cx - 29.0, cy), (cx + 29.0, cy)], M.s_for_tip_end(t, SEAT + 15.0), True, "wall dispense 10 mm below the splash wall", ()))
    if S.shelf_plan.get("IG", "").startswith("p1000"):           # S: p1000 rack on the shelf (carriage B only, by design)
        x, y = S.IG_RACK_X, S.SLOTS["IG"][1]
        top = S.SHELF["t"] - 15.0 + 72.0
        out.append(("IG", "S1000", [(x - 49.5, y + 31.5), (x + 49.5, y + 31.5)], M.nozzle_top("S1000") - (top - 8.0), False,
                    "tip pick-up from the shelf rack (carriage B only)", ("nozzles", "pipette")))
    return out


def gripper_targets(S, pl, zs_shaker, plan):
    t = {}
    for n, p in pl.items():
        k, (cx, cy) = p["key"], p["c"]
        h = SEAT + p["h"]
        if n.endswith("0"):
            continue                                              # column 0: pipette-only (N6)
        if k == "WN1":
            t[n] = ((cx, cy), SEAT + DKK.WN["gap"] + 2.0, "deep-well plate on WN-1")
        elif k in ("MB1", "CPB1", "MAG2", "STRIP"):
            t[n] = ((cx, cy), SEAT + 2.0, "plate at its skirt")
        elif k == "TC1":
            t[n] = ((cx, cy), P.TC1_PLATE_TOP_Z - 16.1 + 2.0, "plate in TC-1")
        elif k in ("STACK", "STACK2"):
            s_ = DKK.STACK["pcr_plate_full_skirt"]
            t[n] = ((cx, cy), SEAT + (s_["n"] - 1) * s_["pitch"] + s_["grip_above_unit_bottom"], "top plate of a stack of 4")
        elif k == "LIDPARK2":
            t[n] = ((cx, cy), SEAT + 47.5 - 9.5 + 3.0, "top lid of 5")
        elif k == "SHAKER":
            t[n] = ((cx + 8.0, cy), zs_shaker + 3.0, "plate on the shaker (lid off)")
        elif k in ("P10", "P200", "P300"):
            t[n] = ((cx, cy), h - DKK.WAFER_PLATE["t"] - 1.0, "refill wafer lips on the frame")
        elif k in ("CTB1", "CTB2", "ATR1", "ATR2", "ATR3"):
            t[n] = ((cx, cy), h + 9.5 - 6.5, "tube-rack lid TRL-1 on the rack")
    for slot, c in plan["slots"].items():
        x, y = S.SLOTS[slot]
        if c["kind"] == "handoff":
            t[c["name"]] = ((x, y), S.SHELF["t"] + 3.0 + 2.0, "plate at the edge hand-off")
        elif c["kind"].startswith(("tower", "spent")):
            t[slot] = ((x, y), S.SHELF["t"] + c["height_mm"] - DKK.WAFER_PLATE["t"] - 1.0, f"top wafer of {c['kind']}")
        elif c["kind"] == "lidpark":
            t[f"{slot}_lidpark"] = ((x, y), S.SHELF["t"] + c["height_mm"] - 9.5 + 3.0, "top lid on the shelf lid park")
    return t


def in_travel(S, xc, yf):
    return S.XC0 - 1e-6 <= xc <= S.XC1 + 1e-6 and S.YF0 - 1e-6 <= yf <= S.YF1 + 1e-6


def run_checks(S, scene, head, zs_shaker, pl, plan, log=print):
    res = {"transit": [], "lowered": [], "gripper": [], "dock": [], "eject": [], "reach_fail": [], "reach_limit_body_in_labware": [], "configs": 0}
    by_place_tool = {}
    ax_x = P.L_AXIS_X
    XC0, XC1, YF0, YF1 = S.XC0, S.XC1, S.YF0, S.YF1
    dock_yf = S.dock_y - AY

    def run(hp, label, skip=lambda hn, it: False):
        res["configs"] += 1
        return M.check_config(scene, head, hp, label, skip)
    pairs = [("M300", "S1000"), ("SYR", "M10"), ("M50", "S10")]
    pts = [(n, p["c"]) for n, p in pl.items()] + [(s_, S.SLOTS[s_]) for s_ in S.SLOTS]
    for n, (x, y) in pts:
        for ax in ("carriage_A", "carriage_B", "gripper"):
            xc, yf = x - ax_x[ax], y - (GRIP_Y if ax == "gripper" else AY)
            xc, yf = min(max(xc, XC0), XC1), min(max(yf, YF0), YF1)
            for tA, tB in pairs:
                tips = yf < dock_yf - 94.5
                res["transit"] += run(head.parts(xc, yf, toolA=tA, toolB=tB, tipsA=tips, tipsB=tips), f"transit {ax} over {n}, A {tA} B {tB} (tips {'on' if tips else 'off'})")
    for xc in (XC0, XC1):
        for yf in (YF0, YF1):
            for tA, tB in pairs:
                tips = yf < dock_yf - 94.5
                res["transit"] += run(head.parts(xc, yf, toolA=tA, toolB=tB, tipsA=tips, tipsB=tips), f"travel corner xc {xc:g} yf {yf:g}, A {tA} B {tB}")
    for off in (-154.5, -94.5, -54.5, -14.5):
        yf = dock_yf + off
        res["transit"] += run(head.parts(XC1, yf, toolA="M300", toolB="S1000", tipsA=False, tipsB=False), f"dock entry lane xc {XC1:g} yf {yf:g} (no tips)")
    res["transit"] += run(head.parts(XC1, YF1, toolA="M300", toolB="S1000", tipsA=False, tipsB=False), f"dock entry lane xc {XC1:g} yf {YF1:g} (no tips)")
    for xc in np.linspace(XC0, XC1, 12):
        res["transit"] += run(head.parts(float(xc), dock_yf, toolA="M300", toolB="S1000", tipsA=False, tipsB=False), f"along the dock row xc {xc:.0f} (no tips)")
    log("transit configs", res["configs"])
    reach_lim = {}
    for n, key, ptsxy, s_t, tips, what, excl in pipette_targets(S, pl):
        for ax in ("carriage_A", "carriage_B"):
            ok_all, worst = True, []
            for (x, y) in ptsxy:
                xc, yf = x - ax_x[ax], y - AY
                if not in_travel(S, xc, yf):
                    res["reach_fail"].append({"place": n, "tool": key, "carriage": ax, "point": [x, y], "xc": round(xc, 1), "yf": round(yf, 1)})
                    ok_all = False
                    continue
                s_use = min(s_t, P.L_Z_TRAVEL)
                if s_t > P.L_Z_TRAVEL + 1e-6:
                    res["reach_fail"].append({"place": n, "tool": key, "carriage": ax, "depth_short_mm": round(s_t - P.L_Z_TRAVEL, 1)})
                    ok_all = False
                for frac in (0.5, 0.85, 1.0):
                    s = s_use * frac
                    kw = dict(sA=s, sB=0.0, toolA=key, toolB="M300", tipsA=tips) if ax == "carriage_A" else dict(sA=0.0, sB=s, toolA="M300", toolB=key, tipsB=tips)

                    def skip(hn, it, n=n, ax=ax, excl=excl):
                        return (it.get("place") == n and (it["cls"] == "labware" or it.get("tool_contact")) and hn.startswith(ax + ":tool:")
                                and hn.split(":")[-1] in excl)
                    h = run(head.parts(xc, yf, **kw), f"{key} on {ax} lowered at {n} ({what}) s {s:.0f} at ({x:.0f}, {y:.0f})", skip)
                    lim = [q for q in h if q["place"] == n and q["cls"] == "labware" and q["head_part"].startswith(ax + ":tool:")]
                    h = [q for q in h if q not in lim]
                    res["reach_limit_body_in_labware"] += lim
                    res["lowered"] += h
                    worst += h
                    if lim:
                        reach_lim.setdefault((n, key), set()).add(round(frac, 2))
            rl = reach_lim.get((n, key))
            by_place_tool.setdefault(n, {})[f"{key}@{ax}"] = {"reached": ok_all and not rl, "interferences": len(worst), "s_target": round(s_t, 1), "what": what,
                                                              "reach_limit": (f"tool body meets the target labware at {min(rl) * 100:.0f} % of the stroke to the floor: "
                                                                              "cannot reach the floor") if rl else None,
                                                              "out_of_travel": not ok_all and not rl}
    log("pipette configs", res["configs"])
    grip = gripper_targets(S, pl, zs_shaker, plan)
    for n, ((x, y), zf, what) in grip.items():
        xc, yf = x - ax_x["gripper"], y - GRIP_Y
        s_t = M.grip_s(zf)
        if not in_travel(S, xc, yf) or s_t > P.L_Z_TRAVEL + 1e-6 or s_t < 0:
            res["reach_fail"].append({"place": n, "tool": "gripper", "xc": round(xc, 1), "yf": round(yf, 1), "s": round(s_t, 1)})
        nh = 0
        for frac in (0.5, 0.85, 1.0):
            h = run(head.parts(min(max(xc, XC0), XC1), min(max(yf, YF0), YF1), sG=min(max(s_t, 0), P.L_Z_TRAVEL) * frac, toolA="M300", toolB="S1000"),
                    f"gripper at {n} ({what}) s {s_t * frac:.0f}")
            res["gripper"] += h
            nh += len(h)
        by_place_tool.setdefault(n, {})["gripper"] = {"reached": in_travel(S, xc, yf) and 0 <= s_t <= P.L_Z_TRAVEL, "interferences": nh, "s_target": round(s_t, 1), "what": what}
    log("gripper configs", res["configs"])
    sd = P.L_DOCK_S
    for i in range(8):
        key = M.DOCK_TOOLS_FULL[i]
        bay = f"BAY{i + 1}"
        for ax in ("carriage_A", "carriage_B"):
            xc, yf = S.DOCK_X[i] - ax_x[ax], dock_yf
            if not in_travel(S, xc, yf):
                res["reach_fail"].append({"place": bay, "tool": f"{ax} exchange", "xc": round(xc, 1), "yf": round(yf, 1)})
                continue
            for mode in ("drop-off", "pick-up"):
                for s in (0.0, sd - 40.0, sd - 7.0, sd - 1.0):
                    tool = key if mode == "drop-off" else None
                    kw = dict(sA=s, sB=0.0, toolA=tool, toolB="M300", tipsA=False, tipsB=False) if ax == "carriage_A" else \
                        dict(sA=0.0, sB=s, toolA="M300", toolB=tool, tipsA=False, tipsB=False)

                    def skip(hn, it, bay=bay, ax=ax, mode=mode, s=s):
                        if it.get("place") != bay or not hn.startswith(ax):
                            return False
                        leaf = hn.split(":")[-1]
                        if mode == "drop-off" and it["group"] == "parked_tool":
                            return True
                        if mode == "pick-up" and it["group"] == "parked_tool" and leaf in ("vees", "bracket", "jaw", "cartridge", "pusher_rod", "pusher_collar") and \
                                it.get("leaf") in ("balls", "stud", "push_rod", "tool_plate", "magnets", "eject_pin", "slider") and s >= sd - 13.0:
                            return True
                        if mode == "pick-up" and it["group"] == "parked_tool" and leaf == "hall_board" and it.get("leaf") in ("magnets", "tool_plate") and s >= sd - 7.0:
                            return True                       # the Hall sensor tips 1 mm above the bracket underside meet the plate top only when seated
                        return False
                    res["dock"] += run(head.parts(xc, yf, **kw), f"dock {mode} {bay} by {ax} ({key}) s {s:.0f}", skip)
    log("dock configs", res["configs"])
    tw = [n for n, p in pl.items() if p["key"] == "TIPWASTE"][0]
    cx, cy = pl[tw]["c"]
    for key in PIPETTES:
        for ax in ("carriage_A", "carriage_B"):
            xc, yf = cx - ax_x[ax], cy - AY
            kw = dict(sA=-P.Z_EJECT_TRAVEL, toolA=key, toolB="M300") if ax == "carriage_A" else dict(sB=-P.Z_EJECT_TRAVEL, toolA="M300", toolB=key)
            res["eject"] += run(head.parts(xc, yf, **kw), f"eject over-travel {key} on {ax} at {tw}")
    log("eject configs", res["configs"])
    return res, by_place_tool


# ---------------------------------------------------------------------------------------------------------------------------------
# static checks
# ---------------------------------------------------------------------------------------------------------------------------------
DESIGNED_NAMES = [   # (substring a, substring b, why): contacts that are the design, not interferences
    ("YCH-F_y_chain_envelope", "YCB-F_y_chain_end_bracket", "chain moving end"),
    ("XCH-F_x_chain_envelope", "x_chain_end_bracket", "chain moving end"),
    ("MGN15H_Y_block", "MGN15_Y_rail", "blocks on the rail"),
]


def owner(it):
    return it.get("place") or it["name"]


def static_pairs(items, label=""):
    """every pair of items with different owners whose solids overlap > 1 mm3 (bounding-box prefilter)"""
    sc = M.Scene(items)
    hits = []
    n_bool = 0
    for i, a in enumerate(items):
        for j in sc.candidates(sc.bb[i]):
            if j <= i:
                continue
            b = items[j]
            if owner(a) == owner(b):
                continue
            if any((x in a["name"] and y in b["name"]) or (x in b["name"] and y in a["name"]) for x, y, _ in DESIGNED_NAMES):
                continue
            n_bool += 1
            v = M.overlap_volume(a["solid"], b["solid"])
            if v > 1.0 or v < 0:
                ga, gb = a["group"], b["group"]
                if ga == "station" and gb == "station":
                    kind = "station_vs_station"
                elif "station" in (ga, gb):
                    kind = "station_vs_" + (gb if ga == "station" else ga)
                elif ga == gb:
                    kind = f"within_{ga}"
                else:
                    kind = f"{min(ga, gb)}_vs_{max(ga, gb)}"
                hits.append({"kind": kind, "a": a["name"], "b": b["name"], "mm3": round(v, 1)})
    by = {}
    for h in hits:
        by[h["kind"]] = by.get(h["kind"], 0) + 1
    return {"pairs_tested_boolean": n_bool, "overlaps": hits, "by_kind": by, "count": len(hits),
            "method": "every placed item against every other with a different owner (place, slot, bay or part): stations, modules, racks and "
                      "labware against each other and against frame, deck, shelf, dock, drawer, panels and services; bounding-box prefilter, "
                      "OCC boolean common > 1 mm3; designed contacts excluded: " + "; ".join(f"{x} / {y} ({w})" for x, y, w in DESIGNED_NAMES)}


def rev_l_station_check():
    """the committed revision L scene (machine_l, unmodified) through the same all-pairs check"""
    plan = M.tip_supply_plan_l()
    static, zs = M.scene_static(plan)
    r = static_pairs(static, "rev L")
    r["note"] = ("revision L as committed (machine_l scene_static); machine_l's own static check tests only across groups, and all stations are "
                 "one group: station-to-station overlaps were never tested")
    return r


def gantry_checks(S, static, head, gantry_fn):
    g_hits = []
    sc = M.Scene(static)
    for yf in (S.YF0, (S.YF0 + S.YF1) / 2, S.YF1):
        for it in gantry_fn(S, yf):
            b = it["solid"].BoundingBox()
            for i in sc.candidates(np.array([b.xmin, b.xmax, b.ymin, b.ymax, b.zmin, b.zmax])):
                o = static[i]
                if any((x in it["name"] and y in o["name"]) or (x in o["name"] and y in it["name"]) for x, y, _ in DESIGNED_NAMES):
                    continue
                v = M.overlap_volume(it["solid"], o["solid"])
                if v > 1.0:
                    g_hits.append({"yf": yf, "a": it["name"], "b": o["name"], "mm3": round(v, 1)})
    gsc = M.Scene(gantry_fn(S, 0.0))
    hg = []
    for xc in (S.XC0, 0.0, S.XC1):
        for s in (0.0, P.L_Z_TRAVEL, -P.Z_EJECT_TRAVEL):
            hg += M.check_config(gsc, head, head.parts(xc, 0.0, sA=s, sB=s, sG=max(s, 0.0)), f"head vs gantry xc {xc:g} s {s:g}",
                                 skip=lambda hn, it: (hn.split(":")[-1].startswith("xblock") and "X_rail" in it["name"]) or
                                 (hn.endswith("x_chain_end_bracket") and "x_chain_envelope" in it["name"]))
    return g_hits, hg


# ---------------------------------------------------------------------------------------------------------------------------------
# V5: TPK-S tolerance stack (every capture and clearance of the park / re-pick motion against the error sources)
# ---------------------------------------------------------------------------------------------------------------------------------
COMB_ERRORS_ASSUMED = {   # mm, per side (worst case); sigma = worst / 3 except where noted
    "eight-channel nozzle scatter, residual after cutting the plates to the measured row (flatbed scan at 1200 dpi, ASSUMED)": 0.05,
    "coupling repeatability at the nozzle (CT-1 pass value, 3 sigma)": 0.10,
    "Y belt after the 1.04 s settle dwell (dynamics settle target)": 0.05,
    "X/Y closed-loop positioning repeatability (ASSUMED)": 0.02,
    "thermal (20 K on the Al deck over 300 mm vs steel screws, partly calibrated, ASSUMED)": 0.03,
    "plate profile tolerance, CNC (ASSUMED)": 0.03,
    "upper-to-lower plate alignment on the four posts (ASSUMED)": 0.05}


def comb_tolerance(stiff_h_120, belt_stretch_2ms2, slide_acc=0.25, nozzle_end_od=3.8, tip_bore=None):
    """stiff_h_120: M1 horizontal deflection at 120 N (size report); belt_stretch_2ms2: Y belt static stretch at 2 m/s2 (EA 3e4) from the
    dynamics report (V5's 0.29 mm figure).  The comb location and each tool's nozzle offset are calibrated at commissioning (head camera
    over the comb fiducial), so common-mode errors are out; per exchange and per channel errors stay."""
    tip = TPKS["tip"]
    col, noz = P.TIP_COLLAR_OD[tip], P.TIP_NOZZLE_OD_AT_COLLAR[tip]
    tip_bore = tip_bore or noz
    e = dict(COMB_ERRORS_ASSUMED)
    e["belt stretch during the slide at %.1f m/s2 (V5: %.2f mm at 2 m/s2, scaled)" % (slide_acc, belt_stretch_2ms2)] = round(belt_stretch_2ms2 * slide_acc / 2.0, 3)
    e["beam deflection under the slide friction (8 collars x 1 N, ASSUMED; V5: %.2f mm at 120 N, scaled)" % stiff_h_120] = round(stiff_h_120 * 8.0 / 120.0, 3)
    worst = sum(e.values())
    rss = math.sqrt(sum((v / 3.0) ** 2 for v in e.values())) * 3.0
    cp = (TPKS["pocket_d"] - col) / 2
    checks = {
        "entry: tips through the upper entry hole (radius clearance + 1.0 chamfer)": (TPKS["entry_d"] / 2 - col / 2 + TPKS["chamfer_entry"], worst),
        "entry: tip body through the lower slot (body <= 5.0 mm below the collar, ASSUMED: MEASURE)": ((TPKS["ws"] - 5.0) / 2, worst),
        "slide: nozzle in the upper slot": ((TPKS["wn"] - noz) / 2, worst),
        "seat: collar into its pocket (clearance + 0.3 chamfer)": (cp + TPKS["chamfer_pocket"], worst),
        "seat: collar land on the pocket floor (overlap left after the pocket clearance; >= 0.25 needed)": ((col - TPKS["ws"]) / 2 - cp, 0.25),
        "strip: collar into the upper-plate counterbore (clearance + 0.3 chamfer)": (cp + 0.3, worst + stiff_h_120 * 0.1),
        "strip: collar overlap under the upper slot edge after the counterbore clearance and plate alignment (>= 0.3 needed)":
            ((col - TPKS["wn"]) / 2 - cp - e["upper-to-lower plate alignment on the four posts (ASSUMED)"], 0.30),
        "re-pick: nozzle end into the parked collar (bore %.1f vs nozzle end %.1f ASSUMED), collar free in its pocket" % (tip_bore, nozzle_end_od):
            ((tip_bore - nozzle_end_od) / 2, worst + cp)}
    rows, ok = {}, True
    for k, (cap, need) in checks.items():
        need_rss = rss if need in (worst, worst + stiff_h_120 * 0.1) else (math.hypot(rss, cp) if need == worst + cp else need)
        p_ = cap >= need and (cap >= 1.5 * need_rss if need_rss != need else True)
        ok &= p_
        rows[k] = {"capacity_mm": round(cap, 3), "worst_case_mm": round(need, 3), "rss_x1.5_mm": round(1.5 * need_rss, 3) if need_rss != need else None,
                   "margin_worst": round(cap / need, 2) if need else None, "pass": p_}
    unmeasured = dict(e, **{list(e)[0]: 0.30})
    w_u = sum(unmeasured.values())
    return {"error_sources_mm": e, "worst_case_sum_mm": round(worst, 3), "rss_3sigma_mm": round(rss, 3), "checks": rows, "pass": ok,
            "rule": "pass = capacity >= worst-case sum AND >= 1.5 x the RSS (3 sigma) sum",
            "if_plates_were_not_cut_to_the_measured_row (scatter +/-0.30, V5)": {
                "worst_case_sum_mm": round(w_u, 3), "slide_clearance_mm": round((TPKS["wn"] - noz) / 2, 3),
                "pass": (TPKS["wn"] - noz) / 2 >= w_u, "verdict": "fails: this is why the plates are cut to the measured nozzle row"},
            "why_not_+/-0.75 (V5's suggestion)": "the p300 collar (%.1f) is only %.1f mm wider than the nozzle (%.1f): a +/-0.75 slot would leave "
                                                 "no collar under the strip edge; the counterbore over each spot centres the collar for the "
                                                 "strip instead, so the slot can stay narrow" % (col, col - noz, noz)}
