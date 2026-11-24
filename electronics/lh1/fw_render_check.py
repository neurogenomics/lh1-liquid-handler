#!/usr/bin/env python3
"""Behavioural check of the guard macros in firmware/lh1/printer.cfg (+ its size file) and bench_rig.cfg.

Thin caller of the klipper-guardcheck package (packages/klipper-guardcheck): the cases are data in
electronics/lh1/guard_cases.yaml.  Each guard macro's Jinja template is rendered the way Klipper's gcode_macro does
against a mocked `printer` status object, and the case asserts BEHAVIOUR: the macro refuses (action_raise_error) in
each bad state and, in the good state, emits the expected base command.  It does not run Klipper: called macros are
not expanded (each guard is tested on its own).

Size family (ADR-011): printer.cfg is the shared core and includes one size file.  The package does not follow
[include], so fw_config.py first writes what Klipper reads for each size (electronics/lh1/out/firmware_resolved_<size>.cfg).
Runs:
  1. guard_cases.yaml against the size being built (the core's include line: S) -> the headline n / failed;
  2. the same machine cases against the M and L size files (config override);
  3. generated GEOMETRY PIN cases per size (out/guard_cases_sizes.yaml), built from
     cad/lh1/machine_family/out/<size>/family_<size>.json: dock row, dock zone and exit, Y travel end, nozzle offset, every
     CAD module place's SAFE line and the carriage-only places.  The expected numbers come from the CAD report at run
     time, never from the firmware.
Prints one JSON object {"jinja2", "n", "failed", "cases", "by_size": {...}}; exit 1 if any case of any run fails.
Needs jinja2 and the package (uv pip install -e packages/klipper-guardcheck).  Run by electronics/lh1/check.py.
"""
import copy
import re
import json
import sys
from pathlib import Path

import yaml
sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "tools" / "vendor"))  # vendored helper packages
from klipper_guardcheck import run_case_file

import fw_config

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[1]
MODULE_STATIONS = {"CTB1": "a3", "CTB2": "a3", "STRIP": "b2", "CPB1": "g4", "HAZ": "g0", "BAL": "d3", "WN1": "d4"}


def family(size):
    return json.loads((ROOT / f"cad/lh1/machine_family/out/{size}/family_{size}.json").read_text())


def tip_parking_cases(s, d, ax, ndy):
    """tip parking per size, expected values from the CAD tip_parking block (place, reach, tools admitted, random-access
    groups, s values and speeds).  ADR-014 #6: S is random access (TPK-S groups); M and L publish no spot geometry."""
    import gen_size_files
    tp = d.get("tip_parking") or {}
    if not tp.get("present"):
        return []
    p = tp["place"]
    x, y = d["deck_positions"][p]["centre_xy"]
    reach = next(v for k, v in tp.items() if k.startswith("reach"))
    reached = sorted({k.split("@")[0] for k, v in reach.items() if v.get("reached") and v.get("interferences") == 0})
    tools = sorted(set(tp.get("tools_admitted") or reached) & set(reached))
    other = next(t for t in ("M50", "M10", "S100") if t not in tools)
    t0 = tools[0]
    pg = {"X": x, "Y": y, "LINE": "a", "WHO": "t", "C": "a"}
    st = lambda **k: {"gcode_macro _LH_STATE": k}
    cs = [
        {"macro": "_LH_PLACE_GUARD", "params": pg, "state": f"{s}: CAD tip parking {p} ({tp['station']}), carriage A with {t0}", "set": st(mounted_a=t0, tips_a=1), "expect": "accept"},
        {"macro": "_LH_PLACE_GUARD", "params": pg, "state": f"{s}: CAD tip parking {p}, carriage A with {other} (not admitted by the CAD)", "set": st(mounted_a=other), "expect": "refuse", "message": "TIP_PARKING"},
        {"macro": "_LH_PLACE_GUARD", "params": {"X": x, "Y": y, "LINE": "b", "WHO": "G0"}, "state": f"{s}: gripper descending over the tip parking {p}", "expect": "refuse", "message": "TIP_PARKING"},
        {"macro": "_LH_PLACE_GUARD", "params": {"X": x, "Y": y, "LINE": "b", "WHO": "G0", "DOWN": 0}, "state": f"{s}: gripper raising over the tip parking {p}", "expect": "accept"},
        {"macro": "LH_SIZE_STATUS", "state": f"{s}: tip parking reported", "expect": "accept", "emits": [f"TIP_PARKING={p.lower()}:{tp['station']}:"]},
    ]
    geo = gen_size_files.tip_parking(d)[p.lower()][8]
    n = tp["capacity_sets"]
    pk = {"CARRIAGE": "A", "TOOL": t0, "WELLS": "s4:c1"}
    def arm(spots, **k):
        return {"gcode_macro LH_TIP_PARK": {"armed": True, "strip_force_measured_n": 120.0, "parked": sum(1 for e in spots if e), "spots": spots},
                "gcode_macro _LH_STATE": dict({"mounted_a": t0, "tips_a": 1}, **k)}
    cs.append({"macro": "LH_TIP_PARK_SET", "params": {"CONFIRM": 1, "SETS": n + 1}, "state": f"{s}: person declares more sets than the comb holds", "expect": "refuse"})
    cs.append({"macro": "LH_TIP_PARK_SET", "params": {"CONFIRM": 1, "SETS": 0}, "state": f"{s}: person empties the comb: every spot free", "expect": "accept",
               "emits": ["VARIABLE=parked VALUE=0", "VARIABLE=spots VALUE='[" + ",".join(['""'] * (len(geo["spots"]) if geo else n)) + "]'"]})
    if geo is None:
        cs.append({"macro": "LH_TIP_PARK", "params": pk, "state": f"{s}: no random-access spot geometry published for {tp['station']}", "set": arm([""] * n), "expect": "refuse", "message": "NO_GEOMETRY"})
        cs.append({"macro": "LH_TIP_UNPARK", "params": pk, "state": f"{s}: re-pick refused without spot geometry", "set": arm([""] * n, tips_a=0), "expect": "refuse", "message": "NO_GEOMETRY"})
        return cs
    sp = geo["spots"]
    m = len(sp)
    yy = round(geo["row_y"] - ndy, 2)
    free = [""] * m
    own = f"{t0}|s4:c1"
    full = [f"{t0}|w{i}" for i in range(m)]
    s3 = list(free); s3[2] = own                    # spot 3 holds the set of t0 for s4:c1
    s1 = list(free); s1[0] = f"{t0}|s7:c2"
    cs += [
        {"macro": "LH_TIP_PARK", "params": pk, "state": f"{s}: tip parking not commissioned", "set": {"gcode_macro _LH_STATE": {"mounted_a": t0, "tips_a": 1}, "gcode_macro LH_TIP_PARK": {"parked": 0, "spots": free}}, "expect": "refuse", "message": "NOT COMMISSIONED"},
        {"macro": "LH_TIP_PARK", "params": pk, "state": f"{s}: parking unknown after a restart", "set": {"gcode_macro LH_TIP_PARK": {"armed": True, "parked": -1, "spots": []}, "gcode_macro _LH_STATE": {"mounted_a": t0, "tips_a": 1}}, "expect": "refuse", "message": "PARKING_UNKNOWN"},
        {"macro": "LH_TIP_PARK", "params": pk, "state": f"{s}: no tips on the carriage", "set": arm(free, tips_a=0), "expect": "refuse", "message": "no tips"},
        {"macro": "LH_TIP_PARK", "params": pk, "state": f"{s}: unknown tip state (ejected / parked / power-up, not observed)", "set": arm(free, tips_a=-1), "expect": "refuse", "message": "TIPS_UNKNOWN"},
        {"macro": "LH_TIP_PARK", "params": {"CARRIAGE": "A", "TOOL": other, "WELLS": "s4:c1"}, "state": f"{s}: wrong tool: {other} is not admitted (CAD tool list {tools})", "set": arm(free, mounted_a=other), "expect": "refuse", "message": "TIP_PARKING"},
        {"macro": "LH_TIP_PARK", "params": pk, "state": f"{s}: wrong tool: the carriage carries another tool than the command names", "set": arm(free, mounted_a=other), "expect": "refuse", "message": "WRONG_TOOL"},
        {"macro": "LH_TIP_PARK", "params": {"CARRIAGE": "A", "TOOL": t0}, "state": f"{s}: no WELLS label (A4: a parked set carries tool and wells)", "set": arm(free), "expect": "refuse", "message": "WELLS"},
        {"macro": "LH_TIP_PARK", "params": dict(pk, SPOT=3), "state": f"{s}: occupied spot named", "set": arm(s3), "expect": "refuse", "message": "SPOT_OCCUPIED"},
        {"macro": "LH_TIP_PARK", "params": dict(pk, SPOT=m + 1), "state": f"{s}: spot out of range (the CAD has {m})", "set": arm(free), "expect": "refuse"},
        {"macro": "LH_TIP_PARK", "params": pk, "state": f"{s}: comb full ({m} spots, CAD capacity {tp['capacity_tips']} tips)", "set": arm(full), "expect": "refuse", "message": "PARKING_FULL"},
        {"macro": "LH_TIP_PARK", "params": pk, "state": f"{s}: strip force not measured yet", "set": dict(arm(free), **{"gcode_macro LH_TIP_PARK": {"armed": True, "parked": 0, "spots": free, "strip_force_measured_n": -1.0}}), "expect": "refuse", "message": "STRIP_FORCE_UNMEASURED"},
        {"macro": "LH_TIP_PARK", "params": pk, "state": f"{s}: measured strip force above the coupling limit", "set": dict(arm(free), **{"gcode_macro LH_TIP_PARK": {"armed": True, "parked": 0, "spots": free, "strip_force_measured_n": 200.0}}), "expect": "refuse", "message": "STRIP_FORCE_HIGH"},
        {"macro": "LH_TIP_PARK", "params": pk, "state": f"{s}: first free spot = spot 1: CAD entry x {sp[0][1]}, slide to x {sp[0][2]}, s_above {geo['s_above']}, s_slide {geo['s_slide']}, seat {geo['s_seat']}, strip {geo['strip_mm']} mm at the force current",
         "set": arm(free), "expect": "accept",
         "emits": [f"G0 X{round(sp[0][1] - ax, 2)} Y{yy}", f"_LH_CARRIAGE_Z C=a Z={geo['s_above']}", f"_LH_CARRIAGE_Z C=a Z={geo['s_slide']} SPEED={geo['z_mm_s']}", "_LH_XY_GUARD C=a",
                   f"SET_VELOCITY_LIMIT_BASE ACCEL={geo['slide_accel']}", f"G0.1 X{round(sp[0][2] - ax, 2)} F{geo['slide_mm_s'] * 60}", f"_LH_CARRIAGE_Z C=a Z={geo['s_seat']} SPEED={geo['z_mm_s']}",
                   "_LH_Z_FORCE C=a TOOL=M300 ON=1", f"_LH_CARRIAGE_Z C=a Z={round(geo['s_seat'] - geo['strip_mm'], 2)} SPEED={geo['strip_mm_s']}", "_LH_Z_FORCE C=a TOOL=M300 ON=0",
                   "VARIABLE=spots VALUE='[\"" + own + "\"," + ",".join(['""'] * (m - 1)) + "]'", "VARIABLE=parked VALUE=1", "VARIABLE=tips_a VALUE=-1", "RESULT=PARKED SPOT=1"],
         "emits_regex": [f"ACCEL={geo['slide_accel']}[^\\n]*\\n\\s*G0.1[^\\n]*\\n\\s*SET_VELOCITY_LIMIT_BASE ACCEL=2000"]},
        {"macro": "LH_TIP_PARK", "params": dict(pk, SPOT=m), "state": f"{s}: random access: the LAST spot named while spot 1 is occupied: its own group's entry x {sp[-1][1]}, slide to {sp[-1][2]}",
         "set": arm(s1), "expect": "accept", "emits": [f"G0 X{round(sp[-1][1] - ax, 2)} Y{yy}", f"G0.1 X{round(sp[-1][2] - ax, 2)}", "VARIABLE=parked VALUE=2", f"RESULT=PARKED SPOT={m}"]},
        {"macro": "LH_TIP_UNPARK", "params": pk, "state": f"{s}: re-pick the set owned by {t0} / s4:c1 (spot 3, any order): spot x {sp[2][2]}, force-ended pick-up to s {geo['s_seat'] + geo['repick_over']}, up to s_slide, slide to the group's entry x {sp[2][1]}",
         "set": arm(s3, tips_a=0), "expect": "accept",
         "emits": [f"G0 X{round(sp[2][2] - ax, 2)} Y{yy}", f"LH_PICK_TIP CARRIAGE=A TOOL={t0} Z_START={round(geo['s_seat'] + geo['repick_over'] - 6.0, 2)} Z_MAX={round(geo['s_seat'] + geo['repick_over'], 2)}",
                   f"_LH_CARRIAGE_Z C=a Z={geo['s_slide']}", f"G0.1 X{round(sp[2][1] - ax, 2)}", "VARIABLE=parked VALUE=0", "RESULT=REPICKED SPOT=3"]},
        {"macro": "LH_TIP_UNPARK", "params": dict(pk, SPOT=1), "state": f"{s}: empty spot named", "set": arm(s3, tips_a=0), "expect": "refuse", "message": "SPOT_EMPTY"},
        {"macro": "LH_TIP_UNPARK", "params": dict(pk, SPOT=1), "state": f"{s}: spot owned by other wells", "set": arm(s1, tips_a=0), "expect": "refuse", "message": "SET_OWNER"},
        {"macro": "LH_TIP_UNPARK", "params": pk, "state": f"{s}: no set owned by this tool and wells", "set": arm(s1, tips_a=0), "expect": "refuse", "message": "SET_OWNER"},
        {"macro": "LH_TIP_UNPARK", "params": pk, "state": f"{s}: re-pick with tips on", "set": arm(s3, tips_a=1), "expect": "refuse", "message": "tips already on"},
        {"macro": "LH_TIP_UNPARK", "params": pk, "state": f"{s}: re-pick with the tip state unknown", "set": arm(s3, tips_a=-1), "expect": "refuse", "message": "TIPS_UNKNOWN"},
        {"macro": "LH_TIP_UNPARK", "params": {"CARRIAGE": "A", "TOOL": other, "WELLS": "s4:c1"}, "state": f"{s}: re-pick with a tool the comb does not admit", "set": arm(s3, tips_a=0, mounted_a=other), "expect": "refuse", "message": "TIP_PARKING"},
        {"macro": "LH_TIP_UNPARK", "params": pk, "state": f"{s}: re-pick when parking is unknown after a restart", "set": {"gcode_macro LH_TIP_PARK": {"armed": True, "parked": -1, "spots": []}, "gcode_macro _LH_STATE": {"mounted_a": t0, "tips_a": 0}}, "expect": "refuse", "message": "PARKING_UNKNOWN"},
        {"macro": "LH_TIP_PARK_STATUS", "state": f"{s}: status lists each spot's owner", "set": arm(s3), "expect": "accept", "emits": [f"LH_TIP_PARK KNOWN=1 PARKED=1 SPOTS=1:EMPTY,2:EMPTY,3:{own}"]},
    ]
    return cs


def fw_config_size_var(s, k):
    import configparser, ast
    cp = configparser.RawConfigParser(strict=False, delimiters=(":",), comment_prefixes=("#",), inline_comment_prefixes=("#",))
    cp.read_string(fw_config.out_path(s).read_text())
    return ast.literal_eval(cp.get("gcode_macro _LH_SIZE", f"variable_{k}"))


def size_cases(base_status, base_vars):
    """geometry pins for every size, expected values read from the CAD family report"""
    cf = {"configs": {}, "printers": {}, "cases": []}
    for s in fw_config.SIZES:
        cf["configs"][s] = fw_config.out_path(s).name
        d = family(s)
        st = copy.deepcopy(base_status)
        mv = copy.deepcopy(base_vars)
        mv["_LH_SIZE"] = {}
        for p in ("d3", "d4"):
            for ln in ("a", "b"):
                st.setdefault(f"gcode_button safe_{p}_{ln}", {"state": "PRESSED"})
        cf["printers"][f"m{s}"] = {"config": s, "status": st, "macro_variables": mv}
        dock = d["dock"]["printer_y_over_the_bays (dock_yf)"]
        zone = d["dock"]["dock_zone_yf"]
        ymax = d["travel"]["Y"]["travel_yf"][1]
        ndy = d["tool_carriages"]["nozzle_dy_from_beam_front_face"]
        bx = d["tool_carriages"]["axes"]["carriage_B"]
        ax = d["tool_carriages"]["axes"]["carriage_A"]
        gx = d["tool_carriages"]["axes"]["gripper"]
        exit_ = round(dock - 124.5, 1)
        gdy = d["tool_carriages"]["gripper_dy"]
        win = fw_config_size_var(s, "shelf_window_x")
        okx, oky = d["frame"]["shelf"]["columns_x"]["outer"], d["frame"]["dock_axis_y"]
        cs = [
            {"macro": "LH_SIZE_STATUS", "state": f"{s}: values = CAD family_{s}", "expect": "accept",
             "emits": [f"SIZE={s} Y_MAX={ymax:.1f} DOCK_YF={dock:.1f} DOCK_ZONE_YF={zone:.1f} DOCK_EXIT_YF={exit_:.1f}"]},
            {"macro": "_LH_DOCK_ENTER", "state": f"{s}: dock row from CAD", "expect": "accept", "emits": ["G0.1 X465.0", f"G0.1 Y{dock}"]},
            {"macro": "LH_DOCK_LEAVE", "state": f"{s}: dock exit", "expect": "accept", "emits": [f"G0.1 Y{exit_}"]},
            {"macro": "_LH_GMOVE", "params": {"CMD": "G0", "Y": zone}, "state": f"{s}: target at the CAD dock zone", "expect": "refuse"},
            {"macro": "_LH_GMOVE", "params": {"CMD": "G0", "Y": zone - 1.0}, "state": f"{s}: target 1 mm in front of the CAD dock zone", "expect": "accept"},
            {"macro": "_LH_GMOVE", "params": {"CMD": "G0", "X": 0}, "state": f"{s}: head inside the CAD dock zone", "set": {"toolhead": {"position": {"y": zone + 1.0}}}, "expect": "refuse"},
            {"macro": "_LH_ZMOVE", "params": {"STEPPER": "z_a", "MOVE": 10, "SPEED": 10, "ACCEL": 300}, "state": f"{s}: lowering over the CAD dock zone", "set": {"toolhead": {"position": {"y": zone}}}, "expect": "refuse"},
            {"macro": "_LH_GMOVE", "params": {"CMD": "G0", "Z": 10}, "state": f"{s}: gripper finger offset = CAD gripper_dy {gdy} (A5; nozzles {ndy})", "set": {"toolhead": {"position": {"x": 0.0, "y": 100.0}}},
             "expect": "accept", "emits": [f"_LH_PLACE_GUARD X={0.0 + gx} Y={100.0 + gdy}"]},
            {"macro": "_LH_ZMOVE", "params": {"STEPPER": "z_a", "MOVE": 10, "SPEED": 10, "ACCEL": 300}, "state": f"{s}: carriage A nozzle = CAD axis {ax}, offset {ndy}",
             "set": {"toolhead": {"position": {"x": 0.0, "y": 100.0}}}, "expect": "accept", "emits": [f"_LH_PLACE_GUARD X={0.0 + ax} Y={100.0 + ndy}"]},
            # A5 / V3: the gripper reaches the CAD's spent-wafer slot OK at the dock row's y through the shelf window
            {"macro": "_LH_GMOVE", "params": {"CMD": "G0", "X": okx - gx, "Y": oky - gdy}, "state": f"{s}: gripper to the CAD shelf slot OK ({okx}, {oky}) from the front",
             "set": {"toolhead": {"position": {"x": okx - gx, "y": zone - 60.0}}}, "expect": "accept"},
            {"macro": "_LH_GMOVE", "params": {"CMD": "G0", "X": win - 1.0, "Y": oky - gdy}, "state": f"{s}: dock row's y 1 mm left of the shelf window",
             "set": {"toolhead": {"position": {"x": win - 1.0, "y": zone - 60.0}}}, "expect": "refuse"},
        ]
        # A3 / V6: depth caps from the CAD reach report (by_place_and_tool s_target) at a deck place, per carriage and the gripper
        rep_ = json.loads((ROOT / f"cad/lh1/machine_family/out/{s}/report.json").read_text())["by_place_and_tool"]
        for pl_, tools_ in rep_.items():
            if pl_ not in d["deck_positions"]:
                continue
            x_, y_ = d["deck_positions"][pl_]["centre_xy"]
            for k_, v_ in tools_.items():
                if not (v_.get("reached") and v_.get("interferences") == 0) or str(v_.get("what", "")).startswith(("tip pick-up", "tip parking")):
                    continue
                if k_ == "gripper":
                    c_, hx_, hy_, tool_ = "g", x_ - gx, y_ - gdy, "gripper"
                elif k_ == "M300@carriage_B":
                    c_, hx_, hy_, tool_ = "b", x_ - bx, y_ - ndy, "M300"
                else:
                    continue
                cap_ = v_["s_target"]
                cs.append({"macro": "_LH_DEPTH_GUARD", "params": {"C": c_, "TOOL": tool_, "S": cap_, "X": hx_, "Y": hy_, "WHO": "t"}, "state": f"{s}: {k_} at {pl_} to the CAD s_target {cap_}", "expect": "accept"})
                cs.append({"macro": "_LH_DEPTH_GUARD", "params": {"C": c_, "TOOL": tool_, "S": cap_ + 0.5, "X": hx_, "Y": hy_, "WHO": "t"}, "state": f"{s}: {k_} at {pl_} 0.5 mm below the CAD s_target", "expect": "refuse", "message": "DEPTH"})
        for p, v in d["deck_positions"].items():
            if p.startswith("_") or v["station"] not in MODULE_STATIONS:
                continue
            x, y = v["centre_xy"]
            con = MODULE_STATIONS[v["station"]]
            cs.append({"macro": "_LH_PLACE_GUARD", "params": {"X": x, "Y": y, "LINE": "a", "WHO": "t"}, "state": f"{s}: CAD place {p} ({v['station']}) module NOT safe",
                       "set": {f"gcode_button safe_{con}_a": {"state": "RELEASED"}}, "expect": "refuse"})
            cs.append({"macro": "_LH_PLACE_GUARD", "params": {"X": x, "Y": y, "LINE": "b", "WHO": "t"}, "state": f"{s}: CAD place {p} gripper line open",
                       "set": {f"gcode_button safe_{con}_b": {"state": "RELEASED"}}, "expect": "refuse"})
            cs.append({"macro": "_LH_PLACE_GUARD", "params": {"X": x, "Y": y, "LINE": "a", "WHO": "t"}, "state": f"{s}: CAD place {p} safe", "expect": "accept"})
        for p, v in d["deck_positions"].items():
            if p == "IG" and any("carriage B only" in r for r in d["planner_rules"]):
                x, y = v["centre_xy"]
                cs.append({"macro": "_LH_PLACE_GUARD", "params": {"X": x - 50.0, "Y": y, "LINE": "a", "WHO": "LH_PICK_TIP", "C": "a"},
                           "state": f"{s}: p1000 rack IG on carriage A (CAD rule: carriage B only)", "expect": "refuse", "message": "CARRIAGE_ONLY"})
                cs.append({"macro": "_LH_PLACE_GUARD", "params": {"X": x, "Y": y, "LINE": "a", "WHO": "LH_PICK_TIP", "C": "b"},
                           "state": f"{s}: p1000 rack IG on carriage B", "expect": "accept"})
        cs += tip_parking_cases(s, d, ax, ndy)
        if s == "L":
            x, y = d["deck_positions"]["D0"]["centre_xy"]
            cs.append({"macro": "_LH_PLACE_GUARD", "params": {"X": x, "Y": y, "LINE": "a", "WHO": "LH_PICK_TIP", "C": "a"},
                       "state": "L: p1000 rack D0 on carriage A (both carriages reach it in L)", "expect": "accept"})
        cf["cases"].append({"printer": f"m{s}", "cases": cs})
    return cf


# ------------------------------------------------------------------ V2: stale reads (23 Nov 2026)
# Klipper renders a gcode_macro's whole template when the macro STARTS, and the host runs up to about 2 s ahead of the
# motion (G4 and SYNC=1 moves do not make it wait).  So (R1) no template may read `printer` after it has issued a move,
# and (R2) a macro that reads a switch (gcode_button) or a heater must not be called after a move unless an M400 comes
# between; a PUBLIC macro (host command) and homing count as "after a move" at their start (the host's own moves may still
# be queued).  Moves: G0/G1 (and their renamed bases), G28, G4, MANUAL_STEPPER(_BASE), SET_PIN(_BASE), and every macro that
# issues one (transitively).
MOVE_RE = re.compile(r"^(G0|G1|G0\.1|G1\.1|G28|G4|MANUAL_STEPPER|MANUAL_STEPPER_BASE|SET_PIN|SET_PIN_BASE)$")


def _cmd(line):
    s_ = line.strip()
    while s_.startswith("{%"):
        e_ = s_.find("%}")
        if e_ < 0:
            return ""
        s_ = s_[e_ + 2:].strip()
    if not s_ or s_.startswith("{") or s_.startswith("#"):
        return ""
    return s_.split()[0]


def stale_read_lint(cfg_path):
    import configparser
    cp = configparser.RawConfigParser(strict=False, delimiters=(":",), comment_prefixes=("#", ";"), inline_comment_prefixes=("#",))
    cp.read_string(Path(cfg_path).read_text())
    bodies = {}
    for sec in cp.sections():
        if cp.has_option(sec, "gcode") and (sec.startswith("gcode_macro ") or sec in ("homing_override",) or sec.startswith("delayed_gcode ")):
            bodies[sec] = cp.get(sec, "gcode")
    name = lambda sec: sec.split(" ", 1)[1] if " " in sec else sec
    by_name = {name(s_): s_ for s_ in bodies}
    renamed = {cp.get(s_, "rename_existing").strip(): name(s_) for s_ in bodies if cp.has_option(s_, "rename_existing")}
    readers = {name(s_) for s_, b in bodies.items() if re.search(r'printer\[\s*["\']gcode_button|printer\[\s*["\']heater_generic', b)}
    movers = set()
    changed = True
    while changed:
        changed = False
        for s_, b in bodies.items():
            n_ = name(s_)
            if n_ in movers:
                continue
            for ln in b.splitlines():
                c_ = _cmd(ln)
                if c_ and (MOVE_RE.match(c_) or c_ in movers):
                    movers.add(n_)
                    changed = True
                    break
    bad = []
    for s_, b in bodies.items():
        n_ = name(s_)
        public = not n_.startswith("_") or s_ == "homing_override"
        pending = public
        moved = False
        for i_, ln in enumerate(b.splitlines()):
            c_ = _cmd(ln)
            if moved and "printer" in ln:
                bad.append(f"R1 [{s_}] line {i_ + 1}: reads printer after a move in the same template: {ln.strip()[:90]}")
            if c_ == "M400":
                pending = False
                continue
            if c_ in readers and pending and c_ != n_:
                bad.append(f"R2 [{s_}] line {i_ + 1}: calls {c_} (reads a switch) with no M400 after the last move")
            if c_ and (MOVE_RE.match(c_) or c_ in movers) and c_ not in ("RESPOND",):
                moved = True
                pending = True
    return {"readers": len(readers), "movers": len(movers), "violations": bad}


# ------------------------------------------------------------------ A2: guard first (expanded call chains)
CHAIN = [
    # (macro, params, state set, expected refusal text): carriage A carries M300; the command names M50 AND the plunger is
    # past the first stop / the volume is out of the M50 range, so a body check rendered before the guard would answer
    ("LH_ASPIRATE", {"CARRIAGE": "A", "TOOL": "M50", "UL": 100}, {"gcode_macro _LH_STATE": {"pos_a": 24.0}}, "WRONG_TOOL"),
    ("LH_DISPENSE", {"CARRIAGE": "A", "TOOL": "M50", "UL": 100}, {"gcode_macro _LH_STATE": {"pos_a": 24.0}}, "WRONG_TOOL"),
    ("LH_PREPARE", {"CARRIAGE": "A", "TOOL": "P99"}, {}, "WRONG_TOOL"),
    ("LH_PICK_TIP", {"CARRIAGE": "A", "TOOL": "M50", "Z_START": 280, "Z_MAX": 300}, {}, "WRONG_TOOL"),
    ("LH_TIP_CHECK", {"CARRIAGE": "A", "TOOL": "M50", "EXPECT": 1}, {}, "WRONG_TOOL"),
    ("LH_NOZZLE_CHECK", {"CARRIAGE": "A", "TOOL": "M50"}, {"gcode_macro LH_TIP_CHECK": {"armed": True}}, "WRONG_TOOL"),
    ("LH_EJECT_TIP", {"CARRIAGE": "A", "TOOL": "M50"}, {"gcode_macro LH_EJECT_TIP": {"armed_ejector": True}}, "does not carry"),
    # controls: the right tool reaches the body's own check
    ("LH_ASPIRATE", {"CARRIAGE": "A", "TOOL": "M300", "UL": 100}, {"gcode_macro _LH_STATE": {"pos_a": 24.0}}, "beyond the first stop"),
    ("LH_DISPENSE", {"CARRIAGE": "A", "TOOL": "M300", "UL": 500}, {}, "outside 0..300"),
    ("LH_PICK_TIP", {"CARRIAGE": "A", "TOOL": "M300", "Z_START": 280, "Z_MAX": 300}, {}, "pick window"),
]


def chain_cases(data):
    """A2: render a public command and every macro it calls, depth first, against one status (the state a macro sees
    when it starts): the FIRST refusal met is what the host gets.  Asserts the guard's WRONG_TOOL comes first."""
    from klipper_guardcheck.config import KlipperConfig
    from klipper_guardcheck.render import Macro, Refused, build_status, _deep_merge, _coordify
    cfg = KlipperConfig.load(fw_config.out_path(fw_config.included_size()[0]))
    base = build_status(cfg, data["printers"]["machine"])
    names = set(cfg.macros())

    def run(name, params, st, depth=0):
        out = Macro(cfg, name).render(_coordify(st), params)
        for ln in out.splitlines():
            c_ = _cmd(ln)
            if c_ in names and depth < 6:
                prm = {}
                for tok in ln.split()[1:]:
                    if "=" in tok:
                        k_, v_ = tok.split("=", 1)
                        prm[k_] = v_
                run(c_, prm, st, depth + 1)
    res = []
    for m_, prm, set_, want in CHAIN:
        st = copy.deepcopy(base)
        _deep_merge(st, set_)
        try:
            run(m_, prm, st)
            got = "accept"
        except Refused as e:
            got = str(e)
        res.append({"macro": m_, "params": prm, "state": f"A2 chain: first refusal contains {want!r}", "want": "refuse", "got": got[:160],
                    "msg": got[:160], "out": "", "ok": want in got})
    return res


# ------------------------------------------------------------------ reduced build profile MVP (24 Nov 2026)
ABSENT_MSG = ("DEVICE_ABSENT", "CARRIAGE_ABSENT")


MVP_CLASSES = {
    "APPLIES": "same result as in the starter",
    "ABSENT": "now refused DEVICE_ABSENT / CARRIAGE_ABSENT (the device or carriage is not fitted)",
    "BEHIND_ABSENT": "internal macro of an absent device, reached only through commands the MVP refuses (never runs)",
    "DECLARED": "latch / Hall / cable-loop sensing case: the MVP's tool is declared (fixed clamp), covered by guard_cases_MVP.yaml",
    "ABSENT_INTERLOCK": "interlock on a device that is not fitted (gripper state, dock lift bar): it no longer blocks",
    "PROFILE_REPLACED": "macro the MVP profile replaces (status reply, watchdog tick without a watchdog)",
    "DIFFERS": "unexplained: a failure"}


def classify_mvp(r, behind):
    if r["ok"]:
        return "APPLIES"
    if r["got"] == "refuse" and any(a in r["msg"] for a in ABSENT_MSG):
        return "ABSENT"
    if r["macro"] in behind:
        return "BEHIND_ABSENT"
    if r["macro"] in ("_LH_TOOLS_LOCKED", "_LH_POWERUP_READ"):
        return "DECLARED"
    if r["macro"] in ("_LH_GMOVE", "_LH_XY_GUARD") and re.search(r"gripper UNKNOWN|dock bar|bar not down|bar up", r["state"]) and r["got"] == "accept":
        return "ABSENT_INTERLOCK"
    if r["macro"] in ("LH_HEAD_STATUS", "delayed_gcode _LH_WD_TICK"):
        return "PROFILE_REPLACED"
    return "DIFFERS"


def mvp_status(cfg_secs, status, mvars):
    st = {k: v for k, v in status.items() if not k.startswith(("gcode_button ", "heater_generic ", "output_pin ")) or k in cfg_secs}
    mv = {k: v for k, v in mvars.items() if f"gcode_macro {k}" in cfg_secs}
    return st, mv


def mvp_runs(data):
    """(1) every machine case of guard_cases.yaml against the MVP: as written (the MVP's carriage is A, the cases' base carriage; carriage-B cases now refuse CARRIAGE_ABSENT),
    status without the absent devices; each case classified APPLIES (same result), ABSENT (now refused DEVICE_ABSENT /
    CARRIAGE_ABSENT) or DIFFERS (listed with its reason class); (2) the S geometry pins against the MVP, same classes;
    (3) guard_cases_MVP.yaml: the MVP's own cases (fixed tool, absent devices, pipetting / tip / eject / travel / depth on B);
    (4) stale-read lint; (5) reachability: a macro reading an absent device's object is reached only through a refused command"""
    import configparser
    p_, info_ = fw_config.write_profile("MVP")
    cp = configparser.RawConfigParser(strict=False, delimiters=(":",), comment_prefixes=("#", ";"), inline_comment_prefixes=("#",))
    cp.read_string(p_.read_text())
    secs = set(cp.sections())
    base = data["printers"]["machine"]
    st, mv = mvp_status(secs, base["status"], base.get("macro_variables") or {})
    cf = {"configs": {"mvp": p_.name}, "printers": {"machine": {"config": "mvp", "status": st, "macro_variables": mv, "let": base.get("let") or {}}}, "cases": []}
    mach = [c for c in data["cases"] if (c.get("printer") or "machine") == "machine"]
    cf["cases"] = mach
    tmp = HERE / "out" / "guard_cases_MVP_starter.yaml"
    tmp.write_text("# GENERATED by fw_render_check.py: the machine cases of guard_cases.yaml against the MVP\n" + yaml.safe_dump(cf, sort_keys=False, width=200))
    try:
        rr = run_case_file(tmp)["cases"]
    except Exception as e:      # a case file the package cannot load
        return {"error": f"{type(e).__name__}: {e}"}
    reach = mvp_reachability(cp, secs)
    behind = set(reach["internal_behind_refused"])
    cls = {"APPLIES": 0, "ABSENT": 0, "BEHIND_ABSENT": 0, "DECLARED": 0, "ABSENT_INTERLOCK": 0, "PROFILE_REPLACED": 0, "DIFFERS": 0}
    differs, classified = [], []
    for r in rr:
        k_ = classify_mvp(r, behind)
        cls[k_] += 1
        classified.append({"macro": r["macro"], "state": r["state"], "class": k_})
        if k_ == "DIFFERS":
            differs.append({"macro": r["macro"], "state": r["state"], "want": r["want"], "got": r["got"], "msg": r["msg"][:120], "problems": r.get("problems")})
    # (2) S geometry pins against the MVP
    sc = yaml.safe_load((HERE / "out" / "guard_cases_sizes.yaml").read_text())
    sS = next(c for c in sc["cases"] if c["printer"] == "mS")["cases"]
    ps = sc["printers"]["mS"]
    st2, mv2 = mvp_status(secs, ps["status"], ps.get("macro_variables") or {})
    gp = {"configs": {"mvp": p_.name}, "printers": {"m": {"config": "mvp", "status": st2, "macro_variables": mv2}}, "cases": [{"printer": "m", "cases": sS}]}
    tg = HERE / "out" / "guard_cases_MVP_pins.yaml"
    tg.write_text("# GENERATED by fw_render_check.py: the S geometry pins against the MVP\n" + yaml.safe_dump(gp, sort_keys=False, width=200))
    rg = run_case_file(tg)["cases"]
    gcls = {"APPLIES": 0, "ABSENT": 0, "DIFFERS": 0}
    gdiff = []
    for r in rg:
        if r["ok"]:
            gcls["APPLIES"] += 1
        elif r["got"] == "refuse" and any(a in r["msg"] for a in ABSENT_MSG):
            gcls["ABSENT"] += 1
        else:
            gcls["DIFFERS"] += 1
            gdiff.append({"macro": r["macro"], "state": r["state"], "want": r["want"], "got": r["got"], "msg": r["msg"][:120]})
    # (3) the MVP's own cases
    own = run_case_file(HERE / "guard_cases_MVP.yaml", {"mvp": str(p_)})
    lint = stale_read_lint(p_)
    return {"config": str(p_.relative_to(ROOT)), "sections_cut": len(info_["cut"]),
            "s_cases": {"n": len(rr), **cls, "classes": MVP_CLASSES, "differs": differs, "cases": classified},
            "s_geometry_pins": {"n": len(rg), **gcls, "differs": gdiff},
            "guard_cases_MVP": {"n": own["n"], "failed": own["failed"], "failed_cases": [f"{c['macro']} [{c['state']}] {c.get('problems')}" for c in own["cases"] if not c["ok"]][:20]},
            "stale_read_lint": {"violations": len(lint["violations"]), "first": lint["violations"][:10]},
            "reachability": reach}


def mvp_reachability(cp, secs):
    """every macro whose template names an object the MVP does not have (fixed name, not a profile-guarded branch) must be
    reachable only from public commands the profile replaced with a DEVICE_ABSENT refusal"""
    bodies = {s.split(" ", 1)[1] if " " in s else s: cp.get(s, "gcode") for s in cp.sections() if cp.has_option(s, "gcode")}
    refused = {n for n, b in bodies.items() if "DEVICE_ABSENT" in b and len([ln for ln in b.splitlines() if ln.strip()]) == 1}
    bad_obj = {}
    for n, b in bodies.items():
        if n in refused or "_LH_PROFILE" in b:
            continue
        objs = set(re.findall(r"printer\[\s*['\"]((?:gcode_button|heater_generic) [a-z0-9_]+)['\"]\s*\]", b))
        pins = {"output_pin " + p for p in re.findall(r"PIN=([a-z_]+)", b)}
        st_ = {"manual_stepper " + p for p in re.findall(r"STEPPER=([a-z_]+)\b", b) if not p.endswith("_") and p != "stepper_z"}
        miss = sorted(o for o in objs | pins | st_ if o not in secs)
        if miss:
            bad_obj[n] = miss
    absent = set(re.findall(r'"([a-z_]+)"', cp.get("gcode_macro _LH_PROFILE", "variable_absent")))
    callers = {n: set() for n in bodies}
    for n, b in bodies.items():
        skip = 0        # inside {% if "<absent group>" not in pf.absent %} ... {% endif %}: never rendered in this build
        for ln in b.splitlines():
            s_ = ln.strip()
            if skip:
                skip += s_.count("{% if ") - s_.count("{% endif %}")
                continue
            m_ = re.match(r'\{% if "([a-z_]+)" not in pf\.absent %\}$', s_)
            if m_ and m_.group(1) in absent:
                skip = 1
                continue
            c = _cmd(ln)
            if c in callers and c != n:
                callers[c].add(n)
    triggers = {s.split(" ", 1)[-1] for s in cp.sections() if s.startswith(("gcode_button ", "delayed_gcode ")) or s == "homing_override"}
    problems = {}
    for n in bad_obj:
        seen, stack, open_ = set(), [n], []
        while stack:
            x = stack.pop()
            if x in seen:
                continue
            seen.add(x)
            if x in refused:
                continue
            if not x.startswith("_") or x in triggers:      # a public command or a trigger reaches it
                if x != n or not x.startswith("_"):
                    open_.append(x)
            stack += list(callers.get(x, ()))
        if open_:
            problems[n] = {"absent_objects": bad_obj[n], "reached_from": sorted(open_)}
    # internal macros reached ONLY through refused commands (never runs in this build)
    behind = set()
    for n in bodies:
        if not n.startswith("_") or n in triggers:
            continue
        seen, stack, entry = set(), [n], []
        while stack:
            x = stack.pop()
            if x in seen:
                continue
            seen.add(x)
            if x in refused:
                continue
            if (not x.startswith("_") or x in triggers) and x != n:
                entry.append(x)
            stack += list(callers.get(x, ()))
        if not entry:       # no public command or trigger reaches it in this build (orphaned by the refusals)
            behind.add(n)
    return {"macros_naming_absent_objects": len(bad_obj), "only_behind_refused_commands": len(bad_obj) - len(problems), "problems": problems,
            "internal_behind_refused": sorted(behind)}


def main() -> int:
    for s in fw_config.SIZES:
        fw_config.write(s)
    built = fw_config.included_size()[0]
    main_cases = HERE / "guard_cases.yaml"
    res = run_case_file(main_cases, {"machine": str(fw_config.out_path(built))})
    by_size = {built: {"guard_cases": {"n": res["n"], "failed": res["failed"]}}}
    failed = res["failed"]
    extra = []
    data = yaml.safe_load(main_cases.read_text())
    for s in fw_config.SIZES:
        if s == built:
            continue
        r_ = run_case_file(main_cases, {"machine": str(fw_config.out_path(s))})
        f_ = [c for c in r_["cases"] if not c["ok"]]
        by_size.setdefault(s, {})["guard_cases"] = {"n": r_["n"], "failed": len(f_),
                                                    "failed_cases": [f"{c['macro']} [{c['state']}] {c.get('problems')}" for c in f_][:20]}
        failed += len(f_)
    sc = size_cases(data["printers"]["machine"]["status"], data["printers"]["machine"].get("macro_variables") or {})
    sp = HERE / "out" / "guard_cases_sizes.yaml"
    sp.write_text("# GENERATED by fw_render_check.py from cad/lh1/machine_family/out/<size>/family_<size>.json: do not edit\n"
                  + yaml.safe_dump(sc, sort_keys=False, width=200))
    rs = run_case_file(sp)
    for s in fw_config.SIZES:
        mine = [c for c in rs["cases"] if c["state"].startswith(f"{s}:")]
        by_size.setdefault(s, {})["geometry_pins"] = {"n": len(mine), "failed": sum(1 for c in mine if not c["ok"])}
    failed += rs["failed"]
    extra = [c for c in rs["cases"] if not c["ok"]]
    lint = {s_: stale_read_lint(fw_config.out_path(s_)) for s_ in fw_config.SIZES}
    for s_, l_ in lint.items():
        by_size[s_]["stale_read_lint"] = {"violations": len(l_["violations"]), "first": l_["violations"][:10]}
        failed += len(l_["violations"])
    ch = chain_cases(data)
    by_size[built]["chain_cases"] = {"n": len(ch), "failed": sum(1 for c in ch if not c["ok"])}
    failed += sum(1 for c in ch if not c["ok"])
    extra += [c for c in ch if not c["ok"]]
    mvp = mvp_runs(data)
    mvp_failed = (1 if "error" in mvp else 0)
    if "error" not in mvp:
        mvp_failed = (mvp["guard_cases_MVP"]["failed"] + mvp["stale_read_lint"]["violations"] + len(mvp["reachability"]["problems"])
                      + mvp["s_cases"]["DIFFERS"] + mvp["s_geometry_pins"]["DIFFERS"])
        by_size["MVP"] = {"guard_cases_MVP": mvp["guard_cases_MVP"], "stale_read_lint": mvp["stale_read_lint"],
                          "s_cases": {"n": mvp["s_cases"]["n"], "failed": mvp["s_cases"]["DIFFERS"]},
                          "s_geometry_pins": {"n": mvp["s_geometry_pins"]["n"], "failed": mvp["s_geometry_pins"]["DIFFERS"]},
                          "reachability": {"n": mvp["reachability"]["macros_naming_absent_objects"], "failed": len(mvp["reachability"]["problems"])}}
    else:
        by_size["MVP"] = {"run": {"n": 0, "failed": 1, "failed_cases": [mvp["error"]]}}
    (HERE / "out" / "guard_cases_MVP_report.json").write_text(json.dumps(mvp, indent=1))
    failed += mvp_failed
    out = {"jinja2": res["jinja2"], "n": res["n"], "failed": res["failed"], "built_size": built,
           "by_size": by_size, "total_failed_all_runs": failed, "cases": res["cases"] + extra}
    print(json.dumps(out, indent=1))
    return 1 if failed else 0


if __name__ == "__main__":
    sys.exit(main())
