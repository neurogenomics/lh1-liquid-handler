#!/usr/bin/env python3
"""Write firmware/lh1/lh1_size_<S|M|L>.cfg from the family CAD reports (cad/lh1/machine_family/out/<size>/family_<size>.json).

The size files hold ONLY what differs between the sizes (ADR-011): Y travel end, dock row, dock zone and exit, the
deck-module places with their SAFE-line connector, the carriage-only places (planner rule), the station list, and the
size-only inputs (row-D SAFE lines in L, the H1 and H2 shutters in M and L: the starter has no open hand-off).  check.py and fw_render_check.py pin every value to the
same CAD reports, so a CAD change shows as a failed check until this script is re-run (and the result reviewed).
Run: .venv/bin/python electronics/lh1/gen_size_files.py
"""
import json
import re
from pathlib import Path

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[1]
MOD = {"CTB1": "a3", "CTB2": "a3", "STRIP": "b2", "CPB1": "g4", "HAZ": "g0", "BAL": "d3", "WN1": "d4"}
ROWD_PINS = {"d3": ("PE12", "PB1"), "d4": ("PE13", "PB2")}
DESC = {"S": "STARTER (built first): 4 rows F A B G, 16 places",
        "M": "INTERMEDIATE: 5 rows F A B C G (kit S->M adds row C: no electrical module; the H1 and H2 hand-off shutters are fitted)",
        "L": "LARGE: 6 rows F A B C D G (kit M->L adds row D; the balance D3 and the weighing nest D4 bring two module connectors)"}
CARRIAGES = {"carriage_A": "a", "carriage_B": "b"}


def _num(pat, txt, what):
    m = re.search(pat, txt)
    if not m:
        raise SystemExit(f"gen_size_files: the CAD tip_parking motion text no longer states {what} ({pat!r})")
    return float(m.group(1))


def tip_geometry(tp, s_targets, audit_rows):
    """ADR-014 #6: random-access geometry from the CAD tip_parking block (S: TPK-S groups).  Returns None where the CAD
    publishes no groups (none since the integration of 23 Nov 2026: TPK-S in every size).  spots: [group, entry x, spot x] in spot order (group 1
    left spot, group 1 right spot, ...); row_y: the nozzle-row centre over the eight rows; s_above / s_slide / s_seat and the
    speeds from the CAD 'motion (firmware and planner)' text and s_values; strip travel from the CAD mechanism audit; the
    re-pick window ends at the CAD depth cap there: the CAD reach s_target, which is the re-pick's own over-travel end (s_seat + 1,
    checked in the model with bare nozzles; integration 23 Nov 2026: was s_seat - 0.5 + 1), at most 1 mm below s_seat."""
    if not tp.get("random_access") or not tp.get("groups"):
        return None
    mot = next(v for k, v in tp.items() if k.startswith("motion"))
    park = " ".join(next(v for k, v in mot.items() if k.startswith("park")))
    if not any("force-ended" in ln for k, v in mot.items() if k.startswith("re-pick") for ln in v):
        raise SystemExit("gen_size_files: the CAD re-pick is no longer force-ended (LH_TIP_UNPARK uses LH_PICK_TIP)")
    spots = []
    for g in tp["groups"]:
        for x in sorted(g["spots_x"]):
            spots.append([g["group"], g["entry_x"], x])
    rows = tp["rows_y"]
    s_seat = float(tp["s_values"]["s_seat"])
    strip = next(r for r in audit_rows if r["mechanism"].startswith("tip-comb strip"))
    cap = min(s_targets)
    return {"spots": spots, "row_y": round(sum(rows) / len(rows), 2),
            "s_above": _num(r"s_above = (\d+(?:\.\d+)?)", park, "s_above"),
            "s_slide": float(tp["s_values"]["s_slide"]), "s_seat": s_seat,
            "z_mm_s": _num(r"Z down at <= (\d+(?:\.\d+)?) mm/s", park, "the Z speed into the comb"),
            "slide_mm_s": _num(r"X to the spot .*?at <= (\d+(?:\.\d+)?) mm/s", park, "the slide speed"),
            "slide_accel": _num(r"acceleration <= (\d+(?:\.\d+)?) m/s2", park, "the slide acceleration") * 1000.0,
            "strip_mm_s": _num(r"strip: Z up .*?at <= (\d+(?:\.\d+)?) mm/s", park, "the strip speed"),
            "strip_mm": float(strip["travel"]["needed_mm"]),
            "repick_over": round(max(0.0, min(1.0, cap - s_seat)), 2)}


def tip_parking(d):
    """the size's tip-parking place from the CAD tip_parking block: place -> [x, y, half x, half y, [carriages], [tools],
    station, eight-channel sets, geometry].  Half sizes: the comb bounding box where the CAD publishes it (TPK-S in S), else
    the place half (TPK-1 at C0 in M, L).  Carriages: the CAD reach entries '<tool>@carriage_<A|B>' reached with zero
    interferences.  Tools: the CAD tools_admitted list where published (ADR-014: ['M300']), else the reached tools.
    geometry: tip_geometry() (random access, S) or None (no spot geometry published: LH_TIP_PARK refuses NO_GEOMETRY)."""
    tp = d.get("tip_parking") or {}
    if not tp.get("present"):
        return {}
    reach = next((v for k, v in tp.items() if k.startswith("reach")), {})
    ok = [k.split("@") for k, v in reach.items() if v.get("reached") and v.get("interferences") == 0]
    tools = sorted(set(tp["tools_admitted"]) & {t for t, _ in ok}) if tp.get("tools_admitted") else sorted({t for t, _ in ok})
    cars = sorted({CARRIAGES[c] for _, c in ok})
    x, y = tp.get("centre_xy") or d["deck_positions"][tp["place"]]["centre_xy"]
    if "comb_bbox" in tp:
        b = tp["comb_bbox"]
        hx, hy = round((b[1] - b[0]) / 2, 1), round((b[3] - b[2]) / 2, 1)
    else:
        hx, hy = 66.0, 55.0
    s_t = sorted({v.get("s_target") for v in reach.values() if v.get("s_target") is not None})
    geo = tip_geometry(tp, s_t, (d.get("mechanism_audit") or {}).get("rows") or []) if s_t else None
    return {tp["place"].lower(): [x, y, hx, hy, cars, tools, tp["station"], tp["capacity_sets"], geo]}


SEAT_Z = 5.0          # CAD keepouts note: z 0 = deck top, labware seat z 5 (place heights are above the seat)
PLACE_HALF = (66.0, 55.0)
SHELF_HALF = (65.0, 55.0)
def shelf_slot_name(d, key):
    """reach-report keys to shelf slots: a hand-off by its name (H1, H2) at the slot the CAD shelf_slots names, a lid park by '<slot>_lidpark'"""
    ss = d.get("shelf_slots") or {}
    for k, v in ss.items():
        if not k.startswith("_") and v.get("handoff") == key:
            return k
    return key.split("_")[0] if key.endswith("_lidpark") else key


def shelf_xy(d, slot):
    """Shelf slot centre from the CAD (family_<size> shelf_slots centre_xy; integration 23 Nov 2026: published, no longer derived from the
    slot name)."""
    v = (d.get("shelf_slots") or {}).get(slot)
    return None if not v else [float(v["centre_xy"][0]), float(v["centre_xy"][1])]


def zones(d):
    """B (23 Nov 2026): every CAD item a lowered axis must clear when it moves sideways: each deck place (top = seat +
    the larger of its height and its height limit, so taller labware allowed there is covered), each above-deck keepout
    box, each shelf slot (ASSUMED slot XY).  [x0, x1, y0, y1, top z, name]."""
    out = []
    for p, v in d["deck_positions"].items():
        if p.startswith("_"):
            continue
        x, y = v["centre_xy"]
        h = max(float(v.get("height_mm") or 0.0), float(v.get("height_limit_mm (R1/R2/N1/N2)") or 0.0))
        out.append([x - PLACE_HALF[0], x + PLACE_HALF[0], y - PLACE_HALF[1], y + PLACE_HALF[1], round(SEAT_Z + h, 1), p])
    for k, v in (d.get("keepouts") or {}).items():
        if k.startswith("_"):
            continue
        b = v.get("union") or v.get("box")
        if b and b[5] > 0:
            out.append([b[0], b[1], b[2], b[3], float(b[5]), k.split(" (")[0]])
    for k, v in (d.get("shelf") or {}).items():
        xy = shelf_xy(d, k)
        if xy:
            out.append([xy[0] - SHELF_HALF[0], xy[0] + SHELF_HALF[0], xy[1] - SHELF_HALF[1], xy[1] + SHELF_HALF[1],
                        round(SEAT_Z + float(v.get("height_mm") or 0.0), 1), "shelf_" + k])
    return out


def depth(d, rep):
    """A3 / V6 (23 Nov 2026): deepest carriage or gripper stroke per place and tool from the CAD reach report
    (report.json by_place_and_tool s_target: into the wells / tubes to the floor, onto the tip collars, the plate seat).
    Tip pick-up and tip-parking places get +1.0 mm (the force-ended pick-up window ends 1 mm past the seat).  A tool not
    listed (or not reached with zero interference) has no cap entry: it may not go below the clearance height there.
    place -> [x, y, half x, half y, {"<tool>@<a|b>" or "gripper": s max}]"""
    out = {}
    for p, tools in (rep.get("by_place_and_tool") or {}).items():
        if p in d["deck_positions"]:
            xy, half = d["deck_positions"][p]["centre_xy"], PLACE_HALF
        else:
            xy, half = shelf_xy(d, shelf_slot_name(d, p)), SHELF_HALF
            if xy is None:
                continue
        caps = {}
        for k, v in tools.items():
            if not (v.get("reached") and v.get("interferences") == 0 and v.get("s_target") is not None):
                continue
            # tip pick-up: the force-ended window ends 1 mm past the CAD's nominal pick depth; tip parking: the CAD target is already the
            # re-pick's over-travel end (s_seat + 1, checked)
            extra = 1.0 if str(v.get("what", "")).startswith("tip pick-up") else 0.0
            key = "gripper" if k == "gripper" else k.split("@")[0] + "@" + k.split("@carriage_")[1].lower()
            caps[key] = round(float(v["s_target"]) + extra, 2)
        out[p.lower()] = [xy[0], xy[1], half[0], half[1], caps]
    return out


SHUTTER_H2 = """
# Edge hand-off H1 (M and L; the starter has NO open hand-off: blank plates at H1 and H2, integration 23 Nov 2026): shutter-closed
# switch, NO ('^!': PRESSED = closed); status for the gripper hand-off macros
[gcode_button shutter_h1]
pin: ^!io:PG2
press_gcode:
    _LH_NOP
release_gcode:
    _LH_NOP

# Second edge hand-off H2 (M and L; the starter has a blank plate there): shutter-closed switch, NO ('^!': PRESSED = closed)
[gcode_button shutter_h2]
pin: ^!io:PC13
press_gcode:
    _LH_NOP
release_gcode:
    _LH_NOP
"""


def text(s):
    d = json.loads((ROOT / f"cad/lh1/machine_family/out/{s}/family_{s}.json").read_text())
    dock = d["dock"]["printer_y_over_the_bays (dock_yf)"]
    zone = d["dock"]["dock_zone_yf"]
    ymax = d["travel"]["Y"]["travel_yf"][1]
    places, stations, co = {}, {}, {}
    for p, v in d["deck_positions"].items():
        if p.startswith("_"):
            continue
        stations[p] = v["station"]
        if v["station"] in MOD:
            places[p.lower()] = [v["centre_xy"][0], v["centre_xy"][1], MOD[v["station"]]]
    rule = [r for r in d["planner_rules"] if "carriage B only" in r]
    if "IG" in d["deck_positions"] and rule:
        x, y = d["deck_positions"]["IG"]["centre_xy"]
        co["ig"] = [x, y, 66.0, 55.0, ["b"]]
    pl = "{" + ", ".join(f'"{k}": [{v[0]}, {v[1]}, "{v[2]}"]' for k, v in places.items()) + "}"
    st = "{" + ", ".join(f'"{k}": "{v}"' for k, v in stations.items()) + "}"
    tpk = tip_parking(d)
    tps = "{" + ", ".join(f'"{k}": [{v[0]}, {v[1]}, {v[2]}, {v[3]}, ' + json.dumps(v[4]) + ", " + json.dumps(v[5]) + f', "{v[6]}", {v[7]}, ' + ("None" if v[8] is None else repr(v[8])) + "]" for k, v in tpk.items()) + "}"
    rep = json.loads((ROOT / f"cad/lh1/machine_family/out/{s}/report.json").read_text())
    bays = d["frame"]["dock_bays_x"]
    # A5 / V3: the shelf window behind the deck, at the dock row's Y but right of the dock bays: head X from which every
    # axis (carriage A, axis_dx -90, the leftmost) and a tool of bay width (one bay pitch) clear the last bay by 10 mm
    win_x = round(max(bays) + (bays[1] - bays[0]) - min(d["tool_carriages"]["axes"].values()) + 10.0, 1)
    shk = next(([v["centre_xy"][0], v["centre_xy"][1], PLACE_HALF[0], PLACE_HALF[1]] for p, v in d["deck_positions"].items()
                if not p.startswith("_") and v["station"] == "SHAKER"), None)
    zjs = "[" + ", ".join("[" + ", ".join(f"{a:g}" if isinstance(a, (int, float)) else f'"{a}"' for a in z) + "]" for z in zones(d)) + "]"
    djs = "{" + ", ".join(f'"{k}": [{v[0]:g}, {v[1]:g}, {v[2]:g}, {v[3]:g}, ' + json.dumps(v[4]) + "]" for k, v in depth(d, rep).items()) + "}"
    tpc = (f"CAD tip_parking: {tpk and list(tpk.values())[0][6]} at {tpk and list(tpk)[0].upper()}, " + ("comb bounding box" if "comb_bbox" in d["tip_parking"] else "place half (no comb box published)")) if tpk else "none in this size"
    cos = "{" + ", ".join(f'"{k}": [{v[0]}, {v[1]}, {v[2]}, {v[3]}, ["b"]]' for k, v in co.items()) + "}"
    out = f"""# LH-1 size file: {s} = {DESC[s]}.
# Included at the END of printer.cfg (shared core); everything that differs between the sizes is here and nowhere else.
# GENERATED by electronics/lh1/gen_size_files.py from cad/lh1/machine_family/out/{s}/family_{s}.json (travel.Y, dock
# 'printer_y_over_the_bays (dock_yf)' and dock_zone_yf, deck_positions, planner_rules, tip_parking); dock_exit_yf = dock_yf - 124.5
# (cr_answers, rev L ratio).  fw_render_check.py and check.py pin every value to that report.  PROVISIONAL until the
# commissioning dry run measures the dock row with the head camera (bay markers DMK-F).
# Klipper merges this [stepper_y] with the core's [stepper_y] (same section named twice: the later options are added).

[stepper_y]
position_max: {ymax}            # family_{s} travel.Y travel_yf upper end (the gripper at the shelf slot OK at the dock row)

[gcode_macro _LH_SIZE]
variable_size: "{s}"
variable_y_max: {ymax}               # = [stepper_y] position_max (check.py)
variable_dock_yf: {dock}            # printer Y with the coupling axes over the bay row (dock axis y {d['frame']['dock_axis_y']} + 102.5)
variable_dock_zone_yf: {zone}        # at and beyond this printer Y the head is over the dock row (only LH_TOOL_* / LH_DOCK_LEAVE go there)
variable_dock_exit_yf: {round(dock - 124.5, 1)}        # LH_DOCK_LEAVE target, in front of the dock zone
# deck-module places with a SAFE-line connector: place -> [x, y, connector]; A3 and A4 are one module zone on connector a3
variable_places: {pl}
# places only one carriage may serve (planner rule: {rule[0] if co else 'none in this size'}): place -> [x, y, half x, half y, [carriages]]
variable_carriage_only: {cos}
# tip parking ({tpc}): place -> [x, y, half x, half y, [carriages], [tools], station, eight-channel sets,
#   random-access geometry {{spots: [group, entry x, spot x] per spot, row_y, s_above, s_slide, s_seat, speeds, strip travel,
#   re-pick overtravel}} or None where the CAD publishes no spot geometry (ADR-014: random access in S; M, L still LIFO TPK-1)];
# only these carriages, carrying one of these tools (tips mounted to set down, bare nozzles to re-pick), may descend there;
# the gripper never descends there (_LH_PLACE_GUARD TIP_PARKING)
variable_tip_parking: {tps}
# B: items a LOWERED axis must clear when it moves sideways (_LH_TRAVEL_GUARD): [x0, x1, y0, y1, top z, name] from the CAD
# deck places (seat z 5 + max(height, height limit)), the above-deck keepouts and the shelf slots (CAD shelf_slots XY)
variable_zones: {zjs}
# A3 / V6: deepest stroke per place and tool (CAD report by_place_and_tool s_target; +1 mm at tip pick-up / parking places):
# place -> [x, y, half x, half y, {{"<tool>@<carriage>" | "gripper": s max}}]; a tool without an entry stays above the clearance height
variable_depth: {djs}
# A5 / V3: head X from which the gripper may enter the dock row's Y (shelf slots OK, ...): right of the bays by a bay and 10 mm
variable_shelf_window_x: {win_x}
# A6: the heater-shaker place (station SHAKER): descents there need LH_SHAKER STATE=IDLE (host-declared)
variable_shaker: {json.dumps(shk)}
# every station by place (status and host cross-check; the guards use places and carriage_only)
variable_stations: {st}
gcode:
"""
    if s == "L":
        out += "\n# Row D module places (L only): balance D3 and weighing nest D4; these inputs are reserved and unwired in S and M.\n"
        for k, (a, b) in ROWD_PINS.items():
            out += f"[gcode_button safe_{k}_a]\npin: ^!io:{a}\npress_gcode:\n    _LH_NOP\n[gcode_button safe_{k}_b]\npin: ^!io:{b}\npress_gcode:\n    _LH_NOP\n"
    if s in ("M", "L"):
        out += SHUTTER_H2
    return out


ABSENT_WHY = {"dock": "dock, latch sensing and tool exchange", "gripper": "gripper axis", "thermocycler": "thermocycler TC-1 and its THERM path",
              "shaker": "heater-shaker", "tip_parking": "tip parking comb", "extraction": "extraction fan and airflow switch (no ethanol steps)",
              "uv": "UV-C option", "deck_light": "deck light"}


def profile_text(name):
    """firmware/lh1/lh1_profile_<name>.cfg from electronics/lh1/profile_<name>.yaml: included LAST by printer_<name>.cfg"""
    import fw_config
    p = fw_config.profile_yaml(name)
    pz = p["phantom_z"]
    ft = "{" + ", ".join(f'"{k}": "{v}"' for k, v in p["fixed_tools"].items()) + "}"
    o = f"""# LH-1 build profile {name}: GENERATED by electronics/lh1/gen_size_files.py --profile {name} from electronics/lh1/profile_{name}.yaml.
# Do not edit.  Included LAST by printer_{name}.cfg (itself generated from the shared core printer.cfg without the hardware
# sections of the absent devices), after lh1_size_{p['base_size']}.cfg.  The core macros run unchanged; they read _LH_PROFILE:
# absent groups refuse DEVICE_ABSENT, carriage B refuses CARRIAGE_ABSENT, and carriage A's tool is DECLARED here ({p['fixed_tools']}),
# never treated as sensed (no latch, no Hall code).  Guard: {p['guard']}.  DESIGNED and SIMULATED only: Klipper has not loaded it.

[gcode_macro _LH_PROFILE]
variable_name: "{name}"
variable_absent: {json.dumps(p['absent_groups'])}
variable_carriages: {json.dumps(p['carriages'])}
variable_fixed_tools: {ft}

# Printer Z = the gripper Z on head_a EZ3 (core [stepper_z], pins unchanged): MMB_A is fitted for carriage A but socket EZ3 has no
# driver and no motor (its [tmc5160 stepper_z] is cut).  Klipper's cartesian kinematics needs the Z rail, so homing_override marks
# Z homed at 0 (set_position_z) and every G0/G1 Z refuses DEVICE_ABSENT: this axis never moves.  {pz['note']}
[homing_override]
set_position_z: 0

# no THERM path and no watchdog relay: the tick only re-arms itself (THERM can never be on: LH_THERM_ON refuses)
[delayed_gcode _LH_WD_TICK]
gcode:
    UPDATE_DELAYED_GCODE_BASE ID=_LH_WD_TICK DURATION=1

[gcode_macro LH_HEAD_STATUS]
gcode:
    {{% set s = printer["gcode_macro _LH_STATE"] %}}
    RESPOND TYPE=command MSG="LH_HEAD BUILD={name} A={{s.mounted_a or 'EMPTY'}} (fixed) B=ABSENT TIPS_A={{s.tips_a}} NOZZLE_OK_A={{s.nozzle_ok_a}} HOLD={{s.hold or 'NONE'}} PHOLD={{s.phold or 'NONE'}} AIRFLOW=ABSENT BIN={{'OK' if printer['gcode_button bin_clear'].state == 'PRESSED' else 'FULL'}} DRAWER={{'IN' if printer['gcode_button drawer'].state == 'PRESSED' else 'OUT'}} WASTE=ABSENT BAYS=ABSENT"
"""
    for g, cmds in p["absent_commands"].items():
        if g not in p["absent_groups"]:
            continue
        for c in cmds:
            o += (f"\n[gcode_macro {c}]\ngcode:\n    {{action_raise_error(\"{c} refused: DEVICE_ABSENT: {ABSENT_WHY.get(g, g)} not fitted in build {name}\")}}\n")
    return o


if __name__ == "__main__":
    import sys
    if len(sys.argv) > 2 and sys.argv[1] == "--profile":
        import fw_config
        n_ = sys.argv[2]
        (ROOT / f"firmware/lh1/lh1_profile_{n_}.cfg").write_text(profile_text(n_))
        core_, cut_ = fw_config.profile_core_text(n_)
        (ROOT / f"firmware/lh1/printer_{n_}.cfg").write_text(core_)
        print(fw_config.write_profile(n_)[0], f"{len(cut_)} core sections cut")
    else:
        for s_ in ("S", "M", "L"):
            (ROOT / f"firmware/lh1/lh1_size_{s_}.cfg").write_text(text(s_))
            print(s_)
