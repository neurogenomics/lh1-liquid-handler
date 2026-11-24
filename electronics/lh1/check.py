"""Checks for the LH-1 electrical architecture, wiring harness and Klipper configuration (paper checks, not a bench test).

Run: .venv/bin/python electronics/lh1/check.py  -> electronics/lh1/out/check_report.json
Architecture (system.yaml)
* rail, branch and supply loading <= 80 % of the fuse / supply rating;
* e-stop removes motion, pipette and thermal power but not logic; the door removes motion only;
* every heater rail passes through its own independent thermostats (three on the thermocycler);
* driver sockets: main board 0-7, each head board 1-4, none used twice.
Firmware (firmware/lh1/printer.cfg)
* parses as INI, no pin assigned twice, every stepper has step/dir/enable, travel limits equal params.py,
  steppers per MCU fit that board's sockets, no TMC UART address reused, every MCU in the cfg is a board here;
* every pin used in printer.cfg is wired (wiring.csv fw_pins) to the device it drives or reads, and every
  fw_pins entry in wiring.csv exists in printer.cfg.
Harness (wiring.csv, cable_types.yaml)
* every device in system.yaml appears in wiring.csv and the reverse; cable ids unique; known cable types;
* each drag chain (sizes read from the CAD reports): conductor cross-section fill <= max_fill_fraction, bend
  radius >= bend_factor x OD for every cable in it, only chain-rated cables, bend height fits the space;
  misfits that are recorded as open CAD requests are reported as warnings;
* power cables: voltage drop < 3 % (2 x length for a pair, 1 x length for a single leg whose return is another
  cable); end-to-end drop on the longest feeds reported;
* no cable carries more than its connector contact rating (and its conductors fit the contact's wire range) or its own ampacity; every fused cable's ampacity
  is at least its fuse rating.
Cabinet (control_bay.yaml): items inside the plate and the bay height, no overlaps, heat and airflow.
"""
import ast
import configparser
import csv
import importlib.util
import json
import shutil
import subprocess
import math
import re
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "tools" / "vendor"))  # vendored helper packages
import harnessgen as hg      # packages/harnessgen: wiring-table checks (uv pip install -e packages/harnessgen)
import yaml

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[1]
RHO = 0.0172          # ohm mm2 / m, copper at 20 C


def load_cfg(path=None, text=None):
    cp = configparser.RawConfigParser(strict=True, delimiters=(":",), comment_prefixes=("#",), inline_comment_prefixes=("#",))
    cp.read_string(text if text is not None else path.read_text())
    return cp


sys.path.insert(0, str(HERE))
import fw_config  # noqa: E402  (resolves printer.cfg + its size file the way Klipper does)


def family(size):
    return json.loads((ROOT / f"cad/lh1/machine_family/out/{size}/family_{size}.json").read_text())


MODULE_STATIONS = {"CTB1": "a3", "CTB2": "a3", "STRIP": "b2", "CPB1": "g4", "HAZ": "g0", "BAL": "d3", "WN1": "d4"}


def family_checks(sysd, fail, warn):
    """ADR-011 size family: structure of the firmware (shared core + ONE size file) and every PROVISIONAL geometry value
    of each size file against the CAD report of that size.  The size being built (system.yaml family.build_size) must be
    the size the core includes; a disagreement with ITS CAD report fails, and so does one in the M and L files."""
    out = {}
    core = fw_config.core_text()
    built, inc = fw_config.included_size(core)
    want = sysd["family"]["build_size"]
    out["core_includes"] = inc
    if built is None:
        fail("SIZE_INCLUDE", f"printer.cfg must include exactly one lh1_size_<S|M|L>.cfg, found {inc}")
    elif built != want:
        fail("SIZE_INCLUDE", f"printer.cfg includes size {built} but system.yaml family.build_size is {want}")
    ccp = configparser.RawConfigParser(strict=True, delimiters=(":",), comment_prefixes=("#",), inline_comment_prefixes=("#",))
    ccp.read_string(fw_config.INCLUDE.sub("", core))
    if ccp.has_option("stepper_y", "position_max") or ccp.has_section("gcode_macro _LH_SIZE"):
        fail("SIZE_VALUE_IN_CORE", "printer.cfg (core) must not hold the Y travel end or _LH_SIZE: they belong in the size file")
    for s in fw_config.SIZES:
        try:
            txt, info = fw_config.resolve(s)
        except Exception as exc:          # noqa: BLE001
            fail("SIZE_FILE", f"{s}: {exc}")
            continue
        if info["bad_size_sections"]:
            fail("SIZE_FILE_SCOPE", f"lh1_size_{s}.cfg holds shared sections {info['bad_size_sections']}")
        cp = load_cfg(text=txt)
        d = family(s)
        zv = lambda k: ast.literal_eval(cp.get("gcode_macro _LH_SIZE", f"variable_{k}"))
        gv = lambda k: ast.literal_eval(cp.get("gcode_macro _LH_GEOM", f"variable_{k}"))
        pins = {}
        def pin(label, got, cad):
            pins[label] = {"firmware": got, "cad": cad}
            if (isinstance(cad, (int, float)) and abs(float(got) - float(cad)) > 1e-6) or (not isinstance(cad, (int, float)) and got != cad):
                fail("PROVISIONAL_VS_CAD", f"size {s}{' (BUILT)' if s == want else ''}: {label} = {got} but cad family_{s} = {cad}")
        dock = d["dock"]["printer_y_over_the_bays (dock_yf)"]
        pin("_LH_SIZE size", zv("size"), s)
        pin("_LH_SIZE dock_yf", zv("dock_yf"), dock)
        pin("_LH_SIZE dock_zone_yf", zv("dock_zone_yf"), d["dock"]["dock_zone_yf"])
        pin("_LH_SIZE dock_exit_yf", zv("dock_exit_yf"), round(dock - 124.5, 1))
        pin("dock_yf + noz_dy (= CAD dock axis y)", zv("dock_yf") + gv("noz_dy"), d["frame"]["dock_axis_y"])
        pin("_LH_GEOM noz_dy", gv("noz_dy"), d["tool_carriages"]["nozzle_dy_from_beam_front_face"])
        for k, n in (("a", "carriage_A"), ("b", "carriage_B"), ("g", "gripper")):
            pin(f"_LH_GEOM axis_dx.{k}", gv("axis_dx")[k], d["tool_carriages"]["axes"][n])
        pin("_LH_GEOM bay_x0", gv("bay_x0"), d["frame"]["dock_bays_x"][0])
        pin("_LH_GEOM bay_pitch", gv("bay_pitch"), d["frame"]["dock_bays_x"][1] - d["frame"]["dock_bays_x"][0])
        pin("_LH_GEOM dock_entry_x", gv("dock_entry_x"), d["travel"]["X"]["travel_xc"][1])
        yt, xt = d["travel"]["Y"]["travel_yf"], d["travel"]["X"]["travel_xc"]
        pin("[stepper_y] position_max", cp.getfloat("stepper_y", "position_max"), yt[1])
        pin("_LH_SIZE y_max", zv("y_max"), yt[1])
        pin("[stepper_y] position_min", cp.getfloat("stepper_y", "position_min"), yt[0])
        pin("[stepper_y] position_endstop (front end stops)", cp.getfloat("stepper_y", "position_endstop"), yt[0])
        # X travel is family-wide (core): pinned to the BUILT size's CAD; a non-built size whose report still differs from
        # the built one is a CAD report not yet regenerated (warning), not a firmware fault
        xb = family(want)["travel"]["X"]["travel_xc"]
        for i_, k_ in ((0, "position_min"), (1, "position_max")):
            if s != want and abs(xt[i_] - xb[i_]) > 1e-6:
                warn("CAD_SIZES_DISAGREE", f"[stepper_x] {k_}: family_{s} {xt[i_]} != family_{want} {xb[i_]} (family-wide value: the CAD report of {s} is not regenerated yet)")
                pin(f"[stepper_x] {k_} (built size)", cp.getfloat("stepper_x", k_), xb[i_])
            else:
                pin(f"[stepper_x] {k_}", cp.getfloat("stepper_x", k_), xt[i_])
        cad_places = {p.lower(): [v["centre_xy"][0], v["centre_xy"][1], MODULE_STATIONS[v["station"]]]
                      for p, v in d["deck_positions"].items() if not p.startswith("_") and v["station"] in MODULE_STATIONS}
        pin("_LH_SIZE places (CAD module places)", zv("places"), cad_places)
        pin("_LH_SIZE stations", zv("stations"), {p: v["station"] for p, v in d["deck_positions"].items() if not p.startswith("_")})
        co_cad = {}
        if any("carriage B only" in r for r in d["planner_rules"]) and "IG" in d["deck_positions"]:
            x, y = d["deck_positions"]["IG"]["centre_xy"]
            co_cad["ig"] = [x, y, 66.0, 55.0, ["b"]]
        pin("_LH_SIZE carriage_only (planner rule 'carriage B only')", zv("carriage_only"), co_cad)
        # tip parking: the size file's place = the CAD tip_parking block (gen_size_files.tip_parking), plus independent
        # invariants against the CAD deck positions and the firmware tool table
        import gen_size_files    # noqa: E402
        tp_fw = zv("tip_parking")
        pin("_LH_SIZE tip_parking (CAD tip_parking)", tp_fw, gen_size_files.tip_parking(d))
        tpc = d.get("tip_parking") or {}
        for p_, v_ in tp_fw.items():
            dp_ = d["deck_positions"].get(p_.upper(), {})
            if dp_.get("station") != v_[6] or [float(a) for a in dp_.get("centre_xy", [])] != [float(v_[0]), float(v_[1])]:
                fail("TIP_PARKING_VS_CAD", f"size {s}: tip parking {p_} {v_[6]} at {v_[:2]} but CAD deck_positions {p_.upper()} = {dp_.get('station')} at {dp_.get('centre_xy')}")
            if v_[7] * 8 != tpc.get("capacity_tips"):
                fail("TIP_PARKING_VS_CAD", f"size {s}: {v_[7]} eight-channel sets != CAD capacity_tips {tpc.get('capacity_tips')}")
            eight_ = ast.literal_eval(cp.get("gcode_macro _LH_TOOLDEF", "variable_eight"))
            if any(not eight_.get(t_) for t_ in v_[5]) or not v_[5]:
                fail("TIP_PARKING_TOOLS", f"size {s}: tip parking tools {v_[5]} must be eight-channel tools of _LH_TOOLDEF")
            if v_[8] is not None:
                g_ = v_[8]
                xs_ = [x_ for _, e_, sx_ in g_["spots"] for x_ in (e_, sx_)]
                if not all(v_[0] - v_[2] <= x_ <= v_[0] + v_[2] for x_ in xs_):
                    fail("TIP_PARKING_VS_CAD", f"size {s}: a spot or entry x outside the comb box")
                if len(g_["spots"]) != v_[7] or len({e_ for _, e_, _ in g_["spots"]}) != len(tpc.get("groups") or []):
                    fail("TIP_PARKING_VS_CAD", f"size {s}: {len(g_['spots'])} spots / groups vs CAD capacity {v_[7]} sets")
                if not (g_["s_above"] < g_["s_slide"] < g_["s_seat"]) or g_["strip_mm"] <= g_["s_seat"] - g_["s_slide"]:
                    fail("TIP_PARKING_VS_CAD", f"size {s}: s_above < s_slide < s_seat and a strip travel beyond the detent expected")
                if g_["slide_accel"] > 250.0 + 1e-6 or g_["slide_mm_s"] > 10.0 or g_["strip_mm_s"] > 5.0:
                    fail("TIP_PARKING_VS_CAD", f"size {s}: comb speeds above the CAD limits")
        if tpc.get("present") and not tp_fw:
            fail("TIP_PARKING_VS_CAD", f"size {s}: CAD has tip parking {tpc.get('station')} at {tpc.get('place')}, the size file none")
        for con in sorted({v[2] for v in cad_places.values()}):
            for ln in ("a", "b"):
                if not cp.has_section(f"gcode_button safe_{con}_{ln}"):
                    fail("MODULE_SAFE_INPUT", f"size {s}: CAD module connector {con} has no [gcode_button safe_{con}_{ln}]")
        out[s] = {"tip_parking": tp_fw, "pins": len(pins), "mismatches": [k for k, v in pins.items() if v["firmware"] != v["cad"] and not (isinstance(v["cad"], (int, float)) and abs(float(v["firmware"]) - float(v["cad"])) <= 1e-6)],
                  "dock_yf": zv("dock_yf"), "y_max": zv("y_max")}
    return out


def pin_name(val):
    return re.sub(r"[\^!~]", "", val.split()[0])


sysd_global = {}


def enclosure_checks(sysd, cab, fail, warn):
    """Control compartment and device positions against the family CAD (cad/lh1/machine_family/scene.py: COMPARTMENT, the
    CBP-F / CBP2-F plate boxes, the base and feet).  The family CAD does not model the e-stop, reset, door sensor, inlet or
    PE stud: they are checked to lie inside the S enclosure envelope and reported as not in the CAD."""
    out = {}
    src = (ROOT / "cad/lh1/machine_family/scene.py").read_text()
    m = re.search(r"COMPARTMENT = dict\(x0=(-?[\d.]+), x1=(-?[\d.]+), y0=(-?[\d.]+), y1=(-?[\d.]+),", src)
    if not m:
        fail("COMPARTMENT_NOT_IN_CAD", "no COMPARTMENT in machine_family/scene.py")
        return out
    cad = [float(v) for v in m.groups()]
    cx = cab["compartment"]
    out["compartment_cad_xy"] = cad
    if [float(v) for v in cx["x"] + cx["y"]] != cad:
        fail("COMPARTMENT_MISMATCH", f"control_bay.yaml compartment {cx['x']} {cx['y']} vs CAD {cad}")
    for pl, pat in (("CBP", r'"CBP-F_control_bay_plate[^"]*", B\((-?[\d.]+), (-?[\d.]+),'), ("CBP2", r'"CBP2-F_rear_electronics_plate[^"]*", B\((-?[\d.]+), (-?[\d.]+),')):
        mm = re.search(pat, src)
        if not mm:
            fail("PLATE_NOT_IN_CAD", pl)
            continue
        cxr = [float(mm.group(1)), float(mm.group(2))]
        if [float(v) for v in cab["plates"][pl]["x"]] != cxr:
            fail("PLATE_MISMATCH", f"{pl} x {cab['plates'][pl]['x']} vs CAD {cxr}")
        out[pl] = {"cad_x": cxr, "layout_x": cab["plates"][pl]["x"]}
    vol = re.search(r'"CBX-F_device_volume_(\d+)', src)
    if vol and any(abs(p_["height_available_mm"] - float(vol.group(1))) > 0.5 for k_, p_ in cab["plates"].items() if k_ in ("CBP", "CBP2")):
        fail("CBP_HEIGHT", f"plate height != CAD device volume {vol.group(1)} mm")
    for n, d in sysd["devices"].items():
        if d["group"] == "control_bay" and d["kind"] not in ("inlet", "fan"):
            x, y, z = d["pos"]
            if not (cx["x"][0] <= x <= cx["x"][1] and cx["y"][0] <= y <= cx["y"][1] and cx["z"][0] <= z <= cx["z"][1]):
                fail("BAY_DEVICE_OUTSIDE", f"{n} at {d['pos']} is not inside the control compartment")
    env = family("S")["envelope_mm"]
    for n in ("ESTOP", "RESET_PB", "DOOR_SW", "MAINS_INLET", "PE_STUD"):
        x, y, z = sysd["devices"][n]["pos"]
        if not (env["x"][0] <= x <= env["x"][1] and env["y"][0] <= y <= env["y"][1] and env["z"][0] <= z <= env["z"][1]):
            fail("DEVICE_OUTSIDE_ENVELOPE", f"{n} at {[x, y, z]} outside the S envelope {env['x']} {env['y']} {env['z']}")
    # panel parts against the CAD services_interface envelopes (21 Nov 2026: CR-F3 answered)
    si = family(sysd["family"]["build_size"]).get("services_interface") or {}
    oc, bw = si.get("operator_controls") or {}, next((v for k, v in si.items() if k.startswith("back_wall_cut_outs")), {})
    ap = (si.get("airflow") or {}).get("parts") or {}
    cad_env = {"ESTOP": next((v["envelope"] for k, v in oc.items() if "e_stop" in k), None),
               "RESET_PB": next((v["envelope"] for k, v in oc.items() if "reset" in k), None),
               "DOOR_SW": next((v["envelope"] for k, v in oc.items() if k.startswith("CTL-F_door_sensor_coded")), None),
               "MAINS_INLET": (bw.get("mains_inlet_IEC_C14_fused_switched") or {}).get("envelope"),
               "PE_STUD": (bw.get("earth_stud_M6") or {}).get("envelope"),
               "CAB_FAN1": next((v for k, v in ap.items() if "fan_92mm_1" in k), None),
               "CAB_FAN2": next((v for k, v in ap.items() if "fan_92mm_2" in k), None),
               "CTB_FANCON": (si.get("CTB-1_fan_connector") or {}).get("envelope")}
    for k_, v_ in ((si.get("tip_bin_full_beam") or {}).get("parts") or {}).items():
        if "emitter" in k_:
            cad_env["BIN_TX"] = v_
        elif "receiver" in k_:
            cad_env["BIN_RX"] = v_
    out["panel_parts_vs_cad"] = {}
    for n, e in cad_env.items():
        if e is None:
            fail("PANEL_PART_NOT_IN_CAD", f"{n}: no envelope in the CAD services_interface block")
            continue
        if n not in sysd["devices"]:
            fail("PANEL_PART_NO_DEVICE", f"CAD envelope for {n} but no such device in system.yaml")
            continue
        x, y, z = sysd["devices"][n]["pos"]
        ok = e[0] - 0.5 <= x <= e[1] + 0.5 and e[2] - 0.5 <= y <= e[3] + 0.5 and e[4] - 0.5 <= z <= e[5] + 0.5
        out["panel_parts_vs_cad"][n] = {"pos": [x, y, z], "cad": e, "ok": ok}
        if not ok:
            fail("PANEL_PART_VS_CAD", f"{n} at {[x, y, z]} outside its CAD envelope {e}")
    # compartment air: everything in the sealed compartment leaves through its own intake fans (CAD services_interface airflow)
    heat = sum(it["heat_w"] for it in cab["items"])
    need_cfm = heat / (1.2 * 1005 * 10.0) * 2118.88
    fan = cab["compartment"]["intake_fan"]
    have_cfm = fan["through_grilles_cfm"]
    one_cfm = fan.get("one_failed_cfm", have_cfm)
    air_cad = si.get("airflow") or {}
    out["air"] = {"heat_w": round(heat, 1), "cad_heat_w": air_cad.get("heat_W"), "needed_cfm_10K": round(need_cfm, 1), "fan_cfm_through_grilles": have_cfm,
                  "rise_K_with_fans": round(heat / (1.2 * 1005 * have_cfm / 2118.88), 1),
                  "rise_K_one_fan_failed": round(heat / (1.2 * 1005 * one_cfm / 2118.88), 1), "ratio": round(have_cfm / need_cfm, 2)}
    if air_cad and (abs(air_cad.get("provided_CFM", -1) - have_cfm) > 0.05 or abs(air_cad.get("one_fan_failed_CFM", -1) - one_cfm) > 0.05):
        fail("COMPARTMENT_AIRFLOW_VS_CAD", f"control_bay.yaml fans {have_cfm} / one failed {one_cfm} CFM != CAD {air_cad.get('provided_CFM')} / {air_cad.get('one_fan_failed_CFM')}")
    if need_cfm * 1.5 > have_cfm:
        open_ = [c["id"] for c in sysd.get("open_cad_requests", []) if c.get("chain") == "compartment_air"]
        (warn if open_ else fail)("COMPARTMENT_AIRFLOW", f"compartment {heat:.0f} W needs {need_cfm:.1f} CFM for 10 K (x 1.5 = {need_cfm * 1.5:.1f}); the {fan.get('count', 1)} x {fan['size_mm']} mm fans give {have_cfm} CFM ({out['air']['rise_K_with_fans']} K rise)" + (f" [open CAD request {open_[0]}]" if open_ else ""))
    if one_cfm < need_cfm:
        wired_ = all(any(r_["to_device"] == "XDB5" and r_["from_device"] == f_ and "rotation detect" in r_["signal"] for r_ in csv.DictReader(open(HERE / "wiring.csv"))) for f_ in ("CAB_FAN1", "CAB_FAN2"))
        warn("COMPARTMENT_ONE_FAN", f"one compartment fan failed: {one_cfm} CFM for {heat:.0f} W = {out['air']['rise_K_one_fan_failed']} K rise (> 10 K); "
             + ("each fan's rotation-detect output is wired (io PF0 / PF1): a stopped fan is reported and LH_THERM_ON refused" if wired_ else "no fan-fail signal is wired") + " (class figures, UNVERIFIED)")
    return out


def fs_syr_cap(cp, td, upm):
    """largest single dispense the SYR first stop allows (uL)"""
    return ast.literal_eval(cp.get(td, "variable_first_stop"))["SYR"] * upm


def mechanism_checks(cp, fail, warn):
    """printer.cfg against the CAD (revision L): homing direction vs end-stop side, rotation_distance vs drive train,
    Z travel / dock / pusher numbers vs params.py L_* and the head_l report (values parsed, never typed here).
    Revision L: the fixed-head plunger, comb (K7) and per-tool travel checks of revision W are retired with the fixed
    head (ADR-010); their rev L equivalents are the pusher stroke table, dock and Z-travel checks below."""
    frame = json.loads((ROOT / "cad/lh1/frame_gantry/out/report.json").read_text())
    head = json.loads((ROOT / "cad/lh1/head/out/report.json").read_text())
    head_l = json.loads((ROOT / "cad/lh1/head_l/out/report.json").read_text())
    spec = importlib.util.spec_from_file_location("p_mech", ROOT / "cad" / "lh1" / "params.py")
    P = importlib.util.module_from_spec(spec); spec.loader.exec_module(P)
    res = {}
    desc = {p["name"]: p.get("description", "") for p in frame["parts"]}
    sides = {}
    for name, d in desc.items():
        m = re.search(r"([XY]) end-stop \(([a-z-]+)\)", d)
        if m:
            sides.setdefault(m.group(1), []).append(m.group(2))
    def expect(sec, positive):
        pos_dir = cp.getboolean(sec, "homing_positive_dir", fallback=False)
        pe = cp.getfloat(sec, "position_endstop")
        want_pe = cp.getfloat(sec, "position_max") if positive else cp.getfloat(sec, "position_min")
        ok = pos_dir == positive and abs(pe - want_pe) < 1e-6
        if not ok:
            fail("HOMING_DIRECTION", f"[{sec}] homes {'+' if pos_dir else '-'} with position_endstop {pe}; the CAD end stop is on the {'+' if positive else '-'} side")
        return ok
    if "X" in sides:
        res["x_homing_ok"] = expect("stepper_x", "right" in sides["X"][0])
    if "Y" in sides:
        res["y_homing_ok"] = expect("stepper_y", not all(s.startswith("front") for s in sides["Y"]))
    want = {}
    pul = next((b for b in frame["bought_parts"] if re.search(r"GT2 pulley (\d+)T", b["item"])), None)
    if pul:
        teeth = int(re.search(r"GT2 pulley (\d+)T", pul["item"]).group(1))
        for s in ("stepper_x", "stepper_y", "stepper_y1"):
            want[s] = (2.0 * teeth, f"GT2 2 mm x {teeth}T (frame bought_parts)")
    zl = float(re.search(r"Tr8x(\d+(?:\.\d+)?)", head["checks"]["z_thrust"]["screw"]).group(1))
    for s in ("stepper_z", "manual_stepper z_a", "manual_stepper z_b"):
        want[s] = (zl, f"Tr8x{zl:g} (Z drive unchanged from G5, study 7.1; head report z_thrust)")
    g = re.search(r"(\d+)T\s*->\s*(\d+)T.*Tr8x(\d+(?:\.\d+)?)", head["checks"]["gripper_force"]["drive"])
    if g:
        want["manual_stepper gripper"] = (float(g.group(3)) * int(g.group(1)) / int(g.group(2)), "head report gripper_force.drive")
    for s in ("manual_stepper pusher_a", "manual_stepper pusher_b"):
        want[s] = (sysd_global["plunger_motor"]["lead_mm"], "system.yaml plunger_motor.lead_mm (captive linear pusher)")
    rd = {}
    for sec, (v, src) in want.items():
        got = cp.getfloat(sec, "rotation_distance")
        rd[sec] = {"cfg": got, "cad": round(v, 4), "source": src}
        if abs(got - v) > 1e-6:
            fail("ROTATION_DISTANCE", f"[{sec}] rotation_distance {got} but the drive gives {v:g} ({src})")
    res["rotation_distance"] = rd
    trav = {}
    def cmp(label, got, cad, src):
        trav[label] = {"cfg": got, "cad": cad, "source": src}
        if abs(got - cad) > 1e-6:
            fail("TRAVEL_VS_MODULE", f"{label} = {got} but {src} = {cad}")
    for sec in ("stepper_z", "manual_stepper z_a", "manual_stepper z_b"):
        cmp(f"[{sec}] position_max", cp.getfloat(sec, "position_max"), P.L_Z_TRAVEL, "params L_Z_TRAVEL")
    for sec in ("manual_stepper z_a", "manual_stepper z_b"):
        cmp(f"[{sec}] position_min", cp.getfloat(sec, "position_min"), -P.Z_EJECT_TRAVEL, "-params Z_EJECT_TRAVEL")
    cmp("[stepper_z] position_min (gripper: no over-travel)", cp.getfloat("stepper_z", "position_min"), 0.0, "rule")
    m = re.search(r"\((\d+(?:\.\d+)?)-(\d+(?:\.\d+)?) mm stroke", head_l["tool_interface"]["plunger"])
    pmax = float(m.group(2)) if m else None
    for sec in ("manual_stepper pusher_a", "manual_stepper pusher_b"):
        if pmax is None:
            fail("CAD_PUSHER_MISSING", "head_l tool_interface.plunger has no '0-35 mm stroke'")
        else:
            # ADR-014: the actuator's soft limit (CAD 0.5 .. 37.5 of actuator stroke = 0 .. 37.0 from the switch) bounds Klipper;
            # the tool interface stroke (0-35) bounds every work move (_LH_TOOLDEF pusher_max, _LH_PUSHER_MOVE)
            sl_ = sysd_global["plunger_motor"]["soft_limits_mm"]
            cmp(f"[{sec}] position_max", cp.getfloat(sec, "position_max"), sl_[1] - sl_[0], "CAD pusher soft limits (plunger_motor.soft_limits_mm)")
    if pmax is not None:
        cmp("_LH_TOOLDEF pusher_max", float(cp.get("gcode_macro _LH_TOOLDEF", "variable_pusher_max")), pmax, "head_l tool_interface pusher stroke")
    # tool table vs head_l tools: blow-out = tool stroke (capped by the pusher), first stop below it, SYR disarmed if it does not fit
    td = "gcode_macro _LH_TOOLDEF"
    var = lambda k: ast.literal_eval(cp.get(td, f"variable_{k}"))
    blow, fs, armed, codes = var("blowout"), var("first_stop"), var("armed"), var("codes")
    hi = float(cp.get(td, "variable_contact_hi"))
    tools = {k: dict(v) for k, v in head_l["tools"].items()}
    syr = sysd_global["family"]["syringe_tool"]
    cra = json.loads((ROOT / "cad/lh1/machine_family/out/cr_answers.json").read_text())["open_question_syringe_stroke"]
    if "DIRECTLY" not in cra["redesign"] or "inside the 35 mm pusher stroke" not in cra["redesign"]:
        fail("SYR_CAD", "cr_answers no longer describes the direct-drive syringe inside the 35 mm stroke: re-decide SYR")
    tools["SYR"]["stroke"] = syr["stroke_mm"]      # family: direct drive (cr_answers); head_l still lists the retired 60 mm lever stroke
    hi_tool = ast.literal_eval(cp.get(td, "variable_contact_hi_tool")) if cp.has_option(td, "variable_contact_hi_tool") else {}
    # SYR against the CAD family syringe_tool block (built size): stroke = plunger travel, contact window = the CAD's, both
    # inside the CAD pusher stroke; the volume table from the bore (fill / travel); a return spring is specified
    syc = family(sysd_global["family"]["build_size"]).get("syringe_tool") or {}
    res["syringe_vs_cad"] = {"stroke": (syr["stroke_mm"], syc.get("plunger_travel_mm")), "contact_window": (hi_tool.get("SYR"), syc.get("contact_window_mm (pusher travel before it meets the plunger)")),
                             "pusher": (pmax, syc.get("pusher_stroke_mm")), "fill_ml": syc.get("fill_mL"), "return_spring": bool(syc.get("return_spring"))}
    if not syc:
        fail("SYR_CAD", "family report has no syringe_tool block")
    else:
        cw = syc.get("contact_window_mm (pusher travel before it meets the plunger)")
        if abs(syr["stroke_mm"] - syc["plunger_travel_mm"]) > 1e-6 or hi_tool.get("SYR") != cw or syr.get("contact_window_mm") != cw:
            fail("SYR_CAD", f"SYR stroke {syr['stroke_mm']} / contact window {hi_tool.get('SYR')} (system.yaml {syr.get('contact_window_mm')}) != CAD plunger travel {syc['plunger_travel_mm']} / window {cw}")
        if syc["pusher_stroke_mm"] != pmax:
            fail("SYR_CAD", f"CAD syringe pusher stroke {syc['pusher_stroke_mm']} != head_l {pmax}")
        upm_ = ast.literal_eval(cp.get(td, "variable_ul_per_mm"))["SYR"]
        if abs(upm_ * syc["plunger_travel_mm"] / 1000.0 - syc["fill_mL"]) > 0.01:
            fail("SYR_CAD", f"SYR ul_per_mm {upm_} x {syc['plunger_travel_mm']} mm != CAD fill {syc['fill_mL']} mL")
        if ast.literal_eval(cp.get(td, "variable_vmax"))["SYR"] > fs_syr_cap(cp, td, upm_):
            fail("SYR_CAD", "SYR vmax larger than the first stop allows")
        if not syc.get("return_spring"):
            fail("SYR_CAD", "CAD syringe_tool has no return spring: the plunger would not follow the pusher up")
    if optional_ := ast.literal_eval(cp.get(td, "variable_optional")):
        for tn_ in optional_:
            if ast.literal_eval(cp.get(td, "variable_commissioned")).get(tn_):
                warn("OPTIONAL_TOOL_COMMISSIONED", f"{tn_} is commissioned: only after the rig tests")
    tab = {}
    for tn, tv in tools.items():
        if tn not in blow:
            fail("TOOL_TABLE", f"_LH_TOOLDEF has no entry for head_l tool {tn}")
            continue
        fits = tv["stroke"] + hi_tool.get(tn, hi) <= (pmax or 0) + 1e-9
        tab[tn] = {"stroke": tv["stroke"], "blowout": blow[tn], "fits_pusher": fits, "armed": armed[tn]}
        if fits and abs(blow[tn] - tv["stroke"]) > 1e-6:
            fail("TOOL_TABLE", f"{tn}: blow-out {blow[tn]} != head_l stroke {tv['stroke']}")
        if not fits and armed[tn]:
            fail("TOOL_TABLE", f"{tn}: stroke {tv['stroke']} + contact {hi_tool.get(tn, hi)} > pusher {pmax} mm but the tool is armed")
        if fs[tn] >= blow[tn]:
            fail("TOOL_TABLE", f"{tn}: first stop {fs[tn]} not below the blow-out {blow[tn]}")
    res["tool_table"] = tab
    # Hall codes: distinct, never 0, EVEN weight (one lost bit can never read as another tool or as 'empty')
    vals = list(codes.values())
    if len(set(vals)) != len(vals) or 0 in vals or any(bin(v).count("1") % 2 for v in vals) or any(v > 15 for v in vals):
        fail("TOOL_CODES", f"tool codes must be distinct 4-bit even-weight values, got {codes}")
    if set(codes) != set(tools):
        fail("TOOL_CODES", f"code table {sorted(codes)} != head_l tools {sorted(tools)}")
    # dock and geometry against params L_*
    ge = "gcode_macro _LH_GEOM"
    gf = lambda k: float(cp.get(ge, f"variable_{k}"))
    cmp("_LH_GEOM bay_x0", gf("bay_x0"), P.L_DOCK_X0, "params L_DOCK_X0")
    cmp("_LH_GEOM bay_pitch", gf("bay_pitch"), P.L_DOCK_PITCH, "params L_DOCK_PITCH")
    cmp("_LH_GEOM dock_s", gf("dock_s"), P.L_DOCK_S, "params L_DOCK_S")
    fz_ = family(sysd_global["family"]["build_size"])
    cmp("_LH_SIZE dock_yf + _LH_GEOM noz_dy", float(cp.get("gcode_macro _LH_SIZE", "variable_dock_yf")) + gf("noz_dy"), fz_["frame"]["dock_axis_y"], "cad machine_family frame.dock_axis_y (built size)")
    cmp("_LH_GEOM dock_entry_x", gf("dock_entry_x"), P.L_XC_RANGE[1], "params L_XC_RANGE right end (rev_l entry_rule)")
    ax = ast.literal_eval(cp.get(ge, "variable_axis_dx"))
    for k, n in (("a", "carriage_A"), ("b", "carriage_B"), ("g", "gripper")):
        cmp(f"_LH_GEOM axis_dx.{k}", ax[k], P.L_AXIS_X[n], f"params L_AXIS_X {n}")
    if not P.L_DOCK_S < cp.getfloat("manual_stepper z_a", "position_max"):
        fail("DOCK_DEPTH", "dock seat stroke must lie inside the carriage travel")
    for i in range(8):
        bx = gf("bay_x0") + i * gf("bay_pitch")
        for k in ("a", "b"):
            xc = bx - ax[k]
            if not (cp.getfloat("stepper_x", "position_min") <= xc <= cp.getfloat("stepper_x", "position_max")):
                fail("DOCK_REACH", f"carriage {k} cannot reach bay {i + 1} (X {xc})")
    res["module_places"] = sorted(ast.literal_eval(cp.get("gcode_macro _LH_SIZE", "variable_places")))     # pinned to the CAD in family_checks
    res["travel"] = trav
    return res


def review_checks(cp, sysd, rows, fail, warn):
    """Reviews 2026-11-13 (A12, B, V, W) carried to revision L, plus the revision L guards (ADR-010, head_l safety)."""
    out = {}
    # 23 Nov 2026 (V2 / A2): the public motion and pipetting commands check their parameters and hand over (after an M400
    # or after the guard) to an internal macro that holds the state checks: a public command's body is read with its hand-off
    handoff = {"gcode_macro G0": "gcode_macro _LH_GMOVE", "gcode_macro G1": "gcode_macro _LH_GMOVE",
               "gcode_macro MANUAL_STEPPER": "gcode_macro _LH_ZMOVE", "gcode_macro LH_PICK_TIP": "gcode_macro _LH_PICK_TIP_DO",
               "gcode_macro LH_ASPIRATE": "gcode_macro _LH_ASPIRATE_DO", "gcode_macro LH_DISPENSE": "gcode_macro _LH_DISPENSE_DO",
               "gcode_macro LH_PREPARE": "gcode_macro _LH_PREPARE_DO"}

    def body(sec):
        own = cp.get(sec, "gcode") if cp.has_section(sec) and cp.has_option(sec, "gcode") else ""
        h_ = handoff.get(sec)
        return own + ("\n" + cp.get(h_, "gcode") if h_ and cp.has_section(h_) and cp.has_option(h_, "gcode") else "")
    # --- homing order (rev L): power-up read, gripper Z (G28 Z), carriages, chain, back-off of empty carriages, pushers, chain, X then Y
    hb = body("homing_override")
    if not cp.has_section("homing_override"):
        fail("NO_HOMING_OVERRIDE", "every G28 must raise all Z axes first: [homing_override] missing")
    seq = [hb.find(k) for k in ("_LH_LIFT_HOME", "_LH_POWERUP_READ", "G28 Z", "MANUAL_STEPPER_BASE STEPPER=z_{c} SET_POSITION", "_LH_REQUIRE_TOOLS_UP", "_LH_ZBACKOFF C={c}", "_LH_PUSHER_HOME C={c}", "_LH_TOOLS_LOCKED", "G28 X", "G28 Y")]
    order_ok = all(x >= 0 for x in seq) and seq == sorted(seq) and "STEPPER=pusher_" not in hb
    # build profile (24 Nov 2026): the core's loops run over _LH_PROFILE carriages; the built size must be the FULL profile
    pfv = {k_: cp.get("gcode_macro _LH_PROFILE", "variable_" + k_, fallback=None) for k_ in ("name", "absent", "carriages", "fixed_tools")} if cp.has_section("gcode_macro _LH_PROFILE") else {}
    if pfv != {"name": '"FULL"', "absent": "[]", "carriages": '["a", "b"]', "fixed_tools": "{}"}:
        fail("PROFILE_NOT_FULL", f"the size build must run the FULL profile (every device): _LH_PROFILE {pfv}")
    out["homing_order"] = {"positions": seq, "ok": order_ok}
    if not order_ok:
        fail("HOMING_ORDER", "homing_override must: home the dock lift bar DOWN first (ADR-014: it only lets a latch close), read both carriages, home gripper Z, carriages, check the chain, back off empty carriages, home the pushers through _LH_PUSHER_HOME (<= 0.50 A), check carriages, then X before Y")
    if "homed VALUE=1" not in hb or "homed VALUE=0" not in hb:
        fail("HOMED_FLAG", "homing_override must clear 'homed' at the start and set it at the end")
    if 'mounted_" ~ c] == ""' not in body("gcode_macro _LH_ZBACKOFF") or "inverted_probe" not in body("gcode_macro _LH_ZBACKOFF"):
        fail("POWERUP_TOOL", "_LH_ZBACKOFF must back off only an EMPTY carriage (a carriage with a tool is homed Z up only)")
    pr = body("gcode_macro _LH_POWERUP_READ")
    if not all(k in pr for k in ("HALF_LOCK", "cable_", "VARIABLE=tips_", "VARIABLE=nozzle_ok_")):
        fail("POWERUP_TOOL", "_LH_POWERUP_READ must refuse a half-lock / lost cable and mark a found tool as tips-possibly-on and nozzle-not-checked")
    for sec in ("homing_override", "gcode_macro LH_HOME_ALL"):
        if re.search(r"STEPPER=gripper\b", body(sec)):
            fail("HOMING_MOVES_GRIPPER", f"[{sec}] moves the gripper jaws: a held plate would drop")
    st = "gcode_macro _LH_STATE"
    if not (cp.getint(st, "variable_homed") == 0 and cp.getint(st, "variable_grip") == 0 and cp.get(st, "variable_hold").strip('"') == ""):
        fail("STATE_DEFAULTS", "_LH_STATE must start with homed 0, grip 0 (UNKNOWN) and no hold after every restart")
    for name, new in (("G0", "G0.1"), ("G1", "G1.1"), ("MANUAL_STEPPER", "MANUAL_STEPPER_BASE")):
        if cp.get(f"gcode_macro {name}", "rename_existing", fallback="") != new:
            fail("NO_MOTION_GUARD", f"{name} must be overridden (rename_existing: {new}) with the homed guard")
    motion = re.compile(r"MANUAL_STEPPER|\bG0\b|\bG1\b|G0\.1|G1\.1|_LH_PUSHER_MOVE|_LH_CARRIAGE_Z|LH_PREPARE|_LH_DOCK_ENTER|LH_DOCK_LEAVE")
    guarded = []
    for sec in cp.sections():
        if not sec.startswith("gcode_macro "):
            continue
        name = sec.split(" ", 1)[1]
        if name.startswith("_") or name in ("LH_HOME_ALL",):
            continue
        b = body(sec)
        if motion.search(b):
            if ".homed" not in b and "homed %}" not in b and "_LH_EXCH_GUARD" not in b and "_LH_PIPETTE_GUARD" not in b:
                fail("MACRO_NOT_HOMED_GUARDED", f"{name} moves axes but does not refuse when not homed since restart/e-stop")
            else:
                guarded.append(name)
    out["homed_guarded_macros"] = sorted(guarded)
    for g_ in ("_LH_EXCH_GUARD", "_LH_PIPETTE_GUARD"):
        if ".homed" not in body(f"gcode_macro {g_}"):
            fail("MACRO_NOT_HOMED_GUARDED", f"{g_} must refuse when not homed")
    if "grip == 0" not in body("gcode_macro LH_GRIPPER") or "CONFIRM" not in body("gcode_macro LH_GRIPPER_RELEASE"):
        fail("GRIPPER_UNKNOWN_STATE", "LH_GRIPPER must refuse while the gripper state is UNKNOWN; LH_GRIPPER_RELEASE must need CONFIRM=1")
    # --- TC-1 guard (family F1): zone must CONTAIN the CAD family keepouts (machine_family report 'keepouts', same in every
    # size): the above-deck parts (z1 > 0) of thermocycler_TC-1, the lid travel corridor and the lid park union; min z = K1
    ko = yaml.safe_load((ROOT / "cad/lh1/assembly/out/keepouts.yaml").read_text())
    rules = {r["id"].split("_")[0]: r for r in ko["rules"]}
    zmin = float(re.search(r"below z (\d+(?:\.\d+)?)", rules["K1"]["rule"]).group(1))
    f1_ = family(sysd["family"]["build_size"])["deck_positions"]["F1"]
    if f1_["station"] != "TC1":
        fail("TC_GUARD_VS_CAD", f"family F1 is {f1_['station']}, not TC-1")
    boxes = {}
    for s_ in fw_config.SIZES:
        kof = family(s_).get("keepouts") or {}
        tc_ = next((v for k_, v in kof.items() if k_.startswith("thermocycler")), None)
        if not tc_:
            fail("TC_KEEPOUT_MISSING", f"family_{s_} keepouts has no thermocycler entry")
            continue
        if tc_.get("place") != "F1":
            fail("TC_GUARD_VS_CAD", f"family_{s_} keepouts thermocycler place {tc_.get('place')} != F1")
        b_ = {f"TC-1 {k_}": v for k_, v in tc_["parts"].items() if v[5] > 0}
        b_.update({k_.split(" (")[0]: v["box"] for k_, v in kof.items() if k_.startswith("TC-1_lid_travel_corridor")})
        b_.update({k_: v["union"] for k_, v in kof.items() if k_.startswith("TC-1_lid_park")})
        if s_ == sysd["family"]["build_size"]:
            boxes = b_
        elif b_ != boxes and boxes:
            fail("TC_KEEPOUT_SIZES", f"family_{s_} TC-1 keepouts differ from the built size (the CAD says rows F, A, B never move)")
    k2 = {"x": [min(v[0] for v in boxes.values()), max(v[1] for v in boxes.values())],
          "y": [min(v[2] for v in boxes.values()), max(v[3] for v in boxes.values())]} if boxes else {"x": [0, 0], "y": [0, 0]}
    g = "gcode_macro _LH_TC_GUARD"
    gv = lambda k: cp.getfloat(g, f"variable_{k}")
    c1 = {"min_tool_z": (gv("min_tool_z"), zmin), "zone_x": ([gv("zone_x0"), gv("zone_x1")], k2["x"]), "zone_y": ([gv("zone_y0"), gv("zone_y1")], k2["y"]),
          "cad_boxes": sorted(boxes)}
    out["c1_guard_vs_keepouts"] = c1
    if gv("min_tool_z") != zmin or gv("zone_x0") > k2["x"][0] or gv("zone_x1") < k2["x"][1] or gv("zone_y0") > k2["y"][0] or gv("zone_y1") < k2["y"][1]:
        fail("C1_GUARD_VS_CAD", f"_LH_TC_GUARD zone must contain the CAD family keepouts of TC-1 (above deck, lid corridor, lid park) and use K1's z: {c1}")
    strip_top = max((v[5] for k_, v in boxes.items() if "guide_strip" in k_), default=0.0)
    if strip_top + 2.0 > zmin:
        warn("TC_STRIP_FIXED_OBSTACLE", f"the TC-1 lid guide strip (CAD keepouts, top z {strip_top:g}) is a fixed obstacle above the lid guard's z {zmin:g}: the firmware does not guard fixed obstacles (planner keep-out)")
    if 'gcode_button lid_parked' not in body(g) or "tip_len" not in body(g):
        fail("C1_GUARD_NO_LID", "_LH_TC_GUARD must read the lid PARKED switch and count the mounted tool's tip length")
    spec_ = importlib.util.spec_from_file_location("p_c1", ROOT / "cad" / "lh1" / "params.py")
    P_ = importlib.util.module_from_spec(spec_); spec_.loader.exec_module(P_)
    nt = ast.literal_eval(cp.get("gcode_macro _LH_TOOLDEF", "variable_noz_top"))
    for tn, v in nt.items():
        want_ = P_.L_NOZZLE_TOP_Z["p1000" if tn == "S1000" else "standard"]
        if v != want_:
            fail("C1_GUARD_VS_CAD", f"_LH_TOOLDEF noz_top {tn} {v} != params L_NOZZLE_TOP_Z {want_}")
    # --- hardware lid permissive / tools-up chain: LOG7 -> Z_UP x3 in series (system.yaml interlocks.tools_up_chain) -> ZC
    ups = sysd["interlocks"]["tools_up_chain"]
    series = [r for r in rows if "SAFETY lid permissive chain" in r["signal"] and ("feed" in r["signal"] or r["signal"].endswith("chain") or "chain end" in r["signal"])]
    deg = {}
    for r in series:
        for d in (r["from_device"], r["to_device"]):
            deg[d] = deg.get(d, 0) + 1
    zs = [d for d in sysd["devices"] if d.startswith("Z_UP_")]
    ok = sorted(zs) == sorted(ups) and len(ups) == 3 and all(deg.get(u) == 2 for u in ups) and deg.get("XT") == 2 and len(series) == len(ups) + 1
    out["lid_permissive_chain"] = {"switches": ups, "cables": [r["cable_id"] for r in series], "series_ok": ok}
    if not ok:
        fail("LID_PERMISSIVE_CHAIN", f"XT LOG7 -> {ups} in series -> XT ZC is not a single series chain of all three Z axes")
    # --- contactor mirror contacts in a safety-relay feedback loop
    srs = sysd["safety_relays"]
    members = {m: sr for sr, v in srs.items() for m in v["feedback_loop"]}
    for dname, d in sysd["devices"].items():
        if d["kind"] in ("contactor", "relay") and dname not in members:
            fail("CONTACTOR_NOT_MONITORED", f"{dname} removes energy but has no mirror contact in a safety-relay feedback loop")
    for m, sr in members.items():
        sig = "SAFETY reset" if m == "RESET_PB" else f"feedback loop {sr}"
        n = sum(1 for r in rows if m in (r["from_device"], r["to_device"]) and sig in r["signal"])
        if n < (1 if m == "RESET_PB" else 2):
            fail("FEEDBACK_LOOP_WIRING", f"{m}: mirror contact not wired in and out of the {sr} feedback loop ({n} cables)")
    for sr, v in srs.items():
        if not re.search(r"\d{6}", v["part"]):
            fail("SAFETY_RELAY_PART", f"{sr}: no order number")
        for t, uses in v["terminals"].items():
            if str(t) not in map(str, v["allowed_terminals"]):
                fail("SAFETY_RELAY_TERMINAL", f"{sr}: terminal {t} does not exist on {v['part']}")
            if len(uses) > 1 and str(t) not in map(str, v.get("shared_by_design", {})):
                fail("SAFETY_RELAY_TERMINAL_SHARED", f"{sr}.{t} carries {uses}")
        for r in rows:
            for side in ("from", "to"):
                if r[f"{side}_device"] == sr:
                    for t in re.split(r"[,\s]+", r[f"{side}_pins"]):
                        if t and t not in map(str, v["terminals"]):
                            fail("SAFETY_RELAY_WIRING", f"{r['cable_id']}: {sr} terminal {t} not in system.yaml safety_relays.{sr}.terminals")
    fmap = sysd["firmware_map"]
    for dname, d in sysd["devices"].items():
        if d["kind"] == "motor" and "closed-loop" in d["desc"]:
            secs = [k for k, v in fmap.items() if v == dname and k.startswith("gcode_button servo_alarm")]
            if not secs or not all("M112" in cp.get(s_, "press_gcode", fallback="") for s_ in secs):
                fail("SERVO_ALARM", f"{dname}: alarm output not wired to an input that stops the machine")
    dj = sysd["octopus_diag_jumpers"]
    used_pins = {re.sub(r"[\^!~]", "", cp.get(s_, k).split()[0]).split(":")[-1] for s_ in cp.sections() for k, _ in cp.items(s_) if k.endswith("pin") and (":" not in cp.get(s_, k) or cp.get(s_, k).replace("^", "").replace("!", "").startswith("io:"))}
    for sock in dj["fitted"]:
        if dj["diag_to_stop"][sock] in used_pins:
            fail("DIAG_JUMPER", f"DIAG jumper on socket {sock} shorts the driver onto {dj['diag_to_stop'][sock]} (main or io board)")
    out["diag_jumpers_fitted"] = dj["fitted"]
    return review_checks_2(cp, sysd, rows, fail, warn, out, body)



def review_checks_2(cp, sysd, rows, fail, warn, out, body):
    # --- over-voltage on the MOTION bus without suppressors (ADR-003): every event, no-TVS model, against the
    # lowest-rated part on the bus (system.yaml overvoltage_motion)
    ov = sysd["overvoltage_motion"]
    v1, marg = ov["supply_max_v"], ov["margin_v"]
    lim_part = min(ov["parts_on_bus"], key=lambda k_: ov["parts_on_bus"][k_]["limit_v"])
    v_allow = ov["parts_on_bus"][lim_part]["limit_v"] - marg
    c_min = sum(b_["uf"] * (1 - b_["tol"]) for b_ in ov["bulk"]) * 1e-6
    fc = ov["feed_cable"]
    frow = next((r for r in rows if r["cable_id"] == fc.get("cable_id")), None)
    if frow is None or abs(int(frow["length_mm_estimate"]) / 1000.0 - fc["length_m"]) > 0.051:
        fail("OVERVOLTAGE_STALE", f"overvoltage_motion feed {fc['length_m']} m != wiring.csv {fc.get('cable_id')} {frow and frow['length_mm_estimate']} mm: recompute")
    lh, rs = fc["length_m"] * fc["l_uh_per_m"] * 1e-6, fc["length_m"] * 2 * fc["r_ohm_per_m"]
    zeta = rs / 2 * math.sqrt(c_min / lh)
    # closure (contactor making onto the discharged bulk): series R-L-C step response peak
    v_cl = v1 * (1 + math.exp(-math.pi * zeta / math.sqrt(1 - zeta ** 2))) if zeta < 1 else v1
    ev_rep = [f"closure: L {lh * 1e6:.2f} uH, R {rs:.3f} ohm, C {c_min * 1e6:.0f} uF, zeta {zeta:.2f} -> peak {v_cl:.2f} V"]
    worst = v_cl
    for e_ in ov["events"]:
        if e_["kind"] == "regen":
            pm0 = 2 * e_["ke_j"] / e_["t_s"]
            pnet = pm0 - e_["p_loss_w"]
            e_net = pnet ** 2 * e_["t_s"] / (2 * pm0) if pnet > 0 else 0.0
            i_net = max(pnet, 0.0) / v1
            vpk = v1 if i_net <= e_.get("standing_load_a", 0.0) else math.sqrt(v1 ** 2 + 2 * e_net / c_min)
        else:
            vpk = math.sqrt(v1 ** 2 + 2 * e_["energy_j"] / c_min)
            if e_.get("source_max_v") is not None:
                vpk = min(vpk, e_["source_max_v"])
        worst = max(worst, vpk)
        ev_rep.append(f"{e_['id']}: peak {vpk:.2f} V")
        if vpk > v_allow + 1e-9:
            fail("OVERVOLTAGE", f"{e_['id']}: {vpk:.2f} V > {lim_part} {ov['parts_on_bus'][lim_part]['limit_v']} V - {marg} V")
    if v_cl > v_allow:
        fail("OVERVOLTAGE", f"closure peak {v_cl:.2f} V > {v_allow} V")
    out["overvoltage_motion"] = {"lowest_rated_part": f"{lim_part} {ov['parts_on_bus'][lim_part]['limit_v']} V ({ov['parts_on_bus'][lim_part]['kind']})",
                                 "allowed_v": v_allow, "worst_event_v": round(worst, 2), "events": ev_rep, "bulk_uf_min": round(c_min * 1e6)}
    # --- verifier guards carried to revision L
    if "_LH_CARRIAGE_IS_UP" not in body("gcode_macro LH_PREPARE"):
        fail("TOOL_UP_GUARD", "LH_PREPARE (and so LH_TIP_CHECK EXPECT=1) must refuse unless the carriage is up")
    if not re.search(r"params\.UL\|float <= 0", body("gcode_macro LH_DISPENSE")):
        fail("DISPENSE_VOLUME_GUARD", "LH_DISPENSE must refuse UL <= 0")
    ms = body("gcode_macro MANUAL_STEPPER")
    if 'st.startswith("pusher_")' not in ms or '"gripper"' not in ms:
        fail("RAW_PLUNGER_MOVE", "raw MANUAL_STEPPER on pushers/gripper must be refused (LH_ macros only)")
    for sec in cp.sections():
        b = body(sec)
        if sec.startswith("gcode_macro ") and sec != "gcode_macro MANUAL_STEPPER":
            for m in re.finditer(r"^\s*MANUAL_STEPPER STEPPER=(\S+)", b, re.M):
                if m.group(1).startswith(("pusher", "gripper")):
                    fail("RAW_PLUNGER_MOVE", f"[{sec}] moves {m.group(1)} through the guarded MANUAL_STEPPER")
    for g_ in ("G0", "G1"):
        b = body(f"gcode_macro {g_}")
        if "xy and params.Z is defined" not in b or "xy and s.grip == 0" not in b:
            fail("G0_GUARD", f"{g_}: must refuse combined XY+Z moves and XY moves with the gripper state UNKNOWN")
        if "_LH_TOOLS_LOCKED" not in b or "dock_zone_yf" not in b or "s.hold" not in b:
            fail("XY_WITHOUT_LOCK", f"{g_}: XY moves must require both carriages locked/empty, refuse the dock row and any hold (head_l TOOL PRESENT AND LOCKED)")
    if "params.X|default" not in body("gcode_macro _LH_TC_GUARD"):
        fail("C1_GUARD_TARGET", "_LH_TC_GUARD must accept the target X/Y")
    # homing time recomputed from travels and speeds (rev L: three Z axes, two pushers, X then Y)
    zt = cp.getfloat("stepper_z", "position_max") + 2.0
    td_ = lambda k: float(cp.get("gcode_macro _LH_TOOLDEF", f"variable_{k}"))
    dk_ = lambda k: float(cp.get("gcode_macro _LH_DOCK", f"variable_{k}"))
    t_home = (dk_("stroke_mm") + 3.0 + 1.0) / dk_("home_speed") \
        + 3 * zt / cp.getfloat("stepper_z", "homing_speed") + 2 * 3.0 / 2.0 + 2 * ((td_("pusher_home_from") - td_("pusher_home_to")) / td_("pusher_home_speed") + 4.0) \
        + (cp.getfloat("stepper_x", "position_max") - cp.getfloat("stepper_x", "position_min")) / cp.getfloat("stepper_x", "homing_speed") \
        + (cp.getfloat("stepper_y", "position_max") - cp.getfloat("stepper_y", "position_min")) / cp.getfloat("stepper_y", "homing_speed")
    out["homing_time_s_computed"] = round(t_home, 1)
    if cp.getfloat("gcode_macro _LH_STATE", "variable_homing_time_s") < t_home:
        fail("HOMING_TIME", f"variable_homing_time_s {cp.get('gcode_macro _LH_STATE', 'variable_homing_time_s')} < computed {t_home:.0f} s")
    pm = sysd["plunger_motor"]
    for k in ("a", "b"):
        i_run = cp.getfloat(f"tmc5160 manual_stepper pusher_{k}", "run_current")
        f_ = 2 * math.pi * pm["holding_torque_nm"] * min(1.0, i_run / pm["rated_current_a"]) * pm["torque_at_speed_fraction"] * pm["efficiency"] / (pm["lead_mm"] / 1000)
        out[f"pusher_{k}_force_n"] = round(f_, 1)
        if f_ < pm["force_needed_n"] * pm["margin"]:
            fail("PLUNGER_FORCE", f"pusher_{k}: {f_:.0f} N at {i_run} A < {pm['force_needed_n']} N x {pm['margin']}")
    # fail-safe inputs: modifier in printer.cfg and contact sense in the wiring
    fs_ = sysd["input_failsafe"]["inputs"]
    for name, v in ((n_, v_) for n_, v_ in fs_.items() if isinstance(v_, dict)):
        secs = v.get("sections") or ([v["section"]] if "section" in v else [])
        for sec in secs:
            if not cp.has_section(sec) and sysd["family"]["build_size"] not in v.get("sizes", ["S", "M", "L"]):
                continue                    # size-only input not fitted in the built size (its size file holds it; fw_render_check renders it)
            key = "endstop_pin" if not sec.startswith("gcode_button") else "pin"
            raw = cp.get(sec, key).split()[0]
            mod = re.match(r"[\^!~]*", raw).group(0)
            pin = re.sub(r"[\^!~]", "", raw)
            if mod != v["modifier"]:
                fail("INPUT_NOT_FAILSAFE", f"[{sec}] {key} {raw}: expected modifier '{v['modifier']}' for '{v['contact']}' ({v['high_means']})")
            rows_ = [r for r in rows if pin in [p_.strip() for p_ in r["fw_pins"].split(",")] and r["cable_type"] != "PLUG_IN"]
            text = " ".join(r["from_pins"] + " " + r["to_pins"] + " " + r["from_connector"] + " " + r["to_connector"] + " " + r["notes"] for r in rows_)
            want = v.get("word") or {"NC": "NC", "NO": "NO", "NPN light-on": "LIGHT-ON", "ALM conducting while healthy": "CONDUCT", "closed while door closed": "CLOSED while the door is closed"}[v["contact"]]
            if want.lower() not in text.lower():
                fail("INPUT_WIRING_SENSE", f"{name} [{sec}] pin {pin}: wiring does not state contact '{v['contact']}'")
    # over-temperature acts on both THERM contactors
    km5 = [r for r in rows if {"KM5", "TC_RELAY"} == {r["from_device"], r["to_device"]} and "thermostat chain" in r["signal"]]
    feed = [r for r in rows if r["from_device"] == "SR1" and r["to_device"] == "TC_TS_BLOCK"]
    if not km5 or not feed:
        fail("OVERTEMP_ONE_CHANNEL", "TC-1 thermostat chain must be fed from SR1 and end on BOTH TC_RELAY and KM5 coils")
    # --- K14 settle, K15 MB-1 lift-off, K16 force-ended pick-up, tip gate y (rev L)
    ko2 = yaml.safe_load((ROOT / "cad/lh1/assembly/out/keepouts.yaml").read_text())
    rl = {r["id"].split("_")[0]: r for r in ko2["rules"]}
    deck2 = json.loads((ROOT / "cad/lh1/deck_modules/out/report.json").read_text())
    if "K15" in rl:
        m15 = re.search(r"<= (\d+(?:\.\d+)?) mm/s and <= (\d+(?:\.\d+)?) m/s", rl["K15"]["rule"])
        fm_ = family(sysd["family"]["build_size"])["deck_positions"]
        b2c = next(v["centre_xy"] for v in fm_.values() if isinstance(v, dict) and v.get("station") == "MB1")   # family: MB-1 at B4
        g = "gcode_macro _LH_MB1"
        got = (cp.getfloat(g, "variable_b2_x"), cp.getfloat(g, "variable_b2_y"), cp.getfloat(g, "variable_max_speed"), cp.getfloat(g, "variable_max_accel"))
        exp = (b2c[0], b2c[1], float(m15.group(1)), float(m15.group(2)) * 1000.0)
        out["mb1_liftoff"] = {"cfg": got, "cad": exp}
        if any(abs(a - b) > 1e-6 for a, b in zip(got, exp)):
            fail("MB1_LIFTOFF", f"_LH_MB1 {got} vs CAD {exp}")
        if "m.max_speed" not in body("gcode_macro LH_GRIPPER_LIFT"):
            fail("MB1_LIFTOFF", "LH_GRIPPER_LIFT must limit the gripper lift-off over MB-1")
    if "K14" in rl:
        want_ms = round(rl["K14"]["numbers"]["dwell_s"] * 1000)
        if cp.getint("gcode_macro _LH_TOOLDEF", "variable_settle_ms") != want_ms:
            fail("K14_SETTLE", f"settle_ms != {want_ms}")
        for sec in ("gcode_macro LH_PICK_TIP", "gcode_macro LH_EJECT_TIP", "gcode_macro LH_DISPENSE", "gcode_macro LH_TOOL_PICK", "gcode_macro LH_TOOL_PARK"):
            if "settle_ms" not in body(sec):
                fail("K14_SETTLE", f"[{sec}] lacks the K14 settle dwell")
    if "K16" in rl:
        pk = body("gcode_macro LH_PICK_TIP")
        if cp.get("gcode_macro LH_PICK_TIP", "variable_method").strip('"') != "switch" or "pick_window" not in pk or 'gcode_button force_' not in pk:
            fail("K16_PICKUP", "LH_PICK_TIP must end on the push-rod force switch, refuse a tripped switch at the start and bound the stroke by the tool's pick window (jammed rod)")
    tg = ko2.get("tip_check_gate")
    if tg:
        # family (integration 23 Nov 2026): the CAD publishes the B1 tip-gate beam (keepouts tip_check_gate_beam: y, z, emitter /
        # receiver faces, scan nozzle x); gate_y / gate_x / gate_z and every tool's gate stroke follow it
        fm2_ = family(sysd["family"]["build_size"])
        b1_ = next(v["centre_xy"] for v in fm2_["deck_positions"].values() if isinstance(v, dict) and v.get("station") == "TIPWASTE")
        bm_ = (fm2_.get("keepouts") or {}).get("tip_check_gate_beam")
        lt_ = "gcode_macro LH_TIP_CHECK"
        if not bm_:
            fail("TIP_GATE_VS_CAD", "family keepouts publish no tip_check_gate_beam")
        else:
            got_ = {k_: cp.getfloat(lt_, f"variable_gate_{k_}") for k_ in ("x", "y", "z")}
            out["tip_gate_y"] = {"cfg": got_["y"], "cad": bm_["beam_y"], "gate_x": (got_["x"], bm_["gate_x"]), "gate_z": (got_["z"], bm_["beam_z"])}
            if abs(got_["y"] - bm_["beam_y"]) > 1e-6 or abs(got_["x"] - bm_["gate_x"]) > 1e-6 or abs(got_["z"] - bm_["beam_z"]) > 1e-6:
                fail("TIP_GATE_VS_CAD", f"LH_TIP_CHECK gate x/y/z {got_} != CAD tip_check_gate_beam x {bm_['gate_x']} y {bm_['beam_y']} z {bm_['beam_z']}")
            if not (bm_["x_faces"][0] < bm_["gate_x"] < bm_["x_faces"][1]):
                fail("TIP_GATE_VS_CAD", "the scan nozzle x is not between the emitter and receiver faces")
            tdv_ = lambda k_: ast.literal_eval(cp.get("gcode_macro _LH_TOOLDEF", f"variable_{k_}"))
            gs_, ngs_, nt_, tl_, tp_ = tdv_("gate_s"), tdv_("nozzle_gate_s"), tdv_("noz_top"), tdv_("tip_len"), tdv_("tip")
            for t_ in gs_:
                if not tp_.get(t_):
                    continue
                want_ = nt_[t_] - tl_[tp_[t_]] - bm_["beam_z"]
                if abs(gs_[t_] - want_) > 1e-6 or abs(ngs_[t_] - (nt_[t_] - bm_["beam_z"] - 7.5)) > 1e-6:
                    fail("TIP_GATE_VS_CAD", f"{t_}: gate_s {gs_[t_]} / nozzle_gate_s {ngs_[t_]} != tip ends / nozzle 7.5 mm above at beam z {bm_['beam_z']} ({want_} / {nt_[t_] - bm_['beam_z'] - 7.5})")
        tipxy = ast.literal_eval(cp.get("gcode_macro _LH_GEOM", "variable_tip_xy"))
        wxy = ast.literal_eval(cp.get("gcode_macro _LH_GEOM", "variable_waste_xy"))
        lw_ = next(v["centre_xy"] for v in fm2_["deck_positions"].values() if isinstance(v, dict) and v.get("station") == "LW")
        if [float(v) for v in tipxy] != [float(v) for v in b1_] or [float(v) for v in wxy] != [float(v) for v in lw_]:
            fail("STATION_XY_VS_CAD", f"_LH_GEOM tip_xy {tipxy} / waste_xy {wxy} != CAD TIPWASTE {b1_} / LW {lw_}")
    # --- eject method B per carriage: travel = params, over-travel only in _LH_EJECT_OVERTRAVEL, current lifts >= 1.5 x the 90 N eject (head_l case B')
    spec_ = importlib.util.spec_from_file_location("p_ej", ROOT / "cad" / "lh1" / "params.py")
    P_ = importlib.util.module_from_spec(spec_); spec_.loader.exec_module(P_)
    ej = "gcode_macro LH_EJECT_TIP"
    if abs(cp.getfloat(ej, "variable_ej_travel") - P_.Z_EJECT_TRAVEL) > 1e-6:
        fail("EJECT_B_VS_CAD", "ej_travel != params Z_EJECT_TRAVEL")
    hl = json.loads((ROOT / "cad/lh1/head_l/out/report.json").read_text())
    # head_l report (ADR-014 latch, settled 23 Nov 2026): the eject force is published as coupling.eject_force_N, and its holding
    # case in coupling.loads_and_holding ('tip eject <F> N ...') must state the same force
    f_ej = hl["coupling"].get("eject_force_N")
    k_ = next((k for k in (hl["coupling"].get("loads_and_holding") or {}) if k.startswith("tip eject")), None)
    if f_ej is None or not k_ or abs(float(re.search(r"tip eject (\d+(?:\.\d+)?) N", k_).group(1)) - f_ej) > 1e-6:
        fail("EJECT_B_VS_CAD", f"head report coupling.eject_force_N {f_ej} missing or not the 'tip eject' holding case ({k_})")
    hdc_ = json.loads((ROOT / "cad/lh1/head/out/report.json").read_text())["checks"]
    hd_ = hdc_["z_thrust"]
    ms_ = hdc_["z_screw_axial_support_rev_g4"]["run_current"]["motor"] + " " + hd_["motor"]; mu_ = max(float(k[3:]) for k in hd_ if k.startswith("mu_"))
    usable = float(re.search(r"usable (?:torque assumed )?([\d.]+) N m", ms_).group(1)); rated = float(re.search(r"rated ([\d.]+) A", ms_).group(1))
    lift = 2 * math.pi * usable * (cp.getfloat(ej, "variable_ej_current") / rated) * hd_[f"mu_{mu_:.2f}"]["efficiency"] / 0.002
    out["eject_b"] = {"eject_force_N (head_l B')": f_ej, "lift_at_ej_current_N": round(lift, 1)}
    if f_ej is None or lift < 1.5 * f_ej:
        fail("EJECT_B_VS_CAD", f"eject over-travel current lifts {lift:.0f} N < 1.5 x {f_ej} N (head_l coupling case B')")
    eo = body("gcode_macro _LH_EJECT_OVERTRAVEL")
    if "{-" in eo or "MOVE={0 - v.ej_travel}" not in eo:
        fail("EJECT_B_SIGN", "_LH_EJECT_OVERTRAVEL must write MOVE={0 - v.ej_travel}: '{-' is Jinja whitespace control and renders +12 (12 mm DOWN)")
    for sec in cp.sections():
        if "{-" in body(sec):
            fail("JINJA_MINUS", f"[{sec}] contains '{{-' (Jinja whitespace control, drops a minus sign)")
    if cp.getboolean(ej, "variable_armed_ejector"):
        warn("EJECT_B_ARMED", "method-B ejection is armed: only after the commissioning dry run")
    neg = re.compile(r"(MOVE=-|MOVE=\{-|MOVE=\{0 - |Z=-|Z=\{-|Z-\d|Z\{-|SET_POSITION=-|STROKE=-)")
    for sec in cp.sections():
        # ADR-014: the dock lift's lowering and the pusher homing end on their switches 1.0 / 0.6 mm below 0 (homing moves): not a Z axis
        if sec.startswith(("gcode_macro", "homing_override")) and sec not in ("gcode_macro _LH_EJECT_OVERTRAVEL", "gcode_macro _LH_LIFT_HOME", "gcode_macro _LH_LIFT_DOWN") and neg.search(body(sec)):
            fail("NEGATIVE_Z_OUTSIDE_EJECT", f"[{sec}] can command a position below 0")
    for g_ in ("G0", "G1"):
        if "params.Z|float < 0" not in body(f"gcode_macro {g_}"):
            fail("NEGATIVE_Z_GUARD", f"{g_} must refuse Z below 0")
    if "params.MOVE|float < 0" not in body("gcode_macro MANUAL_STEPPER"):
        fail("NEGATIVE_Z_GUARD", "MANUAL_STEPPER must refuse MOVE below 0")
    if "_LH_EJECT_OVERTRAVEL" not in body(ej) or "ej_pos_tol" not in eo or "bin_clear" not in body(ej) or "drawer" not in body(ej):
        fail("EJECT_B_POSITION_GUARD", "the over-travel must re-check the funnel position; LH_EJECT_TIP must refuse with the tip bin full or the drawer open")
    # --- V2: wrappers rebuild commands, refuse firmware-only parameters; position-setting commands refused
    for name in ("G0", "G1", "MANUAL_STEPPER"):
        b_ = body(f"gcode_macro {name}")
        if "rawparams" in b_:
            fail("WRAPPER_PASSTHROUGH", f"{name} passes rawparams through")
    if "is not allowed" not in body("gcode_macro MANUAL_STEPPER") or "SYNC=0 is not allowed" not in body("gcode_macro MANUAL_STEPPER"):
        fail("WRAPPER_PASSTHROUGH", "MANUAL_STEPPER must refuse SET_POSITION / STOP_ON_ENDSTOP / ENABLE / SYNC=0")
    for name in ("G92", "SET_GCODE_OFFSET", "SET_KINEMATIC_POSITION"):
        if "action_raise_error" not in body(f"gcode_macro {name}"):
            fail("POSITION_SETTER_OPEN", f"{name} must be overridden to refuse")
    if cp.has_section("force_move") and cp.getboolean("force_move", "enable_force_move", fallback=False):
        fail("POSITION_SETTER_OPEN", "[force_move] enable_force_move must stay False")
    sv = body("gcode_macro SET_GCODE_VARIABLE")
    if "_LH_STATE" not in sv or "action_raise_error" not in sv:
        fail("STATE_UNPROTECTED", "SET_GCODE_VARIABLE must refuse writes to the firmware state macros")
    for sec in cp.sections():
        if sec != "gcode_macro SET_GCODE_VARIABLE" and re.search(r"(?m)^\s*SET_GCODE_VARIABLE MACRO=", body(sec)):
            fail("STATE_UNPROTECTED", f"[{sec}] writes state through the guarded SET_GCODE_VARIABLE (use the base)")
    if not sysd.get("firmware_residual_risks"):
        fail("RESIDUAL_RISK_UNDOCUMENTED", "system.yaml firmware_residual_risks missing")
    # --- V3: chain on a free Octopus input; homing checks it before any back-off; back-off-and-retrigger
    tu = pin_name(cp.get("gcode_button tools_up", "pin")) if cp.has_section("gcode_button tools_up") else None
    users = [s_ for s_ in cp.sections() for k_, v_ in cp.items(s_) if k_.endswith("pin") and pin_name(v_) == tu]
    if not tu or users != ["gcode_button tools_up"]:
        fail("TOOLS_UP_INPUT", f"the all-tools-up chain must be read on a pin used only by [gcode_button tools_up] (users {users})")
    hb = body("homing_override")
    i_chk, i_back, i_xy = hb.find("_LH_REQUIRE_TOOLS_UP"), hb.find("_LH_ZBACKOFF"), hb.find("G28 X")
    if not (0 <= i_chk < i_back < i_xy) or hb.count("_LH_REQUIRE_TOOLS_UP") < 2:
        fail("HOMING_CHAIN_ORDER", "homing must check the chain before any back-off move and again before G28 X Y")
    if "STOP_ON_ENDSTOP=1" in hb or "STOP_ON_ENDSTOP=2" in hb:
        fail("HOMING_ENDSTOP_MODE", "use Klipper's named STOP_ON_ENDSTOP values (home / inverted_probe)")
    if cp.getfloat("stepper_z", "homing_retract_dist") < 2.0:
        fail("HOMING_RETRACT", "stepper_z homing_retract_dist must be >= 2 mm so a never-opening switch is detected")
    for g_ in ("G0", "G1"):
        if 'gcode_button tools_up' not in body(f"gcode_macro {g_}"):
            fail("XY_WITHOUT_CHAIN", f"{g_} XY moves must require the all-tools-up chain")
    # --- V4: sideways base moves only after _LH_XY_GUARD; Z up before the plunger is released
    for name in ("LH_TIP_CHECK", "LH_NOZZLE_CHECK", "LH_TOOL_PARK", "LH_TOOL_PICK", "_LH_DOCK_ENTER", "LH_DOCK_LEAVE"):
        lines_ = [l_.strip() for l_ in body(f"gcode_macro {name}").splitlines()]
        for i_, l_ in enumerate(lines_):
            if re.match(r"G0\.1 .*[XY]", l_) and not any(x_.startswith("_LH_XY_GUARD") for x_ in lines_[max(0, i_ - 2):i_]):
                fail("SIDEWAYS_UNGUARDED", f"{name}: '{l_}' without _LH_XY_GUARD")
    eb_ = body(ej)
    if not (0 <= eb_.find("_LH_CARRIAGE_Z C={c} Z=0") < eb_.find("_LH_PUSHER_MOVE")):
        fail("EJECT_ORDER", "LH_EJECT_TIP must raise Z before releasing the plunger")
    if cp.get("gcode_button gripped", "pin").split()[0].startswith("^!"):
        fail("GRIPPED_NOT_FAILSAFE", "gripped switch must be NC ('^')")
    if "_LH_GRIP_RELEASED_CHECK" not in body("gcode_macro LH_GRIPPER"):
        fail("GRIPPED_NOT_FAILSAFE", "LH_GRIPPER STATE=OPEN must check the gripped switch reads released")
    # --- W5: built-in command audit (klipper-guardcheck package; table: system.yaml firmware_command_audit, wrapped by
    # command_audit.yaml).  Every command the table marks refused / wrapped has a gcode_macro of that name in the config
    # it applies to: refused = the macro raises an error and never calls the base; wrapped = the macro renames the
    # built-in (or, for G28, [homing_override]).  Sections that would register further commands must not appear without
    # a new audit row; every command the table says is registered must be classified.
    from klipper_guardcheck import audit as kg_audit, load_table as kg_load_table
    rig_cp = load_cfg(ROOT / "firmware/lh1/bench_rig.cfg") if (ROOT / "firmware/lh1/bench_rig.cfg").exists() else None
    table_ = kg_load_table(HERE / "command_audit.yaml")
    if not table_.get("commands"):
        fail("COMMAND_AUDIT_MISSING", "system.yaml firmware_command_audit missing")
    aud_ = kg_audit(table_, {"machine": cp, "rig": rig_cp})
    for f_ in aud_["findings"]:
        fail(f_["code"], f_["msg"])
    aud_rep = aud_["report"]
    if not (sysd.get("klipper_target") or {}).get("minimum"):
        fail("KLIPPER_TARGET_MISSING", "system.yaml klipper_target missing")
    elif "NOT PINNED" in str(sysd["klipper_target"].get("commit")):
        warn("KLIPPER_NOT_PINNED", "no Klipper commit pinned (system.yaml klipper_target.commit): record it at installation and re-run the audit")
    out["command_audit"] = aud_rep
    # --- behavioural guard check: render the guard macros' Jinja with a mocked printer (fw_render_check.py ->
    # klipper-guardcheck, cases in guard_cases.yaml)
    py_ = None
    try:
        import klipper_guardcheck  # noqa: F401  (imports jinja2)
        py_ = sys.executable
    except ImportError:
        for cand_ in ("python3", "/opt/homebrew/bin/python3", "/usr/bin/python3"):
            w_ = shutil.which(cand_) or (cand_ if Path(cand_).exists() else None)
            if w_ and subprocess.run([w_, "-c", "import klipper_guardcheck"], capture_output=True).returncode == 0:
                py_ = w_
                break
    if py_ is None:
        fail("GUARD_RENDER_UNAVAILABLE", "klipper_guardcheck / jinja2 not found (uv pip install -e packages/klipper-guardcheck): the guard behaviour was not checked")
    else:
        r_ = subprocess.run([py_, str(HERE / "fw_render_check.py")], capture_output=True, text=True)
        try:
            rr_ = json.loads(r_.stdout)
        except json.JSONDecodeError:
            rr_ = {"failed": -1, "cases": [], "jinja2": "?"}
            fail("GUARD_RENDER_ERROR", (r_.stderr or r_.stdout)[-400:])
        out["guard_render"] = {"interpreter": Path(py_).name, "jinja2": rr_.get("jinja2"), "cases": rr_.get("n"), "failed": rr_.get("failed"),
                               "built_size": rr_.get("built_size"), "by_size": rr_.get("by_size"), "total_failed_all_runs": rr_.get("total_failed_all_runs")}
        for s_, v_ in (rr_.get("by_size") or {}).items():
            for k_, vv_ in v_.items():
                if vv_.get("failed"):
                    fail("GUARD_BEHAVIOUR", f"size {s_} {k_}: {vv_['failed']} of {vv_['n']} cases failed {vv_.get('failed_cases', '')}"[:400])
        for c_ in rr_.get("cases", []):
            if not c_["ok"]:
                fail("GUARD_BEHAVIOUR", f"{c_['macro']} {c_['params']} [{c_['state']}]: want {c_['want']}, got {c_['got']} {c_['msg']} {c_['out']}")
    # --- Z drive current decision (system.yaml z_drive): thrust at the force current, stall at the highest current any
    # macro can set, driver limit.  Physics recomputed here from the motor/screw inputs; stall dynamic + weight and the
    # bearing rating read from the CAD report (assembly rev_g4 screw_axial_support, = head report screw_support_report).
    zd = sysd.get("z_drive") or {}
    hd_path = ROOT / "cad/lh1/head/out/report.json"
    if not zd or not hd_path.exists():
        fail("Z_DRIVE_UNDECIDED", "system.yaml z_drive or the head report is missing")
    else:
        # inputs READ FROM THE HEAD REPORT: motor constants from its motor string, screw from z_thrust, bearing rating and
        # the stall at rated current from z_screw_axial_support_rev_g4 load cases
        spec_p = importlib.util.spec_from_file_location("p_zd", ROOT / "cad" / "lh1" / "params.py")
        P_z = importlib.util.module_from_spec(spec_p); spec_p.loader.exec_module(P_z)
        hd = json.loads(hd_path.read_text())["checks"]
        zt_, zs_h = hd["z_thrust"], hd["z_screw_axial_support_rev_g4"]
        mstr = zs_h["run_current"]["motor"] + " " + zt_["motor"]
        num = lambda pat: float(re.search(pat, mstr).group(1))
        hold_, usable_, rated_ = num(r"holding ([\d.]+) N m"), num(r"usable (?:torque assumed )?([\d.]+) N m"), num(r"rated ([\d.]+) A")
        lead_ = float(re.search(r"Tr8x(\d+(?:\.\d+)?)", zt_["screw"]).group(1))
        d2_ = float(re.search(r"d2 = ([\d.]+)", zt_["screw"]).group(1))
        half_ = float(re.search(r"(\d+) deg trapezoidal", zt_["screw"]).group(1)) / 2
        mus_ = sorted(float(k_[3:]) for k_ in zt_ if k_.startswith("mu_"))
        m_ = zd["motor"]
        for k_, v_ in (("holding_torque_nm", hold_), ("usable_torque_nm", usable_), ("rated_current_a", rated_)):
            if abs(m_[k_] - v_) > 1e-6:
                fail("Z_DRIVE_STALE", f"system.yaml z_drive.motor.{k_} {m_[k_]} != head report {v_}")
        lam = math.atan(lead_ / (math.pi * d2_))
        eta = lambda mu: math.tan(lam) / math.tan(lam + math.atan(mu / math.cos(math.radians(half_))))
        for mu in mus_:
            if abs(eta(mu) - zt_[f"mu_{mu:.2f}"]["efficiency"]) > 0.002:
                fail("Z_DRIVE_MODEL", f"screw efficiency at mu {mu}: {eta(mu):.3f} != head report {zt_[f'mu_{mu:.2f}']['efficiency']}")
        thrust = lambda i: 2 * math.pi * usable_ * (i / rated_) * eta(max(mus_)) / (lead_ / 1000)
        stall_static = lambda i: 2 * math.pi * hold_ * (i / rated_) * eta(min(mus_)) / (lead_ / 1000)
        cases_ = {c_["case"]: c_ for c_ in zs_h["load_cases"]}
        stall_rated = cases_.get("stall onto ZHS-1 at rated current (static + dynamic + weight)")
        if not stall_rated:
            fail("Z_DRIVE_CAD_MISSING", "head report has no 'stall onto ZHS-1 at rated current' load case")
        else:
            c0a = stall_rated["C0a_N (ASSUMED)"]
            dyn_w = stall_rated["force_N"] - stall_static(rated_)          # rotor dynamic + carriage weight (CAD)
            f_need = zd["thrust_margin"] * P_z.Z_MIN_THRUST_N
            floor_ = f_need / thrust(rated_) * rated_
            ceil_bearing = (c0a / zd["bearing_min_sf"] - dyn_w) / stall_static(rated_) * rated_
            ceil_ = min(ceil_bearing, zd["driver_burst_limit_a"])
            zc_ = "gcode_macro _LH_ZCUR"
            i_run = cp.getfloat(zc_, "variable_run_current")
            i_force = cp.getfloat(zc_, "variable_force_current")
            z_secs = ("tmc5160 stepper_z", "tmc5160 manual_stepper z_a", "tmc5160 manual_stepper z_b")
            for zs_ in z_secs:
                if abs(cp.getfloat(zs_, "run_current") - i_run) > 1e-6:
                    fail("Z_CURRENT_MISMATCH", f"[{zs_}] run_current {cp.getfloat(zs_, 'run_current')} != _LH_ZCUR run_current {i_run}")
            i_max = max([i_run, i_force] + [cp.getfloat(zs_, "run_current") for zs_ in z_secs] + [cp.getfloat("gcode_macro LH_EJECT_TIP", "variable_ej_current")])
            st_max = stall_static(i_max) + dyn_w
            hrc = zs_h["run_current"]
            out["z_drive"] = {"motor (head report)": {"holding_Nm": hold_, "usable_Nm": usable_, "rated_A": rated_},
                              "force_current_A": i_force, "run_current_A": i_run, "max_settable_A": i_max,
                              "window_A": [round(floor_, 3), round(ceil_, 3)], "ceiling_bearing_A": round(ceil_bearing, 3),
                              "head_report_window_A": [hrc.get("force_current_floor_A (1.1 x Z_MIN_THRUST_N at mu 0.20)"),
                                                       hrc.get("force_current_ceiling_A (bearing SF 2 incl. dynamic + weight)")],
                              "thrust_at_force_current_N": round(thrust(i_force), 1), "required_N": round(f_need, 1),
                              "stall_at_max_settable_N": round(st_max, 0), "bearing_C0a_N": c0a, "bearing_sf": round(c0a / st_max, 2)}
            if i_force < floor_ - 1e-9:
                fail("Z_THRUST_LOW", f"Z force current {i_force} A < floor {floor_:.3f} A: thrust {thrust(i_force):.0f} N < {zd['thrust_margin']} x Z_MIN_THRUST_N = {f_need:.0f} N at mu {max(mus_)}")
            if i_force > ceil_bearing + 1e-9 or st_max > c0a / zd["bearing_min_sf"]:
                fail("Z_STALL_HIGH", f"stall at {i_max} A: {st_max:.0f} N > F6-14M C0a {c0a} / {zd['bearing_min_sf']} (ceiling {ceil_bearing:.3f} A)")
            if i_force > zd["driver_burst_limit_a"] + 1e-6 or i_max > zd["driver_rms_max_a"] + 1e-6:
                fail("Z_DRIVER_OVERLOAD", f"Z force current {i_force} A / max {i_max} A beyond the driver limits {zd['driver_burst_limit_a']} / {zd['driver_rms_max_a']} A")
            if i_run > zd["driver_continuous_limit_a"] + 1e-6:
                fail("Z_DRIVER_OVERLOAD", f"Z run current {i_run} A > continuous limit {zd['driver_continuous_limit_a']} A")
            if rig_cp is not None:
                rc_force = rig_cp.getfloat("gcode_macro _BR", "variable_force_current")
                if abs(rc_force - i_force) > 1e-6:
                    fail("Z_CURRENT_MISMATCH", f"rig force_current {rc_force} != machine force_current {i_force}")
    # --- B (23 Nov 2026): Z speed limit = corner speed at the run current (thrust basis kept) and the screw's critical speed
    zsp = (sysd.get("z_drive") or {}).get("speed")
    if zsp:
        zm = sysd["z_drive"]["motor"]
        i_pk = cp.getfloat("tmc5160 manual_stepper z_a", "run_current") * math.sqrt(2)
        ke_ = zm["holding_torque_nm"] / (math.sqrt(2) * zm["rated_current_a"])
        lo_, hi_ = 0.0, 2000.0
        for _ in range(80):
            w_ = (lo_ + hi_) / 2
            if (i_pk * zsp["phase_resistance_ohm"] + ke_ * w_) ** 2 + (i_pk * zsp["pole_pairs"] * zsp["inductance_mh"] / 1000 * w_) ** 2 > zsp["supply_v"] ** 2:
                hi_ = w_
            else:
                lo_ = w_
        lead_z = cp.getfloat("manual_stepper z_a", "rotation_distance")
        v_corner = lo_ / (2 * math.pi) * lead_z
        d_, L_ = zsp["screw_root_mm"], zsp["screw_free_length_mm"]
        n_crit = 1.875 ** 2 / (2 * math.pi * L_ ** 2) * math.sqrt(2.1e5 * (math.pi * d_ ** 4 / 64) / (7.85e-9 * math.pi * d_ ** 2 / 4))
        v_crit = n_crit * lead_z * zsp["critical_speed_factor"]
        v_set = {sec: cp.getfloat(sec, "velocity") for sec in ("manual_stepper z_a", "manual_stepper z_b")}
        v_set["printer max_z_velocity"] = cp.getfloat("printer", "max_z_velocity")
        v_set["_LH_GEOM z_fast"] = cp.getfloat("gcode_macro _LH_GEOM", "variable_z_fast")
        out["z_speed"] = {"corner_mm_s": round(v_corner, 1), "screw_critical_x_factor_mm_s": round(v_crit, 1), "set": v_set}
        for k_, v_ in v_set.items():
            if v_ > min(v_corner, v_crit) + 1e-6:
                fail("Z_SPEED", f"{k_} {v_} mm/s > the Z drive's limit {min(v_corner, v_crit):.1f} mm/s (corner {v_corner:.1f}, screw {v_crit:.1f}: system.yaml z_drive.speed)")
    tmc_users = sorted(s_ for s_ in cp.sections() if s_.startswith("gcode_macro") and re.search(r"(?m)^\s*SET_TMC_CURRENT(_BASE)?\b", body(s_)))
    if set(tmc_users) - {"gcode_macro _LH_Z_FORCE", "gcode_macro _LH_Z_NOMINAL", "gcode_macro _LH_EJECT_OVERTRAVEL", "gcode_macro _LH_LIFT_HOME", "gcode_macro _LH_PUSHER_HOME"}:
        fail("TMC_CURRENT_OUTSIDE_FIRMWARE", f"driver current set outside the Z-force / eject macros: {tmc_users}")
    pk_ = body("gcode_macro LH_PICK_TIP")
    if not (0 <= pk_.find("_LH_Z_FORCE C={c} TOOL={t} ON=1") < pk_.find("_LH_PICK_STEP") < pk_.find("_LH_Z_FORCE C={c} TOOL={t} ON=0") < pk_.find("_LH_PICK_END")):
        fail("Z_FORCE_ORDER", "LH_PICK_TIP must raise the Z current only around the stepped pick-up and restore it before _LH_PICK_END")
    if "_LH_Z_NOMINAL" not in body("homing_override"):
        fail("Z_FORCE_ORDER", "homing must restore the Z run currents (_LH_Z_NOMINAL)")
    # --- W3: tip gate positive reference (supply test) and full result
    if not cp.has_section("output_pin tip_gate_power"):
        fail("TIP_GATE_NO_SUPPLY_TEST", "[output_pin tip_gate_power] missing")
    tcb_ = body("gcode_macro LH_TIP_CHECK")
    if not (0 <= tcb_.find("SET_PIN PIN=tip_gate_power VALUE=0") < tcb_.find("KIND=off") < tcb_.find("SET_PIN PIN=tip_gate_power VALUE=1") < tcb_.find("KIND=ref") < tcb_.find("KIND=tip")):
        fail("TIP_GATE_NO_SUPPLY_TEST", "LH_TIP_CHECK must switch the gate off (must read blocked), on, then take the clear reference before the tips")
    ev_ = body("gcode_macro _LH_TIPGATE_EVAL")
    if not all(k_ in ev_ for k_ in ("TIPS=", "GAPS=", "RESOLVABLE=", "RESULT=")):
        fail("TIP_GATE_RESULT", "_LH_TIPGATE_EVAL must reply TIPS / GAPS / RESOLVABLE / RESULT")
    # --- tip gate
    tc = body("gcode_macro LH_TIP_CHECK") + body("gcode_macro _LH_TIPGATE_SAMPLE")
    if "KIND=ref" not in tc or 'params.KIND == "ref"' not in tc:
        fail("TIP_GATE_SELFTEST", "LH_TIP_CHECK must require a clear reference reading (dead sensor must fail)")
    if "gcode_button tip_gate" not in tc or "NOT OBSERVED" not in tc:
        fail("TIP_CHECK", "LH_TIP_CHECK must read the tip gate and report NOT OBSERVED while not armed")
    if not cp.getboolean("gcode_macro LH_TIP_CHECK", "variable_armed"):
        warn("TIP_GATE_NOT_ARMED", "tip gate beam position now from the CAD (family keepouts tip_check_gate_beam: y -60, z 105, faces); LH_TIP_CHECK stays disarmed (NOT OBSERVED) until the commissioning dry run measures the beam with real tips")
    out["rev_l"] = rev_l_checks(cp, sysd, rows, fail, warn, body)
    out["adr014"] = adr014_checks(cp, sysd, rows, fail, warn, body)
    return out





def adr014_checks(cp, sysd, rows, fail, warn, body):
    """ADR-014 (23 Nov 2026): the dock-lift exchange, the LGA35 pusher and the random-access tip parking, against the CAD
    (published_changes_23nov, head report, mechanism audit) and system.yaml.  Values parsed, never typed here."""
    out = {}
    fam = family(sysd["family"]["build_size"])
    pub = " ".join(fam["published_changes_23nov"]["firmware"] + fam["published_changes_23nov"]["wiring (electrical owner)"])
    head = json.loads((ROOT / "cad/lh1/head_l/out/report.json").read_text())
    dl = head["dock_bay"]["dock_lift"]
    num = lambda pat: float(re.search(pat, pub).group(1))
    # ---- plunger pusher
    pm = sysd["plunger_motor"]
    td = lambda k: float(cp.get("gcode_macro _LH_TOOLDEF", f"variable_{k}"))
    want = {"rotation_distance": num(r"rotation_distance ([\d.]+) mm"), "run": num(r"run ([\d.]+) A"), "home_i": num(r"homing <= ([\d.]+) A"),
            "home_v": num(r"homing <= [\d.]+ A and <= ([\d.]+) mm/s"), "endstop_i": num(r"above ([\d.]+) A"), "force_v": num(r"force strokes <= ([\d.]+) mm/s"),
            "soft": [float(x) for x in re.search(r"soft limits ([\d.]+) \.\. ([\d.]+)", pub).groups()]}
    got = {}
    for c in ("a", "b"):
        sec, tsec = f"manual_stepper pusher_{c}", f"tmc5160 manual_stepper pusher_{c}"
        got[c] = {"rotation_distance": cp.getfloat(sec, "rotation_distance"), "run": cp.getfloat(tsec, "run_current"), "velocity": cp.getfloat(sec, "velocity"),
                  "position_min": cp.getfloat(sec, "position_min"), "position_max": cp.getfloat(sec, "position_max")}
        g_ = got[c]
        if abs(g_["rotation_distance"] - want["rotation_distance"]) > 1e-9 or abs(g_["rotation_distance"] - pm["lead_mm"]) > 1e-9:
            fail("PUSHER_VS_CAD", f"[{sec}] rotation_distance {g_['rotation_distance']} != CAD {want['rotation_distance']} / plunger_motor.lead_mm {pm['lead_mm']}")
        if abs(g_["run"] - want["run"]) > 1e-9 or abs(g_["run"] - pm["run_current_a"]) > 1e-9 or abs(td("pusher_run_current") - g_["run"]) > 1e-9:
            fail("PUSHER_VS_CAD", f"[{tsec}] run_current {g_['run']} != CAD {want['run']} A / plunger_motor / _LH_TOOLDEF pusher_run_current")
        if g_["velocity"] > want["force_v"] + 1e-9:
            fail("PUSHER_SPEED", f"[{sec}] velocity {g_['velocity']} > the audited {want['force_v']} mm/s")
        if abs(g_["position_max"] - (want["soft"][1] - want["soft"][0])) > 1e-9:
            fail("PUSHER_VS_CAD", f"[{sec}] position_max {g_['position_max']} != CAD soft limits {want['soft']} from the switch")
        if not (-(want["soft"][0]) - 0.15 <= g_["position_min"] <= -(want["soft"][0])):
            fail("PUSHER_VS_CAD", f"[{sec}] position_min {g_['position_min']} must let the homing reach just past the internal stop (-{want['soft'][0]}) and no further")
    if td("pusher_vmax") > want["force_v"] + 1e-9 or td("pusher_home_speed") > want["home_v"] + 1e-9:
        fail("PUSHER_SPEED", "_LH_TOOLDEF pusher_vmax / pusher_home_speed above the CAD limits")
    if td("pusher_home_current") > want["home_i"] + 1e-9 or td("pusher_home_current") >= want["endstop_i"]:
        fail("PUSHER_HOMING_CURRENT", f"pusher homing current {td('pusher_home_current')} A must be <= {want['home_i']} A and below the maker's {want['endstop_i']} A")
    ph = body("gcode_macro _LH_PUSHER_HOME")
    i_lo, i_stop, i_hi = ph.find("CURRENT={d.pusher_home_current}"), ph.find("STOP_ON_ENDSTOP=home"), ph.rfind("CURRENT={d.pusher_run_current}")
    if not (0 <= i_lo < i_stop < i_hi) or "SPEED={d.pusher_home_speed}" not in ph:
        fail("PUSHER_HOMING_CURRENT", "_LH_PUSHER_HOME must lower the current before the homing move and restore it after the last one")
    for sec_ in [s_ for s_ in cp.sections() if s_.startswith(("gcode_macro", "homing_override"))]:
        b_ = body(sec_)
        if "STEPPER=pusher_" in b_ and "STOP_ON_ENDSTOP" in b_ and sec_ != "gcode_macro _LH_PUSHER_HOME":
            fail("PUSHER_HOMING_CURRENT", f"[{sec_}] homes a pusher outside _LH_PUSHER_HOME (run current into the internal stop)")
    if "pusher_vmax" not in body("gcode_macro _LH_PUSHER_MOVE"):
        fail("PUSHER_SPEED", "_LH_PUSHER_MOVE must refuse speeds above pusher_vmax")
    out["pusher"] = {"cad": want, "cfg": got}
    # ---- dock lift drive (the force audit's latch-release row)
    dk = sysd["dock_lift"]
    dv = lambda k: float(cp.get("gcode_macro _LH_DOCK", f"variable_{k}"))
    zr = sysd["z_drive"]["recomputed"]["worst_thrust_N_at"]
    gross = float(zr[f"{dk['run_current_a']:.1f}"])
    net = gross - dk["bar_load_n"]
    rel = head["coupling"]["latch"]["release"]["max_force_at_roller_N"]
    zm = sysd["z_drive"]["motor"]
    stall = 2 * math.pi * zm["holding_torque_nm"] * dk["run_current_a"] / zm["rated_current_a"] * sysd["z_drive"]["recomputed"]["efficiency"]["mu_0.10"] / (sysd["z_drive"]["screw"]["lead_mm"] / 1000)
    audit_row = next((r for r in fam["mechanism_audit"]["rows"] if r["mechanism"].startswith("latch release (LTG-1")), {})
    out["dock_lift"] = {"gross_N": gross, "net_N": round(net, 1), "release_N": rel, "margin_net": round(net / rel, 2), "audit_thrust_N": audit_row.get("thrust_available_N"),
                        "audit_margin": audit_row.get("margin"), "stall_static_N_mu010": round(stall)}
    rc_ = dk["recomputed"]
    if abs(rc_["gross_thrust_N_at"][f"{dk['run_current_a']:.1f}"] - gross) > 0.05 or abs(rc_["net_thrust_N_at"][f"{dk['run_current_a']:.1f}"] - net) > 0.05 \
            or abs(rc_["release_force_N"] - rel) > 0.05 or abs(rc_["margin_net"] - round(net / rel, 2)) > 0.005 or abs(rc_["stall_static_N_at_1p3"] - stall) > 1.0:
        fail("DOCK_LIFT_RECOMPUTED", f"system.yaml dock_lift.recomputed differs from check.py: gross {gross}, net {net:.1f}, release {rel}, margin {net / rel:.2f}, stall {stall:.0f}")
    if net / rel < 1.5:
        fail("DOCK_LIFT_THRUST", f"dock lift net thrust {net:.1f} N / latch release {rel} N = {net / rel:.2f} < 1.5")
    if not audit_row:
        fail("DOCK_LIFT_AUDIT_SOURCE", "the CAD mechanism audit has no latch-release row")
    elif abs(float(audit_row["thrust_available_N"]) - net) > 0.05:
        fail("DOCK_LIFT_AUDIT_SOURCE", f"the CAD audit's dock-lift thrust {audit_row['thrust_available_N']} N is not this drive's NET figure at {dk['run_current_a']} A ({net:.1f} N = {gross} - bar {dk['bar_load_n']})")
    elif abs(float(audit_row["margin"]) - round(net / rel, 2)) > 0.005:
        fail("DOCK_LIFT_AUDIT_SOURCE", f"the CAD audit's latch-release margin {audit_row['margin']} != net {net:.1f} / {rel} = {net / rel:.2f}")
    pairs = {"[manual_stepper dock_lift] rotation_distance": (cp.getfloat("manual_stepper dock_lift", "rotation_distance"), sysd["z_drive"]["screw"]["lead_mm"]),
             "[manual_stepper dock_lift] position_max": (cp.getfloat("manual_stepper dock_lift", "position_max"), dl["stroke_mm"]),
             "[tmc5160 manual_stepper dock_lift] run_current": (cp.getfloat("tmc5160 manual_stepper dock_lift", "run_current"), dk["run_current_a"]),
             "_LH_DOCK run_current": (dv("run_current"), dk["run_current_a"]), "CAD dock lift run current": (float(dl["run_current_A"]), dk["run_current_a"]),
             "_LH_DOCK home_current": (dv("home_current"), dk["home_current_a"]), "_LH_DOCK stroke_mm": (dv("stroke_mm"), dl["stroke_mm"]),
             "_LH_DOCK rise_mm": (dv("rise_mm"), dl["stroke_check"]["roller_rise_mm"]), "_LH_DOCK gap_mm": (dv("gap_mm"), dl["stroke_check"]["approach_gap_mm"]),
             "_LH_DOCK follow_mm": (dv("follow_mm"), dl["interleave_mm"]), "_LH_DOCK speed": (dv("speed"), dl["speed_mm_s"]),
             "[manual_stepper dock_lift] velocity": (cp.getfloat("manual_stepper dock_lift", "velocity"), dl["speed_mm_s"]),
             "_LH_DOCK open_height_mm": (dv("open_height_mm"), dk["open_height_mm"]), "_LH_DOCK xy_tol_mm": (dv("xy_tol_mm"), 0.5),
             "_LH_DOCK inrow_s": (dv("inrow_s"), float(cp.get("gcode_macro _LH_GEOM", "variable_dock_s")) - float(cp.get("gcode_macro _LH_GEOM", "variable_park_lift_mm"))),
             "_LH_DOCK inrow_s (CAD in-row stroke, head report dock_bay)": (dv("inrow_s"), float(head["dock_bay"]["in_row_stroke_s"])),
             "_LH_DOCK open_height_mm (CAD dock_lift)": (dv("open_height_mm"), float(dl["open_height_mm"]))}
    if not head.get("inrow_clearance", {}).get("pass"):
        fail("INROW_CLEARANCE", f"CAD in-row move clearance {head.get('inrow_clearance', {}).get('min_clearance_over_mm')} mm below the dock rule")
    for k_, (a_, b_) in pairs.items():
        if abs(a_ - b_) > 1e-6:
            fail("DOCK_LIFT_VS_CAD", f"{k_} = {a_} but CAD / system.yaml = {b_}")
    needed = dv("rise_mm") + dv("gap_mm") + dv("follow_mm")
    if needed > dv("stroke_mm") or not (dv("rise_mm") < dv("open_height_mm") < dv("rise_mm") + dv("gap_mm")):
        fail("DOCK_LIFT_VS_CAD", f"bar travel {needed} mm vs stroke {dv('stroke_mm')}, open-height switch {dv('open_height_mm')} must lie between the roller rise and the open position")
    corner = float(re.search(r"at 1.3 A ([\d.]+) mm/s", sysd["z_drive"]["speed"]["result"]).group(1))
    if dv("speed") > corner:
        fail("DOCK_LIFT_SPEED", f"bar speed {dv('speed')} above the corner speed at 1.3 A ({corner} mm/s): the thrust basis would not hold")
    # ---- exchange structure (head report safety.sequences and guards)
    drop, pick, lg = body("gcode_macro _LH_DROP"), body("gcode_macro _LH_PICKUP"), body("gcode_macro _LH_LIFT_GUARD")
    ov, ex = body("gcode_macro _LH_OPEN_VERIFY"), body("gcode_macro _LH_EXCH_GUARD")
    seqd = [drop.find(k) for k in ("MOVE={g.dock_s} SPEED", "_LH_LIFT_GUARD", "_LH_LIFT_MOVE TO={up}", "_LH_OPEN_VERIFY", "STEPPER=dock_lift MOVE={up + i", "STEPPER=z_{c} MOVE={g.dock_s - i", "_LH_LIFT_DOWN", "_LH_RELEASE_VERIFY")]
    seqp = [pick.find(k) for k in ("MOVE={s0} SPEED", "_LH_LIFT_GUARD", "_LH_LIFT_MOVE TO={up}", "_LH_OPEN_VERIFY", "STEPPER=z_{c} MOVE={s0 + i", "STEPPER=dock_lift MOVE={up - i", "_LH_LIFT_DOWN", "_LH_PICK_VERIFY", "MOVE=0 SPEED", "_LH_BAY_EMPTY_VERIFY")]
    need = {"drop-off: seat, guard, bar up, open check, bar-first follow, bar down, release check (in order)": all(x >= 0 for x in seqd) and seqd == sorted(seqd),
            "pick-up: approach, guard, bar up, open check, Z-first follow, bar down, lock check, Z up, bay check (in order)": all(x >= 0 for x in seqp) and seqp == sorted(seqp),
            "bar-up guard: exchange, bar down, at the bay centre, at the step-1 stroke, other carriage clear, bay switch, latch + code per mode":
                all(k in lg for k in ("s.exch", "k.bar != 0", "k.xy_tol_mm", "want_s", 'lo not in ("locked", "empty")', "gcode_button bay", 'lk != "locked"', 'lk != "empty"', "ns.code != d.codes[m]", "ns.code != 0")),
            "positive OPEN check (bar height switch AND latch NOT CLOSED), hold then lower": "dock_bar_open" in ov and 'lk != "open"' in ov and "_LH_HOLD_LOWERED" in ov,
            "hold latched before the bar is lowered": body("gcode_macro _LH_HOLD_LOWERED").find("VARIABLE=hold") < body("gcode_macro _LH_HOLD_LOWERED").find("_LH_LIFT_DOWN"),
            "every lowering ends on the DOWN switch": "STOP_ON_ENDSTOP=home" in body("gcode_macro _LH_LIFT_DOWN") and "VARIABLE=bar VALUE=-1.0" in body("gcode_macro _LH_LIFT_DOWN"),
            "exchange guard: no tips, Z at 0, chain, bar down, carriages consistent, bay state per mode": all(k in ex for k in ("s.tips_a or s.tips_b", "zpos_z_b > 0.01", "tools_up", "k.bar != 0", "_LH_TOOLS_LOCKED", "BAY_OCCUPIED", "BAY_EMPTY")),
            "X/Y refused unless the bar is down": "_LH_DOCK" in body("gcode_macro _LH_GMOVE") and "_LH_DOCK" in body("gcode_macro _LH_XY_GUARD"),
            "the bar moves only inside the exchange macros and homing": all(("STEPPER=dock_lift" not in body(s_)) or s_ in ("gcode_macro _LH_LIFT_MOVE", "gcode_macro _LH_LIFT_DOWN", "gcode_macro _LH_LIFT_HOME", "gcode_macro _LH_DROP", "gcode_macro _LH_PICKUP") for s_ in cp.sections() if s_.startswith("gcode_macro")),
            "exchange flag set before and cleared after the dock moves": all(body(s_).find("exch VALUE=1") < body(s_).find("_LH_DROP" if "PICK" not in s_ else "_LH_PICKUP") < body(s_).rfind("exch VALUE=0") for s_ in ("gcode_macro LH_TOOL_PARK", "gcode_macro LH_TOOL_PICK", "gcode_macro LH_TOOL_EXCHANGE")),
            "in-row move only through _LH_INROW_GUARD": body("gcode_macro LH_TOOL_EXCHANGE").find("_LH_INROW_GUARD") < body("gcode_macro LH_TOOL_EXCHANGE").find("G0.1 X{g.bay_x0 + (m - 1)")}
    out["exchange"] = need
    for k_, v_ in need.items():
        if not v_:
            fail("EXCHANGE_GUARD", f"tool exchange (ADR-014): {k_}")
    for c in ("a", "b"):
        for s_ in (f"latch_{c}_empty", f"latch_{c}_open"):
            if "_LH_CARRIAGE_EVENT" not in cp.get(f"gcode_button {s_}", "press_gcode", fallback="") or "_LH_CARRIAGE_EVENT" not in cp.get(f"gcode_button {s_}", "release_gcode", fallback=""):
                fail("DROPPED_TOOL_MONITOR", f"[gcode_button {s_}] must call _LH_CARRIAGE_EVENT on both edges")
        bands = sorted((tuple(float(x) for x in cp.get(f"gcode_button {n_}", "analog_range").split(",")), n_) for n_ in (f"latch_{c}", f"latch_{c}_empty", f"latch_{c}_open"))
        if any(bands[i][0][1] >= bands[i + 1][0][0] for i in range(2)):
            fail("LATCH_BANDS", f"carriage {c}: latch analog bands overlap {bands}")
        # ladder values from the CAD (head report coupling.latch.sensing: single-edge LOCK and EMPTY switches, both made when empty)
        lad_ = head["coupling"]["latch"]["sensing"]["ladder_ohm"]
        lad = {"LOCKED": lad_["LOCKED"], "EMPTY": lad_["CLOSED-EMPTY"], "OPEN": lad_["NOT CLOSED"]}
        for (lo, hi), n_ in bands:
            r_ = lad["EMPTY" if n_.endswith("_empty") else ("OPEN" if n_.endswith("_open") else "LOCKED")]
            if not (lo * 1.15 <= r_ <= hi / 1.15):
                fail("LATCH_BANDS", f"[gcode_button {n_}] band {lo}..{hi} does not hold the ladder value {r_:.0f} ohm with 15 % margin")
    # ---- tip parking: strip force limit from the head report coupling (the latch preload holds a strip with margin 1.5)
    hold = head["coupling"]["loads_and_holding"]
    k120 = next(v for k, v in hold.items() if k.startswith("comb strip 120"))
    k250 = next(v for k, v in hold.items() if k.startswith("comb strip 250"))
    slope = (k250["preload_to_just_hold_N"] - k120["preload_to_just_hold_N"]) / 130.0
    lim = 120.0 + (head["coupling"]["latch_preload_min_N"] / 1.5 - k120["preload_to_just_hold_N"]) / slope
    got_lim = float(cp.get("gcode_macro LH_TIP_PARK", "variable_strip_force_limit_n"))
    out["strip_force_limit_N"] = {"computed": round(lim, 1), "cfg": got_lim}
    if abs(got_lim - round(lim)) > 1.0:
        fail("STRIP_FORCE_LIMIT", f"LH_TIP_PARK strip_force_limit_n {got_lim} != {lim:.0f} N (latch preload {head['coupling']['latch_preload_min_N']} N / 1.5, head report comb strip cases)")
    if abs(float(cp.get("gcode_macro LH_TIP_PARK", "variable_accel_restore")) - cp.getfloat("printer", "max_accel")) > 1e-6:
        fail("TIP_PARKING_ACCEL", "LH_TIP_PARK accel_restore must equal [printer] max_accel")
    tp_tools = sorted(fam["tip_parking"].get("tools_admitted") or [])
    if tp_tools and [sorted(v[5]) for v in ast.literal_eval(cp.get("gcode_macro _LH_SIZE", "variable_tip_parking")).values()] != [tp_tools]:
        fail("TIP_PARKING_TOOLS", f"size file tool list differs from the CAD tools_admitted {tp_tools}")
    out["exchange_time"] = exchange_time(cp, sysd, fam, head)
    return out


def exchange_time(cp, sysd, fam, head):
    """Modelled time of the firmware's own exchange (LH_TOOL_EXCHANGE, the CAD's park-then-pick in one pass) and of the
    single macros, from printer.cfg speeds and accelerations (trapezoids), the CAD geometry and a 0.25 s allowance per
    verify (M400 + a button report, ESTIMATE).  Start and end at the deck centre (0, 0); mean bay distances as the CAD
    (3 pitches between the park and the pick bay; the pick bay at the row's mean bay).  SIMULATED, not measured."""
    g = lambda k: float(cp.get("gcode_macro _LH_GEOM", f"variable_{k}"))
    k = lambda n: float(cp.get("gcode_macro _LH_DOCK", f"variable_{n}"))
    def tr(d, v, a):
        d = abs(d)
        return 2 * math.sqrt(d / a) if d < v * v / a else d / v + v / a
    vxy, axy = g("xy_speed"), cp.getfloat("printer", "max_accel")
    vz, vzs, az = g("z_fast"), g("z_slow"), cp.getfloat("manual_stepper z_a", "accel")
    vb, ab = k("speed"), cp.getfloat("manual_stepper dock_lift", "accel")
    settle = float(cp.get("gcode_macro _LH_TOOLDEF", "variable_settle_ms")) / 1000.0
    ver = 0.25
    bays = fam["frame"]["dock_bays_x"]
    xm = sum(bays) / len(bays)
    dock_yf = fam["dock"]["printer_y_over_the_bays (dock_yf)"]
    ent = g("dock_entry_x")
    exit_y = round(dock_yf - 124.5, 1)
    s, ap, up = g("dock_s"), k("approach_mm"), k("rise_mm") + k("gap_mm")
    nf = int(k("follow_mm") / k("follow_step_mm"))
    t = {}
    t["XY in: deck centre -> X 465 -> dock row -> bay"] = tr(ent, vxy, axy) + tr(dock_yf, vxy, axy) + tr(ent - xm, vxy, axy)
    t["settle"] = settle
    t["drop 1: Z to the seat (fast + 15 slow) + guard"] = tr(s - ap, vz, az) + tr(ap, vzs, az) + ver
    t["drop 2: bar up to rise + gap + open check"] = tr(up, vb, ab) + ver
    t["drop 3: 13 x (bar 1 mm, Z 1 mm)"] = nf * (tr(k("follow_step_mm"), vb, ab) + tr(k("follow_step_mm"), vzs, az))
    t["drop 4: bar down onto its switch + release check"] = tr(up + k("follow_mm"), vb, ab) + ver
    t["empty carriage up to inrow_s"] = tr(s - k("follow_mm") - k("inrow_s"), vz, az)
    t["in-row X to the pick bay (3 pitches) + guard + settle"] = tr(3 * (bays[1] - bays[0]), vxy, axy) + ver + settle
    t["pick 1: Z to seat - 13 (fast + 15 slow) + guard"] = tr(s - k("follow_mm") - ap - k("inrow_s"), vz, az) + tr(ap, vzs, az) + ver
    t["pick 2: bar up to rise + gap + 13 + open check"] = tr(up + k("follow_mm"), vb, ab) + ver
    t["pick 3: 13 x (Z 1 mm, bar 1 mm)"] = nf * (tr(k("follow_step_mm"), vb, ab) + tr(k("follow_step_mm"), vzs, az))
    t["pick 4: bar down onto its switch + lock check"] = tr(up, vb, ab) + ver
    t["pick 5: Z up 15 slow + fast to 0 + bay check"] = tr(ap, vzs, az) + tr(s - ap, vz, az) + ver
    t["XY out: X 465, dock exit, back to the deck centre"] = tr(ent - xm, vxy, axy) + tr(dock_yf - exit_y, vxy, axy) + tr(math.hypot(ent, exit_y), vxy, axy)
    tot = sum(t.values())
    park_only = t["XY in: deck centre -> X 465 -> dock row -> bay"] + settle + sum(v for k_, v in t.items() if k_.startswith("drop")) + tr(s - k("follow_mm"), vz, az) + t["XY out: X 465, dock exit, back to the deck centre"]
    pick_only = t["XY in: deck centre -> X 465 -> dock row -> bay"] + settle + tr(s - k("follow_mm") - ap, vz, az) + tr(ap, vzs, az) + ver + sum(v for k_, v in t.items() if k_.startswith(("pick 2", "pick 3", "pick 4", "pick 5"))) + t["XY out: X 465, dock exit, back to the deck centre"]
    cf = (s * 0 + 1)
    cont = float(cp.get("gcode_macro _LH_TOOLDEF", "variable_contact_hi")) / 0.2 / 2 * (0.2 / 1.0 + 0.02 + ver)
    return {"LH_TOOL_EXCHANGE_s": round(tot, 1), "LH_TOOL_PARK_s": round(park_only, 1), "LH_TOOL_PICK_s": round(pick_only, 1),
            "plus_pusher_contact_search_s (typical, half the window)": round(cont * cf, 1), "plus_first_pickup_nozzle_check_s (CAD)": head["exchange"].get("plus_first_pickup_nozzle_check_s"),
            "cad_head_model_s": head["exchange"]["total_s"], "steps_s": {k_: round(v, 2) for k_, v in t.items()},
            "basis": "printer.cfg speeds/accelerations; XY 150 mm/s (_LH_GEOM xy_speed, the CAD assumed 300); 0.25 s per verify (M400 + button report, ESTIMATE); trapezoids; SIMULATED"}

def rev_l_checks(cp, sysd, rows, fail, warn, body):
    """Revision L (ADR-009, ADR-010, head_l safety, deck_k Y7): structure the guards must have, spare capacity, rails."""
    out = {}
    # exchange macros: every check of the head_l guard list
    pk, pa = body("gcode_macro LH_TOOL_PICK"), body("gcode_macro LH_TOOL_PARK")
    eg = body("gcode_macro _LH_EXCH_GUARD")
    need = {"no tips on either carriage": "s.tips_a or s.tips_b" in eg, "all Z at 0": "zpos_z_b > 0.01" in eg,
            "chain closed": "tools_up" in eg, "carriages consistent": "_LH_TOOLS_LOCKED" in eg, "no hold": "s.hold" in eg,
            "park: bay empty before (guard after an M400)": 'bay" ~ n].state == "PRESSED"' in eg and "MODE=drop" in pa and pa.find("M400") < pa.find("_LH_EXCH_GUARD"),
            "park: released + bay occupied after": "_LH_RELEASE_VERIFY" in body("gcode_macro _LH_DROP"),
            "pick: bay occupied before": 'bay" ~ n].state != "PRESSED"' in eg and "MODE=pick" in pk,
            "pick: code + latch before leaving": body("gcode_macro _LH_PICKUP").find("_LH_PICK_VERIFY") < body("gcode_macro _LH_PICKUP").find("MOVE=0 SPEED"),
            "pick: bay empty after": "_LH_BAY_EMPTY_VERIFY" in body("gcode_macro _LH_PICKUP"), "pick: nozzle check required (first pick-up of the tool since restart)": "nozzle_ok_{c} VALUE={1 if seen else 0}" in pk and '("," ~ t ~ ",") in s.nozzle_seen' in pk,
            "pick: pusher contact found": "_LH_PUSHER_FIND" in pk,
            "enter/leave at the right end": "dock_entry_x" in body("gcode_macro _LH_DOCK_ENTER") and "dock_entry_x" in body("gcode_macro LH_DOCK_LEAVE")}
    out["exchange"] = need
    for k, v in need.items():
        if not v:
            fail("EXCHANGE_GUARD", f"tool exchange: {k}")
    lk = body("gcode_macro _LH_TOOLS_LOCKED")
    for k in ("HALF_LOCK", "INVALID_CODE", "WRONG_TOOL", "DROPPED_TOOL", "CABLE", "LATCH_SWITCH", "LATCH_WIRE", "LATCH_OPEN"):
        if k not in lk:
            fail("LOCK_GUARD", f"_LH_TOOLS_LOCKED lacks the {k} refusal")
    for c in ("a", "b"):
        for s in [f"latch_{c}"] + [f"hall_{c}{i}" for i in range(4)]:
            if "_LH_CARRIAGE_EVENT" not in cp.get(f"gcode_button {s}", "release_gcode", fallback="") or "_LH_CARRIAGE_EVENT" not in cp.get(f"gcode_button {s}", "press_gcode", fallback=""):
                fail("DROPPED_TOOL_MONITOR", f"[gcode_button {s}] must call _LH_CARRIAGE_EVENT on both edges")
    if "M112" not in body("gcode_macro _LH_CARRIAGE_EVENT") or "s.exch == 0" not in body("gcode_macro _LH_CARRIAGE_EVENT"):
        fail("DROPPED_TOOL_MONITOR", "_LH_CARRIAGE_EVENT must M112 outside an exchange")
    # Y7 airflow, waste, bin, UV
    if "airflow" not in body("gcode_macro LH_ETHANOL") or "phold" not in body("gcode_macro _LH_AIRFLOW_LOST") or "_LH_AIRFLOW_LOST" not in cp.get("gcode_button airflow", "release_gcode"):
        fail("AIRFLOW_INTERLOCK", "no ethanol step without proven airflow; losing it during one must latch the AIRFLOW process hold")
    if "phold" not in body("gcode_macro _LH_PIPETTE_GUARD"):
        fail("AIRFLOW_INTERLOCK", "pipetting must be refused under the process hold")
    if "waste1_ok" not in body("gcode_macro LH_DISPENSE") or "waste2_ok" not in body("gcode_macro LH_DISPENSE"):
        fail("WASTE_FULL", "LH_DISPENSE must refuse over A4 with a waste bottle full")
    uv = body("gcode_macro LH_UV")
    if not all(k in uv for k in ("lock_fitted", "gcode_button door", "s.ethanol")) or "uv_enable" not in body("gcode_macro SET_PIN"):
        fail("UV_INTERLOCK", "LH_UV must need the option, guard locking, a closed door and no ethanol step; SET_PIN must refuse uv_enable")
    uvrow = [r for r in rows if r["to_device"] == "K_UV" and r["from_device"] == "FT"]
    br = sysd["branches"].get("F24", {})
    if not uvrow or br.get("rail") != "MOTION" or "DOOR" not in sysd["rails"]["MOTION"]["switched_by"]:
        fail("UV_INTERLOCK", "UV supply must come from MOTION (after the door contactors) through K_UV")
    # module place guard on every carriage / gripper descent
    for sec in ("gcode_macro MANUAL_STEPPER", "gcode_macro G0", "gcode_macro G1", "gcode_macro LH_PICK_TIP"):
        if "_LH_PLACE_GUARD" not in body(sec):
            fail("MODULE_SAFE_GUARD", f"[{sec}] must check the module SAFE line of the place below")
    # deck connectors exactly where the CAD lists them
    bs_ = sysd["family"]["build_size"]
    fd_ = family(bs_)["deck_positions"]
    # family: the CAD fits a connector at every 'fitted' place; module places (SAFE lines) are those with a module station;
    # B1 (tip station) and B3 (liquid waste) take the same connector part for their dedicated leads (station connectors)
    fitted = {p for p, v in fd_.items() if not p.startswith("_") and str(v.get("connector", "")).startswith("fitted") and v["station"] in MODULE_STATIONS}
    fitted = {MODULE_STATIONS[fd_[p]["station"]].upper() for p in fitted}
    station_con = sorted(p for p, v in fd_.items() if not p.startswith("_") and str(v.get("connector", "")).startswith("fitted") and v["station"] not in MODULE_STATIONS)
    have = {d.split("_", 1)[1] for d, v in sysd["devices"].items() if d.startswith("MODCONN_") and bs_ in v.get("sizes", ["S", "M", "L"])}
    out["connectors"] = {"module_places": sorted(have), "station_connectors": station_con}
    if fitted != have:
        fail("MODULE_CONNECTORS", f"module connectors {sorted(have)} != CAD fitted module places {sorted(fitted)} (size {bs_})")
    for p in have:
        rr = [r for r in rows if f"MODCONN_{p}" in (r["from_device"], r["to_device"])]
        kinds = {r["cable_type"] for r in rr}
        f_ = [r for r in rr if re.match(r"F4\d", r["protected_by"])]
        if not f_ or not any(t.startswith("USB") for t in kinds) or sysd["branches"][f_[0]["protected_by"]]["fuse_a"] > 3.15:
            fail("MODULE_CONNECTORS", f"MODCONN_{p}: needs a 24 V lead fused <= 3.15 A and a USB lead")
    # nothing exchanged carries electronics: no device on a tool, every head device on a carriage
    if any(d["kind"] in ("controller", "usb_device") and d["group"] == "head" and n not in ("MMB_A", "MMB_B") for n, d in sysd["devices"].items()):
        fail("PASSIVE_TOOLS", "an MCU or USB device in the head group other than the two carriage-plate MMB boards")
    # USB length (USB 2.0: 5 m per segment)
    for r in rows:
        if r["cable_type"].startswith("USB") and int(r["length_mm_estimate"]) > 5000:
            fail("USB_LENGTH", f"{r['cable_id']}: {r['length_mm_estimate']} mm > 5 m")
    # ADR-009 spare capacity, recomputed
    sockets = {"main": sysd["controller"]["driver_sockets"], "io": sysd["io_controller"]["driver_sockets"]} | {k: v["driver_sockets"] for k, v in sysd["head_controllers"].items()}
    spare_sock = sum(1 for s in sockets.values() for v in s.values() if str(v).startswith("spare") and "breakout" not in str(v))
    used = set()
    for sec in cp.sections():
        for k, v in cp.items(sec):
            if k.endswith("pin"):
                used.add(pin_name(v))
    main_gpio = ["PE7", "PE8", "PE9", "PE10", "PE12", "PE13", "PE14", "PE15", "PA4", "PA5", "PA6", "PA7", "PB1", "PB2", "PC5", "PC15", "PB10", "PC6", "PF2", "PE4"]
    head_in = ["PA15", "PA10", "PD9", "PD8", "PC2", "PC7", "PA9", "PB12", "PB10", "PB1", "PC5", "PC6", "PA8", "PB11", "PB2", "PB0", "PC4"]
    spare_in = sum(1 for p in main_gpio if p not in used) + sum(1 for h in ("head_a", "head_b") for p in head_in if f"{h}:{p}" not in used) \
        + sum(1 for p in ("PF0", "PF1") if f"io:{p}" not in used)
    outs = {"main": ["PB0", "PB11", "PA1", "PD12", "PD13", "PD14"], "io": ["PA0", "PB11", "PA1", "PE5", "PD12", "PD13", "PD14", "PD15"]}
    spare_out = sum(1 for b, ps in outs.items() for p in ps if (p if b == "main" else f"io:{p}") not in used)
    hubs = sysd["family"]["io_budget"]["S"]["usb"]["spare_ports"]
    tec = sysd["family"]["io_budget"]["S"]["thermal"]["spare_tec_channels"]
    out["spares"] = {"driver_sockets": spare_sock, "digital_inputs": spare_in, "switched_outputs": spare_out, "usb_ports": hubs, "tec_channels": tec}
    for k, v, m in (("driver_sockets", spare_sock, 2), ("digital_inputs", spare_in, 4), ("switched_outputs", spare_out, 2), ("usb_ports", hubs, 2)):
        if v < m:
            fail("ADR009_SPARES", f"{k}: {v} spare < {m}")
    if tec < 2:
        warn("ADR009_LOWER_FIGURE", f"spare fitted TEC channels {tec} < 2: lower figure stated in system.yaml family.io_budget (TEC_WARM, two channels, reserved: space, fuse, USB port, supply headroom)")
    # 30 % margin on every supply (ADR-009 rule 5)
    by_supply = {}
    for rail, spec in sysd["rails"].items():
        by_supply[spec["supply"]] = by_supply.get(spec["supply"], 0.0) + sum(l["a"] for l in sysd["loads"] if l["rail"] == rail)
    margins = {}
    for sup, load in by_supply.items():
        cap = sysd["supplies"][sup]["a_max"]
        margins[sup] = round(load / cap, 3)
        if load > cap / 1.3 + 1e-9:
            fail("ADR009_RAIL_MARGIN", f"{sup}: {load:.2f} A > {cap} / 1.3")
    out["supply_utilisation"] = margins
    # Y7: every contact-bearing device outside the control bay must be enclosed; TC_RELAY / TB_RELAY in the bay
    # family: EVERY relay, contactor, SSR, supply and safety relay is in the sealed control compartment (separate air)
    cab_ = yaml.safe_load((HERE / "control_bay.yaml").read_text())
    cx_ = cab_["compartment"]
    spark_kinds = ("contactor", "relay", "safety_relay", "gate_relay", "watchdog", "ssr", "interface_relay", "supply")
    inside = []
    for n, d_ in sysd["devices"].items():
        if d_["kind"] in spark_kinds or n in ("TG_SW",):
            x_, y_, z_ = d_["pos"]
            ok_ = d_["group"] == "control_bay" and cx_["x"][0] <= x_ <= cx_["x"][1] and cx_["y"][0] <= y_ <= cx_["y"][1] and cx_["z"][0] <= z_ <= cx_["z"][1]
            inside.append(n)
            if not ok_:
                fail("Y7_SPARK_SOURCE", f"{n} ({d_['kind']}) at {d_['pos']}: every relay, contactor and supply must be in the sealed control compartment {cx_['x']} x {cx_['y']} x {cx_['z']}")
    out["in_compartment"] = sorted(inside)
    if len(sysd["family"]["y7_ignition_sources"]) < 10:
        fail("Y7_SPARK_SOURCE", "the Y7 ignition-source table is incomplete")
    return out


def conductor_protection(rows, TYPES, sysd, fail, warn):
    """V8 (verification 21 Nov 2026): every fuse against every conductor it protects, from an IEC 60204-1 Table 6 based
    current-carrying capacity (system.yaml conductor_protection), NOT from the generic cable_types ampacity class.
    Iz = table (installation method, 40 C ambient) x installation factor (chain / bundle) x grouping factor.
    Rule (IEC 60204-1 7.2 / IEC 60364-4-43): fuse rating In <= Iz for every cable whose protected_by names a fuse.
    Cables protected only by a supply's own current limit are reported (short feeder runs) as warnings, not failures."""
    cpd = sysd["conductor_protection"]
    tab = {float(k): v for k, v in cpd["iz_table_a"].items()}
    fuses = ({spec["fuse"]: spec["fuse_a"] for spec in sysd["rails"].values()} | {k: v["fuse_a"] for k, v in sysd["branches"].items()}
             | {k: v["fuse_a"] for k, v in sysd.get("inline_fuses", {}).items()})
    table, worst = [], None
    for r in rows:
        t_ = TYPES.get(r["cable_type"]) or {}
        if t_.get("flat") or r["cable_type"] == "PLUG_IN" or not r["signal"].startswith(("24V", "0V", "5V", "TEC", "HEATER", "MAINS")):
            continue
        g_ = float(t_.get("gauge_mm2") or 0.0)
        m_ = re.match(r"^(F\d+)", r["protected_by"])
        chain = r["route"] != "fixed"
        if g_ in tab:
            base = tab[g_]
        else:
            base = float(t_.get("ampacity_a") or 0.0) * cpd["small_gauge_factor"]
        k_inst = cpd["factor_chain"] if chain else cpd["factor_fixed"]
        iz = round(base * k_inst, 2)
        if not m_:
            if "current limit" in r["protected_by"] and float(r["max_current_a"] or 0) > 0:
                lim = cpd["supply_limit_a"].get(r["from_device"]) or cpd["supply_limit_a"].get(r["to_device"])
                if lim and lim > iz:
                    warn("SUPPLY_FEEDER_SHORT_RUN", f"{r['cable_id']} {g_} mm2 ({r['length_mm_estimate']} mm, Iz {iz} A) is protected only by the supply's "
                         f"current limit (about {lim} A): short feeder before the first fuse (competent person: IEC 60204-1 7.2.8)")
            continue
        fa = fuses.get(m_.group(1))
        if fa is None:
            continue
        rec = {"cable": r["cable_id"], "fuse": m_.group(1), "fuse_a": fa, "mm2": g_, "chain": chain, "iz_a": iz,
               "load_a": float(r["max_current_a"] or 0.0), "ok": fa <= iz + 1e-9}
        table.append(rec)
        if not rec["ok"]:
            fail("FUSE_ABOVE_CONDUCTOR", f"{r['cable_id']}: fuse {m_.group(1)} {fa} A > Iz {iz} A of its {g_} mm2 conductor "
                 f"({'chain' if chain else 'fixed'}, IEC 60204-1 Table 6 basis, system.yaml conductor_protection)")
        if rec["load_a"] > iz + 1e-9:
            fail("LOAD_ABOVE_CONDUCTOR", f"{r['cable_id']}: load {rec['load_a']} A > Iz {iz} A")
        if worst is None or fa / iz > worst[1]:
            worst = (r["cable_id"], round(fa / iz, 2))
    return {"checked": len(table), "failed": sum(1 for x in table if not x["ok"]), "highest_fuse_to_iz": worst,
            "table": table}


def ffc_check(rows, TYPES, sysd, cp, fail, warn):
    """Head flat cables (family CR-L2): current per conductor and voltage drop of the motor phases at the cable's pitch.
    Pusher phases on carriage B are doubled (two ways each).  Limits: the cable's ampacity class per conductor, the ZIF
    adapter's contact rating, and 3 % of 24 V for the drop over the loop length (450 mm, out and back)."""
    ct = yaml.safe_load((HERE / "cable_types.yaml").read_text())["connectors"]
    out = {}
    hl = sysd["chains"]["head_loop"]
    run = {"pusher_a": cp.getfloat("tmc5160 manual_stepper pusher_a", "run_current"), "pusher_b": cp.getfloat("tmc5160 manual_stepper pusher_b", "run_current"),
           "gripper": cp.getfloat("tmc5160 manual_stepper gripper", "run_current")}
    rated = sysd["plunger_motor"]["rated_current_a"]
    for inst, motor_dev, key in (("HL_A", "PUSH_MOTOR_A", "pusher_a"), ("HL_B", "PUSH_MOTOR_B", "pusher_b"), ("HL_G", "GRIP_MOTOR", "gripper")):
        r = next((x for x in rows if x["to_device"] == motor_dev and f"head_loop:{inst}" in x["route"]), None)
        if r is None:
            fail("FFC_MOTOR_ROW", f"{inst}: no motor row")
            continue
        t_ = TYPES[r["cable_type"]]
        par = int(r["conductors"]) // 4
        i_c = run[key] / par
        fam_ = next(k for k in ct if r["connector_part"].split(" | ")[0].startswith(k) and k.startswith("ZIF"))
        a_con = ct[fam_]["a_per_contact"]
        r_c = RHO / t_["gauge_mm2"] * 0.45
        v = run[key] * 2 * r_c / par
        rec = {"cable": r["cable_type"], "ways_per_phase": par, "run_current_A": run[key], "A_per_conductor": round(i_c, 3),
               "conductor_rating_A": t_["ampacity_a"], "contact_rating_A": a_con, "use_of_rating_pct": round(100 * i_c / min(t_["ampacity_a"], a_con)),
               "phase_loop_resistance_ohm": round(2 * r_c / par, 3), "drop_V_at_run": round(v, 3), "drop_pct_of_24V": round(100 * v / 24.0, 2)}
        # ADR-014: the driver current is rms; a stepper phase is a sine, so each conductor carries i_c rms and sqrt(2) i_c peak.
        # FFC conductor and ZIF contact ratings are thermal (continuous) ratings: compared with the rms value.  If the part
        # bought states only a PEAK rating, the peak is what counts (purchase condition in system.yaml plunger_motor).
        lim_ = min(t_["ampacity_a"], a_con)
        rec["A_per_conductor_rms"] = round(i_c, 3)
        rec["A_per_conductor_peak"] = round(i_c * math.sqrt(2), 3)
        rec["rating_basis"] = "continuous (thermal) rating: rms compared (80 % rule)"
        rec["use_if_the_rating_were_peak_pct"] = round(100 * i_c * math.sqrt(2) / lim_)
        rec["continuous_rating_needed_A"] = round(i_c / 0.8, 3)
        rec["peak_rating_needed_A (if only a peak rating is published)"] = round(i_c * math.sqrt(2) / 0.8, 3)
        if key.startswith("pusher"):
            i_rated = rated / par
            rec["A_per_conductor_at_motor_rated"] = round(i_rated, 3)
            rec["at_rated_current"] = "acceptable" if i_rated <= 0.8 * lim_ else "NOT ACCEPTABLE at the motor's rated current: run current must stay limited (printer.cfg run_current)"
        out[inst] = rec
        if i_c > 0.8 * min(t_["ampacity_a"], a_con) + 1e-9:
            fail("FFC_CONDUCTOR_CURRENT", f"{inst}: {i_c:.2f} A per conductor > 80 % of {min(t_['ampacity_a'], a_con)} A ({r['cable_type']}, {fam_})")
        if v / 24.0 >= 0.03:
            fail("FFC_DROP", f"{inst}: {v:.2f} V drop per phase loop ({100 * v / 24:.1f} % of 24 V)")
    if run["pusher_a"] != run["pusher_b"]:
        fail("PUSHER_CURRENT", "both pushers must run at the same current (same tool force on either carriage)")
    if abs(run["pusher_a"] - sysd["plunger_motor"]["run_current_a"]) > 1e-9:
        fail("PUSHER_CURRENT", f"pusher run current {run['pusher_a']} A != plunger_motor.run_current_a {sysd['plunger_motor']['run_current_a']} A (the FFC verdict is for that current only)")
    hi_ = [k_ for k_, r_ in out.items() if r_["use_if_the_rating_were_peak_pct"] > 80]
    if hi_:
        warn("FFC_RATING_BASIS", "the head FFC check compares rms per conductor with a continuous (thermal) rating; if the cable or ZIF bought states only a PEAK rating, "
             + ", ".join(f"{k_} {out[k_]['A_per_conductor_peak']} A peak = {out[k_]['use_if_the_rating_were_peak_pct']} %" for k_ in hi_)
             + " of it: buy parts with a published continuous rating >= " + ", ".join(f"{k_} {out[k_]['continuous_rating_needed_A']} A" for k_ in hi_) + " (system.yaml plunger_motor purchase condition)")
    return out


def family_budgets(sysd, rows, cp):
    """ADR-009 rule 5 / ADR-011 'sized for L': rail, supply and I/O budgets for S and for L from the loads (a load with
    'sizes' counts only in those sizes; reserves are counted), plus the mains inlet and the worst coincident load."""
    out = {}
    for s in ("S", "L"):
        loads = [l for l in sysd["loads"] if s in l.get("sizes", ["S", "M", "L"])]
        rails, sup = {}, {}
        for rail, spec in sysd["rails"].items():
            a = sum(l["a"] for l in loads if l["rail"] == rail)
            rails[rail] = {"load_a": round(a, 2), "limit_a": spec["fuse_a"], "pct": round(100 * a / spec["fuse_a"])}
            sup[spec["supply"]] = sup.get(spec["supply"], 0.0) + a
        sups = {k: {"load_a": round(v, 2), "a_max": sysd["supplies"][k]["a_max"], "pct": round(100 * v / sysd["supplies"][k]["a_max"]),
                    "margin_rule_77pct": v <= sysd["supplies"][k]["a_max"] / 1.3 + 1e-9} for k, v in sup.items()}
        out[s] = {"rails": rails, "supplies": sups}
    return out


def host_command_checks(cp, fail):
    """Every command klipper_link.render() can emit is defined in printer.cfg with the parameters it sends."""
    import ast
    if not (ROOT / "src/retain/backends/klipper_link.py").exists():
        return  # public export: the host software is not in this repository, so its emitted commands cannot be listed
    src = (ROOT / "src/retain/backends/klipper_link.py").read_text()
    tree = ast.parse(src)
    funcs = {f.name: f for f in tree.body if isinstance(f, ast.FunctionDef)}
    def consts(node):
        """string constants in source order; calls to module functions are expanded in place (docstrings skipped)"""
        if isinstance(node, ast.Constant):
            return [node.value] if isinstance(node.value, str) else []
        if isinstance(node, ast.Call) and isinstance(node.func, ast.Name) and node.func.id in funcs:
            body = funcs[node.func.id].body
            body = body[1:] if body and isinstance(body[0], ast.Expr) and isinstance(getattr(body[0], "value", None), ast.Constant) else body
            return [c for st in body for c in consts(st)]
        return [c for ch in ast.iter_child_nodes(node) for c in consts(ch)]
    emitted = {}
    for ret in ast.walk(funcs["render"]):
        if isinstance(ret, ast.Return) and isinstance(ret.value, ast.List):
            for el in ret.value.elts:
                text = "".join(consts(el))
                name = text.strip().split(" ")[0]
                if name.startswith("LH_") or name == "MANUAL_STEPPER":
                    emitted.setdefault(name, set()).update(re.findall(r"([A-Z_]+)=", text))
    z_axis = next(n for n in tree.body if isinstance(n, ast.Assign) and getattr(n.targets[0], "id", "") == "Z_AXIS")
    axes = [v.value for v in z_axis.value.values if v.value != "printer_z"]
    for a in axes:
        if not cp.has_section(f"manual_stepper {a}"):
            fail("HOST_AXIS_UNDEFINED", f"klipper_link Z_AXIS uses {a}; no [manual_stepper {a}]")
    report = {}
    for name, ps in emitted.items():
        if name == "MANUAL_STEPPER":
            continue
        sec = f"gcode_macro {name}"
        excl = sysd_global.get("host_commands_not_in_klipper", {})
        if name in excl:
            if cp.has_section(sec):
                fail("HOST_COMMAND_SHOULD_BE_REMOVED", f"{name} must not be defined in printer.cfg: {excl[name]}")
            warn_list.append(f"host still emits {name}: {excl[name]}")
            report[name] = {"excluded": excl[name]}
            continue
        if not cp.has_section(sec):
            fail("HOST_COMMAND_UNDEFINED", f"{name} is emitted by klipper_link.py but not defined in printer.cfg")
            continue
        body = cp.get(sec, "gcode")
        used = set(re.findall(r"params\.([A-Z_]+)", body)) | set(re.findall(r"params\[['\"]([A-Z_]+)['\"]\]", body))
        missing = sorted(ps - used)
        report[name] = {"params_sent": sorted(ps), "params_read": sorted(used)}
        if missing:
            fail("HOST_PARAM_UNUSED", f"{name}: klipper_link sends {missing} but the macro never reads them")
    return report


warn_list = []

def adr003_checks(cp, sysd, rows, TYPES, CONN, fail, warn):
    """ADR-003 checks live in check_adr003.py (same directory)"""
    spec_ = importlib.util.spec_from_file_location("check_adr003", HERE / "check_adr003.py")
    mod_ = importlib.util.module_from_spec(spec_)
    spec_.loader.exec_module(mod_)
    return mod_.run(cp, sysd, rows, TYPES, CONN, fail, warn)


def mvp_checks(sysd, fail, warn):
    """reduced build profile MVP (profile_MVP.yaml, 24 Nov 2026): strict subset of the starter wiring, firmware pins against the MVP
    wiring, generated firmware files in step with the profile and the core, budgets, homing with the declared tool"""
    out = {}
    prof_p = HERE / "profile_MVP.yaml"
    if not prof_p.exists():
        return out
    prof = yaml.safe_load(prof_p.read_text())
    od = HERE / "out" / "mvp"
    need = ["wiring_MVP.csv", "electrical_bom_MVP.csv", "budgets_MVP.json", "joint_counts_MVP.json", "control_bay_MVP.yaml", "upgrade_steps.json", "mvp_electrical_notes.txt"]
    miss = [n for n in need if not (od / n).exists()]
    if miss:
        fail("MVP_OUTPUT_MISSING", f"run build_harness.py --profile MVP: {miss}")
        return out
    S = {r["cable_id"]: r for r in csv.DictReader(open(HERE / "wiring.csv"))}
    M = list(csv.DictReader(open(od / "wiring_MVP.csv")))
    absent = set(prof["absent_devices"])
    for r in M:
        s = S.get(r["cable_id"])
        if s is None:
            fail("MVP_NOT_SUBSET", f"{r['cable_id']} is not a starter cable")
            continue
        diff = [k for k in r if k != "notes" and r[k] != s[k]]
        if diff:
            fail("MVP_NOT_SUBSET", f"{r['cable_id']} differs from the starter in {diff}")
        ends = {r["from_device"], r["to_device"]} & absent
        if ends and r["cable_id"] not in prof["keep_cables"]:
            fail("MVP_ABSENT_END", f"{r['cable_id']} ends on absent {sorted(ends)} without a bridge")
    for b in prof["bridges"]:
        if not all(c in {r["cable_id"] for r in M} for c in b["cables"]):
            fail("MVP_BRIDGE", f"bridge at {b['at']}: its cables {b['cables']} are not all in the MVP")
    present = {d for d, v in sysd["devices"].items() if d not in absent and "S" in v.get("sizes", ["S", "M", "L"])}
    wired = {r["from_device"] for r in M} | {r["to_device"] for r in M}
    for d in sorted(present - wired):
        if "RESERVED" not in sysd["devices"][d]["desc"]:
            fail("MVP_DEVICE_NOT_WIRED", d)
    fz = sorted({f for r in M for f in re.findall(r"\bF\d+\b", r["protected_by"])}, key=lambda f: int(f[1:]))
    if set(fz) != set(prof["fuses_fitted"]):
        fail("MVP_FUSES", f"fuses on MVP cables {fz} != profile fuses_fitted {prof['fuses_fitted']}")
    # firmware files in step with the profile and the core
    import gen_size_files
    fwp = ROOT / "firmware/lh1/lh1_profile_MVP.cfg"
    if not fwp.exists() or fwp.read_text() != gen_size_files.profile_text("MVP"):
        fail("MVP_FIRMWARE_STALE", "firmware/lh1/lh1_profile_MVP.cfg is not what gen_size_files.py --profile MVP writes from profile_MVP.yaml")
    core_ = ROOT / "firmware/lh1/printer_MVP.cfg"
    if not core_.exists() or core_.read_text() != fw_config.profile_core_text("MVP")[0]:
        fail("MVP_FIRMWARE_STALE", "firmware/lh1/printer_MVP.cfg is not the core printer.cfg less the absent sections: re-run gen_size_files.py --profile MVP")
    txt, info = fw_config.resolve_profile("MVP")
    cp = load_cfg(text=txt)
    pv = cp.get("gcode_macro _LH_PROFILE", "variable_carriages", fallback="")
    if pv.replace(" ", "") != json.dumps(prof["carriages"]).replace(" ", ""):
        fail("MVP_PROFILE_VARS", f"_LH_PROFILE carriages {pv} != profile {prof['carriages']}")
    if "set_position_z" not in dict(cp.items("homing_override")):
        fail("MVP_HOMING", "no gripper Z: homing_override needs set_position_z")
    # pins: every MVP firmware pin wired to its device in wiring_MVP.csv; every fw pin of an MVP cable (not a plug-in breakout) used
    fw_rows = {}
    for r in M:
        for p_ in filter(None, (x.strip() for x in r["fw_pins"].split(","))):
            fw_rows.setdefault(p_, []).append(r)
    fmap, emap = sysd["firmware_map"], sysd["endstop_map"]
    unwired_ok = {("stepper_z", k) for k in ("step_pin", "dir_pin", "enable_pin", "endstop_pin")}     # head_a EZ3 empty: declared at 0, never moved
    pins, n_ = set(), 0
    for sec in cp.sections():
        for key, val in cp.items(sec):
            if not key.endswith("pin"):
                continue
            pin = pin_name(val)
            pins.add(pin)
            if (sec, key) in unwired_ok:
                continue
            dev = emap.get(sec) if key == "endstop_pin" else fmap.get(sec)
            devl = dev if isinstance(dev, list) else [dev]
            n_ += 1
            if not any(d_ in (r["from_device"], r["to_device"]) for r in fw_rows.get(pin, []) for d_ in devl):
                fail("MVP_PIN_NOT_WIRED", f"[{sec}] {key} {pin} not wired to {dev} in wiring_MVP.csv")
    for p_, rs in fw_rows.items():
        if p_ not in pins and not all(r["cable_type"] == "PLUG_IN" for r in rs):
            fail("MVP_WIRED_PIN_NOT_IN_CFG", f"{p_} ({rs[0]['cable_id']}) not used in the MVP firmware")
    bud = json.loads((od / "budgets_MVP.json").read_text())
    for k, v in list(bud["rails"].items()) + list(bud["branches"].items()):
        if v["pct"] > 80:
            fail("MVP_BUDGET", f"{k} {v['pct']} % > 80 %")
    for k, v in bud["supplies"].items():
        if not v["margin_rule_77pct"]:
            fail("MVP_BUDGET", f"{k} over 77 %")
    if bud["compartment"]["ratio"] < 1.5:
        fail("MVP_COMPARTMENT_AIR", f"one fan gives {bud['compartment']['ratio']} x the air for 10 K (< 1.5)")
    if prof["guard"] != "door" or "DOOR_SW" in absent or "SR2" in absent:
        fail("MVP_GUARD", "the MVP keeps the starter's door channel (MVP CAD guarding)")
    stages = json.loads((od / "upgrade_steps.json").read_text())
    road = json.loads((ROOT / "cad/lh1/machine_family/out/MVP/roadmap.json").read_text()) if (ROOT / "cad/lh1/machine_family/out/MVP/roadmap.json").exists() else None
    if road:
        ids = [s["roadmap_stage"] for s in stages["stages"]]
        if ids != sorted(ids) or set(ids) != {s["id"] for s in road["stages"] if s["id"] > 0}:
            fail("MVP_ROADMAP_ORDER", f"upgrade_steps roadmap stages {ids} do not follow roadmap.json")
    out = {"cables": len(M), "starter_cables": len(S), "pins_checked": n_, "fuses": fz, "sections_cut": len(info["cut"]),
           "budgets": {k: v["pct"] for k, v in bud["rails"].items()}, "compartment_w": bud["compartment"]["heat_w"]}
    return out


def main() -> int:
    sysd = yaml.safe_load((HERE / "system.yaml").read_text())
    sysd_global.update(sysd)
    ct = yaml.safe_load((HERE / "cable_types.yaml").read_text())
    TYPES, CONN = ct["cable_types"], ct["connectors"]
    findings, budget, info = [], {}, {}

    def fail(code, msg): findings.append({"severity": "error", "code": code, "msg": msg})
    def warn(code, msg): findings.append({"severity": "warning", "code": code, "msg": msg})

    # ---------------- architecture ----------------
    loads = {l["id"]: l for l in sysd["loads"]}
    by_supply: dict[str, float] = {}
    for rail, spec in sysd["rails"].items():
        load = sum(l["a"] for l in sysd["loads"] if l["rail"] == rail)
        budget[rail] = {"load_a": round(load, 2), "fuse_a": spec["fuse_a"]}
        by_supply[spec["supply"]] = by_supply.get(spec["supply"], 0.0) + load
        if load > 0.8 * spec["fuse_a"]:
            fail("RAIL_OVER_FUSE", f"{rail}: {load:.2f} A exceeds 80 % of {spec['fuse_a']} A")
    for br, spec in sysd.get("branches", {}).items():
        missing = [l for l in spec["loads"] if l not in loads]
        if missing:
            fail("BRANCH_UNKNOWN_LOAD", f"{br}: {missing}")
            continue
        load = sum(loads[l]["a"] for l in spec["loads"])
        if any(loads[l]["rail"] != spec["rail"] for l in spec["loads"]):
            fail("BRANCH_RAIL", f"{br}: a load is not on {spec['rail']}")
        budget[br] = {"load_a": round(load, 2), "fuse_a": spec["fuse_a"]}
        if load > 0.8 * spec["fuse_a"]:
            fail("BRANCH_OVER_FUSE", f"{br}: {load:.2f} A exceeds 80 % of {spec['fuse_a']} A")
        if spec["fuse_a"] > sysd["rails"][spec["rail"]]["fuse_a"]:
            fail("BRANCH_FUSE_ABOVE_RAIL", br)
    for sup, load in by_supply.items():
        cap = sysd["supplies"][sup]["a_max"]
        budget[sup] = {"load_a": round(load, 2), "a_max": cap, "utilisation": round(load / cap, 2)}
        if load > 0.8 * cap:
            fail("SUPPLY_OVERLOAD", f"{sup}: {load:.1f} A exceeds 80 % of {cap} A")
    chain = {c["id"]: set(c["breaks"]) for c in sysd["safety_chain"]}
    heater_rails = {r for r in sysd["rails"] if r.startswith("THERM")}
    if not ({"MOTION"} | heater_rails) <= chain["ESTOP"]:
        fail("ESTOP_INCOMPLETE", "e-stop must remove motion and all thermal power")
    if "PIPETTE" in sysd["rails"]:
        fail("RETIRED_REFERENCE", "ADR-003: the reserved PIPETTE rail is dropped from the first build")
    if "LOGIC" in chain["ESTOP"] | chain["DOOR"]:
        fail("LOGIC_SWITCHED", "logic must stay powered to keep and log state")
    if chain["DOOR"] != {"MOTION"}:
        fail("DOOR_SCOPE", "the door interlock should stop motion only")
    for th in ("OVERTEMP_BLOCK", "OVERTEMP_SINK", "OVERTEMP_LID"):
        if chain.get(th) != {"THERM"} or th not in sysd["rails"]["THERM"]["switched_by"]:
            fail("NO_INDEPENDENT_OVERTEMP", th)
    for rail in heater_rails:
        stats = [c for c in sysd["rails"][rail]["switched_by"] if c.startswith("OVERTEMP")]
        if len(stats) < 2 or any(chain.get(c) != {rail} for c in stats):
            fail("NO_INDEPENDENT_OVERTEMP", f"{rail}: needs at least block and sink thermostats of its own")
    for load in sysd["loads"]:
        if "motor" in load["id"] and load["rail"] != "MOTION":
            fail("MOVING_LOAD_NOT_ON_MOTION", load["id"])
    sockets = sysd["controller"]["driver_sockets"]
    if sorted(sockets) != list(range(8)):
        fail("DRIVER_SOCKETS", f"expected sockets 0-7, got {sorted(sockets)}")
    if sorted(sysd["io_controller"]["driver_sockets"]) != list(range(8)):
        fail("DRIVER_SOCKETS", "io board: expected sockets 0-7")
    for hb, spec in sysd["head_controllers"].items():
        if sorted(spec["driver_sockets"]) != [1, 2, 3, 4]:
            fail("DRIVER_SOCKETS", f"{hb}: expected EZ sockets 1-4")

    # ---------------- printer.cfg ----------------
    info["family"] = family_checks(sysd, fail, warn)
    BUILT = sysd["family"]["build_size"]
    try:
        cp = load_cfg(text=fw_config.resolve(BUILT)[0])
    except (configparser.Error, ValueError, FileNotFoundError) as exc:
        fail("CFG_PARSE", str(exc))
        cp = None
    pins: dict[str, str] = {}
    sec_pins: dict[str, list[tuple[str, str]]] = {}
    if cp:
        for sec in cp.sections():
            for key, val in cp.items(sec):
                if key.endswith("pin"):
                    pin = pin_name(val)
                    adc_ = sec.startswith("gcode_button") and cp.has_option(sec, "analog_range")
                    if pin in pins and not (adc_ and pins[pin].startswith("ADC ")):
                        fail("PIN_REUSED", f"{pin} in [{sec}] {key} and {pins[pin]}")
                    if adc_:      # ADR-014 latch ladder: several analog-range buttons share one ADC pin (Klipper buttons.py)
                        pins[pin] = f"ADC [{sec}] {key}"
                        sec_pins.setdefault(sec, []).append((key, pin))
                        continue
                    pins[pin] = f"[{sec}] {key}"
                    sec_pins.setdefault(sec, []).append((key, pin))
            if sec.startswith(("stepper_", "manual_stepper ")):
                for need in ("step_pin", "dir_pin", "enable_pin"):
                    if not cp.has_option(sec, need):
                        fail("STEPPER_INCOMPLETE", f"[{sec}] lacks {need}")
        spec = importlib.util.spec_from_file_location("p", ROOT / "cad" / "lh1" / "params.py")
        P = importlib.util.module_from_spec(spec); spec.loader.exec_module(P)
        # X: carriage centre +/- TRAVEL_X/2.  Y: the tool-reference y = beam front y + TOOL_REFERENCE_LOCAL_Y over the CAD's
        # BEAM_FRONT_Y_WINDOW (the host's lh1_geometry.carriage_xy uses the same window), so rev H gives -310..200
        # revision L: X = carriage centre over params L_XC_RANGE, Y = beam front face over L_YF_RANGE (rev_l travel)
        fb_ = family(BUILT)["travel"]
        want = {"stepper_x": tuple(fb_["X"]["travel_xc"]), "stepper_y": tuple(fb_["Y"]["travel_yf"])}
        info["y_limits_from_cad"] = want["stepper_y"]
        for sec, (lo, hi) in want.items():
            if abs(cp.getfloat(sec, "position_max") - hi) > 1e-6 or abs(cp.getfloat(sec, "position_min") - lo) > 1e-6:
                fail("TRAVEL_MISMATCH", f"[{sec}] limits {cp.getfloat(sec, 'position_min')}..{cp.getfloat(sec, 'position_max')} differ from the CAD {lo}..{hi}")
        if abs(cp.getfloat("stepper_y", "position_endstop") - want["stepper_y"][0]) > 1e-6:
            fail("TRAVEL_MISMATCH", "[stepper_y] position_endstop must be the front end of the window (front end stops)")
        for sec in ("stepper_z", "manual_stepper z_a", "manual_stepper z_b"):
            pm_ = cp.getfloat(sec, "position_max")
            if abs(pm_ - P.L_Z_TRAVEL) > 1e-6:
                fail("TRAVEL_MISMATCH", f"[{sec}] position_max {pm_} vs L_Z_TRAVEL {P.L_Z_TRAVEL}")
        # steppers per MCU vs that board's driver sockets
        mcus = {"": "main"} | {s.split()[1]: s.split()[1] for s in cp.sections() if s.startswith("mcu ")}
        for m in mcus:
            if m and m not in sysd["head_controllers"] and m != "io":
                fail("UNKNOWN_MCU", f"[mcu {m}] has no board in system.yaml")
        per_mcu: dict[str, list[str]] = {}
        for sec in cp.sections():
            if sec.startswith(("stepper_", "manual_stepper ")):
                sp = pin_name(cp.get(sec, "step_pin"))
                per_mcu.setdefault(sp.split(":")[0] if ":" in sp else "", []).append(sec)
        main_used = [v for v in sockets.values() if not str(v).startswith("spare")]
        if len(per_mcu.get("", [])) != len(main_used):
            fail("DRIVER_COUNT", f"main board: {len(per_mcu.get('', []))} steppers in cfg, {len(main_used)} sockets used in system.yaml")
        if per_mcu.get("io"):
            fail("DRIVER_COUNT", "io board: no stepper expected (all sockets spare or input breakouts)")
        for hb, spec_hb in sysd["head_controllers"].items():
            if len(per_mcu.get(hb, [])) != sum(1 for v in spec_hb["driver_sockets"].values() if not str(v).startswith("spare")):
                fail("DRIVER_COUNT", f"{hb}: {len(per_mcu.get(hb, []))} steppers in cfg, {len(spec_hb['driver_sockets'])} sockets")
        addrs = [pin_name(cp.get(s, "cs_pin")) for s in cp.sections() if s.startswith("tmc5160")]
        n_head = sum(1 for spec_hb in sysd["head_controllers"].values() for v in spec_hb["driver_sockets"].values() if not str(v).startswith("spare"))
        n_main_tmc = sum(1 for v in sockets.values() if "TMC5160" in str(v))
        if len(addrs) != len(set(addrs)) or sum(1 for a_ in addrs if a_.startswith("head_")) != n_head or sum(1 for a_ in addrs if ":" not in a_) != n_main_tmc:
            fail("SPI_CS_REUSED", f"{n_head} head + {n_main_tmc} main-board TMC5160 sections with distinct CS pins expected: {addrs}")
        if any(s.startswith("tmc2209") for s in cp.sections()):
            fail("HEAD_DRIVER_CLASS", "ADR-003: the head drivers are EZ5160 Pro (tmc5160), no tmc2209 section may remain")
        for s_ in cp.sections():
            if s_.startswith("tmc5160") and cp.get(s_, "spi_bus", fallback="") != "spi1":
                fail("HEAD_DRIVER_CLASS", f"[{s_}] must use spi_bus spi1 (MMB CAN V2.0)")
        info["steppers_per_mcu"] = {k or "main": len(v) for k, v in per_mcu.items()}
        info["mechanism_vs_cad"] = mechanism_checks(cp, fail, warn)
        hc_f = []
        info["host_commands"] = host_command_checks(cp, lambda c_, m_: hc_f.append((c_, m_)))
        if hc_f:
            warn("HOST_LINK_BEHIND_REV_L", f"src/retain/backends/klipper_link.py still emits the revision W interface ({len(hc_f)} mismatches, e.g. {hc_f[0][1][:120]}): planner/software owner to update (system.yaml rev_l.owners.planner_software)")
        rev_d = ROOT / "cad/lh1/assembly/out/changes_rev_d.yaml"
        if rev_d.exists():
            info["cad_requests"] = {"closed": sysd.get("closed_cad_requests", []), "partly": sysd.get("partly_closed_cad_requests", {}),
                                    "open": [c["id"] for c in sysd.get("open_cad_requests", [])]}
        else:
            warn("CAD_REV_D_PENDING", "cad/lh1/assembly/out/changes_rev_d.yaml not present: CR-01..CR-13 still open; LH_EJECT_TIP geometry provisional (armed False)")

    # ---------------- wiring.csv ----------------
    rows = hg.read_rows(HERE / "wiring.csv")
    for f_ in hg.check_ids(rows):
        fail(f_["code"], f_["msg"])
    devs = {d for d, v in sysd["devices"].items() if sysd["family"]["build_size"] in v.get("sizes", ["S", "M", "L"])}
    wired = {r["from_device"] for r in rows} | {r["to_device"] for r in rows}
    for d in sorted(devs - wired):
        fail("DEVICE_NOT_WIRED", d)
    for d in sorted(wired - devs):
        fail("WIRING_UNKNOWN_DEVICE", d)
    for f_ in hg.check_types(rows, TYPES):
        fail(f_["code"], f_["msg"])

    # firmware pins <-> wiring
    fw_rows: dict[str, list[dict]] = {}
    for r in rows:
        for p in filter(None, (x.strip() for x in r["fw_pins"].split(","))):
            fw_rows.setdefault(p, []).append(r)
    fmap, emap = sysd["firmware_map"], sysd["endstop_map"]
    n_pin_checked = 0
    for sec, plist in sec_pins.items():
        for key, pin in plist:
            dev = emap.get(sec) if key == "endstop_pin" else fmap.get(sec)
            if dev is None:
                fail("PIN_NO_DEVICE_MAP", f"[{sec}] {key} {pin}: no device in firmware_map/endstop_map")
                continue
            devl = dev if isinstance(dev, list) else [dev]
            ok = any(d_ in (r["from_device"], r["to_device"]) for r in fw_rows.get(pin, []) for d_ in devl)
            n_pin_checked += 1
            if not ok:
                fail("PIN_NOT_WIRED", f"[{sec}] {key} {pin} is not wired to {dev} in wiring.csv")
    for p in fw_rows:
        if p not in pins:
            fail("WIRED_PIN_NOT_IN_CFG", f"{p} ({fw_rows[p][0]['cable_id']}) is not used in printer.cfg")
    info["cfg_pins_checked_against_wiring"] = n_pin_checked

    # ---------------- drag chains ----------------
    tc = json.loads((ROOT / "cad/lh1/thermocycler/out/report.json").read_text())
    tcc = next(b for b in tc["bought_parts"] if "drag chain" in b["item"].lower())
    tw, th = map(float, re.search(r"inner >=\s*(\d+)\s*x\s*(\d+)", tcc["spec"]).groups())
    tr = float(re.search(r"R\s*(\d+)", tcc["spec"]).group(1))
    ch = sysd["chains"]
    # revision L: the X/Y chains are not in the CAD (machine_l): sized from the electrical request (CR-L1), warned below
    cra_ = json.loads((ROOT / "cad/lh1/machine_family/out/cr_answers.json").read_text())["CR-L1 cable chains"]
    for k_ in ("X chain", "Y chain"):
        if "inner 38 x 18" not in cra_[k_] and "same chain" not in cra_[k_]:
            fail("CHAIN_VS_CAD", f"cr_answers {k_} no longer an inner 38 x 18 chain")
    chains = {k: {"w": ch[k]["inner_mm"][0], "h": ch[k]["inner_mm"][1], "r": ch[k]["radius_mm"], "bend_h_avail": None,
                  "outer_h": ch[k]["outer_h_assumed_mm"], "src": ("" if ch[k].get("cad") else "REQUEST ") + ch[k]["source"][:60]} for k in ("x_chain", "y_chain")}
    chains["tc_chain"] = {"w": max(tw, th), "h": min(tw, th), "r": tr, "bend_h_avail": None, "outer_h": ch["tc_chain"]["outer_h_assumed_mm"], "src": "CAD"}
    hl = ch["head_loop"]
    loops = {}
    for inst in hl["instances"]:
        name = f"head_loop:{inst}"
        members = [r for r in rows if name in r["route"].split("+")]
        ct_ = hl["cable_types"][inst]
        t = TYPES[ct_]
        n = sum(int(r["conductors"]) for r in members)
        r_loop = hl["loop_radius_mm"]
        r_min = t.get("bend_radius_min_mm", t["bend_factor"] * t["od_mm"])
        free = hl["stroke_mm"] / 2 + math.pi * r_loop + 20.0
        loops[name] = {"cable": ct_, "conductors": n, "ways": t["ways"], "loop_radius_mm": r_loop, "min_radius_mm": r_min,
                       "free_length_mm": round(free, 1), "src": "CAD machine_family CR-L2"}
        if any(r["cable_type"] != ct_ for r in members):
            fail("LOOP_CABLE_TYPE", f"{name}: every conductor group must be in the one {ct_} flat cable")
        if n > t["ways"]:
            fail("LOOP_WAYS", f"{name}: {n} conductors > {t['ways']} ways")
        if r_loop + 1e-9 < r_min:
            fail("LOOP_BEND_RADIUS", f"{name}: loop R {r_loop} < cable minimum {r_min} mm")
        if free + 150.0 > 450.0 + 1e-6:      # 450 mm cable (build_harness CHAIN_LEN head_loop) = free loop + about 150 mm clamps and adapter tails
            fail("LOOP_LENGTH", f"{name}: free loop {free:.0f} mm + 150 mm terminations does not fit the 450 mm flat cable")
        if hl.get("cad_free_length_mm") and free + 1e-6 < hl["cad_free_length_mm"]:
            fail("LOOP_LENGTH", f"{name}: free loop {free:.0f} mm < CAD free length {hl['cad_free_length_mm']} mm")
    info["head_loops"] = loops
    info["ffc_check"] = ffc_check(rows, TYPES, sysd, cp, fail, warn) if cp else {}
    info["conductor_protection"] = conductor_protection(rows, TYPES, sysd, fail, warn)
    open_req = {c["chain"]: c["id"] for c in sysd.get("open_cad_requests", [])}
    maxfill = sysd["max_fill_fraction"]
    chain_report = {}
    for name, c in chains.items():
        members = [r for r in rows if name in r["route"].split("+") and "RESERVED" not in r["notes"]]
        n_cond, area, need_r, problems = 0, 0.0, 0.0, []
        for r in members:
            t = TYPES[r["cable_type"]]
            n = int(r["conductors"])
            k = n if t.get("single_core") else 1
            n_cond += n
            area += k * math.pi / 4 * t["od_mm"] ** 2
            need_r = max(need_r, t["bend_factor"] * t["od_mm"])
            if not t.get("chain_rated"):
                problems.append(("NOT_CHAIN_RATED", f"{r['cable_id']} ({r['cable_type']}) in {name}"))
            if t["od_mm"] > c["h"]:
                problems.append(("CABLE_TOO_THICK", f"{r['cable_id']} OD {t['od_mm']} > inner height {c['h']}"))
        fill = area / (c["w"] * c["h"])
        rep = {"source": c["src"], "inner_mm": [c["w"], c["h"]], "bend_radius_mm": c["r"], "cables": len(members), "conductors": n_cond,
               "conductor_area_mm2": round(area, 1), "fill": round(fill, 2), "max_fill": maxfill, "required_bend_radius_mm": round(need_r, 1)}
        if fill > maxfill:
            problems.append(("CHAIN_FILL", f"{name}: fill {fill:.2f} > {maxfill} ({n_cond} conductors, {area:.1f} mm2 in {c['w']} x {c['h']})"))
        if need_r > c["r"] + 1e-9:
            problems.append(("CHAIN_BEND_RADIUS", f"{name}: cables need R >= {need_r:.1f} mm, chain R {c['r']}"))
        if c["bend_h_avail"] is not None:
            bh = 2 * c["r"] + c["outer_h"]
            rep["bend_height_mm"] = bh
            rep["bend_height_available_mm"] = c["bend_h_avail"]
            if bh > c["bend_h_avail"]:
                problems.append(("CHAIN_BEND_HEIGHT", f"{name}: 2R + outer height = {bh} mm > {c['bend_h_avail']} mm available"))
        kind = name.split(":")[0]
        for code, msg in problems:
            if kind in open_req or name in open_req:
                warn(code, f"{msg} [open CAD request {open_req.get(name, open_req.get(kind))}]")
            else:
                fail(code, msg)
        if c["src"].startswith("REQUEST"):
            warn("CHAIN_NOT_IN_CAD", f"{name}: sized from the request in system.yaml, not from CAD")
        rep["status"] = "fits" if not problems else ("open CAD request " + open_req.get(name, open_req.get(kind, "?")))
        chain_report[name] = rep
    for r in rows:
        for tok in r["route"].split("+"):
            if tok != "fixed" and tok not in chains and tok not in loops:
                fail("ROUTE_UNKNOWN", f"{r['cable_id']}: {tok}")

    # ---------------- voltage drop, ampacity, connectors ----------------
    # harnessgen checks: ampacity, fuse protects cable, voltage drop < 3 %, connector ratings / crimp gauge / positions
    fuses = ({spec["fuse"]: spec["fuse_a"] for spec in sysd["rails"].values()} | {k: v["fuse_a"] for k, v in sysd["branches"].items()}
             | {k: v["fuse_a"] for k, v in sysd.get("inline_fuses", {}).items()})
    hg_rules = {"fuses": fuses, "fuse_pattern": r"^(F\d+)", "fuse_exempt_types": ["PLUG_IN"],
                "voltage_classes": [{"prefix": "24V", "volts": 24.0}, {"prefix": "5V", "volts": 5.0}, {"prefix": "0V", "volts": 24.0},
                                    {"prefix": "MAINS", "volts": 230.0}, {"prefix": "TEC", "volts": 24.0}, {"prefix": "HEATER", "volts": 24.0}],
                "max_drop_pct": 3.0, "resistivity_ohm_mm2_per_m": RHO,
                "pin_count_exempt_families": ["Molex Mini-Fit Jr"], "pin_count_exempt_ends": ["JST-PH 6"]}
    hg_find, drops = hg.check_rows(rows, TYPES, CONN, hg_rules)
    for f_ in hg_find:
        fail(f_["code"], f_["msg"])
    # end-to-end positive + negative path for the longest feeds (cable resistances only)
    def leg(cid, legs_override=None):
        return hg.cable_resistance(next(x for x in rows if x["cable_id"] == cid), TYPES, hg_rules, legs_override)
    paths = {"head MOTION_H (3.85 A)": ([("W-P-01", 15.9), ("W-P-06", 11.05), ("W-P-07", 11.05), ("W-P-08", 11.05), ("W-P-09", 11.05), ("W-P-10", 11.05),
                                        ("W-P-28", 3.85), ("W-P-02", 15.9)]),
             "THERM_AUX (14.6 A)": ([("W-P-50", 15.6), ("W-P-14", 14.6), ("W-P-15", 14.6), ("W-P-16", 14.6), ("W-P-27", 14.6), ("W-K-16", 14.6), ("W-K-10", 5.84), ("W-K-13", 5.84)]),
             "X motor (2.0 A)": ([("W-P-01", 19.7), ("W-P-06", 10.7), ("W-P-07", 10.7), ("W-P-08", 10.7), ("W-P-09", 10.7), ("W-P-10", 10.7),
                                  ("W-P-18", 5.5), ("W-P-30", 2.0), ("W-P-02", 19.7)]),
             "TC-1 THERM (16.3 A)": ([("W-P-03", 16.3), ("W-P-34", 16.3), ("W-P-35", 16.3), ("W-P-36", 16.3)])}
    e2e = {}
    for name, segs in paths.items():
        v = sum(leg(cid) * i for cid, i in segs)
        e2e[name] = {"volts": round(v, 2), "pct_of_24V": round(100 * v / 24, 1)}
        if v / 24 > 0.05:
            warn("END_TO_END_DROP", f"{name}: {v:.2f} V ({100 * v / 24:.1f} %) excluding fuse/contact resistance")
    info["end_to_end_drop_cables_only"] = e2e
    info["worst_cable_drops_pct"] = dict(sorted(drops.items(), key=lambda kv: -kv[1])[:6])

    # ---------------- ADR-003: bought thermal control, gate relays, watchdog, head supply, wiring rules ----------------
    info["adr003"] = adr003_checks(cp, sysd, rows, TYPES, CONN, fail, warn) if cp else {}
    ts = sysd["therm_switch"]
    if ts["break_a_at_32vdc"] < sysd["psu_short_circuit_a"]:
        fail("THERM_SWITCH_BREAK", f"break rating {ts['break_a_at_32vdc']} A < supply short-circuit current {sysd['psu_short_circuit_a']} A")
    info["review_2026_11_13"] = review_checks(cp, sysd, rows, fail, warn) if cp else {}

    # ---------------- cabinet ----------------
    cab = yaml.safe_load((HERE / "control_bay.yaml").read_text())
    plates = cab["plates"]
    placed = []
    heat = 0.0
    for it in cab["items"]:
        heat += it["heat_w"] * it.get("qty", 1)
        if "at" not in it:
            continue
        pl = plates[it["plate"]]
        x0, y0 = it["at"]
        fx, fy, fz = it["footprint_mm"]
        if it.get("rot", 0) == 90:
            fx, fy = fy, fx
        if x0 < pl["x"][0] - 1e-6 or x0 + fx > pl["x"][1] + 1e-6 or y0 < pl["y"][0] - 1e-6 or y0 + fy > pl["y"][1] + 1e-6:
            fail("CABINET_OFF_PLATE", f"{it['id']} on {it['plate']}")
        if fz > pl["height_available_mm"] + 1e-6:
            (warn if pl.get("height_request") else fail)("CABINET_TOO_TALL", f"{it['id']}: {fz} mm > {pl['height_available_mm']} mm on {it['plate']}"
                                                         + (f" [{pl['height_request']}]" if pl.get("height_request") else ""))
        for o in placed:
            if o[0] == it["plate"] and x0 < o[2] and o[1] < x0 + fx and y0 < o[4] and o[3] < y0 + fy:
                fail("CABINET_OVERLAP", f"{it['id']} overlaps {o[5]}")
        placed.append((it["plate"], x0, x0 + fx, y0, y0 + fy, it["id"]))
    if abs(heat - cab["totals"]["heat_w_bay_excluding_psus"] - cab["totals"]["heat_w_psus"]) > 0.5:
        fail("CABINET_HEAT_SUM", f"items sum {heat:.1f} W, totals say {cab['totals']['heat_w_bay_excluding_psus'] + cab['totals']['heat_w_psus']}")
    info["cabinet_heat_w"] = round(heat, 1)
    info["budgets"] = family_budgets(sysd, rows, cp)
    (HERE / "out").mkdir(exist_ok=True)
    (HERE / "out" / "budgets.json").write_text(json.dumps(info["budgets"], indent=1))
    info["enclosure_vs_cad"] = enclosure_checks(sysd, cab, fail, warn)

    # ---------------- bench rig BR-1 ----------------
    rig_cfg = ROOT / "firmware/lh1/bench_rig.cfg"
    rig_csv = HERE / "bench_rig_wiring.csv"
    if (ROOT / "cad/lh1/bench_rig/out/report.json").exists():
        if not rig_cfg.exists() or not rig_csv.exists():
            fail("BENCH_RIG_MISSING", "bench rig report exists but bench_rig.cfg / bench_rig_wiring.csv do not")
        else:
            rc = load_cfg(rig_cfg)
            rrows = list(csv.DictReader(open(rig_csv)))
            rpins = {}
            for sec in rc.sections():
                for key, val in rc.items(sec):
                    if key.endswith("pin"):
                        pin = pin_name(val)
                        adc_ = sec.startswith("gcode_button") and rc.has_option(sec, "analog_range")
                        if pin in rpins and not (adc_ and rpins[pin].startswith("ADC ")):
                            fail("RIG_PIN_REUSED", f"{pin} in [{sec}] and {rpins[pin]}")
                        rpins[pin] = ("ADC " + sec) if adc_ else sec
                        # NO-type inputs by design (broken wire = the safe reading): Z-up, coupling latch, Hall code bits (rev L),
                        # the rig lift bar OPEN-height switch (ADR-014)
                        no_type = ("gcode_button rig_zup", "gcode_button rig_latch", "gcode_button rig_bar_open") + tuple(f"gcode_button rig_hall{i}" for i in range(4))
                        if key == "endstop_pin" and not val.startswith("^") or (key == "pin" and sec.startswith("gcode_button") and sec not in no_type and val.split()[0].startswith("^!")):
                            fail("RIG_INPUT_NOT_FAILSAFE", f"[{sec}] {val}: NC switches need '^' without '!'")
            wired = {p_.strip() for r in rrows for p_ in r["fw_pins"].split(",") if p_.strip()}
            for pin, sec in rpins.items():
                if pin not in wired:
                    fail("RIG_PIN_NOT_WIRED", f"bench_rig.cfg [{sec}] pin {pin} not in bench_rig_wiring.csv")
            for p_ in wired - set(rpins):
                fail("RIG_WIRED_PIN_NOT_IN_CFG", p_)
            rep_ = json.loads((ROOT / "cad/lh1/bench_rig/out/report.json").read_text())
            fwn = rep_.get("firmware_numbers_rev_g4") or {}
            if not fwn:
                fail("RIG_CAD_NUMBERS_MISSING", "bench report has no firmware_numbers_rev_g4")
            BRv = lambda k_: rc.get("gcode_macro _BR", f"variable_{k_}")
            rig_tab = {}
            for tool_, sx in fwn.get("station_x", {}).items():
                for k_, xk in (("tip_rack", "x_rack"), ("plate_nest", "x_plate"), ("balance", "x_balance"), ("eject_B", "x_eject"),
                               ("strip_comb", f"x_comb_{tool_}")):
                    if abs(float(BRv(xk)) - sx[k_]) > 1e-6:
                        fail("RIG_STATION_X", f"_BR {xk} {BRv(xk)} != report firmware_numbers_rev_g4.station_x.{tool_}.{k_} {sx[k_]}")
            pmax_ = rc.getfloat("manual_stepper rig_z", "position_max")
            if "rig_z_position_max" in fwn and pmax_ > fwn["rig_z_position_max"] + 1e-6:
                fail("RIG_DEPTH_BEYOND_HARD_STOP", f"rig_z position_max {pmax_} > report rig_z_position_max {fwn['rig_z_position_max']}")
            for tool_, pp in fwn.get("per_pipette", {}).items():
                dmx = rc.getfloat("gcode_macro _BR", f"variable_depth_max_{tool_}")
                zcb = rc.getfloat("gcode_macro _BR", f"variable_z_comb_{tool_}")
                stop_ = pp.get("zhs1_stop_s")
                rig_tab[tool_] = {"strip_nominal": zcb, "strip_worst": pp.get("comb_strip_stroke_worst_s"), "depth_max": dmx,
                                  "cad_depth_max": pp.get("depth_max"), "stop": stop_}
                if dmx > pp["depth_max"] + 1e-6 or (stop_ is not None and dmx > stop_ - 1.0 + 1e-6):
                    fail("RIG_DEPTH_BEYOND_HARD_STOP", f"_BR depth_max_{tool_} {dmx} > CAD depth_max {pp['depth_max']} (stop {stop_} - 1)")
                if dmx > pmax_ + 1e-6:
                    fail("RIG_DEPTH_BEYOND_HARD_STOP", f"_BR depth_max_{tool_} {dmx} > rig_z position_max {pmax_}")
                if abs(zcb - pp["comb_strip_stroke_nominal_s"]) > 1e-6:
                    fail("RIG_STRIP_STROKE", f"_BR z_comb_{tool_} {zcb} != report {pp['comb_strip_stroke_nominal_s']}")
                worst_ = pp.get("comb_strip_stroke_worst_s", zcb)
                if not (zcb < dmx and worst_ < dmx and (stop_ is None or dmx < stop_)):
                    fail("RIG_STRIP_BEYOND_LIMIT", f"{tool_}: strip {zcb} (worst {worst_}) < depth limit {dmx} < stop {stop_} does not hold")
            info["rig_depths"] = rig_tab
            for mac in ("BR_PLUNGER_SWEEP", "BR_DISPENSE", "BR_PICK", "BR_STRIP", "BR_ENDURANCE", "BR_CT_PARK", "BR_CT_PICK", "BR_CT_EXCHANGE", "BR_CT_LOCK_READ", "BR_CT_CODE_READ", "BR_SYRINGE"):
                if not rc.has_section(f"gcode_macro {mac}"):
                    fail("RIG_MACRO_MISSING", mac)
            ej_b = rep_.get("stations_nozzle_x_mm", {}).get("eject_B")
            if ej_b is not None and abs(rc.getfloat("gcode_macro _BR", "variable_x_eject") - ej_b) > 1e-6:
                fail("RIG_STATION_X", f"_BR x_eject != report eject_B {ej_b}")
            spec_r = importlib.util.spec_from_file_location("p_rig", ROOT / "cad" / "lh1" / "params.py")
            P_r = importlib.util.module_from_spec(spec_r); spec_r.loader.exec_module(P_r)
            if abs(rc.getfloat("gcode_macro _BR", "variable_z_ejector") + P_r.Z_EJECT_TRAVEL) > 1e-6 or abs(rc.getfloat("manual_stepper rig_z", "position_min") + P_r.Z_EJECT_TRAVEL) > 1e-6:
                fail("RIG_EJECT_TRAVEL", "rig z_ejector / position_min must equal -Z_EJECT_TRAVEL")
            rb = lambda sec_: rc.get(sec_, "gcode", fallback="") if rc.has_section(sec_) else ""
            allb = "\n".join(rb(s_) for s_ in rc.sections())
            if re.search(r"MANUAL_STEPPER STEPPER=rig_[zx]", allb):
                fail("RIG_UNGUARDED_MOVE", "rig Z/X moves must go through _BR_Z / _BR_X / _BR_XSLIDE")
            if "inverted_probe" not in rb("gcode_macro BR_HOME") or "_BR_REQUIRE_ZUP" not in rb("gcode_macro BR_HOME"):
                fail("RIG_HOMING", "BR_HOME needs back-off-and-retrigger and the Z-up check")
            for sec_ in ("gcode_macro SET_GCODE_VARIABLE",):
                if "action_raise_error" not in rb(sec_):
                    fail("RIG_STATE_UNPROTECTED", "bench_rig.cfg must refuse SET_GCODE_VARIABLE (rig state)")
            if re.search(r"(?m)^\s*SET_GCODE_VARIABLE MACRO=", allb):
                fail("RIG_STATE_UNPROTECTED", "rig macros must write state with SET_GCODE_VARIABLE_BASE")
            if re.search(r"force_g\s*\|\s*default", rb("gcode_macro _BR_FORCE")) or "is_calibrated" not in rb("gcode_macro _BR_FORCE"):
                fail("RIG_LOADCELL_DEFAULT", "_BR_FORCE must refuse an uncalibrated / missing reading, never default to 0")
            for mac_, lc_ in (("BR_PICK", "rack"), ("BR_STRIP", "comb"), ("BR_STRIP", "eject"), ("BR_PLUNGER_SWEEP", "yoke")):
                if f"_BR_LC_READY LC={lc_}" not in rb(f"gcode_macro {mac_}"):
                    fail("RIG_LOADCELL_NOT_CHECKED", f"{mac_} must run _BR_LC_READY LC={lc_} before its force-limited stroke")
            n_lc = len([s_ for s_ in rc.sections() if s_.startswith("load_cell ")])
            if n_lc != 5 or "five [load_cell]" not in rig_cfg.read_text():
                fail("RIG_HEADER", f"bench_rig.cfg has {n_lc} [load_cell] sections; the header must say five")
            if "depth_max_" not in rb("gcode_macro _BR_Z") or "_BR_REQUIRE_ZUP" not in rb("gcode_macro _BR_X"):
                fail("RIG_GUARDS", "_BR_Z must enforce the depth limit and _BR_X the Z-up switch")
            if "rerack" in rb("gcode_macro BR_STRIP") .replace("rerack removed", ""):
                fail("RIG_RERACK", "BR_STRIP rerack must stay removed")
            if not rc.get("gcode_button rig_zup", "pin").split()[0].startswith("^!"):
                fail("RIG_ZUP_NOT_FAILSAFE", "rig Z-up switch must be NO closed at the top ('^!')")
            if "METHOD" not in rc.get("gcode_macro BR_STRIP", "gcode"):
                fail("RIG_MACRO_MISSING", "BR_STRIP needs a METHOD parameter (comb / ejector)")
            info["bench_rig"] = {"pins": len(rpins), "cables": len(rrows)}
    info["mvp"] = mvp_checks(sysd, fail, warn)
    findings.append({"severity": "warning", "code": "NOT_LOADED_BY_KLIPPER",
                     "msg": "printer.cfg is parsed for syntax, duplicates and wiring only; Klipper has not loaded it"})
    findings.append({"severity": "warning", "code": "NOT_SAFETY_ASSESSED",
                     "msg": "no electrical safety assessment has been done (system.yaml safety_assessment)"})
    out = {"passed": not [f for f in findings if f["severity"] == "error"], "budget": budget, "pins_assigned": len(pins),
           "cables": len(rows), "devices": len(devs), "chains": chain_report, "info": info, "findings": findings}
    (HERE / "out").mkdir(exist_ok=True)
    (HERE / "out" / "check_report.json").write_text(json.dumps(out, indent=1))
    print(json.dumps(out, indent=1))
    return 0 if out["passed"] else 1


if __name__ == "__main__":
    sys.exit(main())
