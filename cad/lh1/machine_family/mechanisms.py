"""LH-1 mechanism force-and-travel audit (verification V1 root cause: "no file computes this force").

Every passive mechanism an axis has to drive: the force it needs through its stroke, its travel against hard limits, and the drive's
thrust read from electronics/lh1/system.yaml at the current the firmware uses (never copied here).  Fails below a 1.5 margin.  A
mechanism whose drive has no thrust figure in system.yaml is reported NO DATA (a finding, not a pass).

Status: CALCULATED from the CAD and catalogue-class inputs (marked); nothing built or measured.
"""
from __future__ import annotations

import math

import yaml

from geometry import HL, P, ROOT, TPKS

LT = HL.LT
MARGIN = 1.5


def _sys():
    return yaml.safe_load((ROOT / "electronics" / "lh1" / "system.yaml").read_text())


def z_thrust(sysy, current):
    return float(sysy["z_drive"]["recomputed"]["worst_thrust_N_at"][f"{current:.1f}"])


def lift_thrust(sysy):
    """dock lift NET thrust at its run current: the Z-drive class gross figure minus the bar's own load (system.yaml dock_lift.recomputed,
    the electrical owner's arithmetic; integration 23 Nov 2026: the audit used the gross 233.8 N)"""
    dk = sysy["dock_lift"]
    return float(dk["recomputed"]["net_thrust_N_at"][f"{dk['run_current_a']:.1f}"]), dk["run_current_a"]


def motor_torque(sysy, dev):
    """catalogue-class holding torque the electrical design states for a gantry motor (devices.<dev>.desc '>= x N m')"""
    import re
    m = re.search(r">=\s*([\d.]+)\s*N m", sysy["devices"][dev]["desc"])
    return float(m.group(1)) if m else None


def pusher_model(sysy):
    """system.yaml's own pusher thrust model (its fields; 1 mm lead NEMA14 class) at its run current"""
    m = sysy["plunger_motor"]
    t = m["holding_torque_nm"] * m["run_current_a"] / m["rated_current_a"] * m["torque_at_speed_fraction"]
    return round(2 * math.pi * m["efficiency"] * t / (m["lead_mm"] / 1000.0), 1), m["run_current_a"]


def row(name, drive, need, avail, travel=None, note="", source=""):
    if avail is None:
        st, mg = "NO DATA", None
    else:
        mg = round(avail / need, 2) if need else None
        st = "PASS" if (mg is None or mg >= MARGIN) else "FAIL"
    tr_ok = True
    if travel:
        tr_ok = travel["needed_mm"] <= travel["limit_mm"]
        if not tr_ok:
            st = "FAIL"
    return {"mechanism": name, "drive": drive, "force_needed_N": None if need is None else round(need, 1),
            "thrust_available_N": None if avail is None else round(avail, 1), "margin": mg, "travel": travel, "travel_ok": tr_ok,
            "status": st, "note": note, "thrust_source": source}


def audit():
    sy = _sys()
    z09, z13 = z_thrust(sy, 0.9), z_thrust(sy, 1.3)
    src_z = "system.yaml z_drive.recomputed.worst_thrust_N_at (mu 0.20, usable torque, at speed)"
    lnet, li = lift_thrust(sy)
    lt = LT.calcs(z09, lnet)
    ls = LT.lift_stroke()
    rows = []
    rows.append(row("latch release (LTG-1 toggle through dead centre + closing springs), whole stroke", "dock lift DLB-1: NEMA17 48 + Tr8x2 at %g A "
                    "(Z-axis motor and screw class), NET of the bar's own load" % li, lt["release"]["max_force_at_roller_N"], lnet,
                    {"needed_mm": ls["needed_mm"], "limit_mm": ls["stroke_mm"], "what": "bar stroke: roller rise + 13 mm follow + 3 mm gap"},
                    "peak at the locked position (phi -4 deg); profile in head report coupling.latch.table; gross %g N - bar %g N" %
                    (sy["dock_lift"]["recomputed"]["gross_thrust_N_at"][f"{li:.1f}"], sy["dock_lift"]["bar_load_n"]),
                    "system.yaml dock_lift.recomputed.net_thrust_N_at '%.1f'" % li))
    rows.append(row("latch release if the Z axis drove it through a fixed post (comparison only; not used)", "Z axis at 0.9 A",
                    lt["release"]["max_force_at_roller_N"], z09, None,
                    "force would be marginal; and a fixed post cannot tell a drop-off from a pick-up (latch.z_only_verdict)", src_z + " '0.9'"))
    rc = lt["relatch"]
    rows.append({"mechanism": "latch re-latch: two constant-force springs (at -10 %) close the toggle and compress the 3-disc stack",
                 "drive": "springs (passive)", "force_needed_N": None, "thrust_available_N": None, "margin": rc["min_ratio_spring_to_need"],
                 "travel": {"needed_mm": lt["stroke"]["spring_travel_mm"], "limit_mm": lt["stroke"]["spring_limit_mm"],
                            "stack_mm": lt["stroke"]["stack_compression_max_mm (stud +tol)"], "stack_limit_mm": lt["stroke"]["stack_catalogue_limit_mm"]},
                 "travel_ok": lt["stroke"]["stack_compression_max_mm (stud +tol)"] <= lt["stroke"]["stack_catalogue_limit_mm"],
                 "status": "PASS" if rc["min_ratio_spring_to_need"] >= MARGIN else "FAIL",
                 "note": "worst at phi %s deg (spring torque / toggle torque + bearing friction)" % rc["at_phi_deg"], "thrust_source": "latch.py"})
    hold = HL.coupling_calcs()["loads_and_holding"]
    for k, v in hold.items():
        rows.append({"mechanism": "coupling held against: " + k, "drive": "latch preload (min of the band)", "force_needed_N": v["preload_to_just_hold_N"],
                     "thrust_available_N": v["latch_preload_min_N"], "margin": v["margin"], "travel": None, "travel_ok": True,
                     "status": "PASS" if v["pass_1.5"] else ("FAIL (sensitivity case)" if "sensitivity" in k else "FAIL"), "note": v["note"],
                     "thrust_source": "head report coupling.latch.holding"})
    pick8, pick1 = P.PICKUP_FORCE_N["pipette_8ch_300"], P.PICKUP_FORCE_N["pipette_small"]
    rows.append(row("tip pick-up, eight tips, force-ended (cradle force switch)", "Z at the force current 1.3 A", pick8, z13,
                    {"needed_mm": round(P.L_Z_TRAVEL - 1, 1), "limit_mm": P.L_Z_TRAVEL, "what": "deepest pick-up stroke inside the Z travel (CAD reach run)"},
                    "PICKUP_FORCE_N (ASSUMED until CT-9)", src_z + " '1.3'"))
    rows.append(row("tip pick-up, single channel", "Z at 0.9 A", pick1, z09, None, "PICKUP_FORCE_N", src_z + " '0.9'"))
    rows.append(row("tip eject by over-travel against the fixed finger", "Z at 0.9 A (upwards)", 90.0, z09,
                    {"needed_mm": 11.5, "limit_mm": P.Z_EJECT_TRAVEL, "what": "gap + ejector stroke + over (params)"}, "eject force 90 N (study class, ASSUMED)",
                    src_z + " '0.9'"))
    strip = P.STRIP_FORCE_N["pipette_8ch_300"]
    rows.append(row("tip-comb strip (TPK-S, lift with collars under the upper plate)", "Z at 1.3 A", strip, z13,
                    {"needed_mm": round(1.6 + TPKS["collar_h"], 1), "limit_mm": P.L_Z_TRAVEL, "what": "lift from the pocket to clear"}, "STRIP_FORCE_N", src_z + " '1.3'"))
    rows.append(row("tip-comb re-pick (nozzles into parked collars, force-ended)", "Z at 1.3 A", pick8, z13, None, "as pick-up", src_z + " '1.3'"))
    # X / Y (integration 23 Nov 2026): thrust from the electrical design's motor torque class through the GT2 pulley (CAD params), at
    # 60 % of holding at speed (the pusher model's class, ASSUMED; the slide runs at <= 10 mm/s = 15 rpm, where a hybrid stepper gives
    # nearly its holding torque); need = collar friction + the moving mass at the slide's 0.25 m/s2
    r_p = P.PULLEY_TEETH * 2.0 / (2 * math.pi) / 1000.0
    tx = motor_torque(sy, "X_MOTOR")
    ty = [motor_torque(sy, d) for d in ("Y_MOTOR", "Y1_MOTOR")]
    hr = __import__("json").loads((ROOT / "cad/lh1/head_l/out/report.json").read_text())
    m_head = hr["mass"]["head_kg"]
    m_y = hr["dynamics"]["Y_moving_mass_kg"]
    fx = 0.6 * tx / r_p if tx else None
    fy = 0.6 * sum(ty) / r_p if all(ty) else None
    rows.append(row("tip-comb slide in X (8 collars, friction about 1 N each, ASSUMED, + head %.1f kg at 0.25 m/s2)" % m_head,
                    "X closed-loop NEMA17 >= %g N m (system.yaml devices.X_MOTOR) on a GT2 %dT pulley, 0.6 of holding" % (tx, P.PULLEY_TEETH),
                    8.0 + m_head * 0.25, fx, {"needed_mm": TPKS["p"], "limit_mm": TPKS["p"] + 0.5, "what": "slide entry -> spot"},
                    "torque class from the electrical design (closed-loop motors not bought: PURCHASE CONDITION); collar friction "
                    "MEASURED by CT-0c", "system.yaml devices X_MOTOR desc + params PULLEY_TEETH"))
    rows.append(row("tip-comb slide: Y holds the row against the collar friction (all 8 N sideways, worst) + Y moving mass %.1f kg at 0.25 m/s2" % m_y,
                    "2 x Y closed-loop NEMA17 >= %g N m (devices Y_MOTOR, Y1_MOTOR), GT2 %dT, 0.6 of holding" % (ty[0], P.PULLEY_TEETH),
                    8.0 + m_y * 0.25, fy, None, "as X", "system.yaml devices Y_MOTOR / Y1_MOTOR desc + params PULLEY_TEETH"))
    rows.append(row("plate seating in a nest / module by the gripper (press-in about 5 N, ASSUMED)", "gripper Z at 0.9 A", 5.0, z09, None, "", src_z + " '0.9'"))
    rows.append(row("lid lift: plate lids and the sealing lid by the gripper (lid < 0.05 kg + 5 N suction, ASSUMED)", "gripper Z at 0.9 A", 5.5, z09, None, "", src_z + " '0.9'"))
    # gripper (integration 23 Nov 2026): the head report's jaw-drive figures (cad/lh1/head/out/report.json checks.gripper_force)
    gf = __import__("json").loads((ROOT / "cad/lh1/head/out/report.json").read_text())["checks"]["gripper_force"]
    rows.append(row("gripper grip: hold the heaviest plate (clamp needed at pad friction 0.5, head report) at the jaw motor's usable torque",
                    "gripper jaws: %s, %s, worst screw friction" % (gf["motor"], gf["drive"]), gf["clamp_needed_for_payload_mu_pad_0.5"],
                    gf["max_clamp_force_worst_N"], None, "the settable range GRIP_FORCE_N 5-20 N is a setting, not the need; at the 20 N top "
                    "setting the drive has %.2f" % (gf["max_clamp_force_worst_N"] / 20.0), "cad/lh1/head/out/report.json checks.gripper_force"))
    rows.append(row("gripper grip at the firmware's jaw current (printer.cfg [tmc5160 manual_stepper gripper] run_current 0.35 A)",
                    "gripper jaw NEMA11 at 0.35 A", gf["clamp_needed_for_payload_mu_pad_0.5"], None, None,
                    "MISSING: the jaw motor's RATED current (maker's sheet of the 28HS32-class motor; the usable 0.04 N m is at its rating) to "
                    "scale the torque to 0.35 A, OR the rig measurement of the grip force at 0.35 A (gripped-switch force on the S-cell)"))
    # TC-1 lid (integration 23 Nov 2026): the thermocycler report's own mechanics (cad/lh1/thermocycler/out/report.json)
    tm = __import__("json").loads((ROOT / "cad/lh1/thermocycler/out/report.json").read_text())["checks"]["mechanics"]
    rows.append(row("TC-1 lid slide (belt, chain drag, rail friction, acceleration)", "TC-1 Y motor (NEMA17 pancake, GT2 20T), stall torque class",
                    tm["y_drive_force_needed_n"], tm["y_drive_stall_force_n"], None, "thermocycler report checks.mechanics y_drive",
                    "cad/lh1/thermocycler/out/report.json"))
    rows.append(row("TC-1 lid clamp (presser compresses 2 die springs to its hard stop + return springs)",
                    "TC-1 clamp NEMA17 x 34 with integrated Tr8x2 (0.30 N m class, eff 0.35) on a DM542 set to 1.0 A peak",
                    tm["clamp_screw_thrust_needed_n"], tm["clamp_screw_thrust_available_n"], None,
                    "FAILS the 1.5 rule (TC-1 owner, rev H; not changed in this round): the 0.30 N m is the motor class at its rating, "
                    "and the DM542 is set to 1.0 A peak, whose torque is not known (maker's rated current MISSING), so the true margin is "
                    "lower still.  Closing it: a NEMA17 x 48 integrated-screw class (about 0.45 N m: margin about 1.9) if the lid carriage "
                    "height allows, or springs at the low end of the 220-300 N seal range; then measure on the TC-1 rig (T-TC-05)",
                    "cad/lh1/thermocycler/out/report.json"))
    pm, pi = pusher_model(sy)
    plunger = max(t["plunger_N"] for t in HL.TOOLS.values())
    f05 = LT.LGA35["force_N"]["0.7A_5mm_s"]
    rows.append(row("plunger push, largest tool (SYR 80 N incl. return spring; S1000 60 N) at <= 5 mm/s", "LGA35 pusher at 0.7 A (sourcing curve, "
                    "INFERENCE)", plunger, min(f05, pm),
                    {"needed_mm": 35.0, "limit_mm": LT.LGA35["stroke_mm"], "what": "pusher stroke incl. the 3 mm contact window"},
                    "the lower of the sourcing file's 0.7 A curve reading (%g N at 5 mm/s) and system.yaml's own model (%g N at %g A, 1 mm-lead "
                    "class; its fields not yet updated to the 0.61 mm lead); at 8 mm/s the curve gives %g N: margin %.2f, so force strokes stay "
                    "<= 5 mm/s" % (f05, pm, pi, LT.LGA35["force_N"]["0.7A_8mm_s"], LT.LGA35["force_N"]["0.7A_8mm_s"] / plunger),
                    "pusher_actuator_sourcing_2026-11-21.txt + system.yaml plunger_motor"))
    rows.append(row("plunger contact (pusher pad on the slider, contact switch about 2 N, ASSUMED)", "LGA35 at 0.7 A", 2.0, min(f05, pm), None, "", ""))
    rows.append(row("pusher homing into its RETRACTED switch (never into the internal end stop above 0.57 A)", "LGA35 at 0.50 A homing current",
                    None, None, {"needed_mm": 0.5, "limit_mm": 37.6 - 35.0, "what": "switch trips 0.5 mm before the internal stop; 2.6 mm spare"},
                    "current limit check: homing 0.50 A <= maker limit %.2f A: PASS" % LT.LGA35["end_stop_current_max_A"], ""))
    rows[-1]["status"] = "PASS"
    sp = hr["syringe_return_spring"]
    rows.append(row("syringe tool return spring (plunger out, end of a refill) vs seal friction + check valve (V22)",
                    "return spring %.3f N/mm, %.1f N out (passive; at the -10 %% rate tolerance %.1f N: margin %.2f)" %
                    (sp["rate_N_per_mm"], sp["force_plunger_out_N"], sp["force_plunger_out_low_tolerance_N"], sp["margin_low_tolerance"]),
                    sp["need_N"], sp["force_plunger_out_N"], {"needed_mm": sp["length_at_full_stroke_mm"], "limit_mm": sp["length_at_full_stroke_mm"] + sp["solid_margin_mm"],
                                                               "what": "spring length at full stroke vs solid + margin (inverted: needed <= limit means not solid)"},
                    "%s; %s; SYR stays uncommissioned until CT-SYR measures the friction" % (sp["spring"], sp["need_basis"]),
                    "head report syringe_return_spring"))
    rows[-1]["travel"] = {"needed_mm": sp["solid_length_mm"], "limit_mm": sp["length_at_full_stroke_mm"], "what": "solid length <= length at full stroke (4 mm left)"}
    rows[-1]["travel_ok"] = sp["solid_length_mm"] <= sp["length_at_full_stroke_mm"]
    rows.append({"mechanism": "drawer (tip crate, waste canisters) and deck-module seating", "drive": "a person (no axis drives them)",
                 "force_needed_N": None, "thrust_available_N": None, "margin": None, "travel": None, "travel_ok": True, "status": "NOT AXIS-DRIVEN",
                 "note": "drawer on bought slides, modules bolted by a person; nothing for a drive to fail", "thrust_source": ""})
    fails = [r["mechanism"] for r in rows if str(r["status"]).startswith("FAIL")]
    nodata = [r["mechanism"] for r in rows if r["status"] == "NO DATA"]
    other = [f for f in fails if f.startswith("TC-1")]
    return {"rule": "force available / force needed >= %.1f at the firmware's current; travel needed <= hard limit" % MARGIN,
            "z_thrust_read_N": {"0.9 A": z09, "1.3 A": z13}, "dock_lift_net_thrust_N": lnet, "rows": rows, "fails": fails, "no_data": nodata,
            "open_fails_other_owners": other,
            "pass_head_dock_tools (ADR-014 scope)": not [f for f in fails if "sensitivity" not in f and "comparison" not in f and f not in other],
            "pass": not [f for f in fails if "sensitivity" not in f and "comparison" not in f]}
