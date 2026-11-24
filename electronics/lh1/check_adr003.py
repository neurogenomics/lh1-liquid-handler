"""ADR-003 checks (bought parts in place of HIB-1 and TCB-1), called by electronics/lh1/check.py.

Each check reads the design files (system.yaml, wiring.csv, printer.cfg) and fails honestly:
  retired boards not referenced; all-tools-up + beam gate topology (K_TU, K_CH, beam receiver); host-loss watchdog
  in the TC-1 coil chain and NOT in the cold-block chain; lid-heater SSR on the switched THERM rail with its fuse;
  bought-part ratings against the loads; Octopus pin budget; head supply/CAN (lever splice, XT30 stubs, F22);
  wiring rules (wire number, colour, keyed connectors); tool-made joint count against the pre-ADR-003 count.
"""
import math
import re
from pathlib import Path

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[1]
RETIRED_DEVICES = ("TCB1", "TCB_TM", "HIB", "CAN_PIP", "PIP_MOD")


def pin_name(val):
    return re.sub(r"[\^!~]", "", val.split()[0])


def run(cp, sysd, rows, TYPES, CONN, fail, warn):
    out = {}
    body = lambda sec: cp.get(sec, "gcode") if cp.has_section(sec) and cp.has_option(sec, "gcode") else ""
    by = lambda a, b=None: [r for r in rows if a in (r["from_device"], r["to_device"]) and (b is None or b in (r["from_device"], r["to_device"]))]
    eob = sysd["ease_of_build"]
    rt = eob["ratings"]

    # ---- 1. retired boards: no device, no wiring row, no file dependency; RETIRED.txt present
    for d in RETIRED_DEVICES:
        if d in sysd["devices"] or by(d):
            fail("RETIRED_REFERENCE", f"{d} still in system.yaml devices or wiring.csv")
    for f in ("check.py", "build_harness.py", "draw_harness.py", "build_bench_rig.py", "fw_render_check.py"):
        src = (HERE / f).read_text()
        if re.search(r"electronics/tcb1|tcb1/|hib1/|firmware/tcb1", src.replace("RETIRED", "")):
            fail("RETIRED_REFERENCE", f"{f} still reads a retired board's files")
    for d in ("electronics/tcb1", "firmware/tcb1", "electronics/lh1/hib1"):
        if not (ROOT / d / "RETIRED.txt").exists():
            fail("RETIRED_NOTE_MISSING", f"{d}/RETIRED.txt")
    if "tcb_instances" in sysd or "tvs" in sysd:
        fail("RETIRED_REFERENCE", "system.yaml still has tcb_instances / tvs (retired with TCB-1 / HIB-1)")

    # ---- 2. all-tools-up + beam gate (interlocks.lid_needs_all_tools_up)
    g = {}
    tu = sysd["interlocks"]["tools_up_chain"]       # revision L: Z_UP_A -> Z_UP_B -> Z_UP_GRIP (three Z axes)
    g["chain_start"] = any(r["from_device"] == "XT" and "LOG7" in r["from_connector"] and r["to_device"] == tu[0] for r in rows)
    g["chain_end"] = any(r["from_device"] == tu[-1] and r["to_device"] == "XT" and "ZC" in r["to_connector"] for r in rows)
    g["k_ch_on_zc"] = any(r["from_device"] == "XT" and "ZC" in r["from_connector"] and r["to_device"] == "K_CH" for r in rows)
    g["rx_supply_on_zc"] = any(r["from_device"] == "XT" and "ZC" in r["from_connector"] and r["to_device"] == "TC_BEAM_RX" for r in rows)
    g["rx_out_drives_k_tu"] = any(r["from_device"] == "TC_BEAM_RX" and "OUT" in r["from_connector"] and r["to_device"] == "K_TU" and "A1" in r["to_pins"] for r in rows)
    g["k_tu_no_from_f23"] = any(r["from_device"] == "FT" and "F23" in r["from_connector"] and r["to_device"] == "K_TU" and "13" in r["to_connector"] for r in rows)
    drv_feeds = [r for r in rows if r["to_device"] in ("TC_DRV_SLIDE", "TC_DRV_CLAMP") and r["signal"].startswith("24V")]
    g["drivers_only_via_k_tu"] = bool(drv_feeds) and all(r["from_device"] in ("K_TU", "TC_DRV_SLIDE") for r in drv_feeds)
    g["k_tu_readback_pb6"] = any(r["from_device"] == "K_TU" and r["to_device"] == "MAIN_CTRL" and "PB6" in r["fw_pins"] for r in rows) and \
        cp.has_section("gcode_button lid_gate") and cp.get("gcode_button lid_gate", "pin").split()[0] == "^!PB6"
    g["k_ch_to_pg10"] = any(r["from_device"] == "K_CH" and "PG10" in r["fw_pins"] for r in rows)
    kd = sysd["devices"].get("K_TU", {}).get("desc", "")
    g["k_tu_forcibly_guided"] = "forcibly guided" in kd and "61810-3" in kd
    g["emitter_switched"] = any(r["from_device"] == "MAIN_CTRL" and r["to_device"] == "TC_BEAM_TX" and "PA8" in r["fw_pins"] for r in rows) and \
        cp.has_section("output_pin beam_emitter") and cp.getfloat("output_pin beam_emitter", "value") == 0 and cp.getfloat("output_pin beam_emitter", "shutdown_value") == 0
    bt = body("gcode_macro _LH_BEAM_TEST")
    g["self_test_order"] = 0 <= bt.find("beam_emitter VALUE=0") < bt.find("STATE=off") < bt.find("beam_emitter VALUE=1") < bt.find("STATE=on")
    for mac in ("LH_LID_HOME", "LH_LID_CLOSE", "LH_LID_OPEN"):
        b_ = body(f"gcode_macro {mac}")
        g[f"{mac}_guard_then_test_then_move"] = 0 <= b_.find("_LH_LID_GUARD") < b_.find("_LH_BEAM_TEST") < b_.find("MANUAL_STEPPER_BASE")
    lg = body("gcode_macro _LH_LID_GUARD")
    for k_, needle in (("guard_tools", "tools_up"), ("guard_door", "gcode_button door"), ("guard_gripper", "s.grip == 0"),
                       ("guard_homed", "homed"), ("guard_fault", "l.fault"), ("guard_z", "zpos_z_a")):
        g[k_] = needle in lg
    out["gate"] = g
    for k_, v_ in g.items():
        if not v_:
            fail("LID_GATE", f"all-tools-up/beam gate: {k_} not satisfied")
    chain_ma = rt["K_CH"]["coil_ma"] + rt["beam_rx"]["supply_ma"] + rt["K_TU"]["coil_ma"]
    out["gate"]["chain_current_ma"] = round(chain_ma, 1)
    if chain_ma > rt["z_up_switch"]["a_max"] * 1000:
        fail("LID_GATE", f"chain current {chain_ma:.0f} mA > Z_UP switch rating {rt['z_up_switch']['a_max'] * 1000:.0f} mA")
    if rt["K_TU"]["coil_ma"] > rt["beam_rx"]["out_ma_max"]:
        fail("LID_GATE", f"K_TU coil {rt['K_TU']['coil_ma']} mA > beam receiver output {rt['beam_rx']['out_ma_max']} mA")
    if rt["K_TU"]["contact_a"] < sysd["branches"]["F23"]["fuse_a"]:
        fail("LID_GATE", f"K_TU contact {rt['K_TU']['contact_a']} A < F23 {sysd['branches']['F23']['fuse_a']} A")

    # ---- 3. host-loss watchdog in the TC-1 coil chain, not in the cold-block chain
    def follow(start_dev, start_conn, stop_dev, signal_key):
        path, cur, conn, seen = [start_dev], start_dev, start_conn, set()
        for _ in range(12):
            nxt = [r for r in rows if r["from_device"] == cur and (conn is None or conn in r["from_connector"] or conn in r["from_pins"])
                   and signal_key in r["signal"] and r["cable_id"] not in seen]
            if not nxt:
                return path
            r = nxt[0]
            seen.add(r["cable_id"])
            cur, conn = r["to_device"], None
            path.append(cur)
            if cur == stop_dev:
                return path
        return path
    tc_path = follow("SR1", "34", "TC_RELAY", "coil chain")
    tb_path = follow("XT", "SAFE_E", "TB_RELAY", "coil chain")
    want_tc = ["SR1", "TC_TS_BLOCK", "TC_TS_SINK", "TC_TS_LID", "WDOG", "TC_RELAY"]
    # revision L: the 72 C cut-off and 70 C sink thermostat of each thermal module sit inside the module between its chain in/out
    # size family: the four modules B2, A3, A4, G4 (G0's 37 C module reserved)
    want_tb = ["XT"] + [d for d in ("TM_B2", "TM_A3", "TM_A4", "TM_G4")] + ["TB_RELAY"]
    w = {"tc_chain": tc_path, "tb_chain": tb_path}
    if tc_path != want_tc:
        fail("WATCHDOG_CHAIN", f"TC-1 coil chain {tc_path} != {want_tc}")
    if tb_path != want_tb or "WDOG" in tb_path:
        fail("WATCHDOG_CHAIN", f"cold-block coil chain {tb_path} != {want_tb} (no watchdog: the cold blocks hold on host loss)")
    if not any(r["from_device"] == "TC_RELAY" and "A1" in r["from_pins"] and r["to_device"] == "KM5" for r in rows):
        fail("WATCHDOG_CHAIN", "KM5's coil must hang on the same chain end as TC_RELAY (both channels drop on watchdog trip)")
    if not any(r["from_device"] == "TC_RELAY" and "A2" in r["from_pins"] and r["to_device"] == "MAIN_CTRL" and "PA3" in r["fw_pins"] for r in rows):
        fail("WATCHDOG_CHAIN", "TC_RELAY coil return must go to the Octopus HE1 (PA3, therm_enable)")
    if not any(r["from_device"] == "MAIN_CTRL" and r["to_device"] == "WDOG" and "PE5" in r["fw_pins"] for r in rows):
        fail("WATCHDOG_CHAIN", "WDOG heartbeat must come from the Octopus FAN1 (PE5)")
    hb = "output_pin wd_heartbeat"
    if not (cp.has_section(hb) and cp.getboolean(hb, "pwm") and cp.getfloat(hb, "value") == 0 and cp.getfloat(hb, "shutdown_value") == 0):
        fail("WATCHDOG_CHAIN", "[output_pin wd_heartbeat] must be pwm, value 0, shutdown_value 0")
    else:
        hz = 1.0 / cp.getfloat(hb, "cycle_time")
        lo, hi = rt["WDOG"]["hb_hz"]
        w["heartbeat_hz"] = hz
        if not lo <= hz <= hi:
            fail("WATCHDOG_CHAIN", f"heartbeat {hz} Hz outside the watchdog's {lo}-{hi} Hz")
    tset = cp.getfloat("gcode_macro _LH_THERM", "variable_wd_timeout_s")
    if tset != rt["WDOG"]["timeout_set_s"] or not rt["WDOG"]["timeout_s"][0] <= tset <= rt["WDOG"]["timeout_s"][1]:
        fail("WATCHDOG_CHAIN", f"_LH_THERM wd_timeout_s {tset} must equal the relay setting {rt['WDOG']['timeout_set_s']} within {rt['WDOG']['timeout_s']}")
    on = body("gcode_macro LH_THERM_ON")
    proof = [on.find(x) for x in ("RELAY=off CODE=RELAY_WELDED", "therm_enable VALUE=1", "RELAY=on CODE=CHAIN_OPEN", "wd_heartbeat VALUE=0",
                                  "wd_timeout_s + 1", "RELAY=off CODE=WATCHDOG_FAIL", "RELAY=on CODE=WATCHDOG_NO_REARM", "VARIABLE=on VALUE=1")]
    w["proof_test_order_ok"] = all(x >= 0 for x in proof) and proof == sorted(proof)
    if not w["proof_test_order_ok"]:
        fail("WATCHDOG_PROOF_TEST", "LH_THERM_ON must: weld check, enable, chain closed, stop heartbeat > timeout, relay dropped, re-arm, then arm")
    off = body("gcode_macro LH_THERM_OFF")
    if not (0 <= off.find("therm_enable VALUE=0") < off.find("RELAY=off CODE=RELAY_WELDED")):
        fail("WELD_CHECK", "LH_THERM_OFF must check the TC_RELAY mirror after every off")
    if "M112" not in body("gcode_macro _LH_THERM_FAULT"):
        fail("WELD_CHECK", "a welded THERM contactor must stop Klipper (M112: heartbeat to its shutdown value -> WDOG drops KM5)")
    sp = body("gcode_macro SET_PIN")
    if not ("therm_enable" in sp and "wd_heartbeat" in sp and "action_raise_error" in sp):
        fail("WATCHDOG_CHAIN", "SET_PIN from the host must be refused for therm_enable and wd_heartbeat")
    out["watchdog"] = w

    # ---- 4. lid heater SSR on the switched THERM rail, fused, controlled by HE0
    h = {}
    h["fuse_from_tc_relay"] = any(r["from_device"] == "TC_RELAY" and "2" in r["from_pins"] and r["to_device"] == "FT" and "F31" in r["to_connector"] for r in rows)
    h["ssr_from_f31"] = any(r["from_device"] == "FT" and "F31" in r["from_connector"] and r["to_device"] == "LID_SSR" for r in rows)
    h["heater_from_ssr"] = any(r["from_device"] == "LID_SSR" and r["to_device"] == "TC_LID_HEATER" for r in rows)
    h["no_unswitched_heater_feed"] = all(r["from_device"] in ("LID_SSR", "TC_LID_HEATER") for r in by("TC_LID_HEATER") if r["signal"].startswith("HEATER THERM lid") and "return" not in r["signal"])
    h["ssr_control_he0"] = any(r["from_device"] == "MAIN_CTRL" and r["to_device"] == "LID_SSR" and "PA0" in r["fw_pins"] for r in rows)
    hg = "heater_generic lid_heater"
    h["klipper_heater"] = cp.has_section(hg) and pin_name(cp.get(hg, "heater_pin")) == "PA0" and cp.getfloat(hg, "max_temp") <= 115 and cp.has_section("verify_heater lid_heater")
    i_heater = sysd["overvoltage_motion"]["supply_trim_max_v"] / rt["lid_heater"]["r_ohm"]
    h["heater_current_a"] = round(i_heater, 2)
    h["f31_ok"] = sysd["inline_fuses"]["F31"]["fuse_a"] >= i_heater and rt["LID_SSR"]["i_max"] >= sysd["inline_fuses"]["F31"]["fuse_a"]
    h["ssr_voltage_ok"] = rt["LID_SSR"]["v_max"] >= sysd["overvoltage_motion"]["supply_trim_max_v"]
    out["lid_heater"] = h
    for k_, v_ in h.items():
        if v_ is False:
            fail("LID_HEATER_PATH", f"lid heater: {k_}")
    if "THERM" not in sysd["rails"] or "HOST_WATCHDOG" not in sysd["rails"]["THERM"]["switched_by"]:
        fail("WATCHDOG_CHAIN", "THERM must list HOST_WATCHDOG in switched_by")

    # ---- 5. controller ratings against loads
    r5 = {}
    tc, tcold = rt["TEC_TC"], rt["TEC_COLD"]
    vin = (sysd["overvoltage_motion"]["supply_max_v"], sysd["overvoltage_motion"]["supply_trim_max_v"])
    r5["tec_tc_current"] = (tc["i_limit_set"], tc["i_out_max"], rt["peltier_tc"]["i_need"])
    if not rt["peltier_tc"]["i_need"] <= tc["i_limit_set"] <= tc["i_out_max"]:
        fail("CONTROLLER_RATING", f"TEC_TC limit {tc['i_limit_set']} A vs need {rt['peltier_tc']['i_need']} A / rating {tc['i_out_max']} A")
    if not (tc["v_in"][0] <= vin[0] and vin[1] <= tc["v_in"][1] and tcold["v_in"][0] <= vin[0] and vin[1] <= tcold["v_in"][1]):
        fail("CONTROLLER_RATING", f"TEC controller input range does not cover the 24 V rail {vin}")
    if not tcold["i_limit_set"] <= min(tcold["i_out_max"], rt["peltier_cold"]["i_max"]) or tcold["v_limit_set"] > rt["peltier_cold"]["v_max"]:
        fail("CONTROLLER_RATING", f"TEC_COLD {tcold['i_limit_set']} A / {tcold['v_limit_set']} V vs TEC1-12706 {rt['peltier_cold']['i_max']} A / {rt['peltier_cold']['v_max']} V")
    if tcold["channels"] < 2:
        fail("CONTROLLER_RATING", "TEC_COLD needs one channel per cold block (C3, F3)")
    i_aux = sum(l["a"] for l in sysd["loads"] if l["rail"] == "THERM_AUX")
    p_out = 2 * tcold["i_limit_set"] * tcold["v_limit_set"]
    r5["therm_aux_input_a"] = round(p_out / 0.9 / 24.0, 2)
    if r5["therm_aux_input_a"] > i_aux + 0.05:
        fail("CONTROLLER_RATING", f"THERM_AUX load {i_aux} A below the TEC_COLD input {r5['therm_aux_input_a']} A")
    if rt["WDOG"]["contact_a"] < rt["coil_chain_a"]:
        fail("CONTROLLER_RATING", f"WDOG contact {rt['WDOG']['contact_a']} A < coil chain {rt['coil_chain_a']} A")
    if rt["lid_drivers"]["v_max"] < sysd["overvoltage_motion"]["parts_on_bus"]["DM542_CLASS"]["limit_v"] - 1e-9:
        fail("CONTROLLER_RATING", "lid driver rating and overvoltage_motion parts list disagree")
    out["ratings"] = r5

    # ---- 6. Octopus pin budget: every main-MCU pin in printer.cfg is listed, and nothing listed is unused
    used_cfg = set()
    for sec in cp.sections():
        for k_, v_ in cp.items(sec):
            if k_.endswith("pin") and ":" not in v_.split()[0]:
                used_cfg.add(pin_name(v_))
    listed = {k for k in eob["pin_budget"]["used"] if k != "CAN"}
    out["pin_budget"] = {"used": len(used_cfg), "listed": len(listed), "unlisted": sorted(used_cfg - listed), "stale": sorted(listed - used_cfg)}
    if used_cfg != listed:
        fail("PIN_BUDGET", f"printer.cfg main-MCU pins vs system.yaml pin_budget: unlisted {sorted(used_cfg - listed)}, stale {sorted(listed - used_cfg)}")
    therm_in = {"PF3", "PF4", "PF5", "PF6", "PF7"}
    for sec in cp.sections():
        if cp.has_option(sec, "endstop_pin") and pin_name(cp.get(sec, "endstop_pin")) in therm_in:
            fail("PIN_BUDGET", f"[{sec}] uses an RC-filtered thermistor input as an endstop")

    # ---- 7. head supply and CAN without HIB-1
    c = {}
    c["lever_splice"] = any(r["to_device"] == "CAN_SPLICE" and "F22" in r["protected_by"] for r in rows)
    stubs = [r for r in rows if r["from_device"] == "CAN_SPLICE" and r["to_device"] in ("MMB_A", "MMB_B")]
    c["stubs_mm"] = [int(r["length_mm_estimate"]) for r in stubs]
    if len(stubs) != 2 or max(c["stubs_mm"]) > 300:
        fail("CAN_STUB", f"two XT30(2+2) leads <= 0.3 m from the splice expected, got {c['stubs_mm']}")
    for r in rows:
        if r["protected_by"].startswith("F22") and TYPES[r["cable_type"]]["ampacity_a"] < sysd["branches"]["F22"]["fuse_a"]:
            fail("FUSE_DOES_NOT_PROTECT_CABLE", f"{r['cable_id']}: {r['cable_type']} below F22")
    bus = next(b for b in sysd["buses"] if b["name"] == "can_klipper")
    if "MMB_B" not in bus["note"] or "120 ohm" not in bus["note"]:
        fail("CAN_TERMINATION", "can_klipper must state the termination (Octopus + MMB_B jumper)")
    out["head_supply"] = c

    # ---- 8. wiring rules: wire number, colour, keyed connectors per device
    # board headers and terminal strips by design; thermal modules: chain-in and chain-out are interchangeable in a series loop
    allowed_same = {"MAIN_CTRL", "IO_CTRL", "MMB_A", "MMB_B", "XT", "FT", "FT2", "XM", "CAN_SPLICE", "TEC_TC", "TEC_COLD", "TEC_TUBE", "TEC_WARM", "XDA4",
                    "TM_B2", "TM_G4", "TM_A3", "TM_A4"}   # family thermal modules: chain in / out interchangeable in the series loop
    keyed = {}
    for r in rows:
        if r.get("wire_no") != r["cable_id"] or not r.get("colour"):
            fail("WIRING_RULES", f"{r['cable_id']}: wire number / colour missing")
        for side, idx in (("from", 0), ("to", 1)):
            e = r["connector_part"].split("|")[idx].strip()
            if re.match(r"(pre-made )?(JST|Amass|Molex)", e):
                keyed.setdefault(r[f"{side}_device"], []).append((e, r["cable_id"]))
    clashes = {}
    for d, ends in keyed.items():
        seen = {}
        for e, cid in ends:
            seen.setdefault(e, []).append(cid)
        dup = {e: cids for e, cids in seen.items() if len(cids) > 1}
        if dup and d not in allowed_same:
            clashes[d] = dup
    out["keying"] = {"devices_with_identical_plugs_by_design": sorted(set(keyed) & allowed_same), "clashes": clashes}
    for d, dup in clashes.items():
        fail("WIRING_KEYING", f"{d}: identical pluggable connectors {dup} could be swapped")
    import json as _json
    jc_path = HERE / "out" / "joint_counts.json"
    if jc_path.exists():
        jc = _json.loads(jc_path.read_text())
        before = sysd["wiring_rules"]["joint_count_before"]["tool-made joints, total"]
        out["joints"] = {"before": before, "after": jc["tool-made joints, total"], "after_detail": jc}
        if jc["tool-made joints, total"] >= before:
            fail("JOINT_COUNT", f"tool-made joints {jc['tool-made joints, total']} not below the pre-ADR-003 {before}")
        if any(k.startswith("tool: small-contact crimp") or k.startswith("tool: soldered XT") for k in jc):
            fail("JOINT_COUNT", "hand-crimped small contacts or soldered XT plugs remain (buy pre-made leads)")
    else:
        fail("JOINT_COUNT", "out/joint_counts.json missing: run build_harness.py")
    rig_driver_check(fail)
    return out


def rig_driver_check(fail):
    """the bench rig runs the machine's head driver (EZ5160 Pro) so the rig tests it (review section 4)"""
    t = (HERE.parents[1] / "firmware/lh1/bench_rig.cfg").read_text()
    if re.search(r"(?m)^\[tmc2209 ", t) or len(re.findall(r"(?m)^\[tmc5160 ", t)) != 5:
        fail("RIG_DRIVER_CLASS", "bench_rig.cfg must use five tmc5160 (EZ5160 Pro) sections like the machine (Z, plunger, X module, rev L syringe drive, ADR-014 rig dock lift)")
