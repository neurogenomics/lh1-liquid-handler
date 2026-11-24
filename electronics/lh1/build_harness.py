"""Build the LH-1 wiring table and the electrical order list.

Run: .venv/bin/python electronics/lh1/build_harness.py
  -> electronics/lh1/wiring.csv           one row per cable (conductor group)
  -> electronics/lh1/electrical_bom.csv   every bought electrical item

Length estimate (length_mm_estimate): device positions come from system.yaml `devices.pos`, which were read off
the CAD reports (frame_gantry: PSU plates, bay, chains, motors, end stops; head: tool offsets, board tray, chain
anchor; deck_modules: nests and module bodies; thermocycler: under-deck parts).  A cable's path is the list of
waypoints in its route (bay exit, chain fixed end, chain moving end with the gantry at mid travel ...); each
straight leg is the Manhattan distance (cables follow frame members), a drag chain contributes its own length
from the CAD bought-parts list, the sum is multiplied by 1.10 for routing slack and 200 mm is added for the two
terminations.  Rounded up to 50 mm.  Good to perhaps +/-25 %: cut long, trim on the machine.

The machinery (route length, cut length, colour, pre-made naming, joint / connector / cable counts, table writing) is
the harnessgen package (packages/harnessgen); this file holds the LH-1 data: waypoints, cable rows, the static order
list.  uv pip install -e packages/harnessgen
"""
import json
import math
import re
from pathlib import Path

import yaml
from harnessgen import cable_totals, colour_of as hg_colour_of, connector_counts, cut_length, joint_counts, premade as hg_premade
from harnessgen import route_length, write_rows

HERE = Path(__file__).resolve().parent
SYS = yaml.safe_load((HERE / "system.yaml").read_text())
CT = yaml.safe_load((HERE / "cable_types.yaml").read_text())
TYPES, CONN = CT["cable_types"], CT["connectors"]
FAMILY = SYS["family"]
BUILD = FAMILY["build_size"]                 # the size whose wiring.csv / electrical_bom.csv are written (S)
SIZES = ("S", "M", "L")
EXTRA_ROWS = {"S": 0, "M": 1, "L": 2}         # rows inserted in front of row G (110 mm each, ADR-011)
ROW_DY = 110.0


def fam(size):
    return json.loads((HERE.parents[1] / f"cad/lh1/machine_family/out/{size}/family_{size}.json").read_text())


def fam_report(size):
    return json.loads((HERE.parents[1] / f"cad/lh1/machine_family/out/{size}/report.json").read_text())


SIZE = BUILD
POS, WP, CHAIN_LEN = {}, {}, {}


def set_size(size):
    """device positions and waypoints of one size.  system.yaml devices.pos are the STARTER (S) positions from the family
    CAD; devices marked back: true (row G, dock, back panel, Y motors) move 110 mm per inserted row; head devices move
    with the gantry's mid-travel position; chain lengths and the Y-chain fixed end come from the family report."""
    global SIZE
    SIZE = size
    f, r = fam(size), fam_report(size)
    dyb = EXTRA_ROWS[size] * ROW_DY
    yt = f["travel"]["Y"]["travel_yf"]
    ymid = (yt[0] + yt[1]) / 2
    xt = f["travel"]["X"]["travel_xc"]
    xmid = (xt[0] + xt[1]) / 2
    s_yt = fam("S")["travel"]["Y"]["travel_yf"]
    dyh = ymid - (s_yt[0] + s_yt[1]) / 2
    POS.clear()
    for k, v in SYS["devices"].items():
        x, y, z = v["pos"]
        if v.get("back"):
            y += dyb
        if v["group"] == "head":
            y += dyh
        POS[k] = (float(x), float(y), float(z))
    yc = r["y_chain"]
    gy = f["frame"]["row_y"]["G"]
    WP.clear()
    WP.update({   # family CAD (cad/lh1/machine_family scene.py services, cr_answers CR-L1/L4/L8), gantry at mid travel
        "BAY_TOP": (495.0, 58.0, -190.0),          # gland G2 (M25) in the compartment's right wall at y 58, z -190 (CAD services_interface glands): Y-chain duct
        "YCH_FIX": (668.0, yc["fixed_y"], 330.0),  # Y chain fixed end in the outboard tray (vertical duct YCD-F below it)
        "YCH_MOV": (668.0, ymid - 47.0, 445.0),    # Y chain moving end under the right Y carriage, gantry at mid travel
        "BEAM_R": (630.0, ymid + 25.0, 600.0),     # up the right Y carriage to the beam top
        "XCH_FIX": (138.0, ymid + 25.0, 650.0),    # X chain fixed end at the beam middle (XCT-F tray on the beam top)
        "XCH_MOV": (xmid + 70.0, ymid + 25.0, 650.0),   # X chain moving end on the carriage plate (XCB-F)
        "TCC_FIX": (-245.0, -125.0, 30.0),         # TC-1 lid chain fixed end (TC-1 at F1, its strip behind)
        "TCC_MOV": (-230.0, -250.0, 100.0),        # TC-1 lid chain moving end (lid closed)
        "GLAND_C": (-5.0, 55.0, -160.0),           # floor gland G1 above the compartment into the under-deck rail (CAD services_interface glands G1 x -25..15, y 40..70)
        "BIN_ROUTE": (-66.0, -41.0, -133.0),       # CAD TWB-F_beam_lead_route: up the chute's +X wall from the bin-full beam
        "RAIL_MID": (-5.0, -110.0, -127.0),        # under-deck 3-way duct UCR-F (x -25..15) at rows A/B
        "FRONT_RIGHT": (600.0, -400.0, -150.0),    # along the right side above the floor to the front right upright
        "BACK_BASE": (360.0, 190.0, -280.0),       # inside the compartment (CBP2): rev L route name kept
        "CPL": (0.0, gy - 40.0 + dyb * 0, -90.0),  # connector plane CPL-F under the front edge of row G (back section)
        "DOCK_TRUNK": (0.0, f["frame"]["dock_axis_y"] - 40.0, 0.0),   # along the dock base
        "LEFT_LOW": (-560.0, gy + 120.0, -100.0),  # to the extraction fan on the left panel (back section)
    })
    WP["CPL"] = (0.0, gy - 40.0, -90.0)
    CHAIN_LEN.update({"y_chain": float(yc["chain_length_mm"]), "x_chain": 670.0, "head_loop": 450.0, "tc_chain": 250.0})
    for t_, lx in (("HL_A", -148.0), ("HL_B", -38.0), ("HL_G", 52.0)):    # CR-L2 loop slots (carriage-local x), carriage at mid travel
        WP[f"{t_}_FIX"] = (xmid + lx, ymid - 30.0, 680.0)
        WP[f"{t_}_MOV"] = (xmid + lx, ymid - 80.0, 610.0)


set_size(BUILD)
VIA_EXPAND = {"UNDER_DECK": ("GLAND_C", "RAIL_MID"), "YCH_MOV": ("YCH_MOV", "BEAM_R")}

COLUMNS = ["cable_id", "from_device", "from_connector", "from_pins", "to_device", "to_connector", "to_pins", "signal",
           "conductors", "gauge_mm2", "shielded", "max_current_a", "length_mm_estimate", "route", "connector_part", "notes",
           "cable_type", "protected_by", "fw_pins", "wire_no", "colour"]
PREMADE = ("JST-XH", "JST-PH", "JST VH", "Amass XT30(2+2)", "Amass XT60", "Molex KK fan", "Molex Mini-Fit Jr", "Molex Micro-Fit 3.0")
COLOURS = SYS["wiring_rules"]["colours"]


def colour_of(sig):
    """one colour per rail (system.yaml wiring_rules): the longest matching signal prefix wins"""
    return hg_colour_of(sig, COLOURS)


def premade(end):
    """ADR-003 #3: every small-contact or soldered power plug is bought as a pre-made lead"""
    return hg_premade(end, PREMADE)
ROWS = []


def path_length(a, b, route, via):
    """device -> waypoints -> device; each drag chain / rolling loop in `route` contributes its own length.  A cable
    between two devices in the control compartment ignores its rev L route waypoints (one compartment now)."""
    devs = SYS["devices"]
    if devs[a]["group"] == "control_bay" and devs[b]["group"] == "control_bay":
        via = [v for v in via if v not in ("UNDER_DECK", "BACK_BASE")]
    vv = []
    for v in via:
        vv += list(VIA_EXPAND.get(v, (v,)))
    pts = [POS[a]] + [WP[v] for v in vv] + [POS[b]]
    legs = []
    for c in ([] if route == "fixed" else route.split("+")):
        kind = c.split(":")[0]
        key = c.split(":")[1] if kind == "head_loop" else {"y_chain": "YCH", "x_chain": "XCH", "tc_chain": "TCC"}[kind]
        legs.append((WP[f"{key}_FIX"], WP[f"{key}_MOV"], CHAIN_LEN[kind]))
    return route_length(pts, legs)


def in_size(dev):
    return SIZE in SYS["devices"][dev].get("sizes", SIZES)


PLANE = {}       # cable_id -> True when the cable crosses the connector plane CPL-F (back section: extension lead per inserted row)


def C(cid, a, ac, ap, b, bc, bp, signal, n, ctype, imax, conn, notes="", route="fixed", via=(), shielded=None,
      prot="", fw="", length=None, sizes=None):
    """one cable row.  sizes: the sizes it is fitted in (default every size in which both devices exist)."""
    if sizes is not None and SIZE not in sizes:
        return
    if not (in_size(a) and in_size(b)):
        return
    t = TYPES[ctype]
    if length is None:
        raw = path_length(a, b, route, list(via))
        length = 0 if ctype == "PLUG_IN" else cut_length(raw, slack=1.10, terminations_mm=200.0, round_to_mm=50)
    if "CPL" in via:
        PLANE[cid] = True
        notes = (notes + "; " if notes else "") + "keyed break at the connector plane CPL-F (back section): an upgrade inserts a 110 mm keyed extension lead per row"
    sh = shielded if shielded is not None else ("_SH" in ctype or ctype.startswith("USB"))
    conn = " | ".join(premade(e) for e in conn.split("|"))
    ROWS.append({"cable_id": cid, "from_device": a, "from_connector": ac, "from_pins": ap, "to_device": b, "to_connector": bc,
                 "to_pins": bp, "signal": signal, "conductors": n, "gauge_mm2": t["gauge_mm2"], "shielded": "yes" if sh else "no",
                 "max_current_a": imax, "length_mm_estimate": length, "route": route, "connector_part": conn, "notes": notes,
                 "cable_type": ctype, "protected_by": prot, "fw_pins": fw,
                 "wire_no": cid, "colour": colour_of(signal)})


FER = "ferrule to push-in terminal"
FS = "ferrule to screw terminal"
FS4 = "ferrule to screw terminal 4mm2"
FS6 = "ferrule to screw terminal 6mm2"
HS = "solder + heat-shrink"
CP = "competent person"

def define_rows():
    """every cable row of the CURRENT size (set_size); rows of devices the size lacks are skipped by C()"""
    # ================================ mains and protective earth ================================
    C("W-M-01", "MAINS_INLET", "switched/fused side", "L,N", "PSU_MAIN", "AC IN", "L,N", "MAINS 230 V L/N", 2, "MAINS_2X10", 5.8,
      "insulated receptacle 6.3 mm 2-pin | " + FS, f"{CP} only; T10A H fuses in the inlet (rev L mains budget 7.8 A); 230 V only (at 115 V the 10 A inlet is too small); cable clamp at both ends", prot="inlet T10A")
    C("W-M-02", "PSU_MAIN", "AC IN", "L,N", "PSU_TC", "AC IN", "L,N", "MAINS 230 V L/N (loop)", 2, "MAINS_2X10", 2.9,
      FS + " | " + FS, f"{CP} only; twin-wire ferrules (2 x 1.0 mm2) at PSU_MAIN L and N", prot="inlet T10A")
    C("W-M-04", "PSU_AUX", "AC IN", "L,N", "PSU_5V", "AC L/N", "L,N", "MAINS 230 V L/N (loop to host supply)", 2, "MAINS_2X10", 2.6,
      FS + " | " + FS, f"{CP} only; twin-wire ferrules at PSU_AUX", route="fixed", via=("BACK_BASE",), prot="inlet T10A")
    C("W-M-03", "PSU_TC", "AC IN", "L,N", "PSU_AUX", "AC IN", "L,N", "MAINS 230 V L/N (loop to the THERM_AUX supply)", 2, "MAINS_2X10", 4.5,
      FS + " | " + FS, f"{CP} only; twin-wire ferrules at PSU_TC; in its own duct on rail A, segregated from 24 V wiring", prot="inlet T10A")
    C("W-E-01", "MAINS_INLET", "PE tab", "PE", "PE_STUD", "M6 stud", "1", "PE main", 1, "PE_GY_15", 0.0,
      "insulated receptacle 6.3 mm 1-pin | ring lug M6", f"{CP}; PE must be the last conductor to break if the cable is pulled (longest)", prot="inlet T10A")
    C("W-E-02", "PE_STUD", "M6 stud", "1", "PSU_MAIN", "AC IN", "PE", "PE", 1, "PE_GY_15", 0.0, "ring lug M6 | " + FS, CP)
    C("W-E-03", "PE_STUD", "M6 stud", "1", "PSU_TC", "AC IN", "PE", "PE", 1, "PE_GY_15", 0.0, "ring lug M6 | " + FS, CP)
    for i, (dev, note) in enumerate([("FRAME", "bond across the frame to the left base member (measures joint continuity); an upgrade adds the joint plates: re-measure"),
                                     ("CB_PLATE", "control bay plate CBP-F"), ("CBX_BOX", "control compartment box (carries the inlet, outlets and intake fan)"),
                                     ("DECK_PLATE", "deck: one lead to row F's strip, jumpers strip to strip over the support bars (one more per inserted row)"),
                                     ("PANEL_RIGHT", "removable panel: ring lug on a welded/clinched stud; one jumper per panel sheet"), ("PANEL_LEFT", "carries the extraction fan"),
                                     ("PANEL_REAR", ""), ("PANEL_TOP", ""), ("PANEL_FLOOR", "floor sheets: the compartment's lid")], start=4):
        C(f"W-E-{i:02d}", "PE_STUD", "M6 stud", "1", dev, "bond point", "1", "PE bond", 1, "PE_GY_25", 0.0, "ring lug M6 | ring lug M4",
          f"protective bond; {note}".rstrip("; "))
    C("W-E-16", "PE_STUD", "M6 stud", "1", "PSU_5V", "PE", "PE", "PE", 1, "PE_GY_15", 0.0, "ring lug M6 | " + FS, CP)
    C("W-E-18", "PE_STUD", "M6 stud", "1", "PSU_AUX", "AC IN", "PE", "PE", 1, "PE_GY_15", 0.0, "ring lug M6 | " + FS, CP, route="fixed", via=("BACK_BASE",))
    C("W-E-19", "PE_STUD", "M6 stud", "1", "MAINS_STRIP", "PE", "PE", "PE switched strip", 1, "PE_GY_15", 0.0, "ring lug M6 | " + FS, CP, route="fixed", via=("BACK_BASE",))
    C("W-E-20", "PE_STUD", "M6 stud", "1", "CBP2", "bond point", "1", "PE bond", 1, "PE_GY_25", 0.0, "ring lug M6 | ring lug M4", "protective bond; rear electronics plate CBP2 (rev L)", route="fixed", via=("BACK_BASE",))
    C("W-E-14", "PE_STUD", "M6 stud", "1", "GANTRY", "X beam end", "1", "FE bond (gantry)", 1, "SC_HF_075", 0.0,
      "ring lug M4 | ring lug M4", "functional bond of the moving gantry (green/yellow chain core): no mains on the gantry; rails/balls are not a bond",
      route="y_chain", via=("BAY_TOP", "YCH_FIX", "YCH_MOV"))
    C("W-E-15", "XT", "0V bus", "0V", "PE_STUD", "M6 stud", "1", "0V-PE link (PELV option)", 1, "PE_GY_15", 0.0, FER + " | ring lug M6",
      "RESERVED, NOT FITTED: only if the safety assessment chooses PELV (0 V earthed at one point); as drawn the 24 V system is SELV, 0 V floating")

    # ================================ 24 V distribution (control bay) ================================
    C("W-P-01", "PSU_MAIN", "+V", "+V1,+V2", "FT", "bus F1/F2/F3/F4/F6 in", "jumper bar", "24V PSU_MAIN out", 1, "SC_40", 19.7, FS4 + " | " + FS4,
      "feeder; bridged across fuse inputs with a plug-in jumper bar", prot="PSU_MAIN current limit (about 30 A)")
    C("W-P-02", "PSU_MAIN", "-V", "-V1,-V2", "XT", "0V bus", "0V", "0V PSU_MAIN", 1, "SC_40", 19.7, FS4 + " | " + FS4, "", prot="PSU_MAIN current limit (about 30 A)")
    C("W-P-03", "PSU_TC", "+V", "+V1,+V2", "FT", "F5 in", "1", "24V PSU_TC out", 1, "SC_40", 16.3, FS4 + " | " + FS4, "", prot="PSU_TC current limit (about 30 A)")
    C("W-P-04", "PSU_TC", "-V", "-V3", "XT", "0V bus", "0V", "0V common link PSU_TC-PSU_MAIN", 1, "SC_15", 1.0, FS + " | " + FER,
      "reference link only; the THERM return goes straight to PSU_TC (W-P-36)", prot="PSU_TC limit (reference link, no load current)")
    C("W-P-05", "FT", "F1 out", "2", "XT", "LOGIC", "1", "24V LOGIC", 1, "SC_15", 3.45, FER + " | " + FER, "", prot="F1")
    C("W-P-38", "FT", "F7 out", "2", "XT", "LOG7", "1", "24V LOGIC (F7 small loads)", 1, "SC_05", 1.0, FER + " | " + FER, "F7 in fed from XT LOGIC", prot="F7")
    C("W-P-06", "FT", "F2 out", "2", "KM1", "1/L1", "1", "24V MOTION", 1, "SC_25", 10.7, FER + " | " + FS, "", prot="F2")
    C("W-P-07", "KM1", "2/T1", "2", "KM2", "1/L1", "1", "24V MOTION", 1, "SC_25", 10.7, FS + " | " + FS, "", prot="F2")
    C("W-P-08", "KM2", "2/T1", "2", "KM3", "1/L1", "1", "24V MOTION", 1, "SC_25", 10.7, FS + " | " + FS, "", prot="F2")
    C("W-P-09", "KM3", "2/T1", "2", "KM4", "1/L1", "1", "24V MOTION", 1, "SC_25", 10.7, FS + " | " + FS, "", prot="F2")
    C("W-P-10", "KM4", "2/T1", "2", "FT", "bus F21/F22/F23 in", "jumper bar", "24V MOTION (switched)", 1, "SC_25", 10.7, FS + " | " + FER, "", prot="F2")
    C("W-P-50", "PSU_AUX", "+V", "+V1,+V2", "FT2", "bus F4/F9 in", "jumper bar", "24V PSU_AUX out", 1, "SC_40", 15.6, FS4 + " | " + FS4, "third supply (rev L): THERM_AUX + MODULE", prot="PSU_AUX current limit (about 30 A)")
    C("W-P-51", "PSU_AUX", "-V", "-V3", "XT", "0V bus", "0V", "0V common link PSU_AUX-PSU_MAIN", 1, "SC_15", 1.0, FS + " | " + FER, "reference link + MODULE return; THERM_AUX returns go straight to PSU_AUX", route="fixed", via=("BACK_BASE",), prot="PSU_AUX limit (reference link)")
    C("W-P-14", "FT2", "F4 out", "2", "KM1", "5/L3", "5", "24V THERM_AUX", 1, "SC_40", 14.6, FER + " | " + FS, "", route="fixed", via=("BACK_BASE",), prot="F4")
    C("W-P-15", "KM1", "6/T3", "6", "KM2", "5/L3", "5", "24V THERM_AUX", 1, "SC_40", 14.6, FS + " | " + FS, "", prot="F4")
    C("W-P-16", "KM2", "6/T3", "6", "XT", "THERM_AUX", "1", "24V THERM_AUX (switched)", 1, "SC_40", 14.6, FS + " | " + FER, "", prot="F4")
    C("W-P-17", "FT", "F6 out", "2", "XT", "SAFE", "1", "24V SAFE (safety-circuit supply)", 1, "SC_10", 0.85, FER + " | " + FER, "", prot="F6")
    C("W-P-18", "FT", "F21 out", "2", "XT", "MOT_G", "1", "24V MOTION_G (gantry)", 1, "SC_15", 5.5, FER + " | " + FER, "", prot="F21")
    C("W-P-19", "XT", "LOGIC/0V", "1,0V", "MAIN_CTRL", "POWER (main)", "+,-", "24V LOGIC", 2, "SC_10", 0.4, FER + " | " + FS,
      "Octopus main input: logic, fans header supply, endstop supply", prot="F1")
    C("W-P-20", "XT", "MOT_G/0V", "2,0V", "MAIN_CTRL", "MOTOR_POWER", "+,-", "24V MOTION_G", 2, "SC_15", 0.5, FER + " | " + FS,
      "all eight driver-voltage jumpers set to MOTOR_POWER BEFORE first power (wrong jumper = board damage)", prot="F21")
    C("W-P-22", "PSU_5V", "+Vo/-Vo", "3,4", "HOST", "USB-C power", "VBUS,GND", "5V host", 2, "USBC_PWR", 5.0, FS + " | USB-C",
      "set converter to 5.1-5.2 V at the Pi end; Pi 5: set usb_max_current_enable=1 (no PD from this source)", prot="PSU_5V current limit")
    C("W-P-23", "PSU_5V", "+Vo/-Vo", "3,4", "HUB1", "DC in", "+,-", "5V hub 1", 2, "SC_05", 1.0, FS + " | " + FS, "bought hub with screw-terminal DC input", prot="PSU_5V current limit")
    C("W-P-24", "XT", "LOG7/0V", "1,0V", "CAB_FAN1", "lead", "+,-", "24V LOGIC (F7) compartment intake fan 1", 2, "DEVICE_LEAD", 0.1, FER + " | " + HS, "always on; 92 mm, right wall y 120 (CAD)", prot="F7")
    C("W-P-25", "XT", "LOG7/0V", "7,0V", "CAB_FAN2", "lead", "+,-", "24V LOGIC (F7) compartment intake fan 2", 2, "DEVICE_LEAD", 0.1, FER + " | " + HS, "always on; 92 mm, right wall y 250 (CAD)", prot="F7")
    C("W-P-28", "FT", "F22 out (+) / XT 0V", "2,0V", "CAN_SPLICE", "lever block V+/0V", "V+,0V", "24V MOTION_H (head)", 2, "SC_HF_15", 3.85,
      FER + " | Wago 221 lever splice", "two 1.5 mm2 chain-rated cores (rev L: 4.4 m through the longer chains) through the Y and X chains to the lever-splice block on the head tray (HIB-1 retired)",
      route="y_chain+x_chain", via=("BAY_TOP", "YCH_FIX", "YCH_MOV", "XCH_FIX", "XCH_MOV"), prot="F22")
    C("W-P-29", "XT", "MOT_G", "3", "FT", "F25/F26/F27 in (jumper bar)", "jumper bar", "24V MOTION_G to the per-motor fuses", 1, "SC_15", 5.5, FER + " | " + FER,
      "V8 (23 Nov 2026): F21 10 A protects this 1.5 mm2 feed (Iz 12.2 A); each motor lead has its own 4 A fuse", prot="F21")
    C("W-P-30", "FT", "F25 out / XT 0V", "2,0V", "X_MOTOR", "power", "V+,GND", "24V MOTION_G X motor", 2, "SC_HF_075", 2.0, FER + " | Wago 221 lever splice",
      "then static along the X beam to the left end; own fuse F25 4 A (V8: 0.75 mm2 in the chain, Iz 5.4 A < F21 10 A)", route="y_chain", via=("BAY_TOP", "YCH_FIX", "YCH_MOV"), prot="F25")
    C("W-P-31", "FT", "F26 out / XT 0V", "2,0V", "Y_MOTOR", "power", "V+,GND", "24V MOTION_G Y motor", 2, "MC2_10", 1.5, FER + " | Wago 221 lever splice",
      "under-deck rail, connector plane, up the right rear upright; own fuse F26 4 A (V8: 1.0 mm2 Iz 9.6 A < F21 10 A)", via=("UNDER_DECK", "CPL"), prot="F26")
    C("W-P-32", "FT", "F27 out / XT 0V", "2,0V", "Y1_MOTOR", "power", "V+,GND", "24V MOTION_G Y1 motor", 2, "MC2_10", 1.5, FER + " | Wago 221 lever splice",
      "under-deck rail, connector plane, along the base back member and up the left rear upright; own fuse F27 4 A", via=("UNDER_DECK", "CPL"), prot="F27")
    # ================================ safety circuit ================================
    C("W-S-01", "XT", "SAFE/0V", "1,0V", "SR1", "A1/A2", "A1,A2", "24V SAFE SR1 supply", 2, "SC_05", 0.1, FER + " | " + FER, "", prot="F6")
    C("W-S-02", "XT", "SAFE/0V", "2,0V", "SR2", "A1/A2", "A1,A2", "24V SAFE SR2 supply", 2, "SC_05", 0.1, FER + " | " + FER, "", prot="F6")
    C("W-S-03", "ESTOP", "contact blocks", "11-12(NC1),21-22(NC2),31-32(NC aux)", "XT", "XE", "1-7", "SAFETY e-stop ch1+ch2, aux", 7, "MC7_05_SH", 0.05,
      FS + " | " + FER, "cores 1-2 ch1, 3-4 ch2, 5-6 aux NC (PG11 monitor, fail-safe), 7 spare; screen to 0V at the bay end only",
      route="fixed", via=("FRONT_RIGHT",), prot="SR1 internal (S11/S21 test pulses)", fw="PG11")
    C("W-S-04", "XT", "XE", "1-4", "SR1", "S11,S12,S21,S22", "S11,S12,S21,S22", "SAFETY e-stop ch1+ch2", 4, "SC_05", 0.05, FER + " | " + FER,
      "two channels with cross-fault detection (relay's own test outputs)", prot="SR1 internal")
    C("W-S-05", "XT", "XE", "5-6", "MAIN_CTRL", "STOP_3", "PG11,GND", "SIGNAL e-stop monitor", 2, "MC2_025", 0.01, FER + " | JST-XH 3-pin",
      "monitoring only, not part of the safety function", prot="Octopus pull-up", fw="PG11")
    C("W-S-06", "SR1", "S12 (reset loop)", "S12", "KM1", "21-22 mirror NC", "21", "SAFETY feedback loop SR1", 1, "SC_05", 0.05, FER + " | " + FS,
      "feedback: KM1 NC + KM2 NC + reset button in series to S34", prot="SR1 internal")
    C("W-S-07", "KM1", "21-22 mirror NC", "22", "KM2", "21-22 mirror NC", "21", "SAFETY feedback loop SR1", 1, "SC_05", 0.05, FS + " | " + FS, "", prot="SR1 internal")
    C("W-S-08", "KM2", "21-22 mirror NC", "22", "KM5", "21-22 mirror NC", "21", "SAFETY feedback loop SR1", 1, "SC_05", 0.05, FS + " | " + FS, "", prot="SR1 internal")
    C("W-S-39", "KM5", "21-22 mirror NC", "22", "TC_RELAY", "mirror NC 21-22", "21", "SAFETY feedback loop SR1", 1, "SC_05", 0.05, FS + " | " + FS,
      "to the THERM contactor under the deck and back: a welded THERM contactor blocks SR1 reset", route="fixed", via=("UNDER_DECK",), prot="SR1 internal")
    C("W-S-40", "TC_RELAY", "mirror NC 21-22", "22", "TB_RELAY", "mirror NC 21-22", "21", "SAFETY feedback loop SR1", 1, "SC_05", 0.05, FS + " | " + FS,
      "", route="fixed", via=("UNDER_DECK",), prot="SR1 internal")
    C("W-S-42", "TB_RELAY", "mirror NC 21-22", "22", "KM6", "21-22 mirror NC", "21", "SAFETY feedback loop SR1", 1, "SC_05", 0.05, FS + " | " + FS,
      "on to the mains-strip contactor KM6 (rev L)", route="fixed", via=("UNDER_DECK",), prot="SR1 internal")
    C("W-S-41", "TC_RELAY", "A1 (coil +)", "A1", "KM5", "A1", "A1", "24V thermostat chain end to KM5 coil", 1, "SC_05", 0.15, FS + " | " + FS,
      "KM5 coil hangs on the same thermostat-chain end as TC_RELAY: over-temperature and e-stop open BOTH THERM contactors; coil suppressor on KM5",
      route="fixed", via=("UNDER_DECK",), prot="F6")
    C("W-S-43", "KM5", "A2", "A2", "XT", "0V bus", "0V", "0V coil return KM5", 1, "SC_05", 0.15, FS + " | " + FER, "", prot="F6")
    C("W-S-09", "XT", "XR", "2", "SR1", "S34", "S34", "SAFETY reset", 1, "SC_05", 0.05, FER + " | " + FER, "monitored manual reset (falling edge)", prot="SR1 internal")
    C("W-S-10", "RESET_PB", "NO + lamp", "13-14,X1-X2", "XT", "XR", "1-4", "SAFETY reset button + lamp", 4, "MC4_025", 0.05, FS + " | " + FER,
      "lamp lit from SR1 Y32 when a reset is needed", route="fixed", via=("FRONT_RIGHT",), prot="F6")
    C("W-S-11", "SR1", "Y32 (status out)", "Y32", "XT", "XR", "3", "SIGNAL SR1 status to reset lamp", 1, "SC_05", 0.05, FER + " | " + FER, "", prot="SR1 semiconductor output")
    C("W-S-12", "XT", "SAFE", "3", "SR1", "13,23,33 (contact feed)", "13,23,33", "24V SAFE contact feed", 1, "SC_05", 0.6, FER + " | " + FER, "jumpers 13-23-33 at the relay", prot="F6")
    C("W-S-13", "SR1", "14", "14", "KM1", "A1", "A1", "24V SAFE KM1 coil", 1, "SC_05", 0.15, FER + " | " + FS, "plug-in coil suppressor (varistor/diode) on KM1", prot="F6")
    C("W-S-14", "SR1", "24", "24", "KM2", "A1", "A1", "24V SAFE KM2 coil", 1, "SC_05", 0.15, FER + " | " + FS, "plug-in coil suppressor on KM2", prot="F6")
    C("W-S-15", "KM1", "A2", "A2", "XT", "0V bus", "0V", "0V coil returns KM1+KM2", 2, "SC_05", 0.15, FS + " | " + FER, "second core from KM2 A2", prot="F6")
    C("W-S-16", "DOOR_SW", "sensor lead", "ch1 pair, ch2 pair, signalling pair", "XT", "XD", "1-6", "SAFETY door ch1+ch2, signalling", 6, "DEVICE_LEAD", 0.05,
      "moulded device lead | " + FER, "sensor's own 6-core lead (or its M8/M12 cordset); signalling contact CLOSED while the door is closed, to PB7 (fail-safe)", route="fixed", via=("FRONT_RIGHT",),
      prot="SR2 internal", fw="PB7")
    C("W-S-17", "XT", "XD", "1-4", "SR2", "S11,S12,S21,S22", "S11,S12,S21,S22", "SAFETY door ch1+ch2", 4, "SC_05", 0.05, FER + " | " + FER, "", prot="SR2 internal")
    C("W-S-18", "XT", "XD", "5-6", "MAIN_CTRL", "PROBE port", "PB7,GND", "SIGNAL door monitor", 2, "MC2_025", 0.01, FER + " | JST-XH 3-pin",
      "monitoring only", prot="Octopus pull-up", fw="PB7")
    C("W-S-19", "XT", "SAFE", "4", "SR2", "13,23 (contact feed)", "13,23", "24V SAFE contact feed", 1, "SC_05", 0.4, FER + " | " + FER, "", prot="F6")
    C("W-S-20", "SR2", "14", "14", "KM3", "A1", "A1", "24V SAFE KM3 coil", 1, "SC_05", 0.15, FER + " | " + FS, "coil suppressor", prot="F6")
    C("W-S-21", "SR2", "24", "24", "KM4", "A1", "A1", "24V SAFE KM4 coil", 1, "SC_05", 0.15, FER + " | " + FS, "coil suppressor", prot="F6")
    C("W-S-22", "KM3", "A2", "A2", "XT", "0V bus", "0V", "0V coil returns KM3+KM4", 2, "SC_05", 0.15, FS + " | " + FER, "second core from KM4 A2", prot="F6")
    C("W-S-23", "SR2", "S12 (reset loop)", "S12", "KM3", "21-22 mirror NC", "21", "SAFETY feedback loop SR2", 1, "SC_05", 0.05, FER + " | " + FS, "", prot="SR2 internal")
    C("W-S-24", "KM3", "21-22 mirror NC", "22", "KM4", "21-22 mirror NC", "21", "SAFETY feedback loop SR2", 1, "SC_05", 0.05, FS + " | " + FS, "", prot="SR2 internal")
    C("W-S-25", "KM4", "21-22 mirror NC", "22", "SR2", "S34", "S34", "SAFETY feedback loop SR2 (automatic reset)", 1, "SC_05", 0.05, FS + " | " + FER,
      "automatic reset through the feedback loop; assessor to confirm acceptable for the door", prot="SR2 internal")
    C("W-S-26", "XT", "SAFE", "5", "KM1", "13-14 aux NO", "13", "24V SAFE to aux chain", 1, "SC_05", 0.2, FER + " | " + FS, "", prot="F6")
    C("W-S-27", "KM1", "13-14 aux NO", "14", "KM2", "13-14 aux NO", "13", "24V SAFE aux chain", 1, "SC_05", 0.2, FS + " | " + FS, "", prot="F6")
    C("W-S-28", "KM2", "13-14 aux NO", "14", "XT", "SAFE_E", "1", "24V SAFE_E (e-stop OK)", 1, "SC_05", 0.2, FS + " | " + FER, "", prot="F6")
    # ================================ host, USB and CAN ================================
    C("W-C-01", "HOST", "USB-A 1", "USB", "MAIN_CTRL", "USB-C", "USB", "USB Klipper + CAN bridge", 4, "USB2_AC", 0.5, "USB-A 4-pin | USB-C", "")
    C("W-C-03", "HOST", "USB-A 3", "USB", "HUB1", "upstream", "USB", "USB hub 1 uplink (bay, thermal, cameras)", 4, "USB2_AB", 0.1, "USB-A 4-pin | USB-B 4-pin", "")
    C("W-C-06", "HUB1", "port 6", "USB", "VC3", "USB", "USB", "USB deck camera VC-3", 4, "USB2_AB", 0.3, "USB-A 4-pin | USB-B 4-pin",
      "up the right front upright to the camera", route="fixed", via=("FRONT_RIGHT",))
    C("W-C-07", "MAIN_CTRL", "CAN", "CANH,CANL", "CAN_SPLICE", "lever block CAN-H/CAN-L", "CANH,CANL", "CAN Klipper 1 Mbit/s", 2, "SC_HF_025", 0.01,
      "pre-made Octopus CAN lead | Wago 221 lever splice", "twisted pair through the chains; 120 ohm ON at the Octopus end; MMB_B 120R jumper ON (far end), MMB_A OFF",
      route="y_chain+x_chain", via=("BAY_TOP", "YCH_FIX", "YCH_MOV", "XCH_FIX", "XCH_MOV"), prot="signal")
    C("W-C-09", "CAN_SPLICE", "lever block (V+, 0V, CAN-H, CAN-L)", "V+,0V,CANH,CANL", "MMB_A", "XT30 (power+CAN)", "V+,GND,CANH,CANL", "24V MOTION_H + CAN Klipper", 4, "XT30_LEAD", 2.9,
      "Wago 221 lever splice | Amass XT30(2+2) 4-pin", "bought XT30(2+2) toolhead lead 0.15 m (CAN stub 0.15 m < 0.3 m at 1 Mbit/s); keyed plug", length=150, prot="F22")
    C("W-C-10", "CAN_SPLICE", "lever block (V+, 0V, CAN-H, CAN-L)", "V+,0V,CANH,CANL", "MMB_B", "XT30 (power+CAN)", "V+,GND,CANH,CANL", "24V MOTION_H + CAN Klipper", 4, "XT30_LEAD", 1.6,
      "Wago 221 lever splice | Amass XT30(2+2) 4-pin", "bought XT30(2+2) toolhead lead 0.15 m; MMB_B terminates the bus", length=150, prot="F22")
    C("W-C-12", "MAIN_CTRL", "Driver0 socket", "STEP,DIR,EN,GND", "XDA0", "pins", "STEP,DIR,EN,GND", "STEP/DIR X", 4, "PLUG_IN", 0.01,
      "board plug-in | board plug-in", "breakout plugged into socket 0; no TMC2209 fitted there", fw="PF13,PF12,PF14")
    C("W-C-13", "MAIN_CTRL", "Driver1 socket", "STEP,DIR,EN,GND", "XDA1", "pins", "STEP,DIR,EN,GND", "STEP/DIR Y", 4, "PLUG_IN", 0.01,
      "board plug-in | board plug-in", "", fw="PG0,PG1,PF15")
    C("W-C-14", "MAIN_CTRL", "Driver3 socket", "STEP,DIR,EN,GND", "XDA3", "pins", "STEP,DIR,EN,GND", "STEP/DIR Y1", 4, "PLUG_IN", 0.01,
      "board plug-in | board plug-in", "", fw="PG4,PC1,PA2")

    # ================================ gantry and frame motion ================================
    C("W-G-01", "XDA0", "terminals", "STEP,DIR,EN,COM", "X_MOTOR", "signal", "PUL,DIR,ENA,COM", "STEP/DIR X", 4, "SC_HF_025", 0.02,
      FS + " | JST-XH 4-pin", "3.3 V logic: confirm the closed-loop driver accepts it or use a 5 V breakout; alarm output on W-G-03 (PG14)",
      route="y_chain", via=("BAY_TOP", "YCH_FIX", "YCH_MOV"), prot="signal", fw="PF13,PF12,PF14")
    C("W-G-02", "MAIN_CTRL", "STOP_0", "PG6,GND", "X_ENDSTOP", "C,NC", "C,NC", "ENDSTOP X", 2, "SC_HF_025", 0.01, "JST-XH 3-pin | " + HS,
      "normally open to GND; along the X beam to the right end", route="y_chain", via=("BAY_TOP", "YCH_FIX", "YCH_MOV"), prot="signal", fw="PG6")
    C("W-F-01", "XDA1", "terminals", "STEP,DIR,EN,COM", "Y_MOTOR", "signal", "PUL,DIR,ENA,COM", "STEP/DIR Y", 6, "MC6_025_SH", 0.02,
      FS + " | JST-XH 6-pin", "pins 5-6 spare (driver alarm output later); confirm the motor's own connector", via=("UNDER_DECK", "CPL"), prot="signal", fw="PG0,PG1,PF15")
    C("W-F-02", "XDA3", "terminals", "STEP,DIR,EN,COM", "Y1_MOTOR", "signal", "PUL,DIR,ENA,COM", "STEP/DIR Y1", 6, "MC6_025_SH", 0.02,
      FS + " | JST-XH 6-pin", "pins 5-6 spare (alarm later)", via=("UNDER_DECK", "CPL"), prot="signal", fw="PG4,PC1,PA2")
    C("W-F-03", "MAIN_CTRL", "STOP_1", "PG9,GND", "Y_ENDSTOP", "C,NC", "C,NC", "ENDSTOP Y", 2, "MC2_025", 0.01, "JST-XH 3-pin | " + HS,
      "along the base front member, up the front left upright", via=("FRONT_RIGHT",), prot="signal", fw="PG9")

    C("W-F-04", "MAIN_CTRL", "STOP_4", "PG12,GND", "Y_ENDSTOP_R", "C,NC", "C,NC", "ENDSTOP Y right", 2, "MC2_025", 0.01, "JST-XH 3-pin | " + HS,
      "along the base front member, up the front right upright; second Y switch for independent homing / squaring", via=("FRONT_RIGHT",), prot="signal", fw="PG12")

    C("W-G-03", "X_MOTOR", "ALM", "ALM", "MAIN_CTRL", "STOP_6", "PG14", "SIGNAL servo alarm X", 1, "SC_HF_025", 0.01, "JST-XH 2-pin | JST-XH 3-pin",
      "ALM set to CONDUCT while healthy (fail-safe: alarm or broken wire reads high -> M112); GND via the COM core of W-G-01", route="y_chain", via=("YCH_MOV", "YCH_FIX", "BAY_TOP"), prot="signal", fw="PG14")
    C("W-F-05", "Y_MOTOR", "ALM", "ALM,COM", "MAIN_CTRL", "STOP_7", "PG15,GND", "SIGNAL servo alarm Y", 2, "MC2_025", 0.01, "JST-XH 2-pin | JST-XH 3-pin",
      "ALM conducting while healthy (fail-safe)", via=("UNDER_DECK", "CPL"), prot="signal", fw="PG15")
    C("W-F-06", "Y1_MOTOR", "ALM", "ALM,COM", "MAIN_CTRL", "PWR-DET", "PC0,GND", "SIGNAL servo alarm Y1", 2, "MC2_025", 0.01, "JST-XH 2-pin | JST-XH 3-pin",
      "PWR-DET header used as an input; ALM conducting while healthy (fail-safe)", via=("UNDER_DECK", "CPL"), prot="signal", fw="PC0")
    C("W-T-18", "MAIN_CTRL", "STOP_5", "PG13,GND", "TC_SW_PARKED", "C,NO", "C,NO", "ENDSTOP TC lid parked (interlock)", 2, "MC2_025", 0.01, "JST-XH 3-pin | " + HS,
      "firmware interlock: tools may go below z 129 over the TC-1 zone (F1) only while this is made", route="fixed", via=("UNDER_DECK",), prot="signal", fw="PG13")
    C("W-D-08", "TIP_GATE_RX", "lead", "+24,0V,OUT", "XT", "XG", "1-3", "24V LOGIC (F7) tip gate receiver + OUT", 3, "MC3_025", 0.03, "moulded device lead | " + FER,
      "rev K separated through-beam pair (fork retired): receiver XG.1 from LOG7, XG.2 0V; NPN LIGHT-ON (fail-safe: dead, unpowered or unplugged reads blocked); position provisional", route="fixed", via=("UNDER_DECK",), prot="F7", fw="PF9")
    C("W-D-09", "XT", "XG", "3", "XDA4", "terminal PF9", "PF9", "SIGNAL tip gate", 1, "SC_05", 0.01, FER + " | " + FS, "NPN light-on only (3.3 V input)", prot="signal", fw="PF9")
    C("W-D-10", "MAIN_CTRL", "FAN5", "+,-", "TG_SW", "SSR input", "A1+,A2-", "SIGNAL tip-gate supply switch (W3 supply test)", 2, "MC2_025", 0.02,
      "JST-XH 2-pin | " + FER, "Octopus FAN5 header (voltage jumper on VIN = 24 V; '-' is the low-side MOSFET on PD15) drives the SSR terminal input; FAN5 off -> gate unpowered -> reads 'blocked'", prot="signal", fw="PD15")
    C("W-D-11", "XT", "LOG7", "2", "TG_SW", "SSR output +", "13+", "24V LOGIC (F7) tip gate supply, switched", 1, "SC_05", 0.03, FER + " | " + FER, "SSR output side supply from F7", prot="F7")
    C("W-D-12", "TG_SW", "SSR output", "14", "XT", "XG", "4", "24V LOGIC (F7) tip gate EMITTER supply, switched", 1, "SC_05", 0.03, FER + " | " + FER,
      "the EMITTER's +24 V comes through TG_SW (W3: emitter off -> receiver must read blocked)", prot="F7")
    C("W-D-13", "XT", "XG", "4,2", "TIP_GATE_TX", "lead", "+24,0V", "24V LOGIC (F7) tip gate emitter (switched)", 2, "MC2_025", 0.03, FER + " | moulded device lead", "", route="fixed", via=("UNDER_DECK",), prot="F7")
    C("W-C-15", "MAIN_CTRL", "Driver4 socket", "STEP(PF9),DIR,EN,GND", "XDA4", "pins", "PF9,PF10,PG2,GND", "SIGNAL breakout (inputs)", 4, "PLUG_IN", 0.01,
      "board plug-in | board plug-in", "passive breakout: socket 4 pins as inputs", fw="PF9")
    ZCH = ("UNDER_DECK", "BAY_TOP", "YCH_FIX", "YCH_MOV", "XCH_FIX", "XCH_MOV")
    C("W-Z-01", "XT", "LOG7", "3", "Z_UP_A", "C", "C", "SAFETY lid permissive chain (all tools up) feed", 1, "SC_HF_025", 0.08, FER + " | Wago 221 lever splice",
      "+24 V from F7 through the chains to the first Z_UP (carriage A); chain current about 79 mA (K_CH + beam receiver + K_TU coil)", route="y_chain+x_chain",
      via=("BAY_TOP", "YCH_FIX", "YCH_MOV", "XCH_FIX", "XCH_MOV"), prot="F7")
    C("W-Z-02", "Z_UP_A", "NO", "NO", "Z_UP_B", "C", "C", "SAFETY lid permissive chain", 1, "SC_HF_025", 0.08, "Wago 221 lever splice | Wago 221 lever splice",
      "THREE roller-lever NO contacts in series (carriage A, carriage B, gripper axis), closed only from the normal top up to the eject top; switches with factory leads (sealed type, Y7)", prot="F7")
    C("W-Z-03", "Z_UP_B", "NO", "NO", "Z_UP_GRIP", "C", "C", "SAFETY lid permissive chain", 1, "SC_HF_025", 0.08, "Wago 221 lever splice | Wago 221 lever splice", "", prot="F7")
    C("W-Z-04", "Z_UP_GRIP", "NO", "NO", "XT", "ZC (chain end)", "1", "SAFETY lid permissive chain end", 1, "SC_HF_025", 0.08, "Wago 221 lever splice | " + FER,
      "chain end ZC back in the bay", route="y_chain+x_chain", via=tuple(reversed(ZCH[1:])), prot="F7")
    C("W-Z-06", "XT", "ZC/0V", "2,0V", "K_CH", "A1/A2", "A1,A2", "SAFETY lid permissive chain: K_CH coil", 2, "MC2_025", 0.011, FER + " | " + FER,
      "chain-status relay coil (10.4 mA)", route="fixed", via=("UNDER_DECK",), prot="F7")
    C("W-Z-07", "K_CH", "11-14 (C, NO)", "11,14", "MAIN_CTRL", "STOP_2", "PG10,GND", "SIGNAL all-tools-up chain status to Octopus PG10", 2, "MC2_025", 0.001,
      FER + " | JST-XH 3-pin", "volt-free NO contact: closed while the chain is energised; PG10 ^! (broken wire = 'not all up')", route="fixed", via=("UNDER_DECK",), prot="signal", fw="PG10")
    C("W-Z-08", "XT", "ZC/0V", "3,0V", "TC_BEAM_RX", "lead brown/blue", "+V,0V", "SAFETY lid permissive chain: beam receiver supply", 2, "MC2_025", 0.07, FER + " | moulded device lead",
      "the receiver is powered only while every tool is up", route="fixed", via=("UNDER_DECK",), prot="F7")
    C("W-Z-09", "TC_BEAM_RX", "lead black (PNP OUT)", "OUT", "K_TU", "A1 (coil +)", "A1", "SAFETY gate: beam receiver output drives K_TU", 1, "SC_05", 0.04, "moulded device lead | " + FER,
      "PNP light-ON output (<= 50 mA) -> K_TU coil 38.2 mA", route="fixed", via=("UNDER_DECK",), prot="F7")
    C("W-Z-10", "K_TU", "A2 (coil -)", "A2", "XT", "0V bus", "0V", "0V gate relay coil return", 1, "SC_05", 0.04, FER + " | " + FER, "", route="fixed", via=("UNDER_DECK",), prot="F7")
    C("W-Z-11", "K_TU", "21-22 (NC, forcibly guided)", "21,22", "MAIN_CTRL", "BLTouch port", "PB6,GND", "SIGNAL lid gate readback (K_TU NC)", 2, "MC2_025", 0.001,
      FER + " | JST-XH 5-pin", "NC closed = gate OFF; read before and after every lid move (self-test)", route="fixed", via=("UNDER_DECK",), prot="signal", fw="PB6")
    C("W-Z-12", "FT", "F23 out", "2", "K_TU", "13 (NO in)", "13", "24V MOTION lid drivers (to the gate)", 1, "SC_05", 0.9, FER + " | " + FER,
      "", route="fixed", via=("UNDER_DECK",), prot="F23")
    C("W-Z-13", "K_TU", "14 (NO out)", "14", "TC_DRV_SLIDE", "V+", "V+", "24V MOTION lid drivers (gated)", 1, "SC_05", 0.9, FER + " | " + FS,
      "second core on to TC_DRV_CLAMP V+ (loop)", prot="F23")
    C("W-Z-14", "TC_DRV_SLIDE", "V+", "V+", "TC_DRV_CLAMP", "V+", "V+", "24V MOTION lid drivers (gated)", 1, "SC_05", 0.45, FS + " | " + FS, "", prot="F23")
    C("W-Z-15", "XT", "0V bus", "0V", "TC_DRV_SLIDE", "GND", "GND", "0V lid drivers", 2, "SC_05", 0.9, FER + " | " + FS, "second core to TC_DRV_CLAMP GND",
      route="fixed", via=("UNDER_DECK",), prot="F23 (return)")
    C("W-Z-16", "MAIN_CTRL", "FAN0", "+,-", "TC_BEAM_TX", "lead brown/blue", "+V,0V", "24V LOGIC beam emitter (switched by FAN0)", 2, "MC2_025", 0.03,
      "JST-XH 2-pin | moulded device lead", "emitter off at rest and during the self-test", route="fixed", via=("UNDER_DECK",), prot="Octopus fan fuse", fw="PA8")

    # ================================ head (revision L: two tool carriages + gripper axis, ADR-010) ================================
    # Nothing that is exchanged carries electronics: every switch, Hall sensor and the pusher motor of a carriage is ON the
    # carriage and reaches its MMB through that carriage's own permanent 20-way flat cable (rolling loop HL_A / HL_B).
    for i, (mot, mmb, m, pins, sw, es) in enumerate((("Z_MOTOR_A", "MMB_A", "M1", "PD4,PD3,PD5,PB5", "Z_HOME_A", "PA15"),
                                                      ("Z_MOTOR_B", "MMB_B", "M1", "PD4,PD3,PD5,PB5", "Z_HOME_B", "PA15"),
                                                      ("Z_MOTOR_GRIP", "MMB_A", "M3", "PC15,PC11,PC10,PB3", "Z_HOME_GRIP", "PD9"))):
        hb = "head_a" if mmb == "MMB_A" else "head_b"
        fw = ",".join(f"{hb}:{p}" for p in pins.split(","))
        C(f"W-H-{i + 1:02d}", mmb, f"{m} (EZ{m[1]})", "1A,1B,2A,2B", mot, "motor", "A+,A-,B+,B-", f"MOTOR {mot}", 4, "MOTOR_LEAD", 0.9,
          "JST-XH 4-pin | JST-PH 6-pin", "motor's own lead, on the carriage plate (motor fixed, does not move with Z)", prot="driver current limit", fw=fw)
        C(f"W-H-{i + 5:02d}", mmb, f"endstop {es}", f"{es},GND", sw, "C,NC", "C,NC", f"ENDSTOP {sw}", 2, "MC2_025", 0.01,
          "JST-XH 3-pin | Wago 221 lever splice", "KW10 ROLLER LEVER on the cam bar, wired NC (broken wire reads triggered: fail-safe); switch bought with its factory lead, lever splice to the pre-made lead",
          prot="signal", fw=f"{hb}:{es}")
    for c, mmb, hb in (("A", "MMB_A", "head_a"),):         # carriage A: 20-way 1.0 mm FFC (21 mm wide fits its slot, CR-L2)
        hl = f"HL_{c}"
        n0 = 10 if c == "A" else 20
        loop = dict(route=f"head_loop:{hl}", via=(f"{hl}_FIX", f"{hl}_MOV"))
        zif = "ZIF adapter 1.0 mm 20-way 20-pin | ZIF adapter 1.0 mm 20-way 20-pin"
        C(f"W-H-{n0 + 1}", mmb, "M2 (EZ2)", "1A,1B,2A,2B", f"PUSH_MOTOR_{c}", "motor lead splice", "A+,A-,B+,B-", f"MOTOR PUSH_MOTOR_{c}", 4, "FFC_20_1MM", 0.7,
          zif, f"ways 1-4 of carriage {c}'s 20-way 1.0 mm FFC rolling loop; ZIF-to-terminal adapters at both ends, the motor's own lead into the carriage-end adapter",
          prot="driver current limit", fw=",".join(f"{hb}:{p}" for p in ("PC9", "PC8", "PD2", "PB4")), **loop)
        C(f"W-H-{n0 + 2}", mmb, "endstop PA10", "PA10,GND", f"PUSH_HOME_{c}", "C,NC", "C,NC", f"ENDSTOP PUSH_HOME_{c}", 2, "FFC_20_1MM", 0.01, zif,
          "ways 5-6: pusher RETRACTED switch, NC (fail-safe)", prot="signal", fw=f"{hb}:PA10", **loop)
        C(f"W-H-{n0 + 3}", mmb, "header in_h_1", "PA9,GND", f"PUSH_CONTACT_{c}", "C,NC", "C,NC", f"ENDSTOP PUSH_CONTACT_{c} (pusher pad on the slider)", 2, "FFC_20_1MM", 0.01, zif,
          "ways 7-8: pusher contact switch, NC (broken wire reads 'contact' at once: the contact window check holds)", prot="signal", fw=f"{hb}:PA9", **loop)
        C(f"W-H-{n0 + 4}", mmb, "header in_h_0", "PC7,GND", f"FORCE_SW_{c}", "C,NC", "C,NC", f"ENDSTOP FORCE_SW_{c} (push-rod cradle force switch)", 2, "FFC_20_1MM", 0.01, zif,
          "ways 9-10: D2F on the push-rod, wired NC (broken wire reads tripped: pick-up refused at the start; a jammed rod reads not tripped: caught by the pick window)",
          prot="signal", fw=f"{hb}:PC7", **loop)
        C(f"W-H-{n0 + 5}", mmb, "sensor PC2 (ADC)", "PC2,GND", f"LATCH_SW_{c}", "ladder (2 switches + 4 resistors, lever splices)", "SIG,RET", f"SIGNAL LATCH_SW_{c} (LOCKED / CLOSED-EMPTY / NOT CLOSED)", 2, "FFC_20_1MM", 0.01, zif,
          "ways 11-12: ADR-014 three-state latch (CAD latch.SENSE): single-edge LOCK and EMPTY switches, ladder Rs 1.0 k series, R0 22 k, LOCK adds 4.7 k, EMPTY adds 2.0 k (LOCKED 4.87 k, CLOSED-EMPTY 2.32 k with both made, NOT CLOSED 23.0 k), assembled tool-free in lever splices on the CAD switch bracket LSB-L at the carriage end (no soldering), read as three analog bands on the MMB thermistor input; a broken wire or a short reads no band (LATCH_WIRE fault)", prot="signal", fw=f"{hb}:PC2", **loop)
        cab = "PC6" if c == "A" else "PA8"
        C(f"W-H-{n0 + 6}", mmb, "header in_h_2..5 + 5V/GND + cable loop", "PB12,PB10,PB1,PC5,5V,GND," + cab, f"HALL_{c}", "4 x Hall (V+, GND, OUT0-3) + loop",
          "V+,GND,O0,O1,O2,O3,LOOP", f"SIGNAL HALL_{c} tool code + cable loop", 7, "FFC_20_1MM", 0.05, zif,
          f"ways 13-19: four bought unipolar Hall switches (open collector, magnet = low), supplied from the MMB 5 V (pin UNVERIFIED); way 19 is looped to GND in the carriage-end adapter and read on {cab} (pulled cable reads 'cable lost', not 'empty'); way 20 spare",
          prot="signal", fw=",".join(f"{hb}:{p}" for p in ("PB12", "PB10", "PB1", "PC5", cab)), **loop)
    # carriage B (family CR-L2): only a 10.5 mm wide cable fits its 13 mm slot: 20-way 0.5 mm FFC, NEMA14 pusher phases on two
    # ways each (8 ways), the four switch signals, Hall supply + four outputs, the cable loop and a shared switch/Hall return on
    # two ways = 20 ways, NONE spare.  0.35 A per conductor at the 0.7 A pusher run current (system.yaml family.ffc_check).
    zb = "ZIF adapter 0.5 mm 20-way 20-pin | ZIF adapter 0.5 mm 20-way 20-pin"
    loopb = dict(route="head_loop:HL_B", via=("HL_B_FIX", "HL_B_MOV"))
    C("W-H-21", "MMB_B", "M2 (EZ2)", "1A,1B,2A,2B", "PUSH_MOTOR_B", "motor lead splice", "A+,A-,B+,B-", "MOTOR PUSH_MOTOR_B (phases doubled)", 8, "FFC_20_05MM", 0.35,
      zb, "ways 1-8: each pusher phase on TWO adjacent ways (A+ 1-2, A- 3-4, B+ 5-6, B- 7-8), paralleled in both ZIF-to-terminal adapters; max_current is PER CONDUCTOR (0.7 A run / 2)",
      prot="driver current limit", fw="head_b:PC9,head_b:PC8,head_b:PD2,head_b:PB4", **loopb)
    C("W-H-22", "MMB_B", "endstop PA10", "PA10,GND (ways 19-20)", "PUSH_HOME_B", "C,NC", "C,NC", "ENDSTOP PUSH_HOME_B", 1, "FFC_20_05MM", 0.01, zb,
      "way 9: pusher RETRACTED switch (sealed, factory lead), NC (fail-safe); return on the shared ways 19-20", prot="signal", fw="head_b:PA10", **loopb)
    C("W-H-23", "MMB_B", "header in_h_1", "PA9,GND (ways 19-20)", "PUSH_CONTACT_B", "C,NC", "C,NC", "ENDSTOP PUSH_CONTACT_B (pusher pad on the slider)", 1, "FFC_20_05MM", 0.01, zb,
      "way 10: pusher contact switch, NC (broken wire reads 'contact' at once: the contact window check holds)", prot="signal", fw="head_b:PA9", **loopb)
    C("W-H-24", "MMB_B", "header in_h_0", "PC7,GND (ways 19-20)", "FORCE_SW_B", "C,NC", "C,NC", "ENDSTOP FORCE_SW_B (push-rod cradle force switch)", 1, "FFC_20_05MM", 0.01, zb,
      "way 11: push-rod switch wired NC (broken wire reads tripped: pick-up refused at the start)", prot="signal", fw="head_b:PC7", **loopb)
    C("W-H-25", "MMB_B", "sensor PC2 (ADC)", "PC2,GND (ways 19-20)", "LATCH_SW_B", "ladder (2 switches + 4 resistors, lever splices)", "SIG,RET", "SIGNAL LATCH_SW_B (LOCKED / CLOSED-EMPTY / NOT CLOSED)", 1, "FFC_20_05MM", 0.01, zb,
      "way 12: ADR-014 three-state latch resistor ladder (as carriage A) on one way, return on the shared ways 19-20; a broken wire or a short reads no band (LATCH_WIRE fault)", prot="signal", fw="head_b:PC2", **loopb)
    C("W-H-26", "MMB_B", "header in_h_2..5 + 5V + cable loop", "PB12,PB10,PB1,PC5,5V,PA8", "HALL_B", "4 x leaded Hall (V+, OUT0-3) + loop",
      "V+,O0,O1,O2,O3,LOOP", "SIGNAL HALL_B tool code + cable loop", 6, "FFC_20_05MM", 0.05, zb,
      "ways 13-18: four bought leaded Hall switches (open collector, magnet = low) on the MMB 5 V (pin UNVERIFIED); way 18 looped to GND (the shared return) in the carriage-end adapter, read on PA8 (pulled cable reads 'cable lost')",
      prot="signal", fw="head_b:PB12,head_b:PB10,head_b:PB1,head_b:PC5,head_b:PA8", **loopb)
    C("W-H-27", "MMB_B", "GND (header)", "GND,GND", "HALL_B", "shared return (switches + Hall)", "0V,0V", "0V carriage B switch and Hall return (shared)", 2, "FFC_20_05MM", 0.1, zb,
      "ways 19-20: one return shared by the four switches, the Hall switches and the cable loop, on TWO ways: a single broken way changes nothing; both broken = every NC switch triggered, latch 'not locked', cable 'lost' (all fail-safe)", prot="signal", **loopb)
    # gripper axis (family): 12-way 0.5 mm FFC, motor single ways (0.35 A run = 70 % of the 0.5 A per-way class), 4 ways spare
    zg = "ZIF adapter 0.5 mm 12-way 12-pin | ZIF adapter 0.5 mm 12-way 12-pin"
    loopg = dict(route="head_loop:HL_G", via=("HL_G_FIX", "HL_G_MOV"))
    C("W-H-09", "MMB_B", "M3 (EZ3)", "1A,1B,2A,2B", "GRIP_MOTOR", "motor lead splice", "A+,A-,B+,B-", "MOTOR GRIP_MOTOR", 4, "FFC_12_05MM", 0.35,
      zg, "ways 1-4 of the gripper axis's 12-way 0.5 mm FFC rolling loop HL_G", prot="driver current limit", fw="head_b:PC15,head_b:PC11,head_b:PC10,head_b:PB3", **loopg)
    C("W-H-31", "MMB_B", "endstop PD9", "PD9,GND", "GRIP_OPEN_SW", "C,NC", "C,NC", "ENDSTOP GRIP_OPEN_SW", 2, "FFC_12_05MM", 0.01,
      zg, "ways 5-6: jaws-open switch (sealed, factory lead), NC (fail-safe)", prot="signal", fw="head_b:PD9", **loopg)
    C("W-H-32", "MMB_B", "header in_l_0", "PC6,GND", "GRIPPED_SW", "C,NC", "C,NC", "ENDSTOP GRIPPED_SW", 2, "FFC_12_05MM", 0.01,
      zg, "ways 7-8: 'gripped' switch on the sprung finger (sealed, factory lead), wired NC: opens when gripped; a broken wire reads 'gripped' and is caught when the jaws open; ways 9-12 spare",
      prot="signal", fw="head_b:PC6", **loopg)
    # head camera and the installed spares in the X and Y chains (ADR-004, deck_k control_spares USB_and_chain)
    YX = ("BAY_TOP", "YCH_FIX", "YCH_MOV", "XCH_FIX", "XCH_MOV")
    C("W-C-30", "HUB1", "port 7", "USB", "VC1", "USB", "USB", "USB head camera VC-1 (ring light on the camera's USB power)", 4, "USB2_HF", 0.25,
      "USB-A 4-pin | USB-B 4-pin", "chain-rated high-flex USB 2.0 through the Y and X chains (<= 5 m including the hub lead)", route="y_chain+x_chain", via=YX)
    C("W-C-31", "HUB1", "port 8 (spare)", "USB", "HEAD_SPARE", "USB (coiled, labelled)", "USB", "USB spare to the head (pressure-sensing channel reserve)", 4, "USB2_HF", 0.0,
      "USB-A 4-pin | USB-B 4-pin", "SPARE, installed in the chains, not connected at the hub (ADR-010 cable bay reserve)", route="y_chain+x_chain", via=YX)
    C("W-C-32", "XT", "XS spare 1-8", "1-8", "HEAD_SPARE", "push-in strip 1-8", "1-8", "SIGNAL spare 8-core to the head", 8, "MC8_025_HF", 0.0,
      FER + " | " + FER, "SPARE 8 x 0.25 mm2 chain-rated, terminated on spare terminals at both ends (deck_k control_spares)", route="y_chain+x_chain", via=YX)

    # ================================ deck modules ================================

    # ================================ thermocycler TC-1 and cold blocks C3/F3 (ADR-003: bought controllers) ==========
    TB = "maker's terminal-block plug"
    C("W-P-34", "FT", "F5 out", "2", "KM5", "1/L1", "1", "24V THERM", 1, "SC_60", 16.3, FS6 + " | " + FS6,
      "KM5 three poles in series (maker's DC connection: 2/T1-3/L2, 4/T2-5/L3 linked with 6 mm2 jumpers; V8 23 Nov 2026: 6 mm2 under F5 25 A)", prot="F5")
    C("W-P-41", "KM5", "6/T3", "6", "TC_RELAY", "1 (main)", "1", "24V THERM (second channel)", 1, "SC_60", 16.3, FS6 + " | " + FS6,
      "to the THERM DC contactor", route="fixed", via=("UNDER_DECK",), prot="F5")
    C("W-P-35", "TC_RELAY", "2 (main)", "2", "FT", "F32 in", "1", "24V THERM (switched) to the TEC fuse", 1, "SC_60", 13.2, FS6 + " | " + FS6,
      "short 6 mm2 jumper (F5 25 A upstream; V8: 4 mm2 Iz 23 A < 25 A)", route="fixed", via=("UNDER_DECK",), prot="F5")
    C("W-P-44", "FT", "F32 out", "2", "TEC_TC", "power in +", "+", "24V THERM (switched) to the TC-1 block controller", 1, "SC_25", 13.2, FER + " | " + TB,
      "TEC-1163 TB plug (PLG-34434): VERIFY its wire range takes 2.5 mm2", route="fixed", via=("UNDER_DECK",), prot="F32")
    C("W-P-36", "TEC_TC", "power in -", "-", "PSU_TC", "-V", "-V1,-V2", "0V THERM return", 1, "SC_25", 16.3, TB + " | " + FS,
      "THERM return straight to its own supply", route="fixed", via=("UNDER_DECK",), prot="F32 (return)")
    C("W-P-42", "TC_RELAY", "2 (main)", "2", "FT", "F31 in", "1", "24V THERM (switched) to the lid-heater fuse", 1, "SC_60", 3.7, FS6 + " | " + FS6,
      "second 6 mm2 conductor under TC_RELAY 2 (F5 25 A upstream)", route="fixed", via=("UNDER_DECK",), prot="F5")
    C("W-P-43", "FT", "F31 out", "2", "LID_SSR", "1 (+ load in)", "1", "24V THERM lid heater (fused)", 1, "SC_15", 3.7, FER + " | " + FS, "", route="fixed", via=("UNDER_DECK",), prot="F31")
    C("W-T-02", "LID_SSR", "2 (load out)", "2", "TC_LID_HEATER", "leads", "+", "HEATER THERM lid", 1, "SC_HF_075", 3.7, FS + " | Wago 221 lever splice",
      "through the TC-1 lid chain; lever splice to the heater's own leads in the lid", route="tc_chain", via=("TCC_FIX", "TCC_MOV"), prot="F31")
    C("W-T-20", "TC_LID_HEATER", "leads", "-", "PSU_TC", "-V", "-V1,-V2", "HEATER THERM lid return", 1, "SC_HF_075", 3.7, "Wago 221 lever splice | " + FS,
      "", route="tc_chain", via=("TCC_MOV", "TCC_FIX"), prot="F31 (return)")
    C("W-T-21", "MAIN_CTRL", "HE0", "+,-", "LID_SSR", "3+/4- (control)", "3,4", "SIGNAL lid heater SSR control", 2, "MC2_025", 0.011,
      "JST VH 2-pin | " + FS, "HE0 + is VIN 24 V, HE0 - is the MOSFET: SSR input 4-32 V DC, 9-11 mA", route="fixed", via=("UNDER_DECK",), prot="Octopus heater fuse", fw="PA0")
    C("W-S-29", "SR1", "34", "34", "TC_TS_BLOCK", "tab", "1", "24V SAFE THERM coil chain (from SR1 33-34)", 1, "SC_05", 0.4,
      FER + " | insulated receptacle 6.3 mm 1-pin", "SR1 output 33-34: 0.4 A = TC_RELAY + KM5 coils through the three thermostats and WDOG; from 23 Nov 2026 through the T1A fuse terminal F36 on rail E (V8: the 0.25 mm2 coil-chain leads in the TC-1 chain, Iz 1.6 A, are below F6 2 A; an open F36 drops THERM: fail-safe)",
      route="fixed", via=("UNDER_DECK",), prot="F6")
    C("W-S-30", "TC_TS_BLOCK", "tab", "2", "TC_TS_SINK", "tab", "1", "24V SAFE_E coil chain", 1, "SC_05", 0.4,
      "insulated receptacle 6.3 mm 1-pin | insulated receptacle 6.3 mm 1-pin", "high-temperature sleeving near the block", prot="F36")
    C("W-S-31", "TC_TS_SINK", "tab", "2", "TC_TS_LID", "tab", "1", "24V SAFE_E coil chain", 1, "SC_HF_025", 0.4,
      "insulated receptacle 6.3 mm 1-pin | insulated receptacle 6.3 mm 1-pin", "", route="tc_chain", via=("TCC_FIX", "TCC_MOV"), prot="F36")
    C("W-S-32", "TC_TS_LID", "tab", "2", "WDOG", "contact C", "C", "24V SAFE_E coil chain to the host-loss watchdog", 1, "SC_HF_025", 0.4,
      "insulated receptacle 6.3 mm 1-pin | " + FS, "", route="tc_chain", via=("TCC_MOV", "TCC_FIX"), prot="F36")
    C("W-S-33", "WDOG", "contact (opens on loss)", "NC/NO per proof test", "TC_RELAY", "A1 (coil +)", "A1", "24V SAFE_E coil chain end to the THERM contactors", 1, "SC_05", 0.4,
      FS + " | " + FS, "use the contact that OPENS on loss of heartbeat (data sheet: relay energises on loss -> NC); LH_THERM_ON proves it", prot="F36")
    C("W-S-34", "TC_RELAY", "A2 (coil -)", "A2", "MAIN_CTRL", "HE1", "-", "SIGNAL THERM enable: TC_RELAY coil return through HE1", 1, "SC_05", 0.2,
      FS + " | JST VH 2-pin", "HE1 MOSFET can only OPEN the coil circuit; the contactor's plug-in suppressor takes the coil energy", prot="F6", fw="PA3")
    C("W-S-44", "XT", "LOG7/0V", "4,0V", "WDOG", "supply +/-", "+,-", "24V LOGIC watchdog supply", 2, "SC_05", 0.05, FER + " | " + FS, "", route="fixed", via=("UNDER_DECK",), prot="F7")
    C("W-S-45", "MAIN_CTRL", "FAN1", "+,-", "WDOG", "heartbeat in +/-", "HB+,HB-", "SIGNAL watchdog heartbeat (2 Hz from the Octopus)", 2, "MC2_025", 0.01,
      "JST-XH 2-pin | " + FS, "FAN1 + = 24 V, FAN1 - = MOSFET: the isolated sourcing input sees 24 V pulses", route="fixed", via=("UNDER_DECK",), prot="Octopus fan fuse", fw="PE5")
    C("W-S-46", "TC_RELAY", "mirror NC 2 (31-32)", "31,32", "MAIN_CTRL", "T0", "PF4,GND", "SIGNAL TC_RELAY mirror 2 (weld check)", 2, "MC2_025", 0.001,
      FS + " | JST-XH 2-pin", "second mirror contact (auxiliary block): closed while the contactor is OFF", route="fixed", via=("UNDER_DECK",), prot="signal", fw="PF4")
    C("W-S-47", "TB_RELAY", "mirror NC 2 (31-32)", "31,32", "MAIN_CTRL", "T1", "PF5,GND", "SIGNAL TB_RELAY mirror 2 (status)", 2, "MC2_025", 0.001,
      FS + " | JST-XH 2-pin", "", route="fixed", via=("UNDER_DECK",), prot="signal", fw="PF5")
    C("W-C-04", "HUB1", "port 1", "USB", "TEC_TC", "USB", "USB", "USB TC-1 block controller (MeCom)", 4, "USB2_AB", 0.1, "USB-A 4-pin | USB (device side, per maker)",
      "", route="fixed", via=("UNDER_DECK",))
    C("W-T-01", "TEC_TC", "TEC out +/-", "+,-", "TC_TEC", "string leads", "+,-", "TEC THERM TC-1 string", 2, "SC_25_SIL", 13.05, TB + " | Wago 221 lever splice",
      "two modules in series; lever splices to the module leads (4 mm2 class, 32 A)", prot="TEC_TC 13 A current limit + F5")
    C("W-T-03", "XT", "LOG7/0V", "5,0V", "TC_FAN1", "4-pin", "+24,GND", "24V LOGIC fan TC-1", 2, "DEVICE_LEAD", 0.25, FER + " | Molex KK fan 4-pin",
      "pre-made fan pigtail; PWM and tach left open (full speed)", route="fixed", via=("UNDER_DECK",), prot="F7")
    C("W-T-04", "XT", "LOG7/0V", "6,0V", "TC_FAN2", "4-pin", "+24,GND", "24V LOGIC fan TC-1", 2, "DEVICE_LEAD", 0.25, FER + " | Molex KK fan 4-pin", "", route="fixed", via=("UNDER_DECK",), prot="F7")
    C("W-C-16", "MAIN_CTRL", "Driver2 socket", "STEP,DIR,EN,GND", "XDA2", "pins", "STEP,DIR,EN,GND", "STEP/DIR TC lid slide", 4, "PLUG_IN", 0.01,
      "board plug-in | board plug-in", "breakout with 5 V level shift (DM542 inputs need 4-5 V high)", fw="PF11,PG3,PG5")
    C("W-C-17", "MAIN_CTRL", "Driver5 socket", "STEP,DIR,EN,GND", "XDA5", "pins", "STEP,DIR,EN,GND", "STEP/DIR TC lid clamp", 4, "PLUG_IN", 0.01,
      "board plug-in | board plug-in", "breakout with 5 V level shift", fw="PC13,PF0,PF1")
    C("W-T-22", "XDA2", "terminals", "STEP,DIR,EN,COM", "TC_DRV_SLIDE", "signal", "PUL,DIR,ENA,COM", "STEP/DIR TC lid slide", 4, "MC4_025", 0.02, FS + " | " + FS,
      "", route="fixed", via=("UNDER_DECK",), prot="signal")
    C("W-T-23", "XDA5", "terminals", "STEP,DIR,EN,COM", "TC_DRV_CLAMP", "signal", "PUL,DIR,ENA,COM", "STEP/DIR TC lid clamp", 4, "MC4_025", 0.02, FS + " | " + FS,
      "", route="fixed", via=("UNDER_DECK",), prot="signal")
    C("W-T-05", "TC_DRV_SLIDE", "A+,A-,B+,B-", "A+,A-,B+,B-", "TC_M_SLIDE", "motor", "A+,A-,B+,B-", "MOTOR TC lid slide", 4, "MOTOR_LEAD", 1.0,
      FS + " | JST-PH 6-pin", "bought motor lead (PH-6 to bare ends) into the driver's screw terminals", prot="DM542 current setting")
    C("W-T-06", "TC_DRV_CLAMP", "A+,A-,B+,B-", "A+,A-,B+,B-", "TC_M_CLAMP", "motor", "A+,A-,B+,B-", "MOTOR TC clamp", 4, "SC_HF_025", 1.0,
      FS + " | Wago 221 lever splice", "through the TC-1 lid chain; lever splices to the motor's bought PH-6 lead on the lid carriage", route="tc_chain", via=("TCC_FIX", "TCC_MOV"), prot="DM542 current setting")
    C("W-T-07", "TC_S_OPEN", "pre-wired lead", "V,G,S", "XT", "XL", "1-3", "SENSOR lid open (supply + output)", 3, "DEVICE_LEAD", 0.02, "moulded device lead | " + FER,
      "slot sensor with its maker's pre-wired lead (EE-1006 class); XL.1/2 from LOG7/0V", route="fixed", via=("UNDER_DECK",), prot="F7", fw="PF10")
    C("W-T-24", "XT", "XL", "3", "XDA4", "terminal PF10", "PF10", "SIGNAL lid open (slide endstop)", 1, "SC_05", 0.01, FER + " | " + FS, "", prot="signal", fw="PF10")
    C("W-T-08", "TC_S_CLOSED", "pre-wired lead", "V,G,S", "XT", "XL", "4-6", "SENSOR lid closed (supply + output)", 3, "DEVICE_LEAD", 0.02, "moulded device lead | " + FER,
      "slot sensor with its pre-wired lead, set dark-ON (flag in the slot = output on = closed)", route="fixed", via=("UNDER_DECK",), prot="F7", fw="PF6")
    C("W-T-25", "XT", "XL", "6", "MAIN_CTRL", "T2", "PF6,GND", "SIGNAL lid closed", 2, "MC2_025", 0.001, FER + " | JST-XH 2-pin", "", prot="signal", fw="PF6")
    C("W-T-09", "TC_SW_CLAMP", "C,NC", "C,NC", "XDA4", "terminals PG2/GND", "PG2,GND", "ENDSTOP TC clamp UP", 2, "SC_HF_025", 0.01, HS + " | " + FS,
      "wired NC (fail-safe)", route="tc_chain", via=("TCC_MOV", "TCC_FIX"), prot="signal", fw="PG2")
    C("W-T-17", "TC_SW_SEALED", "C,NO", "C,NO", "MAIN_CTRL", "T3", "PF7,GND", "ENDSTOP TC clamp SEALED", 2, "SC_HF_025", 0.01, HS + " | JST-XH 2-pin",
      "", route="tc_chain", via=("TCC_MOV", "TCC_FIX"), prot="signal", fw="PF7")
    for i, (ntc, port) in enumerate([("TC_NTC_BLOCK", "object sensor"), ("TC_NTC_SINK", "aux NTC 1"), ("TC_NTC_A1", "aux NTC 2")]):
        C(f"W-T-{12 + i:02d}", "TEC_TC", port, "1,2", ntc, "leads", "1,2", f"SENSOR NTC {ntc}", 2, "MC2_025", 0.001, TB + " | Wago 221 lever splice",
          "10 k NTC (B57861S, Steinhart-Hart in the controller)", prot="signal")
    C("W-T-16", "TC_NTC_LID", "leads", "1,2", "MAIN_CTRL", "TB", "PF3,GND", "SENSOR NTC TC_NTC_LID", 2, "SC_HF_025", 0.001, "Wago 221 lever splice | JST-XH 2-pin", "",
      route="tc_chain", via=("TCC_MOV", "TCC_FIX"), prot="signal", fw="PF3")
    # --- thermal modules on THERM_AUX (size family: the same four in every size) ---
    C("W-P-27", "XT", "THERM_AUX", "1", "TB_RELAY", "1 (main)", "1", "24V THERM_AUX", 1, "SC_40", 14.6, FER + " | " + FS,
      "THERM_AUX through its own DC contactor (CBP2, same compartment)", prot="F4")
    C("W-C-05", "HUB1", "port 2", "USB", "TEC_COLD", "USB", "USB", "USB cold-block controller (MeCom)", 4, "USB2_AB", 0.1,
      "USB-A 4-pin | USB (device side, per maker)", "", route="fixed", via=("UNDER_DECK",))
    # ================================ thermal modules on THERM_AUX (family: B2, G4, A3, A4; G0 reserved) ================================
    # Four rev K thermal modules (2 x TEC1-12706 each, 3.5 A / 18 V): TEC_COLD ch1 B2 strips / ch2 G4 cold plate CPB-1,
    # TEC_TUBE ch1 A3 CTB-1 / ch2 A4 CTB-2.  TEC_WARM (37 C module under G0) is RESERVED in every size: DIN space, fuse F35,
    # USB port and the PSU_AUX headroom are kept; its rows below are marked RESERVED (not bought, not fitted).
    for tec, f in (("TEC_COLD", "F33"), ("TEC_TUBE", "F34"), ("TEC_WARM", "F35")):
        res = "RESERVED, NOT FITTED: 37 C module at G0 (family open question: hazardous liquid at 37 C); " if tec == "TEC_WARM" else ""
        C(f"W-K-{ {'TEC_COLD': 10, 'TEC_TUBE': 11, 'TEC_WARM': 12}[tec] }", "FT2", f"{f} out", "2", tec, "power in +", "+", f"24V THERM_AUX (switched) {tec}", 1, "SC_15", 5.84,
          FER + " | " + TB, res + "2 channels x 3.5 A x 18 V / 0.9 / 24 V", route="fixed", via=("UNDER_DECK",), prot=f)
        C(f"W-K-{ {'TEC_COLD': 13, 'TEC_TUBE': 14, 'TEC_WARM': 15}[tec] }", tec, "power in -", "-", "PSU_AUX", "-V", "-V1,-V2,-V3", f"0V THERM_AUX return {tec}", 1, "SC_15", 5.84,
          TB + " | " + FS, res + "return straight to its own supply", route="fixed", via=("UNDER_DECK",), prot=f"{f} (return)")
    C("W-K-16", "TB_RELAY", "2 (main)", "2", "FT2", "F33/F34/F35 in (jumper bar)", "1", "24V THERM_AUX (switched) to the controller fuses", 1, "SC_40", 14.6,
      FS + " | " + FER, "", prot="F4")
    for i, (tec, ch, tm) in enumerate((("TEC_COLD", "ch1", "TM_B2"), ("TEC_COLD", "ch2", "TM_G4"), ("TEC_TUBE", "ch1", "TM_A3"), ("TEC_TUBE", "ch2", "TM_A4"))):
        C(f"W-K-{i + 1:02d}", tec, f"{ch} TEC out +/-, object NTC, sink NTC", "T+,T-,N1a,N1b,N2a,N2b", tm, "thermal connector (8-way, 6 used)", "1-6",
          f"TEC THERM_AUX {tm} Peltier + NTCs", 6, "MC6_075", 3.5, TB + " | Molex Mini-Fit Jr 8-pin",
          "bought pre-made Mini-Fit Jr 8-pin lead, flying ends into the maker's TB plug; 8-way (not 6) so it cannot be mated with the place's 6-way standard connector",
          route="fixed", via=("UNDER_DECK", "CPL") if tm == "TM_G4" else ("UNDER_DECK",), prot=f"{tec} 3.5 A current limit")
    # thermostat coil chain: SAFE_E -> each module's 72 C cut-off + 70 C sink thermostat (inside the module, between its
    # chain-in and chain-out 2-way connectors) -> TB_RELAY coil.  A 37 C module at G0 later goes into the chain with one
    # more pre-made 2-way lead (option kit).
    CHAIN_TM = ["TM_B2", "TM_A3", "TM_A4", "TM_G4"]
    C("W-S-35", "XT", "SAFE_E", "3", "TM_B2", "chain in (2-way)", "1,2", "24V SAFE_E cold-module coil chain", 2, "MC2_05", 0.2, FER + " | Molex Mini-Fit Jr 2-pin",
      "pre-made Mini-Fit Jr 2-pin lead (second core spare)", route="fixed", via=("UNDER_DECK",), prot="F6")
    for i, (a, b) in enumerate(zip(CHAIN_TM, CHAIN_TM[1:])):
        C(f"W-S-{60 + i}", a, "chain out (2-way)", "1,2", b, "chain in (2-way)", "1,2", "24V SAFE_E cold-module coil chain", 2, "MC2_05", 0.2,
          "Molex Mini-Fit Jr 2-pin | Molex Mini-Fit Jr 2-pin", "pre-made module-to-module chain lead; chain in/out of a module are interchangeable in a series loop",
          route="fixed", via=("UNDER_DECK", "CPL") if b == "TM_G4" else ("UNDER_DECK",), prot="F6")
    C("W-S-37", "TM_G4", "chain out (2-way)", "1,2", "TB_RELAY", "A1 (coil +)", "A1", "24V SAFE_E cold-module coil chain end", 2, "MC2_05", 0.2, "Molex Mini-Fit Jr 2-pin | " + FS,
      "", route="fixed", via=("CPL", "UNDER_DECK"), prot="F6")
    C("W-S-38", "TB_RELAY", "A2 (coil -)", "A2", "XT", "0V bus", "0V", "0V cold-module contactor coil return", 1, "SC_05", 0.2, FS + " | " + FER,
      "no host control: the cold modules hold on host loss", prot="F6")

    # ================================ deck module standard connectors (ADR-010; family places 'connector: fitted') ================================
    # One keyed 6-way connector CON-F per module place under the deck (24 V 3 A fused, 0 V, SAFE A, SAFE B, 0 V sense, spare)
    # plus a USB 2.0 panel receptacle.  Module places: A3 (zone A3 + A4), B2, G4, G0 (reserved: loop-back plug) in every
    # size; D3 (balance) and D4 (weighing nest) in L.  B1 (tip station) and B3 (liquid waste) use the same CON-F part as
    # STATION connectors for their dedicated leads (tip gate, VC-2 light; level switches), not SAFE lines.  24 V from the
    # MODULE rail (PSU_AUX through KM1/KM2 pole 3) through one 3.15 A fuse per place in XM (8 positions, sized for L).
    SAFE_A = {"A3": "PE7", "B2": "PE8", "G4": "PE9", "G0": "PE10", "D3": "PE12", "D4": "PE13"}
    SAFE_B = {"A3": "PA4", "B2": "PA5", "G4": "PA6", "G0": "PA7", "D3": "PB1", "D4": "PB2"}
    BACKP = {"G4", "G0", "D3", "D4"}
    for i, p in enumerate(SAFE_A):
        f = f"F{41 + i}"
        lb = "; G0 is kept for a 37 C module: a bought loop-back plug closes SAFE A/B until that module is fitted" if p == "G0" else ""
        C(f"W-N-{i + 1:02d}", "XM", f"{f} out, 0V, SA_{p}, SB_{p}, 0V, spare", "1-6", f"MODCONN_{p}", "standard connector 6-way", "1-6",
          f"24V MODULE place {p} + SAFE lines", 6, "MC6_05", 3.15, FER + " | Molex Mini-Fit Jr 6-pin",
          f"bought pre-made Mini-Fit Jr 6-pin lead; flying ends: 24 V/0 V/0 V sense into XM, SAFE A and SAFE B straight into the EXP breakout terminals on the same rail (no trunk); SAFE lines closed to 0 V by the module while safe (unplugged = NOT safe){lb}",
          route="fixed", via=("UNDER_DECK", "CPL") if p in BACKP else ("UNDER_DECK",), prot=f, fw=f"io:{SAFE_A[p]},io:{SAFE_B[p]}")
        C(f"W-N-{i + 11:02d}", "HUB2", f"port {i + 2}", "USB", f"MODCONN_{p}", "USB-B panel receptacle", "USB", f"USB deck place {p}", 4, "USB2_AB", 0.5,
          "USB-A 4-pin | USB-B 4-pin", "USB extension to the place's panel receptacle", route="fixed", via=("UNDER_DECK", "CPL") if p in BACKP else ("UNDER_DECK",))
    sa_ = [v for k, v in SAFE_A.items() if in_size(f"MODCONN_{k}")]
    sb_ = [v for k, v in SAFE_B.items() if in_size(f"MODCONN_{k}")]
    C("W-N-22", "IO_CTRL", "EXP1", "10-way IDC", "EXPB1", "IDC", "10-way", "SIGNAL EXP1 breakout", 10, "PLUG_IN", 0.01, "board plug-in | board plug-in",
      "passive EXP breakout (screw terminals) on a bought 10-way IDC ribbon", fw=",".join(f"io:{v}" for v in sa_))
    C("W-N-23", "IO_CTRL", "EXP2", "10-way IDC", "EXPB2", "IDC", "10-way", "SIGNAL EXP2 breakout", 10, "PLUG_IN", 0.01, "board plug-in | board plug-in",
      "", fw=",".join(f"io:{v}" for v in sb_))
    # fitted modules mate with the place connectors
    for j, (p, tm) in enumerate((("B2", "TM_B2"), ("A3", "TM_A3"), ("G4", "TM_G4"))):
        C(f"W-N-3{j}", f"MODCONN_{p}", "standard connector 6-way", "1-6", tm, "module plug", "1-6", f"24V MODULE {tm} fan + SAFE loop-back", 6, "PLUG_IN", 0.2 if p != "A3" else 0.4,
          "board plug-in | board plug-in", "thermal module: fan on pins 1-2, SAFE A/B looped to 0 V inside the module (a thermal block is always safe to enter; its own thermostats act on THERM_AUX)"
          + ("; zone A3+A4: the second module's fan is on the CAD 2-way fan connector CTB_FANCON (W-N-35/36)" if p == "A3" else ""))
    C("W-N-35", "XM", "F41 out / 0V", "2,0V", "CTB_FANCON", "panel connector (2-way)", "1,2", "24V MODULE zone A3+A4 second fan (F41)", 2, "MC2_05", 0.2,
      FER + " | Molex Micro-Fit 3.0 2-pin", "CAD services_interface CTB-1_fan_connector: keyed 2-way panel connector beside the A3 place connector, lead to gland G1; a DIFFERENT family from the Mini-Fit 2-pin chain leads so a fan lead can never enter the thermostat coil chain",
      route="fixed", via=("UNDER_DECK",), prot="F41")
    C("W-N-36", "CTB_FANCON", "panel connector (2-way)", "1,2", "TM_A4", "fan lead", "1,2", "24V MODULE TM_A4 heat-sink fan", 2, "PLUG_IN", 0.2,
      "board plug-in | board plug-in", "the zone's second heat-sink fan plugs in here (replaces the module-to-module fan-out); CAD text names CTB-1's fan: CTB-1's own fan is on the A3 place connector pins 1-2 (module owner to confirm)")
    C("W-N-41", "MODCONN_D3", "USB-B", "USB", "BALANCE", "data port", "USB", "USB balance (0.1 mg, host device)", 4, "USB2_AB", 0.1, "USB-B 4-pin | USB-B 4-pin",
      "L only: balance's own USB cable (or RS232 + bought USB adapter); SAFE A/B on a bought loop-back plug")
    C("W-N-42", "HUB2", "port 8", "USB", "SHAKER", "USB", "USB", "USB heater-shaker (host device, own supply)", 4, "USB2_AB", 0.1, "USB-A 4-pin | USB-B 4-pin",
      "F4 has no place connector (the bought shaker brings its own mains/USB leads): USB extension from HUB2 under the deck", route="fixed", via=("UNDER_DECK",))
    C("W-N-43", "MODCONN_D4", "USB-B", "USB", "WEIGH_IF", "USB", "USB", "USB load-cell bridge interface (host device)", 4, "USB2_AB", 0.1, "USB-B 4-pin | USB-B 4-pin",
      "L only: bought 4-channel USB bridge interface (WN-1 + 3 spare)")
    C("W-N-44", "WN1_CELL", "bridge lead", "E+,E-,S+,S-", "WEIGH_IF", "bridge input 0", "E+,E-,S+,S-", "SENSOR load cell WN-1", 4, "DEVICE_LEAD", 0.005,
      "moulded device lead | " + FS, "L only: cell's own screened 4-wire lead into the interface's screw terminals", prot="signal")
    C("W-N-45", "MAINS_STRIP", "outlet 1", "L,N,PE", "SHAKER", "own supply", "L,N,PE", "MAINS heater-shaker supply (moulded cord)", 3, "DEVICE_LEAD", 1.0,
      "moulded device lead | moulded device lead", "shaker's own bought supply on the switched strip (> 72 W rule, ADR-010); plugging in is not wiring")
    C("W-N-46", "MAINS_STRIP", "outlet 2", "L,N,PE", "BALANCE", "own adapter", "L,N,PE", "MAINS balance adapter (moulded cord)", 3, "DEVICE_LEAD", 0.1,
      "moulded device lead | moulded device lead", "L only (outlet 2 fitted in every size, unused in S/M); restarts (re-tare) after an e-stop")
    C("W-P-54", "KM2", "4/T2", "4", "XM", "MOD bus in", "1", "24V MODULE (switched) to the place fuses", 1, "SC_15", 1.0, FS + " | " + FER, "", prot="F9")
    C("W-P-52", "FT2", "F9 out", "2", "KM1", "3/L2", "3", "24V MODULE", 1, "SC_15", 1.0, FER + " | " + FS, "pole 2 of the e-stop contactors", prot="F9")
    C("W-P-53", "KM1", "4/T2", "4", "KM2", "3/L2", "3", "24V MODULE", 1, "SC_15", 1.0, FS + " | " + FS, "", prot="F9")
    C("W-P-55", "XT", "0V bus", "0V", "XM", "0V bus", "0V", "0V MODULE and SAFE return", 1, "SC_15", 1.0, FER + " | " + FER, "", prot="F9 (return)")

    # ================================ io board, dock, environment ================================
    C("W-P-60", "XT", "LOGIC/0V", "6,0V", "IO_CTRL", "POWER (main)", "+,-", "24V LOGIC io board (inputs, HE/FAN outputs)", 2, "SC_10", 0.8, FER + " | " + FS,
      "second Octopus Pro: logic, heater/fan outputs (deck light, VC-2 light, K_UV coil) from its main input; MOTOR_POWER unused", route="fixed", via=("BACK_BASE",), prot="F1")
    C("W-C-20", "HOST", "USB-A 2", "USB", "IO_CTRL", "USB-C", "USB", "USB Klipper io MCU", 4, "USB2_AC", 0.5, "USB-A 4-pin | USB-C", "direct to the host, not through a hub")
    C("W-C-21", "HOST", "USB-A 4", "USB", "HUB2", "upstream", "USB", "USB hub 2 uplink (head, deck places)", 4, "USB2_AB", 0.1, "USB-A 4-pin | USB-B 4-pin", "", route="fixed", via=("BACK_BASE",))
    C("W-P-24b", "PSU_5V", "+Vo/-Vo", "3,4", "HUB2", "DC in", "+,-", "5V hub 2", 2, "SC_05", 1.5, FS + " | " + FS, "bought hub with screw-terminal DC input", route="fixed", via=("BACK_BASE",), prot="PSU_5V current limit")
    for i, x in enumerate((-296, -218, -140, -62, 16, 94, 172, 250)):
        pin = ("PG6", "PG9", "PG10", "PG11", "PG12", "PG13", "PG14", "PG15")[i]
        C(f"W-I-{i + 1:02d}", "IO_CTRL", f"STOP_{i}", f"{pin},GND", f"BAY_SW_{i + 1}", "C,NC", "C,NC", f"ENDSTOP dock bay {i + 1} occupied", 2, "MC2_025", 0.01,
          "JST-XH 3-pin | Wago 221 lever splice", "sealed switch with factory lead, wired NC (broken wire reads 'occupied': no drop-off into that bay); lever splice to the pre-made JST-XH lead",
          route="fixed", via=("UNDER_DECK", "CPL", "DOCK_TRUNK"), prot="signal", fw=f"io:{pin}")
    # ADR-014 dock lift DLB-1: motor from the main Octopus socket 6 (MOTION_G), bar DOWN slot sensor and bar OPEN-height switch on
    # the io EXP1 breakout (spare in every size: PE14, PE15); all three through the connector plane like the bay switches
    C("W-I-40", "MAIN_CTRL", "M6 (Driver6 socket, TMC5160 stepstick)", "1A,1B,2A,2B", "DOCK_LIFT_MOTOR", "motor", "A+,A-,B+,B-", "MOTOR DOCK_LIFT_MOTOR", 4, "MC4_05_SH", 1.3,
      "Wago 221 lever splice | Wago 221 lever splice", "4 x 0.5 mm2 screened (screen at the board end only) spliced at the board end to a pre-made JST-XH 4-pin lead on the socket-6 motor header and at the lift to the motor's own JST-PH 6 lead (both pre-made, no crimping); 1.3 A rms while the bar moves",
      route="fixed", via=("UNDER_DECK", "CPL", "DOCK_TRUNK"), prot="driver current limit", fw="PE2,PE3,PD4,PE1")
    C("W-I-41", "XT", "XK", "1-3", "DOCK_LIFT_HOME", "pre-wired lead (V, G, S)", "V,G,S", "SENSOR dock lift bar DOWN (supply + output)", 3, "MC3_025", 0.02,
      FER + " | Wago 221 lever splice", "slot sensor (EE-SX67x class, NPN, its pre-wired lead spliced at the lift): output conducts with the flag in the slot = bar DOWN; XK.1/2 from LOG7/0V (F7)",
      route="fixed", via=("UNDER_DECK", "CPL", "DOCK_TRUNK"), prot="F7", fw="io:PE14")
    C("W-I-42", "XT", "XK", "3", "EXPB1", "terminal PE14", "PE14", "SIGNAL dock lift bar DOWN (endstop)", 1, "SC_05", 0.01, FER + " | " + FS,
      "slot sensor output to io PE14 ('^!': broken wire = not home)", prot="signal", fw="io:PE14")
    C("W-I-43", "DOCK_BAR_OPEN", "C,NO", "C,NO", "EXPB1", "terminals PE15/GND", "PE15,GND", "SIGNAL dock lift bar at the OPEN height", 2, "MC2_025", 0.001,
      "Wago 221 lever splice | " + FS, "sealed switch with factory lead made by the bar's flag from 33.1 mm up, wired NO ('^!': broken wire = not at height)",
      route="fixed", via=("UNDER_DECK", "CPL", "DOCK_TRUNK"), prot="signal", fw="io:PE15")
    C("W-I-10", "IO_CTRL", "PWR-DET", "PC0,GND", "AIRFLOW_SW", "C,NO", "C,NO", "SIGNAL extraction airflow proven", 2, "MC2_025", 0.001, "JST-XH 3-pin | Wago 221 lever splice",
      "differential-pressure switch NO contact CLOSED on proven flow ('^!': broken wire = not proven); at the extraction fan, low on the left panel (CR-L6)", route="fixed", via=("UNDER_DECK", "CPL", "LEFT_LOW"), prot="signal", fw="io:PC0")
    C("W-I-11", "IO_CTRL", "PROBE port", "PB7,GND", "DRAWER_SW", "C,NO", "C,NO", "SIGNAL tip drawer closed", 2, "MC2_025", 0.001, "JST-XH 3-pin | Wago 221 lever splice",
      "NO made with the drawer fully in (broken wire = open = no tip drop)", route="fixed", via=("BACK_BASE",), prot="signal", fw="io:PB7")
    C("W-I-12", "XT", "XB", "1-4", "BIN_JB", "lever splice", "+RX,+TX,0V,OUT", "24V LOGIC (F7) bin-full beam pair supply + OUT", 4, "MC4_025", 0.06, FER + " | Wago 221 lever splice",
      "CAD services_interface tip_bin_full_beam: own 4-core lead from the chute outlet up the chute's +X wall, on the base top to the service loops and floor gland G1 (not through the B1 station connector); receiver NPN LIGHT-ON",
      route="fixed", via=("BIN_ROUTE", "GLAND_C", "BACK_BASE"), prot="F7", fw="io:PB6")
    C("W-I-12a", "BIN_RX", "lead", "+24,0V,OUT", "BIN_JB", "lever splice", "+RX,0V,OUT", "24V LOGIC bin-full receiver + OUT", 3, "DEVICE_LEAD", 0.03, "moulded device lead | Wago 221 lever splice",
      "receiver across the chute outlet 15 mm above it, NPN LIGHT-ON (same part as the tip gate pair)", prot="F7", fw="io:PB6")
    C("W-I-13", "XT", "XB", "3", "IO_CTRL", "BLTouch port", "PB6,GND", "SIGNAL tip bin clear", 2, "MC2_025", 0.01, FER + " | JST-XH 5-pin", "LIGHT-ON: broken wire / blocked beam = full", prot="signal", fw="io:PB6")
    C("W-I-14", "BIN_TX", "lead", "+24,0V", "BIN_JB", "lever splice", "+TX,0V", "24V LOGIC bin-full emitter", 2, "DEVICE_LEAD", 0.03, "moulded device lead | Wago 221 lever splice", "emitter at the chute outlet (CAD TWB-F_bin_full_beam_emitter)", prot="F7")
    for i, (lvl, pin, port) in enumerate((("LVL_1", "PF9", "XDB4 terminal PF9"), ("LVL_2", "PF10", "XDB4 terminal PF10"))):
        C(f"W-I-{15 + 2 * i}", lvl, "lead", "+24,0V,OUT", "XT", "XW", f"{1 + 3 * i}-{3 + 3 * i}", f"24V LOGIC (F7) waste level {lvl} + OUT", 3, "MC3_025", 0.03,
          "moulded device lead | " + FER, "capacitive level switch, NPN, set to CONDUCT while the level is BELOW 80 % (full, dead or broken wire = full)", route="fixed",
          via=("BACK_BASE",), prot="F7", fw=f"io:{pin}")
        C(f"W-I-{16 + 2 * i}", "XT", "XW", f"{3 + 3 * i}", "XDB4", port, pin, f"SIGNAL waste level {lvl}", 1, "SC_05", 0.01, FER + " | " + FS, "", prot="signal", fw=f"io:{pin}")
    C("W-I-19", "SHUT_H1", "C,NO", "C,NO", "XDB4", "terminals PG2/GND", "PG2,GND", "SIGNAL hand-off shutter H1 closed", 2, "MC2_025", 0.001, "Wago 221 lever splice | " + FS,
      "M and L only (the starter has blank plates in H1 and H2, CAD 23 Nov 2026): NO made when the shutter is closed", route="fixed", via=("BAY_TOP", "FRONT_RIGHT"), prot="signal", fw="io:PG2")
    C("W-I-20", "SHUT_H2", "C,NO", "C,NO", "XDB5", "terminals PC13/GND", "PC13,GND", "SIGNAL hand-off shutter H2 closed", 2, "MC2_025", 0.001, "Wago 221 lever splice | " + FS,
      "M and L only (the starter has a blank plate in H2): NO made when the shutter is closed", route="fixed", via=("BAY_TOP", "FRONT_RIGHT"), prot="signal", fw="io:PC13")
    C("W-I-21", "IO_CTRL", "Driver4 socket", "STEP,DIR,EN,GND", "XDB4", "pins", "PF9,PF10,PG2,GND", "SIGNAL breakout (inputs)", 4, "PLUG_IN", 0.01, "board plug-in | board plug-in",
      "passive breakout, socket 4 pins as inputs; PG2 = shutter H1 (M, L; reserved in S: blank plates at H1 and H2, CAD 23 Nov 2026)", fw="io:PF9,io:PF10" + (",io:PG2" if SIZE != "S" else ""))
    C("W-I-22", "IO_CTRL", "Driver5 socket", "STEP,DIR,EN,GND", "XDB5", "pins", "PC13,PF0,PF1,GND", "SIGNAL breakout (inputs)", 4, "PLUG_IN", 0.01, "board plug-in | board plug-in",
      "passive breakout; PF0/PF1 compartment fan-fail inputs (23 Nov 2026); PC13 = shutter H2 (M, L; reserved in S)", fw="io:PF0,io:PF1" + (",io:PC13" if SIZE != "S" else ""))
    for i_, (fan_, pin_) in enumerate((("CAB_FAN1", "PF0"), ("CAB_FAN2", "PF1"))):
        C(f"W-I-{30 + i_}", fan_, "RD lead", "RD", "XDB5", f"terminal {pin_}", pin_, f"SIGNAL compartment fan {i_ + 1} rotation detect", 1, "DEVICE_LEAD", 0.005,
          "solder + heat-shrink | " + FS, "open collector, LOW while the fan turns (fan 0 V = board 0 V); stopped fan, dead output or broken wire reads HIGH = FAILED (fail-safe); purchase condition: 92 mm 24 V fan with RD output",
          prot="signal", fw=f"io:{pin_}")
    C("W-I-23", "IO_CTRL", "HE2", "+,-", "DECK_LIGHT", "lead", "+,-", "24V LOGIC deck LED strip (switched)", 2, "MC2_05", 0.5, "JST VH 2-pin | Wago 221 lever splice",
      "", route="fixed", via=("BACK_BASE",), prot="Octopus heater fuse", fw="io:PB0")
    C("W-I-24", "IO_CTRL", "FAN0", "+,-", "VC2_LIGHT", "lead", "+,-", "24V LOGIC VC-2 backlight (switched)", 2, "MC2_025", 0.05, "JST-XH 2-pin | Wago 221 lever splice",
      "", route="fixed", via=("UNDER_DECK",), prot="Octopus fan fuse", fw="io:PA8")
    C("W-I-25", "XT", "LOG8/0V", "1,0V", "EXT_FAN", "lead", "+24,0V", "24V LOGIC (F8) extraction fan, always on", 2, "MC2_075", 1.0, FER + " | Wago 221 lever splice",
      "EC fan runs whenever the machine is on (not switched by firmware: ventilation never depends on the host); its speed input left at full; fan outside the left panel, intake grille low (CR-L6)", route="fixed", via=("UNDER_DECK", "CPL", "LEFT_LOW"), prot="F8")
    C("W-P-61", "FT", "F8 out", "2", "XT", "LOG8", "1", "24V LOGIC (F8 extraction)", 1, "SC_05", 1.0, FER + " | " + FER, "", prot="F8")
    C("W-C-22", "HUB1", "port 5", "USB", "VC2", "USB", "USB", "USB tip-check camera VC-2", 4, "USB2_AB", 0.25, "USB-A 4-pin | USB-B 4-pin", "", route="fixed", via=("UNDER_DECK",))
    C("W-C-23", "HUB1", "port 3", "USB", "TEC_TUBE", "USB", "USB", "USB tube-block controller (MeCom)", 4, "USB2_AB", 0.1, "USB-A 4-pin | USB (device side, per maker)", "", route="fixed", via=("UNDER_DECK",))
    C("W-C-24", "HUB1", "port 4", "USB", "TEC_WARM", "USB", "USB", "USB warm-position controller (MeCom)", 4, "USB2_AB", 0.1, "USB-A 4-pin | USB (device side, per maker)", "RESERVED, NOT FITTED: TEC_WARM (37 C module at G0); hub port kept free", route="fixed", via=("UNDER_DECK",))
    # ---- UV-C OPTION (reserved, not fitted): supply from MOTION after the door contactors, firmware enable through K_UV
    C("W-U-01", "FT", "F24 out", "2", "K_UV", "13 (NO in)", "13", "24V MOTION UV option (after KM3/KM4)", 1, "SC_05", 0.8, FER + " | " + FER, "RESERVED, NOT FITTED: UV-C option", prot="F24")
    C("W-U-02", "K_UV", "14 (NO out)", "14", "UV_BALLAST", "24 V in", "+", "24V MOTION UV ballast (switched)", 2, "MC2_05", 0.8, FER + " | " + FS, "RESERVED, NOT FITTED", prot="F24")
    C("W-U-03", "IO_CTRL", "HE1", "+,-", "K_UV", "A1/A2", "A1,A2", "SIGNAL UV enable (K_UV coil)", 2, "MC2_025", 0.04, "JST VH 2-pin | " + FER, "RESERVED, NOT FITTED", prot="Octopus heater fuse", fw="io:PA3")
    # ---- DC UPS reservation (deck_k Y6): space on CBP2 and an input position on FT2 (LOGIC + cold-module hold)
    C("W-P-70", "UPS", "24 V out", "+,-", "FT2", "UPS in (spare position)", "1,0V", "24V UPS reserve", 2, "SC_25", 6.0, FS + " | " + FER, "RESERVED, NOT FITTED: DC UPS (sizing in system.yaml rev_l)", prot="UPS own limit")
    # ---- mains: switched strip after KM6 (coil from SAFE_E, mirror in SR1's loop)
    C("W-M-05", "PSU_5V", "AC L/N", "L,N", "KM6", "1/L1, 3/L2", "L,N", "MAINS 230 V L/N to the strip contactor", 2, "MAINS_2X10", 2.0, FS + " | " + FS, f"{CP} only", prot="inlet T10A")
    C("W-M-06", "KM6", "2/T1, 4/T2", "L,N", "MAINS_STRIP", "L,N", "L,N", "MAINS 230 V L/N switched strip", 2, "MAINS_2X10", 2.0, FS + " | " + FS, f"{CP} only; panel-mount IEC C13 outlets", prot="inlet T10A")
    C("W-S-50", "XT", "SAFE_E", "4", "KM6", "A1", "A1", "24V SAFE_E strip contactor coil", 1, "SC_05", 0.15, FER + " | " + FS, "e-stop removes the mains strip (shaker, balance)", route="fixed", via=("BACK_BASE",), prot="F6")
    C("W-S-51", "KM6", "A2", "A2", "XT", "0V bus", "0V", "0V coil return KM6", 1, "SC_05", 0.15, FS + " | " + FER, "", route="fixed", via=("BACK_BASE",), prot="F6")
    C("W-S-52", "KM6", "21-22 mirror NC", "22", "XT", "XR", "1", "SAFETY feedback loop SR1", 1, "SC_05", 0.05, FS + " | " + FER, "back to the reset terminals", route="fixed", via=("BACK_BASE",), prot="SR1 internal")




# ================================ electrical order list ================================
STATIC_BOM = [
    # (category, item, spec, qty)
    ("supply", "24 V 600 W enclosed supply", "Mean Well LRS-600-24 or equal: 24 V 25 A, 215 x 115 x 30 mm, 230 V input, fan cooled, UKCA/CE, IEC 62368-1/61010-1 approved; output set to 24.0 V. SAME ITEM as bom/lh1_bom.csv B-24_V_600_W_supply (qty 2): order once", 2),
    ("mains", "IEC inlet module", "IEC 60320 C14, 10 A 250 V, with 2-pole rocker switch and 2-pole 5 x 20 mm fuse drawer, snap-in or screw panel mount, 6.3 mm tabs (Schurter DD11/KM01 class)", 1),
    ("mains", "mains fuse", "5 x 20 mm ceramic, time-lag T8A H 250 V (IEC 60127-2), for the inlet + 2 spares", 4),
    ("mains", "mains lead", "IEC C13 to local plug (UK BS 1363 with 10 A or 13 A fuse), 2 m, 1.0 mm2", 1),
    ("mains", "cable clamp / strain relief", "P-clips 6-8 mm nylon with M4 screw for the mains flex at inlet and PSUs", 4),
    ("safety", "THERM second-channel contactor KM5", "same part as the THERM contactors (mirror NC, DC-rated, poles in series), coil from the end of the TC-1 coil chain (SR1 33-34 -> thermostats -> watchdog)", 1),
    ("note", "plunger actuator purchase condition", "NEMA11 captive linear (in bom/lh1_bom.csv): 1 mm lead, >= 30 mm stroke, >= 60 N thrust at 10 mm/s and 24 V (40 N needed x 1.5, mechanical review M6), rated about 0.67 A; supplier thrust/speed curve required. All end-stop micro switches are wired NC (fail-safe): KW10/KW11/D2F must be changeover (C/NO/NC) types", 0),
    ("note", "cradle force switches", "D2F switches of the revision F cradle (disc-spring stack) are in bom/lh1_bom.csv: they must be changeover types so they can be wired NC (fail-safe)", 0),
    ("switch", "Z-up roller-lever micro switches (gold contacts)", "roller lever, changeover, 0.1 A gold-crossbar (micro-load) contacts, sealed type with factory lead where it fits: 3 x Z-up (carriages A, B, gripper axis) + 1 spare", 4),
    ("switch", "D2F-01 hinge-lever micro switch", "Omron D2F-01L class, changeover: TC-1 lid PARKED + 1 spare", 2),
    ("note", "Z home switches and couplers", "KW10 Z home switches are ROLLER-LEVER types on the ZUC-1 cam bars (rev G1) and the couplers are D20 x 20 (rev G): both in bom/lh1_bom.csv; KW10 must be changeover (wired NC)", 0),
    ("sensor", "through-beam pair (tip gate, bin full)", "separated emitter/receiver photoelectric pair, range >= 150 mm, aperture <= 1 mm (tip gate), response <= 1 ms, 12-24 V DC, receiver NPN open-collector LIGHT-ON, flying leads 2 m (rev K tip station: the fork is retired)", 2),
    ("controller", "passive driver-socket breakout", "stepstick-footprint breakout, direct pins to screw terminals, NO buffers or level shifters (socket 4 used as inputs)", 1),
    ("safety", "safety relay, e-stop (SR1)", "Pilz PNOZ s4, 24 V DC, 3 n/o 1 n/c, order no. 750104 (screw terminals): dual-channel with short detection, monitored manual start with feedback loop (manual 21396-EN-23)", 1),
    ("safety", "safety relay, door (SR2)", "Pilz PNOZ s4 750104, set to dual-channel safety gate with short detection, automatic start with feedback loop", 1),
    ("safety", "contactor with mirror contact (KM1-KM4)", "3-pole contactor, 24 V DC coil, main contacts DC-1 >= 16 A at 24 V per pole, 1 NC auxiliary as mirror contact (IEC 60947-4-1 Annex F), size S00, DIN rail; e.g. Siemens 3RT2016-1BB42 (3 NO + 1 NC)", 4),
    ("safety", "front auxiliary contact block", "1 NO + 1 NC (or 2 NO) front-mounted auxiliary block for KM1 and KM2 (SAFE_E chain), matching the contactor series (Siemens 3RH2911-1HA11 class)", 2),
    ("safety", "coil surge suppressor", "plug-in varistor or diode-varistor module for 24 V DC contactor coils, matching the contactor series", 4),
    ("safety", "e-stop push-button", "22 mm panel-mount mushroom 40 mm, red, latching twist-to-release, 2 NC positive-opening (IEC 60947-5-5) safety contacts + 1 NC auxiliary for the monitor (fail-safe), IP65, with a yellow 60 mm legend plate; contact stack <= 45 mm behind the 5 mm PC panel (reserved depth, rev D ESTOP_env)", 1),
    ("safety", "reset push-button", "22 mm panel-mount illuminated blue push-button, 1 NO, 24 V LED, for the 22.5 mm hole in ENC-FR (rev D)", 1),
    ("safety", "door safety sensor", "coded non-contact magnetic safety sensor + actuator, ISO 14119 type 4 low coding, 2 safety contacts (equivalent to the SR2 input) + 1 signalling contact that is CLOSED while the actuator is present (door closed), 2 m 6-core lead, compatible with the chosen safety relay (manufacturer's evaluation list); e.g. Schmersal BNS 260 / Pilz PSEN ma class", 1),
    ("distribution", "fuse terminal 5 x 20, LED blown indicator", "DIN-rail fuse terminal for 5 x 20 mm fuses, 6.3 A max, 24 V LED blown-fuse indicator (Phoenix UT 4-HESILED 24 (5X20) class); +3 for the per-motor gantry fuses F25-F27 (V8, 23 Nov 2026), fed from MOT_G (F21), +1 for F36 (TC-1 coil chain, rail E)", 11),
    ("distribution", "fuse terminal 6.3 x 32", "DIN-rail fuse terminal for 6.3 x 32 mm fuses, 16 A (Phoenix UT 6-HESILED 24 class) for F21, F4 (12.5 A, cold blocks C3 + F3) and F32 (16 A, TEC_TC feed)", 3),
    ("distribution", "fuse holder 10 x 38", "DIN-rail 1-pole fuse holder for 10 x 38 mm cartridges, 32 A, DC-rated (F2, F5)", 2),
    ("distribution", "fuses 5 x 20", "time-lag 5 x 20 mm, 250 V: F1 5 A, F6 2 A, F7 2 A, F22 6.3 A, F23 2 A, F31 5 A (lid heater), F25/F26/F27 T4A (X, Y, Y1 motors, V8), F36 T1A (TC-1 coil chain); one set fitted + one spare set", 22),
    ("distribution", "fuse 6.3 x 32", "6.3 x 32 mm slow-blow: F21 10 A, F4 12.5 A, F32 16 A, + 1 spare of each", 6),
    ("distribution", "fuses 10 x 38", "10 x 38 mm gG, DC-rated >= 32 V: F2 16 A, F5 25 A, + 1 spare of each", 4),
    ("distribution", "distribution block 12-way", "push-in distribution block 2.5 mm2 12 positions with DIN-rail adapter, 24 A (Phoenix PTFIX 12X2,5 class): 0V bus x 2, LOGIC, LOG7, MOT_G, SAFE/SAFE_E (colour-coded: blue 0 V, red 24 V)", 6),
    ("distribution", "SSR interface terminal TG_SW", "DIN-rail solid-state relay terminal, 24 V DC input, DC output >= 0.1 A at 24 V, LED (Finder 38.81 class, 24 V DC in / 24 V DC out): switches the tip-gate supply for the W3 supply test", 1),
    ("distribution", "feed-through terminal ZC / XL / XB / XK", "push-in DIN-rail feed-through terminals for the chain end ZC (x 3), the lid sensors XL (x 6), the dock lift slot sensor XK (x 3, ADR-014)", 12),
    ("distribution", "feed-through terminal 2.5 mm2", "push-in DIN-rail terminal 2.5 mm2 (XE e-stop x 7, XD door x 6, XR reset x 4, THERM_AUX x 2) with end plates and marker strips", 21),
    ("distribution", "DIN rail", "35 x 7.5 mm top-hat rail, slotted, cut to 290 mm (rails A and B) + end stops", 2),
    ("distribution", "DIN end stop", "screwless end clamp", 6),
    ("distribution", "jumper bars", "plug-in bridges for the fuse terminals and the feed-through terminals (2- and 5-position)", 4),
    ("controller", "main controller", "BigTreeTech Octopus Pro v1.1 (STM32H723), with on-board CAN transceiver; CAN cable/connector kit", 1),
    ("controller", "step/dir breakout", "stepstick-socket external-driver breakout with screw terminals (STEP, DIR, EN, GND) and a 5 V level shift, for Octopus sockets 0, 1, 3 (X/Y) and 2, 5 (TC-1 lid drivers)", 5),
    ("thermal", "lid stepper drivers", "external step/dir stepper driver, Leadshine DM542 class: supply >= 20-48 V DC (MOTION, via K_TU), 1.0 A peak setting available, 5 V logic inputs, enable input, screw terminals; DIP 3200 pulses/rev (TC-1 lid slide and clamp)", 2),
    ("controller", "head board", "BigTreeTech MMB CAN V2.0 (STM32G0B1, CAN + USB, XT30 2+2 input) with XT30 lead", 2),
    ("controller", "EZ5160 Pro driver", "BigTreeTech EZ5160 Pro (TMC5160, VM 8-48 V, SPI): 6 fitted (3 per MMB: Z, pusher, gripper Z / jaws) + 1 spare; read the sense-resistor marking", 7),
    ("controller", "dock lift driver (ADR-014)", "TMC5160 SPI stepstick for the main Octopus Pro socket 6 (BTT TMC5160 class, external MOSFETs, >= 2.5 A rms, VM >= 48 V: PURCHASE CONDITION, UNVERIFIED part), with heat sink; read the sense-resistor marking; DIAG pin jumper 6 stays removed", 1),
    ("note", "dock lift motor (ADR-014)", "NEMA17 48 mm with integrated Tr8x2 screw and anti-backlash nut (DLB-1) is a CAD bought part (head report dock_bay.dock_lift): order it there; its JST-PH 6 socket takes the 4 x 0.5 mm2 screened lead W-I-30", 0),
    ("sensor", "dock lift DOWN slot sensor (ADR-014)", "slot-type photo-microsensor 5-24 V DC NPN, light-ON/dark-ON selectable, set so the output CONDUCTS with the bar's flag in the slot, with the maker's pre-wired lead 1-2 m (Omron EE-SX67x + EE-1006 class, same as the TC-1 lid slot sensors) + 1 spare", 2),
    ("sensor", "latch ladder resistors (ADR-014)", "metal-film 0.25 W 1 % axial: 1.0 k, 22 k, 4.7 k, 2.0 k per carriage (CAD latch.SENSE; EMPTY 2.0 k, was 1.5 k) in lever splices on the latch switch bracket LSB-L (no soldering) x 2 carriages + 2 sets spare", 4),
    ("sensor", "latch ladder lever splices (ADR-014)", "Wago 221-415 (5-way) x 2 + 221-412 (2-way) x 2 per carriage, on the CAD ladder block LLB-L (switch leads, resistor leads, way and return jumpers)", 8),
    ("controller", "io board", "second BigTreeTech Octopus Pro v1.1 (same part as the main controller): Klipper MCU 'io' on USB; ALL EIGHT DIAG jumpers removed", 1),
    ("controller", "passive breakouts (io)", "stepstick-footprint passive breakout (socket 4, 5 of the io board) + two passive EXP1/EXP2 breakouts with screw terminals and 10-way IDC ribbons", 4),
    ("supply", "24 V 600 W enclosed supply PSU_AUX", "third Mean Well LRS-600-24 class (THERM_AUX + MODULE, rev L)", 1),
    ("thermal", "TEC controller TEC_TUBE", "Meerstetter TEC-1166-TB (2 channels, +/-5 A) with 2 x PLG-34434; CHF 619 (13 Nov 2026 price).  TEC_WARM (the same part) is RESERVED in every size (37 C module at G0): NOT ordered (saves CHF 619)", 1),
    ("thermal", "thermal-module leads", "pre-made Molex Mini-Fit Jr 8-pin lead 1.5 m with flying ends, 6 x 0.75 mm2 used (Peltier + two NTC pairs): B2, G4, A3, A4", 4),
    ("thermal", "thermostat chain leads", "pre-made Molex Mini-Fit Jr 2-pin leads 0.5-1.5 m (module chain in/out: 5) + 1 bridging lead for an empty thermal place", 6),
    ("thermal", "module zone fan connector", "Molex Micro-Fit 3.0 2-pin keyed panel receptacle (CAD CTB-1_fan_connector beside A3) + pre-made 2-pin lead to XM F41 through gland G1; a different family from the chain leads on purpose", 1),
    ("sensor", "bin-full beam junction", "Wago 221 lever splices (4 ways) at the chute outlet joining the beam pair's moulded leads to the 4-core lead (CAD tip_bin_full_beam)", 1),
    ("distribution", "place leads", "pre-made Molex Mini-Fit Jr 6-pin lead 2 m with flying ends, 6 x 0.5 mm2 (one per module place: A3, B2, G4, G0) + USB A-B extension to a panel USB-B receptacle; D3, D4 come with the M->L kit", 4),
    ("distribution", "place fuse terminals F41-F48", "5 x 20 fuse terminals with LED, 8 positions (sized for L: F45/F46 for D3/D4, F47/F48 spare)", 8),
    ("distribution", "place fuses", "T3.15A 5 x 20 for F41-F44 + 2 spare (D3/D4 fuses in the M->L kit)", 6),
    ("safety", "G0 loop-back plug", "bought Mini-Fit Jr 6-pin plug with SAFE A and SAFE B looped to 0 V (pins 3, 4 to 2): G0 reserved module place", 1),
    ("distribution", "fuse terminals FT2", "F4 10 x 38 DC-rated 20 A holder; F9 4 A and F33-F35 8 A (6.3 x 32)", 5),
    ("safety", "mains strip contactor KM6 + IEC outlets", "2-pole contactor AC-1 >= 10 A 230 V, 24 V DC coil, mirror NC aux (Annex F) + suppressor; 2 x panel IEC C13 outlets: COMPETENT PERSON", 1),
    ("distribution", "DIN rail E + end stops", "35 mm rail 135 mm on CBP2 for K_TU, K_CH, WDOG, LID_SSR, TG_SW, F31 (moved into the sealed compartment: the TEP IP65 box is dropped)", 1),
    ("switch", "sealed micro switches with factory leads", "IP67 subminiature switch, changeover, factory lead 0.5-1 m, envelope <= 10.2 x 6.4 x 20 mm (CAD CR-L7 class, UNVERIFIED code): head 12 (pusher retracted x 2, contact x 2, latch LOCK x 2 and EMPTY x 2 (ADR-014 three-state latch), force x 2, gripper jaws-open + gripped), dock bays 8, dock lift bar OPEN height 1 (ADR-014), drawer 1, shutter H1 1, + 2 spare (H2 switch with the S->M kit)", 25),
    ("sensor", "Hall switches", "6.5 mm cylindrical LEADED unipolar Hall switch, open collector, 4.5-24 V, smooth body for a reamed hole (CAD CR-L7, UNVERIFIED part): 4 per carriage + 2 spare", 10),
    ("sensor", "airflow switch", "differential-pressure switch 20-300 Pa adjustable, NO on rising pressure, factory lead (UNVERIFIED)", 1),
    ("sensor", "waste level switches", "capacitive non-contact level switch 24 V, NPN output selectable to conduct below the level (UNVERIFIED)", 2),
    ("fans", "extraction fan", "24 V EC (brushless) inline duct fan 30 m3/h class, 100 mm spigot (UNVERIFIED part)", 1),
    ("note", "USB load-cell interface", "4-channel 24-bit USB bridge interface (Phidgets 1046 class, UNVERIFIED) for WN-1 at D4: L only, in the M->L kit (out/kits_electrical.csv)", 0),
    ("motor", "carriage plunger pusher (ADR-014 #5)", "Nanotec LGA351S12-A-UIAP-038 captive NEMA 14 linear actuator with integrated guide (0.6096 mm lead, 38.1 mm stroke, 1.2 A / 1.85 ohm / 2.46 mH winding, JST B6B-EH-A) + maker's lead ZK-JST-EHR-6-0.5M-S; EUR 127 each (21 Nov 2026, 5 in stock: order the machine's 2 with the rig's 2 together); driven at 0.7 A ONLY (system.yaml plunger_motor: the 'winding <= 1.0 A' condition is replaced by the FFC continuous-rating condition)", 2),
    ("cable", "head flat cables", "high-flex FFC 450 mm: 20-way 1.0 mm (carriage A) x 1 + 1 spare, 20-way 0.5 mm (carriage B: CONTINUOUS rating >= 0.44 A per conductor at 40 C, or >= 0.62 A if only a peak rating is published: ADR-014 plunger_motor purchase condition) x 1 + 1 spare, 12-way 0.5 mm (gripper) x 1 + 1 spare; ZIF-to-terminal adapters: 1.0 mm 20-way x 2 (+1), 0.5 mm 20-way x 2 (+1), 0.5 mm 12-way x 2 (+1), contacts >= 0.5 A (0.5 mm) / 1.0 A (1.0 mm)", 1),
    ("note", "cable chains", "X and Y chains (igus E2 medium class 38 x 18 R55) are CAD bought parts (cad machine_family parts_S: XCH-F, YCH-F): order them there, not here", 0),
    ("controller", "head supply/CAN splice", "WAGO 221-413 lever connectors (V+, 0V, CAN-H, CAN-L) in a printed strain-relief cover on the head tray + 1 spare", 5),
    ("controller", "XT30(2+2) toolhead lead", "pre-made XT30(2+2) lead, 2 x 18 AWG + 2 x 24 AWG twisted, about 0.15 m, flying ends (BTT EBB/MMB kit class)", 2),
    ("controller", "closed-loop motor X", "NEMA17 integrated closed-loop stepper + driver, >= 0.65 N m holding, body <= 60 mm, 24 V supply, step/dir/enable inputs compatible with 3.3 V (or via a 5 V breakout); alarm output open-collector, programmable to CONDUCT while healthy (fail-safe). SAME ITEM as bom/lh1_bom.csv B-NEMA17_closed-loop_stepper: order once", 1),
    ("controller", "Z motor (purchase condition)", "NEMA17 48 mm class (CAD rev G5), 1.8 deg, >= 0.52 N m holding at <= 1.7 A rated phase current, plain 5 mm shaft, 4-wire bipolar, DETACHABLE JST-PH 6 lead socket; driven at 0.9 A rms (force strokes 1.3 A, window 1.22-1.35 A: system.yaml z_drive) by the head board's EZ5160 Pro; ordered on bom/lh1_bom.csv (head): order once, check the motor's datasheet holding torque and rated current against z_drive before buying", 4),
    ("controller", "closed-loop motor Y", "NEMA17 integrated closed-loop stepper + driver, >= 0.45 N m holding, 24 V, step/dir/enable, alarm output programmable to conduct while healthy. SAME ITEM as bom/lh1_bom.csv B-NEMA17_closed-loop_stepper: order once", 2),
    ("host", "host computer", "Raspberry Pi 4 Model B 4 GB (or Pi 5 4 GB) with heatsink case, 32 GB A2 microSD or USB SSD", 1),
    ("host", "host 5 V mains supply", "DIN-rail enclosed 85-264 VAC -> 5 V 10 A (50 W) supply, adjustable to 5.2 V, IEC 62368-1 / 61010-1 approved, touch-safe terminals (Mean Well MDR-60-5 class); fed from the switched, fused inlet", 1),
    ("host", "USB hub", "USB 2.0 10-port hub with external 5 V input on screw terminals, metal case (UNVERIFIED class): HUB1 (bay), HUB2 (CBP2)", 2),
    ("host", "cameras VC-1, VC-2, VC-3", "USB UVC cameras per ADR-004 / rev K: VC-1 10 mm module with ring light, VC-2 38 x 38 board + M12 f 8 mm, VC-3 board camera with 120 deg lens", 3),
    ("thermal", "TC-1 block TEC controller", "Meerstetter TEC-1163-TB (1 channel, +/-25 A / 0-56 V, 11.5-63 V in, USB/RS485, MeCom protocol), with PLG-34434 terminal-block plug; CHF 485 (meerstetter.ch, 13 Nov 2026)", 1),
    ("thermal", "cold-block TEC controller", "Meerstetter TEC-1166-TB (2 channels, +/-5 A / 0-56 V, 11.5-63 V in, USB), with 2 x PLG-34434; CHF 619 (meerstetter.ch, 13 Nov 2026)", 1),
    ("thermal", "THERM contactors TC_RELAY, TB_RELAY", "contactor with TWO mirror NC auxiliaries (IEC 60947-4-1 Annex F; one for SR1's loop, one read by the Octopus), DC-rated >= 40 A make/break at 32 V DC with poles in series (e.g. Siemens 3RT2026 class + auxiliary block; verify DC-1 table), 24 V DC coil <= 0.2 A with plug-in suppressor", 2),
    ("thermal", "lid heater SSR", "DC solid-state relay, DIN rail: Sensata Crydom DR10D10 (1-100 V DC, 10 A, 4-32 V DC control) or equal", 1),
    ("thermal", "lid-heater fuse terminal F31", "DIN-rail fuse terminal 5 x 20 with 5 A time-lag fuse (lid heater, THERM side of LID_SSR)", 1),
    ("safety", "gate relay K_TU", "Finder 7S.12.9.024.5110 relay module with forcibly guided contacts (EN 61810-3 type A), 1 NO + 1 NC, 6 A, 24 V DC coil 38.2 mA, DIN rail", 1),
    ("safety", "chain-status relay K_CH", "Finder 38.51.7.024.0050 class DIN interface relay, 1 CO 6 A, 24 V DC sensitive coil (about 10 mA)", 1),
    ("safety", "host-loss watchdog relay WDOG", "Red Lion 6HBWDOG1 heartbeat watchdog relay (12-24 V DC, heartbeat 1-50 Hz, timeout 1-200 s, form C 2 A at 30 V DC, DIN); price not seen", 1),
    ("sensor", "TC-1 lid through-beam pair", "subminiature through-beam photoelectric sensor pair, PNP LIGHT-ON receiver output >= 50 mA, 12-24 V DC, response <= 1 ms, pre-wired 2 m leads (Omron E3T-ST13 class: CHECK the output type and current on the part bought)", 1),
    ("sensor", "TC-1 lid slot sensors", "slot-type photo-microsensor 5-24 V DC NPN, light-ON/dark-ON selectable, with the maker's pre-wired connector lead 1-2 m (Omron EE-SX67x + EE-1006 class)", 2),
    ("thermal", "NTC 10 k (cold-block sinks)", "TDK/EPCOS B57861S0103F040 10 k 1 % B25/100 3988 K, for TEC_COLD aux inputs (C3, F3)", 2),
    ("switch", "TC-1 clamp SEALED micro switch", "D2F-class subminiature micro switch with lever at the TC-1 presser hard stop (Octopus PF7)", 1),
    ("connectors", "pre-made leads", "bought pre-crimped leads: JST-XH 2/3/4/5-pin 0.3-3 m with flying ends (end stops, sensors), JST-PH6-to-bare 3D-printer motor leads, XT30(2+2), fan extension pigtails (ADR-003 #3); lengths from wiring.csv length_mm_estimate, never shortened by re-crimping", 1),
    ("fans", "compartment intake fans", "92 x 92 x 25 mm 24 V DC fan, about 40 CFM free air, <= 0.15 A, ball bearing, with finger guard and washable filter (CAD CBX-F_intake_fan_92mm_1/2, right wall); class figures UNVERIFIED: the CAD airflow needs >= 40 CFM each before the 0.6 derate", 2),
    ("note", "items NOT repeated here", "motors (Z, gripper, plungers, magnet lift, TC-1), Z/X/Y micro switches, gripped D2F, Peltiers, fans (TC-1, thermal block), thermistors, KSD/thermal cut-offs, TC-1 H-bridge (TCB-1 BOM), drag chains (X, Y 10x10 R18; TC-1 mini chain) are in bom/lh1_bom.csv. Added here as NEW switches (CAD requests CR-04/05): D2F-class micro switch with lever, 4 pcs", 0),

]


# ADR-003 #3 joint classes (tool-made versus tool-free ends) and the hand-crimped families (none left after ADR-003)
# are harnessgen's defaults (harnessgen.counts.DEFAULT_JOINT_TYPES / DEFAULT_HAND_CRIMP).


def write_bom(rows, path=None, static=None, extra=()):
    # cable to buy per type; plug-in / device / motor / XT30 leads and RESERVED (not fitted) rows are not cut
    by_type = cable_totals(rows, TYPES, no_cable_types=("PLUG_IN", "DEVICE_LEAD", "MOTOR_LEAD", "XT30_LEAD"), skip_note="RESERVED")
    out = []
    for cat, item, spec, qty in (STATIC_BOM if static is None else static):
        if qty or cat == "note":
            out.append({"category": cat, "item": item, "quantity": qty, "unit": "pcs", "specification": spec})
    n_plane = sum(1 for r in rows if "connector plane" in r["notes"] and "RESERVED" not in r["notes"])
    out.append({"category": "connectors", "item": "connector-plane break (CPL-F)", "quantity": n_plane, "unit": "pairs",
                "specification": "one keyed in-line plug/socket pair per cable crossing the connector plane CPL-F (wiring.csv notes 'connector plane'), "
                                 "bought as pre-made leads in the cable's own family (Mini-Fit Jr for power/places, JST-XH for switches and step/dir, USB for USB); "
                                 "every pair on one cable distinct in key or pin count from its neighbours (ADR-003 #3)"})
    for ct, mm in sorted(by_type.items()):
        m = math.ceil(mm * 1.2 / 1000.0)
        t = TYPES[ct]
        colour = ""
        if t.get("single_core") and ct.startswith(("SC_", "SC_HF")):
            colour = "; buy in red (+24 V), black (0 V) and for signals white/grey/violet in proportion to the wiring table"
        out.append({"category": "cable", "item": ct, "quantity": m, "unit": "m",
                    "specification": f"{t['desc']}; total {mm / 1000:.1f} m in wiring.csv + 20 % spare{colour}"})
    housings, crimps, ferrules = connector_counts(rows)
    for (fam, pins), q in sorted(housings.items(), key=lambda kv: (kv[0][0], kv[0][1] or 0)):
        spec = f"{fam} housing, {pins} positions (+20 % spare)" if pins else f"{fam} (+20 % spare)"
        if fam.startswith("insulated receptacle"):
            spec = f"{fam} fully insulated female, for 0.5-1.5 mm2 (2.5-4 mm2 for THERM), +20 % spare"
        if fam.startswith("ring lug"):
            spec = f"{fam} insulated ring terminal for 1.5-2.5 mm2 (6 mm2 for the braid), +20 % spare"
        out.append({"category": "connectors", "item": f"{fam}{'' if pins is None else f' {pins}-pin'}", "quantity": math.ceil(q * 1.2),
                    "unit": "pcs", "specification": spec})
    for (fam, g), q in sorted(crimps.items()):
        out.append({"category": "connectors", "item": f"{fam} crimp contact for {g} mm2", "quantity": math.ceil(q * 1.3), "unit": "pcs",
                    "specification": f"{fam} female crimp terminal suited to {g} mm2 conductors (+30 % spare: crimping losses)"})
    for g, q in sorted(ferrules.items()):
        out.append({"category": "connectors", "item": f"bootlace ferrule {g} mm2", "quantity": math.ceil(q * 1.3 / 10) * 10, "unit": "pcs",
                    "specification": f"insulated bootlace ferrule DIN 46228-4 for {g} mm2 (twin-wire ferrules 2 x 1.0 mm2: 4 pcs extra)"})
    out += [

        {"category": "sleeving", "item": "braided PET expandable sleeving 6 mm", "quantity": 10, "unit": "m", "specification": "for static bundles along the frame"},
        {"category": "sleeving", "item": "braided PET expandable sleeving 10 mm", "quantity": 5, "unit": "m", "specification": "trunk bay -> Y chain and under-deck"},
        {"category": "sleeving", "item": "silicone glass-fibre sleeving 3 mm", "quantity": 2, "unit": "m", "specification": "high-temperature, near the TC-1 block and lid heater"},
        {"category": "sleeving", "item": "heat-shrink assortment 3:1 adhesive", "quantity": 1, "unit": "kit", "specification": "1.6-12 mm, adhesive-lined"},
        {"category": "fixing", "item": "cable-tie mounts and ties", "quantity": 1, "unit": "kit", "specification": "adhesive/screw cable-tie bases (M4), UV-stable ties 2.5 x 100 and 3.6 x 200"},
        {"category": "fixing", "item": "wire duct", "quantity": 2, "unit": "m", "specification": "slotted wiring duct 25 x 25 mm for the control bay (cut to rails)"},
        {"category": "fixing", "item": "cable glands / grommets", "quantity": 6, "unit": "pcs", "specification": "M16/M20 glands or edge grommets where cables pass panels and the bay lid"},
        {"category": "earthing", "item": "PE stud kit", "quantity": 1, "unit": "kit", "specification": "M6 stud/bolt, serrated lock washers (anodising-piercing), nuts, PE label (IEC 60417-5019)"},
        {"category": "labels", "item": "wire and terminal markers", "quantity": 1, "unit": "kit", "specification": "heat-shrink printed markers with the cable_id at both ends; terminal marker strips"},
    ]
    out += list(extra)
    write_rows(out, path or (HERE / "electrical_bom.csv"), columns=["category", "item", "quantity", "unit", "specification"])
    return out


def sync_control_bay_from_cad():
    """Copy the control-bay device volume (height and start x) from the frame CAD into control_bay.yaml, so the
    layout follows every CAD revision.  check.py fails if the two ever differ."""
    import importlib.util
    import re
    root = HERE.parents[1]
    src = (root / "cad/lh1/frame_gantry/build.py").read_text()
    m = re.search(r'"RES_control_bay_devices", box\(\(([\d.]+), (-?[\d.]+), ZB1 \+ 3\.0\), \(([\d.]+), ([\d.]+), -P\.DECK_T(?: ([+-]) ([\d.]+))?\)\)', src)
    if not m:
        print("control bay: RES_control_bay_devices not found in the frame CAD; control_bay.yaml left as is")
        return
    spec = importlib.util.spec_from_file_location("p_sync", root / "cad/lh1/params.py")
    P = importlib.util.module_from_spec(spec); spec.loader.exec_module(P)
    top = -P.DECK_T + ((-1 if m.group(5) == "-" else 1) * float(m.group(6)) if m.group(5) else 0.0)
    h = round(top - (P.BASE_Z + 3.0), 1)
    x0 = float(m.group(1))
    path = HERE / "control_bay.yaml"
    t = path.read_text()
    t = re.sub(r"height_available_mm: [\d.]+(\s+# SYNCED)", f"height_available_mm: {h:g}\\1", t, count=1)
    t = re.sub(r"usable_x0: [\d.]+(\s+# SYNCED)", f"usable_x0: {x0}\\1", t, count=1)
    path.write_text(t)
    print(f"control bay synced from CAD: height {h:g} mm, devices from x {x0}")


def rows_for(size):
    set_size(size)
    ROWS.clear()
    PLANE.clear()
    define_rows()
    return [dict(r) for r in ROWS], dict(PLANE)


def kits(per_size, planes):
    """electrical content of the upgrade kits S -> M and M -> L, generated from the cable rows of the two sizes"""
    out = {}
    for a, b in (("S", "M"), ("M", "L")):
        ra = {r["cable_id"]: r for r in per_size[a]}
        rb = {r["cable_id"]: r for r in per_size[b]}
        added = [rb[c] for c in rb if c not in ra and "RESERVED" not in rb[c]["notes"]]
        ext = [rb[c] for c in rb if c in ra and planes[b].get(c) and "RESERVED" not in rb[c]["notes"] and rb[c]["cable_type"] != "PLUG_IN"]
        devs = sorted(d for d, v in SYS["devices"].items() if b in v.get("sizes", SIZES) and a not in v.get("sizes", SIZES))
        items = []
        for r in added:
            items.append({"kind": "added lead", "cable_id": r["cable_id"], "from": r["from_device"], "to": r["to_device"], "signal": r["signal"],
                          "cable_type": r["cable_type"], "conductors": r["conductors"], "length_mm": r["length_mm_estimate"], "ends": r["connector_part"]})
        for r in ext:
            items.append({"kind": "extension lead 110 mm (keyed, inserted at the connector plane CPL-F)", "cable_id": r["cable_id"], "from": r["from_device"],
                          "to": r["to_device"], "signal": r["signal"], "cable_type": r["cable_type"], "conductors": r["conductors"], "length_mm": 110,
                          "ends": "keyed plug + socket of the plane connector"})
        for d in devs:
            items.append({"kind": "added device / module", "device": d, "desc": SYS["devices"][d]["desc"]})
        out[f"{a}_to_{b}"] = {"items": items, "counts": {"added_leads": len(added), "extension_leads": len(ext), "added_devices": len(devs)},
                              "note": "Y-chain cables are cut to the L length in every size (surplus coiled as a service loop at the floor gland): no kit content; the Y chain itself gets links (CAD kit)"}
    return out


def main():
    per_size, planes = {}, {}
    for s in SIZES:
        per_size[s], planes[s] = rows_for(s)
    # cables through the Y chain (and the X chain, which is the same in every size) are cut ONCE for L
    longest = {}
    for s in SIZES:
        for r in per_size[s]:
            if "y_chain" in r["route"]:
                longest[r["cable_id"]] = max(longest.get(r["cable_id"], 0), int(r["length_mm_estimate"]))
    for s in SIZES:
        for r in per_size[s]:
            if r["cable_id"] in longest and int(r["length_mm_estimate"]) < longest[r["cable_id"]]:
                extra = longest[r["cable_id"]] - int(r["length_mm_estimate"])
                r["notes"] = (r["notes"] + "; " if r["notes"] else "") + f"cut for L (+{extra} mm service loop at the floor gland in {s})"
                r["length_mm_estimate"] = longest[r["cable_id"]]
    rows = per_size[BUILD]
    (HERE / "out").mkdir(exist_ok=True)
    write_rows(rows, HERE / "wiring.csv", columns=COLUMNS)
    for s in SIZES:
        if s != BUILD:
            write_rows(per_size[s], HERE / "out" / f"wiring_{s}.csv", columns=COLUMNS)
    bom = write_bom(rows)
    jc = joint_counts(rows)
    jc_all = {s: joint_counts(per_size[s]) for s in SIZES}
    (HERE / "out" / "joint_counts.json").write_text(json.dumps(dict(jc, size=BUILD, by_size={s: v.get("tool-made joints, total") for s, v in jc_all.items()}), indent=1))
    k = kits(per_size, planes)
    (HERE / "out" / "kits_electrical.json").write_text(json.dumps(k, indent=1))
    flat = [dict(kit=kn, **{kk: str(vv) for kk, vv in it.items()}) for kn, kv in k.items() for it in kv["items"]]
    write_rows(flat, HERE / "out" / "kits_electrical.csv", columns=["kit", "kind", "cable_id", "device", "from", "to", "signal", "cable_type", "conductors", "length_mm", "ends", "desc"])
    print(f"size {BUILD}: wiring.csv {len(rows)} cables; electrical_bom.csv {len(bom)} lines; tool-made joints {jc['tool-made joints, total']}; "
          f"M {len(per_size['M'])} / L {len(per_size['L'])} cables; kits " + ", ".join(f"{kn}: {kv['counts']}" for kn, kv in k.items()))


# ================================ reduced build profiles (MVP) ================================
# A profile (profile_<name>.yaml) is the build size with devices LEFT OUT.  Its rows are the size's rows filtered: a row stays
# when both ends are fitted (or it is listed in keep_cables: it ends on a bridge part), unless listed in absent_cables.  Rows
# are copied, never re-made: same cable id, ends, connector positions, length and fuse, so every later stage only adds.
# Run: .venv/bin/python electronics/lh1/build_harness.py --profile MVP  ->  electronics/lh1/out/mvp/
MVP_BOM = {   # starter STATIC_BOM item (prefix) -> MVP quantity [, specification replacing the starter's]
    "24 V 600 W enclosed supply PSU_AUX": 0, "24 V 600 W enclosed supply": (1, "Mean Well LRS-600-24 or equal (PSU_MAIN only in the MVP: PSU_TC and PSU_AUX come with the thermocycler / deck-module stages; their mains loop is bridged by terminal blocks, see 'MVP bridge parts')"),
    "cable clamp / strain relief": 3, "THERM second-channel contactor KM5": 0,
    "Z-up roller-lever micro switches": (2, "roller lever, changeover, gold micro-load contacts, sealed with factory lead: Z_UP_A + 1 spare (A and gripper switches with their stages; their chain positions are lever-splice bridges)"),
    "D2F-01 hinge-lever micro switch": 0, "front auxiliary contact block": 0,
    "fuse terminal 5 x 20, LED blown indicator": (7, "DIN fuse terminal 5 x 20 with LED: F1, F6, F7, F22, F25, F26, F27 (F8, F23, F24, F36 positions left empty on rail B / E)"),
    "fuse terminal 6.3 x 32": (1, "DIN fuse terminal 6.3 x 32: F21 (F4, F32 with their stages)"),
    "fuse holder 10 x 38": (1, "DIN 10 x 38 DC-rated holder: F2 (F5 with the thermocycler)"),
    "fuses 5 x 20": (14, "time-lag 5 x 20: F1 5 A, F6 2 A, F7 2 A, F22 6.3 A, F25/F26/F27 T4A: one set + one spare set"),
    "fuse 6.3 x 32": (2, "F21 10 A + 1 spare"), "fuses 10 x 38": (2, "F2 16 A + 1 spare"),
    "feed-through terminal ZC / XL / XB / XK": (3, "push-in feed-through terminals for the chain end ZC x 3 (XL, XK with their stages)"),
    "feed-through terminal 2.5 mm2": (17, "push-in 2.5 mm2 terminals XE x 7, XD x 6, XR x 4 with end plates and markers (THERM_AUX x 2 with the modules)"),
    "step/dir breakout": (3, "stepstick-socket breakout with screw terminals and 5 V level shift for main sockets 0, 1, 3 (X, Y, Y1)"),
    "lid stepper drivers": 0, "head board": (1, "BigTreeTech MMB CAN V2.0 (MMB_A) with XT30 lead; its 120 ohm CAN jumper ON in the MVP (bus end)"),
    "EZ5160 Pro driver": (3, "BigTreeTech EZ5160 Pro: 2 fitted on MMB_A (EZ1 Z_A, EZ2 pusher A; EZ3 gripper Z left empty) + 1 spare"),
    "dock lift driver": 0, "dock lift DOWN slot sensor": 0, "latch ladder resistors": 0, "latch ladder lever splices": 0,
    "passive breakouts (io)": (1, "stepstick-footprint passive breakout for io socket 5 (XDB5: compartment fan RD); XDB4 with the liquid waste, EXP breakouts with the modules"),
    "TEC controller TEC_TUBE": 0, "thermal-module leads": 0, "thermostat chain leads": 0, "module zone fan connector": 0,
    "place leads": 0, "place fuse terminals F41-F48": 0, "place fuses": 0, "G0 loop-back plug": 0, "fuse terminals FT2": 0,
    "mains strip contactor KM6": 0,
    "DIN rail E + end stops": (1, "35 mm rail 135 mm on CBP2 for K_CH only in the MVP (K_TU, WDOG, LID_SSR, F31 later; TG_SW sits on rail B)"),
    "sealed micro switches with factory leads": (6, "IP67 subminiature changeover with factory lead: pusher retracted A, force A, drawer + 2 spare (contact A is a D2F, bom/lh1_bom.csv)"),
    "Hall switches": 0, "airflow switch": 0, "extraction fan": 0, "waste level switches": 0,
    "carriage plunger pusher": (1, "Nanotec LGA351S12-A-UIAP-038 + maker's lead, carriage A only, run at 0.7 A"),
    "head flat cables": (1, "high-flex FFC 450 mm 20-way 1.0 mm (carriage A: continuous rating >= 0.875 A per conductor, ffc_check verdict_A) x 1 + 1 spare; 1.0 mm 20-way ZIF-to-terminal adapters x 2 (+1)"),
    "XT30(2+2) toolhead lead": 1,
    "Z motor (purchase condition)": (2, "NEMA17 48 mm Z motor (z_drive purchase condition): Z_A + 1 spare"),
    "USB hub": 0, "cameras VC-1, VC-2, VC-3": 0, "TC-1 block TEC controller": 0, "cold-block TEC controller": 0,
    "THERM contactors TC_RELAY, TB_RELAY": 0, "lid heater SSR": 0, "lid-heater fuse terminal F31": 0, "gate relay K_TU": 0,
    "host-loss watchdog relay WDOG": 0, "TC-1 lid through-beam pair": 0, "TC-1 lid slot sensors": 0, "NTC 10 k (cold-block sinks)": 0,
    "TC-1 clamp SEALED micro switch": 0, "compartment intake fans": (1, "ONE 92 mm 24 V fan with RD output, guard and filter at the CAD fan-1 hole; the fan-2 hole gets a blanking plate (mechanical owner)"),
    "dock lift motor (ADR-014)": None, "USB load-cell interface": None,
}


def load_profile(name):
    p = yaml.safe_load((HERE / f"profile_{name}.yaml").read_text())
    if p["guard"] != "door":
        raise SystemExit(f"profile {name}: the starter's door channel is fitted in every build (guard: door), not {p['guard']}")
    return p


def profile_rows(prof, base_rows):
    absent = set(prof["absent_devices"])
    out = []
    for r in base_rows:
        if r["cable_id"] in prof["absent_cables"] or "RESERVED" in r["notes"]:
            continue
        ends_ok = r["from_device"] not in absent and r["to_device"] not in absent
        if ends_ok or r["cable_id"] in prof["keep_cables"]:
            rr = dict(r)
            if not ends_ok:
                rr["notes"] = (rr["notes"] + "; " if rr["notes"] else "") + f"{prof['profile']}: ends on a bridge part at the {prof['keep_cables'][r['cable_id']]} position (profile bridges)"
            out.append(rr)
    return out


def bom_for_profile(prof):
    static = []
    for cat, item, spec, qty in STATIC_BOM:
        key = next((k for k in sorted(MVP_BOM, key=len, reverse=True) if item.startswith(k)), None)
        if key is None:
            static.append((cat, item, spec, qty))
            continue
        v = MVP_BOM[key]
        if v is None:
            continue
        q, sp = (v if isinstance(v, tuple) else (v, spec))
        static.append((cat, item, sp, q))
    extra = [{"category": "bridge", "item": f"MVP bridge part at {b['at']}", "quantity": 1, "unit": "pcs",
              "specification": b["part"] + ("; COMPETENT PERSON" if b["competent_person"] else "") + f"; removed at stage {b['removed_at']}"} for b in prof["bridges"]]
    return static, extra


def _budget(prof, cab):
    loads = {l["id"]: l for l in prof["loads"]}
    rails, sups, brs = {}, {}, {}
    for rail in prof["rails"]:
        spec = SYS["rails"][rail]
        a = sum(l["a"] for l in prof["loads"] if l["rail"] == rail)
        rails[rail] = {"load_a": round(a, 2), "fuse": spec["fuse"], "limit_a": spec["fuse_a"], "pct": round(100 * a / spec["fuse_a"])}
        sups[spec["supply"]] = sups.get(spec["supply"], 0.0) + a
    for br, ids in prof["branches"].items():
        a = sum(loads[i]["a"] for i in ids)
        fa = SYS["branches"][br]["fuse_a"]
        brs[br] = {"load_a": round(a, 2), "fuse_a": fa, "pct": round(100 * a / fa), "loads": ids}
    sp = {k: {"load_a": round(v, 2), "a_max": SYS["supplies"][k]["a_max"], "pct": round(100 * v / SYS["supplies"][k]["a_max"]),
              "margin_rule_77pct": v <= SYS["supplies"][k]["a_max"] / 1.3 + 1e-9} for k, v in sups.items()}
    main_w = sups.get("PSU_MAIN", 0) * 24 / 0.9
    v5_w = sups.get("PSU_5V", 0) * 5.2 / 0.85
    mains_a = (main_w + v5_w) / 230.0
    items = {it["id"]: it for it in cab["items"]}
    heat = {k: prof["heat_w"][k]["w"] for k in ("PSU_MAIN", "PSU_5V")}
    for k in prof["heat_w"]["others"]:
        heat[k] = items[k]["heat_w"]
    heat["RAIL_E (K_CH)"] = prof["heat_w"]["rail_e_w"]
    hw = sum(heat.values())
    fan = cab["compartment"]["intake_fan"]
    one = fan["one_failed_cfm"]                    # one fan through the grilles (CAD airflow block)
    need = hw / (1.2 * 1005 * 10.0) * 2118.88
    rise = hw / (1.2 * 1005 * one / 2118.88)
    return {"profile": prof["profile"], "guard": prof["guard"], "rails": rails, "branches": brs, "supplies": sp,
            "mains": {"worst_coincident_a_at_230V": round(mains_a, 2), "inlet_fuse": "T10A H", "pct": round(10 * mains_a),
                      "basis": "PSU_MAIN load x 24 V / 0.9 + PSU_5V load x 5.2 V / 0.85, at 230 V, power factor ~1 (LRS-600 PFC: UNVERIFIED); one LRS-600 cold-start inrush instead of three"},
            "compartment": {"heat_w": round(hw, 1), "items_w": heat, "fans_fitted": 1, "needed_cfm_10K": round(need, 1),
                            "one_fan_cfm_through_grilles": one, "rise_K_one_fan": round(rise, 1), "ratio": round(one / need, 2),
                            "rule": "one fan must give >= 1.5 x the air for a 10 K rise (check.py COMPARTMENT_AIRFLOW rule); the second fan is added with PSU_AUX (deck modules stage)",
                            "fan_failed": "no forced air: the sheet box (about 1.1 m2) sheds about 5 W/K by natural convection: about 9 K above room air at this heat (ESTIMATE); the fan's RD output (io PF0) reports the failure"}}


def _control_bay_profile(prof, cab):
    absent = set(prof["absent_devices"])
    fit = {"PSU_TC": "PSU_TC", "PSU_AUX": "PSU_AUX", "KM5": "KM5", "TC_RELAY": "TC_RELAY", "TB_RELAY": "TB_RELAY", "KM6": "KM6", "FT2": "FT2", "XM": "XM",
           "HUB1": "HUB1", "HUB2": "HUB2", "UPS": "UPS", "CAB_FAN2": "CAB_FAN2", "MAINS_STRIP": "MAINS_STRIP", "SR2": "SR2", "KM3": "KM3", "KM4": "KM4"}
    items = []
    for it in cab["items"]:
        dev = fit.get(it["id"])
        state = "EMPTY position (starter part later)" if dev and dev in absent else ("PARTLY fitted" if it["id"] in ("RAIL_E", "FT_5x20", "FT_6x32", "FT_10x38", "XT_FEED_A", "XT_FEED_B") else "fitted")
        if it["id"] == "UPS":
            state = "RESERVED space (as starter)"
        br = [b for b in prof["bridges"] if b["at"] == dev]
        items.append({"id": it["id"], "plate": it["plate"], "at": it.get("at"), "state": state, **({"bridge": br[0]["part"]} if br else {})})
    return {"profile": prof["profile"], "note": "The compartment layout is the starter's (control_bay.yaml) unchanged: same plates, rails and positions; "
            "the MVP leaves positions EMPTY (or holds a bridge part there) so each stage only fills them.  Rail E holds K_CH only; rail B fuse "
            "terminals F8/F23/F24 and rail E F31/F36 positions empty; the fan-2 hole gets a blanking plate.", "items": items}


def _stages(prof, base_rows):
    """upgrade_steps.json: each roadmap stage after the profile, cumulative, from the starter rows"""
    fitted = set(d for d in SYS["devices"] if d not in prof["absent_devices"])
    have = {r["cable_id"] for r in profile_rows(prof, base_rows)}
    bridged = {b["at"]: b for b in prof["bridges"]}
    out = []
    for st in prof["stages"]:
        fitted |= set(st["devices"])
        add = [r for r in base_rows if r["cable_id"] not in have and "RESERVED" not in r["notes"]
               and ((r["from_device"] in fitted and r["to_device"] in fitted and r["cable_id"] not in prof["absent_cables"]) or r["cable_id"] in st["cables_extra"])]
        have |= {r["cable_id"] for r in add}
        fuses = sorted({m for r in add for m in re.findall(r"\bF\d+\b", r["protected_by"]) if m not in prof["fuses_fitted"]}, key=lambda f: int(f[1:]))
        prof["fuses_fitted"] = prof["fuses_fitted"] + fuses
        sups = [d for d in st["devices"] if SYS["devices"][d]["kind"] == "supply"]
        secs = [s for g in st["groups"] for s in prof["firmware_sections"].get(g, [])]
        cmds = [c for g in st["groups"] for c in prof["absent_commands"].get(g, [])]
        rm = [b for b in prof["bridges"] if b["removed_at"] == st["id"] or (st["id"] == "modules_common" and b["removed_at"] == "deck_modules")]
        mains = any(r["signal"].startswith(("MAINS", "PE")) for r in add) or sups
        retest = list(st["retest"])
        if any(r["signal"].startswith(("PE", "FE")) for r in add) or mains:
            retest.insert(0, "earth continuity < 0.1 ohm from the inlet PE pin to every new exposed part (COMPETENT PERSON)")
        if any(b["at"] in ("KM5", "TC_RELAY", "TB_RELAY", "KM6") for b in rm) or any(r["signal"].startswith("SAFETY") for r in add):
            retest.insert(0, "e-stop proof test: SR1 reset only with every mirror contact closed; pressing the e-stop removes every switched rail")
        out.append({"stage": st["id"], "roadmap_stage": st["roadmap"], "title": st["title"], "devices_added": st["devices"],
                    "cables_added": [{"cable_id": r["cable_id"], "from": r["from_device"], "to": r["to_device"], "signal": r["signal"], "cable_type": r["cable_type"],
                                      "length_mm": r["length_mm_estimate"], "fuse": r["protected_by"]} for r in add],
                    "counts": {"cables": len(add), "devices": len(st["devices"])},
                    "fuses_added": fuses, "supplies_added": sups,
                    "bridge_parts_removed": [f"{b['at']}: {b['part']}" for b in rm],
                    "firmware": {"profile_absent_group_dropped": st["groups"], "sections_added": secs, "commands_restored": cmds, "lines": st["firmware"]},
                    "retest": retest})
    left = sorted(d for d in SYS["devices"] if d not in fitted and "S" in SYS["devices"][d].get("sizes", SIZES))
    return {"profile": prof["profile"], "base": f"starter {prof['base_size']} (wiring.csv)", "roadmap": "cad/lh1/machine_family/out/MVP/roadmap.json stages 1-7 (stage 0 = the MVP)",
            "order": [s["stage"] for s in out], "stages": out, "settings_changed": prof.get("settings", []),
            "never_fitted_in_the_starter": left,
            "note": "Order is the roadmap's; a stage may move, but its list is cumulative from the MVP (a cable appears at the first stage where both its ends are fitted). "
                    "Cables are the starter's rows (same id, ends, length, fuse). DESIGNED and SIMULATED only."}


def main_profile(name):
    prof = load_profile(name)
    per_size, _ = {}, None
    for s in SIZES:
        per_size[s], _ = rows_for(s)
    longest = {}
    for s in SIZES:
        for r in per_size[s]:
            if "y_chain" in r["route"]:
                longest[r["cable_id"]] = max(longest.get(r["cable_id"], 0), int(r["length_mm_estimate"]))
    base = per_size[prof["base_size"]]
    for r in base:
        if r["cable_id"] in longest and int(r["length_mm_estimate"]) < longest[r["cable_id"]]:
            extra = longest[r["cable_id"]] - int(r["length_mm_estimate"])
            r["notes"] = (r["notes"] + "; " if r["notes"] else "") + f"cut for L (+{extra} mm service loop at the floor gland in {prof['base_size']})"
            r["length_mm_estimate"] = longest[r["cable_id"]]
    rows = profile_rows(prof, base)
    sfx = name
    od = HERE / "out" / name.lower()
    od.mkdir(parents=True, exist_ok=True)
    write_rows(rows, od / f"wiring_{sfx}.csv", columns=COLUMNS)
    static, extra = bom_for_profile(prof)
    bom = write_bom(rows, od / f"electrical_bom_{sfx}.csv", static, extra)
    jc = joint_counts(rows)
    (od / f"joint_counts_{sfx}.json").write_text(json.dumps(dict(jc, profile=name, guard=prof["guard"], starter_total=joint_counts(base).get("tool-made joints, total")), indent=1))
    cab = yaml.safe_load((HERE / "control_bay.yaml").read_text())
    bud = _budget(prof, cab)
    (od / f"budgets_{sfx}.json").write_text(json.dumps(bud, indent=1))
    (od / f"control_bay_{sfx}.yaml").write_text("# GENERATED by build_harness.py --profile " + name + ": do not edit\n" + yaml.safe_dump(_control_bay_profile(prof, cab), sort_keys=False, width=160))
    (od / "upgrade_steps.json").write_text(json.dumps(_stages(load_profile(name), base), indent=1))
    (od / "mvp_electrical_notes.txt").write_text(notes_text(prof, rows, base, bom, jc, bud))
    print(f"profile {sfx}: {len(rows)} of {len(base)} starter cables; BOM {len(bom)} lines; tool-made joints {jc.get('tool-made joints, total')}; "
          f"LOGIC {bud['rails']['LOGIC']['pct']} % MOTION {bud['rails']['MOTION']['pct']} % PSU_MAIN {bud['supplies']['PSU_MAIN']['pct']} %; compartment {bud['compartment']['heat_w']} W")
    return rows


def notes_text(prof, rows, base, bom, jc, bud):
    from collections import Counter
    absent = set(prof["absent_devices"])
    fitted = [d for d in SYS["devices"] if d not in absent and "S" in SYS["devices"][d].get("sizes", SIZES)]
    by_kind = Counter(SYS["devices"][d]["kind"] for d in fitted)
    by_type = Counter(r["cable_type"] for r in rows)
    by_sig = Counter(r["signal"].split()[0] for r in rows)
    jc_lines = "\n".join(f"  {k}: {v}" for k, v in jc.items() if not isinstance(v, (dict, list)))
    L = []
    w = L.append
    w("LH-1 MVP ELECTRICS IN PLAIN WORDS (generated by electronics/lh1/build_harness.py --profile MVP from profile_MVP.yaml; do not edit)")
    w("Status: DESIGNED and SIMULATED only. Nothing is built; Klipper has not loaded the files; no electrical safety assessment has been done.")
    w("")
    w("1. WHAT THE MVP IS")
    w("The starter (size S) with its full enclosure, frame, gantry and deck, ONE tool carriage (carriage A, as the MVP CAD cad/lh1/machine_family/out/MVP)")
    w("carrying ONE eight-channel 30-300 uL pipette (tool M300) held by the MCL-1 manual clamp fork with one M5 set screw. The tool is DECLARED in the")
    w("firmware (_LH_PROFILE fixed_tools {a: M300}), not sensed: no latch switches, no Hall code, no dock. Deck (CAD): 300 uL tip frame G3, 12-column")
    w("reservoir F2, plates B2 and B4, tip waste B1 with its tip-check gate (camera not fitted, backlight and receiver bracket fitted); every other window blanked.")
    w("Every cable is a starter cable with the same id, ends, connector positions, length and fuse (out/mvp/wiring_MVP.csv is a strict subset of wiring.csv).")
    w("")
    w("2. WHAT IS IN IT")
    w(f"Fitted devices: {len(fitted)} of the starter's {sum(1 for d, v in SYS['devices'].items() if 'S' in v.get('sizes', SIZES))}. By kind: " + ", ".join(f"{k} {v}" for k, v in sorted(by_kind.items())))
    w("Boards: main Octopus Pro v1.1 (X, Y, Y1 step/dir to the closed-loop motors; CAN bridge; e-stop and door monitors; tip gate),")
    w("  io Octopus Pro (tip drawer, bin-full beam, gate backlight, compartment fan rotation detect), ONE MMB CAN V2.0 (MMB_A: Z_A, pusher A,")
    w("  Z home, pusher retracted, contact and force switches; its 120 ohm CAN jumper ON as the bus end; EZ3 (gripper Z) left empty),")
    w("  Raspberry Pi host (USB to both Octopus boards; no USB hubs).")
    w("Supplies: PSU_MAIN (24 V 25 A, LOGIC + MOTION) and PSU_5V (host). PSU_TC and PSU_AUX are NOT fitted: their mains loop is joined by covered terminal blocks in their positions.")
    w("Drivers: three step/dir breakouts to the X, Y, Y1 closed-loop motors; two EZ5160 Pro on MMB_A (Z_A 0.9 A, pusher 0.7 A).")
    w("Switches and sensors: X, Y, Y-right end stops; three servo-alarm inputs; Z_A home (NC); Z_UP_A in the all-tools-up chain (K_CH on rail E reads it);")
    w("  pusher retracted, contact and force switches (NC); tip-gate beam pair with its supply switch TG_SW and the VC-2 backlight; drawer switch; bin-full beam; one fan RD.")
    w("Safety: e-stop SR1 + KM1/KM2 (monitored manual reset) and the coded door sensor SR2 + KM3/KM4, both in series in MOTION, exactly as the starter.")
    w(f"Fuses fitted: {', '.join(prof['fuses_fitted'])} (starter numbers; every other position empty).")
    w("")
    w("3. COUNTS")
    w(f"Cables: {len(rows)} of the starter's {len(base)}. By type: " + ", ".join(f"{k} {v}" for k, v in sorted(by_type.items())))
    w("By function: " + ", ".join(f"{k} {v}" for k, v in sorted(by_sig.items())))
    w("Joints (harnessgen classes, ADR-003 tool-made vs tool-free):")
    w(jc_lines)
    w(f"Bill of materials: {len(bom)} lines (out/mvp/electrical_bom_MVP.csv), including {len(prof['bridges'])} bridge parts.")
    w("")
    w("4. POWER BUDGET (out/mvp/budgets_MVP.json)")
    for k, v in bud["rails"].items():
        w(f"  rail {k}: {v['load_a']} A of {v['limit_a']} A ({v['fuse']}), {v['pct']} %")
    for k, v in bud["branches"].items():
        w(f"  branch {k}: {v['load_a']} A of {v['fuse_a']} A, {v['pct']} %")
    for k, v in bud["supplies"].items():
        w(f"  supply {k}: {v['load_a']} A of {v['a_max']} A, {v['pct']} % (rule <= 77 %: {v['margin_rule_77pct']})")
    w(f"  mains: about {bud['mains']['worst_coincident_a_at_230V']} A at 230 V on the T10A H inlet fuse")
    c = bud["compartment"]
    w(f"  compartment heat {c['heat_w']} W: needs {c['needed_cfm_10K']} CFM for a 10 K rise; ONE fan gives {c['one_fan_cfm_through_grilles']} CFM ({c['rise_K_one_fan']} K, {c['ratio']} x): one fan is enough;")
    w("  the second fan comes with PSU_AUX (deck modules). A stopped fan is reported (io PF0); with no forced air the box is about 9 K above room air (ESTIMATE).")
    w("")
    w("5. SAFETY ARRANGEMENT AND WHY")
    w(prof["guard_decision"])
    w("The SR1 feedback loop keeps its starter wiring: the mirror contacts of the absent KM5, TC_RELAY, TB_RELAY and KM6 are replaced by feed-through bridge")
    w("terminals in their rail positions (they monitor contactors that do not exist); each is removed when its contactor is fitted. SR1 output 33-34 is unused.")
    w("The all-tools-up chain runs through lever-splice bridges at the empty Z_UP_B and Z_UP_GRIP positions (axes that do not exist are 'up').")
    w("")
    w("6. COMPETENT PERSON, EVEN AT MVP STAGE")
    w("  - mains: IEC inlet with switch and T10A H fuses, W-M-01 to PSU_MAIN, the loop W-M-02/03/04 through the two covered terminal-block bridges to PSU_5V, cord grips, mains in its own duct;")
    w("  - protective earth: PE stud, every bond (frame, plates, compartment box, deck, panels, floor sheets, CBP2, gantry functional bond), continuity < 0.1 ohm, insulation resistance, PAT regime;")
    w("  - SR1/SR2 with KM1-KM4: settings, feedback loops including the four bridge terminals, e-stop and door proof tests, stop category / PL claim;")
    w("  - the guard (full enclosure, coded door sensor, as the starter); SELV/PELV decision; conductor protection table for the fitted fuses.")
    w("")
    w("7. ORDER OF WIRING")
    w("  1 plates, DIN rails, PE stud; every PE bond (W-E-*) and continuity measured before anything is powered;")
    w("  2 COMPETENT PERSON: inlet, PSU_MAIN, bridge blocks, PSU_5V (W-M-*), insulation test; supplies powered with no load, PSU_MAIN set to 24.0 V, PSU_5V to 5.1-5.2 V;")
    w("  3 rail A/B: fuse terminals (fuses OUT), XT distribution, SR1, SR2, KM1-KM4, the four feedback-loop bridge terminals, e-stop, reset, door sensor (W-S-*);")
    w("  4 LOGIC (F1, F6, F7) to both Octopus boards (all driver-voltage jumpers on MOTOR_POWER, all DIAG jumpers removed), host USB;")
    w("  5 gantry: breakouts, X/Y/Y1 motors and alarms, end stops (W-G-*, W-F-*); MOTION feeds W-P-06..10, F21, F25-F27;")
    w("  6 head: chains: head supply W-P-28 and CAN W-C-07 to the lever-splice block, XT30 lead W-C-09 to MMB_A (120 ohm ON at MMB_A in the MVP), Z_A, 1.0 mm FFC to carriage A,")
    w("    all-tools-up chain with its two lever-splice bridges, K_CH;")
    w("  7 deck and services: tip gate, TG_SW and the gate backlight, drawer, bin beam, compartment fan 1.")
    w("")
    w("8. POWER-ON CHECKS")
    w("  a supplies alone (fuses out): voltages; b fuses in one rail at a time, boards enumerate (USB, can0, canbus_query shows MMB_B);")
    w("  c with MOTION fuse F2 OUT: e-stop pressed -> SR1 will not reset; released -> reset lamp, KM1/KM2 pull in; a bridge terminal opened -> reset refused;")
    w("    door open -> KM3/KM4 drop; d every NC input broken-wire test (unplug: reads triggered/blocked); tools-up chain opens when Z_A is lowered;")
    w("  e F2 in: motors enable, servo alarms read healthy; G28 (gripper Z declared at 0, Z_A up, pusher A homed at 0.50 A and its contact found, X then Y);")
    w("  f tip gate commissioning (gate stays NOT armed until the dry run measures the beam with real tips); g check.py and fw_render_check.py green.")
    w("")
    w("9. FIRMWARE")
    w("Klipper loads firmware/lh1/printer_MVP.cfg (GENERATED from printer.cfg without the absent devices' sections) which includes lh1_size_S.cfg and")
    w("lh1_profile_MVP.cfg. Every command of an absent device refuses with DEVICE_ABSENT (CARRIAGE_ABSENT for carriage B). Tips: the force-ended pick-up")
    w("observes 'on'; after an eject the tips are UNKNOWN until the tip gate (fitted, as in the full design) sees bare nozzles; while the gate is not yet")
    w("commissioned a person confirms (LH_TIPS_CONFIRM), exactly as the starter. Printer Z (the gripper Z on head_a EZ3) has no motor: homing_override")
    w("declares it at 0 and every G0/G1 Z refuses. Dispensing over B3 refuses DEVICE_ABSENT (no liquid waste); module, shaker, TC-1 and tip-parking places refuse descents.")
    w("")
    w("10. UNVERIFIED")
    for u in ["homing_override set_position_z with an unpopulated head_a EZ3 socket on the Klipper commit to be installed",
              "MMB_A as the CAN bus end (its 120 ohm jumper) with the Octopus at the other end",
              "LRS-600 inrush / PFC; MDR-60-5 trim; KM1/KM2 DC-1 ratings (as starter)",
              "the four bridge terminals and two mains terminal blocks: ratings, ferrule fit and whether the competent person accepts bridges in a feedback loop",
              "MVP loads (one Z, one pusher, SR/KM coil currents) and the PSU_MAIN loss: catalogue-class estimates",
              "compartment with one fan: class figures, natural-convection estimate; blanking plate over the fan-2 hole (mechanical)",
              "every starter UNVERIFIED part fact that the MVP keeps (system.yaml family.unverified): pusher force at 0.7 A, FFC/ZIF continuous rating, MMB header inputs, sealed switches",
              "cable lengths are the starter's (the MVP CAD keeps the starter's frame, compartment, chains and places; it publishes no cable routes)",
              "OPEN with the CAD: parts_MVP.csv fits BOTH 92 mm compartment fans; the heat calculation needs one (2.4 x the 10 K air). Wired here: fan 1 only, fan-2 hole blanked. If fan 2 is kept, add W-P-25 (F7) and W-I-31 (io PF1) and [gcode_button cab_fan_ok_2]: two starter cables, no other change"]:
        w("  - " + u)
    return "\n".join(L) + "\n"


if __name__ == "__main__":
    import argparse

    ap = argparse.ArgumentParser()
    ap.add_argument("--profile")
    a_ = ap.parse_args()
    if a_.profile:
        main_profile(a_.profile)
    else:
        main()
