"""LH-1 revision K deck stations (ADR-004 vision, ADR-005 weighing, ADR-006 manual actions, ADR-007 tip economy).

Built from bought parts first (ADR-003).  Every figure is CALCULATED from the model or from an input marked ASSUMED;
nothing is built or measured.  The assembly (cad/lh1/assembly/build.py) imports placed_items() and checks every item
against the head over its working and transit configurations.

Stations (nest -> station, params K_STATIONS):
  A2 tip parking TPK-1 (ADR-007 d1)      A3, D2 tip-rack stacks (2 units)   C2 plate stack (8 units)
  B1 weighing nest WN-1 (+ passive cold strip carrier PCB-1 when mixes are made)   B2 magnet block on a load cell (variant WN-M)
  B3 bought shaker on SK-1 (ADR-006 d1) + lid LD-1 (d2), parked on D3 over the sealing lid (LIFO)
  F2 balance station BAL-1 (ADR-005 d8)    A1 VC-2 side camera + backlight at the tip gate (ADR-004 d4)

Out: cad/lh1/deck_k/out/{report.json, <part>.step / .dxf / .stl}
"""
from __future__ import annotations

import importlib.util
import json
import math
import sys
from pathlib import Path

import cadquery as cq

HERE = Path(__file__).resolve().parent
LH1 = HERE.parent
sys.path.insert(0, str(LH1))
import params as P  # noqa: E402

OUT = HERE / "out"


def _load(name, rel):
    spec = importlib.util.spec_from_file_location(name, str(LH1 / rel))
    m = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(m)
    return m


DK = _load("deck_build_k", "deck_modules/build.py")
B, cyl, mv, fuse_all, cut_all = DK.B, DK.cyl, DK.mv, DK.fuse_all, DK.cut_all
SEAT = P.LABWARE_SEAT_Z
ST = P.K_STATIONS
NEST = {n: P.nest_centre(n) for row in P.NEST_NAMES for n in row}
RHO = {"al": 2.70e-6, "pa12": 1.01e-6, "steel": 7.85e-6, "pp": 0.90e-6}


# =====================================================================================================
# 1. weighing nest WN-1 (B1) and magnet-block variant WN-M (B2)
# =====================================================================================================
CELL = dict(L=80.0, W=12.7, H=12.7, rated_kg=1.0, defl_at_rated_mm=0.3, holes=(6.0, 18.0),
            spec="single-point aluminium bar load cell 1 kg, about 80 x 12.7 x 12.7, 2 x M4 each end 12 apart (NOMINAL: measure the cell "
                 "bought), HX711-class amplifier board on a plug-in lead")
CELL_M = dict(CELL, rated_kg=3.0, spec="single-point bar load cell 3 kg class (magnet block MB-1 about 1.1 kg + plate): same envelope, NOMINAL")
WN = dict(gap=0.5, plat=(100.0, 60.0, 6.0), brk=(-75.0, -25.0, -15.0, 15.0, 6.0), insert_screws=[(sx * 40.0, sy * 22.0) for sx in (-1, 1) for sy in (-1, 1)])
Z_DECK_BOT = -P.DECK_T


def wn_parts(cell=CELL):
    """nest coordinates.  The cell hangs under the deck window; its fixed end is screwed to WNB-1 on the deck underside (2 x M4
    through the window web), its free end carries the WNP-1 spider inside the window; the IN-W insert sits on WNP-1, 0.5 mm clear
    of the deck all round (the deck no longer carries it)."""
    bx0, bx1, by0, by1, bt = WN["brk"]
    brk = B(bx0, bx1, by0, by1, Z_DECK_BOT - bt, Z_DECK_BOT)
    brk = cut_all(brk, [cyl(x, y, Z_DECK_BOT - bt - 1, Z_DECK_BOT + 1, 4.5) for x in (-67.0,) for y in (-8.0, 8.0)]
                  + [cyl(-40.0 + h, 0.0, Z_DECK_BOT - bt - 1, Z_DECK_BOT + 1, 4.5) for h in cell["holes"]]
                  + [cyl(-45.0, 0.0, Z_DECK_BOT - bt - 1, Z_DECK_BOT + 1, 3.3)])             # M4 tapped: overload set screw
    zc1 = Z_DECK_BOT - bt                       # cell top under the bracket
    cellb = B(-cell["L"] / 2, cell["L"] / 2, -cell["W"] / 2, cell["W"] / 2, zc1 - cell["H"], zc1)
    px, py, pt = WN["plat"]
    z_ins_bot = WN["gap"]
    plat = B(-px / 2, px / 2, -py / 2, py / 2, z_ins_bot - pt, z_ins_bot)
    plat = cut_all(plat, [cyl(x, y, z_ins_bot - pt - 1, z_ins_bot + 1, 3.3) for x, y in WN["insert_screws"]]
                   + [cyl(40.0 - h, 0.0, z_ins_bot - pt - 1, z_ins_bot + 1, 4.5) for h in cell["holes"]])
    spacer = B(cell["L"] / 2 - 12.0, cell["L"] / 2, -cell["W"] / 2, cell["W"] / 2, zc1, z_ins_bot - pt)
    screw = cyl(-45.0, 0.0, zc1 - 6.0, z_ins_bot - pt - WN["gap"], 3.3)                    # overload stop, gap set by instrument
    cover = B(-cell["L"] / 2 - 2, cell["L"] / 2 - 14.0, -cell["W"] / 2 - 2.0, cell["W"] / 2 + 2.0, zc1 - cell["H"] - 2.0, zc1 - cell["H"])
    ins, feats = DK.insert_plate(tap_m3=DK.locator_tap_points())
    ins = cut_all(ins, [cyl(x, y, -1, P.INSERT_T + 1, DK.M4_CLR) for x, y in WN["insert_screws"]])
    ins = mv(ins, z=WN["gap"])
    return dict(bracket=brk, cell=cellb, platform=plat, spacer=spacer, stop_screw=screw, drip_cover=cover, insert=ins)


def wn_checks(cell=CELL):
    k_v = cell["rated_kg"] * 9.81 / cell["defl_at_rated_mm"]
    budget = 0.2                           # mm, tip-touch / tip-entry Z budget (ASSUMED; the planner's value governs)
    return {"cell": cell["spec"], "vertical_stiffness_N_per_mm (ASSUMED rated deflection)": round(k_v, 1),
            "sink_at_tip_touch_N": {f"{f:g} N": round(f / k_v, 3) for f in (1.0, 2.0, 5.0)},
            "tip_entry_budget_mm (ASSUMED)": budget, "max_touch_force_within_budget_N": round(budget * k_v, 1),
            "lateral_stiffness_N_per_mm (ASSUMED 10 x vertical, parallelogram cell)": round(10 * k_v, 0),
            "lateral_deflection_at_1N_wall_touch_mm": round(1.0 / (10 * k_v), 4),
            "height_added_mm": WN["gap"], "height_note": "the IN-W insert floats 0.5 mm above the deck on the cell: labware seat z 5.5 instead "
            "of 5.0; K rules and Z travel unchanged within 0.5 mm (the cell is under the deck window)",
            "overload_stop": "M4 set screw in WNB-1 under the WNP-1 spider, gap 0.5 mm set by instrument (screw in until the reading moves "
                             "1 g, back off 4 GG-3 notches = 0.47 mm): engages at about "
                             f"{0.5 * k_v / 9.81 * 1000:.0f} g on top of the tare",
            "spill_guard": "IN-W keeps a 0.5 mm air gap all round (no wicking path); a PA12 drip cover over the cell under the window "
                           "sheds drips to the deck underside; the amplifier sits on the deck underside 60 mm away",
            "settle_and_read": "read with the gantry stopped and the shaker off (rule K-W1); settle time from the bench rig test"}


# =====================================================================================================
# 2. passive cold strip carrier PCB-1 (on WN-1 for mix making; chilled on a cold module)
# =====================================================================================================
PCB = dict(L=P.SBS_L, W=P.SBS_W, H=20.0, well_d=5.9, well_depth=16.0, foot_h=2.0)


def pcb_part():
    s = B(-PCB["L"] / 2, PCB["L"] / 2, -PCB["W"] / 2, PCB["W"] / 2, 0.0, PCB["H"])
    tools = [cyl(-49.5 + 9.0 * c, -31.5 + 9.0 * r, PCB["H"] - PCB["well_depth"], PCB["H"] + 1, PCB["well_d"]) for c in range(12) for r in range(8)]
    tools.append(B(-PCB["L"] / 2 - 1, PCB["L"] / 2 + 1, -PCB["W"] / 2 - 1, -PCB["W"] / 2 + 3.0, 2.0, 6.0))      # gripper band relief (front)
    tools.append(B(-PCB["L"] / 2 - 1, PCB["L"] / 2 + 1, PCB["W"] / 2 - 3.0, PCB["W"] / 2 + 1, 2.0, 6.0))
    return cut_all(s, tools)


def pcb_hold_time(m_al):
    c_al, c_w = 900.0, 4186.0
    m_w = 96 * 20e-6                        # 20 uL per well (ASSUMED average mix content)
    m_pp = 12 * 8 * 0.12e-3                 # 96 strip tubes
    C = m_al * c_al + m_w * c_w + m_pp * 1900.0
    A_top = P.SBS_L * P.SBS_W * 1e-6
    A_side = 2 * (P.SBS_L + P.SBS_W) * PCB["H"] * 1e-6
    out = {}
    for lab, h_top, foam in (("bare", 10.0, None), ("foam_sleeve_10mm_and_PA12_feet", 10.0, 0.010)):
        G_top = h_top * A_top * 1.3                                  # tube tops proud of the block: x 1.3 (ASSUMED)
        G_side = (1 / (1 / 10.0 + foam / 0.035)) * A_side if foam else 10.0 * A_side
        G_bot = (0.22 * 4 * 25e-6 / 2e-3) if foam else 10.0 * A_top * 0.5   # 4 PA12 feet 5 x 5 x 2 on the platform, else air under
        G = G_top + G_side + G_bot
        t = C / G * math.log((25.0 - 2.0) / (25.0 - 8.0))
        out[lab] = {"G_W_per_K": round(G, 3), "minutes_from_2C_to_8C": round(t / 60.0, 1)}
    return {"mass_al_kg": round(m_al, 3), "heat_capacity_J_per_K": round(C, 1), "ambient_C": 25.0, "start_C": 2.0, **out,
            "assumptions": "h 10 W/m2K natural convection + radiation; 20 uL per well; foam k 0.035; chilled to 2 C on a cold module (C3/F3) "
                           "and moved by the gripper (mass within the 0.6 kg payload)",
            "verdict": "a mix-making window of the order of 10-20 min, not the 60 min placeholder in the capability catalogue"}


# =====================================================================================================
# 3. tip parking TPK-1 (A2)
# =====================================================================================================
TPK = dict(z_lower_top=72.0, t_lower=8.0, t_upper=6.0, collar_h=8.0, x_entry=-58.0, x_spots=[-49.4 + 8.6 * k for k in range(13)],   # 13 sets: 96 waste + 1 clean-up
           half=(63.0, 40.5), tray_h=4.0, posts=[(sx * 58.0, sy * 35.0) for sx in (-1, 1) for sy in (-1, 1)])   # y <= 40.5: IN-P tag visible


def tpk_dims(tip="p300"):
    col = P.TIP_COLLAR_OD[tip]
    noz = P.TIP_NOZZLE_OD_AT_COLLAR[tip]
    return dict(ws=round(col - 1.2, 2), wn=round(noz + 2 * P.COMB_SLOT_CLEARANCE, 2), entry_d=round(col + 0.6, 2), collar=col)


def tpk_parts(tip="p300"):
    d = tpk_dims(tip)
    hx, hy = TPK["half"]
    zl1 = TPK["z_lower_top"]
    zl0 = zl1 - TPK["t_lower"]
    zu0 = zl1 + TPK["collar_h"] + 0.6
    zu1 = zu0 + TPK["t_upper"]
    rows = [(k - 3.5) * P.WELL_PITCH_96 for k in range(8)]
    x0, x1 = TPK["x_entry"], TPK["x_spots"][-1]
    lower = B(-hx, hx, -hy, hy, zl0, zl1)
    upper = B(-hx, hx, -hy, hy, zu0, zu1)
    lt, ut = [], []
    for y in rows:
        lt.append(B(x0, x1, y - d["ws"] / 2, y + d["ws"] / 2, zl0 - 1, zl1 + 1))
        lt += [cyl(x0, y, zl0 - 1, zl1 + 1, d["ws"]), cyl(x1, y, zl0 - 1, zl1 + 1, d["ws"])]
        ut.append(B(x0, x1, y - d["wn"] / 2, y + d["wn"] / 2, zu0 - 1, zu1 + 1))
        ut += [cyl(x1, y, zu0 - 1, zu1 + 1, d["wn"]), cyl(x0, y, zu0 - 1, zu1 + 1, d["entry_d"])]
    for x, y in TPK["posts"]:
        lt.append(cyl(x, y, zl0 - 1, zl1 + 1, 3.4))
        ut.append(cyl(x, y, zu0 - 1, zu1 + 1, 3.4))
    lower, upper = cut_all(lower, lt), cut_all(upper, ut)
    posts = fuse_all([cyl(x, y, SEAT, zl0, 10.0) for x, y in TPK["posts"]] + [cyl(x, y, zl1, zu0, 10.0) for x, y in TPK["posts"]])
    posts = cut_all(posts, [cyl(x, y, SEAT - 1, zu1 + 1, 3.4) for x, y in TPK["posts"]])
    tray = B(-hx + 6, hx - 6, -hy + 3, hy - 3, SEAT, SEAT + TPK["tray_h"])
    tray = tray.cut(B(-hx + 8, hx - 8, -hy + 5, hy - 5, SEAT + 1.5, SEAT + TPK["tray_h"] + 1))
    tray = tray.cut(fuse_all([cyl(x, y, SEAT - 1, SEAT + TPK["tray_h"] + 1, 11.0) for x, y in TPK["posts"]]))
    tips = [B(x - d["collar"] / 2, x + d["collar"] / 2, y - d["collar"] / 2, y + d["collar"] / 2, zl1 - P.TIP_BELOW_NOZZLE[tip], zl1 + TPK["collar_h"])
            for x in TPK["x_spots"] for y in rows]
    return dict(lower=lower, upper=upper, posts=posts, tray=tray), tips


def tpk_checks():
    out = {}
    for tool, tip in P.TOOL_TIP.items():
        d = tpk_dims(tip)
        land_l = P.WELL_PITCH_96 - d["ws"]
        span = 2 * TPK["half"][0]
        I_l = land_l * TPK["t_lower"] ** 3 / 12
        f_tip = P.PICKUP_FORCE_N[tool] / (8 if tool != "pipette_small" else 1)
        defl = f_tip * span ** 3 / (48 * 70000.0 * I_l)
        land_u = P.WELL_PITCH_96 - d["wn"]
        I_u = land_u * TPK["t_upper"] ** 3 / 12
        f_strip = P.STRIP_FORCE_N[tool] / (8 if tool != "pipette_small" else 1)
        out[tool] = {"tip": tip, "lower_slot_w (shaft)": d["ws"], "upper_slot_w (nozzle)": d["wn"], "entry_hole_d": d["entry_d"],
                     "collar_overlap_each_side_upper_mm": round((d["collar"] - d["wn"]) / 2, 2),
                     "collar_overlap_each_side_lower_mm": round((d["collar"] - d["ws"]) / 2, 2),
                     "pickup_force_per_tip_N": f_tip, "lower_land_deflection_mm": round(defl, 3),
                     "strip_force_per_tip_N": f_strip, "upper_land_deflection_mm": round(f_strip * span ** 3 / (48 * 70000.0 * I_u), 3),
                     "tip_end_z_parked": round(TPK["z_lower_top"] - P.TIP_BELOW_NOZZLE[tip], 1),
                     "clear_above_drip_pad_mm": round(TPK["z_lower_top"] - P.TIP_BELOW_NOZZLE[tip] - (SEAT + TPK["tray_h"]), 1)}
    return {"capacity": {"eight_channel_sets": len(TPK["x_spots"]), "tips_8ch": 8 * len(TPK["x_spots"]),
                         "single_tips": 8 * len(TPK["x_spots"]), "rule": "LIFO per slot row: park the deepest spot first, re-pick from "
                         "the entry end; one TPK-1 per tip type (slot widths), the p300 plate fitted by default"},
            "sequence": "set down: lower the tips through the D(collar + 0.6) entry hole until the collar rests on the lower plate, slide +X "
                        "to the spot, lift (the upper plate holds the collar: tips stay).  Re-pick: lower the nozzle through the upper "
                        "slot into the parked collar to the pick-up force (collar on the lower plate), slide -X to the entry, lift",
            "force_path": "nozzle -> tip collar -> lower-plate lands (8 mm Al, end-supported over 126 mm) -> 4 posts -> IN-P -> deck",
            "per_tool": out,
            "drip": "parked tips hang inside the TPK-1 drip tray (absorbent pad) with their ends >= 4 mm above the pad; the tray walls "
                    "enclose the slot field, so no parked tip is above labware",
            "keep_outs": ["K-P1: at A2 only parking moves (entry x -58 nest-local, spots x -46..53); no labware at A2",
                          "K-P2: the eight-channel tools at A2 overhang A1 (HP-2B to A1-local x 49): A1 gate posts/backlight below "
                          "z 95, HP-2B stays above z 110 at the parking depth (checked in the assembly)"],
            "material": "lower plate 8 mm and upper plate 6 mm laser-cut 5052 (slots from the MEASURED tip and nozzle ODs, as TW-4D); "
                        "4 x M3 posts (PA12); drip tray PA12 with a cut-to-size absorbent pad"}


# =====================================================================================================
# 4. shaker (F3 in rev K; first placed at B3), lid LD-1, lid park (D3)
# =====================================================================================================
SHAKER = {"A_cooled_LH_module": dict(L=147.0, W=104.0, H=110.0, mass=2.5, temp="4..70 C (ASSUMED from the class; check the maker's sheet)",
                                     confirm="footprint, plate-seat height, clamp jaw travel, mounting holes, controller interface"),
          "B_clamping_shaker_no_temp": dict(L=145.0, W=100.0, H=75.0, mass=1.6, temp="room only: does NOT meet 4-37 C",
                                            confirm="as A")}
SHAKER_USE = "A_cooled_LH_module"
SK = dict(t=6.0, mounts_h=10.0, x_off=29.0, y_off=-2.5, mounts=[(sx * 60.0, sy * 40.0) for sx in (-1, 1) for sy in (-1, 1)])
# x_off 29: the body's -X face at nest-local x -44.5 clears CB-1 / HP-2B of the eight-channel tools working at B2 column 12 (they reach
# B3-local x -46.5 / -57.8 at depth: found by the assembly); the +X face at 102.5 (global 234.5) stays inside the 250 deck edge.
LID = dict(L=127.8, W=85.5, H=9.5, skirt=7.0, mass_kg=0.03, ceiling_above_rim=1.0)


def shaker_parts(cls=SHAKER_USE):
    s = SHAKER[cls]
    z0 = SEAT
    sk = B(-70.0 + SK["x_off"], 70.0 + SK["x_off"], -50.0, 50.0, z0, z0 + SK["t"]).fuse(B(-66.0, -70.0 + SK["x_off"] + 1.0, -20.0, 20.0, z0, z0 + SK["t"]))
    sk = cut_all(sk, [B(-62.2, -49.8, -6.2, 6.2, z0 + SK["t"] - 0.3, z0 + SK["t"] + 1)] +                     # tag pocket (beside the shaker body, -X)
                 [cyl(-56.0, y, z0 + SK["t"] - 1.5, z0 + SK["t"] + 1, 2.0) for y in (-14.0, 14.0)])
    sk = cut_all(sk, [cyl(x + SK["x_off"], y, z0 - 1, z0 + SK["t"] + 1, 4.5) for x, y in SK["mounts"]]
                 + [cyl(x, y, z0 - 1, z0 + SK["t"] + 1, 3.4) for x, y in DK.locator_tap_points()[:4]])
    zm = z0 + SK["t"]
    mounts = fuse_all([cyl(x + SK["x_off"], y, zm, zm + SK["mounts_h"], 15.0) for x, y in SK["mounts"]])
    zb = zm + SK["mounts_h"]
    body = B(-s["L"] / 2 + SK["x_off"], s["L"] / 2 + SK["x_off"], -s["W"] / 2, s["W"] / 2, zb, zb + s["H"])
    z_seat = zb + s["H"]
    plate = B(-P.SBS_L / 2 + SK["x_off"], P.SBS_L / 2 + SK["x_off"], -P.SBS_W / 2, P.SBS_W / 2, z_seat, z_seat + DK.PCR["H"])
    lid = B(-LID["L"] / 2 + SK["x_off"], LID["L"] / 2 + SK["x_off"], -LID["W"] / 2, LID["W"] / 2, z_seat + DK.PCR["H"] + LID["ceiling_above_rim"] - (LID["H"] - 1.5),
            z_seat + DK.PCR["H"] + LID["ceiling_above_rim"] + 1.5)
    parts = dict(adapter=sk, mounts=mounts, body=body, plate=plate, lid=lid)
    return {k: mv(v, y=SK["y_off"]) for k, v in parts.items()}, z_seat


def lid_contents_check():
    p = DK.PCR
    a_cyl = math.pi / 4 * p["od_cyl"] ** 2

    def height_for(v_ul):
        d1, d2, hc = p["od_bot"], p["od_cone_top"], p["cone_h"]
        v_cone = math.pi / 12 * hc * (d1 * d1 + d1 * d2 + d2 * d2)
        if v_ul <= v_cone:
            lo, hi = 0.0, hc
            for _ in range(50):
                m = (lo + hi) / 2
                dm = d1 + (d2 - d1) * m / hc
                (lo, hi) = (m, hi) if math.pi / 12 * m * (d1 * d1 + d1 * dm + dm * dm) < v_ul else (lo, m)
            return lo, (d1 + (d2 - d1) * lo / hc) / 2
        return hc + (v_ul - v_cone) / a_cyl, p["od_cyl"] / 2
    rim = p["H"]
    rows = {}
    for v in (50.0, 100.0, 150.0, 180.0):
        h, r = height_for(v)
        z_liq = p["well_bottom"] + h
        free_rim = rim - z_liq
        free_lid = free_rim + LID["ceiling_above_rim"]
        orbit = 1.5e-3                     # m, orbital diameter/2 class value (ASSUMED)
        w_max = math.sqrt(max(free_rim - 1.0, 0.0) * 1e-3 * 9.81 / (r * 1e-3 * orbit))
        rows[f"{v:g}_uL"] = {"liquid_top_below_rim_mm": round(free_rim, 2), "to_lid_ceiling_mm": round(free_lid, 2),
                             "max_rpm_keep_1mm_below_rim": round(w_max * 60 / (2 * math.pi))}
    return {"plate": "0.2 mL full-skirt PCR plate (deck_modules PCR, NOMINAL)", "lid": "bought SBS universal lid class, ceiling 1.0 mm above "
            "the well rims (condensation rings, ASSUMED)", "at_rest": "liquid cannot touch the lid at any fill up to the 180 uL wash cap", **rows,
            "shaking_rule": "surface tilt estimate dh = r_well * w^2 * r_orbit / g (orbit 1.5 mm ASSUMED): the planner caps rpm per fill "
                            "(table above) so the meniscus stays >= 1 mm below the rim; bead steps are slow anyway (ADR-006 addendum)"}


def rotation_concept():
    return {"question": "on-deck rotation or rocking of a SEALED plate about a horizontal axis, loaded by the gripper (2024 evidence: plate on "
                        "edge on a rotator kept beads better than vigorous flat shaking)",
            "concept": "a cradle (two side jaws closing on the plate's long sides, like the gripper) on a horizontal shaft along X under a "
                       "deck position, turned by a geared NEMA17 through +/-90 deg (rocking) or 360 deg (end over end); the gripper sets the "
                       "sealed plate flat into the open cradle at the top of its swing, the jaws close, the cradle rotates; it returns flat "
                       "and opens for the gripper",
            "space": "a 128 x 86 plate turning about its long axis sweeps a 128 x 86 diameter cylinder (86 mm): with the axis 50 mm above the "
                     "deck it needs z 7..93 over the position and the two neighbours' edges in Y kept clear: it takes one position and the "
                     "rows beside it must hold low labware only",
            "sealing": "the plate must be sealed liquid-tight (heat seal or adhesive film): the gripper lid LD-1 does not seal; heat sealing is "
                       "not on the deck (ADR-006: plate sealer not found cheaply) - so each rotation needs a manual seal, which defeats "
                       "the purpose unless a cap-mat or clamped silicone lid that the gripper can fit is proven leak-tight",
            "amateur_build": "the mechanics (shaft, two bearings, a geared stepper, printed jaws, a home switch) are within an amateur's reach; "
                             "the leak-tight closure the gripper can place is NOT, and it is the deciding risk",
            "verdict": "feasible as mechanics, NOT feasible as an automated step until a gripper-placed leak-tight closure exists; first test on "
                       "the bench rig: orbital shaking at low speed on the shaker (ADR-006 consequence 1) before building a rotator",
            "cost_estimate": "about GBP 80-150 of bought parts (stepper + gearbox, bearings, shaft, switch) plus printed jaws (ESTIMATE)"}


def shaker_checks(z_seat):
    out = {}
    for k, s in SHAKER.items():
        zs = SEAT + SK["t"] + SK["mounts_h"] + s["H"]
        k_mount = 4 * 2.0                                  # N/mm, four soft sorbothane-class feet (ASSUMED)
        fn = math.sqrt(k_mount * 1000 / s["mass"]) / (2 * math.pi)
        out[k] = {"envelope_mm (ASSUMED)": [s["L"], s["W"], s["H"]], "mass_kg (ASSUMED)": s["mass"], "temperature": s["temp"],
                  "confirm_from_maker_drawing": s["confirm"], "plate_seat_z": round(zs, 1), "plate_top_z": round(zs + DK.PCR["H"], 1),
                  "lid_top_z": round(zs + DK.PCR["H"] + LID["ceiling_above_rim"] + 1.5, 1),
                  "under_MAX_LABWARE_H_150": zs + DK.PCR["H"] + 2.5 <= P.MAX_LABWARE_H,
                  "isolation_natural_frequency_Hz": round(fn, 1),
                  "transmissibility_at_600_1000_1500_rpm": {str(r): round(1 / abs((r / 60.0 / fn) ** 2 - 1), 2) for r in (600, 1000, 1500)}}
    return {"position": "F3 (rev K modelled: front-right corner, SK x_off 29 / y_off -2.5 so the body clears F2 and the deck edge; "
                        "D4 in the K2 layout)", "classes": out, "used_in_model": SHAKER_USE,
            "under_deck_depth_mm": 0.0, "under_deck_note": "stands on the deck (SK-1 on the IN-P insert); only its cable goes below, through the "
            "F3 window, along the base members to the control bay (about 300 mm)",
            "vibration": "the shaker is internally balanced (class); it stands on four soft feet (f_n above) on SK-1, so at 1000 rpm and above "
                         "the force reaching the deck is cut to the transmissibility shown; the weighing nests (B1) and VC-1 are read only with "
                         "the shaker stopped (rule K-W1), and the gantry does not depend on it (vision and weight only hold a run)",
            "gripper_access": "the gripper closes on the plate's long sides (Y): with the shaker's clamp open the jaws must leave >= 9.3 mm "
                              "beside each long side at the grip band (finger 6 mm + 3.3 mm open travel): CONFIRM on the maker's drawing",
            "lid": lid_contents_check(), "rotation_concept_study": rotation_concept(), "lid_park": "D3: LD-1 parks on top of the TC-1 sealing lid on the LS-1 pedestal (two lids, last on "
            "first off, rule K-L1)"}


# =====================================================================================================
# 5. balance station BAL-1 (F2)
# =====================================================================================================
BAL_ASSUMED = dict(L=220.0, W=340.0, H=100.0, pan_d=90.0)
BAL_BAY = dict(x=(-70.0, 65.0), y=(-338.0, -237.0), z=(-130.0, -8.0))   # z0 -130: the stand plate stays 3 mm above the base members         # global, F2 (calculated from the H/J layout)
BAL_CELL = dict(L=125.0, W=95.0, H=100.0, pan_d=90.0, stem_d=20.0, pan_z=9.0,
                spec="precision weighing cell class with separate electronics, 0.1 mg, cell <= 125 x 95 x 100 (ASSUMED; no drawing in the repository)")
SHIELD = dict(od=104.0, wall=2.0, h=60.0)


BAL_PAN_XY = (NEST["F2"][0], -287.5)        # centred in the bay between the y -350 and y -225 deck beams (12.5 mm in front of F2)


def bal_parts():
    cx, cy = BAL_PAN_XY
    zb0 = BAL_BAY["z"][0]
    cell = B(cx - BAL_CELL["L"] / 2, cx + BAL_CELL["L"] / 2, cy - BAL_CELL["W"] / 2, cy + BAL_CELL["W"] / 2, zb0, zb0 + BAL_CELL["H"])
    stem = cyl(cx, cy, zb0 + BAL_CELL["H"], BAL_CELL["pan_z"] - 1.0, BAL_CELL["stem_d"])
    pan = cyl(cx, cy, BAL_CELL["pan_z"] - 1.0, BAL_CELL["pan_z"], BAL_CELL["pan_d"])
    nx, ny = NEST["F2"]
    ins = B(nx - DK.INS_HX, nx + DK.INS_HX, ny - DK.INS_HY, ny + DK.INS_HY, 0.0, P.INSERT_T)
    ins = cut_all(ins, [cyl(cx, cy, -1, P.INSERT_T + 1, BAL_CELL["pan_d"] + 6.0)]
                  + [DK.csk(nx + x, ny + y, P.INSERT_T, 0, DK.M4_CLR, DK.M4_CSK) for x, y in P.INSERT_SCREWS])
    sh = cyl(cx, cy, P.INSERT_T, P.INSERT_T + SHIELD["h"], SHIELD["od"]).cut(cyl(cx, cy, P.INSERT_T - 1, P.INSERT_T + SHIELD["h"] + 1, SHIELD["od"] - 2 * SHIELD["wall"]))
    legs = fuse_all([B(cx + sx * 50 - 10, cx + sx * 50 + 10, ly - 10, ly + 10, -192.0, zb0) for sx in (-1, 1) for ly in (-335.0, -240.0)])
    top = B(cx - 64, cx + 64, -345.0, -232.0, zb0 - 3.0, zb0)      # +/-64: clear of the TEC-1166 envelope (x >= 66)
    return dict(cell=cell, stem=stem, pan=pan, insert=ins, shield=sh, stand_legs=legs, stand_top=top)


def bal_checks():
    a = BAL_ASSUMED
    bay = [BAL_BAY["x"][1] - BAL_BAY["x"][0], BAL_BAY["y"][1] - BAL_BAY["y"][0], BAL_BAY["z"][1] - BAL_BAY["z"][0]]
    return {"assumed_balance_mm": [a["L"], a["W"], a["H"]], "assumed_fits": False,
            "why_not": "every place a pan can be reached by all three pipettes (x -189..185, y -310..177) has deck beams every 110 mm in Y "
                       "under it (z -46..-6) and the cold, thermocycler, tip-bin and magnet hardware beside it; a 220 x 340 body below the deck "
                       "crosses two beams, and on top of the deck it would take three positions and stand 100 mm high",
            "bay_under_F2_mm (x, y, z; CALCULATED)": bay, "bay_bounds_global": BAL_BAY,
            "fits_in_bay": "a weighing cell <= 125 x 95 x 100 with separate electronics (class ASSUMED): " + BAL_CELL["spec"],
            "pan": {"centre_xy": list(BAL_PAN_XY), "z": BAL_CELL["pan_z"], "d": BAL_CELL["pan_d"]},
            "shield": f"PA12 cylinder OD {SHIELD['od']:g} x {SHIELD['h']:g} on the IN-B insert, open on top: tips enter from above",
            "stand": "BST-1: 4 legs of 20 x 20 extrusion cut to length standing on the bench through the base-member gaps (y -370..-300 and "
                     "-260..-220), >= 5 mm from every frame member: the balance does not touch the frame",
            "reach": "F2 is reached by all three pipettes (rev H reach check); eight-channel tips spread 63 mm in Y: inside the 100 mm "
                     "shield bore",
            "conflict": "the stage-4 waste-container path (x -60..60 under F2, not in the first build) must move: wiring/stage-4 owner",
            "to_confirm": "the balance's own drawing (cell and pan positions, stem, electronics cable) before the IN-B hole and BST-1 are cut"}


# =====================================================================================================
# 6b. tip supply (ADR-007 d11): refill-wafer towers + spent-wafer stack on the TSS-1 shelf beside the deck; tip frames on the deck
# =====================================================================================================
WAFER = {"p300": dict(pitch=12.0, tip_l=P.TIP_BELOW_NOZZLE["p300"]), "p200": dict(pitch=10.0, tip_l=P.TIP_BELOW_NOZZLE["p200"]),
         "p10": dict(pitch=7.0, tip_l=P.TIP_BELOW_NOZZLE["p10"])}
WAFER_PLATE = dict(L=122.0, W=80.0, t=2.5, lip_h=4.0, lip_t=1.2, mass_kg=0.020, tip_mass_g={"p300": 0.65, "p200": 0.5, "p10": 0.3},
                   note="ASSUMED refill wafer: 96 tips in a 122 x 80 x 2.5 plate with a 4 mm vertical lip on each long side (the gripper "
                        "closes on the lips at 80 mm, GRIP_CLOSED); the tips of one wafer nest in the tips of the wafer below.  Purchasing "
                        "must confirm that the chosen tips are sold as nested refills of this form, and their real nesting pitch")
SHELF = dict(x=(242.0, 382.0), y=(-358.0, 178.0), t=4.0, flange=20.0)
SPENT_PITCH = 4.0
TIP_DEMAND = dict(total=1900, margin=1.25, split={"p300": 0.55, "p200": 0.25, "p10": 0.20}, parked_p300=96 + 8)


def tower_max(tip):
    w = WAFER[tip]
    return int((P.MAX_LABWARE_H - (w["tip_l"] + WAFER_PLATE["t"])) // w["pitch"]) + 1


def tower_height(tip, n):
    w = WAFER[tip]
    return SEAT + w["tip_l"] + WAFER_PLATE["t"] + (n - 1) * w["pitch"]


def tip_supply_plan():
    need = {t: math.ceil(TIP_DEMAND["total"] * TIP_DEMAND["margin"] * f / 96.0) for t, f in TIP_DEMAND["split"].items()}
    towers = {}
    for sl, (kind, _) in P.K_TIP_SHELF_SLOTS.items():
        if kind.startswith("tower_"):
            towers.setdefault(kind[6:], []).append(sl)
    cap = {t: tower_max(t) * len(towers.get(t, [])) + 1 for t in WAFER}      # + the wafer already on the deck frame
    spent_n = sum(need.values()) - 3
    return {"demand_tips": {t: round(TIP_DEMAND["total"] * TIP_DEMAND["margin"] * f) for t, f in TIP_DEMAND["split"].items()},
            "basis": "96-sample day 1, about 1,900 tips x 1.25 margin, split p300 55 % / p200 25 % / p10 20 % (planning assumption until "
                     "the planner reports)", "wafers_needed": need, "towers": towers,
            "wafers_per_tower_max": {t: tower_max(t) for t in WAFER}, "wafer_capacity_incl_deck_frame": cap,
            "covers_the_run": all(cap[t] >= need[t] for t in WAFER),
            "spent_wafers": spent_n, "spent_stack_height_mm": round(spent_n * SPENT_PITCH, 1),
            "tip_parking": "TPK-1 now holds 13 eight-channel sets = 104 p300 tips: 96 parked waste tips + one clean-up set"}


def tip_supply_parts():
    x0, x1 = SHELF["x"]
    y0, y1 = SHELF["y"]
    t = SHELF["t"]
    shelf = B(x0, x1, y0, y1, 0.0, t).fuse(B(x1 - t, x1, y0, y1, 0.0, SHELF["flange"]))      # laser-bent tray, outer flange UP (bay cover below)
    shelf = cut_all(shelf, [cyl(246.0, y, -1, t + 1, 4.5) for y in (-330.0, -240.0, -150.0, -60.0, 30.0, 120.0)])   # M4 into the deck margin
    out = {"shelf": shelf}
    brk = []
    for y in (-330.0, -200.0, 165.0):                                         # 3 brackets outside the control-bay cover (|y| > 148)
        brk.append(B(240.0, x1, y - 2.0, y + 2.0, -40.0, 0.0))
    out["brackets"] = fuse_all(brk)
    cx = P.K_TIP_SHELF_X
    items = {}
    for sl, (kind, y) in P.K_TIP_SHELF_SLOTS.items():
        if kind.startswith("tower_"):
            tip = kind[6:]
            h = tower_height(tip, tower_max(tip)) - SEAT
        else:
            h = tip_supply_plan()["spent_wafers"] * SPENT_PITCH
        items[sl] = B(cx - WAFER_PLATE["L"] / 2, cx + WAFER_PLATE["L"] / 2, y - WAFER_PLATE["W"] / 2 - 1.5, y + WAFER_PLATE["W"] / 2 + 1.5, t, t + h)
        g = []                                     # TSG-1 corner guides (PA12) keep each stack square, clear of the long-side lips
        for sx in (-1, 1):
            for sy in (-1, 1):
                gx, gy = cx + sx * (WAFER_PLATE["L"] / 2 + 2.0), y + sy * (WAFER_PLATE["W"] / 2 - 5.0)
                g.append(B(gx - 2.0, gx + 2.0, gy - 6.0, gy + 6.0, t, t + 40.0))
        items[sl + "_guides"] = fuse_all(g)
    out["slots"] = items
    return out


def tip_frame(tip):
    """working tip position on the deck: the refill system's own rack base (bought, ASSUMED) on the IN-P locators, one wafer on it"""
    h = P.TIP_RACK_H[tip] if tip in P.TIP_RACK_H else WAFER[tip]["tip_l"] + 8.0
    return B(-P.SBS_L / 2, P.SBS_L / 2, -P.SBS_W / 2, P.SBS_W / 2, SEAT, SEAT + h)


def shelf_gripper_limits():
    fz = P.GRIPPER_FINGERTIP_Z_AT_TOP
    plan = tip_supply_plan()
    lim = {}
    for sl, (kind, y) in P.K_TIP_SHELF_SLOTS.items():
        if kind.startswith("tower_"):
            top = SHELF["t"] + tower_height(kind[6:], tower_max(kind[6:])) - SEAT
        else:
            top = SHELF["t"] + plan["spent_wafers"] * SPENT_PITCH
        lim[sl] = round(fz - (top - WAFER_PLATE["t"] - 1.0), 1)            # fingertips at the top wafer's lips
    return lim


def tip_supply_checks():
    plan = tip_supply_plan()
    t_xy, t_z, t_grip, settle = 4 * 1.0, 4 * 150.0 / 40.0, 4 * 1.5, 2 * 1.04
    return {"chosen": "(b) nested refill wafers in gripper-served towers on a side shelf beside the deck, fixed tip frames on the deck, "
                      "and a spent-wafer stack",
            "options_compared": {
                "a_stacked_racks": "an ordinary rack cannot sit on the one below (its base lands on the tip collars); a stackable class adds "
                                   "a rack height per unit, and the assembly showed a 133 mm stack (2 racks) is hit by the eight-channel "
                                   "cradles working at neighbouring deck positions: 1 rack per deck position, no gain",
                "b_refill_wafers": "a wafer adds only its nesting pitch (ASSUMED 7-12 mm): " + ", ".join(
                    f"{tower_max(t)} {t}" for t in WAFER) + " wafers per 150 mm tower; on a shelf beside the deck no tool works deep "
                    "next to it; the gripper moves one wafer (about 80 g) per exchange; nothing falls or slides: CHOSEN",
                "c_lift_magazine": "a bought lead-screw lift indexing rack carriers to a hand-off height: under the deck the control bay and "
                                   "PSUs leave no column 130 mm wide; above the deck the head passes at z 160, so a magazine there holds 2 "
                                   "racks: more parts (motor, driver, switch) for no more capacity than (b)",
                "d_more_deck_positions": f"{sum(plan['wafers_needed'].values())} racks would need as many positions: several more rows; "
                                         "rejected for tips (kept below for plate capacity)"},
            "shelf": {"part": "TSS-1: 140 x 536 tray of 4 mm 5052, laser-cut and bent (20 mm outer flange), screwed to the deck margin "
                              "(6 x M4) and carried by 3 laser brackets on the right deck side beam outside the control-bay cover; "
                              "lifts off for control-bay access",
                      "slots_global_x": P.K_TIP_SHELF_X, "slots": {k: {"use": v[0], "y": v[1]} for k, v in P.K_TIP_SHELF_SLOTS.items()},
                      "reach": "gripper only (gripper x <= 332; the slots are at x 298); the pipettes never work there",
                      "clearances": "slot units x 251..373: beyond the eight-channel tool plates working column-3 column 12 at depth (to x 246); "
                                    "stack tops <= z 155 under the carriage plate (z 160)"},
            "wafer": WAFER_PLATE, "plan": plan,
            "exchange_path": "1 gripper takes the spent wafer from the deck tip frame (lips, closed at 80 mm) -> 2 places it on the S5 spent "
                             "stack -> 3 takes the top wafer of that type's tower -> 4 lowers it onto the deck frame -> 5 VC-1 tip inventory "
                             "check (ADR-004)",
            "time_per_exchange_s_ESTIMATE": round(t_xy + t_z + t_grip + settle),
            "time_basis": "4 XY moves of 300-450 mm (about 1 s each at 400 mm/s), 4 Z strokes of about 150 mm at 40 mm/s, 4 grips or "
                          "releases of 1.5 s, 2 settles of 1.04 s",
            "chute_for_empties": "rejected: a 122 x 80 wafer has no free drop path under the shelf (control-bay cover, PSU plates); a flat "
                                 "stack of spent wafers in S5 cannot jam and holds the whole run",
            "gripper_depth_limits_s": shelf_gripper_limits()}


# =====================================================================================================
# 6. stacks, VC-2, markers
# =====================================================================================================
STACK = {"tiprack_stackable": dict(unit_h=64.0, pitch=64.0, grip_above_unit_bottom=55.0, n=P.K_STACK_UNITS["tiprack_stackable"]),
         "pcr_plate_full_skirt": dict(unit_h=16.1, pitch=10.0, grip_above_unit_bottom=3.0, n=P.K_STACK_UNITS["pcr_plate_full_skirt"]),
         "deepwell_44": dict(unit_h=44.0, pitch=44.0, grip_above_unit_bottom=3.0, n=P.K_STACK_UNITS["deepwell_44"])}


def stack_height(cls):
    s = STACK[cls]
    return SEAT + (s["n"] - 1) * s["pitch"] + s["unit_h"]


def stack_checks():
    fz_top = P.GRIPPER_FINGERTIP_Z_AT_TOP
    out = {}
    for cls, s in STACK.items():
        rows = []
        for n in range(1, 16):
            top = SEAT + (n - 1) * s["pitch"] + s["unit_h"]
            grip = SEAT + (n - 1) * s["pitch"] + s["grip_above_unit_bottom"]
            rows.append({"units": n, "stack_top_z": round(top, 1), "top_unit_grip_z": round(grip, 1),
                         "below_MAX_LABWARE_H_150": top <= P.MAX_LABWARE_H, "grip_reachable": grip <= fz_top})
        nmax = max(r["units"] for r in rows if r["below_MAX_LABWARE_H_150"] and r["grip_reachable"])
        carried_bottom = fz_top - s["grip_above_unit_bottom"]
        out[cls] = {"max_units": nmax, "used": s["n"], "table": rows, "carried_unit_bottom_at_full_lift_z": round(carried_bottom, 1),
                    "carry_rule": f"while carrying one, pass only over items whose tops are below z {carried_bottom - 5:.0f}"}
    out["tiprack_note"] = ("tip racks stack only as a STACKABLE class whose base nests over the tip collars of the rack below (pitch = rack "
                           "height, ASSUMED 64): a plain rack sits on the collars and is not stackable.  The gripper takes a rack at its "
                           "top frame (band 55-60 mm above the rack bottom, ASSUMED) because its fingers are shorter than a rack")
    out["guides"] = "STK-1: 4 PA12 corner guides on the IN-P (corner-locator holes), away from the long-side grip band"
    return out


VC2 = dict(cam=(-100.0, -66.0, -24.0, 14.0, 52.0, 102.0), light=(52.0, 53.5, -12.0, 2.0, 62.5, 95.0))   # A1 nest coords


def vc2_parts():
    x0, x1, y0, y1, z0, z1 = VC2["cam"]
    cam = B(x0, x1, y0, y1, z0, z1)
    brk = B(x0 - 4.0, x0, y0, y1, P.INSERT_T - 11.0, z1)       # PA12 bracket on the deck margin (2 x M4 into the deck, new holes)
    lx0, lx1, ly0, ly1, lz0, lz1 = VC2["light"]
    light = B(lx0, lx1, ly0, ly1, lz0, lz1)
    return dict(camera=cam, bracket=brk, backlight=light)


def vc2_checks():
    t = DK.TW["tipcheck"]
    return {"camera": "USB UVC board camera 38 x 38 with an M12 lens f 8 mm (ASSUMED), on the -X side of A1 looking +X along the beam, "
                      "optical axis at nest y -5 (the beam plane) and z 77",
            "backlight": "edge-lit LED strip 1.5 x 14 x 32.5 mm on the -X face of the +X gate post, z 62.5..95 (x <= 53.5: 2.3 mm clear of "
                         "HP-2B at A2 column 1)",
            "one_move_serves_both": "during the Y traverse each tip crosses the beam plane; the camera sees that tip in silhouette at the "
                                    "same instant the beam breaks: 1 image per tip (8 for an eight-channel tool, 1 for the small tool)",
            "view": "tip ends z 59.5 up to z 95 (the lower 35 mm of each tip, where liquid sits); working distance about 135 mm "
                    f"(camera face A1-local x -66 to the tip path x {t['path_x']:g}); field about 40 x 30 mm (ASSUMED lens)",
            "pitch_mm": P.WELL_PITCH_96, "axes": 1,
            "tip_sensor_decision": "the geometry needs the SEPARATED THROUGH-BEAM PAIR (TCK-1 posts at A1-local x +/-55..62, beam span about "
                                   "107 mm).  A fork sensor would have to straddle the tip path at x -42 (arms about x -62..-21 up to z ~75), "
                                   "inside the eight-channel strip-move envelope (manifold x -40..-4 above z ~59): the electrical parts list "
                                   "must change its fork sensor to an emitter/receiver pair with >= 120 mm range"}


def marker_global(nest):
    cx, cy = NEST[nest]
    m = DK.MARKER
    holes = [[round(cx + x, 2), round(cy + y, 2)] for x, y in m["holes"]]
    px0, px1, py0, py1 = m["pocket"]
    return {"tag_centre_xy": [round(cx + (px0 + px1) / 2, 2), round(cy + (py0 + py1) / 2, 2)], "hole_xy": holes,
            "hole_d": m["hole_d"], "top_z": SEAT + (WN["gap"] if ST.get(nest) == "weighing" else 0.0)}


# =====================================================================================================
# placement for the assembly
# =====================================================================================================
def placed_items():
    """list of dict(name, part, cls, nest, solid): cls 'part' | 'bought' | 'reserved' | 'labware' (labware is skipped for the
    tool working at its own nest, as the assembly does for every nest)"""
    out = []

    def put(name, part, cls, nest, s, local=True):
        cx, cy = NEST[nest]
        out.append(dict(name=name, part=part, cls=cls, nest=nest, solid=mv(s, cx, cy) if local else s))
    for nest, kind in ST.items():
        if kind == "weighing":
            w = wn_parts()
            for k, part in (("bracket", "WNB-1_cell_bracket"), ("platform", "WNP-1_platform"), ("spacer", "WNS-1_cell_spacer"),
                            ("drip_cover", "WNC-1_drip_cover")):
                put(f"{nest}_{k}", part, "part", nest, w[k])
            put(f"{nest}_cell", "load_cell_1kg_single_point", "bought", nest, w["cell"])
        if kind == "magnet_weighable":
            w = wn_parts(CELL_M)
            put(f"{nest}_cell_m", "load_cell_3kg_single_point_RESERVED", "reserved", nest, w["cell"])
        if kind == "tip_parking":
            p_, tips = tpk_parts()
            for k, part in (("lower", "TPK-1L_lower_plate_p300"), ("upper", "TPK-1U_upper_plate_p300"), ("posts", "TPK-1P_posts"),
                            ("tray", "TPK-1T_drip_tray")):
                put(f"{nest}_tpk_{k}", part, "labware", nest, p_[k])
        if kind == "shaker":
            sp, _ = shaker_parts()
            put(f"{nest}_sk1", "SK-1_shaker_adapter", "labware", nest, sp["adapter"])
            put(f"{nest}_mounts", "antivibration_feet", "labware", nest, sp["mounts"])
            put(f"{nest}_shaker", "shaker_module_ENVELOPE", "labware", nest, sp["body"])
            put(f"{nest}_plate", "labware_pcr_plate", "labware", nest, sp["plate"])
            put(f"{nest}_lid", "LD-1_plate_lid", "labware", nest, sp["lid"])
        if kind.startswith("tip_frame_"):
            tip = kind[len("tip_frame_"):]
            put(f"{nest}_tipframe", f"tip_frame_{tip}_with_wafer", "labware", nest, tip_frame(tip))
        if kind.startswith("stack_"):
            cls = "tiprack_stackable" if kind == "stack_tiprack" else "pcr_plate_full_skirt"
            h = stack_height(cls)                       # rev K: plate stacks of 4 (51 mm)
            put(f"{nest}_stack", f"labware_stack_{cls}", "labware", nest, B(-P.SBS_L / 2, P.SBS_L / 2, -P.SBS_W / 2, P.SBS_W / 2, SEAT, h))
        if kind == "lid_park_2":
            z = SEAT + DK.LS["h"] + 4.6
            put(f"{nest}_parked_LD1", "LD-1_plate_lid", "labware", nest, B(-LID["L"] / 2, LID["L"] / 2, -LID["W"] / 2, LID["W"] / 2, z, z + LID["H"]))
        if kind == "balance":
            b = bal_parts()
            for k, part, cls in (("insert", "IN-B_balance_insert", "part"), ("shield", "DS-1_draught_shield", "labware"),
                                 ("pan", "balance_pan", "labware"), ("stem", "balance_stem", "bought"), ("cell", "balance_cell_ENVELOPE", "bought"),
                                 ("stand_legs", "BST-1_stand_legs", "part"), ("stand_top", "BST-1_stand_plate", "part")):
                out.append(dict(name=f"{nest}_bal_{k}", part=part, cls=cls, nest=nest, solid=b[k]))
    ts = tip_supply_parts()
    out.append(dict(name="TSS_shelf", part="TSS-1_tip_supply_shelf", cls="part", nest="S0", solid=ts["shelf"]))
    out.append(dict(name="TSS_brackets", part="TSB-1_shelf_bracket", cls="part", nest="S0", solid=ts["brackets"]))
    for sl, sol in ts["slots"].items():
        out.append(dict(name=f"TSS_{sl}", part=("TSG-1_stack_guide" if sl.endswith("_guides") else f"tip_supply_{sl}"),
                        cls=("part" if sl.endswith("_guides") else "labware"), nest=sl.split("_")[0], solid=sol))
    v = vc2_parts()
    for k, part, cls in (("camera", "VC-2_camera_ENVELOPE", "bought"), ("bracket", "VC-2B_camera_bracket", "part"),
                         ("backlight", "VC-2L_backlight_strip", "bought")):
        put(f"A1_vc2_{k}", part, cls, "A1", v[k])
    return out


GRIP_S_MAX = {}       # nest -> deepest gripper stroke (fingertips at the top unit's grip band), filled in main()


def gripper_limits():
    fz = P.GRIPPER_FINGERTIP_Z_AT_TOP
    lim = {}
    for nest, kind in ST.items():
        if kind.startswith("stack_"):
            cls = "tiprack_stackable" if kind == "stack_tiprack" else "pcr_plate_full_skirt"
            s = STACK[cls]
            grip = SEAT + (s["n"] - 1) * s["pitch"] + s["grip_above_unit_bottom"]
            lim[nest] = round(fz - grip, 1)
        if kind == "shaker":
            _, zs = shaker_parts()
            lim[nest] = round(fz - (zs + 3.0), 1)
        if kind == "lid_park_2":
            lim[nest] = round(fz - (SEAT + DK.LS["h"] + 4.6 + 3.0), 1)
        if kind.startswith("tip_frame_"):         # fingertips at the wafer lips on the frame
            tip = kind[len("tip_frame_"):]
            lim[nest] = round(fz - (SEAT + P.TIP_RACK_H.get(tip, 60.0) - WAFER_PLATE["t"] - 1.0), 1)
        if kind == "weighing":                      # fingertips stay 2 mm above the floating IN-W (it is 0.5 mm higher)
            lim[nest] = round(fz - (SEAT + WN["gap"] + 2.0), 1)
    return lim


NO_GRIPPER = [n for n, k in ST.items() if k in ("tip_parking", "balance", "weighing_pipette_only")]
PIPETTES_SKIP = [n for n, k in ST.items() if k.startswith("stack_") or k in ("lid_park_2", "shaker")]   # the shaker is gripper-only


# =====================================================================================================
def main():
    OUT.mkdir(parents=True, exist_ok=True)
    parts, valid = {}, {}

    def export(name, s, proc, mat, qty, stl=False, dxf=False):
        cq.exporters.export(cq.Workplane().add(s), str(OUT / f"{name}.step"))
        if stl:
            cq.exporters.export(cq.Workplane().add(s), str(OUT / f"{name}.stl"), tolerance=0.05, angularTolerance=0.2)
        if dxf:
            cq.exporters.export(cq.Workplane().add(s).faces("<Z"), str(OUT / f"{name}.dxf"))
        bb = s.BoundingBox()
        valid[name] = s.isValid()
        dens = RHO["pa12"] if proc.startswith("print") else RHO["al"]
        parts[name] = {"name": name, "process": proc, "material": mat, "qty": qty, "valid_solid": s.isValid(),
                       "envelope_mm": [round(bb.xlen, 1), round(bb.ylen, 1), round(bb.zlen, 1)], "mass_kg": round(s.Volume() * dens, 4)}
    w = wn_parts()
    n_wn = sum(1 for k in ST.values() if k == "weighing")
    export("WNB-1_cell_bracket", w["bracket"], "sheet_laser", "Al 5052-H32 6 mm, 3 tapped holes", n_wn, dxf=True)
    export("WNP-1_platform", w["platform"], "sheet_laser", "Al 5052-H32 6 mm", n_wn, dxf=True)
    export("WNS-1_cell_spacer", w["spacer"], "cnc", "Al 6061 block 12 x 12.7 x 6.5", n_wn)
    export("WNC-1_drip_cover", w["drip_cover"], "print_mjf_pa12", "PA12 (MJF)", n_wn, stl=True)
    export("IN-W_weighing_insert", w["insert"], "sheet_laser", "Al 5052-H32 5 mm (IN-P outline, screws into WNP-1)", n_wn, dxf=True)
    pcb = pcb_part()
    export("PCB-1_passive_cold_strip_carrier", pcb, "cnc", "Al 6061-T6, hard anodised", 1)
    tp, _ = tpk_parts()
    export("TPK-1L_lower_plate_p300", tp["lower"], "sheet_laser", "Al 5052-H32 8 mm", 1, dxf=True)
    export("TPK-1U_upper_plate_p300", tp["upper"], "sheet_laser", "Al 5052-H32 6 mm", 1, dxf=True)
    export("TPK-1P_posts", tp["posts"], "print_mjf_pa12", "PA12 (MJF), 4 posts", 1, stl=True)
    export("TPK-1T_drip_tray", tp["tray"], "print_mjf_pa12", "PA12 (MJF) + absorbent pad", 1, stl=True)
    sp, zs = shaker_parts()
    export("SK-1_shaker_adapter", sp["adapter"], "sheet_laser", "Al 5052-H32 6 mm", 1, dxf=True)
    b = bal_parts()
    cx, cy = NEST["F2"]
    export("IN-B_balance_insert", mv(b["insert"], -cx, -cy), "sheet_laser", "Al 5052-H32 5 mm", 1, dxf=True)
    export("DS-1_draught_shield", mv(b["shield"], -cx, -cy), "print_mjf_pa12", "PA12 (MJF)", 1, stl=True)
    export("BST-1_stand_plate", mv(b["stand_top"], -cx, -cy), "sheet_laser", "Al 5052-H32 3 mm", 1, dxf=True)
    v = vc2_parts()
    export("VC-2B_camera_bracket", v["bracket"], "print_mjf_pa12", "PA12 (MJF)", 1, stl=True)
    guide = B(-P.SBS_L / 2 - 4.0, -P.SBS_L / 2 + 6.0, -P.SBS_W / 2 - 4.0, -P.SBS_W / 2 + 6.0, SEAT, SEAT + 140.0).cut(
        B(-P.SBS_L / 2 - 0.3, -P.SBS_L / 2 + 7.0, -P.SBS_W / 2 - 0.3, -P.SBS_W / 2 + 7.0, SEAT - 1, SEAT + 141.0))
    export("STK-1_stack_corner_guide", guide, "print_mjf_pa12", "PA12 (MJF), 4 per stack", 4 * sum(1 for k in ST.values() if k.startswith("stack_")), stl=True)
    lim = gripper_limits()
    GRIP_S_MAX.update(lim)
    ts = tip_supply_parts()
    export("TSS-1_tip_supply_shelf", ts["shelf"], "sheet_laser_bent", "Al 5052-H32 4 mm, 1 bend", 1)
    export("TSB-1_shelf_bracket", B(0.0, 138.0, 0.0, 40.0, 0.0, 4.0), "sheet_laser", "Al 5052-H32 4 mm (138 x 40 flat; stands on edge)", 3, dxf=True)
    export("TSG-1_stack_guide", B(-2.0, 2.0, -6.0, 6.0, 0.0, 40.0), "print_mjf_pa12", "PA12 (MJF), 4 per shelf slot", 4 * len(P.K_TIP_SHELF_SLOTS), stl=True)
    export("LW-1_twin_waste_funnel", lw_parts(), "print_mjf_pa12", "PA12 or PP (MJF), solvent/hazardous-liquid resistant", 1, stl=True)
    m_pcb = pcb.Volume() * RHO["al"]
    rep = {"module": "LH-1 revision K deck stations", "status": "CAD and calculation only; nothing built or measured",
           "stations": ST, "parts": list(parts.values()), "all_valid": all(valid.values()),
           "bought": [{"item": CELL["spec"], "qty": n_wn}, {"item": "HX711-class amplifier board with plug-in lead", "qty": n_wn},
                      {"item": "shaker module, class " + SHAKER_USE + " (" + SHAKER[SHAKER_USE]["temp"] + ") + its controller", "qty": 1},
                      {"item": "sorbothane-class anti-vibration feet M4, about 2 N/mm each (ASSUMED)", "qty": 4},
                      {"item": "SBS universal plate lid, clear, condensation rings (gripper lid LD-1)", "qty": 2},
                      {"item": BAL_CELL["spec"], "qty": 1}, {"item": "20 x 20 extrusion cut to length (BST-1 legs)", "qty": 4},
                      {"item": "USB UVC board camera 38 x 38 + M12 lens f 8 mm (VC-2)", "qty": 1},
                      {"item": "edge-lit LED strip backlight 32 x 14 mm, 12 or 24 V (VC-2)", "qty": 1},
                      {"item": "closed-cell foam sleeve 10 mm for PCB-1", "qty": 1},
                      {"item": "HDPE 2 L waste bottle with 2-port cap + carbon vent cartridge (LW-1, K2)", "qty": 2},
                      {"item": "FEP tube 8 mm ID, 2 m (LW-1 drains)", "qty": 1},
                      {"item": "capacitive non-contact level switch 24 V PNP (waste bottles)", "qty": 2}],
           "checks": {"tip_supply": tip_supply_checks(), "weighing_nest_WN-1": wn_checks(), "magnet_block_WN-M": {**wn_checks(CELL_M), "feasible": True,
                      "note": "MB-1 is passive (no cable on the weighed part): feasible on a 3 kg cell; resolution about 3 x coarser than WN-1 "
                              "(same amplifier); RESERVED volume only in rev K, decide after the bench rig test (ADR-005)"},
                      "passive_cold_block_PCB-1": pcb_hold_time(m_pcb), "tip_parking_TPK-1": tpk_checks(), "shaker": shaker_checks(zs),
                      "balance_F2": bal_checks(), "stacks": stack_checks(), "tip_station_VC-2": vc2_checks(),
                      "gripper_depth_limits_s": lim},
           "markers": {n: marker_global(n) for n in NEST},
           "adr008_study": {"pipettes_and_reach": reach_table(), "fifth_axis": fifth_axis_study(), "cold_tube_thermal": cold_tube_thermal(),
                            "tube_layout": tube_layout_proposal(), "tubes_ASSUMED": TUBES, "pipettes_ASSUMED": PIPETTES_K},
           "item10": item10(), "k2_layout": {"places": {n: {"kind": v[0], "height_mm": v[1], "pipette_work": v[2],
                                                             "centre_xy": [K2_COLS[int(n[1])], K2_ROWS[n[0]]]} for n, v in K2_PLACES.items()},
                                              "dock": K2_DOCK, "frame": K2_FRAME, "rule_check": k2_rules()},
           "headroom": headroom(), "control_spares": control_spares(), "module_standard": module_standard(),
           "survey_adjustments": survey_adjustments()}
    rep["adr008_study"]["fifth_axis"] = {"status": "DROPPED for revision K (ADR-010: pipettes exchanged at a dock; head = revision L)",
                                         "earlier_study_kept_for_reference": rep["adr008_study"]["fifth_axis"]}
    rep["adr008_study"]["tube_layout"]["superseded_by"] = "k2_layout (tubes in row F, tip frames in row G, dock behind G)"
    (OUT / "report.json").write_text(json.dumps(rep, indent=1, default=str))
    return rep



# =====================================================================================================
# 9. ADR-008: large single-channel pipette and tube zones (STUDY: calculations and concept parts; placement needs revision L)
# =====================================================================================================
TUBES = {   # ASSUMED nominal tube geometry (confirm against the tubes bought): id_top, length, cone_h, cone_tip_id, max_ml, od
    "1.5mL": dict(id_top=8.7, length=39.0, cone_h=17.0, cone_tip_id=2.0, max_ml=1.5, od=10.8),
    "2mL": dict(id_top=8.9, length=40.0, cone_h=9.0, cone_tip_id=5.0, max_ml=2.0, od=10.8),
    "5mL": dict(id_top=15.5, length=57.0, cone_h=12.0, cone_tip_id=4.0, max_ml=5.0, od=17.0),
    "15mL_conical": dict(id_top=14.5, length=120.0, cone_h=22.0, cone_tip_id=2.5, max_ml=15.0, od=17.0),
    "50mL_conical": dict(id_top=27.5, length=115.0, cone_h=20.0, cone_tip_id=4.0, max_ml=50.0, od=29.5)}
PIPETTES_K = {   # ASSUMED geometry below the nozzle end: tip length; the pipette shaft profile above the nozzle (height above nozzle, OD)
    "pipette_small": dict(range_ul=(0.5, 10), tip="p10", tip_l=P.TIP_BELOW_NOZZLE["p10"], tip_od_top=5.9,
                          shaft=[(0.0, 4.5), (20.0, 7.0), (60.0, 28.0)]),
    "pipette_8ch_50": dict(range_ul=(5, 50), tip="p200", tip_l=P.TIP_BELOW_NOZZLE["p200"], tip_od_top=7.2, shaft=[(0.0, 5.0), (14.0, 32.0)]),
    "pipette_8ch_300": dict(range_ul=(30, 300), tip="p300", tip_l=P.TIP_BELOW_NOZZLE["p300"], tip_od_top=7.4, shaft=[(0.0, 5.2), (18.0, 36.0)]),
    "pipette_1000": dict(range_ul=(100, 1000), tip="p1000", tip_l=P.TIP_BELOW_NOZZLE["p1000"], tip_od_top=9.0,
                         shaft=[(0.0, 8.0), (30.0, 11.0), (60.0, 26.0)])}


def _tube_vol_below(t, h):
    """uL in tube t below height h above its inside bottom (truncated cone + cylinder)"""
    r0, r1, hc = t["cone_tip_id"] / 2, t["id_top"] / 2, t["cone_h"]
    if h <= hc:
        rh = r0 + (r1 - r0) * h / hc
        return math.pi * h / 3 * (r0 * r0 + r0 * rh + rh * rh)
    return math.pi * hc / 3 * (r0 * r0 + r0 * r1 + r1 * r1) + math.pi * r1 * r1 * (h - hc)


def reach_table():
    out = {}
    for tool, p in PIPETTES_K.items():
        rows = {}
        for tn, t in TUBES.items():
            # the deepest point: tip end at the tube bottom unless the pipette shaft (or the tip collar) meets the tube mouth first
            entry = p["tip_l"]
            for h_above, od in p["shaft"]:
                if od > t["id_top"] - 0.5:
                    entry = p["tip_l"] + h_above
                    break
            else:
                entry = p["tip_l"] + p["shaft"][-1][0]
            if p["tip_od_top"] > t["id_top"] - 0.5:
                entry = min(entry, p["tip_l"])
            short = max(0.0, t["length"] - 1.0 - entry)          # 1 mm wall at the bottom
            vmin = _tube_vol_below(t, short + 3.0) / 1000.0        # tip end 3 mm under the surface
            rows[tn] = {"reach_depth_mm": round(min(entry, t["length"] - 1.0), 1), "bottom_reached": short <= 0.0,
                        "min_volume_mL": round(vmin, 2) if short > 0 else round(_tube_vol_below(t, 3.0) / 1000.0, 3),
                        "shaft_enters_bore": any(od <= t["id_top"] - 0.5 for _, od in p["shaft"][1:])}
        out[tool] = {"range_uL": p["range_ul"], "tip": p["tip"], "tip_below_nozzle_mm": p["tip_l"], "tubes": rows}
    return out


def cold_tube_thermal():
    k_w, h_air, t_pp, k_pp = 0.6, 8.0, 1.0e-3, 0.2
    res = {}
    for tn in ("1.5mL", "5mL", "15mL_conical"):
        t = TUBES[tn]
        D = t["id_top"] * 1e-3
        A = math.pi * D * D / 4
        U = 1 / (1 / h_air + t_pp / k_pp)
        m = math.sqrt(U * math.pi * (D + 2 * t_pp) / (k_w * A))
        lim = {}
        for amb in (25.0, 30.0):
            c = (amb - 4.0) / (amb - 6.0)
            lim[f"{amb:g}C"] = round(math.acosh(c) / m * 1000, 1)
        q = {f"{amb:g}C": round(k_w * A * m * (amb - 4.0) * math.tanh(m * 6e-3), 4) for amb in (25.0, 30.0)}
        res[tn] = {"max_unenclosed_liquid_mm (liquid top <= 6 C)": lim, "heat_per_tube_W_at_6mm_exposed": q}
    load = {}
    for amb in (25.0, 30.0):
        tubes = 24 * res["1.5mL"]["heat_per_tube_W_at_6mm_exposed"][f"{amb:g}C"] + 4 * res["5mL"]["heat_per_tube_W_at_6mm_exposed"][f"{amb:g}C"] \
            + 2 * res["15mL_conical"]["heat_per_tube_W_at_6mm_exposed"][f"{amb:g}C"] + 1 * 2.5 * res["15mL_conical"]["heat_per_tube_W_at_6mm_exposed"][f"{amb:g}C"]
        walls = 2 * (2.6 * 2 * (0.128 + 0.086) * 0.080) * (amb - 4.0)          # 2 blocks, 10 mm foam sides (U 2.6)
        top = 2 * 10.0 * 0.128 * 0.086 * 0.5 * (amb - 4.0)                    # half the block top is metal between open tubes
        evap = 36 * 0.002                                                       # about 2 mW latent per open mouth (ASSUMED)
        load[f"{amb:g}C"] = round(tubes + walls + top + evap, 2)
    m_block = 2 * 1.9                                                          # kg, two 128 x 86 x 80 Al blocks with pockets (CALCULATED order)
    C = m_block * 900.0 + 24 * 1.0e-3 * 4186 + 4 * 4.0e-3 * 4186 + 2 * 12e-3 * 4186 + 16e-3 * 4186
    G_off = 2 * 2 * 0.45 + (load["25C"] / 21.0)                                # TECs off conduct about 0.45 W/K each (ASSUMED) + the leak above
    t8 = C / G_off * math.log((25.0 - 4.0) / (25.0 - 8.0))
    return {"assumptions": "liquid column above the block conducts (warm water over cold water is stable: no convection); tube wall PP 1 mm; "
                           "air h 8 W/m2K; caps open; block 4 C; tubes loaded cold",
            "per_tube": res, "steady_load_W_two_blocks": load,
            "cooling": "two thermal modules of the rev K kind (2 x TEC1-12706 at 3.5 A / 18 V, the bought 0.21 K/W sink), one per block: "
                       "each carries half the load above (about 3.7 / 4.6 W at 25 / 30 C) against the pumped heat at 4 C in "
                       "deck_modules cold_block_rev_j (about 24 W at 30 C: margin about 5, CALCULATED); the 80 mm block adds mass, not load",
            "block_height": "pockets 80 mm deep: a 15 mL tube is cold to 86 mm of liquid (about 12 mL), a 5 mL tube full, 1.5/2 mL full; the "
                            "liquid must not stand more than the limits above over the block top",
            "time_to_8C_if_cooling_stops_min": round(t8 / 60.0, 1),
            "condensation": "block top sloped 1 deg to a drip channel and an absorbent strip on the insert (replaced per run); closed-cell "
                            "foam on the sides; the NTC well is sealed",
            "sensor": "an NTC in a water-filled 1.5 mL dummy tube in one pocket of each block (reads liquid, not metal), plus the block NTC "
                      "the module already has",
            "caps": "tubes stand open during a run (no simple cap handling found: the gripper cannot unscrew); question for the planner: "
                    "evaporation from an open 15 mL mouth (about 170 mm2) over the run at 4 C and at 25 C, per reagent"}


def fifth_axis_study():
    axis_kg = 0.029 + 0.018 + 0.004 + 0.159 + 0.107 + 2 * 0.054 + 0.203 + 0.36 + 0.015 + 0.118 + 0.02 + 0.014 + 0.004 + 0.003
    cradle_kg = 0.25 + 0.10 + 0.02 + 0.02 + 0.15 + 0.004
    pip_kg, plate_ext_kg, stiff_kg = 0.10, 0.32, 0.34
    add = axis_kg + cradle_kg + pip_kg + plate_ext_kg + stiff_kg
    m_head, m_y = 13.55, 19.3
    return {"options": {
                "fifth_Z_axis": "a fifth copy of the existing Z axis (same rail, motor, screw, ZC-1, TP-1) and a CB-1-type cradle with the "
                                "same NEMA11 plunger actuator, at carriage-local x +185 (74 mm pitch): every part is one the builder "
                                "already makes or buys four of",
                "exchange_at_a_dock": "the small pipette and the 1,000 uL pipette exchanged on axis 0: needs a repeatable kinematic "
                                      "coupling, a latch the gantry can work, and a blind-mating connector for the actuator and two "
                                      "switches; a dock position on the deck; new mechanics an amateur cannot check without gauges"},
            "chosen": "fifth Z axis (simpler to build: no coupling, latch or connector mating)",
            "mass_added_kg_CALCULATED": round(add, 2), "head_kg": round(m_head + add, 2),
            "Y_belt_mode_Hz": f"15.2 -> {15.2 * math.sqrt(m_y / (m_y + add)):.1f} (f ~ 1/sqrt(m))",
            "X_belt_mode_worst_Hz": f"10.9 -> {10.9 * math.sqrt(m_head / (m_head + add)):.1f}; X settle 1.045 -> "
                                    f"{1.045 * math.sqrt((m_head + add) / m_head):.2f} s: the 1.04 s dwell must rise to that, or the X belt "
                                    "is doubled (EA x 2: modes x 1.41)",
            "X_travel_and_frame": "the carriage plate grows +74 mm at +X: to keep the X travel (+/-295) the right Y carriage, X brackets "
                                  "and side beam move +74 mm: frame +74 mm wide, X beam 980 -> 1054 mm",
            "reach_of_the_1000uL_tool": "carriage x -295..295 + 185: x -110..480: columns 2 and 3, the tip-supply shelf and a column beyond "
                                        "it; NOT column 1 (A1, B1, C1, D1, F1)",
            "M1": "single channel, pick-up about 30 N (ASSUMED for 1,000 uL tips): lateral about 0.06 mm against 0.5 (the 20 N small tool "
                  "is 0.042 mm); the 150 N frame share is unchanged; the outer axis adds about 5 % to the roll term",
            "status": "STUDY: not modelled in rev K (it changes N_TOOLS, the carriage plate, X beam and frame width: revision L)"}


def tube_layout_proposal():
    return {"basis": "planner requirements derivation (upper bound, 14 Nov 2026): cold 24 x 1.5/2 mL, 4 x 5 mL, 2 x 15 mL, 1 x 50 mL; ambient "
                     "12 x 1.5/2 mL, 6 x 5 mL, 4 x 15 mL, 4 x 50 mL; T7 buffer must NOT be chilled (ambient 1.5 mL next to the cold block); "
                     "liquids over 50 mL stay in reservoirs",
            "cold": {"blocks": {"CTB-1 (G2)": "24 x 1.5/2 mL (6 x 4 at 13 mm) + 4 x 5 mL (2 x 2 at 19 mm)",
                                "CTB-2 (G3)": "2 x 15 mL (at 20 mm) + 1 x 50 mL (D30 pocket): the 16 mL single-cell PCR mix stands 40 mm high in "
                                              "a 50 mL tube, inside the 80 mm pocket"},
                     "mounting": "each block replaces TM-3 on a rev K thermal module (same 4 clamp screws): blocks are interchangeable, so the "
                                 "size mix changes by swapping blocks", "block": "Al 6061, 128 x 86 x 80, pockets 80 deep, foam sides"},
            "ambient": {"ATR-2 (G1)": "12 x 1.5/2 mL (6 x 2 at 13 mm) + 6 x 5 mL (3 x 2 at 19 mm): PA12 on an IN-P beside CTB-1; the T7 "
                                      "buffer goes here (ambient, next to the cold block)",
                        "ATR-1 (outer shelf column x 440)": "4 x 50 mL (2 x 2 at 33 mm) + 4 x 15 mL (2 x 2 at 20 mm): reached by the 1,000 uL "
                                                            "tool only (x <= 480)",
                        "weighable": "ATR-2 on a WN-1 (rack + tubes about 0.2 kg: inside the 1 kg cell); ATR-1 is too heavy full (4 x 50 mL "
                                     "= 0.2 kg tubes + up to 0.2 kg liquid + rack: about 0.6 kg: inside 1 kg, possible)"},
            "where": "back row G (three positions, y 275): G1 ATR-2, G2 CTB-1, G3 CTB-2; ATR-1 on a second shelf column at x 440 beside "
                     "the tip shelf",
            "frame_growth": "+110 mm deep (row G, as rev H at the front) and +74 mm wide (fifth Z axis): envelope about 1204 x 941 x 751 mm "
                            "(ESTIMATE); mass about +8 kg (frame +5.6, fifth axis +2.4); Y travel +110 at the back",
            "reach": "G1 at x -132: small and eight-channel tools; the 1,000 uL tool only over its right half (x >= -110); G2/G3: all "
                     "pipettes; ATR-1: 1,000 uL tool only",
            "keep_outs": "15 mL tubes stand 40 mm proud of an 80 mm block (top about z 140) and 50 mL tubes about 120 mm: nothing taller "
                         "than about 64 mm may stand beside a position where an eight-channel tool works at column 1/12; row G is behind "
                         "D (D1 TC lid park, D3 lid park: no pipette works there; D2 the p300 frame: the eight-channel's rear parts pass "
                         "over G2 at depth: CHECK in revision L, or move the p300 frame)",
            "markers": "each block and rack carries the 12 mm tag pocket and two D2 holes in its back margin (same pattern as the inserts)",
            "tip_supply_1000uL": "a fixed rack of 96 x 1,000 uL tips on the outer shelf column (x 440), reached only by the 1,000 uL tool "
                                 "(loaded at the start; ASSUMED enough for the mixes of one run until the planner counts); no parking for "
                                 "1,000 uL tips (one tip per ingredient)"}



# =====================================================================================================
# 10. Item 10 (audit findings X1-X13, Y1-Y17), ADR-009 headroom and ADR-010 deck preparation.
#     The frame is sized ONCE here as layout "K2".  CALCULATION AND CONCEPT: the 3D frame, deck and assembly in this
#     revision are still the rev H/K frame (params.py may only gain values; TRAVEL_Y, DECK_W etc. are unchanged).
#     The head is the present four-tool head, kept as a PLACEHOLDER (ADR-010: exchange head = revision L).
# =====================================================================================================
K2_COLS = {1: -132.0, 2: 0.0, 3: 132.0, 4: 264.0}
K2_ROWS = {"F": -275.0, "A": -165.0, "B": -55.0, "C": 55.0, "D": 165.0, "G": 275.0}
# kind, height above the seat (mm, ASSUMED class value), pipette work at the place: "deep" (into wells/tubes), "shallow" (tip pick /
# eject / free dispense at the top), "centre" (dispense at the centre only, e.g. waste), None (gripper only or nothing)
K2_PLACES = {
    "F1": ("tube_rack_ambient ATR-1 (6 x 50 mL + 6 x 15 mL), recessed 60 mm into the window (under-deck class 60)", 100.0, "deep_single_only"),
    "F2": ("balance (under-deck bay, draught shield)", 60.0, "shallow"),
    "F3": ("MODULE ZONE (F3+F4, connector): cold_tube_block CTB-1 (48 x 1.5/2 mL) on a rev K thermal module", 82.0, "deep_single_only"),
    "F4": ("MODULE ZONE (F3+F4, connector): cold_tube_block CTB-2 (6 x 5, 4 x 15, 3 x 50 mL), recessed 60 mm", 100.0, "deep_single_only"),
    "A1": ("tip_waste (funnel, gate, VC-2) -> vertical chute -> under-base bin", 0.0, "shallow"),
    "A2": ("tube_rack_ambient ATR-2 (24 x 1.5/2 + 10 x 5 mL; T7 buffer), recessed 15 mm so its top stays under the 64 mm K19 limit "
           "beside A3", 55.0, "deep_single_only"),
    "A3": ("warm position: thermal module at 37 C, trough or tube block (hazardous liquid)", 44.0, "deep"),
    "A4": ("liquid_waste LW-1 (two streams)", 45.0, "centre"),
    "B1": ("weighing WN-1 (labware)", 44.0, "deep"),
    "B2": ("magnet MB-1 (labware)", 16.0, "deep"),
    "B3": ("cold plate position CPB-1, full 96-well plate (labware)", 30.0, "deep"),
    "B4": ("magnet 2: low-elution magnet, post-PCR (labware)", 16.0, "deep"),
    "C1": ("thermocycler TC-1 (labware)", 21.0, "deep"),
    "C2": ("tip parking TPK-1 (pre-PCR tips only)", 50.0, "shallow"),
    "C3": ("cold strip carrier, rev K module (labware)", 23.5, "deep"),
    "C4": ("plate stack x4, gripper only (labware)", 51.0, None),
    "D1": ("TC-1 lid park", 40.0, None),
    "D2": ("plate stack x4, gripper only (labware)", 51.0, None),
    "D3": ("lid park 2: shaker lid, sealing lid and 3 rescue lids (5 x 9.5 = 47.5 mm)", 47.5, None),
    "D4": ("shaker (heated, cooled; second shaker reserve = C2) (labware)", 130.0, None),
    "G1": ("tip frame p10 (gripper-refilled)", 50.0, "shallow"),
    "G2": ("tip frame p200", 64.0, "shallow"),
    "G3": ("tip frame p300", 72.0, "shallow"),
    "G4": ("tip frame p1000 (the fourth tip type, ADR-010 study 4.5), rack recessed 50 mm", 100.0, "shallow")}
# wide-bore p200 tips: racks exchanged into the G2 frame by the gripper from a shelf slot when a cell step needs them
K2_HEAD_ASSUMED = {"source": "ADR-010 study result (evidence/external/exchange_head_study_2026-11-14.txt), ASSUMED for deck-level checks",
                   "axes": "two identical tool carriages + gripper axis at about 80 mm pitch (local x -80, 0, +80 ASSUMED)",
                   "width_mm": 250.0, "mass_kg": 11.0, "tool_x_width_mm": 66.0}
K2_DOCK = {"where": "one row of 8 bays BEHIND row G along X at 72 mm pitch (study 4.4): bay centres x -186..+318, y about 393; bay "
                    "depth about 110, height 200 (ASSUMED); the frame and Y travel grow 150 mm at the back for it (study figure)",
           "reach": "carriage range x -190..+450 (X travel 640): carriage at local -80 reaches tool x -270..370, carriage at 0 reaches "
                    "-190..450: every bay (-186..318) is reachable by BOTH carriages (CALCULATED from the ASSUMED head)",
           "clear_of_labware": "the bays stand behind row G (row G back edge y 318, dock front y 338): tools enter from above; row G tip "
                               "frames are passed over at transit height",
           "design": "NOT designed (coupling, latch posts, bay cradles: revision L); reserved space on the back frame member",
           "first_build_tools (ADR-010 market adjustments)": ["eight-channel 5-50 uL", "eight-channel 30-300 uL", "single 0.5-10 uL",
                                                              "single 10-100 uL", "single 100-1,000 uL", "syringe dispense tool (if the rig "
                                                              "test passes)", "free", "free (pressure-sensing channel later)"],
           "cable_bay": "bay 8 (x +318, the end bay) has a reserved route for a permanently connected cable: a 20 x 40 mm channel along the "
                        "back frame member to the control bay and space for a cable retractor above the bay (a bought pressure-sensing "
                        "channel); every other tool is passive, with no electrical contact through the coupling",
           "edge_hand_off": {"H1": [510.0, -275.0], "H2": [510.0, -165.0],
                             "dimensions": {"nest": "SBS footprint on the outer shelf column, one plate footprint (about 130 mm) beyond "
                                                    "the deck edge, plate seat at deck height (z 7)",
                                            "gripper": "reaches the nest centre at x 510 (tool x up to 530 with the ASSUMED head: 20 mm "
                                                       "margin) at its normal plate height",
                                            "panel_opening": "160 (Y) x 90 (Z) mm in the right panel at each nest, plate seat 15 mm above "
                                                             "the opening's lower edge, sliding PC shutter on the door interlock",
                                            "bench_space": "clear bench 600 x 600 mm beside the right panel for an instrument whose own "
                                                           "plate carrier reaches 70 mm through the opening to the nest"},
                             "note": "the gantry cannot carry the gripper outside its own frame (the Y carriages are at the frame sides), "
                                     "so 'beyond the frame' is met as beyond the deck edge, inside the side bay, at the panel"}}
K2_FRAME = {"columns": 4, "rows": 6, "places": 24, "X_carriage_travel_mm": 640.0, "X_travel_was": 590.0, "Y_travel_mm": 770.0,
            "Y_travel_was": 510.0, "X_beam_mm": 1170.0, "X_beam_was": 980.0, "Y_rails_mm": 920.0, "Y_rails_was": 660.0,
            "growth": "Y +260 at the back (row G +110, dock +150), width +198 (fourth column + second shelf column with the hand-offs), "
                      "height +140 (under-base drawer feet)",
            "tool_point_range": "x -270..+530 (column 1 wells to the hand-offs), y -310..+400 (row F front wells to the dock)",
            "envelope_mm_ESTIMATE": [1330, 1091, 891], "envelope_was_mm": [1130, 831, 751],
            "shelf": "inner column x 400 (tip towers, gripper only), outer column x 510 (hand-offs H1/H2 at the front, spent wafers, "
                     "wide-bore p200 racks)"}


K2_R2_KEEPOUT = {}


def k2_rules():
    """Rule check of the K2 layout with the rev K clearance rules (not a CAD interference run):
    R1 behind (+Y) a place worked DEEP: height <= 51 mm (rev K stack cap from the head's rear parts); behind SHALLOW work <= 72;
    R2 beside (X) a place an eight-channel tool works deep at column 1/12: height <= 64 (K19);
    R3 a frame used for single-tip pick-up on an eight-channel tool: the place behind (+Y) <= frame height - 5 (bare nozzles)."""
    res, bad = [], []
    for n, (kind, h, work) in K2_PLACES.items():
        r, c = n[0], int(n[1])
        rows = list(K2_ROWS)
        behind = f"{rows[rows.index(r) + 1]}{c}" if rows.index(r) + 1 < len(rows) else None
        if work and behind:
            lim = 51.0 if work.startswith("deep") else 72.0
            hb = K2_PLACES[behind][1]
            ok = hb <= lim
            res.append({"rule": "R1", "place": n, "behind": behind, "height": hb, "limit": lim, "ok": ok})
            ok or bad.append(res[-1])
        if work == "deep":            # eight-channel work; "deep_single_only" places are worked by single channels only
            for cc in (c - 1, c + 1):
                nb = f"{r}{cc}"
                if (n, nb) in K2_R2_KEEPOUT:
                    res.append({"rule": "R2", "place": n, "beside": nb, "ok": True, "resolved_by": K2_R2_KEEPOUT[(n, nb)]})
                    continue
                if nb in K2_PLACES:
                    hb = K2_PLACES[nb][1]
                    ok = hb <= 64.0
                    res.append({"rule": "R2", "place": n, "beside": nb, "height": hb, "limit": 64.0, "ok": ok})
                    ok or bad.append(res[-1])
    return {"rules": k2_rules.__doc__.split("\n", 1)[1].strip(), "n_checked": len(res), "violations": bad,
            "single_tip_pick_R3": "row G frames: the place behind is the dock front margin (no labware): bare nozzles clear (frames are "
                                  "the back row for this reason)",
            "F4_note": "CTB-2 (160 mm with 50 mL tubes) is worked by single channels only; F3 beside it is worked by an eight-channel "
                       "tool only away from its +X edge (columns 1-10): planner keep-out rule",
            "status": "rule check only; the CAD interference run of K2 is revision L with the exchange head"}


def lw_parts():
    """LW-1 twin waste funnel (X1, Y16): two hoppers 56 x 76 at the top (one per waste stream), 45 deep, D10 outlets with 3 mm
    walls, a 4 mm inward splash lip; sits in the standard 118 x 78 window on the 4 x M4 pattern."""
    def hopper(x0, w, d, h, dout, off=0.0):
        return (cq.Workplane("XY").workplane(offset=off).center(x0, 0).rect(w, d).workplane(offset=-h).circle(dout / 2).loft()).val()
    outer = [hopper(x, 56.0, 76.0, 45.0, 16.0) for x in (-29.0, 29.0)]
    inner = [hopper(x, 51.0, 71.0, 45.04, 10.0, 0.02) for x in (-29.0, 29.0)]
    flange = B(-62.0, 62.0, -43.0, 43.0, -3.0, 0.0)
    lip = B(-60.0, 60.0, -41.0, 41.0, 0.0, 25.0).cut(B(-56.0, 56.0, -37.0, 37.0, -1.0, 26.0))   # splash wall 25 mm above the deck
    spig = [cyl(x, 0.0, -65.0, -44.0, 8.0) for x in (-29.0, 29.0)]
    s = fuse_all([flange, lip] + outer + spig)
    s = cut_all(s, inner + [cyl(x, 0.0, -66.0, -40.0, 5.0) for x in (-29.0, 29.0)] +
                [cyl(x, y, -4.0, 1.0, 2.25) for x, y in P.INSERT_SCREWS])
    return s


def tip_bin_k2():
    tips_req = 2500
    tips_max_audit = 4462
    v_tip = {"p300": 2.0, "p200": 1.7, "p10": 0.6, "p1000": 4.5}       # cm3 per tip, loose bulk (ASSUMED: a bag of 1,000 p200 tips ~1.7 L)
    split = {"p300": 0.55, "p200": 0.25, "p10": 0.20}
    v_mean = sum(v_tip[t] * f for t, f in split.items())
    fill = 0.70                                                          # usable fraction of a bin filled by a falling pile (ASSUMED)
    need = lambda n: n * v_mean / fill / 1000.0
    bin_L = 20.0
    tw3 = 0.102 * 0.124 * 0.070 * 1000
    return {"packing_ASSUMED": f"loose bulk {v_tip} cm3 per tip; mean {v_mean:.2f} cm3 at the p300/p200/p10 split; pile fill {fill}",
            "rev_K_bin_TW-3_L": round(tw3, 2), "rev_K_bin_holds_tips": int(tw3 * 1000 * fill / v_mean),
            "required_L_for": {"2,500 tips": round(need(tips_req), 1), "1.5 x 2,500 = 3,750": round(need(1.5 * tips_req), 1),
                               "4,462 (audit maximum)": round(need(tips_max_audit), 1), "1.5 x 4,462": round(need(1.5 * tips_max_audit), 1)},
            "provided": f"TB-2: {bin_L:g} L bought lidded PP crate class (about 400 x 300 x 170 mm) in an under-base drawer; holds about "
                        f"{int(bin_L * 1000 * fill / v_mean)} tips: {bin_L * 1000 * fill / v_mean / tips_req:.1f} x 2,500 and "
                        f"{bin_L * 1000 * fill / v_mean / tips_max_audit:.2f} x the audit maximum (CALCULATED from the ASSUMED packing)",
            "chute": "vertical PP duct 100 x 100 mm from the A1 funnel through the base, no horizontal step and no narrowing below 100 mm: "
                     "longer than the longest tip (88 mm p1000) in both directions, so a tip cannot lie across it and bridge; funnel "
                     "outlet widened to 100 x 100 (TW-1 change, revision L CAD)",
            "removal": "the drawer slides out to the front on two bought drawer slides, no tools; a bag liner; the chute bottom ends 30 mm "
                       "above the crate rim so the crate lifts out",
            "bin_full": "IR through-beam pair across the crate 30 mm below the rim (same part as the tip-sensor pair) + a drawer-closed "
                        "microswitch: inputs to the controller (no drawer or full: no tip drop, hold)",
            "liquid": "no liquid goes to the tip bin (LW-1 takes it): tips drip only their residue"}


def ethanol_ventilation():
    """Y7: calculation only (CALCULATED from ASSUMED inputs): needs review by a competent person before any build."""
    R, M, T, Pa = 8.314, 0.046, 303.0, 101325.0
    rho_v = Pa * M / (R * T)                       # kg/m3 of pure ethanol vapour at 30 C
    p_v80 = 7.5e3                                  # Pa, ethanol partial pressure over 80 % v/v ethanol at 30 C (ASSUMED ~ 0.72 x 10.4 kPa)
    lfl = 0.033                                    # lower flammable limit, volume fraction
    ait = 363.0                                    # C, auto-ignition temperature (lowest commonly quoted value)
    rho_l = 789.0
    vols = {"rev_K_frame (modelled)": 0.45, "K2_frame": 0.68}        # m3 free air (gross inner volume less 15 % contents, ESTIMATE)
    areas = {"trough 80 % ethanol, open 1-well reservoir 120 x 80": 0.0096, "second open trough (wash)": 0.0096,
             "one 96-well plate of ethanol wash open (96 x 28 mm2)": 0.0027}
    A_open = sum(areas.values())
    out = {}
    for k_m in (0.003, 0.006):                     # m/s mass-transfer coefficient, still air .. light draught (ASSUMED range)
        flux = k_m * p_v80 * M / (R * T)           # kg/m2 s into ethanol-free air
        G = flux * A_open                          # kg/s
        out[f"k={k_m * 1000:g} mm/s"] = {"evaporation_g_per_h": round(G * 3600 * 1000, 1)}
        for name, V in vols.items():
            m_lfl = lfl * V * rho_v
            q_ideal = G / rho_v / (0.25 * lfl)     # m3/s for 25 % LFL with perfect mixing
            out[f"k={k_m * 1000:g} mm/s"][name] = {
                "ethanol_at_LFL_g": round(m_lfl * 1000, 1), "as_liquid_mL": round(m_lfl / rho_l * 1e6, 0),
                "hours_to_LFL_no_exchange": round(m_lfl / G / 3600, 2),
                "air_needed_m3_per_h_ideal_mixing": round(q_ideal * 3600, 2), "air_needed_m3_per_h_K5": round(5 * q_ideal * 3600, 1)}
    sat_frac = p_v80 / Pa
    return {"status": "CALCULATION ONLY - must be reviewed by a competent person (DSEAR/ATEX-type assessment) before anything is built",
            "assumptions": {"temperature_C": 30, "ethanol_80pct_vapour_pressure_kPa": p_v80 / 1000, "LFL_vol_pct": lfl * 100,
                            "auto_ignition_C": ait, "open_areas_m2": areas, "free_volume_m3": vols,
                            "mixing_factor_K": "5 (ventilation of a box with local sources, ASSUMED)",
                            "longest_run_h": 23, "liquid_waste": "LW-1 drains to closed bottles OUTSIDE the enclosure (X1): 0 open waste"},
            "equilibrium": f"vapour over 80 % ethanol is about {sat_frac * 100:.1f} % by volume at 30 C, above the {lfl * 100:.1f} % LFL: a closed "
                           "box with open ethanol WILL reach the LFL if left long enough",
            "results": out,
            "response": {
                "supply": "ethanol in a lidded narrow-mouth reservoir (ER-1: bought 1-well reservoir with a lid the gripper lifts, or a "
                          "slotted cover with 8 x D5 holes at 9 mm pitch: open area about 0.16 x the trough), so the open area falls about "
                          "6x; plates of ethanol wash are open only during the wash",
                "waste": "LW-1 to two closed 2 L HDPE bottles in the under-base drawer outside the working volume (separate bunded "
                         "compartment, vent through an activated-carbon cartridge, not into the enclosure)",
                "extraction": "bought 24 V EC (brushless) inline duct fan 30 m3/h class on a 100 mm duct spigot in the back panel top, "
                              "make-up air through a filtered inlet low at the front: about 44 air changes per hour of the K2 volume and "
                              "at least 5 x the K5 requirement above for the uncovered case",
                "proving": "differential-pressure airflow switch (bought, 20-300 Pa adjustable) across the fan, read by the controller: "
                           "no proven flow -> no ethanol step starts and the run holds (planner rule)",
                "cold_blocks": "the airflow over the cold modules is from front-low to back-high: check the cold-block load and "
                               "condensation at commissioning"},
            "ignition_sources_for_the_electrical_owner": [
                {"source": "TC-1 heated lid", "surface_C": 105, "vs_AIT": "well below 363 C"},
                {"source": "TC-1 block", "surface_C": 99, "vs_AIT": "below"},
                {"source": "shaker heater (class 4-70 C)", "surface_C": 75, "vs_AIT": "below"},
                {"source": "TEC hot sides / sinks (cold modules, warm position)", "surface_C": 45, "vs_AIT": "below"},
                {"source": "stepper motors (deck and head)", "surface_C": 70, "vs_AIT": "below (brushless: no commutator spark)"},
                {"source": "fans", "surface_C": 40, "vs_AIT": "below; BLDC only, no brushed fans inside"},
                {"source": "relay and contactor contacts, safety relays", "surface_C": None,
                 "vs_AIT": "SPARK source (ethanol MIE about 0.65 mJ): keep in the separate control bay with its own air, not in the deck "
                           "volume"},
                {"source": "microswitches, door interlock, E-stop contacts in the deck volume", "surface_C": None,
                 "vs_AIT": "small sparks: acceptable only with the enclosure held below 25 % LFL; prefer sealed or optical types"},
                {"source": "connectors unplugged live, USB hubs", "surface_C": None, "vs_AIT": "spark on disconnection: no live unplugging"},
                {"source": "LED lighting, cameras, load-cell amplifiers", "surface_C": 50, "vs_AIT": "below"},
                {"source": "PSUs", "surface_C": 80, "vs_AIT": "below; in the control bay (separate air)"},
                {"source": "UV-C lamp ballasts (option, X2)", "surface_C": 60, "vs_AIT": "below; lamp off during ethanol steps (interlock)"}]}


def plate_working_volume():
    """Y16: largest fill a full-skirt 0.2 mL PCR plate tolerates when moved by the gripper and when shaken (CALCULATED from the
    nominal well model in deck_modules; MEASURE the plate chosen)."""
    pcr, wall = DK.PCR, 0.25
    r0, r1, hc, rc = pcr["od_bot"] / 2 - wall, pcr["od_cone_top"] / 2 - wall, pcr["cone_h"], pcr["od_cyl"] / 2 - wall

    def vol(h):
        if h <= hc:
            r = r0 + (r1 - r0) * h / hc
            return math.pi * h / 3 * (r0 * r0 + r0 * r + r * r)
        return vol(hc) + math.pi * rc * rc * (h - hc)
    depth = pcr["H"] - pcr["well_bottom"]
    a = 2.0                                                            # m/s2 gripper move (ASSUMED rule value)
    tilt_rise = (a / 9.81) * (2 * rc) / 2
    move_free = 2.0 + tilt_rise                                        # freeboard: 2 mm + slosh tilt rise
    shake_free = 6.0                                                   # freeboard for orbital shaking 2 mm orbit, ~1,000 rpm (ASSUMED rule)
    return {"well_depth_inner_mm": round(depth, 1), "brim_volume_uL": round(vol(depth)),
            "max_fill_moved_by_gripper_uL": int(vol(depth - move_free)),
            "max_fill_shaken_uL": int(vol(depth - shake_free)),
            "basis": f"gripper at <= {a} m/s2: surface tilt rise {tilt_rise:.2f} mm + 2 mm freeboard; shaking: {shake_free} mm freeboard "
                     "(ASSUMED from common shaker guidance: about half the well); 200 uL in a 0.2 mL well is at the brim and must not be "
                     "moved or shaken: use a deep-well or a 0.5 mL plate for those steps",
            "for_the_planner": "enforce fill <= max_fill_moved_by_gripper_uL for any gripper move and <= max_fill_shaken_uL on the shaker"}


def cold_plate_position():
    """Y4: CPB-1, a full 96-well plate on a rev K thermal module (same TEC pair and 0.21 K/W sink): CALCULATED load."""
    h, amb, t = 10.0, 30.0, 4.0
    a_top = 0.128 * 0.086
    a_side = 2 * (0.128 + 0.086) * 0.02
    q = h * (a_top + a_side) * (amb - t) + 2.0                       # + 2 W through the insert and locators (ASSUMED)
    return {"part": "CPB-1: Al 6061 block 127 x 85 x 20, 96 conical wells matched to the full-skirt plate (well-to-block contact), "
                    "SBS corner locators with 2 mm chamfered lead-ins, replaces TM-3 on the rev K module (same 4 clamp screws)",
            "heat_load_W_at_30C": round(q, 1), "pumped_at_4C_W": "about 24 (cold_block_rev_j, 3.5 A / 18 V, 0.21 K/W sink)",
            "margin": round(24.0 / q, 1),
            "gripper": "places the plate from above with the plate lid ON (LD-1 against condensation and evaporation); the skirt lands on "
                       "the locator chamfers; seat check by the weighing-free method: VC-1 reads the plate top height at two corners",
            "where": "B3 in K2 (cold plate position); C3 keeps the strip carrier; either module swaps to the other block in 4 screws",
            "sample_temperature": "an NTC in a water-filled well of a dummy strip at commissioning; in a run, the block NTC only"}


def item10():
    return {
        "X1_liquid_waste": {"part": "LW-1 twin funnel (exported, PA12 MJF or MJF PP): two streams (hazardous/solvent, biological) at "
                                    "A4 in K2, 396 mm from the A1 tip gate and VC-2, behind no tall item",
                            "volume": "500 mL per 96-sample day ASSUMED (planner to report): two 2 L closed HDPE bottles = 4 x the "
                                      "assumption per stream (Y16: two streams)",
                            "drain": "D10 spigots -> 8 mm ID FEP tubes -> bottle caps with two ports (drain in, vent to a carbon "
                                     "cartridge), falling all the way (no traps); bottles in a bunded compartment of the under-base "
                                     "drawer, separated from the electronics by a solid wall; no route into the control bay",
                            "level": "a non-contact capacitive level switch (bought, 24 V PNP) on each bottle at 80 %: input to the "
                                     "controller; bottle-present by the drawer switch",
                            "splash": "pipettes dispense with the tip 10 mm below the 25 mm splash wall onto the sloped hopper face "
                                      "(wall dispense, no free fall onto liquid); the nearest labware (A3, B4) is 66 mm beyond the wall",
                            "reach": "centre-only dispense: every pipette of the present head and of revision L"},
        "X2_contamination": {"zones": "post-amplification work on the right half: TC-1 (C1) -> post-PCR clean-up on magnet 2 at B4 -> "
                                      "waste A4; pre-PCR plates stay on the left/centre (B1-B3); post-PCR tips are never parked "
                                      "(TPK-1 is pre-PCR only); the gripper carries post-PCR plates lidded and not over open "
                                      "pre-PCR plates (planner rule)",
                             "UV": "option: 2 x 8 W T5 254 nm ozone-free germicidal lamps under the roof, on a switched output through "
                                   "the door interlock and a timer; PC door and panels block UV-C (verify the grade); lamps off "
                                   "during ethanol steps",
                             "attacked_by_UV_C": "PA12 (surface yellowing, slow embrittlement), PP labware, PU belts and cable sheaths, "
                                                 "closed-cell PE foam, sorbothane feet, printed paper tags, adhesives: belts and cables "
                                                 "under the beam shadow; tags laser-etched on anodised Al or UV-stable polyester; "
                                                 "foam covered",
                             "attacked_by_bleach": "bare aluminium (anodised tolerates a short contact then a water rinse), N52 "
                                                   "magnets' NiCuNi coating (MB-1 pockets sealed with an epoxy cap), load cells "
                                                   "(drip covers, wipe only), PA12 mildly; NEVER bleach where hazardous liquid may be "
                                                   "present (toxic gas): the hazardous liquid waste stream is separate (LW-1) and never "
                                                   "bleached",
                             "surfaces": "deck and inserts anodised Al, smooth PA12, no fabric; absorbent strips are disposable",
                             "air": "a HEPA fan-filter unit is NOT needed for the first build; reserve a 160 x 160 roof opening and a 24 V "
                                    "fan connector (it must be balanced against the Y7 extraction)"},
        "X4_X13_Y12_head": "DROPPED from revision K (ADR-010: exchangeable pipettes, revision L).  Present head kept as a PLACEHOLDER; reach "
                           "figures in rev_k.pipettes are for the present tools; the 1,000 uL and further pipettes come with revision L",
        "X6_caps": {"choice": "screw-cap tubes (1.5/2/5/15/50 mL) with caps removed at loading into a numbered cap tray (CT-1, a PA12 "
                              "plate with 48 cap pockets beside the rack), or cap-less tubes; no snap caps on the deck",
                    "clearance": "with caps off the tube rim is the top: CTB-1 rims at about seat + 82 mm, CTB-2 50 mL rims at about seat "
                                 "+ 160 mm, under the head's transit height (head lowest part with tips at transit: rev H/K assembly, "
                                 "about z 160 for tool plates) only if the 50 mL tubes stand in an 80 mm block: CHECK in revision L "
                                 "with the exchange head",
                    "long_open_tubes": "a gripper lid TRL-1 (SBS lid with tube-mouth bosses) over each tube block between uses; "
                                       "counted as loading and lid moves by the planner",
                    "loading": "cap removal is a counted manual loading action (planner)"},
        "X7_low_elution_magnet": {"MB-1_at_10uL": "liquid 1.98 mm high (CALCULATED); the side pellet sits in the liquid (magnet top edge "
                                                  "5 mm), but the tip-to-pellet gap at a 9 uL draw is about 0.97 mm before the XY "
                                                  "positioning stack (deck_modules mb1 check)",
                                  "decision": "the gap is too small to rely on: magnet 2 at B4 is a bought low-elution magnet plate "
                                              "(spring-cushioned, PCR plate, SBS) on the standard place; MB-1 stays for washes",
                                  "bench_rig": "added: elute 8 / 10 / 16 uL from MB-1 and from the low-elution magnet, measure bead "
                                               "carry-over and DNA yield (bench rig list)"},
        "X8_keys_and_labels": {"keyed_nests": "each corner-locator set gets one KEY post at the A1 corner that fills the plate's "
                                              "chamfered corner: a plate turned 180 degrees sits 2 mm high on the post and VC-1 "
                                              "detects it (plate top height); chamfer location depends on the plate brand: MEASURE",
                               "2D_label": "a 6 x 6 mm DataMatrix (10 x 10 modules, 0.5 mm) on the top frame of the plate at the A1 short "
                                           "edge, and on the top margin of each tube rack; VC-1 needs >= 3 px per module (6 px/mm at "
                                           "the plate top) - camera owner to confirm"},
        "X10_Y1_Y14_tips": {"bin": tip_bin_k2(),
                            "tip_type_check": "each tip frame carries its own tag (type in the tag ID) read by VC-1, and VC-2 measures "
                                              "the tip length below the nozzle (p10 32, p200 51, p300 59 mm nominal; wide-bore p200 "
                                              "registered as its own type and length)",
                            "wide_bore": "wide-bore p200 tips: same collar as p200, so the p200 frame (G2, racks swapped in by the "
                                         "gripper from the outer shelf column) and the tip gate take them; "
                                         "TPK-1 is sized for p300 collars: wide-bore tips are not parked (single use); the gate "
                                         "beam sees the wider end; supply as racks of 96 in G4 (refill wafers rarely sold: "
                                         "purchasing to confirm)"},
        "Y2_warm_position": "A3 in K2: a rev K thermal module run at 37 C (the TEC pair heats as well as cools; class 4-40 C) with a "
                            "lidded 1-well trough or a tube block for hazardous liquid; the shaker is not used (it is busy with samples)",
        "Y4_cold_plate": cold_plate_position(),
        "Y6_rescue": {"covers": "3 spare SBS lids stacked on lid park 2 (D3, under the 51 mm cap): the gripper or a person covers sample plates on "
                                "a stop", "access": "every sample place is in rows F-C, within 600 mm of the door; the head is moved by "
                                                     "hand (belts back-drive with drivers off); position is then re-taken from the "
                                                     "deck markers by VC-1 instead of a re-home (firmware/software owner)",
                      "ups": "reserved: 300 x 120 x 200 mm in the control bay and a 24 V input connector for a DC UPS (controller + "
                             "cold modules); electrical owner to size"},
        "Y7_ethanol": ethanol_ventilation(),
        "Y8_reservoirs": "ethanol and hazardous liquid in lidded or slotted-cover reservoirs (ER-1); hazardous liquid at A3 (warm), water and buffers "
                         "in reservoirs on B1/C3 or the ATR racks, never on a place beside A3 (A2 and A4 hold racks and waste, B3 "
                         "the cold plate)",
        "Y16_plate_volume": plate_working_volume()}


def headroom():
    rows = []

    def add(kind, cap, req, prov, unit, note=""):
        if req is None or prov is None:
            ok, rule = None, "1.5 x (electrical owner)"
        elif kind == "consumable":
            ok = prov >= req + 2
            rule = "largest need + 2 spare"
        else:
            ok = prov >= 1.5 * req
            rule = "1.5 x"
        rows.append({"kind": kind, "capacity": cap, "required_largest_known": req, "provided": prov, "unit": unit,
                     "ratio": round(prov / req, 2) if req and prov else None, "spare": round(prov - req, 2) if req is not None and prov is not None else None, "rule": rule, "meets_rule": ok,
                     "note": note})
    lab = [n for n, v in K2_PLACES.items() if "(labware)" in v[0]]
    lab_places = len(lab) + 3 * sum(1 for n in lab if "stack x4" in K2_PLACES[n][0])     # a stack of 4 counts 4
    add("consumable", "deck labware places", 13, lab_places, "places", "required = rev H provision (13, ASSUMED); stacks of 4 count 4")
    add("consumable", "cold 1.5/2 mL tubes", 24, 48, "tubes")
    add("consumable", "cold 5 mL tubes", 4, 6, "tubes")
    add("consumable", "cold 15 mL tubes", 2, 4, "tubes")
    add("consumable", "cold 50 mL tubes", 1, 3, "tubes")
    add("consumable", "ambient 1.5/2 mL tubes", 12, 24, "tubes")
    add("consumable", "ambient 5 mL tubes", 6, 10, "tubes")
    add("consumable", "ambient 15 mL tubes", 4, 6, "tubes", "ATR-1 at F1")
    add("consumable", "ambient 50 mL tubes", 4, 6, "tubes", "ATR-1 at F1")
    add("consumable", "tip frames (tip types)", 4, 4, "frames", "LOWER: p10, p200 (+ wide-bore racks swapped in), p300, p1000 in row G; "
        "a further type goes on a labware place (B row) when a protocol needs it; a fifth and sixth frame would cost two labware places")
    add("consumable", "tips held on deck and shelf", 2500, 4416, "tips", "4 frames x 96 + 6 towers on the inner shelf column")
    add("consumable", "parked tips", 104, 104, "tips", "LOWER: one TPK-1; an overflow costs fresh tips, not a stop")
    add("consumable", "tip waste", 2500, int(20.0 * 1000 * 0.7 / 1.585), "tips", "20 L under-base bin; ASSUMED packing; audit maximum "
        "4,462 also covered")
    add("consumable", "liquid waste", 500, 4000, "mL", "two 2 L bottles (two streams); 500 mL ASSUMED")
    add("consumable", "cold plate positions (full plate)", 1, 2, "plates", "LOWER than +2: B3 CPB-1 and C3 convertible in 4 screws")
    add("consumable", "warm positions", 1, 1, "places", "LOWER: A3; any labware place takes a thermal module")
    add("consumable", "magnet positions", 1, 2, "places", "B2 MB-1, B4 low-elution")
    add("consumable", "pipette dock bays", 6, 8, "bays", "five pipettes + the syringe dispense tool, 2 free (ADR-010)")
    add("consumable", "edge hand-off positions", 2, 2, "places", "ADR-010 minimum")
    add("fixed", "cooling: cold plate block", 7.1, 24.0, "W", "pumped at 4 C vs load at 30 C ambient")
    add("fixed", "cooling: tube blocks", 4.6, 24.0, "W", "per block at 30 C")
    add("fixed", "Z clearance over the tallest item (revision L head)", 33.0, 50.0, "mm", "requirement set on the revision L head: lowest "
        "transit point >= z 177; the present head has 33 mm")
    add("fixed", "X carriage travel", 640.0, 640.0, "mm", "travel is sized to the reach needed plus the present end margins; 1.5 x "
        "applied to the reach-to-limit margins is the revision L check (ASSUMED interpretation: the rule cannot mean 1.5 x the length)")
    add("fixed", "frame lateral load (M1 at 150 N = 1.25 x 120 N)", 150.0, 150.0, "N", "FAILS already: 0.566 mm vs 0.5 mm limit with "
        "the present head; K2 beam about 0.61 mm: the revision L head and coupling must recover it; 1.5 x 120 = 180 N not met")
    add("fixed", "extraction air (ideal mixing, 3 mm/s, all open)", 2.12, 30.0, "m3/h", "5.9 x the K5 figure (no lids)")
    add("fixed", "24 V and TEC supplies", None, None, "W", "electrical owner: 1.5 x the fitted load (control_spares)")
    return {"rule": "ADR-009 as adjusted (ADR-010 market survey): fixed capacities (travel, Z clearance, frame load, power, cooling) "
                    ">= 1.5 x; consumable places (tips, tubes, plates) >= largest need + 2 spare of each kind; lower figures named",
            "table": rows, "labware_places_list": lab,
            "cost_of_margin": {
                "envelope_mm": {"was (rev K modelled)": K2_FRAME["envelope_was_mm"], "K2": K2_FRAME["envelope_mm_ESTIMATE"]},
                "mass_kg": "about 88 -> about 113 (frame +13 for Y +260, +5 for width +198, drawer/feet +3, dock +4, fan/waste +2, head -2.6 "
                           "with the ASSUMED 11 kg head: ESTIMATE)",
                "Y_belt_mode_Hz": "15.2 -> about 13.6 (loop 1364 -> 1884 mm; Y moving mass 19.25 -> about 17.3 kg with the ASSUMED 11 kg "
                                  "head); residual settle about 0.66 s < 1.04 s dwell",
                "X_belt_mode_Hz": "worst 10.9 -> about 11.0 at EA 3e4 (span +21 %, head 13.55 -> 11 kg: settle about 1.03 s, AT the 1.04 s "
                                  "dwell: no margin) and 15.4 -> about 15.5 at EA 6e4 (settle 0.49 s): specify the X belt EA >= 6e4 N "
                                  "(15 mm glass-fibre GT2 or steel cord) - measure on the rig",
                "with_present_head_placeholder": "13.55 kg head: X worst about 9.9 Hz at EA 3e4 (settle about 1.25 s: fails the dwell); "
                                                 "the K2 frame is sized for the revision L head",
                "M1": "X beam torsion x1.19 and beam bending x1.70 (beam 980 -> 1170 mm); side beams unchanged with intermediate "
                      "uprights (rev H method): CALCULATED with the present head in the assembly report rev_k.headroom.M1_K2_estimate; "
                      "the study's coupling adds about 0.013 mm",
                "frame_grows_once_to": K2_FRAME}}


def control_spares():
    return {"for_the_electrical_owner (ASSUMED by the mechanical design)": {
        "motor_axes": "the present 4-tool head + X, 2 x Y; revision L head unknown: keep >= 2 spare stepper driver sockets",
        "thermal_channels": "cold modules x2 (B3 CPB-1, C3 strips), warm position A3, tube blocks x2 (F3, F4), shaker controller, TC-1; "
                            "spare: >= 2 TEC channels (second thermal C2, one more) and >= 4 NTC inputs",
        "weighing": "WN-1, magnet-weighing reserve, balance (serial); spare >= 1 HX711-class channel",
        "inputs": "tip bin full, drawer closed, 2 bottle levels, airflow proving switch, edge hand-off shutter x2, tip-sensor pair: "
                  ">= 4 spare digital inputs",
        "switched_outputs": "extraction fan, UV-C lamps (through the interlock), LED light, 2 spare 24 V outputs (bulk dispense valve, "
                            "HEPA fan)",
        "deck_connector_per_place": "one keyed connector per place (ADR-010 study): 24 V 3 A fused, USB 2.0, two fail-safe lines; "
                                    "24 places in K2 = 24 fused 24 V outputs (72 A if all drew 3 A: size the 24 V rail for the fitted "
                                    "modules + 30 %), a USB tree of 24 downstream ports, 48 fail-safe inputs; modules over 72 W on a "
                                    "switched mains strip",
        "dock": "8 bay-occupied switches; the head's Hall tool-code sensors and latch switches are on the carriages (revision L)",
        "USB_and_chain": "VC-1, VC-2, VC-3, balance, shaker: hub with >= 2 spare ports; in the X/Y cable chains one spare USB and one "
                         "spare 8-core cable for the revision L head; chain cross-section +30 %",
        "UPS": "24 V input for a DC UPS (controller + cold modules), space reserved"}}


def survey_adjustments():
    """ADR-010 market-survey adjustments applied at deck level (calculation and reservation)."""
    return {
        "module_zone": {"places": "F3+F4 (front right, a double place like C1+D1: one 250 x 98 window, the outer bolt patterns kept)",
                        "sized_for": "a bought heater-shaker about 152 x 90 x 82 mm OR a bought temperature block about 194 x 90 x 84 mm "
                                     "(survey figures, to be confirmed), each with plate landscape; both at once do not fit (346 > 264 mm) "
                                     "- the K2 heater-shaker stands at D4 (its own powered place) so the deck has both",
                        "under_deck": "through to the bench (rev K thermal modules reach z -115); tall tubes recessed 60 mm",
                        "height": "class 150 above the seat; nothing worked deep in front (row F is the front) and the beside rule "
                                  "is met (single channels only at F3/F4)", "connector": "fitted"},
        "connectors_fitted_at": {"F3+F4": "module zone", "A1": "tip gate, VC-2 (exists)", "A3": "warm thermal module",
                                 "B1": "weighing nest", "B3": "cold plate module", "C1+D1": "thermocycler (exists)",
                                 "C3": "cold strip module", "D4": "heater-shaker", "F2": "balance"},
        "cable_route_only": [n for n in K2_PLACES if n not in ("F3", "F4", "A1", "A3", "B1", "B3", "C1", "D1", "C3", "D4", "F2")],
        "storage_tower_vs_deck_stacks": {
            "deck_stacks (chosen for the first build)": "C4 and D2, 4 plates each (51 mm cap from the head's rear parts): no moving "
                                                        "part, no controller channel, two deck places",
            "side_storage_tower (reserved at H2)": "shelves beside the machine presenting one shelf at a time at the hand-off height "
                                                   "need their own lift axis (the gripper Z cannot reach a tall tower): about 10-20 "
                                                   "plates or tip racks, a stepper, lead screw, rails and a home switch (custom module, "
                                                   "ESTIMATE £150-300 parts) and a driver channel",
            "verdict": "the tower is NOT simpler than stacking; it frees C4/D2 (two labware places, 8 stacked plates) when capacity "
                       "runs short: reserve it at H2, build later"},
        "roof_reservation": "a blanked 320 x 320 mm opening in the roof panel with 4 x M5 rivnuts on a 300 mm square for a bought fan "
                            "filter unit (HEPA class) and a fused feed (24 V or mains) to the roof; UV-C on the door interlock (item "
                            "10b).  The FFU must be balanced against the Y7 extraction (the extraction wins: flow out > flow in)",
        "z_clearance": {"tallest_items_z": "TC-1 lid 127 (rev H/K); tubes and the p1000 rack recessed so nothing else stands above "
                                           "about z 110",
                        "present_head_transit_lowest_z": 160.0, "present_margin_mm": 33.0,
                        "rule_1.5x": "the revision L head shall keep its lowest transit point at or above z 177 (127 + 1.5 x 33); "
                                     "the frame keeps the beam at z 400"}}


def module_standard():
    """ADR-010 deck module standard (study result, adopted) and how the present (modelled rev K) places meet it."""
    present = {   # place: (use, height class above the seat, under-deck class, meets footprint/bolt pattern, note)
        "A1": ("tip waste funnel + gate", 21, "through", True, "chute to the bin"),
        "A2": ("tip parking TPK-1", 72, 5, True, "lower plate top 72: at the class limit"),
        "A3": ("p10 tip frame", 72, 5, True, ""),
        "B1": ("weighing nest WN-1", 72, 60, True, "insert floats 0.5 mm on the cell; cell bracket inside 60"),
        "B2": ("magnet MB-1", 21, 60, True, "MB-1 body 30 mm below the insert"),
        "B3": ("cold module (strip carrier)", 72, "through", True, "carrier 23.5 > 21; sink + fan to z -115 > 110"),
        "C1+D1": ("thermocycler TC-1 + lid park", 150, "through", True, "the double place (merged window, MERGED_WINDOWS)"),
        "C2": ("p200 tip frame", 72, 5, True, ""),
        "C3": ("cold module (strip carrier)", 72, "through", True, "as B3"),
        "D2": ("p300 tip frame", 72, 5, True, "72: at the class limit"),
        "D3": ("lid park 2", 72, 5, True, "pedestal + 2 lids"),
        "F1": ("plain, pipette only", 72, 5, True, ""),
        "F2": ("balance", 72, "through", True, "draught shield 60; cell bay to z -130"),
        "F3": ("shaker on SK-1", 150, 5, False, "the shaker (147 mm) is wider than the 132 pitch and offset 29 mm: meets NO footprint "
                                                 "class -> FIX: an EDGE place rule (a module wider than the pitch only at a deck-edge "
                                                 "place, overhanging outwards: F3 is the right-front corner today, D4 in K2)")}
    return {"standard (ADR-010 study, adopted)": {
                "pitch_mm": [132.0, 110.0], "insert_plate_mm": [130.0, 108.0, 5.0],
                "bolt_pattern": "4 x M4 at (+/-57, +/-47) + 2 x D4 dowels at (0, +/-47), the +Y one in a slot (P.INSERT_SCREWS, "
                                "P.INSERT_DOWELS): unchanged since rev A",
                "window_mm": [P.WINDOW_L, P.WINDOW_W], "double_place": "C1+D1 pattern (118 x 188 window, outer screws and dowels)",
                "height_classes_above_seat_mm": [21, 72, 150], "under_deck_classes_mm": [5, 60, 110, "through to the bench"],
                "connector": "one keyed connector per place on the module's rear face below the deck: 24 V 3 A fused, USB 2.0, two "
                             "fail-safe lines; cable route to it; modules over 72 W bring their own bought supply on a switched strip",
                "connector_cable_route_FIX": "no present place has one: every window gets a 30 x 14 mm notch in its rear (+Y) edge at "
                                             "x +15..+45 (13 mm clear of the +Y dowel and 12 mm of the M4 at +57) for the plug and "
                                             "cable; deck plate change in revision L (the rev K deck DXFs are unchanged)"},
            "present_places": {k: {"use": v[0], "height_class_mm": v[1], "under_deck_class": v[2], "footprint_and_bolts": v[3],
                                   "note": v[4]} for k, v in present.items()},
            "over_72_W": "cold modules (2 x TEC at 3.5 A / 18 V, about 63 W + fan) and the warm position run from the existing TEC "
                         "supply; the shaker (heater + cooler) and TC-1 bring their own supplies",
            "K2_places": list(K2_PLACES),
            "reserved_modules (ADR-009 item 4)": {"second thermal position": "any labware place by fitting a thermal module (B4/C3 "
                                                                             "nearest the TEC supply); A3 is the warm one",
                                                  "second magnet": "B4 (low-elution, fitted)",
                                                  "second shaker": "an edge place: F-row or column 4 with nothing worked beside it "
                                                                   "(C4 in place of the stack)",
                                                  "plate sealer / lid stack": "lid stack on D3 (5 lids) or a stack place (C4/D2); sealer "
                                                                              "at an edge hand-off (H1/H2)",
                                                  "bulk dispense nozzle": "a spare dock bay (ADR-010) and one spare 24 V output"}}


if __name__ == "__main__":
    r = main()
    print(json.dumps({"all_valid": r["all_valid"], "n_parts": len(r["parts"]), "gripper_limits": r["checks"]["gripper_depth_limits_s"],
                      "pcb": r["checks"]["passive_cold_block_PCB-1"], "stacks": {k: v.get("max_units") for k, v in r["checks"]["stacks"].items()
                                                                                    if isinstance(v, dict)}}, indent=1))
