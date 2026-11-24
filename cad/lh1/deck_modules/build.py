"""LH-1 deck plate and deck modules (CadQuery 2.5.2).

Run from the project root:  .venv/bin/python cad/lh1/deck_modules/build.py
Out:  cad/lh1/deck_modules/out/  STEP per custom part, DXF per laser-cut part (layers cut, drill,
      ream, tap, engrave, note), STL per printed part, one assembly STEP per module,
      deck_assembly.step (default layout) and report.json (parts, bought parts, checks).

Every interface number comes from cad/lh1/params.py (imported as P).  Numbers defined here are
this module's own design choices (locator walls, magnet layout, thermal stack ...), each named.
Maturity: FABRICATION CANDIDATE FOR A FIRST PROTOTYPE.  Nothing here is validated; labware
geometry (PCR plate wells, skirts, tip collars) is nominal and must be measured (gate G0).
"""
from __future__ import annotations

import importlib.util
import json
import math
import sys
from pathlib import Path

import cadquery as cq
import ezdxf

HERE = Path(__file__).resolve().parent
LH1 = HERE.parent
sys.path.insert(0, str(LH1))
import params as P  # noqa: E402
sys.path.insert(0, str(LH1 / "assembly"))      # integration rev D: in-memory params overrides
import params_override  # noqa: E402
params_override.apply(P)

OUT = HERE / "out"
V = cq.Vector

# =====================================================================================
# Design parameters of this module (mm).  Interface values are derived from params.py.
# =====================================================================================
# Nests whose windows are merged into one bay for another engineer's module.  The bay replaces
# both windows and the 32 mm web between them with ONE window WINDOW_L x (2*WINDOW_W + web).
MERGED_WINDOWS = [("C1", "D1")]
DEFAULT_LAYOUT = {   # nest -> (insert type, labware on it)
    "A1": ("tip_waste", None),
    "A2": ("plain", "tiprack"), "A3": ("plain", "tiprack"),
    "B1": ("plain", "pcr_plate_sample"), "B2": ("magnet_block", "pcr_plate_full_skirt"), "B3": ("plain", "pcr_plate_fresh"),
    "C1": ("thermocycler_bay", None), "D1": ("thermocycler_bay", None),
    "C2": ("plain", "reservoir"), "C3": ("thermal", "tube_block_24"),
    "D2": ("plain", "tiprack"), "D3": ("plain_handoff", None),
    # rev H: front row F (params FRONT_ROW): F1 pipette-only (plain insert, reservoir <= 44 mm; no tip rack: tip-check gate, K11), F2 plain (all four
    # tools), F3 second cold strip carrier (same thermal module as C3)
    **({"F1": ("plain", "reservoir"), "F2": ("plain", None), "F3": ("thermal", "tube_block_24")} if hasattr(P, "FRONT_ROW") else {}),
}
if getattr(P, "REV_K", False):   # rev K: the station hardware and stacks come from cad/lh1/deck_k (labware None here); F2 = balance insert
    for _n in ("A2", "A3", "C2", "D2", "F3"):
        DEFAULT_LAYOUT[_n] = ("plain", None)
    DEFAULT_LAYOUT["B3"] = ("thermal", "tube_block_24")          # second cold block (was F3; the shaker took F3)
    DEFAULT_LAYOUT["F2"] = ("balance", None)
DECK_Y0 = getattr(P, "DECK_Y0", -P.DECK_W / 2)                                   # rev H: deck front edge
N_THERMAL = sum(1 for v in DEFAULT_LAYOUT.values() if v[0] == "thermal")          # rev H: 2
N_INSERTS = sum(1 for v in DEFAULT_LAYOUT.values() if v[0] != "thermocycler_bay")  # rev H: 13

SEAT = P.LABWARE_SEAT_Z
SBS_HX, SBS_HY = P.SBS_L / 2, P.SBS_W / 2
WIN_HX, WIN_HY = P.WINDOW_L / 2, P.WINDOW_W / 2
INS_HX, INS_HY = P.INSERT_L / 2, P.INSERT_W / 2
Z_UNDER_MIN = -(P.DECK_T + P.UNDER_DECK_CLEAR)          # lowest allowed z of anything below the deck
WEB_X = P.NEST_PITCH_X - P.WINDOW_L                     # 14 mm webs (run along Y)
WEB_Y = P.NEST_PITCH_Y - P.WINDOW_W                     # 32 mm webs (run along X)

# ---- fasteners / DFM
M3_CLR, M4_CLR, M5_CLR = 3.4, 4.5, 5.5
M3_TAP, M4_TAP = 2.5, 3.3
M3_CSK, M4_CSK, M5_CSK = 6.6, 9.0, 10.4                  # 90 deg countersink top diameters
INSERT_SLOT_LEN = 6.0                                     # slot length (along Y) for the +Y dowel

# ---- corner locators (MJF PA12), plain / magnet / hand-off inserts
LOC = dict(play=0.2,          # nominal clearance per side between SBS footprint and locator faces
           h=6.0,             # height above the seat (<= 8 so the gripper corridor stays free)
           wall_x=1.6,        # short-side (X) wall: pitch 132 leaves only 4.24 mm between labware
           y_arm_x0=38.0, y_arm_t=9.0, x_arm_y0=26.0,
           chamfer_y=1.5, chamfer_x=0.8,
           m3=[(42.0, 47.5), (49.0, 47.5)])

# ---- frame interface (for the frame engineer; reference beams only, not designed here)
BEAM_W, BEAM_H = 20.0, 40.0          # 2040 on edge (20 wide, 40 tall) under the deck
I_2040_STRONG = 6.5e4                # mm^4, assumed for 2040 on edge (confirm with the extrusion datasheet)
E_AL = 69000.0                       # N/mm^2
BEAMS_X = {                          # beam centre y -> (x of left support, x of right support)
    **({-350.0: (-230.0, 230.0)} if hasattr(P, "FRONT_ROW") else {}),   # rev H: front beam under row F
    -225.0: (-230.0, 230.0), -110.0: (-230.0, 230.0), 0.0: (-230.0, 230.0),
    110.0: (P.DECK_BEAM_Y110_END_X, 230.0),   # integration: frame rev B stops this beam at x = -47 (was -66, inside TC-1)
    225.0: (-230.0, 230.0)}
BEAMS_Y_X = (-230.0, 230.0)          # side beams along Y under the X margins
# integration: the frame's post DP is 12 x 20 at x -47..-35, y 100..120 (was assumed 12 x 30 at x -72..-60)
TC_POST = (P.DECK_BEAM_Y110_END_X, P.DECK_BEAM_Y110_END_X + 12.0, 100.0, 120.0)

# ---- magnet module
MAG = dict(block=(2.0, 10.0, 20.0),   # N52 block magnet W (X, magnetised through) x L (Y) x H (Z)
           per_row=7,                 # blocks end to end along Y -> 70 mm, covers rows A-H
           seat_depth=6.0,            # depth of the magnet pockets in the carrier
           pellet_top=9.0,            # TUNE ON BENCH: engaged magnet top above the well bottom
           travel=25.0,               # lift travel from home (disengaged) to full-skirt engaged
           carrier=(97.0, 73.0, 30.0),
           opening=(103.0, 76.0),     # opening in the magnet insert plate
           housing_x=(53.0, 58.0), housing_hy=37.0, floor_t=4.0,
           rod_d=6.0, rod_pts=[(0.0, -25.0), (0.0, 25.0)], bushing=(12.0, 19.0),
           screw_d=8.0, nut_body=(10.2, 15.0), nut_flange=(22.0, 3.5),
           motor=(42.3, 34.0), flag=(33.5, 35.5, -5.0, 5.0, 12.0),
           housing_screws=[(sx * 56.0, y) for sx in (-1, 1) for y in (-25.0, 0.0, 25.0)])
# integration rev F (lab head): MB-1 PASSIVE magnetic block is the default in B2.  The same 6 rows x 7 N52 2x10x20 blocks as
# the MM lift, fixed at one height in an MJF PA12 body hung from the MM-1 insert plate; the gripper places the plate on it
# and lifts it off.  The motorised lift MM stays as a documented alternative (not in the base parts list).
MB = dict(pellet_top=5.0,           # magnet top above the inner well bottom (full skirt); 5 mm so a 9 uL elution pellets in the liquid
          ears_x=(48.5, 58.0), body_h=30.0, ear_t=3.0,
          br_t=1.43,                # N52 remanence (typ. 1.42-1.47 T, grade data; measure with a gaussmeter at goods-in)
          polarity="alternating row to row (row k north face toward +X for even k): far field falls as a multipole")
# nominal PCR plate (0.2 mL, 96) - MEASURE: well profile and skirt heights vary by brand
PCR = dict(H=16.1, skirt_t=1.0, deck_top=14.6, deck_t=1.0, well_bottom=1.0, od_bot=2.8,
           od_cone_top=5.4, cone_h=11.0, od_cyl=5.6,
           semi_skirt_bottom=5.0)   # semi-skirt: skirt edge 5 mm above the well bottoms -> wells 4 mm below the seat

# ---- thermal module
TH = dict(opening=(115.0, 77.0), lip=(118.0, 80.0, 6.0), lower=(108.0, 70.0), block_top=14.0,
          block_bottom=-16.0, frame_flange=(124.0, 84.0, 2.5), skirt_out=(114.0, 76.0), skirt_in=(110.0, 72.0),
          skirt_bottom=-17.0, pad_h=0.5, tec=(40.0, 40.0, 3.8), tec_x=23.0, sink=(110.0, 70.0, 6.0, 34.0),
          fin_t=1.5, fin_pitch=6.0, fan=(60.0, 25.0), tube_pitch=18.0, tube_pocket=(11.1, 21.0, 4.0, 20.0),
          clamp=[(0.0, -18.0), (0.0, 18.0), (-50.0, 0.0), (50.0, 0.0)],
          frame_screws=[(sx * 60.0, sy * 25.0) for sx in (-1, 1) for sy in (-1, 1)], drain=(-60.5, 0.0))
# rev K (J finding): BOUGHT bonded/skived-fin sink 110 x 76 x 54 (6 mm base, 48 mm fins 1.0 mm on 3.0 mm pitch) + a 60 x 60 x 38
# high-static-pressure fan (>= 17 CFM free air) blowing up into the fins; theta 0.22 K/W CALCULATED (h 40 W/m2K, ASSUMED).
# 76 mm wide so the module still drops in from above through the 118 x 78 window (1 mm each side).
if getattr(P, "REV_K", False):
    TH.update(sink=(110.0, 76.0, 6.0, 48.0), fin_t=1.0, fin_pitch=3.0, fan=(60.0, 38.0))   # fan bottom z -115 (limit -116); 76 wide: drops through the 78 mm window
TH_K_SINK = dict(h_W_m2K=40.0, flow_m3_s=0.008, note="CALCULATED from ASSUMED h and fan flow: measure on the bench rig")
# ---- tip waste insert
TW = dict(opening=(104.0, 74.0), funnel_out=(116.0, 76.0), funnel_bottom=-44.0,
          outlet_in=(92.0, 60.0), outlet_out=(100.0, 64.0), rail_z=-51.0, bin_body_hx=50.0, bin_flange_hx=55.5,
          # integration rev D: rail_z -50 -> -51 (bin 2.5 mm under the y=-225 beam); rev E: outlet +4 in Y (300 uL tips)
          bin_y=(-80.0, 34.0), bin_bottom=-112.0, bin_wall=2.0,
          strip=dict(x0=20.0, x1=63.0, hy=40.0, t=3.0, slot_w=6.2, slot_len=22.0, screws=[(58.0, -33.0), (58.0, 33.0)]),
          screws=[(sx * 56.0, sy * 20.0) for sx in (-1, 1) for sy in (-1, 1)])
# integration rev D: raised tip-strip comb (replaces the TW-4 plate at z 5..8, which no nozzle could reach: lowest
# nozzle end z 30).  Nest coordinates.  8-channel tools enter the 8 slots from -X, the single channel the narrow slot
# from +X, below the comb with the tip collars under it, then lift: the comb strips the tips into the funnel.
# integration rev F (review M3/M19): machined comb.  Plate COMB_T thick on two end legs, stiffened by a rib under the plate
# behind the slot ends (mid-span support; a post cannot stand there: the funnel opening is under mid-span).  Slot widths
# come from the (nominal, to be measured) nozzle OD at the collar + 2 x COMB_SLOT_CLEARANCE; the single-channel slot is
# moved 4 mm further +X so its tip clears the rib.  One comb serves both eight-channel tools (widest nozzle sets the slot).
_slot8 = round(max(P.TIP_NOZZLE_OD_AT_COLLAR[P.TOOL_TIP[t]] for t in ("pipette_8ch_50", "pipette_8ch_300"))
               + 2 * P.COMB_SLOT_CLEARANCE, 2)
_slot1 = round(P.TIP_NOZZLE_OD_AT_COLLAR[P.TOOL_TIP["pipette_small"]] + 2 * P.COMB_SLOT_CLEARANCE, 2)
TW["comb"] = dict(z_under=P.TIP_STRIP_COMB_UNDERSIDE_Z, t=P.COMB_T, x=(-10.0, 34.0), hy=46.5, leg_y=46.5, foot_y=53.5,
                  slot8_w=_slot8, slot8_x1=14.0, slot1_w=_slot1, slot1_x0=26.0 - _slot1 / 2,
                  rib=P.COMB_RIB, leadin=P.COMB_LEADIN,
                  feet_m3=[(x, sy * 50.0) for x in (-2.0, 26.0) for sy in (-1, 1)],
                  # nozzle-end z and nozzle x (nest coords) per tool for the strip move: start -> engaged, then lift
                  moves={"pipette_8ch_50": dict(z=44.75, x0=-22.0, x1=9.0, y=0.0),
                         "pipette_8ch_300": dict(z=44.5, x0=-22.0, x1=9.0, y=0.0),   # rev E: p300 tips (59 mm)
                         "pipette_small": dict(z=42.0, x0=46.0, x1=26.0, y=0.0)},   # rev F: +4 mm in X (rib)
                  requires="slots from MEASURED nozzle OD at the collar (+0.3/side); collar must overlap each finger by >= 1 mm")


def subplate_repeatability(feats):
    """integration rev F (M7): refit repeatability of the TC-1 plate position (worst case and RSS), from pin/hole fits.
    D4 h6 pin (3.992..4.000) in a D4 H7 hole/slot (4.000..4.012): radial clearance 0..0.020 mm per joint."""
    c = 0.020
    (rx, ry, _), (sx, sy, _) = SUB_DOWELS
    span1 = math.hypot(sx - rx, sy - ry)
    tc = P.nest_centre("C1")
    arm1 = math.hypot(tc[0] - rx, tc[1] - ry)
    # DK-2 on DK-1: translation c (round pin) + rotation c/span about it, seen at the TC plate centre
    e1 = [c, c / span1 * arm1]
    # TC-1 top plate on DK-2: C1 dowel round (-132,8), D1 dowel slot (-132,212): rotation about the round one, centre at 47 mm
    span2 = 204.0
    e2 = [c, c / span2 * 47.0]
    e = e1 + e2
    holes_on_joint = [f for f in feats if f.get("frame") and (
        (abs(f["x"] - SUBPLATE[1]) < f.get("dk", f["d"]) / 2 + 1.0 and SUBPLATE[2] - 5 <= f["y"] <= SUBPLATE[3] + 5) or
        (abs(f["y"] - SUBPLATE[3]) < f.get("dk", f["d"]) / 2 + 1.0 and f["x"] <= SUBPLATE[1]) or
        (abs(f["y"] - SUBPLATE[2]) < f.get("dk", f["d"]) / 2 + 1.0 and f["x"] <= SUBPLATE[1]))]
    split_inserts = []
    for n in ("B1", "C2", "D2", "B2", "D1"):
        on = {("sub" if (f["x"] < SUBPLATE[1] and SUBPLATE[2] <= f["y"] <= SUBPLATE[3]) else "main")
              for f in feats if f.get("nest") == n and f["layer"] == "ream"}
        if len(on) > 1:
            split_inserts.append(n)
    return dict(fit="D4 h6 pins / H7 holes, radial clearance 0..0.020 mm", dk2_on_dk1_pins=[list(d[:2]) for d in SUB_DOWELS],
                tc1_plate_refit_worst_mm=round(sum(e), 3), tc1_plate_refit_rss_mm=round(math.sqrt(sum(v * v for v in e)), 3),
                frame_screws_on_joint_lines=[(f["x"], f["y"]) for f in holes_on_joint],
                inserts_doweled_to_both_plates=split_inserts,
                c2_d2_note="C2/D2 locate on DK-1 dowels; their two -X M4 screws clamp into DK-2 (clearance holes in the insert)",
                assumption="fits per ISO 286; not measured. Excludes deck flatness and thermal growth.")


def comb_strength():
    """integration rev F (M3): comb strength at STRIP_COMB_SAFETY x the largest strip force (8 tips), Al 6061-T6.
    Two cases: (a) whole comb as a beam between the end legs (UDL), section = plate ligament behind the slots + rib;
    (b) one finger between two slots as a cantilever from the slot end to the collar contact at the engaged nozzle x."""
    c = TW["comb"]
    E, sy = 69000.0, 276.0
    F = P.STRIP_COMB_SAFETY * max(P.STRIP_FORCE_N.values())
    t = c["t"]
    rx0, rx1, rd = c["rib"]
    b_pl = c["x"][1] - (c["slot8_x1"] + c["slot8_w"] / 2)        # plate ligament behind the 8ch slot ends
    b_rib = rx1 - rx0
    A1, z1 = b_pl * t, t / 2
    A2, z2 = b_rib * rd, -rd / 2
    zc = (A1 * z1 + A2 * z2) / (A1 + A2)
    I = b_pl * t ** 3 / 12 + A1 * (z1 - zc) ** 2 + b_rib * rd ** 3 / 12 + A2 * (z2 - zc) ** 2
    L = 2 * c["leg_y"]
    w = F / L
    M = w * L ** 2 / 8                                             # simply supported (conservative: legs are integral)
    sig = M * max(t - zc, zc + rd) / I
    dl = 5 * w * L ** 4 / (384 * E * I)
    # finger: width = pitch - slot, depth t, load = one tip's share per finger (two half-tips), lever slot end -> engaged x
    bf = P.WELL_PITCH_96 - c["slot8_w"]
    If = bf * t ** 3 / 12
    Ff = F / 8
    lev = (c["slot8_x1"] + c["slot8_w"] / 2) - c["moves"]["pipette_8ch_50"]["x1"]
    sf = Ff * lev * (t / 2) / If
    df = Ff * lev ** 3 / (3 * E * If)
    catch = {tl: round((P.TIP_COLLAR_OD[P.TOOL_TIP[tl]] - (c["slot8_w"] if "8ch" in tl else c["slot1_w"])) / 2, 2)
             for tl in ("pipette_small", "pipette_8ch_50", "pipette_8ch_300")}
    catch_max = {tl: round((P.TIP_COLLAR_OD[P.TOOL_TIP[tl]] - P.TIP_NOZZLE_OD_AT_COLLAR[P.TOOL_TIP[tl]]) / 2, 2) for tl in catch}
    return dict(material="Al 6061-T6 (yield 276 MPa)", design_force_N=F, basis=f"{P.STRIP_COMB_SAFETY} x max STRIP_FORCE_N",
                beam=dict(span_mm=L, I_mm4=round(I, 1), M_Nmm=round(M, 1), stress_MPa=round(sig, 1),
                          deflection_mm=round(dl, 4), safety_on_yield=round(sy / sig, 1)),
                finger=dict(width_mm=round(bf, 2), lever_mm=round(lev, 2), load_N=round(Ff, 1), stress_MPa=round(sf, 1),
                            deflection_mm=round(df, 4), safety_on_yield=round(sy / sf, 1)),
                slots_mm=dict(eight_channel=c["slot8_w"], single=c["slot1_w"]),
                collar_catch_per_side_mm=catch, catch_required_mm=P.COMB_MIN_CATCH,
                catch_pass={k: v >= P.COMB_MIN_CATCH for k, v in catch.items()},
                catch_upper_bound_zero_clearance_mm=catch_max,
                note="catch uses NOMINAL tip collar OD and nozzle OD (params): FAILS until measured values give >= 1 mm; "
                     "if a tip's (collar OD - nozzle OD)/2 - 0.3 < 1 mm no comb can meet it (the p10 nominal gives 0.7 at zero clearance)",
                pass_strength=sy / sig >= 2.0 and sy / sf >= 2.0)


# integration rev D: TC-1 bay sub-plate (DK-2): deck region x <= -41, 0 <= y <= 225 lifts out with TC-1 (CR item 10)
SUBPLATE = (-P.DECK_L / 2, -48.0, 0.0, 217.0)
# integration rev F (review M7): joints moved off every frame screw line (x -48: 1.8 mm clear of the (-41,110) csk, 7.4 mm
# clear of the C2/D2 screws; y 217: 2.8 mm clear of the y=225 csk, 3.4 mm of the D1 taps; y 0 front joint: the two frame
# screws that sat on it are deleted).  DK-2 is doweled to DK-1 through two half-lap tabs over the y=0 beam (DK-2 upper
# half over a DK-1 lower lip, 3 + 3 mm): D4 h6 pin pressed in DK-1 at (-160, 0), D4 slot along X in both at (-100, 0).
# TC-1 locates on DK-2 only (C1 dowel (-132,8) round, D1 dowel (-132,212) slot); every other insert on DK-1 only.
SUB_TABS = [(-172.0, -148.0), (-112.0, -88.0)]
SUB_TAB_Y = 6.0
SUB_DOWELS = [(-160.0, 0.0, "round"), (-100.0, 0.0, "slot_x")]
# integration rev D: reserved volume for a 120 x 88 TEC controller board (TCB1 copy) for the C3 thermal block: on a
# plate on the base members under D3, components up; module bodies in D3 must stay above z -102
TEC_BOARD_RES = (70.0, 192.0, 100.0, 188.0, -136.0, -102.0)   # rev G: -x edge 72 -> 70 (board copy overhung by 0.8 mm)
# rev H: second TEC controller board for the F3 cold module, on a plate on the inner front base member, components up
TEC_BOARD_RES_F3 = (70.0, 192.0, -330.0, -242.0, -136.0, -102.0)
# rev J: both 120 x 88 TEC-board reservations are RETIRED (TCB-1 is not built, ADR-003 #2): the two-channel TEC-1166 (160 x 100 x 24
# + plugs) for C3 and F3 sits on the frame's CEP plate (envelope 180 x 110 x 40 at x 62..242, y -55..55, z -133..-93)
TEC_BOARD_RES_ACTIVE = not getattr(P, "REV_J", False)
CEP_TEC1166_ENV = (66.0, 176.0, -362.0, -182.0, -133.0, -93.0) if getattr(P, "REV_K", False) else (62.0, 242.0, -55.0, 55.0, -133.0, -93.0)
# integration rev E (review A1): tip-check through-beam gate on the A1 tip-waste insert, beside the comb.  Beam along X at
# nest y -5, z 21; tools traverse in Y at nest x -40 so the 8 tips of an eight-channel cross the beam one by one (per-channel).
# rev H: the eight-channel traverse spans 70 mm in Y and reaches into the Y-neighbour nests (F1 in front, B1 behind): the gate is
# RAISED so the tip ends cross the beam above the labware there.  Height is capped from above by the eight-channel heads
# working at A2 column 1: their HP-2B plates hang over A1 x >= 57.8 down to z 74.5 (50 uL, at its ZHS-1 stop) / 80.5 (300 uL)
# and the pipette bodies over x >= 64 down to z 42.  So: sensor top (beam + 5) <= 72.5 (2.0 mm under HP-2B@1), posts end at
# x 62.5 (2.0 mm from the pipette bodies), beam z 67.5, tip ends z 59.5.  Labware in F1 and B1 must top out <= z 56.5
# (<= 51.5 mm tall on the 5 mm seat: reservoir 44, plates, SM-1 30.9 pass; a 72 mm tip rack does NOT -> rule K11/K20).
# Posts 52.5 mm tall (PA12), 7 mm wide in X; taps x 60 -> 59.5 (wall 1.4 mm; 0.25 mm outside the funnel outer face).
_TCY = 0.0
_TCZ = 46.5 if hasattr(P, "FRONT_ROW") else 0.0
TW["tipcheck"] = dict(beam_y=-5.0 + _TCY, beam_z=21.0 + _TCZ,
                      post_x=(55.5, 62.5) if hasattr(P, "FRONT_ROW") else (56.0, 64.0),
                      post_y=(-12.0 + _TCY, 2.0 + _TCY), post_z1=16.0 + _TCZ,
                      sensor_z1=26.0 + _TCZ,
                      m3=[(59.5 if hasattr(P, "FRONT_ROW") else 60.0, y + _TCY) for y in (-9.0, -1.0)], path_x=-42.0, path_y=(-40.0 + _TCY, 30.0 + _TCY),
                      path_y_single=(-15.0 + _TCY, 5.0 + _TCY),   # rev G1: path_x -40 -> -42 (clear of TW-5)
                      tip_below_beam=8.0)   # single channel: short traverse (stays inside the front Y travel)
# integration rev G1: method-B eject hopper TW-5 on TW-1 (MJF PA12): mouth at z 35 is x -34..-10.5 (inner) by y +/-44.5, sloping
# to the TW-1 opening edge (y +/-37) at the plate; the eight-channel row (+/-31.5, collars r 3.7) gets >= 8 mm all round at the
# mouth.  Clear of the comb (x >= -10, z >= 39) and of the tip-check path (tips x -45.7..-38.3).
TW["hopper"] = dict(x_in=(-34.0, -10.5), y_mouth=44.5, y_base=37.0, z_top=35.0, wall=1.5, eject_x=-22.25,
                    m3=[(-22.25, 44.0), (-22.25, -44.0)])
# integration rev E (review A3): lid park pedestal for the PCR sealing lid in D3 (12 mm, so the lid grip band sits at z 17..21.6)
LS = dict(h=12.0, out=(120.0, 78.0))
# integration rev E: reserved volumes for the later-stage modules (cad/lh1/deck_modules2: SM-1 in B1, WM-1 in B3); not built
STAGE_RES = {"RES_SM1_board": (-360.0, -240.0, -212.0, -124.0, -136.0, -102.0),
             "RES_WM1_board": (75.0, 195.0, -212.0, -124.0, -136.0, -102.0),
             "RES_waste_container_and_shelf": (-60.0, 60.0, -215.0, -105.0, -136.0, -83.0),
             "RES_SM1_duct_vertical": (-219.0, -203.0, -77.0, -33.0, -90.0, -8.5),
             "RES_SM1_duct_to_left_panel": (-560.0, -203.0, -77.0, -33.0, -90.0, -50.0),
             "RES_WM1_duct_vertical": (203.0, 218.0, -77.0, -33.0, -90.0, -8.5),
             "RES_WM1_duct_to_front_panel": (200.0, 240.0, -324.0, -33.0, -90.0, -50.0)}
if getattr(P, "REV_K", False):   # rev K: B3 is the second cold module and the TEC-1166 envelope sits here; the later-stage WM-1 is superseded
    STAGE_RES = {k: v for k, v in STAGE_RES.items() if not k.startswith("RES_WM1")}
# ---- nominal labware envelopes for checks (replace with measured models)
LABWARE_H = {"tiprack": 60.0, "reservoir": 44.0, "plate_96_flat": 14.35}

RHO = {"al5052": 2.68e-6, "al6061": 2.70e-6, "pa12": 1.01e-6, "steel": 7.85e-6, "ndfeb": 7.5e-6}   # kg/mm^3


# =====================================================================================
# geometry helpers
# =====================================================================================
def B(x0, x1, y0, y1, z0, z1) -> cq.Shape:
    return cq.Solid.makeBox(x1 - x0, y1 - y0, z1 - z0, pnt=V(x0, y0, z0))


def cyl(x, y, z0, z1, d) -> cq.Shape:
    return cq.Solid.makeCylinder(d / 2, z1 - z0, V(x, y, z0), V(0, 0, 1))


def cylx(x0, x1, y, z, d) -> cq.Shape:
    return cq.Solid.makeCylinder(d / 2, x1 - x0, V(x0, y, z), V(1, 0, 0))


def csk(x, y, ztop, zbot, d, dk) -> cq.Shape:
    depth = (dk - d) / 2
    c = cyl(x, y, zbot - 0.01, ztop + 0.01, d)
    cone = cq.Solid.makeCone(d / 2, dk / 2 + 0.01, depth + 0.01, V(x, y, ztop - depth), V(0, 0, 1))
    return c.fuse(cone)


def prism(pts, vec) -> cq.Shape:
    w = cq.Wire.makePolygon([V(*p) for p in pts], close=True)
    return cq.Solid.extrudeLinear(cq.Face.makeFromWires(w), V(*vec))


def loft_rect(z0, hx0, hy0, z1, hx1, hy1) -> cq.Shape:
    return cq.Workplane("XY").workplane(offset=z0).rect(2 * hx0, 2 * hy0) \
        .workplane(offset=z1 - z0).rect(2 * hx1, 2 * hy1).loft(combine=True).val()


def cut_all(base: cq.Shape, tools) -> cq.Shape:
    tools = [t for t in tools if t is not None]
    return base.cut(*tools).clean() if tools else base


def fuse_all(shapes) -> cq.Shape:
    s = shapes[0]
    return s.fuse(*shapes[1:]).clean() if len(shapes) > 1 else s


def mv(s: cq.Shape, x=0.0, y=0.0, z=0.0) -> cq.Shape:
    return s.translate(V(x, y, z))


def bbox(s: cq.Shape):
    b = s.BoundingBox()
    return b


def env(s: cq.Shape):
    b = cq.BoundBox._fromTopoDS(s.wrapped, optimal=True)
    return [round(b.xlen, 1), round(b.ylen, 1), round(b.zlen, 1)]


def ivol(a: cq.Shape, b: cq.Shape) -> float:
    ba, bb = a.BoundingBox(), b.BoundingBox()
    if (ba.xmin > bb.xmax or bb.xmin > ba.xmax or ba.ymin > bb.ymax or bb.ymin > ba.ymax
            or ba.zmin > bb.zmax or bb.zmin > ba.zmax):
        return 0.0
    try:
        return abs(a.intersect(b).Volume())
    except Exception:   # noqa: BLE001
        return float("nan")


def nest_xy(name):
    return P.nest_centre(name)


def merged_window_rect(pair):
    (x1, y1), (x2, y2) = nest_xy(pair[0]), nest_xy(pair[1])
    cx, cy = (x1 + x2) / 2, (y1 + y2) / 2
    hy = (abs(y2 - y1) + P.WINDOW_W) / 2
    return cx - WIN_HX, cx + WIN_HX, cy - hy - 3.0, cy + hy      # integration rev D: front edge 3 mm forward (TC block had 1 mm)


MERGED_NESTS = {n for pr in MERGED_WINDOWS for n in pr}


# =====================================================================================
# parts
# =====================================================================================
def corner_locator() -> cq.Shape:
    """+X+Y corner locator in nest coordinates (sits on the insert top, z = SEAT .. SEAT+h)."""
    xi, yi, t, h = SBS_HX + LOC["play"], SBS_HY + LOC["play"], SEAT, LOC["h"]
    xo, yo = xi + LOC["wall_x"], yi + LOC["y_arm_t"]
    body = B(LOC["y_arm_x0"], xo, yi, yo, t, t + h).fuse(B(xi, xo, LOC["x_arm_y0"], yo, t, t + h)).clean()
    cy, cx, top = LOC["chamfer_y"], LOC["chamfer_x"], t + h
    wedge_y = prism([(LOC["y_arm_x0"] - 0.01, yi - 0.01, top - cy), (LOC["y_arm_x0"] - 0.01, yi - 0.01, top + 0.01),
                     (LOC["y_arm_x0"] - 0.01, yi + cy, top + 0.01)], (xi - LOC["y_arm_x0"] + 0.02, 0, 0))
    wedge_x = prism([(xi - 0.01, LOC["x_arm_y0"] - 0.01, top - cx), (xi - 0.01, LOC["x_arm_y0"] - 0.01, top + 0.01),
                     (xi + cx, LOC["x_arm_y0"] - 0.01, top + 0.01)], (0, yi - LOC["x_arm_y0"] + 0.02, 0))
    tools = [wedge_y, wedge_x, cyl(xi, yi, t - 1, top + 1, 2.0)]
    tools += [csk(x, y, top, t, M3_CLR, M3_CSK) for x, y in LOC["m3"]]
    return cut_all(body, tools)


def locator_set() -> list[cq.Shape]:
    pp = corner_locator()
    mp = pp.mirror("YZ")
    return [pp, mp, pp.mirror("XZ"), mp.mirror("XZ")]


def locator_tap_points():
    return [(sx * x, sy * y) for x, y in LOC["m3"] for sx in (-1, 1) for sy in (-1, 1)]


# rev K: 12 mm AprilTag 36h11 sticker in a 12.4 x 12.2 x 0.3 pocket on the BACK margin (x 18.8..31.2; the inner 1.3 mm lies under the
# labware skirt: only the sticker's white quiet zone is covered), plus two D2.0 x 1.5 blind holes cut in the same set-up as the dowel
# holes: position and rotation of the nest for VC-1 (nest coordinates).
MARKER = dict(pocket=(18.8, 31.2, 41.4, 53.6), pocket_depth=0.3, holes=[(12.0, 47.5), (38.0, 47.5)], hole_d=2.0, hole_depth=1.5)


def insert_plate(opening=None, tap_m3=(), csk_m3=(), drills=(), clr_m3=()):
    """Standard INSERT plate (5052, INSERT_T) in nest coordinates, z = 0 .. INSERT_T.
    Returns (shape, features) - features feed the DXF layer assignment."""
    t = P.INSERT_T
    s = B(-INS_HX, INS_HX, -INS_HY, INS_HY, 0, t)
    feats, tools = [], []
    if opening:
        tools.append(B(-opening[0] / 2, opening[0] / 2, -opening[1] / 2, opening[1] / 2, -1, t + 1))
    for x, y in P.INSERT_SCREWS:
        tools.append(csk(x, y, t, 0, M4_CLR, M4_CSK))
        feats.append(dict(x=x, y=y, d=M4_CLR, layer="drill", note=f"M4 clearance, countersink 90 deg to {M4_CSK} top"))
    (dx0, dy0), (dx1, dy1) = P.INSERT_DOWELS
    tools.append(cyl(dx0, dy0, -1, t + 1, P.DOWEL_D))
    feats.append(dict(x=dx0, y=dy0, d=P.DOWEL_D, layer="ream", note="dowel D4 H7, ream"))
    sl = INSERT_SLOT_LEN - P.DOWEL_D
    tools += [B(dx1 - P.DOWEL_D / 2, dx1 + P.DOWEL_D / 2, dy1 - sl / 2, dy1 + sl / 2, -1, t + 1),
              cyl(dx1, dy1 - sl / 2, -1, t + 1, P.DOWEL_D), cyl(dx1, dy1 + sl / 2, -1, t + 1, P.DOWEL_D)]
    feats.append(dict(x=dx1, y=dy1, d=P.DOWEL_D, layer="ream", note=f"slot D4 H7 x {INSERT_SLOT_LEN} along Y, mill/ream", slot=True))
    for x, y in tap_m3:
        tools.append(cyl(x, y, -1, t + 1, M3_TAP))
        feats.append(dict(x=x, y=y, d=M3_TAP, layer="tap", note="M3 tapped through"))
    for x, y in csk_m3:
        tools.append(csk(x, y, t, 0, M3_CLR, M3_CSK))
        feats.append(dict(x=x, y=y, d=M3_CLR, layer="drill", note=f"M3 clearance, countersink to {M3_CSK} top"))
    for x, y in clr_m3:
        tools.append(cyl(x, y, -1, t + 1, M3_CLR))
        feats.append(dict(x=x, y=y, d=M3_CLR, layer="drill", note="M3 clearance (pan head)"))
    for x, y, d in drills:
        tools.append(cyl(x, y, -1, t + 1, d))
        feats.append(dict(x=x, y=y, d=d, layer="drill" if d < t else "cut", note="drain hole"))
    if getattr(P, "REV_K", False):     # rev K (ADR-004 d2): tag pocket + two machined blind holes, back margin
        (px0, px1, py0, py1), holes_ = MARKER["pocket"], MARKER["holes"]
        tools.append(B(px0, px1, py0, py1, t - MARKER["pocket_depth"], t + 1))
        tools += [cyl(x, y, t - MARKER["hole_depth"], t + 1, MARKER["hole_d"]) for x, y in holes_]
    return cut_all(s, tools), feats


# ------------------------------------------------------------------ deck plate
def frame_fixing_holes(split=False):
    rows = ([-350.0] if hasattr(P, "FRONT_ROW") else []) + [-225.0, -110.0, 0.0, 110.0, 225.0]   # rev H: + front beam y -350
    xs = [-230.0, -165.0, -66.0, 66.0, 165.0, 230.0]
    # integration rev D: on y = 0 (DK-1/DK-2 joint) the -165 and -66 holes fell within 10 mm of the enlarged bay window:
    # moved to -205 and -58 (still on the y = 0 beam slot, shared by both plates)
    # integration rev F (M7): y=0 row left of the DK-2 joint deleted (the front joint now runs along y=0); -58 -> -38 (DK-1)
    pts = [(x, y) for y in rows for x in ([-38.0, 66.0, 165.0, 230.0] if y == 0.0 else xs)]
    pts += [(sx * 230.0, y) for sx in (-1, 1) for y in ([-275.0] if hasattr(P, "FRONT_ROW") else []) + [-165.0, -55.0, 55.0, 165.0]]   # rev H: + row F
    keep = []
    for x, y in pts:
        if any(_dist_to_rect(x, y, merged_window_rect(pr)) < 10.0 + M5_CSK / 2 for pr in MERGED_WINDOWS):
            continue
        if y == 110.0 and not (BEAMS_X[110.0][0] + 5 <= x):
            continue                       # beam y=110 does not exist left of its post
        keep.append((x, y))
    keep.append((-41.0, 110.0))            # rev D: csk screw into the DP post; rev F: DK-1 only (joint moved to x -48)
    if split:   # two-piece deck: no hole on the joint line y = 0; use M4 pairs either side
        out = []
        for x, y in keep:
            out += [(x, -6.0, "M4"), (x, 6.0, "M4")] if y == 0.0 else [(x, y, "M5")]
        return out
    return [(x, y, "M5") for x, y in keep]


def _dist_to_rect(x, y, r):
    x0, x1, y0, y1 = r
    dx = max(x0 - x, 0, x - x1)
    dy = max(y0 - y, 0, y - y1)
    return math.hypot(dx, dy)


def deck_features(split=False):
    feats, lost = [], []
    windows = []
    for row in P.NEST_NAMES:
        for n in row:
            if n not in MERGED_NESTS:
                x, y = nest_xy(n)
                windows.append((x - WIN_HX, x + WIN_HX, y - WIN_HY, y + WIN_HY))
    mrects = [merged_window_rect(pr) for pr in MERGED_WINDOWS]
    windows += mrects
    for row in P.NEST_NAMES:
        for n in row:
            cx, cy = nest_xy(n)
            for (sx, sy) in P.INSERT_SCREWS:
                f = dict(x=cx + sx, y=cy + sy, d=M4_TAP, layer="tap", note=f"M4 tapped through ({n} insert)", nest=n)
                (lost if any(_dist_to_rect(f["x"], f["y"], r) < M4_TAP / 2 + 1.0 for r in mrects) else feats).append(f)
            for (dx, dy) in P.INSERT_DOWELS:
                f = dict(x=cx + dx, y=cy + dy, d=P.DOWEL_D, layer="ream", note=f"dowel D4 press fit, ream ({n})", nest=n)
                (lost if any(_dist_to_rect(f["x"], f["y"], r) < P.DOWEL_D / 2 + 1.0 for r in mrects) else feats).append(f)
    for x, y, kind in frame_fixing_holes(split):
        d, dk = (M5_CLR, M5_CSK) if kind == "M5" else (M4_CLR, M4_CSK)
        feats.append(dict(x=x, y=y, d=d, dk=dk, layer="drill" if d < P.DECK_T else "cut",
                          note=f"{kind} countersunk to frame beam, csk 90 deg to {dk} top", frame=True))
    # integration: TC-1 asks for 2 x M4 tapped through in the deck margin strip (params TC1_STRIP_TAPPED_HOLES);
    # 6 mm from the deck edge, clear of the x = -230 side beam (frame report: 2 mm thread-to-beam-face gap)
    for x, y in P.TC1_STRIP_TAPPED_HOLES:
        feats.append(dict(x=x, y=y, d=M4_TAP, layer="tap", note="M4 tapped through (TC-1 lid strip)", nest="TC1_strip"))
    for x, y, kind in SUB_DOWELS:          # integration rev F (M7): DK-1/DK-2 dowels in the half-lap tabs
        feats.append(dict(x=x, y=y, d=P.DOWEL_D, layer="ream", nest="DK1_DK2_joint", slot=(kind != "round"),
                          note="D4 h6 pin pressed in DK-1 lip; DK-2: ream D4 H7" + (" slot 6 along X" if kind != "round" else "")))
    return windows, feats, lost


def deck_plate(split=False, part="main"):
    """part: 'main' = DK-1 without the TC bay sub-plate, 'sub' = DK-2 sub-plate, 'full' = both (integration rev D)"""
    t = P.DECK_T
    s = B(-P.DECK_L / 2, P.DECK_L / 2, DECK_Y0, DECK_Y0 + P.DECK_W, -t, 0)      # rev H: y -360..250
    windows, feats, lost = deck_features(split)
    tools = [B(x0, x1, y0, y1, -t - 1, 1) for x0, x1, y0, y1 in windows]
    for f in feats:
        if f.get("slot"):
            tools += [B(f["x"] - 3.0, f["x"] + 3.0, f["y"] - f["d"] / 2, f["y"] + f["d"] / 2, -t - 1, 1),
                      cyl(f["x"] - 3.0, f["y"], -t - 1, 1, f["d"]), cyl(f["x"] + 3.0, f["y"], -t - 1, 1, f["d"])]
            continue
        tools.append(csk(f["x"], f["y"], 0, -t, f["d"], f["dk"]) if f.get("frame") else cyl(f["x"], f["y"], -t - 1, 1, f["d"]))
    full = cut_all(s, tools)
    # integration rev F (M7): DK-2 region incl. the half-lap tabs (DK-2 upper half over a DK-1 lower lip, y -6..+6)
    sub = B(SUBPLATE[0] - 1, SUBPLATE[1], SUBPLATE[2], SUBPLATE[3], -t - 1, 1)
    for xa, xb in SUB_TABS:
        sub = sub.cut(B(xa, xb, 0.0, SUB_TAB_Y, -t - 1, -t / 2)).fuse(B(xa, xb, -SUB_TAB_Y, 0.01, -t / 2, 1))
    if part == "sub":
        return full.intersect(sub).clean(), windows, feats, lost
    if part == "main":
        return full.cut(sub).clean(), windows, feats, lost
    return full, windows, feats, lost


def dovetail_positions(feats):
    near = [(f["x"]) for f in feats if abs(f["y"]) < 12.0]
    xs, last = [], -1e9
    for xi in range(-235, 236):
        x = float(xi)
        if all(abs(x - h) >= 13.0 for h in near) and x - last >= 70.0:
            xs.append(x)
            last = x
    return xs


def deck_halves():
    """Two-piece option: split on y = 0 over the central beam, joined by laser-cut dovetails."""
    deck, windows, feats, lost = deck_plate(split=True)
    t = P.DECK_T
    xs = dovetail_positions(feats)
    front_region = B(-300, 300, DECK_Y0 - 10, 0, -t - 1, 1)                # rev H: front half y -360..0
    tabs = [prism([(x - 5.0, -0.01, -t - 1), (x + 5.0, -0.01, -t - 1), (x + 7.0, 8.0, -t - 1), (x - 7.0, 8.0, -t - 1)],
                  (0, 0, t + 2)) for x in xs]
    tabs_clr = [prism([(x - 5.1, -0.01, -t - 1), (x + 5.1, -0.01, -t - 1), (x + 7.1, 8.1, -t - 1), (x - 7.1, 8.1, -t - 1)],
                      (0, 0, t + 2)) for x in xs]
    # rev H fix: the DK-1 half-lap lips under DK-2 (SUB_TABS, y 0..6, lower half) hang off the FRONT half; cut at y 0 they were left
    # as two loose 3 mm chips in the back half's STEP/DXF.  They now stay on DK-1F.
    lips = [B(xa, xb, -0.01, SUB_TAB_Y + 0.01, -t - 1, -t / 2) for xa, xb in SUB_TABS]
    front = deck.intersect(fuse_all([front_region] + tabs + lips)).clean()
    back = deck.cut(front_region, *tabs_clr, *lips).clean()
    return front, back, feats, xs


# ------------------------------------------------------------------ magnet module
def pcr_plate(semi=False) -> cq.Shape:
    """Nominal 96 x 0.2 mL PCR plate, local origin at the skirt bottom (full) - wells solid for checks."""
    p = PCR
    z_sk = p["semi_skirt_bottom"] if semi else 0.0
    outer = B(-SBS_HX, SBS_HX, -SBS_HY, SBS_HY, z_sk, p["deck_top"])
    inner = B(-SBS_HX + p["skirt_t"], SBS_HX - p["skirt_t"], -SBS_HY + p["skirt_t"], SBS_HY - p["skirt_t"],
              z_sk - 1, p["deck_top"] - p["deck_t"])
    shell = outer.cut(inner)
    wells = []
    one = cq.Solid.makeCone(p["od_bot"] / 2, p["od_cone_top"] / 2, p["cone_h"], V(0, 0, p["well_bottom"]), V(0, 0, 1)) \
        .fuse(cyl(0, 0, p["well_bottom"] + p["cone_h"], p["H"], p["od_cyl"]))
    for c in range(12):
        for r in range(8):
            x = -SBS_HX + P.A1_OFFSET_X + c * P.WELL_PITCH_96
            y = SBS_HY - P.A1_OFFSET_Y - r * P.WELL_PITCH_96
            wells.append(mv(one, x, y, 0))
    return cq.Compound.makeCompound([shell] + wells)


def well_od_at(h_above_bottom):
    p = PCR
    if h_above_bottom <= 0:
        return p["od_bot"]
    if h_above_bottom <= p["cone_h"]:
        return p["od_bot"] + (p["od_cone_top"] - p["od_bot"]) * h_above_bottom / p["cone_h"]
    return p["od_cyl"]


def magnet_rows_x():
    x_c1 = -SBS_HX + P.A1_OFFSET_X
    return [x_c1 + (2 * k + 0.5) * P.WELL_PITCH_96 for k in range(6)]     # every other column gap


def magnet_geometry():
    """Heights in nest coordinates for both plate types."""
    wb_full = SEAT + PCR["well_bottom"]
    wb_semi = SEAT - (PCR["semi_skirt_bottom"] - PCR["well_bottom"])
    prot = MAG["block"][2] - MAG["seat_depth"]
    top_full = wb_full + MAG["pellet_top"] - prot          # carrier top, engaged, full skirt
    top_semi = wb_semi + MAG["pellet_top"] - prot
    home = top_full - MAG["travel"]
    return dict(well_bottom_full=wb_full, well_bottom_semi=wb_semi, magnet_protrusion=prot,
                carrier_top_engaged_full=top_full, carrier_top_engaged_semi=top_semi, carrier_top_home=home,
                magnet_top_engaged_full=top_full + prot, magnet_top_engaged_semi=top_semi + prot,
                magnet_top_home=home + prot)


def mb1_geometry():
    g = magnet_geometry()
    top = g["well_bottom_full"] + MB["pellet_top"] - g["magnet_protrusion"]   # body top = magnet pocket rim
    return dict(body_top=top, magnet_top=top + g["magnet_protrusion"], magnet_bottom=top + g["magnet_protrusion"] - MAG["block"][2],
                well_bottom=g["well_bottom_full"])


def mb1_body() -> cq.Shape:
    """MB-1 (MJF PA12, integration rev F): passive magnet block.  Carrier-sized body (97 x 73 x 30) with the 6 magnet-row
    pockets, two ears under the MM-1 insert plate (6 x M3 heat-set inserts at the MM housing-screw positions)."""
    m = mb1_geometry()
    L, W, _ = MAG["carrier"]
    H = MB["body_h"]
    top = m["body_top"]
    body = B(-L / 2, L / 2, -W / 2, W / 2, top - H, top)
    ex0, ex1 = MB["ears_x"]
    ears = [B(ex0 - 0.01, ex1, -W / 2, W / 2, -MB["ear_t"], 0.0), B(-ex1, -ex0 + 0.01, -W / 2, W / 2, -MB["ear_t"], 0.0)]
    webs = [B(ex0 - 0.01, ex0 + 4.0, -W / 2, W / 2, top - H, 0.0), B(-ex0 - 4.0, -ex0 + 0.01, -W / 2, W / 2, top - H, 0.0)]
    s = fuse_all([body] + ears + webs)
    mw, ml, mh = MAG["block"]
    rl = MAG["per_row"] * ml
    tools = [B(x - mw / 2 - 0.08, x + mw / 2 + 0.08, -rl / 2 - 0.15, rl / 2 + 0.15, top - MAG["seat_depth"], top + 1) for x in magnet_rows_x()]
    tools += [cyl(x, y, -MB["ear_t"] - 1, 0.01, 4.0) for x, y in MAG["housing_screws"]]         # heat-set insert bores
    tools += [B(-30.0, 30.0, -W / 2 - 1, W / 2 + 1, top - H - 1, top - 12.0)]                   # lightening / magnet push-out window
    return cut_all(s, tools)


def _field_cuboids():
    """(centre, size, magnetisation sign along X) for the 42 MB-1 magnets in nest coordinates"""
    m = mb1_geometry()
    mw, ml, mh = MAG["block"]
    out = []
    for k, x in enumerate(magnet_rows_x()):
        for j in range(MAG["per_row"]):
            y = -MAG["per_row"] * ml / 2 + (j + 0.5) * ml
            out.append(((x, y, m["magnet_bottom"] + mh / 2), (mw, ml, mh), 1.0 if k % 2 == 0 else -1.0))
    return out


def mb1_field(p, n=6):
    """|B| in mT at point p (nest coords) from the 42 magnets: surface-charge model (sigma = Br/mu0 on the two X faces,
    each face split into n x 2n point charges).  Accurate to a few % beyond ~5 mm from a magnet."""
    bx = by = bz = 0.0
    for (cx, cy, cz), (w, l, h), sg in _field_cuboids():
        dA = (l / n) * (h / (2 * n))
        for face, q in ((cx + w / 2, sg), (cx - w / 2, -sg)):
            for i in range(n):
                for j in range(2 * n):
                    yy = cy - l / 2 + (i + 0.5) * l / n
                    zz = cz - h / 2 + (j + 0.5) * h / (2 * n)
                    rx, ry, rz = p[0] - face, p[1] - yy, p[2] - zz
                    r3 = (rx * rx + ry * ry + rz * rz) ** 1.5
                    f = q * dA / r3
                    bx, by, bz = bx + f * rx, by + f * ry, bz + f * rz
    return MB["br_t"] / (4 * math.pi) * math.sqrt(bx * bx + by * by + bz * bz) * 1000.0


def mb1_checks():
    """integration rev F: MB-1 pellet position vs volume, elution draw, pull-off vs grip and Z thrust, stray field."""
    m = mb1_geometry()
    wall = 0.25                                            # ASSUMED PCR well wall (measure)
    r0, r1, hc, rc = PCR["od_bot"] / 2 - wall, PCR["od_cone_top"] / 2 - wall, PCR["cone_h"], PCR["od_cyl"] / 2 - wall

    def vol(h):
        if h <= hc:
            r = r0 + (r1 - r0) * h / hc
            return math.pi * h / 3 * (r0 * r0 + r0 * r + r * r)
        return vol(hc) + math.pi * rc * rc * (h - hc)

    def height(v):
        lo, hi = 0.0, 40.0
        for _ in range(60):
            mid = (lo + hi) / 2
            lo, hi = (mid, hi) if vol(mid) < v else (lo, mid)
        return lo

    def radius(h):
        return r0 + (r1 - r0) * min(h, hc) / hc if h <= hc else rc
    vols = [8, 9, 10, 20, 50, 100, 150, 200]
    liquid = {f"{v}_uL": {"height_above_inner_bottom_mm": round(height(v), 2),
                          "pellet_zone": ("pellet in the liquid on the magnet-side wall, at 0.."
                                          f"{round(min(height(v), MB['pellet_top']), 1)} mm (magnet top edge at {MB['pellet_top']} mm)")}
              for v in vols}
    usable = vol(PCR["H"] - PCR["well_bottom"] - 0.5)
    # elution: 9 uL in the well (8 + 1 dead), draw 8 with a p10 tip offset away from the pellet
    h9 = height(9.0)
    tip_end_od, offset, pellet_t = 0.8, 0.4, 0.3     # ASSUMED p10 tip end OD, tip offset from centre, bead clean-up pellet thickness
    gap = radius(min(h9, 1.0)) + offset - tip_end_od / 2 - pellet_t
    # pull-off: magnets act on the beads only (plate is polypropylene)
    v_mag = 0.1e-6 / 5200.0                          # ASSUMED <= 0.1 mg magnetite-equivalent beads per well
    grad = 100.0                                     # T/m, ASSUMED order of the gradient at a side magnet
    f_beads = 96 * 4.8e5 * v_mag * grad              # saturated magnetite Ms 4.8e5 A/m
    w_plate = (0.007 + 96 * 0.2e-3) * 9.81           # plate + 200 uL in every well
    f_need = f_beads + w_plate
    grip = 2 * 0.5 * 20.0                            # 2 pads, mu 0.5, 20 N clamp (head gripper_force: worst max 23.5 N)
    zthrust = 219.0                                  # head z_thrust: worst lift force (mu 0.20)
    nx, ny = P.NEST_PITCH_X, P.NEST_PITCH_Y
    pts = {"neighbour_plate_nearest_well_X (B1/B3), bead height": (nx - (SBS_HX - P.A1_OFFSET_X), 0.0, m["well_bottom"] + 3.0),
           "neighbour_plate_nearest_well_Y (A2/C2), bead height": (0.0, ny - (SBS_HY - P.A1_OFFSET_Y), m["well_bottom"] + 3.0),
           "gripper NEMA11 / plunger NEMA11 when working B2 (>= 100 mm above the magnets)": (0.0, 0.0, m["magnet_top"] + 100.0),
           "tool parts at 30 mm above the magnet top (pipette springs)": (0.0, 0.0, m["magnet_top"] + 30.0),
           "head Z motors at transit (>= 280 mm above)": (0.0, 0.0, m["magnet_top"] + 280.0),
           "C3 thermal module / TEC board (row neighbour, 132 mm)": (nx, -ny, 0.0),
           "frame Y motor / nearest stepper (> 300 mm)": (300.0, 0.0, 0.0)}
    field = {k: round(mb1_field(v), 3) for k, v in pts.items()}
    pellet_side = {"odd columns (1,3,..11)": "+X wall (magnet row in the gap to their right)",
                   "even columns (2,..12)": "-X wall"}
    return dict(magnet_top_z=round(m["magnet_top"], 2), magnet_top_above_inner_bottom_mm=MB["pellet_top"],
                field_direction=("each row magnetised along X (through its 2 mm); " + MB["polarity"]),
                pellet_side=pellet_side, liquid_heights=liquid,
                well_usable_volume_uL=round(usable, 1),
                elution_9uL=dict(liquid_height_mm=round(h9, 2), tip_to_pellet_gap_mm=round(gap, 2),
                                 verdict=("8 uL can be drawn with the p10 tip offset 0.4 mm away from the pellet side, ~"
                                          f"{round(gap, 2)} mm from the pellet surface (before the XY positioning stack)"),
                                 assumptions="tip end OD 0.8, pellet 0.3 thick, wall 0.25: MEASURE (BR-1 + a bead trial)"),
                pull_off=dict(force_needed_N=round(f_need, 2), bead_magnetic_force_upper_estimate_N=round(f_beads, 3),
                              plate_and_liquid_weight_N=round(w_plate, 2), gripper_holding_N=grip,
                              margin_vs_grip=round(grip / f_need, 1), z_lift_force_worst_N=zthrust,
                              margin_vs_z=round(zthrust / f_need, 0),
                              note="the plate is polypropylene: the magnets pull only on the beads (estimate <= 0.1 mg magnetite-"
                                   "equivalent per well at ~100 T/m). Lift the plate off at <= 10 mm/s and <= 1 m/s^2 so the "
                                   "liquid does not slosh; the pellet stays on the wall by adhesion until resuspended off the magnet"),
                stray_field_mT=field,
                stray_field_limits=("bead pelleting needs tens of mT; < 1 mT at a neighbouring plate is harmless; steppers and "
                                    "mechanical (D2F) or optical switches are unaffected below ~1 mT; Earth field 0.05 mT"),
                handling=["N52 blocks snap together: handle one at a time, keep loose magnets >= 100 mm apart, wear gloves and glasses",
                          "fill the pockets with a non-magnetic (PA/brass) push stick from a jig; mark polarity with a pen before insertion",
                          "insert rows in the alternating order: check each with a compass/polarity tester before pressing",
                          "keep phones, cards, steel tools and implanted-device wearers away (>= 0.5 m) during assembly",
                          "do not drill, heat > 80 C (N52 working limit) or drop (brittle, chips fly)",
                          "store and ship the finished MB-1 with a steel keeper plate over the rows"],
                magnet_stays_magnetised_note="N52 max working 80 C: B2 is not next to the thermocycler exhaust; check B2 air temperature at commissioning")


def magnet_carrier() -> cq.Shape:
    """Carrier (MJF PA12), local z: top = 0, bottom = -H."""
    L, W, H = MAG["carrier"]
    s = B(-L / 2, L / 2, -W / 2, W / 2, -H, 0)
    mw, ml, mh = MAG["block"]
    rl = MAG["per_row"] * ml
    tools = [B(x - mw / 2 - 0.08, x + mw / 2 + 0.08, -rl / 2 - 0.15, rl / 2 + 0.15, -MAG["seat_depth"], 1) for x in magnet_rows_x()]
    for x, y in MAG["rod_pts"]:
        tools.append(cyl(x, y, -H - 1, 1, MAG["rod_d"] + 0.6))
        tools.append(cyl(x, y, -H - 1, -H + MAG["bushing"][1], MAG["bushing"][0]))
    tools.append(cyl(0, 0, -H - 1, -H + 28.0, MAG["screw_d"] + 2.0))
    tools += [cyl(r * math.cos(a), r * math.sin(a), -H - 1, -H + 8, 4.0)
              for r in (8.0,) for a in (math.pi / 4 + k * math.pi / 2 for k in range(4))]
    s = cut_all(s, tools)
    fx0, fx1, fy0, fy1, fh = MAG["flag"]
    return s.fuse(B(fx0, fx1, fy0, fy1, -H - fh, -H)).clean()


def magnet_set(carrier_top) -> cq.Shape:
    mw, ml, mh = MAG["block"]
    z0 = carrier_top - MAG["seat_depth"]
    mags = []
    for x in magnet_rows_x():
        for k in range(MAG["per_row"]):
            y0 = -MAG["per_row"] * ml / 2 + k * ml
            mags.append(B(x - mw / 2, x + mw / 2, y0 + 0.02, y0 + ml - 0.02, z0, z0 + mh))
    return cq.Compound.makeCompound(mags)


def magnet_housing(g) -> cq.Shape:
    """U-frame (MJF PA12) hanging from the magnet insert plate: two end walls + floor, Y sides open."""
    x0, x1 = MAG["housing_x"]
    hy = MAG["housing_hy"]
    z_floor_top = g["carrier_top_home"] - MAG["carrier"][2] - MAG["nut_body"][1] - 1.0
    z_floor_bot = z_floor_top - MAG["floor_t"]
    walls = [B(x0, x1, -hy, hy, z_floor_bot, 0), B(-x1, -x0, -hy, hy, z_floor_bot, 0)]
    floor = B(-x1, x1, -hy, hy, z_floor_bot, z_floor_top)
    bosses = [cyl(x, y, -12.0, 0, 8.0).intersect(B(-x1, x1, -hy, hy, -13, 0)) for x, y in MAG["housing_screws"]]
    rod_boss = [cyl(x, y, z_floor_top, z_floor_top + 8.0, 12.0) for x, y in MAG["rod_pts"]]
    s = fuse_all(walls + [floor] + bosses + rod_boss)
    tools = [cyl(x, y, -10.0, 0.1, 4.0) for x, y in MAG["housing_screws"]]       # M3 heat-set inserts
    tools += [cyl(x, y, z_floor_bot - 1, z_floor_top + 9, MAG["rod_d"]) for x, y in MAG["rod_pts"]]
    tools.append(cyl(0, 0, z_floor_bot - 1, z_floor_top + 1, 22.5))
    tools += [cyl(sx * 15.5, sy * 15.5, z_floor_bot - 1, z_floor_top + 1, M3_CLR) for sx in (-1, 1) for sy in (-1, 1)]
    tools += [cyl(34.5, y, z_floor_bot - 1, z_floor_top + 1, M3_CLR) for y in (-9.5, 9.5)]   # sensor screws
    return cut_all(s, tools), z_floor_top, z_floor_bot


def magnet_bought(g, z_floor_top, z_floor_bot, carrier_top):
    H = MAG["carrier"][2]
    cb = carrier_top - H
    m, ml = MAG["motor"]
    motor = B(-m / 2, m / 2, -m / 2, m / 2, z_floor_bot - ml, z_floor_bot).fuse(cyl(0, 0, z_floor_bot, z_floor_bot + 2, 22.0)) \
        .cut(cyl(0, 0, z_floor_bot - 1, z_floor_bot + 3, MAG["screw_d"] + 0.4))
    screw_top = g["carrier_top_engaged_full"] - H + 2.0
    screw = cyl(0, 0, z_floor_bot, screw_top, MAG["screw_d"])
    nd, nl = MAG["nut_body"]
    fd, ft = MAG["nut_flange"]
    nut = cyl(0, 0, cb - nl, cb, nd).fuse(cyl(0, 0, cb - ft, cb, fd)).cut(cyl(0, 0, cb - nl - 1, cb + 1, MAG["screw_d"] + 0.2))
    rods = [cyl(x, y, z_floor_bot, -1.0, MAG["rod_d"] - 0.01) for x, y in MAG["rod_pts"]]
    bush = [cyl(x, y, cb, cb + MAG["bushing"][1], MAG["bushing"][0] - 0.01).cut(cyl(x, y, cb - 1, cb + 30, MAG["rod_d"] + 0.2))
            for x, y in MAG["rod_pts"]]
    fx0, fx1 = MAG["flag"][0], MAG["flag"][1]
    sensor = fuse_all([B(fx0 - 2.6, fx0 - 0.1, -3, 3, z_floor_top, z_floor_top + 11.0),
                       B(fx1 + 0.1, fx1 + 2.6, -3, 3, z_floor_top, z_floor_top + 11.0),
                       B(fx0 - 2.6, fx1 + 2.6, -12.5, 12.5, z_floor_top, z_floor_top + 3.0)])
    sensor = sensor.cut(B(fx0 - 0.1, fx1 + 0.1, -3.1, 3.1, z_floor_top + 3.0 - 0.01, z_floor_top + 12))
    return dict(motor=motor, lead_screw=screw, nut=nut, rods=cq.Compound.makeCompound(rods),
                bushings=cq.Compound.makeCompound(bush), home_sensor=sensor), screw_top


# ------------------------------------------------------------------ thermal module
def thermal_frame() -> cq.Shape:
    fx, fy, ft = TH["frame_flange"]
    z0 = SEAT
    flange = B(-fx / 2, fx / 2, -fy / 2, fy / 2, z0, z0 + ft)
    so, si = TH["skirt_out"], TH["skirt_in"]
    skirt = B(-so[0] / 2, so[0] / 2, -so[1] / 2, so[1] / 2, TH["skirt_bottom"], z0 + ft)
    s = flange.fuse(skirt).cut(B(-si[0] / 2, si[0] / 2, -si[1] / 2, si[1] / 2, TH["skirt_bottom"] - 1, z0 + ft + 1))
    zt = z0 + ft
    pads = [B(sx * 56.0 - 1.5, sx * 56.0 + 1.5, sy * 25.0 - 4, sy * 25.0 + 4, zt, zt + TH["pad_h"]) for sx in (-1, 1) for sy in (-1, 1)]
    pads += [B(-3, 3, sy * 38.0 - 2, sy * 38.0 + 2, zt, zt + TH["pad_h"]) for sy in (-1, 1)]
    s = fuse_all([s] + pads)
    groove = B(-61.2, 61.2, -41.5, 41.5, zt - 1.0, zt + 1).cut(B(-59.7, 59.7, -40.3, 40.3, zt - 2, zt + 2))
    tools = [groove, cyl(TH["drain"][0], TH["drain"][1], z0 - 1, zt + 1, 3.0)]
    tools += [csk(x, y, zt, z0, M3_CLR, M3_CSK) for x, y in TH["frame_screws"]]
    return cut_all(s, tools)


def thermal_block() -> cq.Shape:
    lx, ly, lt = TH["lip"]
    bx, by = TH["lower"]
    top, bot = TH["block_top"], TH["block_bottom"]
    s = B(-lx / 2, lx / 2, -ly / 2, ly / 2, top - lt, top).fuse(B(-bx / 2, bx / 2, -by / 2, by / 2, bot, top - lt)).clean()
    d, depth, cone_h, cone_deg = TH["tube_pocket"]
    tools = []
    for c in range(6):
        for r in range(4):
            x, y = (c - 2.5) * TH["tube_pitch"], (r - 1.5) * TH["tube_pitch"]
            tools.append(cyl(x, y, top - depth, top + 1, d))
            # integration rev E (DFM): cylindrical flat-bottom bores (drill + end mill); the tapered cone is dropped
    for x, y in TH["clamp"]:
        tools.append(cyl(x, y, bot - 1, bot + 8.0, M3_TAP))
    tools.append(cylx(bx / 2 - 25.0, bx / 2 + 1, 18.0, -13.0, 3.2))       # NTC thermistor pocket
    tools.append(cylx(-bx / 2 - 1, -bx / 2 + 22.0, -18.0, -13.0, 5.2))    # thermal cut-off (TCO) pocket
    return cut_all(s, tools)


def thermal_lower_stack():
    tx, ty, tt = TH["tec"]
    zb = TH["block_bottom"]
    tecs = [B(sx * TH["tec_x"] - tx / 2, sx * TH["tec_x"] + tx / 2, -ty / 2, ty / 2, zb - tt, zb) for sx in (-1, 1)]
    bx, by = TH["lower"]
    foam = B(-bx / 2, bx / 2, -by / 2, by / 2, zb - tt, zb)
    foam = cut_all(foam, [B(sx * TH["tec_x"] - tx / 2 - 0.2, sx * TH["tec_x"] + tx / 2 + 0.2, -ty / 2 - 0.2, ty / 2 + 0.2,
                            zb - tt - 1, zb + 1) for sx in (-1, 1)] + [cyl(x, y, zb - tt - 1, zb + 1, 4.0) for x, y in TH["clamp"]])
    sx_, sy_, sbt, sfh = TH["sink"]
    z_st = zb - tt
    base = B(-sx_ / 2, sx_ / 2, -sy_ / 2, sy_ / 2, z_st - sbt, z_st)
    fins = []
    n = int(round(sy_ / TH["fin_pitch"]))
    for k in range(n):
        y = -sy_ / 2 + TH["fin_pitch"] / 2 + k * TH["fin_pitch"] + TH["fin_pitch"] / 2
        if abs(y) + TH["fin_t"] / 2 > sy_ / 2:
            continue
        fins.append(B(-sx_ / 2, sx_ / 2, y - TH["fin_t"] / 2, y + TH["fin_t"] / 2, z_st - sbt - sfh, z_st - sbt + 0.01))
    sink = fuse_all([base] + fins)
    sink = cut_all(sink, [cyl(x, y, z_st - sbt - 1, z_st + 1, M3_CLR) for x, y in TH["clamp"]]
                   + [cylx(sx * sx_ / 2 - 6 * (sx > 0) - 0.01 * (sx < 0), sx * sx_ / 2 + 6 * (sx < 0) + 0.01 * (sx > 0), 0.0,
                           z_st - sbt / 2, M3_TAP) for sx in (-1, 1)])
    z_sink_bot = z_st - sbt - sfh
    shroud_top, shroud_t = z_sink_bot - 0.2, 3.0
    fan_d, fan_t = TH["fan"]
    shroud = B(-sx_ / 2, sx_ / 2, -sy_ / 2, sy_ / 2, shroud_top - shroud_t, shroud_top)
    tabs = [B(sx_ / 2, sx_ / 2 + 2.0, -10, 10, shroud_top - shroud_t, z_st - 2.0), B(-sx_ / 2 - 2.0, -sx_ / 2, -10, 10, shroud_top - shroud_t, z_st - 2.0)]
    shroud = fuse_all([shroud] + tabs)
    shroud = cut_all(shroud, [cyl(0, 0, shroud_top - shroud_t - 1, shroud_top + 1, 57.0)]
                     + [cyl(sx * 25, sy * 25, shroud_top - shroud_t - 1, shroud_top + 1, M3_CLR) for sx in (-1, 1) for sy in (-1, 1)]
                     + [cylx(sx_ / 2 - 1, sx_ / 2 + 3, 0.0, z_st - sbt / 2, M3_CLR), cylx(-sx_ / 2 - 3, -sx_ / 2 + 1, 0.0, z_st - sbt / 2, M3_CLR)])
    z_fan_top = shroud_top - shroud_t
    fan = B(-fan_d / 2, fan_d / 2, -fan_d / 2, fan_d / 2, z_fan_top - fan_t, z_fan_top).cut(cyl(0, 0, z_fan_top - fan_t - 1, z_fan_top + 1, 56.0))
    return dict(tecs=cq.Compound.makeCompound(tecs), tec_foam=foam, heat_sink=sink, fan_shroud=shroud, fan=fan), z_fan_top - fan_t


# ------------------------------------------------------------------ tip waste insert
def tip_funnel() -> cq.Shape:
    fx, fy = TW["funnel_out"]
    zb = TW["funnel_bottom"]
    ox, oy = TW["opening"]
    outer = loft_rect(zb, TW["outlet_out"][0] / 2, TW["outlet_out"][1] / 2, 0, fx / 2, fy / 2)
    inner = loft_rect(zb - 0.01, TW["outlet_in"][0] / 2, TW["outlet_in"][1] / 2, 0.01, ox / 2, fy / 2 - 2.0)
    body = outer.cut(inner)
    hy = fy / 2
    rz = TW["rail_z"]
    rails = []
    for sx in (-1, 1):
        x_out, x_wall, x_ledge = sx * fx / 2, sx * (fx / 2 - 2.0), sx * (TW["bin_body_hx"] + 2.0)
        rails.append(B(min(x_out, x_wall), max(x_out, x_wall), -hy, hy, rz - 3.0, -10.0))             # hanger wall
        rails.append(B(min(x_out, x_ledge), max(x_out, x_ledge), -hy, hy, rz - 3.0, rz))             # ledge
        rails.append(B(min(x_out, x_ledge), max(x_out, x_ledge), hy - 2.0, hy, rz, rz + 4.0))        # back stop
    bosses = [cyl(x, y, -12.0, 0, 8.0).intersect(B(-fx / 2, fx / 2, -hy, hy, -13, 0)) for x, y in TW["screws"]]
    s = fuse_all([body] + rails + bosses)
    return cut_all(s, [cyl(x, y, -10.0, 0.1, 4.0) for x, y in TW["screws"]])


def tip_bin() -> cq.Shape:
    hx, fhx = TW["bin_body_hx"], TW["bin_flange_hx"]
    y0, y1 = TW["bin_y"]
    zt, zb, w = TW["rail_z"] + 2.5, TW["bin_bottom"], TW["bin_wall"]
    s = B(-hx, hx, y0, y1, zb, zt).cut(B(-hx + w, hx - w, y0 + w, y1 - w, zb + w, zt + 1))
    fl = [B(-fhx, -hx + 0.01, y0, y1, TW["rail_z"], zt), B(hx - 0.01, fhx, y0, y1, TW["rail_z"], zt)]
    handle = B(-20, 20, y0 - 8.0, y0 + 0.01, zt - 14.0, zt - 4.0).cut(B(-14, 14, y0 - 6.0, y0 - 2.0, zt - 15, zt - 3))
    return fuse_all([s] + fl + [handle])


def _leadin(xm, y, w, li, dirn, z0, z1):
    """45 deg in-plane lead-in at a slot mouth at x = xm, slot going in direction dirn (+1: slot runs +X from the mouth)"""
    pts = [(xm - dirn * 0.01, y - w / 2 - li, z0), (xm + dirn * li, y - w / 2, z0), (xm + dirn * li, y + w / 2, z0),
           (xm - dirn * 0.01, y + w / 2 + li, z0)]
    return prism(pts, (0, 0, z1 - z0))


def strip_comb():
    """TW-4D (integration rev F, review M3): CNC Al 6061-T6, hard anodised.  Plate at z_under .. +t with 8 slots (enter
    from -X) and 1 slot (single channel, from +X), lead-ins at every mouth, a rib under the plate behind the slot ends,
    two end legs standing on TW-1 (M3 counterbored from the top).  No bends."""
    c = TW["comb"]
    x0, x1 = c["x"]
    zu, t = c["z_under"], c["t"]
    rx0, rx1, rd = c["rib"]
    plate = B(x0, x1, -c["hy"], c["hy"], zu, zu + t)
    legs = [B(x0, x1, c["leg_y"], c["foot_y"], SEAT, zu + t), B(x0, x1, -c["foot_y"], -c["leg_y"], SEAT, zu + t)]
    rib = B(rx0, rx1, -c["leg_y"] - 0.01, c["leg_y"] + 0.01, zu - rd, zu + 0.01)
    s = fuse_all([plate, rib] + legs)
    tools = []
    li = c["leadin"]
    for k in range(8):
        y = (k - 3.5) * P.WELL_PITCH_96
        w = c["slot8_w"]
        tools += [B(x0 - 1, c["slot8_x1"], y - w / 2, y + w / 2, zu - 1, zu + t + 1), cyl(c["slot8_x1"], y, zu - 1, zu + t + 1, w),
                  _leadin(x0, y, w, li, +1, zu - 1, zu + t + 1)]
    w = c["slot1_w"]
    xe = c["slot1_x0"] + w / 2
    tools += [B(xe, x1 + 1, -w / 2, w / 2, zu - 1, zu + t + 1), cyl(xe, 0.0, zu - 1, zu + t + 1, w),
              _leadin(x1, 0.0, w, li, -1, zu - 1, zu + t + 1)]
    for x, y in c["feet_m3"]:
        tools += [cyl(x, y, SEAT - 1, zu + t + 1, M3_CLR), cyl(x, y, SEAT + 8.0, zu + t + 1, 6.5)]
    return cut_all(s, tools)


def tip_hopper():
    """TW-5 (rev G1): open hopper over the funnel for method-B ejection (nest coords)"""
    h = TW["hopper"]
    x0, x1 = h["x_in"]
    w, zt, z0 = h["wall"], h["z_top"], SEAT
    parts = []
    for sy in (-1, 1):   # sloped Y walls: inner face from y_base at the plate to y_mouth at the top
        pts = [(x0 - w, sy * h["y_base"], z0), (x0 - w, sy * (h["y_base"] + w + 4.0), z0), (x0 - w, sy * (h["y_mouth"] + w), zt),
               (x0 - w, sy * h["y_mouth"], zt)]
        parts.append(prism(pts, (x1 - x0 + w, 0, 0)))
    # one vertical X wall on the -X side only: the +X side stays open because the eight-channel strip move carries its tips
    # through x -22 .. 9 under the comb (rev G1 whole-machine check); the comb edge (x -10) bounds the mouth there
    parts.append(B(x0 - w, x0, -h["y_mouth"] - w, h["y_mouth"] + w, z0, zt))
    s = fuse_all(parts)
    s = s.cut(prism([(x0, -h["y_base"], z0 - 1), (x0, h["y_base"], z0 - 1), (x0, h["y_mouth"], zt + 1), (x0, -h["y_mouth"], zt + 1)], (x1 - x0 + 1, 0, 0)))
    tabs = [B(x - 4, x + 4, y - 4 if y > 0 else y - 4, y + 4, z0, z0 + 3) for x, y in h["m3"]]
    s = fuse_all([s] + tabs)
    return cut_all(s, [cyl(x, y, z0 - 1, z0 + 4, M3_CLR) for x, y in h["m3"]])


def tipcheck_posts():
    """TCK-1 (MJF PA12): two posts on TW-1 carrying a through-beam sensor pair (bought, electrical BOM) facing along X"""
    t = TW["tipcheck"]
    out = []
    for sx in (-1, 1):
        x0, x1 = (t["post_x"][0], t["post_x"][1]) if sx > 0 else (-t["post_x"][1], -t["post_x"][0])
        out.append(cut_all(B(x0, x1, t["post_y"][0], t["post_y"][1], SEAT, t["post_z1"]),
                           [cyl(sx * x, y, SEAT - 1, SEAT + 8, M3_CLR) for x, y in t["m3"]]))
    return out


def tipcheck_sensors():
    t = TW["tipcheck"]
    x0 = t["post_x"][0] - 2.0      # sensor body 10 mm along X, its face 2 mm inboard of the post
    return [B(x0, t["post_x"][1], t["post_y"][0] + 2.0, t["post_y"][1] - 2.0, t["post_z1"], t["sensor_z1"]),
            B(-t["post_x"][1], -x0, t["post_y"][0] + 2.0, t["post_y"][1] - 2.0, t["post_z1"], t["sensor_z1"])]


def lid_pedestal():
    """LS-1 (MJF PA12): pedestal on the D3 plain insert, lifts the parked sealing lid 12 mm for the gripper"""
    w, d = LS["out"]
    return B(-w / 2, w / 2, -d / 2, d / 2, SEAT, SEAT + LS["h"]).cut(B(-w / 2 + 3, w / 2 - 3, -d / 2 + 3, d / 2 - 3, SEAT - 1, SEAT + LS["h"] - 2))


def strip_plate():
    st = TW["strip"]
    t = st["t"]
    s = B(st["x0"], st["x1"], -st["hy"], st["hy"], SEAT, SEAT + t)
    tools, feats = [], []
    for i in range(8):
        y = (i - 3.5) * P.WELL_PITCH_96
        w = st["slot_w"]
        tools += [B(st["x0"] - 1, st["x0"] + st["slot_len"] - w / 2, y - w / 2, y + w / 2, SEAT - 1, SEAT + t + 1),
                  cyl(st["x0"] + st["slot_len"] - w / 2, y, SEAT - 1, SEAT + t + 1, w)]
    for x, y in st["screws"]:
        tools.append(csk(x, y, SEAT + t, SEAT, M3_CLR, M3_CSK))
        feats.append(dict(x=x, y=y, d=M3_CLR, layer="cut", note=f"M3 clearance, countersink to {M3_CSK}"))
    return cut_all(s, tools), feats


# =====================================================================================
# DXF writer (from the bottom face of a flat part; layers by feature registry)
# =====================================================================================
def write_dxf(path: Path, shape: cq.Shape, z_bottom: float, feats, thickness, texts=(), notes=(), top=False):
    doc = ezdxf.new("R2010")
    for name, col in (("cut", 7), ("drill", 1), ("ream", 3), ("tap", 5), ("engrave", 2), ("note", 8)):
        doc.layers.add(name, color=col)
    msp = doc.modelspace()
    zf, sgn = (z_bottom + thickness, 1) if top else (z_bottom, -1)    # rev F: DK-2 is drawn from its TOP face (tabs are upper half)
    faces = [f for f in shape.Faces() if f.geomType() == "PLANE" and abs(f.Center().z - zf) < 1e-3
             and f.normalAt().z * sgn > 0.5]

    def lookup(cx, cy, d=None):
        for f in feats:
            if abs(f["x"] - cx) < 0.05 and abs(f["y"] - cy) < 0.05 and (d is None or f.get("slot") or abs(f["d"] - d) < 0.05):
                return f
        return None

    counts = {}
    for face in faces:
        for wire in [face.outerWire()] + face.innerWires():
            edges = wire.Edges()
            circ = all(e.geomType() == "CIRCLE" for e in edges)
            layer = "cut"
            if circ:
                c = edges[0].arcCenter()
                r = edges[0].radius()
                if all(abs(e.radius() - r) < 1e-4 and (e.arcCenter() - c).Length < 1e-4 for e in edges):
                    f = lookup(c.x, c.y, 2 * r)
                    if f is None and top:          # rev F: countersunk hole seen from the top: draw the hole, not the csk
                        f = next((g for g in feats if abs(g["x"] - c.x) < 0.05 and abs(g["y"] - c.y) < 0.05
                                  and abs(g.get("dk", -1) - 2 * r) < 0.05), None)
                        if f:
                            r = f["d"] / 2
                    layer = f["layer"] if f else ("cut" if 2 * r >= thickness else "drill")
                    msp.add_circle((c.x, c.y), r, dxfattribs={"layer": layer})
                    counts[layer] = counts.get(layer, 0) + 1
                    continue
            bb = wire.BoundingBox()
            f = lookup((bb.xmin + bb.xmax) / 2, (bb.ymin + bb.ymax) / 2)
            if f and f.get("slot"):
                layer = f["layer"]
            counts[layer] = counts.get(layer, 0) + 1
            for e in edges:
                gt = e.geomType()
                if gt == "LINE":
                    a, b = e.startPoint(), e.endPoint()
                    msp.add_line((a.x, a.y), (b.x, b.y), dxfattribs={"layer": layer})
                elif gt == "CIRCLE":
                    c, r = e.arcCenter(), e.radius()
                    a0 = math.degrees(math.atan2(e.startPoint().y - c.y, e.startPoint().x - c.x)) % 360
                    a1 = math.degrees(math.atan2(e.endPoint().y - c.y, e.endPoint().x - c.x)) % 360
                    am = math.degrees(math.atan2(e.positionAt(0.5).y - c.y, e.positionAt(0.5).x - c.x)) % 360
                    if not ((am - a0) % 360 < (a1 - a0) % 360):
                        a0, a1 = a1, a0
                    msp.add_arc((c.x, c.y), r, a0, a1, dxfattribs={"layer": layer})
                else:
                    pts = [e.positionAt(i / 24) for i in range(25)]
                    msp.add_lwpolyline([(p.x, p.y) for p in pts], dxfattribs={"layer": layer})
    sb = shape.BoundingBox()
    for f in feats:   # thread / countersink notes as small text on the note layer
        if not (sb.xmin <= f["x"] <= sb.xmax and sb.ymin <= f["y"] <= sb.ymax):
            continue
        msp.add_text(f["note"][:40], height=1.2, dxfattribs={"layer": "note"}).set_placement((f["x"] + 3, f["y"] + 1))
    for x, y, h, s in texts:
        msp.add_text(s, height=h, dxfattribs={"layer": "engrave"}).set_placement((x, y), align=ezdxf.enums.TextEntityAlignment.MIDDLE_CENTER)
    for i, n in enumerate(notes):
        msp.add_text(n, height=3.0, dxfattribs={"layer": "note"}).set_placement((-P.DECK_L / 2, -P.DECK_W / 2 - 10 - 5 * i))
    doc.saveas(str(path))
    return counts


# =====================================================================================
# calculations
# =====================================================================================
def plate_alpha(b_over_a):
    tab = [(1.0, 0.01160), (1.2, 0.01353), (1.4, 0.01484), (1.6, 0.01570), (1.8, 0.01620), (2.0, 0.01651)]
    for (r0, a0), (r1, a1) in zip(tab, tab[1:]):
        if r0 <= b_over_a <= r1:
            return a0 + (a1 - a0) * (b_over_a - r0) / (r1 - r0)
    return 0.01695


def beam_point_defl(F, x, x0, x1, I=I_2040_STRONG):
    L = x1 - x0
    a = min(max(x - x0, 0.0), L)
    b = L - a
    return F * a * a * b * b / (3 * E_AL * I * L)


def deck_deflection():
    nu = 0.33
    D_ins = E_AL * P.INSERT_T ** 3 / (12 * (1 - nu ** 2))
    a, b = P.WINDOW_W, P.WINDOW_L
    alpha = plate_alpha(b / a)
    I_web = WEB_X * P.DECK_T ** 3 / 12
    ys = sorted(BEAMS_X)
    res = {}

    def at(name, F):
        x, y = nest_xy(name)
        y_lo = max(v for v in ys if v < y and BEAMS_X[v][0] <= x <= BEAMS_X[v][1])
        y_hi = min(v for v in ys if v > y and BEAMS_X[v][0] <= x <= BEAMS_X[v][1])
        sh_hi = (y - y_lo) / (y_hi - y_lo)
        d_lo = beam_point_defl(F * (1 - sh_hi), x, *BEAMS_X[y_lo])
        d_hi = beam_point_defl(F * sh_hi, x, *BEAMS_X[y_hi])
        d_sup = (1 - sh_hi) * d_lo + sh_hi * d_hi
        d_ins = alpha * F * a * a / D_ins
        L_web = (y_hi - BEAM_W / 2) - (y_lo + BEAM_W / 2)
        d_web = (0.1 * F) * L_web ** 3 / (48 * E_AL * I_web)
        return dict(beams_below=[y_lo, y_hi], beam_mm=round(d_sup, 4), insert_plate_mm=round(d_ins, 4),
                    deck_web_mm=round(d_web, 4), total_mm=round(d_sup + d_ins + d_web, 4))

    for row in P.NEST_NAMES:
        for n in row:
            if n in MERGED_NESTS:
                continue
            res[n] = at(n, 200.0)
    worst = max(res, key=lambda k: res[k]["total_mm"])
    centre20 = beam_point_defl(20.0, 0.0, *BEAMS_X[0.0])
    nest20 = {k: round(v["total_mm"] * 20.0 / 200.0, 4) for k, v in res.items()}
    I_strip = WEB_X * P.DECK_T ** 3 / 12
    span_nobeam = 225.0 - 0.0 - BEAM_W
    no_post = 200.0 * span_nobeam ** 3 / (48 * E_AL * I_strip)
    return dict(
        assumptions=["deck supported on 2040 beams on edge (20 wide, 40 tall) under the 32 mm webs at y = -110, 0, 110 "
                     "and under the margins at y = +/-225, side beams at x = +/-230; X beams simply supported between side beams",
                     f"I(2040 strong axis) = {I_2040_STRONG:.0f} mm^4 assumed; E = {E_AL:.0f} MPa",
                     # integration: frame rev B values (beam end x = -47 on a 12 x 20 post at x -47..-35)
                     f"beam y=110 stops at x={BEAMS_X[110.0][0]:g} (clear of TC-1 under the bay) and its end sits on a 12 x 20 mm post at "
                     f"x {TC_POST[0]:g}..{TC_POST[1]:g} (frame rev B); the 14 mm web at x=-66 between the bay and C2/D2 is no longer over "
                     "the beam end (see cad/lh1/assembly report, deck_y110_support)",
                     "load at the nest centre on the insert; insert = simply supported plate over the window (Timoshenko alpha); "
                     "10 % of the load per 14 mm deck web as a point load on a simply supported strip between beam edges; "
                     "contributions summed (conservative)"],
        point_20N_at_deck_centre_mm=round(centre20, 4),
        point_20N_at_deck_centre_note="deck centre (0,0) lies on the y=0 web directly over a beam: plate term ~0, value is beam bending",
        point_20N_at_nest_centre_mm=nest20,
        tip_press_200N_mm=res, tip_press_200N_worst_nest=worst, tip_press_200N_worst_mm=res[worst]["total_mm"],
        without_beam_y110_200N_at_D2_mm=round(no_post, 2),
        without_beam_y110_note="if the y=110 beam is simply omitted the 14 mm webs span 205 mm: not acceptable; post or beam required")


def thermal_budget():
    T_room, T_set = 24.0, 4.0
    h_ext = 10.0                    # W/m2K natural convection + radiation (anodised, eps ~0.8)
    h_ud = 5.0
    k_pa, k_air, k_foam, k_al = 0.22, 0.026, 0.04, 160.0
    lx, ly, lt = TH["lip"]
    bx, by = TH["lower"]
    A_top = lx * ly * 1e-6
    A_lip_side = 2 * (lx + ly) * lt * 1e-6
    A_pads = (4 * 3 * 8 + 2 * 6 * 4) * 1e-6
    t_flange = TH["frame_flange"][2] + TH["pad_h"]
    A_low_side = 2 * (bx + by) * (TH["block_top"] - lt - TH["block_bottom"]) * 1e-6
    U_low = 1 / (0.001 / k_air + 0.002 / k_pa + 1 / h_ud)
    A_foam = (bx * by - 2 * TH["tec"][0] * TH["tec"][1]) * 1e-6
    G_foam = k_foam * A_foam / (TH["tec"][2] * 1e-3)
    G_screws = 4 * 0.012
    rho_v24, rho_v4 = 0.0218 * 0.5, 0.0064      # kg/m3 vapour at 24 C / 50 % RH and saturated at 4 C
    latent = (5.0 / (1.2 * 1005)) * (rho_v24 - rho_v4) * 2.45e6 * A_top

    def load(Th, T_ud, T_set=T_set):
        q = dict(top_convection_radiation=h_ext * A_top * (T_room - T_set),
                 lip_sides=h_ext * A_lip_side * (T_room - T_set),
                 pads_to_insert=k_pa * A_pads / (t_flange * 1e-3) * (T_room - T_set),
                 lower_sides_through_skirt=U_low * A_low_side * (T_ud - T_set),
                 foam_under_block=G_foam * (Th - T_set),
                 clamp_screws=G_screws * (Th - T_set),
                 condensation_latent=latent)
        return q, sum(q.values())

    tec = dict(Vmax=15.2, Imax=6.0, dTmax=66.0, Th_ref=300.0)      # TEC1-12706 class, datasheet values to confirm
    S = tec["Vmax"] / tec["Th_ref"]
    R = (tec["Th_ref"] - tec["dTmax"]) * tec["Vmax"] / (tec["Th_ref"] * tec["Imax"])
    K = (tec["Th_ref"] - tec["dTmax"]) * tec["Vmax"] * tec["Imax"] / (2 * tec["Th_ref"] * tec["dTmax"])
    R_int = 0.15            # K/W block->cold face (paste + spreading), per module
    def req_theta(n_tec, T_ud, Ts):
        best = None
        for Th10 in range(int((T_ud + 1) * 10), 700, 5):
            Th = Th10 / 10
            _, Q = load(Th, T_ud, Ts)
            Qreq = 1.25 * Q
            for i10 in range(2, int(tec["Imax"] * 10) + 1):
                I = i10 / 10
                Tc = Ts - R_int * Qreq / n_tec + 273.15
                ThK = Th + 273.15
                qc = S * I * Tc - 0.5 * I * I * R - K * (ThK - Tc)
                if n_tec * qc < Qreq:
                    continue
                pw = S * I * (ThK - Tc) + I * I * R
                volt = S * (ThK - Tc) + I * R
                Qh = n_tec * (qc + pw)
                theta = (Th - T_ud) / Qh
                if volt <= 12.0 and (best is None or theta > best["theta_KW"]):
                    best = dict(Th_C=Th, I_A=I, V_per_module=round(volt, 2), Qc_W=round(n_tec * qc, 2),
                                P_elec_W=round(n_tec * pw, 1), Qh_W=round(Qh, 1), theta_KW=round(theta, 3))
                break
        return best

    out = {f"{n}xTEC_underdeck_{int(T)}C": req_theta(n, T, T_set) for n in (1, 2) for T in (25.0, 30.0)}
    q30, Q30 = load(38.0, 30.0)
    # achievable sink estimate: 110 x 70 x 40 extrusion, 1.5 mm fins on 6 mm pitch, 60 x 60 x 25 fan
    n_fins = int(round(TH["sink"][1] / TH["fin_pitch"])) if getattr(P, "REV_K", False) else 11
    A_fins = n_fins * 2 * TH["sink"][0] * TH["sink"][3] * 1e-6 + TH["sink"][0] * TH["sink"][1] * 1e-6
    h_fin = TH_K_SINK["h_W_m2K"] if getattr(P, "REV_K", False) else 30.0
    theta_conv = 1 / (h_fin * A_fins * 0.85)
    flow = TH_K_SINK["flow_m3_s"] if getattr(P, "REV_K", False) else 0.0035   # m3/s through the fins
    theta_cal = 1 / (2 * flow * 1.2 * 1005)
    theta_sink_est = theta_conv + theta_cal + 0.05
    lowest = {}
    for n in (1, 2):
        for T in (25.0, 30.0):
            for Ts10 in range(20, 200, 5):
                b = req_theta(n, T, Ts10 / 10)
                if b and b["theta_KW"] >= theta_sink_est:
                    lowest[f"{n}xTEC_underdeck_{int(T)}C"] = Ts10 / 10
                    break
    return dict(
        assumptions=["room 24 C, block 4 C, under-deck air 25 or 30 C", "h = 10 W/m2K on exposed cold Al (convection + radiation)",
                     "2 x TEC1-12706 class (Vmax 15.2 V, Imax 6 A, dTmax 66 K), driven at <= 12 V each (2 in series on 24 V)",
                     "load x 1.25 margin; 0.15 K/W block-to-cold-face per module; foam EPDM k = 0.04",
                     "tube block open (no lid); tubes and reagents add only transient load"],
        steady_load_W_at_4C=round(Q30, 2), load_breakdown_W=({k: round(v, 2) for k, v in q30.items()}),
        required_sink_theta=out,
        estimated_sink_theta_KW=round(theta_sink_est, 3),
        lowest_block_temp_C_with_estimated_sink=lowest,
        verdict="MARGINAL: a fan-cooled sink inside the 118 x 78 window and 110 mm clearance (~0.6 K/W) holds 4 C only with "
                "under-deck air <= ~25 C; with 30 C under-deck air expect ~6-8 C. Remedies in order: duct fresh room air to the "
                "fan intake, lid on the tube block, nylon clamp screws, 1 TEC at ~3 A (better COP than 2) or heat pipe to a "
                "larger sink outside the window (needs a params change).",
        estimated_sink_note="110 x 70 x 40 fin sink + 60 x 60 x 25 fan within the window: ~0.4-0.6 K/W by typical data; "
                            "this estimate uses h=30 W/m2K, fin efficiency 0.85, 3.5 L/s",
        heating_40C_load_W=round((h_ext * (A_top + A_lip_side) + k_pa * A_pads / (t_flange * 1e-3)) * 16 + U_low * A_low_side * 10, 2),
        heating_note="40 C is easy: TEC reversed, heating COP > 1; limit block to 50 C by firmware and the 60-72 C TCO")


def cold_block_rev_j(theta_override=None):
    """rev J (electrical owner): each cold block (C3, F3) = 2 x TEC1-12706-class modules in SERIES on one TEC-1166 channel limited
    to 3.5 A / 18 V (9 V per module).  Steady state and pull-down with a loaded 12-column strip carrier, at 25 and 30 C ambient.
    Same module, load and sink model as thermal_budget (all CALCULATED from ASSUMED inputs; nothing measured)."""
    h_ext, h_ud = 10.0, 5.0
    k_pa, k_air, k_foam = 0.22, 0.026, 0.04
    lx, ly, lt = TH["lip"]
    bx_, by_ = TH["lower"]
    A_top = lx * ly * 1e-6
    A_lip_side = 2 * (lx + ly) * lt * 1e-6
    A_pads = (4 * 3 * 8 + 2 * 6 * 4) * 1e-6
    t_flange = TH["frame_flange"][2] + TH["pad_h"]
    A_low_side = 2 * (bx_ + by_) * (TH["block_top"] - lt - TH["block_bottom"]) * 1e-6
    U_low = 1 / (0.001 / k_air + 0.002 / k_pa + 1 / h_ud)
    A_foam = (bx_ * by_ - 2 * TH["tec"][0] * TH["tec"][1]) * 1e-6
    G_foam = k_foam * A_foam / (TH["tec"][2] * 1e-3)
    G_screws = 4 * 0.012
    strip_top_factor = 1.6          # 96 strip tubes standing proud of the carrier: exposed area x 1.6 (ASSUMED)

    def load(Th, Tamb, Tb):
        rho_sat = {25.0: 0.0230, 30.0: 0.0304}[Tamb]
        latent = (5.0 / (1.2 * 1005)) * (0.5 * rho_sat - 0.0064) * 2.45e6 * A_top * strip_top_factor
        return (h_ext * A_top * strip_top_factor * (Tamb - Tb) + h_ext * A_lip_side * (Tamb - Tb)
                + k_pa * A_pads / (t_flange * 1e-3) * (Tamb - Tb) + U_low * A_low_side * (Tamb - Tb)
                + G_foam * (Th - Tb) + G_screws * (Th - Tb) + max(latent, 0.0))
    tec = dict(Vmax=15.2, Imax=6.0, dTmax=66.0, Th_ref=300.0)
    S = tec["Vmax"] / tec["Th_ref"]
    R = (tec["Th_ref"] - tec["dTmax"]) * tec["Vmax"] / (tec["Th_ref"] * tec["Imax"])
    K = (tec["Th_ref"] - tec["dTmax"]) * tec["Vmax"] * tec["Imax"] / (2 * tec["Th_ref"] * tec["dTmax"])
    R_int, n = 0.15, 2
    theta = theta_override or thermal_budget()["estimated_sink_theta_KW"]   # 110 x 70 x 40 sink + 60 mm fan (estimate, +/-40 %)
    I_lim, V_mod_lim = 3.5, 18.0 / 2

    def op(Tb, Tamb, I):
        """pumped heat Qc (both modules), voltage per module, Th, at block temperature Tb and current I (sink loop solved)"""
        Th = Tamb + 10.0
        for _ in range(60):
            Tc = Tb + 273.15 - R_int * max(load(Th, Tamb, Tb), 0.0) / n
            ThK = Th + 273.15
            qc = S * I * Tc - 0.5 * I * I * R - K * (ThK - Tc)
            pw = S * I * (ThK - Tc) + I * I * R
            Th_new = Tamb + theta * n * (qc + pw)
            if abs(Th_new - Th) < 0.01:
                break
            Th = 0.5 * Th + 0.5 * Th_new
        v = S * (ThK - Tc) + I * R
        return n * qc, v, Th, n * pw

    def I_max_at(Tb, Tamb):
        I = I_lim
        while I > 0.1 and op(Tb, Tamb, I)[1] > V_mod_lim:
            I -= 0.05
        return round(I, 2)
    # block thermal mass: machined block + loaded strip carrier
    m_block = thermal_block().Volume() * RHO["al6061"]
    C = m_block * 900.0 + 0.060 * 900.0 + 96 * 0.12e-3 * 1900.0 + 96 * 50e-9 * 1000.0 * 4186.0   # J/K (carrier 60 g Al, PP, 50 uL water: ASSUMED)
    out = {}
    for Tamb in (25.0, 30.0):
        Ia = I_max_at(4.0, Tamb)
        qc, v, Th, pel = op(4.0, Tamb, Ia)
        Lq = load(Th, Tamb, 4.0)
        # lowest steady block temperature at the limits
        Tmin = None
        for Tb10 in range(-100, 250):
            Tb = Tb10 / 10
            Ii = I_max_at(Tb, Tamb)
            q_, _, Th_, _ = op(Tb, Tamb, Ii)
            if q_ >= load(Th_, Tamb, Tb):
                Tmin = Tb
                break
        # current needed to hold 4 C (no voltage limit; reported)
        I_need = None
        for i in range(5, 61):
            q_, v_, Th_, _ = op(4.0, Tamb, i / 10)
            if q_ >= load(Th_, Tamb, 4.0):
                I_need = dict(I_A=i / 10, V_per_module=round(v_, 2), V_channel=round(2 * v_, 2))
                break
        # pull-down from ambient to 4 C at the limit (explicit Euler, 1 s)
        T, t = Tamb, 0
        while T > 4.0 and t < 7200:
            Ii = I_max_at(T, Tamb)
            q_, _, Th_, _ = op(T, Tamb, Ii)
            T -= (q_ - load(Th_, Tamb, T)) / C
            t += 1
        out[f"ambient_{int(Tamb)}C"] = {
            "current_at_4C_A (voltage-limited to 9 V per module)": Ia, "V_per_module": round(v, 2), "channel_V": round(2 * v, 2),
            "pumped_at_4C_W": round(qc, 2), "load_at_4C_W": round(Lq, 2), "margin_pumped_over_load": round(qc / Lq, 2),
            "hot_side_C": round(Th, 1), "electrical_W": round(pel, 1),
            "lowest_steady_block_C_at_3.5A_18V": Tmin, "current_needed_to_hold_4C": I_need,
            "pull_down_to_4C_min": round(t / 60.0, 1) if t < 7200 else "not reached in 2 h",
            "verdict": ("holds 4 C" if Tmin is not None and Tmin <= 4.0 else "does NOT reach 4 C")}
    if theta_override is not None:
        return out
    sweep = {f"theta_{t:g}_KW": {k: {"lowest_C": v["lowest_steady_block_C_at_3.5A_18V"], "pull_down_min": v["pull_down_to_4C_min"],
                                     "margin": v["margin_pumped_over_load"]} for k, v in cold_block_rev_j(t).items()}
             for t in (0.45, 0.35, 0.30)}
    return {"limits": "TEC-1166 channel 3.5 A / 18 V (two modules in series: 9 V each)", "sink_theta_KW_estimate": theta,
            "answer": (f"YES (rev K sink, theta {theta:.3f} K/W CALCULATED): 4 C held at 25 C ambient (lowest "
                       f"{out['ambient_25C']['lowest_steady_block_C_at_3.5A_18V']} C, pull-down {out['ambient_25C']['pull_down_to_4C_min']} min) "
                       f"and at 30 C (lowest {out['ambient_30C']['lowest_steady_block_C_at_3.5A_18V']} C, pull-down "
                       f"{out['ambient_30C']['pull_down_to_4C_min']} min, margin {out['ambient_30C']['margin_pumped_over_load']}); with the "
                       "sink 50 % worse than calculated (0.30 K/W) still 0.4 C at 30 C, 13 min.  MEASURE on the bench rig: sink theta "
                       "under a 60 W resistor load at the fan's real flow; block temperature, current and voltage at 4 C, 25 and 30 C ambient"
                       if getattr(P, "REV_K", False) and (out["ambient_30C"]["lowest_steady_block_C_at_3.5A_18V"] or 99) <= 4.0 else
                       "NO with the present sink: the block bottoms out at about 9 C (25 C ambient) and 14 C (30 C); more current does "
                       "not help (the hot side runs away through the 0.6 K/W sink).  The limiter is the sink, not the 3.5 A: with a sink "
                       "of <= 0.30 K/W the same 3.5 A / 18 V channel reaches 4 C at 25 C in about 8 min and at 30 C in about 13 min "
                       "(margins 2.2 / 1.5).  CHANGE: cold-module sink to <= 0.30 K/W (taller fins into the 28 mm spare under-deck "
                       "height + a higher-pressure 60 mm fan, or a heat-pipe sink), measure on the rig before ordering."),
            "sink_sweep_at_3.5A_18V": sweep,
            "thermal_mass_J_per_K": round(C, 1), "block_mass_kg": round(m_block, 3), **out,
            "assumptions": ["TEC1-12706 class model (Vmax 15.2 V, Imax 6 A, dTmax 66 K), 0.15 K/W block-to-cold-face per module",
                            "sink 0.6 K/W with the 60 mm fan (estimate +/-40 %), sink air = ambient (under-deck air = room)",
                            "loaded 12-column strip carrier: exposed top area x 1.6, 96 x 50 uL, carrier 60 g aluminium; 50 % RH",
                            "no 1.25 load margin here (thermal_budget keeps it): margin is reported as pumped / load"]}


def magnet_force(carrier_shape, mags):
    m_car = carrier_shape.Volume() * RHO["pa12"]
    m_mag = mags.Volume() * RHO["ndfeb"]
    m_bush = 2 * 0.008
    W = (m_car + m_mag + m_bush) * 9.81
    stray = 5.0
    T_run, lead, eta = 0.20, 2.0, 0.30
    thrust = 2 * math.pi * eta * T_run / (lead * 1e-3)
    return dict(carrier_kg=round(m_car, 3), magnets_kg=round(m_mag, 3), moving_mass_kg=round(m_car + m_mag + m_bush, 3),
                weight_N=round(W, 2), stray_magnetic_pull_N_assumed=stray,
                stray_note="no ferrous part within 30 mm of the magnets except the two guide rods, which sit symmetrically "
                           "between rows (net ~0); 5 N allowance covers asymmetry, steel screws and neighbour modules",
                actuator="NEMA17 with integrated T8 lead screw, lead 2 mm, running torque taken as 0.20 N m, efficiency 0.30",
                available_thrust_N=round(thrust, 1), required_N=round(W + stray, 2),
                force_margin=round(thrust / (W + stray), 1),
                self_locking="lead angle 4.5 deg < friction angle ~8.5 deg: holds position unpowered",
                travel_time_s_at_10mm_s=round(MAG["travel"] / 10.0, 1))


# =====================================================================================
# main
# =====================================================================================
def main():
    OUT.mkdir(parents=True, exist_ok=True)
    for old in OUT.glob("*"):
        old.unlink()
    rep = {"status": "fabrication candidate for a first prototype - NOT validated; nominal labware geometry to be measured",
           "revision": "A (12 Nov 2026), after coordinator changes: tip waste as nest insert A1, hand-off = plain insert D3, "
                       "C1+D1 merged thermocycler bay",
           "parts": [], "bought_parts": [], "checks": {}, "params_change_requests": []}
    parts = {}

    def add_part(name, shape, process, material, thickness, qty, rho, stl=False, dxf=None, note=None):
        assert shape.isValid(), f"invalid solid: {name}"
        envelope = env(shape)
        shape.exportStep(str(OUT / f"{name}.step"))
        if stl:
            shape.exportStl(str(OUT / f"{name}.stl"), tolerance=0.05, angularTolerance=0.2)
        parts[name] = shape
        e = dict(name=name, process=process, material=material, thickness=thickness, qty=qty,
                 envelope_mm=envelope, mass_kg=round(shape.Volume() * rho, 3), valid_solid=bool(shape.isValid()))
        if dxf:
            e["dxf"] = dxf
        if note:
            e["note"] = note
        rep["parts"].append(e)
        return shape

    # ---------------- deck plate (one piece) + two-piece option
    deck, windows, dfeats, lost = deck_plate()
    subp, _, _, _ = deck_plate(part="sub")   # integration rev D
    texts = []
    for row in P.NEST_NAMES:
        for n in row:
            x, y = nest_xy(n)
            if n in MERGED_NESTS:
                continue
            texts.append((x, y - WIN_HY - 8.0, 5.0, n))
    for pr in MERGED_WINDOWS:
        x0, x1, y0, y1 = merged_window_rect(pr)
        texts.append(((x0 + x1) / 2, y0 - 8.0, 4.0, "+".join(pr) + " TC BAY"))
    for i, x in enumerate(P.NEST_X):
        texts.append((x, DECK_Y0 + 12.0, 6.0, str(i + 1)))
    add_part("DK-1_deck_plate", deck, "sheet_laser", "Al 5052-H32", P.DECK_T, 0 if hasattr(P, "FRONT_ROW") else 1, RHO["al5052"], dxf="DK-1_deck_plate.dxf",
             note="rev F (M7): CNC 3 mm rebate from the TOP face on the two DK-2 tabs (x -172..-148 and -112..-88, y 0..6), "
                  "press 2 x D4x10 h6 pins; exceeds the 600 x 400 rule only if the supplier limit is smaller than 500 x 500: verify; else use DK-1F/DK-1B")
    write_dxf(OUT / "DK-1_deck_plate.dxf", deck, -P.DECK_T, dfeats, P.DECK_T, texts,
              notes=["DK-1 deck plate 5052-H32 6 mm. Layers: cut=laser, drill=drill after cut, ream=dowel D4 H7 (drill 3.8, ream),",
                     "tap=M4 tap drill 3.3 then tap through, frame holes M5 csk 90 deg on TOP face, engrave=0.2 mm mark",
                     "flatness 0.3 mm over 500 mm; deburr windows; clear anodise optional"])
    add_part("DK-2_tc_bay_subplate", subp, "sheet_laser", "Al 5052-H32", P.DECK_T, 1, RHO["al5052"], dxf="DK-2_tc_bay_subplate.dxf",
             note="integration rev F (M7): removable sub-plate x -250..-48, y 0..217 + two half-lap tabs (CNC: 3 mm rebate "
                  "from the BOTTOM face on the tabs, ream D4 H7 at (-160,0) and slot at (-100,0)); located on DK-1 by the two "
                  "pins only; held by 2 csk screws at (-230,55)/(-230,165) (TC-1 top plate access holes); carries TC-1 only. "
                  "Lifts out with TC-1 after removing inserts C2/D2 (their two -X screws clamp into DK-2; they LOCATE on DK-1 dowels)")
    write_dxf(OUT / "DK-2_tc_bay_subplate.dxf", subp, -P.DECK_T, dfeats, P.DECK_T, [], top=True,
              notes=["DK-2 seen from the TOP face; tabs x -172..-148 and -112..-88, y -6..0 are 3 mm thick (rebate from the bottom)"])
    front, back, sfeats, dts = deck_halves()
    add_part("DK-1F_deck_front_half", front, "sheet_laser", "Al 5052-H32", P.DECK_T, 1 if hasattr(P, "FRONT_ROW") else 0, RHO["al5052"], dxf="DK-1F_deck_front_half.dxf",
             note="rev H: THE deck (one piece would be 500 x 610 > the 600 x 400 sheet rule): front half 500 x 360 (rows F, A and "
                  "the front of B), joint over the y = 0 beam by laser-cut dovetails, both halves countersunk to every beam; DK-1 is "
                  "kept only as the one-piece drawing (qty 0)")
    add_part("DK-1B_deck_back_half", back, "sheet_laser", "Al 5052-H32", P.DECK_T, 1 if hasattr(P, "FRONT_ROW") else 0, RHO["al5052"], dxf="DK-1B_deck_back_half.dxf",
             note="rev H: back half 500 x 250 less the DK-2 TC bay sub-plate")
    write_dxf(OUT / "DK-1F_deck_front_half.dxf", front, -P.DECK_T, sfeats, P.DECK_T, [t for t in texts if t[1] < 0])
    write_dxf(OUT / "DK-1B_deck_back_half.dxf", back, -P.DECK_T, sfeats, P.DECK_T, [t for t in texts if t[1] > 0])

    # ---------------- plain insert
    locs = locator_set()
    plain_plate, pfe = insert_plate(tap_m3=locator_tap_points())
    n_plain = sum(1 for v in DEFAULT_LAYOUT.values() if v[0] in ("plain", "plain_handoff"))
    add_part("IN-P_plain_insert_plate", plain_plate, "sheet_laser", "Al 5052-H32", P.INSERT_T, n_plain, RHO["al5052"],
             dxf="IN-P_plain_insert_plate.dxf")
    write_dxf(OUT / "IN-P_plain_insert_plate.dxf", plain_plate, 0.0, pfe, P.INSERT_T, [(0, -30, 4, "IN-P")])
    add_part("LOC-1_corner_locator", locs[0], "cnc", "Al 6061-T6, anodise", None, 4 * (n_plain + 1), RHO["al6061"],
             note="integration rev F (M18): machined, locating faces +/-0.05 mm (was MJF PA12 +/-0.3 against 0.2 mm play); "
                  "2 handed pairs per insert (LOC-1 and mirror); pocket 0.15..0.25 mm per side over an SBS plate")
    plain_asm = cq.Assembly(name="IN-P_plain_insert")
    plain_asm.add(plain_plate, name="plate", color=cq.Color(0.75, 0.75, 0.78))
    for i, l in enumerate(locs):
        plain_asm.add(l, name=f"locator_{i}", color=cq.Color(0.2, 0.2, 0.2))
    plain_asm.save(str(OUT / "IN-P_plain_insert_assembly.step"))

    # ---------------- magnet module
    g = magnet_geometry()
    mag_plate, mfe = insert_plate(opening=MAG["opening"], tap_m3=locator_tap_points(), csk_m3=MAG["housing_screws"])
    add_part("MM-1_magnet_insert_plate", mag_plate, "sheet_laser", "Al 5052-H32", P.INSERT_T, 1, RHO["al5052"],
             dxf="MM-1_magnet_insert_plate.dxf")
    write_dxf(OUT / "MM-1_magnet_insert_plate.dxf", mag_plate, 0.0, mfe, P.INSERT_T, [(0, -44, 3, "MM-1")])
    housing, z_ft, z_fb = magnet_housing(g)
    add_part("MM-2_magnet_housing", housing, "print_mjf_pa12", "PA12 (MJF)", None, 0, RHO["pa12"], stl=True,   # rev F: alternative only
             note="6 x M3 heat-set inserts in the top bosses; rods pressed into the floor")
    carrier_local = magnet_carrier()
    # integration rev E (DFM): qty 2 = one carrier + one press-fit test coupon
    add_part("MM-3_magnet_carrier", carrier_local, "print_mjf_pa12", "PA12 (MJF)", None, 0, RHO["pa12"], stl=True,   # rev F: alternative only
             note="42 magnets press-fit (no adhesive); 2 x LM6UU pressed from below; T8 flange nut on 4 x M3 heat-sets")
    carrier_home = mv(carrier_local, z=g["carrier_top_home"])
    mags_home = magnet_set(g["carrier_top_home"])
    # integration rev F: MB-1 passive magnetic block (default); MM-2/MM-3 + lift hardware become the documented alternative
    mb1 = mb1_body()
    mb1_mags = magnet_set(mb1_geometry()["body_top"])
    add_part("MB-1_passive_magnet_block", mb1, "print_mjf_pa12", "PA12 (MJF)", None, 2, RHO["pa12"], stl=True,
             note="integration rev F: 42 N52 2x10x20 in 6 rows, alternating polarity, press-fit; 6 x M3 heat-set inserts in the "
                  "ears, hung from MM-1 by 6 x M3x8 csk; qty 2 = one + one press-fit test coupon")
    mb, screw_top = magnet_bought(g, z_ft, z_fb, g["carrier_top_home"])
    pcr_full_local = pcr_plate(False)
    pcr_semi_local = pcr_plate(True)
    mag_asm = cq.Assembly(name="MM_magnet_module")
    mag_asm.add(mag_plate, name="insert_plate", color=cq.Color(0.75, 0.75, 0.78))
    for i, l in enumerate(locs):
        mag_asm.add(l, name=f"locator_{i}", color=cq.Color(0.2, 0.2, 0.2))
    mag_asm.add(housing, name="housing", color=cq.Color(0.3, 0.3, 0.32))
    mag_asm.add(carrier_home, name="carrier_home", color=cq.Color(0.9, 0.6, 0.1))
    mag_asm.add(mags_home, name="magnets_home", color=cq.Color(0.6, 0.6, 0.65))
    for k, s in mb.items():
        mag_asm.add(s, name=k, color=cq.Color(0.45, 0.45, 0.5))
    mag_asm.add(mv(pcr_full_local, z=SEAT), name="pcr_plate_full_skirt", color=cq.Color(0.9, 0.9, 0.95, 0.5))
    mag_asm.save(str(OUT / "MM_magnet_module_assembly.step"))

    # ---------------- thermal module
    th_plate, tfe = insert_plate(opening=TH["opening"], tap_m3=TH["frame_screws"], drills=[(TH["drain"][0], TH["drain"][1], 4.0)])
    add_part("TM-1_thermal_insert_plate", th_plate, "sheet_laser", "Al 5052-H32", P.INSERT_T, N_THERMAL, RHO["al5052"],
             dxf="TM-1_thermal_insert_plate.dxf")
    write_dxf(OUT / "TM-1_thermal_insert_plate.dxf", th_plate, 0.0, tfe, P.INSERT_T, [(0, -45, 3, "TM-1")])
    frame = thermal_frame()
    add_part("TM-2_insulating_frame", frame, "print_mjf_pa12", "PA12 (MJF)", None, N_THERMAL, RHO["pa12"], stl=True,
             note="condensate gutter around the lip drains via D3 hole + insert D4 hole to a silicone tube; "
                  "inner-side condensate drips from the skirt onto the warm sink and evaporates")
    block = thermal_block()
    add_part("TM-3_cold_block_24tube", block, "cnc", "Al 6061-T6, hard anodised", None, N_THERMAL, RHO["al6061"],
             note="24 x 1.5 mL tubes on 18 mm pitch (same grid as TC24-A); NTC pocket D3.2 from +X, TCO pocket D5.2 from -X; "
                  "4 x M3 clamp holes from below; mill TEC faces flat 0.05")
    stack, z_th_bottom = thermal_lower_stack()
    if getattr(P, "REV_K", False):
        add_part("TM-4_heat_sink_machined", stack["heat_sink"], "cnc", "BOUGHT bonded/skived-fin Al sink 110 x 76 x 54 (fins 48 x 1.0 on 3.0 pitch), drilled/tapped",
                 None, N_THERMAL, RHO["al6061"], note="rev K: bought sink class, theta <= 0.22 K/W with a 60 x 60 x 38 high-static-pressure fan "
                 ">= 17 CFM (CALCULATED from ASSUMED h); drill 4 x D3.4, tap 2 x M3 in base ends; MEASURE theta on the bench rig")
    else:
        add_part("TM-4_heat_sink_machined", stack["heat_sink"], "cnc", "Al 6063 extrusion (bought 110 x 70 x 40), drilled/tapped",
                 None, N_THERMAL, RHO["al6061"], note="IN-HOUSE drill/tap (integration rev E, DFM): buy a 110 mm length of a 70 x 40 fin extrusion; drill 4 x D3.4, tap 2 x M3 in base ends")
    add_part("TM-5_fan_shroud", stack["fan_shroud"], "print_mjf_pa12", "PA12 (MJF)", None, N_THERMAL, RHO["pa12"], stl=True)
    th_asm = cq.Assembly(name="TM_thermal_module")
    th_asm.add(th_plate, name="insert_plate", color=cq.Color(0.75, 0.75, 0.78))
    th_asm.add(frame, name="insulating_frame", color=cq.Color(0.95, 0.95, 0.9))
    th_asm.add(block, name="cold_block", color=cq.Color(0.6, 0.75, 0.95))
    for k, s in stack.items():
        th_asm.add(s, name=k, color=cq.Color(0.4, 0.4, 0.45))
    th_asm.save(str(OUT / "TM_thermal_module_assembly.step"))

    # ---------------- tip waste insert
    tw_plate, wfe = insert_plate(opening=TW["opening"], tap_m3=TW["comb"]["feet_m3"] + [(sx * x, y) for x, y in TW["tipcheck"]["m3"] for sx in (-1, 1)]
                                 + TW["hopper"]["m3"],   # rev G1: TW-5 hopper taps
                                 clr_m3=TW["screws"])   # rev E: + tip-check post taps   # integration rev D
    add_part("TW-1_tip_waste_insert_plate", tw_plate, "sheet_laser", "Al 5052-H32", P.INSERT_T, 1, RHO["al5052"],
             dxf="TW-1_tip_waste_insert_plate.dxf")
    write_dxf(OUT / "TW-1_tip_waste_insert_plate.dxf", tw_plate, 0.0, wfe, P.INSERT_T, [(-30, -44, 3, "TW-1 TIP WASTE")])
    funnel = tip_funnel()
    add_part("TW-2_tip_funnel", funnel, "print_mjf_pa12", "PA12 (MJF)", None, 1, RHO["pa12"], stl=True,
             note="4 x M3 heat-set inserts; bin rails and back stop integral")
    tbin = tip_bin()
    add_part("TW-3_tip_bin", tbin, "print_mjf_pa12", "PA12 (MJF)", None, 2, RHO["pa12"], stl=True,
             note="slides out to the front (-Y) on the funnel rails, no tools; second bin as swap")
    sp = strip_comb()                                   # integration rev D: TW-4 plate -> TW-4D raised comb
    add_part("TW-4D_strip_comb", sp, "cnc", "Al 6061-T6, hard anodise", None, 1, RHO["al6061"],
             note=f"integration rev F (M3): machined comb, plate {TW['comb']['t']} mm at z {TW['comb']['z_under']}, rib "
                  f"{TW['comb']['rib'][2]} deep; slots {TW['comb']['slot8_w']} (8ch, from -X) and {TW['comb']['slot1_w']} (single, "
                  f"from +X) with {TW['comb']['leadin']} mm lead-ins; " + TW["comb"]["requires"])
    (OUT / "TW-4D_strip_comb.dxf").unlink(missing_ok=True)      # rev F: no flat pattern (machined part, STEP is the drawing)
    tw_asm = cq.Assembly(name="TW_tip_waste_insert")
    tw_asm.add(tw_plate, name="insert_plate", color=cq.Color(0.75, 0.75, 0.78))
    tw_asm.add(funnel, name="funnel", color=cq.Color(0.3, 0.3, 0.32))
    tw_asm.add(tbin, name="bin", color=cq.Color(0.85, 0.85, 0.2))
    tw_asm.add(sp, name="strip_comb", color=cq.Color(0.6, 0.6, 0.65))
    hop = tip_hopper()                                   # rev G1
    add_part("TW-5_eject_hopper", hop, "print_mjf_pa12", "PA12 (MJF)", None, 1, RHO["pa12"], stl=True,
             note="rev G1: method-B eject hopper on TW-1 (2 x M3), mouth 23.5 x 89 mm at z 35")
    tw_asm.add(hop, name="eject_hopper", color=cq.Color(0.3, 0.3, 0.32))
    tck = tipcheck_posts()                               # integration rev E (review A1)
    add_part("TCK-1_tipcheck_post", tck[1], "print_mjf_pa12", "PA12 (MJF)", None, 2, RHO["pa12"], stl=True,
             note=f"through-beam tip-check gate on TW-1: beam along X at nest y {TW['tipcheck']['beam_y']:g}, z {TW['tipcheck']['beam_z']:g}; post x {TW['tipcheck']['post_x'][0]:g}..{TW['tipcheck']['post_x'][1]:g}, top z {TW['tipcheck']['post_z1']:g} (sensor pair in the electrical BOM)")
    lsp = lid_pedestal()
    add_part("LS-1_lid_pedestal", lsp, "print_mjf_pa12", "PA12 (MJF)", None, 1, RHO["pa12"], stl=True,
             note="sealing-lid park on the D3 plain insert (review A3); one lid per plate per run")
    tw_asm.save(str(OUT / "TW_tip_waste_assembly.step"))

    # ---------------- existing tube carrier TC24-A (reuse, no new part)
    spec = importlib.util.spec_from_file_location("cad_build", str(LH1.parent / "build.py"))
    cad_build = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(cad_build)
    tc24 = cad_build.tube_carrier().val()

    # ---------------- deck assembly (default layout)
    def labware(kind):
        if kind == "tiprack":
            return B(-SBS_HX, SBS_HX, -SBS_HY, SBS_HY, SEAT, SEAT + LABWARE_H["tiprack"])
        if kind == "reservoir":
            return B(-SBS_HX, SBS_HX, -SBS_HY, SBS_HY, SEAT, SEAT + LABWARE_H["reservoir"])
        if kind and kind.startswith("pcr_plate"):
            return mv(pcr_full_local, z=SEAT)
        return None

    glob = []          # (name, shape, kind) in global coordinates

    def place(name, s, x, y, kind="part"):
        glob.append((name, mv(s, x, y, 0), kind))

    place("deck_plate", deck, 0, 0, "deck")
    place("deck_subplate_tc_bay", subp, 0, 0, "deck")      # integration rev D
    for n, (typ, lw) in DEFAULT_LAYOUT.items():
        x, y = nest_xy(n)
        if typ == "plain_handoff":                       # integration rev E: sealing-lid park pedestal on D3
            place(f"{n}_lid_pedestal", lsp, x, y)
        if typ in ("plain", "plain_handoff"):
            place(f"{n}_insert", plain_plate, x, y)
            for i, l in enumerate(locs):
                place(f"{n}_loc{i}", l, x, y)
        elif typ == "magnet_block":                   # integration rev F: MB-1 passive block
            place(f"{n}_insert", mag_plate, x, y)
            for i, l in enumerate(locs):
                place(f"{n}_loc{i}", l, x, y)
            place(f"{n}_mb1", mb1, x, y)
            place(f"{n}_magnets", mb1_mags, x, y)
        elif typ == "magnet":
            place(f"{n}_insert", mag_plate, x, y)
            for i, l in enumerate(locs):
                place(f"{n}_loc{i}", l, x, y)
            place(f"{n}_housing", housing, x, y)
            place(f"{n}_carrier", carrier_home, x, y)
            place(f"{n}_magnets", mags_home, x, y)
            for k, s in mb.items():
                place(f"{n}_{k}", s, x, y)
        elif typ == "thermal":
            place(f"{n}_insert", th_plate, x, y)
            place(f"{n}_frame", frame, x, y)
            place(f"{n}_block", block, x, y)
            for k, s in stack.items():
                place(f"{n}_{k}", s, x, y)
        elif typ == "tip_waste":
            place(f"{n}_insert", tw_plate, x, y)
            place(f"{n}_funnel", funnel, x, y)
            place(f"{n}_bin", tbin, x, y)
            place(f"{n}_strip_comb", sp, x, y)
            place(f"{n}_eject_hopper", hop, x, y)          # rev G1
            for k_, s_ in enumerate(tck + tipcheck_sensors()):
                place(f"{n}_tipcheck_{k_}", s_, x, y)
        s = labware(lw)
        if s is not None:
            place(f"{n}_labware_{lw}", s, x, y, "labware")
    # frame reference envelopes (not designed here)
    refs = []
    for yb, (xa, xb) in BEAMS_X.items():
        refs.append((f"REF_beam_y{int(yb)}", B(xa if xa > -230 else -240.0, 240.0, yb - BEAM_W / 2, yb + BEAM_W / 2, -P.DECK_T - BEAM_H, -P.DECK_T)))
    for xb in BEAMS_Y_X:
        refs.append((f"REF_beam_x{int(xb)}", B(xb - BEAM_W / 2, xb + BEAM_W / 2, -240.0, 240.0, -P.DECK_T - BEAM_H, -P.DECK_T)))
    x0, x1, y0, y1 = TC_POST
    refs.append(("REF_post_tc_bay", B(x0, x1, y0, y1, P.BASE_Z, -P.DECK_T - BEAM_H)))
    asm = cq.Assembly(name="LH1_deck_assembly")
    colors = {"deck": cq.Color(0.8, 0.8, 0.82), "labware": cq.Color(0.9, 0.9, 0.95, 0.5), "part": cq.Color(0.35, 0.35, 0.4)}
    for name, s, kind in glob:
        asm.add(s, name=name, color=colors[kind])
    for name, s in refs:
        asm.add(s, name=name, color=cq.Color(0.6, 0.3, 0.3, 0.3))
    if TEC_BOARD_RES_ACTIVE:
        refs.append(("RES_tec_board_C3", B(*TEC_BOARD_RES)))   # integration rev D: reserved, see TEC_BOARD_RES
    for k, v in STAGE_RES.items():                         # integration rev E: later-stage reservations
        refs.append((k, B(*v)))
    if TEC_BOARD_RES_ACTIVE:
        asm.add(B(*TEC_BOARD_RES), name="RES_tec_board_C3", color=cq.Color(0.2, 0.6, 0.9, 0.3))
    asm.save(str(OUT / "deck_assembly.step"))

    # =================================================================== checks
    c = rep["checks"]
    # --- window fit and under-deck envelope per module
    def below(s):
        bb = s.BoundingBox()
        return bb

    fit = {}
    mod_defs = {
        "magnet_module": [housing, carrier_home, mags_home, mv(carrier_local, z=g["carrier_top_engaged_full"])] + list(mb.values()),
        "thermal_module": [frame, block] + list(stack.values()),
        "tip_waste_insert": [funnel],
    }
    for mname, shapes in mod_defs.items():
        sub = [s.intersect(B(-300, 300, -300, 300, -300, -0.001)) for s in shapes]
        sub = [s for s in sub if s.Volume() > 1e-6]
        comp = cq.Compound.makeCompound(sub)
        bb = cq.BoundBox._fromTopoDS(comp.wrapped, optimal=True)
        fit[mname] = dict(below_deck_x=[round(bb.xmin, 2), round(bb.xmax, 2)], below_deck_y=[round(bb.ymin, 2), round(bb.ymax, 2)],
                          lowest_z=round(bb.zmin, 2), clearance_to_window_x_mm=round(WIN_HX - max(-bb.xmin, bb.xmax), 2),
                          clearance_to_window_y_mm=round(WIN_HY - max(-bb.ymin, bb.ymax), 2),
                          margin_to_UNDER_DECK_CLEAR_mm=round(bb.zmin - Z_UNDER_MIN, 2),
                          ok=(WIN_HX - max(-bb.xmin, bb.xmax) >= 0.5 and WIN_HY - max(-bb.ymin, bb.ymax) >= 0.5 and bb.zmin >= Z_UNDER_MIN))
    bbb = cq.BoundBox._fromTopoDS(tbin.wrapped, optimal=True)
    fit["tip_bin_(slides_out_to_front,_not_through_window)"] = dict(
        x=[round(bbb.xmin, 2), round(bbb.xmax, 2)], y=[round(bbb.ymin, 2), round(bbb.ymax, 2)], z=[round(bbb.zmin, 2), round(bbb.zmax, 2)],
        within_window_x=bbb.xmax <= WIN_HX - 0.5, margin_to_UNDER_DECK_CLEAR_mm=round(bbb.zmin - Z_UNDER_MIN, 2),
        top_below_front_beam_bottom_mm=round(-P.DECK_T - BEAM_H - bbb.zmax, 2),
        note="bin projects 41 mm forward of the A1 window under the front margin; frame must keep x -183..-81, y -253..-129, "
             "z -116..-46 free and leave a front opening for removal")
    c["module_window_fit"] = fit
    plates_fit = {}
    for nm, s in (("plain", plain_plate), ("magnet", mag_plate), ("magnet_block", mag_plate), ("thermal", th_plate), ("tip_waste", tw_plate)):
        bb = s.BoundingBox()
        plates_fit[nm] = dict(envelope=[round(bb.xlen, 2), round(bb.ylen, 2)], gap_to_neighbour_insert_x_mm=round(P.NEST_PITCH_X - bb.xlen, 2),
                              gap_to_neighbour_insert_y_mm=round(P.NEST_PITCH_Y - bb.ylen, 2))
    c["insert_plates"] = plates_fit

    # --- pairwise interference in the populated deck (bbox prefilter)
    allg = [(n, s) for n, s, k in glob] + refs
    hits = []
    for i in range(len(allg)):
        for j in range(i + 1, len(allg)):
            if allg[i][0].startswith(("REF_", "RES_")) and allg[j][0].startswith(("REF_", "RES_")):   # rev E: reservations overlap each other by design
                continue
            v = ivol(allg[i][1], allg[j][1])
            if v != v or v > 0.01:
                hits.append(dict(a=allg[i][0], b=allg[j][0], volume_mm3=None if v != v else round(v, 3)))
    c["deck_assembly_interference"] = dict(n_solids=len(allg), pairs_with_volume_gt_0p01mm3=hits, ok=not hits)
    eng = mv(carrier_local, z=g["carrier_top_engaged_full"])
    mags_eng = magnet_set(g["carrier_top_engaged_full"])
    mags_eng_s = magnet_set(g["carrier_top_engaged_semi"])
    pf = mv(pcr_full_local, z=SEAT)
    ps = mv(pcr_semi_local, z=SEAT - PCR["semi_skirt_bottom"])
    eng_checks = {}
    for nm, a, b in (("magnets_engaged_vs_full_skirt_plate", mags_eng, pf),
                     ("magnets_engaged_semi_vs_semi_skirt_plate", mags_eng_s, ps),
                     ("carrier_engaged_full_vs_full_plate", eng, pf),
                     ("carrier_engaged_semi_vs_semi_plate", mv(carrier_local, z=g["carrier_top_engaged_semi"]), ps),
                     ("carrier_engaged_vs_insert_plate", eng, mag_plate),
                     ("carrier_engaged_vs_housing", eng, housing),
                     ("carrier_engaged_vs_rods_screw", eng, cq.Compound.makeCompound([mb["rods"], mb["lead_screw"]])),
                     ("magnets_engaged_vs_insert_plate", mags_eng, mag_plate)):
        eng_checks[nm + "_mm3"] = round(ivol(a, b), 4)
    pfm = mv(pcr_full_local, z=SEAT)
    eng_checks["MB1_magnets_vs_full_skirt_plate_mm3"] = round(ivol(mb1_mags, pfm), 4)       # integration rev F
    eng_checks["MB1_body_vs_full_skirt_plate_mm3"] = round(ivol(mb1, pfm), 4)
    eng_checks["MB1_body_vs_insert_plate_mm3"] = round(ivol(mb1, mag_plate), 4)
    eng_checks["MB1_magnets_vs_insert_plate_mm3"] = round(ivol(mb1_mags, mag_plate), 4)
    c["magnet_engaged_interference"] = eng_checks
    c["mb1_passive_magnet_block_rev_f"] = mb1_checks()

    # --- SBS seat play in the corner locators
    def sbs_box(dx=0.0, dy=0.0, grow=0.0, h=10.0):
        return B(-SBS_HX - grow / 2 + dx, SBS_HX + grow / 2 + dx, -SBS_HY - grow / 2 + dy, SBS_HY + grow / 2 + dy, SEAT, SEAT + h)

    lc = cq.Compound.makeCompound(locs)
    c["sbs_seat"] = dict(
        pocket_mm=[round(2 * (SBS_HX + LOC["play"]), 2), round(2 * (SBS_HY + LOC["play"]), 2)],
        nominal_play_per_side_mm=LOC["play"],
        nominal_centred_mm3=round(ivol(sbs_box(), lc), 4),
        shifted_0p19_x_mm3=round(ivol(sbs_box(dx=0.19), lc), 4), shifted_0p19_y_mm3=round(ivol(sbs_box(dy=0.19), lc), 4),
        shifted_0p30_x_mm3=round(ivol(sbs_box(dx=0.30), lc), 4),
        max_tolerance_plate_plus0p25_mm3=round(ivol(sbs_box(grow=0.25), lc), 4),
        positional_repeatability="+/-0.20 mm for a nominal SBS part; +/-0.33 mm for a part at -0.25 mm (ANSI tolerance); "
                                 "a +0.25 mm part still drops in (0.075 mm per side)",
        locator_top_above_seat_mm=LOC["h"], lead_in_chamfer_mm={"long_side": LOC["chamfer_y"], "short_side": LOC["chamfer_x"]},
        gap_between_locators_of_X_neighbours_mm=round(P.NEST_PITCH_X - 2 * (SBS_HX + LOC["play"] + LOC["wall_x"]), 2),
        gap_between_locators_of_Y_neighbours_mm=round(P.NEST_PITCH_Y - 2 * (SBS_HY + LOC["play"] + LOC["y_arm_t"]), 2),
        tc24a_in_plain_insert_mm3=round(ivol(mv(tc24, z=SEAT), lc), 4),
        tc24a_note="existing TC24-A (cad/build.py) reused unchanged as the strip/tube rack; it seats in IN-P with the same play")

    # --- gripper finger corridors at every nest (incl. modules and the TC bay edges)
    corr = {}
    yi = SBS_HY
    yo = P.GRIP_OPEN / 2 + P.GRIP_FINGER_T
    solids_for_corr = [(n, s) for n, s, k in glob]
    for row in P.NEST_NAMES:
        for n in row:
            if n in MERGED_NESTS or DEFAULT_LAYOUT[n][0] == "tip_waste":   # integration rev D: no labware is gripped at the tip waste
                continue
            x, y = nest_xy(n)
            z0 = SEAT + 8.0
            boxes = [B(x - SBS_HX, x + SBS_HX, y + yi + 0.01, y + yo, z0, P.MAX_LABWARE_H),
                     B(x - SBS_HX, x + SBS_HX, y - yo, y - yi - 0.01, z0, P.MAX_LABWARE_H)]
            v = sum(ivol(bx, s) for bx in boxes for _, s in solids_for_corr)
            corr[n] = round(v, 4)
    c["gripper_corridors"] = dict(
        definition=f"both long sides, y from SBS edge to GRIP_OPEN/2 + finger ({yo:.1f} mm from nest centre), "
                   f"full SBS length, z from seat+8 = {SEAT + 8:.0f} to MAX_LABWARE_H",
        intersect_volume_mm3=corr, all_free=all(v == 0 for v in corr.values()),
        row_gap_between_labware_mm=round(P.NEST_PITCH_Y - P.SBS_W, 2),
        two_open_fingers_plus_clearance_mm=round(2 * (yo - yi), 2),
        highest_insert_feature_above_seat_mm=LOC["h"])

    c["deck_deflection"] = deck_deflection()
    c["magnet_module"] = dict(geometry_z_mm=g, layout="6 rows of 7 N52 2x10x20 mm blocks (magnetised through 2 mm), one row in every "
                                                      "other column gap (between columns 1-2, 3-4 ... 11-12): every well has one magnet "
                                                      "face beside it -> side pellet; rows span wells A-H",
                              row_x_mm=[round(v, 2) for v in magnet_rows_x()],
                              pellet_parameter="MAG['pellet_top'] = magnet top above the well bottom when engaged (default 9.0, tune on bench); "
                                               "the lead screw sets engaged height per plate type in firmware",
                              lift_travel_mm=MAG["travel"],
                              disengage_distance_full_mm=round(g["carrier_top_engaged_full"] - g["carrier_top_home"], 2),
                              disengage_distance_semi_mm=round(g["carrier_top_engaged_semi"] - g["carrier_top_home"], 2),
                              magnet_to_well_clearance_at_magnet_top_mm=round((P.WELL_PITCH_96 - well_od_at(MAG["pellet_top"]) - MAG["block"][0]) / 2, 2),
                              carrier_top_below_well_bottom_engaged_mm=round(g["well_bottom_full"] - g["carrier_top_engaged_full"], 2),
                              lead_screw_top_z=round(screw_top, 2),
                              home_sensor=dict(type="slotted optical interrupter (3 mm slot), flag fin on carrier underside",
                                               position_nest_xyz=[34.5, 0.0, round(z_ft, 2)],
                                               flag_in_slot_at_home=True, engaged_by_step_count_from_home=True),
                              force=magnet_force(carrier_local, mags_home))
    c["thermal_budget"] = thermal_budget()
    if getattr(P, "REV_J", False):
        c["cold_block_rev_j"] = cold_block_rev_j()
    zt = TH["block_top"]
    c["thermal_module_geometry"] = dict(block_top_z=zt, tube_top_z_nominal=zt - TH["tube_pocket"][1] + 40.0,
                                        tec_cold_face_z=TH["block_bottom"], lowest_z_fan=round(z_th_bottom, 2),
                                        exhaust="fins along X: air in from below through the fan, out at both X ends under the deck")
    tw = dict(opening_mm=list(TW["opening"]), eight_channel_span_mm=7 * P.WELL_PITCH_96,
              clearance_8ch_tip_collars_each_end_mm=round((TW["opening"][1] - 7 * P.WELL_PITCH_96 - 7.5) / 2, 2),
              bin_volume_L=round((2 * (TW["bin_body_hx"] - TW["bin_wall"])) * (TW["bin_y"][1] - TW["bin_y"][0] - 2 * TW["bin_wall"])
                                 * (TW["rail_z"] + 2.5 - TW["bin_bottom"] - TW["bin_wall"]) * 1e-6, 2),
              bin_capacity_note="~0.7 L: about 3 racks of loose p200 tips (est. 2.4 mL bulk per tip) - empty every run",
              strip_off="optional TW-4 plate: 8 slots 6.2 mm, nozzle enters slot with the tip collar below the plate, Z up strips")
    c["tip_waste"] = tw
    # --- size limits
    sz = []
    for p in rep["parts"]:
        e = sorted(p["envelope_mm"], reverse=True)
        if p["process"] == "sheet_laser":
            okk = e[0] <= 600 and e[1] <= 400
            sz.append(dict(part=p["name"], envelope=p["envelope_mm"], limit="600 x 400", ok=okk))
        elif p["process"].startswith("print"):
            sz.append(dict(part=p["name"], envelope=p["envelope_mm"], limit="250", ok=e[0] <= 250))
    c["size_limits"] = sz
    c["deck_plate_size_flag"] = ("DK-1 is 500 x 500 mm: exceeds the 600 x 400 project rule. Option A (preferred): confirm the "
                                 "laser supplier's bed (most cut 1500 x 3000 sheet; 500 x 500 x 6 is routine) and order one piece. "
                                 "Option B: DK-1F + DK-1B, 500 x 258 each, joined on y = 0 over the central beam by "
                                 f"{len(dts)} laser-cut dovetails at x = {dts}; the y=0 frame holes become M4 csk pairs at y = +/-6.")
    c["all_solids_valid"] = all(p["valid_solid"] for p in rep["parts"])

    # =================================================================== interfaces for others
    rep["deck_frame_fixing_holes"] = dict(
        fastener="M5 countersunk (ISO 10642) into slot-6 T-nuts in the beam top slot; heads flush, covered by inserts where under them",
        assumed_beams=f"2040 on edge: X beams at y = -225, -110, 0, 110 (x >= {BEAMS_X[110.0][0]:g}, post at x {TC_POST[0]:g}..{TC_POST[1]:g}), 225; "
                      "side beams along Y at x = +/-230",   # integration: frame rev B beam end
        holes=[dict(x=f["x"], y=f["y"]) for f in dfeats if f.get("frame")])
    mr = merged_window_rect(MERGED_WINDOWS[0])
    rep["comb_strength_rev_f"] = comb_strength()          # integration rev F (M3)
    rep["subplate_location_rev_f"] = subplate_repeatability(dfeats)   # integration rev F (M7)
    rep["thermocycler_bay"] = dict(
        merged=list(MERGED_WINDOWS[0]),
        window_corners_global=[[mr[0], mr[2]], [mr[1], mr[2]], [mr[1], mr[3]], [mr[0], mr[3]]],
        window_size_mm=[P.WINDOW_L, round(mr[3] - mr[2], 2)],
        retained_m4_tapped=[[f["x"], f["y"]] for f in dfeats if f.get("nest") in MERGED_NESTS and f["layer"] == "tap"],
        retained_dowels=[[f["x"], f["y"]] for f in dfeats if f.get("nest") in MERGED_NESTS and f["layer"] == "ream"],
        lost_inside_merged_window=[dict(x=f["x"], y=f["y"], what=f["note"]) for f in lost],
        note="Only 4 of the 8 M4 holes and 2 of the 4 dowels survive: the other half lay on the removed web (y = 102, 118) and "
             "are now inside the window. The bay module can use the 4 M4 + 2 dowels at y = 8 and y = 212 (pitch 204 mm in Y). "
             f"Frame (rev B): beam y=110 stops at x = {BEAMS_X[110.0][0]:g} on a 12 x 20 post between TC-1 and C2/D2; keep x -191..-73, "   # integration
             "y 16..204 clear below the deck for the thermocycler body. No frame hole within 10 mm of the bay.",
        min_frame_hole_distance_to_bay_mm=round(min(_dist_to_rect(f["x"], f["y"], mr) for f in dfeats if f.get("frame")), 2))
    rep["default_layout"] = {n: dict(insert=t, labware=l, centre=list(nest_xy(n))) for n, (t, l) in DEFAULT_LAYOUT.items()}
    rep["default_layout_note"] = ("10 usable nests: A1 tip waste, A2/A3/D2 tip racks, B1 sample plate, B2 magnet module (PCR plate), "
                                  "B3 fresh plate, C2 reagent reservoir, C3 thermal module (cold reagent tubes), D3 hand-off "
                                  "(plain insert); C1+D1 thermocycler bay (other engineer). Thermal at C3 exhausts to the open right side.")

    nplain = n_plain
    rep["bought_parts"] = [
        dict(item="M4x10 countersunk ISO 10642 A2 (insert to deck)", qty=4 * N_INSERTS),                  # rev H: 13 inserts
        dict(item="Dowel pin D4 m6 x 10 (press into deck, flush underneath)", qty=2 * (N_INSERTS + 1)),   # integration rev D: + 2 in the TC bay
        dict(item="M3x8 countersunk ISO 10642 A2 (locators)", qty=8 * (nplain + 1)),
        dict(item="M5x12 countersunk ISO 10642 + slot-6 T-nut (deck to frame)", qty=len(rep["deck_frame_fixing_holes"]["holes"])),
        dict(item="N52 block magnet 20 x 10 x 2 mm, magnetised through 2 mm, Ni-Cu-Ni", qty=6 * MAG["per_row"]),
        # integration rev F: lift motor, shafts, bushings and home sensor moved to rep['alternative_mm_lift'] (not in the base list)
        dict(item="M3 heat-set inserts (MB-1 ears 6, funnel 4)", qty=10),
        dict(item="ISO 10642 M3x8 A2 (MM-1 insert to MB-1 ears)", qty=6),
        dict(item="M3x12 countersunk (funnel 4, thermal frame 4) / M3x10 nylon PA66 (thermal clamp, with nylon shoulder washers)", qty=4 + 8 * N_THERMAL),   # rev G2: -6 (MM housing retired); rev H: x 2 thermal
        dict(item="TEC1-12706 40 x 40 x 3.8 Peltier", qty=2 * N_THERMAL),
        dict(item="Heat-sink extrusion 70 x 40 mm fin profile, 110 mm length", qty=N_THERMAL),
        dict(item="Fan 60 x 60 x 25, 24 V, ball bearing", qty=N_THERMAL),
        dict(item="NTC 10k glass bead thermistor, 3 mm probe", qty=N_THERMAL),
        dict(item="Axial thermal cut-off fuse 4-5 mm dia, 72 C (block), series with TEC supply", qty=N_THERMAL),
        dict(item="KSD9700 70 C NC bimetal thermostat on the sink end (fan-failure cut-out)", qty=N_THERMAL),
        dict(item="EPDM closed-cell foam 4 mm sheet (TEC gasket, cut to 108 x 70 with TEC windows)", qty=N_THERMAL),
        dict(item="Thermal paste / 0.5 mm pad for TEC faces", qty=N_THERMAL),
        dict(item="Silicone tube 3 x 5 mm (condensate drain), 300 mm", qty=N_THERMAL),
        # integration rev D: small parts the build sequence lists as missing (DM-02..04), and the comb screws
        dict(item="ISO 10642 M3x6 A2 (TW-4D comb feet into TW-1)", qty=4),
        dict(item="ISO 4762 M3x8 A2 (fan shroud)", qty=2 * N_THERMAL),
        dict(item="ISO 4762 M3x10 A2 (thermal fan)", qty=4 * N_THERMAL),
        dict(item="RTV silicone, neutral cure (TEC edge seal)", qty=1),
    ]
    rep["alternative_mm_lift"] = dict(   # integration rev F: documented alternative, NOT in the base parts list
        why_not_default="a contact running a motorised moving-magnet module on an OT-2 reported run failures; LH-1's gripper "
                        "allows the passive MB-1 (fewer parts, no motor, sensor or macro)",
        custom_parts=["MM-2_magnet_housing", "MM-3_magnet_carrier"],
        bought=["NEMA17 34 mm stepper with integrated T8 lead 2 mm lead screw + flange nut", "Linear shaft D6 h6 x 73 mm x 2",
                "LM6UU x 2", "Slotted optical endstop", "M3 heat-set inserts x 10", "ISO 4762 M3x8 x 6, M3x6 x 4"],
        uses="MM-1 insert plate and the same 42 magnets; to fit it, set DEFAULT_LAYOUT B2 = ('magnet', ...)")
    rep["params_change_requests"] = [
        dict(param="SIDE_ZONE_LEFT / SIDE_ZONE_RIGHT", request="remove or mark unused", reason="coordinator decision: side zones are outside the gripper and far-pipette reach (carriage +/-260); tip waste is nest A1, hand-off is plain nest D3"),
        dict(param="new: MERGED_WINDOWS / thermocycler bay", request="add MERGED_WINDOWS = [('C1','D1')] and the bay window 118 x 188 to params.py so frame and thermocycler engineers share it",
             reason="this build implements it locally"),
        dict(param="INSERT_SCREWS of merged nests", request="bay module to use the 4 surviving M4 (y = 8, 212) + 2 dowels; the 4 M4 + 2 dowels at y = 102/118 cannot exist",
             reason="they fall inside the merged window"),
        dict(param="DECK_L x DECK_W = 500 x 500", request="confirm supplier sheet limit, or accept two-piece deck (DK-1F/DK-1B)", reason="project sheet rule 600 x 400"),
        # integration: resolved by the frame (params DECK_BEAM_Y110_END_X = -47); deflection recomputed with that end
        dict(param="frame (not in params)", request=f"X beam at y=110 ends at x={BEAMS_X[110.0][0]:g} on a 12 x 20 mm post (frame rev B, accepted); all deck beams 2040 on edge under 32 mm webs/margins",
             reason="200 N tip press at D2 deflects ~2 mm if the deck spans 0..225 without it"),
    ]
    (OUT / "report.json").write_text(json.dumps(rep, indent=1, default=float))
    return rep


if __name__ == "__main__":
    r = main()
    c = r["checks"]
    print(json.dumps({k: c[k] for k in ("module_window_fit", "sbs_seat", "magnet_engaged_interference", "all_solids_valid")}, indent=1))
    print(json.dumps(c["deck_assembly_interference"], indent=1)[:3000])
    print(json.dumps(c["gripper_corridors"], indent=1))
    print(json.dumps({k: c["deck_deflection"][k] for k in ("point_20N_at_deck_centre_mm", "tip_press_200N_worst_nest", "tip_press_200N_worst_mm")}, indent=1))
    print(json.dumps(c["thermal_budget"], indent=1))
    print(json.dumps(c["magnet_module"], indent=1))
    print(json.dumps(c["size_limits"], indent=1))
    print(json.dumps(r["thermocycler_bay"], indent=1))
