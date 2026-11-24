"""LH-1 revision L, pass 2: the enlarged (K2) machine in 3D with the exchange head (ADR-010), and its whole-machine checks.

Run from the project root:  .venv/bin/python cad/lh1/machine_l/build.py
Out: cad/lh1/machine_l/out/{report.json, rev_l.json, machine_l_assembly.step, <part>.step/.dxf, machine_l_parts.csv}
     and, in cad/lh1/assembly/out: the rev_l key of report.json (rev_h, rev_k untouched), changes_rev_l.yaml,
     lh1_bom_l_proposed.csv, proposed_build_parts_l.yaml.

What is here: the K2 frame grown once (4 x 6 = 24 standard places, raised gantry), the deck plates with the module standard and
connector notches, every revision K station placed as the K2 layout says, the eight-bay dock behind row G, the side shelf with the two
edge hand-offs and the tip towers, the under-base drawer (tip crate, waste bottles), the enclosure with the extraction spigot, the
shuttered hand-off openings and the blanked roof opening; the head_l exchange head placed on the gantry; interference and reach checks
for transit, every tool at every place it serves, the gripper, the dock paths and the eject over-travel.

Revisions H to K are untouched: this module is new and only it and head_l read the L_* names in params.py.

Status: CAD and calculation only.  Nothing built or measured: every figure is CALCULATED from the model or from an input marked ASSUMED.
Frame: global deck frame of params.py (deck top z 0, +Y away from the operator).  Head: carriage centre xc, beam front face y yf,
global z = head-local z + L_X_BEAM_MID_Z.
"""
from __future__ import annotations

import csv
import importlib.util
import json
import math
import sys
import time
from pathlib import Path

import cadquery as cq
import numpy as np

HERE = Path(__file__).resolve().parent
LH1 = HERE.parent
sys.path.insert(0, str(LH1))
import params as P  # noqa: E402

OUT = HERE / "out"
AOUT = LH1 / "assembly" / "out"
T0 = time.time()


def _load(name, rel):
    spec = importlib.util.spec_from_file_location(name, str(LH1 / rel))
    m = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(m)
    return m


HL = _load("head_l_for_machine", "head_l/build.py")
DKK = _load("deck_k_for_machine", "deck_k/build.py")
DK = DKK.DK
B, cyl, mv, fuse_all, cut_all = DK.B, DK.cyl, DK.mv, DK.fuse_all, DK.cut_all
V = cq.Vector
SEAT = P.LABWARE_SEAT_Z
ZB = P.L_X_BEAM_MID_Z
RHO = {"al": 2.70e-6, "steel": 7.85e-6, "pa12": 1.01e-6, "pc": 1.20e-6, "pp": 0.90e-6}
AY = HL.AXY[1]                     # coupling axis, carriage-local y (-102.5)
GRIP_Y = -104.0                    # gripper finger centre, carriage-local y (head_l z_axis_moving)


def log(*a):
    print(f"[machine_l {time.time() - T0:6.1f}s]", *a, flush=True)


# =====================================================================================================
# 1. frame geometry (derived; every rule named)
# =====================================================================================================
_hb = cq.Compound.makeCompound(list(HL.head_parts().values())).BoundingBox()
HEAD_X = (_hb.xmin, _hb.xmax)                       # head-local x extent with tools (-160 .. 141.2)
HEAD_Y = (_hb.ymin, _hb.ymax)
XC0, XC1 = P.L_XC_RANGE
YF0, YF1 = P.L_YF_RANGE
X_MARGIN = 8.5                                      # rev B rule: swept carriage to anything at the gantry ends
MG15 = dict(H=16.0, W=32.0, L=58.8, rail_w=15.0, rail_h=10.0)   # MGN15H catalogue-nominal (to verify on the parts, gate G0)
XR_L = XC0 + HEAD_X[0] - X_MARGIN - MG15["W"] / 2   # left Y rail centre
XR_R = XC1 + HEAD_X[1] + X_MARGIN + MG15["W"] / 2   # right Y rail centre
BEAM = dict(h=100.0, d=50.0, t=5.0, x0=XR_L - MG15["W"] / 2 - 4.5, x1=XR_R + MG15["W"] / 2 + 4.5)
YC = dict(t=P.L_YC_T, dy=(-125.0, 55.0))            # Y carriage plate under each beam end, relative to the beam front face
YB_REL = (-5.0, -90.0)                              # Y block centres relative to the beam front face (rev B)
SB = dict(z1=ZB - BEAM["h"] / 2 - YC["t"] - MG15["H"])   # side (Y-rail) beam top
SB["z0"] = SB["z1"] - 40.0
Y_RAIL = (YF0 + YB_REL[1] - MG15["L"] / 2 - 5.0, YF1 + YB_REL[0] + MG15["L"] / 2 + 5.0)
FY0 = -425.0                                        # frame front (door plane -430); head front at yf_min: YF0 + HEAD_Y[0]
FY1 = YF1 + YC["dy"][1] + 45.0                      # back corner uprights (40 deep) 5 mm behind the Y carriage at the back of travel
BASE = dict(z1=-(P.DECK_T + P.UNDER_DECK_CLEAR + 20.0), t=20.0)    # base top z -136 (as revisions H-K)
BASE["z0"] = BASE["z1"] - BASE["t"]
FOOT_H = 200.0                                      # K2: under-base drawer feet (tip crate 170 mm + drawer)
HEAD_TOP = max(v.BoundingBox().zmax for v in HL.head_parts(sA=-P.Z_EJECT_TRAVEL, sB=-P.Z_EJECT_TRAVEL).values()) + ZB
ROOF_Z = math.ceil((HEAD_TOP + 15.0) / 5.0) * 5.0   # roof frame bottom: 15 mm over the head top at eject over-travel
FX0, FX1 = XR_L - 20.0, XR_R + 20.0                 # outer faces of the side members
EX0, EX1 = XR_L - 70.0, XR_R + 70.0                 # side panels outside the Y motors and idlers
PANEL_T = {"side": 2.0, "door": 5.0, "roof": 2.0, "back": 2.0}
DECK_X, DECK_Y = P.L_DECK_X, P.L_DECK_Y
DECK_SPLIT_Y = 0.0
COLS, ROWS = P.L_COLS, P.L_ROWS
PLACES = {n: dict(kind=v[0], h=v[1], work=v[2], c=(COLS[int(n[1])], ROWS[n[0]])) for n, v in DKK.K2_PLACES.items()}
WIN = (P.WINDOW_L / 2, P.WINDOW_W / 2)
DOCK_N = 8
TC1_STRIP_DX = -15.0          # pass 2 fix: TC-1 lid guide strip 15 mm further out (tools at column 1 of B1/C1 reach x -208.5)
TPK_RECESS = 0.0
# pass 2 layout fixes found by the checks (K2 changes, CALCULATED):
#  - tip parking TPK-1 and the D2 plate stack swap places: the real TPK-1 is 86.6 mm tall and the eight-channel cradles working C1
#    column 12 pass over C2 at z 57-98 (K2 assumed 50 mm); at D2 nothing is worked deep beside it and G2 behind takes <= 72
#  - G4 p1000 rack recessed 75 (was 50), G2 p200 and G3 p300 frames recessed 7, A2 rack recessed 20 (was 15): the eight-channel manifold cheeks
#    working the next column (col 12 / col 1) pass over them
PLACES["C2"], PLACES["D2"] = dict(PLACES["D2"], c=PLACES["C2"]["c"]), dict(PLACES["C2"], c=PLACES["D2"]["c"])
PLACES["D2"]["h"] = round(DKK.TPK["z_lower_top"] + DKK.TPK["collar_h"] + 0.6 + DKK.TPK["t_upper"] - SEAT, 1)
PLACES["G4"]["h"] = 75.0
PLACES["G2"]["h"] = 57.0
PLACES["G3"]["h"] = 65.0
PLACES["A2"]["h"] = 50.0
SHAKER_RECESS = 40.0         # pass 2 fix: shaker at D4 sunk 40 mm on an under-deck tray (1.5 x Z clearance; lidded 44 mm plate fits)
RECESS = {"G4": 75.0, "G2": 7.0, "G3": 7.0, "A2": 20.0, "F1": 60.0, "F4": 60.0}
DOCK_X = [P.L_DOCK_X0 + i * P.L_DOCK_PITCH for i in range(DOCK_N)]
DOCK_Y = P.L_DOCK_Y
DOCK_TOOLS = ["M10", "M50", "M300", "S10", "S1000", "SYR", None, None]       # first build; bay 7 S100 later, bay 8 cable bay
DOCK_TOOLS_FULL = ["M10", "M50", "M300", "S10", "S1000", "SYR", "S100", "M300"]  # worst case for the checks (bay 8: a 72 mm tool)
SHELF = dict(x0=DECK_X[1], x1=XR_R - 30.0, y0=-345.0, y1=DOCK_Y + 60.0, t=4.0)
SHELF_SEAT = SHELF["t"]
# shelf slots: inner column x 410 rows F..G; outer column x 540 rows F..G and at the dock row (the inner one there is bay 8)
SLOTS = {}
for _r, _y in ROWS.items():
    SLOTS[f"I{_r}"] = (P.L_SHELF_X["inner"], _y)
    SLOTS[f"O{_r}"] = (P.L_SHELF_X["outer"], _y)
SLOTS["OK"] = (P.L_SHELF_X["outer"], DOCK_Y)
HANDOFF_SLOTS = {"H1": "OF", "H2": "OA"}


def place_window(n):
    cx, cy = PLACES[n]["c"]
    return (cx - WIN[0], cx + WIN[0], cy - WIN[1], cy + WIN[1])


def merged_window(a, b):
    w1, w2 = place_window(a), place_window(b)
    return (min(w1[0], w2[0]), max(w1[1], w2[1]), min(w1[2], w2[2]), max(w1[3], w2[3]))


def windows():
    out, used = {}, set()
    for a, b in P.L_MERGED_WINDOWS:
        out[f"{a}+{b}"] = merged_window(a, b)
        used |= {a, b}
    for n in PLACES:
        if n not in used:
            out[n] = place_window(n)
    return out


# =====================================================================================================
# 2. frame, gantry, enclosure, drawer (static and Y-moving parts)
# =====================================================================================================
def ext(x0, x1, y0, y1, z0, z1):
    return B(x0, x1, y0, y1, z0, z1)


def tube(x0, x1, y0, y1, z0, z1, t):
    return B(x0, x1, y0, y1, z0, z1).cut(B(x0 - 1, x1 + 1, y0 + t, y1 - t, z0 + t, z1 - t))


DOCK_RAIL_Y = (DOCK_Y - 53.0, DOCK_Y + 42.0)        # under the dock base plate (global y 375.5..500.5)
DECK_BAR_Y = []                        # 40 x 10 Al flat bars on edge under the deck: front edge, row boundaries + 5 (clear of the
for _b in (-220.0, -110.0, 0.0, 110.0, 220.0):    # connector notches, which take the +Y 14 mm of each window edge), back edge
    DECK_BAR_Y.append(_b + 5.0)
DECK_BAR_Y = [DECK_Y[0] + 9.0] + DECK_BAR_Y + [DECK_Y[1] - 9.0]
CHUTE = dict(cx=PLACES["A1"]["c"][0], cy=PLACES["A1"]["c"][1] + 1.0, w=100.0, wall=3.0, z_top=-60.0)
DRAWER = dict(x0=-345.0, x1=470.0, y0=-415.0, y1=-15.0, z0=BASE["z0"] - FOOT_H + 6.0, z1=BASE["z0"] - 4.0, t=3.0)
CRATE = dict(L=500.0, W=340.0, H=170.0, cx=-90.0, cy=-215.0)    # 29 L class (was 20 L: fails 1.5 x at 7,000 tips)
BOTTLE = dict(L=210.0, W=120.0, H=140.0)            # ASSUMED flat 2 L HDPE canister class (a 2 L round bottle is ~280 tall: does not fit)


def deck_bar_x_ranges(y):
    """the bars run between the deck posts at both deck edges; the C/D bar stops either side of the TC-1 body; the F/A bar either
    side of the A1 tip funnel and chute (100 x 100)"""
    x0, x1 = DECK_X[0] - 10.0, DECK_X[1] + 10.0
    tc = (-214.5 - 2.5, -49.5 + 2.5)
    ch = (CHUTE["cx"] - CHUTE["w"] / 2 - CHUTE["wall"] - 2.0, CHUTE["cx"] + CHUTE["w"] / 2 + CHUTE["wall"] + 2.0)
    if abs(y - 115.0) < 1:
        return [(x0, tc[0]), (tc[1], PLACES["D4"]["c"][0] - 80.0)]          # also stops short of the sunk shaker at D4
    if abs(y - (-215.0)) < 1:
        return [(x0, ch[0]), (ch[1], x1)]
    return [(x0, x1)]


def frame_items():
    it = []

    def add(name, s, group="frame", cls="part", mat="al"):
        it.append(dict(name=name, solid=s, group=group, cls=cls, mat=mat))
    zb0, zb1 = BASE["z0"], BASE["z1"]
    # base rectangle (2040 flat) + longitudinal members under the deck posts + cross members
    add("EXT-base_front", ext(FX0, FX1, FY0, FY0 + 40, zb0, zb1))
    add("EXT-base_back", ext(FX0, FX1, FY1 - 40, FY1, zb0, zb1))
    for side, x in (("L", XR_L), ("R", XR_R)):
        add(f"EXT-base_side_{side}", ext(x - 20, x + 20, FY0 + 40, FY1 - 40, zb0, zb1))
    for nm, x in (("deckL", DECK_X[0] - 10.0), ("deckR", DECK_X[1] + 10.0)):
        add(f"EXT-base_long_{nm}", ext(x - 20, x + 20, FY0 + 40, FY1 - 40, zb0, zb1))
    for y in (-335.0, -240.0):                       # balance stand rests here (F2) through sorbothane pads
        add(f"EXT-base_cross_y{y:+.0f}", ext(DECK_X[0] + 10.0, DECK_X[1] - 10.0, y - 20, y + 20, zb0, zb1))
    for y in DOCK_RAIL_Y:                             # under the dock base
        add(f"EXT-base_cross_y{y:+.0f}", ext(XR_L + 20, XR_R - 20, y - 20, y + 20, zb0, zb1))
    # feet (2040 posts) at the corners and mid-sides, drawer below the base
    for x in (XR_L, XR_R):
        for y in (FY0 + 20, 0.0, FY1 - 20):
            add(f"EXT-foot_{x:+.0f}_{y:+.0f}", ext(x - 10, x + 10, y - 20, y + 20, zb0 - FOOT_H, zb0))
    # uprights: corners to the roof frame; mid-side uprights up to the side beams (none at the hand-off openings y -355..-85)
    for x in (XR_L, XR_R):
        for y in (FY0 + 20, FY1 - 20):
            add(f"EXT-upright_{x:+.0f}_{y:+.0f}", ext(x - 10, x + 10, y - 20, y + 20, zb1, ROOF_Z))
        for y in (0.0, 330.0):
            add(f"EXT-upright_mid_{x:+.0f}_{y:+.0f}", ext(x - 10, x + 10, y - 20, y + 20, zb1, SB["z0"]))
    # side beams (2040 on edge) carrying the MGN15 Y rails
    for side, x in (("L", XR_L), ("R", XR_R)):
        add(f"EXT-side_beam_{side}", ext(x - 10, x + 10, FY0 + 40, FY1 - 40, SB["z0"], SB["z1"]))
        add(f"MGN15_Y_rail_{side}", B(x - MG15["rail_w"] / 2, x + MG15["rail_w"] / 2, Y_RAIL[0], Y_RAIL[1], SB["z1"], SB["z1"] + MG15["rail_h"]),
            cls="bought", mat="steel")
    # back top cross member behind the beam's travel; roof frame (2020) above the head
    add("EXT-back_top", ext(XR_L + 10, XR_R - 10, FY1 - 20, FY1, SB["z0"], SB["z1"]))
    for nm, (x0, x1, y0, y1) in {"front": (FX0, FX1, FY0, FY0 + 20), "back": (FX0, FX1, FY1 - 20, FY1),
                                 "left": (FX0, FX0 + 20, FY0 + 20, FY1 - 20), "right": (FX1 - 20, FX1, FY0 + 20, FY1 - 20)}.items():
        add(f"EXT-roof_{nm}", ext(x0, x1, y0, y1, ROOF_Z, ROOF_Z + 20))
    # deck supports: 40 x 10 flat bars on edge on 2020 posts at the deck's X edges
    for y in DECK_BAR_Y:
        for k, (x0, x1) in enumerate(deck_bar_x_ranges(y)):
            add(f"DSB-L_deck_bar_y{y:+.0f}_{k}", B(x0, x1, y - 5, y + 5, -46.0, -P.DECK_T))
        for x in (DECK_X[0] - 10.0, DECK_X[1] + 10.0):
            add(f"EXT-deck_post_{x:+.0f}_{y:+.0f}", ext(x - 10, x + 10, y - 10, y + 10, zb1, -46.0))
    for y in (115.0, -215.0):                        # short 10 x 10 posts (steel tube) under the split bar ends
        for (x0, x1) in deck_bar_x_ranges(y):
            for x in (x0, x1):
                if DECK_X[0] < x < DECK_X[1]:
                    xx = x + 5.0 if x == x0 else x - 5.0
                    add(f"SQT-deck_post_split_{x:+.0f}_{y:+.0f}", ext(xx - 5, xx + 5, y - 5, y + 5, zb1, -46.0), mat="steel")
    # shelf supports (2040 on edge along Y) and the dock base supports
    for x in (SHELF["x0"] + 25.0, SHELF["x1"] - 15.0):
        add(f"EXT-shelf_rail_{x:+.0f}", ext(x - 10, x + 10, SHELF["y0"], SHELF["y1"], -40.0, 0.0))
    for y in DOCK_RAIL_Y:
        add(f"EXT-dock_rail_y{y:+.0f}", ext(XR_L + 10, SHELF["x0"] + 15.0, y - 10, y + 10, -46.0, -6.0))
        for x in (XR_L + 20, SHELF["x0"] + 5.0):
            add(f"EXT-dock_post_{x:+.0f}_{y:+.0f}", ext(x - 10, x + 10, y - 10, y + 10, zb1, -46.0))
    return it


def gantry_items(yf):
    """Y-moving group at beam front face yf: X beam, X rails, Y carriages, Y blocks, X/Y drive envelopes"""
    it = []

    def add(name, s, cls="part", mat="al"):
        it.append(dict(name=name, solid=s, group="gantry", cls=cls, mat=mat))
    add("TUBE-X_beam_100x50x5", tube(BEAM["x0"], BEAM["x1"], yf, yf + BEAM["d"], ZB - BEAM["h"] / 2, ZB + BEAM["h"] / 2, BEAM["t"]))
    for z in (-20.0, 20.0):
        add(f"MGN15_X_rail_z{z:+.0f}", B(BEAM["x0"] + 20, BEAM["x1"] - 20, yf - 5.0, yf, ZB + z - 7.5, ZB + z + 7.5), cls="bought", mat="steel")
    for side, x in (("L", XR_L), ("R", XR_R)):
        xin, xout = (x - MG15["W"] / 2 - 14.0, x + MG15["W"] / 2) if side == "L" else (x - MG15["W"] / 2, x + MG15["W"] / 2 + 14.0)
        add(f"YC-L_y_carriage_{side}", B(xin, xout, yf + YC["dy"][0], yf + YC["dy"][1], ZB - BEAM["h"] / 2 - YC["t"], ZB - BEAM["h"] / 2))
        for yb in YB_REL:
            add(f"MGN15H_Y_block_{side}{yb:+.0f}", B(x - MG15["W"] / 2, x + MG15["W"] / 2, yf + yb - MG15["L"] / 2, yf + yb + MG15["L"] / 2,
                                                   SB["z1"] + 4.0, SB["z1"] + MG15["H"]), cls="bought", mat="steel")
    # X drive: closed-loop NEMA17 on the left Y carriage above the beam end, idler on the right
    add("X_motor_NEMA17_closed_loop", B(XR_L - 21, XR_L + 21, yf + 4, yf + 46, ZB + BEAM["h"] / 2, ZB + BEAM["h"] / 2 + 60), cls="bought")
    add("X_idler_block", B(XR_R - 15, XR_R + 15, yf + 10, yf + 40, ZB + BEAM["h"] / 2, ZB + BEAM["h"] / 2 + 30))
    return it


def y_drive_items():
    it = []
    for side, x in (("L", XR_L), ("R", XR_R)):
        sx = -1 if side == "L" else 1
        it.append(dict(name=f"Y_motor_NEMA17_closed_loop_{side}", solid=B(x + sx * 22 - 21, x + sx * 22 + 21, FY1 - 85, FY1 - 43, SB["z0"] - 62, SB["z0"]),
                       group="frame", cls="bought", mat="al"))
        it.append(dict(name=f"Y_idler_{side}", solid=B(x + sx * 22 - 10, x + sx * 22 + 10, FY0 + 45, FY0 + 75, SB["z0"] - 25, SB["z0"]),
                       group="frame", cls="part", mat="al"))
    return it


def enclosure_items():
    it = []

    def add(name, s, cls="part", mat="al", group="enclosure"):
        it.append(dict(name=name, solid=s, group=group, cls=cls, mat=mat))
    zl, zt = BASE["z0"], ROOF_Z + 20.0
    t = PANEL_T
    # door (PC) in front; left side; back with the extraction spigot; roof with the blanked FFU opening; right side with two openings
    add("ENC-L-DR_door_PC", B(EX0, EX1, FY0 - 5 - t["door"], FY0 - 5, zl, zt), mat="pc")
    add("ENC-L-SL_side_left", B(EX0 - t["side"], EX0, FY0 - 5, FY1, zl, zt))
    back = B(EX0, EX1, FY1, FY1 + t["back"], zl, zt)
    sp = dict(x=150.0, z=ROOF_Z - 80.0, d=100.0)
    add("ENC-L-BK_back_panel", back.cut(cq.Solid.makeCylinder(sp["d"] / 2 + 1, 10, V(sp["x"], FY1 - 3, sp["z"]), V(0, 1, 0))))
    add("EXF-1_extraction_fan_and_spigot", fuse_all([cq.Solid.makeCylinder(sp["d"] / 2, 40, V(sp["x"], FY1 + t["back"], sp["z"]), V(0, 1, 0)),
                                                     cq.Solid.makeCylinder(65.0, 110, V(sp["x"], FY1 + t["back"] + 40, sp["z"]), V(0, 1, 0))]), cls="bought")
    roof = B(EX0, EX1, FY0 - 5, FY1, zt, zt + t["roof"])
    rc = ((FX0 + FX1) / 2, 100.0)
    roof = roof.cut(B(rc[0] - 160, rc[0] + 160, rc[1] - 160, rc[1] + 160, zt - 1, zt + 3))
    add("ENC-L-TOP_roof_panel", roof)
    add("ENC-L-RB_roof_blank_FFU", cut_all(B(rc[0] - 170, rc[0] + 170, rc[1] - 170, rc[1] + 170, zt + t["roof"], zt + t["roof"] + 2.0),
                                           [cyl(rc[0] + sx * 150, rc[1] + sy * 150, zt, zt + 6, 5.5) for sx in (-1, 1) for sy in (-1, 1)]))
    right = B(EX1, EX1 + t["side"], FY0 - 5, FY1, zl, zt)
    ops, shut = [], []
    for h, (hx, hy) in P.L_HANDOFF.items():
        z0 = SHELF_SEAT + 2.0 - 15.0                 # plate seat 15 mm above the opening's lower edge
        ops.append(B(EX1 - 1, EX1 + t["side"] + 1, hy - 80, hy + 80, z0, z0 + 90))
        shut.append((h, B(EX1 + t["side"] + 1, EX1 + t["side"] + 4, hy - 80 - 10, hy + 80 + 10, z0 + 90 + 5, z0 + 90 + 5 + 100)))
    add("ENC-L-SR_side_right", cut_all(right, ops))
    for h, s in shut:
        add(f"ENC-L-HS_{h}_shutter_PC_open", s, mat="pc")
    return it


def drawer_items():
    it = []

    def add(name, s, cls="part", mat="al", group="drawer"):
        it.append(dict(name=name, solid=s, group=group, cls=cls, mat=mat))
    d = DRAWER
    box = B(d["x0"], d["x1"], d["y0"], d["y1"], d["z0"], d["z1"]).cut(B(d["x0"] + d["t"], d["x1"] - d["t"], d["y0"] + d["t"], d["y1"] - d["t"], d["z0"] + d["t"], d["z1"] + 1))
    wall_x = 164.0
    box = box.fuse(B(wall_x - 1.5, wall_x + 1.5, d["y0"], d["y1"], d["z0"], d["z0"] + 120.0))     # bund wall: waste compartment
    add("DRW-L_under_base_drawer", box)
    add("DRW-L_slides", fuse_all([B(d["x0"] - 13, d["x0"], d["y0"], d["y1"], d["z1"] - 50, d["z1"] - 5), B(d["x1"], d["x1"] + 13, d["y0"], d["y1"], d["z1"] - 50, d["z1"] - 5)]),
        cls="bought", mat="steel")
    c = CRATE
    z0 = d["z0"] + d["t"]
    add("TB-2_tip_crate_20L", B(c["cx"] - c["L"] / 2, c["cx"] + c["L"] / 2, c["cy"] - c["W"] / 2, c["cy"] + c["W"] / 2, z0, z0 + c["H"]).cut(
        B(c["cx"] - c["L"] / 2 + 3, c["cx"] + c["L"] / 2 - 3, c["cy"] - c["W"] / 2 + 3, c["cy"] + c["W"] / 2 - 3, z0 + 3, z0 + c["H"] + 1)), cls="bought", mat="pp")
    can_x = (235.0, 400.0)                            # long side along Y under A4; drain 1 straight down, drain 2 offset 69 mm
    for k, x in enumerate(can_x):
        b = BOTTLE
        add(f"LWB-{k + 1}_waste_canister_2L", B(x - b["W"] / 2, x + b["W"] / 2, -165.0 - b["L"] / 2, -165.0 + b["L"] / 2, z0, z0 + b["H"]),
            cls="bought", mat="pp")
    # tip chute from the A1 funnel to 30 mm above the crate rim; two FEP drains from LW-1 at A4 into the canister caps
    ch = CHUTE
    zb = z0 + c["H"] + 30.0
    add("TW-L_tip_chute_100x100", B(ch["cx"] - ch["w"] / 2 - ch["wall"], ch["cx"] + ch["w"] / 2 + ch["wall"], ch["cy"] - ch["w"] / 2 - ch["wall"],
                                    ch["cy"] + ch["w"] / 2 + ch["wall"], zb, ch["z_top"]).cut(
        B(ch["cx"] - ch["w"] / 2, ch["cx"] + ch["w"] / 2, ch["cy"] - ch["w"] / 2, ch["cy"] + ch["w"] / 2, zb - 1, ch["z_top"] + 1)), group="station", mat="pp")
    ax, ay = PLACES["A4"]["c"]
    ztop = z0 + BOTTLE["H"] + 2
    add("LW-1_drain_FEP_1", cyl(ax - 29.0, ay, ztop, -62.0, 10.0), cls="bought", mat="pp", group="station")
    add("LW-1_drain_FEP_2", fuse_all([cyl(ax + 29.0, ay, -120.0, -62.0, 10.0), DK.cylx(ax + 24.0, can_x[1] + 5.0, ay, -120.0, 10.0),
                                      cyl(can_x[1], ay, ztop, -115.0, 10.0)]), cls="bought", mat="pp", group="station")
    return it


def deck_items():
    """deck plates (two pieces) with windows, connector notches, place bolt patterns; plain inserts; connectors at fitted places"""
    it = []
    win = windows()
    cuts_f, cuts_b = [], []
    win["D4"] = (PLACES["D4"]["c"][0] - 77.0, DECK_X[1] + 1.0, PLACES["D4"]["c"][1] - 57.0, PLACES["D4"]["c"][1] + 53.0)   # sunk shaker cut-out
    for nm, (x0, x1, y0, y1) in win.items():
        c = B(x0, x1, y0, y1, -P.DECK_T - 1, 1)
        # one connector notch per window, +Y edge at place-local x +15..+45 (of the window's first place)
        first = nm.split("+")[0]
        px = PLACES[first]["c"][0]
        nx0, nx1, nd = P.L_CONNECTOR_NOTCH
        c = c.fuse(B(px + nx0, px + nx1, y1 - 1, y1 + nd, -P.DECK_T - 1, 1))
        (cuts_f if y0 < DECK_SPLIT_Y else cuts_b).append(c)
    holes_f, holes_b = [], []
    for n, pl in PLACES.items():
        cx, cy = pl["c"]
        for x, y in P.INSERT_SCREWS:
            (holes_f if cy < 0 else holes_b).append(cyl(cx + x, cy + y, -P.DECK_T - 1, 1, 3.3))
        for x, y in P.INSERT_DOWELS:
            (holes_f if cy < 0 else holes_b).append(cyl(cx + x, cy + y, -P.DECK_T - 1, 1, 4.0))
    for x, y in P.TC1_STRIP_TAPPED_HOLES:
        holes_b.append(cyl(x + TC1_STRIP_DX, y, -P.DECK_T - 1, 1, 3.3))
    vb = mv(DKK.vc2_parts()["bracket"], *PLACES["A1"]["c"]).BoundingBox()
    if vb.zmin < 0:                                  # VC-2B foot passes the deck: a clearance slot (as in the rev K deck)
        cuts_f.append(B(vb.xmin - 1, vb.xmax + 1, vb.ymin - 1, vb.ymax + 1, -P.DECK_T - 1, 1).intersect(B(vb.xmin - 1, vb.xmax + 1, vb.ymin - 1, vb.ymax + 1, -10, 1)))
    front = cut_all(B(DECK_X[0], DECK_X[1], DECK_Y[0], DECK_SPLIT_Y, -P.DECK_T, 0.0), cuts_f + holes_f)
    back = cut_all(B(DECK_X[0], DECK_X[1], DECK_SPLIT_Y, DECK_Y[1], -P.DECK_T, 0.0), cuts_b + holes_b)
    it.append(dict(name="DK-L1_deck_front", solid=front, group="deck", cls="part", mat="al"))
    it.append(dict(name="DK-L2_deck_back", solid=back, group="deck", cls="part", mat="al"))
    # connectors (fitted places): keyed panel-mount connector on a bracket under the notch
    for n in P.L_CONNECTOR_PLACES:
        if n in ("C1", "D4"):
            continue                                   # TC-1 (rev H) and the sunk shaker bring their own leads (cut-out edge at D4)
        px, py = PLACES[n]["c"]
        x0c = px + (60.0 if n == "A1" else 18.0)        # A1: beside the tip funnel (it widens to 106 mm under the deck)
        it.append(dict(name=f"CON-L_{n}_deck_connector", solid=B(x0c, x0c + (11.0 if n == "A1" else 24.0), py + WIN[1] + 3.0, py + WIN[1] + 13.0, -40.0, -P.DECK_T - 1.0),
                       group="deck", cls="bought", mat="pa12", place=n))
    return it


def insert_plate(cx, cy, cut_window=False):
    s = B(cx - P.INSERT_L / 2, cx + P.INSERT_L / 2, cy - P.INSERT_W / 2, cy + P.INSERT_W / 2, 0.0, P.INSERT_T)
    if cut_window:
        s = s.cut(B(cx - WIN[0] + 4, cx + WIN[0] - 4, cy - WIN[1] + 4, cy + WIN[1] - 4, -1, P.INSERT_T + 1))
    return s


def sbs(cx, cy, z0, z1, dx=0.0):
    return B(cx - P.SBS_L / 2 + dx, cx + P.SBS_L / 2 + dx, cy - P.SBS_W / 2, cy + P.SBS_W / 2, z0, z1)


def recessed(cx, cy, top, recess):
    return fuse_all([B(cx - WIN[0] + 1, cx + WIN[0] - 1, cy - WIN[1] + 1, cy + WIN[1] - 1, SEAT - recess, 0.01), sbs(cx, cy, 0.0, top)])


def thermal_under(cx, cy, w=None):
    x0, x1, y0, y1 = w if w else (cx - WIN[0] + 1, cx + WIN[0] - 1, cy - WIN[1] + 1, cy + WIN[1] - 1)
    return B(x0, x1, y0, y1, -115.0, -P.DECK_T - 0.5)


def station_items():
    """every K2 place filled as the K2 layout says (deck_k K2_PLACES), revision K parts reused where they exist"""
    it = []

    def put(n, name, s, cls="labware", part=None, local=False, group="station"):
        if local:
            s = mv(s, *PLACES[n]["c"])
        it.append(dict(name=f"{n}_{name}", solid=s, group=group, cls=cls, place=n, part=part or name, mat="al"))
    plain = ("A3", "B2", "B3", "B4", "C2", "C3", "C4", "D1", "D2", "D3", "G1")
    for n in plain:
        put(n, "IN-P_plain_insert", insert_plate(*PLACES[n]["c"]), cls="part", part="IN-P_plain_insert_plate")
    # row F
    cx, cy = PLACES["F1"]["c"]
    put("F1", "ATR-1_rack_50_15mL_recessed60", recessed(cx, cy, SEAT + 100.0, 60.0))
    b = DKK.bal_parts()
    for k in ("cell", "stem", "pan", "insert", "shield", "stand_top"):
        put("F2", f"bal_{k}", b[k], cls="labware" if k in ("shield", "pan") else "part")
    st = b["stand_top"].BoundingBox()
    put("F2", "bal_stand_pads", fuse_all([B(x - 10, x + 10, y - 10, y + 10, BASE["z1"], st.zmin) for x in (st.xmin + 15, st.xmax - 15) for y in (-335.0, -240.0)]),
        cls="bought", part="sorbothane_pad")
    w34 = merged_window("F3", "F4")
    xm = (PLACES["F3"]["c"][0] + PLACES["F4"]["c"][0]) / 2
    put("F3", "thermal_module_under_F3", B(w34[0] + 1, xm, w34[2] + 1, w34[3] - 1, -115.0, -P.DECK_T - 0.5), cls="part", part="thermal_module_rev_K")
    put("F4", "thermal_module_under_F4", B(xm, w34[1] - 1, w34[2] + 1, w34[3] - 1, -115.0, SEAT - 60.0 - 0.5), cls="part", part="thermal_module_rev_K (CTB-2 recessed into it)")
    cx, cy = PLACES["F3"]["c"]
    put("F3", "CTB-1_cold_tube_block", sbs(cx, cy, 0.0, SEAT + 82.0))
    cx, cy = PLACES["F4"]["c"]
    put("F4", "CTB-2_cold_tube_block_recessed60", recessed(cx, cy, SEAT + 100.0, 60.0))
    # row A
    cx, cy = PLACES["A1"]["c"]
    z6 = -P.DECK_T - 0.5
    outer = cq.Solid.makeLoft([cq.Wire.makePolygon([V(cx - WIN[0] + 1, cy - WIN[1] + 1, z6), V(cx + WIN[0] - 1, cy - WIN[1] + 1, z6), V(cx + WIN[0] - 1, cy + WIN[1] - 1, z6),
                                                    V(cx - WIN[0] + 1, cy + WIN[1] - 1, z6)], close=True),
                               cq.Wire.makePolygon([V(CHUTE["cx"] - 53, CHUTE["cy"] - 53, CHUTE["z_top"]), V(CHUTE["cx"] + 53, CHUTE["cy"] - 53, CHUTE["z_top"]),
                                                    V(CHUTE["cx"] + 53, CHUTE["cy"] + 53, CHUTE["z_top"]), V(CHUTE["cx"] - 53, CHUTE["cy"] + 53, CHUTE["z_top"])], close=True)], True)
    inner = cq.Solid.makeLoft([cq.Wire.makePolygon([V(cx - WIN[0] + 4, cy - WIN[1] + 4, z6 + 0.5), V(cx + WIN[0] - 4, cy - WIN[1] + 4, z6 + 0.5),
                                                    V(cx + WIN[0] - 4, cy + WIN[1] - 4, z6 + 0.5), V(cx - WIN[0] + 4, cy + WIN[1] - 4, z6 + 0.5)], close=True),
                               cq.Wire.makePolygon([V(CHUTE["cx"] - 50, CHUTE["cy"] - 50, CHUTE["z_top"] - 0.5), V(CHUTE["cx"] + 50, CHUTE["cy"] - 50, CHUTE["z_top"] - 0.5),
                                                    V(CHUTE["cx"] + 50, CHUTE["cy"] + 50, CHUTE["z_top"] - 0.5), V(CHUTE["cx"] - 50, CHUTE["cy"] + 50, CHUTE["z_top"] - 0.5)], close=True)], True)
    lip = B(cx - WIN[0] + 1, cx + WIN[0] - 1, cy - WIN[1] + 1, cy + WIN[1] - 1, z6, 0.0).cut(B(cx - WIN[0] + 4, cx + WIN[0] - 4, cy - WIN[1] + 4, cy + WIN[1] - 4, z6 - 1, 1.0))
    put("A1", "TW-L_tip_funnel", outer.cut(inner).fuse(lip).clean(), cls="part", part="TW-L_tip_funnel_100x100_outlet")
    v = DKK.vc2_parts()
    for k in ("camera", "bracket", "backlight"):
        put("A1", f"vc2_{k}", v[k], cls="part", local=True)
    cx, cy = PLACES["A2"]["c"]
    put("A2", "ATR-2_rack_recessed20", recessed(cx, cy, SEAT + PLACES["A2"]["h"], RECESS["A2"]))
    cx, cy = PLACES["A3"]["c"]
    put("A3", "thermal_module_under", thermal_under(cx, cy), cls="part", part="thermal_module_rev_K")
    put("A3", "warm_trough_or_tube_block", sbs(cx, cy, SEAT, SEAT + 44.0))
    put("A4", "LW-1_twin_waste_funnel", mv(DKK.lw_parts(), 0, 0, 3.0), cls="part", local=True)    # flange ON the deck (rev K had it 3 mm into it)
    # row B
    w = DKK.wn_parts()
    for k in ("bracket", "platform", "spacer", "drip_cover", "cell", "insert"):
        put("B1", f"wn_{k}", w[k], cls="part", local=True)
    cx, cy = PLACES["B1"]["c"]
    put("B1", "deepwell_plate_44", sbs(cx, cy, SEAT + DKK.WN["gap"], SEAT + 44.0))
    cx, cy = PLACES["B2"]["c"]
    put("B2", "MB-1_magnet_block_under", B(cx - 55, cx + 55, cy - 35, cy + 35, -35.0, -P.DECK_T - 0.5), cls="part", part="MB-1_magnet_block")
    put("B2", "pcr_plate_on_MB-1", sbs(cx, cy, SEAT, SEAT + 16.0))
    cx, cy = PLACES["B3"]["c"]
    put("B3", "thermal_module_under", thermal_under(cx, cy), cls="part", part="thermal_module_rev_K")
    put("B3", "CPB-1_block_and_plate", sbs(cx, cy, SEAT, SEAT + 30.0))
    cx, cy = PLACES["B4"]["c"]
    put("B4", "low_elution_magnet_and_plate", sbs(cx, cy, SEAT, SEAT + 16.0))
    # row C
    cx, cy = PLACES["C1"]["c"]
    put("C1", "TC-1_body_under", B(-214.5, -49.5, 16.0, 204.0, -116.0, -P.DECK_T - 0.5), cls="part", part="TC-1_thermocycler")
    put("C1", "TC-1_block_and_plate", B(cx - WIN[0], cx + WIN[0], cy - WIN[1], cy + WIN[1], 0.0, P.TC1_PLATE_TOP_Z))
    s = P.TC1_LID_STRIP
    put("C1", "TC-1_lid_guide_strip", B(s["x"][0] + TC1_STRIP_DX, s["x"][1] + TC1_STRIP_DX, *s["y"], *s["z"]), cls="part", part="TC-1_lid_guide_strip")
    tp, _ = DKK.tpk_parts()
    for k in ("lower", "upper", "posts", "tray"):
        put("D2", f"tpk_{k}", tp[k], cls="labware", local=True)
    cx, cy = PLACES["C3"]["c"]
    put("C3", "thermal_module_under", thermal_under(cx, cy), cls="part", part="thermal_module_rev_K")
    put("C3", "cold_strip_carrier", sbs(cx, cy, SEAT, SEAT + 23.5))
    stack_top = DKK.stack_height("pcr_plate_full_skirt")
    for n in ("C4", "C2"):
        cx, cy = PLACES[n]["c"]
        put(n, "plate_stack_x4", sbs(cx, cy, SEAT, stack_top))
    # row D
    cx, cy = PLACES["D1"]["c"]
    put("D1", "TC-1_lid_parked", B(cx - 64, cx + 64, cy - 47, cy + 47, SEAT, SEAT + 40.0))
    cx, cy = PLACES["D3"]["c"]
    put("D3", "lid_park_5_lids", sbs(cx, cy, SEAT, SEAT + 47.5))
    sp, zs = DKK.shaker_parts()
    for k in ("adapter", "mounts", "body", "plate", "lid"):
        put("D4", f"shaker_{k}", mv(sp[k], -DKK.SK["x_off"], 0, -SHAKER_RECESS), cls="labware" if k in ("plate", "lid") else "part", local=True)
    cx, cy = PLACES["D4"]["c"]
    zt = SEAT - SHAKER_RECESS
    put("D4", "SKT-L_shaker_tray", fuse_all([B(cx - 76, cx + 76, cy - 54, cy + 54, zt - 6.0, zt)] +
                                            [B(cx + sx * 70 - 4, cx + sx * 70 + 4, cy + sy * 48 - 4, cy + sy * 48 + 4, zt, -P.DECK_T - 0.5) for sx in (-1, 1) for sy in (-1, 1)]),
        cls="part", part="SKT-L_shaker_tray")
    zs = zs - SHAKER_RECESS
    # row G: tip frames (p1000 recessed 50)
    put("G1", "tip_frame_p10", DKK.tip_frame("p10"), local=True)
    cx, cy = PLACES["G3"]["c"]
    put("G3", "tip_frame_p300_recessed7", recessed(cx, cy, SEAT + PLACES["G3"]["h"], RECESS["G3"]))
    cx, cy = PLACES["G2"]["c"]
    put("G2", "tip_frame_p200_recessed7", recessed(cx, cy, SEAT + PLACES["G2"]["h"], RECESS["G2"]))
    cx, cy = PLACES["G4"]["c"]
    put("G4", "tip_frame_p1000_recessed75", recessed(cx, cy, SEAT + PLACES["G4"]["h"], RECESS["G4"]))
    return it, zs


def shelf_items(plan):
    it = []
    sh = SHELF
    plate = B(sh["x0"], sh["x1"], sh["y0"], sh["y1"], 0.0, sh["t"]).fuse(B(sh["x1"] - sh["t"], sh["x1"], sh["y0"], sh["y1"], 0.0, 20.0))
    # the shelf stops short of dock bay 8 (its tool and post) at the dock row
    b8 = DOCK_X[-1] + 45.0
    d4 = PLACES["D4"]["c"]
    plate = plate.cut(B(sh["x0"] - 1, sh["x0"] + 10.0, d4[1] - 60.0, d4[1] + 56.0, -1, 30))     # sunk shaker overhang (body to x 337.5)
    if b8 > sh["x0"]:
        plate = plate.cut(B(sh["x0"] - 1, b8, DOCK_Y - 175.0, sh["y1"] + 1, -1, 30))
    it.append(dict(name="TSS-L_tip_supply_shelf", solid=plate, group="shelf", cls="part", mat="al"))
    for slot, (x, y) in SLOTS.items():
        content = plan["slots"].get(slot)
        if content is None:
            continue
        kind, h = content["kind"], content["height_mm"]
        if kind == "handoff":
            s = B(x - 66, x + 66, y - 54, y + 54, SHELF_SEAT, SHELF_SEAT + 3.0)
            it.append(dict(name=f"{slot}_{content['name']}_nest", solid=s, group="shelf", cls="part", mat="al", place=slot))
            it.append(dict(name=f"{slot}_{content['name']}_plate", solid=sbs(x, y, SHELF_SEAT + 3.0, SHELF_SEAT + 3.0 + 16.1), group="shelf", cls="labware",
                           mat="pp", place=slot))
            continue
        L, W = DKK.WAFER_PLATE["L"], DKK.WAFER_PLATE["W"] + 3.0
        it.append(dict(name=f"{slot}_{kind}", solid=B(x - L / 2, x + L / 2, y - W / 2, y + W / 2, SHELF_SEAT, SHELF_SEAT + h), group="shelf", cls="labware",
                       mat="pp", place=slot))
        g = []
        for sx in (-1, 1):
            for sy in (-1, 1):
                gx, gy = x + sx * (L / 2 + 2.0), y + sy * (W / 2 - 6.5)
                g.append(B(gx - 2.0, gx + 2.0, gy - 6.0, gy + 6.0, SHELF_SEAT, SHELF_SEAT + 40.0))
        it.append(dict(name=f"{slot}_guides", solid=fuse_all(g), group="shelf", cls="part", mat="pa12", place=slot))
    return it


def dock_items(tools):
    """eight bays (head_l bay design) at global positions + the parked tools"""
    it = []
    dy, dz = DOCK_Y - AY, ZB
    parts = HL.bay_parts(x0=DOCK_X[0], y0=dy, z0=dz, n=DOCK_N, base_z=-ZB)
    for k, s in parts.items():
        bay = int(k[3]) if k.startswith("bay") else None
        it.append(dict(name=f"DOCK_{k}", solid=s, group="dock", cls="bought" if "switch" in k else "part", mat="al",
                       place=f"BAY{bay + 1}" if bay is not None else "DOCK"))
    it.append(dict(name="DOCK_cable_retractor_bay8_RESERVED", solid=B(DOCK_X[-1] + 90, DOCK_X[-1] + 150, FY1 - 45, FY1 - 25, ZB + 60, ZB + 120),
                   group="reserved", cls="reserved", mat="al", place="BAY8"))
    for i, key in enumerate(tools):
        if key is None:
            continue
        for k, s in HL.tool_module(DOCK_X[i], P.L_DOCK_S, key).items():
            it.append(dict(name=f"PARKED{i + 1}_{key}_{k}", solid=mv(s, 0, dy, dz), group="parked_tool", cls="labware", mat="al",
                           place=f"BAY{i + 1}", tool=key, leaf=k))
    return it


def control_bay_items():
    return [dict(name="CB-L_control_bay_RESERVED", solid=B(DECK_X[0] + 15, SHELF["x0"] - 10, DOCK_Y - 140.0, FY1 - 50, BASE["z1"] + 5, -50.0),
                 group="reserved", cls="reserved", mat="al")]


# =====================================================================================================
# 3. tip supply (item 4) and shelf plan
# =====================================================================================================
TIP_DEMAND_L = {"p300": 4000, "p200": 2000, "p10": 1200, "p1000": 300}       # coordinator update (planner a418c41): sizing figures
TIP_RUN_96 = {"p300": 3824, "p200": 1776, "p10": 840, "p1000": 300}         # 96-sample day 1 as planned today (p1000 ASSUMED = sizing figure)
TIP_WASTE_REQ = 7000                                                          # coordinator: drawer must take about 7,000 tips with margin
WAFER_L = dict(DKK.WAFER)
WAFER_L["p1000"] = dict(pitch=20.0, tip_l=P.TIP_BELOW_NOZZLE["p1000"])     # ASSUMED nested p1000 refill wafers (purchasing to confirm)


def tower_max_l(tip, cap=P.MAX_LABWARE_H):
    w = WAFER_L[tip]
    return int((cap - (w["tip_l"] + DKK.WAFER_PLATE["t"])) // w["pitch"]) + 1


def tower_h(tip, n):
    w = WAFER_L[tip]
    return w["tip_l"] + DKK.WAFER_PLATE["t"] + (n - 1) * w["pitch"]


def tip_supply_plan_l():
    rows, towers = {}, []
    for tip, n in TIP_DEMAND_L.items():
        wafers = math.ceil(n / 96.0)
        need = wafers + 2                                    # rule: largest need + 2 spare (wafers)
        on_shelf = need - 1                                  # one on the deck frame (G1-G4)
        per = tower_max_l(tip)
        nt = math.ceil(on_shelf / per)
        rows[tip] = {"tips": n, "wafers_needed": wafers, "with_2_spare": need, "on_deck_frame": 1, "on_shelf": on_shelf,
                     "wafers_per_tower_at_150": per, "towers": nt, "capacity_tips": (nt * per + 1) * 96}
        towers += [(tip, min(per, on_shelf - k * per)) for k in range(nt)]
    total_wafers = sum(r["with_2_spare"] for r in rows.values())
    spent_cap = int(P.MAX_LABWARE_H // DKK.SPENT_PITCH)
    free = [s for s in SLOTS if s not in HANDOFF_SLOTS.values()]
    order = ["IF", "IA", "IB", "IC", "ID", "IG", "OB", "OC", "OD", "OG", "OK"]
    slots, overflow = {}, []
    for h, s in HANDOFF_SLOTS.items():
        slots[s] = {"kind": "handoff", "name": h, "height_mm": 19.1}
    items = [(f"tower_{t}", tower_h(t, n), f"{t} x {n} wafers") for t, n in towers] + [("spent_wafers", 150.0, f"first {spent_cap} spent wafers")]
    for (kind, h, what), s in zip(items, order):
        slots[s] = {"kind": kind, "height_mm": round(h, 1), "what": what}
    overflow = [{"kind": k, "what": w} for k, h, w in items[len(order):]]
    run = {}
    for tip, n in TIP_RUN_96.items():
        cap_on = rows[tip]["capacity_tips"]
        wafers_used = math.ceil(n / 96.0)
        refills = max(wafers_used - 1, 0)
        run[tip] = {"tips_96_sample_day1": n, "held_on_machine_at_once": cap_on, "whole_run_on_board": cap_on >= n,
                    "frame_refills": refills, "gripper_moves (spent off + new on)": 2 * refills}
    run["total"] = {"tips": sum(TIP_RUN_96.values()), "held": sum(r["capacity_tips"] for r in rows.values()),
                    "gripper_moves": sum(r["gripper_moves (spent off + new on)"] for r in run.values() if "gripper_moves (spent off + new on)" in r),
                    "note": "a refill is two gripper moves (about 30 s together, rev K tip_supply timing); with the tip rules (p300 ~2,300) the p300 "
                            "refills fall from %d to %d" % (math.ceil(TIP_RUN_96["p300"] / 96) - 1, math.ceil(2300 / 96) - 1)}
    vt = {"p300": 2.0, "p200": 1.7, "p10": 0.6, "p1000": 4.5}
    mean = sum(vt[t] * n for t, n in TIP_RUN_96.items()) / sum(TIP_RUN_96.values())
    crate_gross = CRATE["L"] * CRATE["W"] * CRATE["H"] / 1e6
    waste = {"requirement_tips": TIP_WASTE_REQ, "rule_1.5x_tips": int(1.5 * TIP_WASTE_REQ),
             "mean_cm3_per_tip (run mix, loose, ASSUMED rev K packing)": round(mean, 2), "pile_fill": 0.7,
             "litres_needed_at_1.5x": round(1.5 * TIP_WASTE_REQ * mean / 0.7 / 1000, 1),
             "rev_K_20L_crate_holds_tips": int(20.0 * 1000 * 0.7 / mean), "rev_K_20L_ratio": round(20.0 * 1000 * 0.7 / mean / TIP_WASTE_REQ, 2),
             "verdict_20L": "FAILS the 1.5 x rule (it just covers 7,000 tips)",
             "chosen": f"TB-2L: {CRATE['L']:.0f} x {CRATE['W']:.0f} x {CRATE['H']:.0f} mm lidded PP crate class ({crate_gross:.1f} L gross) in the "
                       "under-base drawer; the waste-canister bay moves right (bund wall at x 164)",
             "chosen_holds_tips": int(crate_gross * 1000 * 0.7 / mean), "chosen_ratio": round(crate_gross * 1000 * 0.7 / mean / TIP_WASTE_REQ, 2)}
    return {"run_96_day1": run, "tip_waste": waste, "basis": "coordinator update (planner a418c41) sizing figures: " + ", ".join(f"{v:,} {k}" for k, v in TIP_DEMAND_L.items()) +
                     "; rule: largest need + 2 spare wafers of each type; towers capped at 150 mm above the shelf seat (MAX_LABWARE_H, as rev K)",
            "per_type": rows, "towers_needed": len(towers), "slots_needed_incl_spent_stack": len(items), "slots_for_tips": len(order),
            "handoffs": {h: s for h, s in HANDOFF_SLOTS.items()}, "slots": slots, "fits": not overflow, "does_not_fit": overflow,
            "spent_wafers": {"total": total_wafers - 4, "one_stack_holds": spent_cap,
                             "rule": "the dedicated stack takes the first spent wafers; every tower slot that empties becomes a spent stack "
                                     "(planner rule), so one dedicated slot is enough"},
            "wafer_assumption": "p300/p200/p10 wafer pitch 12/10/7 mm (rev K, ASSUMED); p1000 nested wafers at 20 mm pitch ASSUMED: if p1000 "
                                "comes only in racks of 96, each rack takes a slot (5 slots)",
            "what_it_would_take": [
                f"{len(overflow)} more slot(s): a third shelf column at x {P.L_SHELF_X['outer'] + 130:.0f} needs the X carriage travel "
                f"and the frame {130:.0f} mm wider (+130 mm of X beam: M1 beam torsion and bending terms grow about 12 % and 40 %)",
                "or towers to 172 mm above the shelf (p300 10 wafers, p10 20): the head's lowest fixed point (z 190) then clears them by "
                "only 14 mm, under the 1.5 x clearance rule: not recommended",
                "or size p300 for the tip-rule figure (about 2,300 + 2 spare wafers = 27 wafers, 4 towers) and keep the 4,000 figure "
                "for the no-rules case only: then the shelf fits with one slot to spare"]}


# =====================================================================================================
# 4. head placement and tips
# =====================================================================================================
TOOL_TIP = {k: t["tip"] for k, t in HL.TOOLS.items()}


def tips_for(key, x, s):
    t = HL.TOOLS[key]
    if not t["tip"]:
        return None
    nz = HL.tpl_top(s) - t["Hn"]
    L = P.TIP_BELOW_NOZZLE[t["tip"]]
    rt = P.TIP_COLLAR_OD[t["tip"]] / 2
    ys = [AY + (k - 3.5) * 9.0 for k in range(8)] if t["eight"] else [AY]
    return fuse_all([cq.Solid.makeCone(0.6, rt, L, V(x, y, nz - L), V(0, 0, 1)) for y in ys])


class Head:
    """head_l parts cached at s = 0 and translated (every moving part of an axis is rigid with its stroke)"""

    def __init__(self):
        self.fixed = {f"fixed:{k}": v for k, v in HL.head_fixed().items()}
        self.axes = {}
        for ax in ("carriage_A", "carriage_B"):
            self.axes[ax] = {f"{ax}:{k}": v for k, v in HL.z_axis_moving(P.L_AXIS_X[ax], 0.0).items()}
        self.axes["gripper"] = {f"gripper:{k}": v for k, v in HL.z_axis_moving(P.L_AXIS_X["gripper"], 0.0, gripper=True).items()}
        for sy in (-1, 1):                            # fingers OPEN for every approach (gap 92 = GRIP_OPEN; head_l models them part-closed)
            k = f"gripper:finger{sy}"
            self.axes["gripper"][k] = self.axes["gripper"][k].translate(V(0, sy * 4.0, 0))
        self.tools, self.tips = {}, {}
        for ax in ("carriage_A", "carriage_B"):
            for key in HL.TOOLS:
                self.tools[(ax, key)] = {f"{ax}:tool:{k}": v for k, v in HL.tool_module(P.L_AXIS_X[ax], 0.0, key).items()}
                tp = tips_for(key, P.L_AXIS_X[ax], 0.0)
                if tp is not None:
                    self.tips[(ax, key)] = tp
        self.bb = {}
        for d in [self.fixed] + list(self.axes.values()) + list(self.tools.values()):
            for k, v in d.items():
                self.bb[id(v)] = v.BoundingBox()
        for v in self.tips.values():
            self.bb[id(v)] = v.BoundingBox()

    def parts(self, xc, yf, sA=0.0, sB=0.0, sG=0.0, toolA="M300", toolB="S1000", tipsA=True, tipsB=True):
        """list of (name, solid0, offset(dx,dy,dz)) without building translated shapes"""
        out = [(k, v, (xc, yf, ZB)) for k, v in self.fixed.items()]
        for ax, s, tool, tips in (("carriage_A", sA, toolA, tipsA), ("carriage_B", sB, toolB, tipsB)):
            off = (xc, yf, ZB - s)
            out += [(k, v, off) for k, v in self.axes[ax].items()]
            if tool:
                out += [(k, v, off) for k, v in self.tools[(ax, tool)].items()]
                if tips and (ax, tool) in self.tips:
                    out.append((f"{ax}:tool:tips", self.tips[(ax, tool)], off))
        out += [(k, v, (xc, yf, ZB - sG)) for k, v in self.axes["gripper"].items()]
        return out

    def bbox(self, v, off):
        b = self.bb[id(v)]
        return (b.xmin + off[0], b.xmax + off[0], b.ymin + off[1], b.ymax + off[1], b.zmin + off[2], b.zmax + off[2])


# =====================================================================================================
# 5. interference engine
# =====================================================================================================
class Scene:
    def __init__(self, items):
        self.items = items
        bbs = []
        for it in items:
            b = it["solid"].BoundingBox()
            bbs.append((b.xmin, b.xmax, b.ymin, b.ymax, b.zmin, b.zmax))
        self.bb = np.array(bbs)

    def candidates(self, bb, tol=0.0):
        S = self.bb
        m = (S[:, 0] < bb[1] + tol) & (S[:, 1] > bb[0] - tol) & (S[:, 2] < bb[3] + tol) & (S[:, 3] > bb[2] - tol) & \
            (S[:, 4] < bb[5] + tol) & (S[:, 5] > bb[4] - tol)
        return np.nonzero(m)[0]


def overlap_volume(a, b):
    try:
        return a.intersect(b).Volume()
    except Exception:  # noqa: BLE001
        return -1.0


def check_config(scene, head, hparts, label, skip=lambda hn, it: False, tol_mm3=1.0):
    hits = []
    for hn, v, off in hparts:
        bb = head.bbox(v, off)
        idx = scene.candidates(bb)
        if not len(idx):
            continue
        moved = None
        for i in idx:
            it = scene.items[i]
            if skip(hn, it):
                continue
            if moved is None:
                moved = v.translate(V(*off))
            vol = overlap_volume(moved, it["solid"])
            if vol > tol_mm3 or vol < 0:
                hits.append({"config": label, "head_part": hn, "item": it["name"], "mm3": round(vol, 1), "place": it.get("place"), "cls": it["cls"]})
    return hits


def lowest_point(head, hparts, prefix=None):
    z = 1e9
    who = None
    for hn, v, off in hparts:
        if prefix and not hn.startswith(prefix):
            continue
        b = head.bbox(v, off)
        if b[4] < z:
            z, who = b[4], hn
    return z, who


# =====================================================================================================
# 6. targets: where each tool works at each place (K2 work types) and how deep
# =====================================================================================================
PIPETTES = [k for k, t in HL.TOOLS.items() if t["tip"]]
EIGHT = [k for k in PIPETTES if HL.TOOLS[k]["eight"]]
SINGLE = [k for k in PIPETTES if not HL.TOOLS[k]["eight"]]
FRAME_TIP = {"G1": "p10", "G2": "p200", "G3": "p300", "G4": "p1000"}
TUBE_BOTTOM = {"F1": SEAT + 100.0 - 115.0, "F4": SEAT + 100.0 - 115.0, "F3": SEAT + 82.0 - 39.0, "A2": SEAT + 50.0 - 57.0}   # 50 mL / 1.5 mL / 5 mL
PLATE_FLOOR = {"B1": SEAT + 2.5, "B2": SEAT + 1.0, "B3": SEAT + 30.0 - 15.0, "B4": SEAT + 1.0, "C1": P.TC1_PLATE_TOP_Z - 15.0,
               "C3": SEAT + 23.5 - 15.0, "A3": SEAT + 3.0}


def nozzle_top(key):
    return HL.tpl_top(0.0) + ZB - HL.TOOLS[key]["Hn"]


def s_for_tip_end(key, z_tip_end, tips=True):
    L = P.TIP_BELOW_NOZZLE[HL.TOOLS[key]["tip"]] if (tips and HL.TOOLS[key]["tip"]) else 0.0
    return nozzle_top(key) - (z_tip_end + L)


def pipette_targets():
    """(place, tool, list of (x, y) nozzle-axis points, s_target, tips_on, what, exclude-classes for target labware)"""
    out = []
    for n, pl in PLACES.items():
        cx, cy = pl["c"]
        w = pl["work"]
        if w == "deep":
            for k in PIPETTES:
                pts = [(cx - 49.5, cy), (cx + 49.5, cy)] if HL.TOOLS[k]["eight"] else [(cx - 49.5, cy + 31.5), (cx + 49.5, cy - 31.5)]
                out.append((n, k, pts, s_for_tip_end(k, PLATE_FLOOR[n] + 0.5), True, "into wells to the floor",
                            ("nozzles", "tips", "pipette") if n == "A3" else ("nozzles", "tips")))
            out.append((n, "SYR", [(cx - 49.5, cy), (cx + 49.5, cy)], nozzle_top("SYR") - (SEAT + pl["h"] + 3.0), False, "jet 3 mm above the labware", ()))
        elif w == "deep_single_only":
            tools = {"F1": ("S100", "S1000"), "F4": ("S100", "S1000"), "F3": SINGLE, "A2": SINGLE}[n]
            for k in tools:
                pts = [(cx - 45.0, cy - 28.0), (cx + 45.0, cy + 28.0)]
                zb = (SEAT + 50.0 - 39.0) if (n == "A2" and k == "S10") else TUBE_BOTTOM[n]
                out.append((n, k, pts, s_for_tip_end(k, zb + 0.5), True, "into tubes to the bottom", ("nozzles", "tips", "pipette")))
        elif n == "A1":
            for k in PIPETTES:
                out.append((n, k, [(cx, cy)], s_for_tip_end(k, SEAT + 10.0), True, "tip eject over the funnel (tip end z 15)", ()))
        elif n in FRAME_TIP:
            for k in PIPETTES:
                if HL.TOOLS[k]["tip"] != FRAME_TIP[n]:
                    continue
                top = SEAT + pl["h"]
                pts = [(cx - 49.5, cy), (cx + 49.5, cy)] if HL.TOOLS[k]["eight"] else [(cx - 49.5, cy + 31.5), (cx + 49.5, cy + 31.5)]
                out.append((n, k, pts, nozzle_top(k) - (top - 8.0), False, "tip pick-up: bare nozzles 8 mm into the collars", ("nozzles", "pipette")))
        elif n == "D2":
            zl1 = DKK.TPK["z_lower_top"] - TPK_RECESS
            for k in ("M300",):
                out.append((n, k, [(cx + DKK.TPK["x_spots"][0], cy), (cx + DKK.TPK["x_spots"][-1], cy)], s_for_tip_end(k, zl1 - P.TIP_BELOW_NOZZLE["p300"] + 1.0),
                            True, "tip parking: collars into the TPK-1 slots", ("nozzles", "tips")))
        elif n == "F2":
            px, py = DKK.BAL_PAN_XY
            for k in SINGLE:
                out.append((n, k, [(px, py)], s_for_tip_end(k, DKK.BAL_CELL["pan_z"] + 20.0), True, "dispense into a tube on the balance pan", ("nozzles", "tips")))
        elif w == "centre":
            for k in PIPETTES:
                out.append((n, k, [(cx - 29.0, cy), (cx + 29.0, cy)], s_for_tip_end(k, SEAT + 15.0), True, "wall dispense 10 mm below the splash wall", ()))
    return out


def gripper_targets(zs_shaker, plan):
    """(place, (x, y), fingertip z, what)"""
    t = {}
    for n, pl in PLACES.items():
        cx, cy = pl["c"]
        h = SEAT + pl["h"]
        if n in ("B1",):
            t[n] = ((cx, cy), SEAT + DKK.WN["gap"] + 2.0, "deep-well plate on WN-1")
        elif n in ("B2", "B3", "B4", "C3", "A3"):
            t[n] = ((cx, cy), SEAT + 2.0 if n != "A3" else h - 6.5, "plate (or trough lid) at its skirt")
        elif n == "C1":
            t[n] = ((cx, cy), P.TC1_PLATE_TOP_Z - 16.1 + 2.0, "plate in TC-1")
        elif n in ("C4", "C2"):
            s_ = DKK.STACK["pcr_plate_full_skirt"]
            t[n] = ((cx, cy), SEAT + (s_["n"] - 1) * s_["pitch"] + s_["grip_above_unit_bottom"], "top plate of a stack of 4")
        elif n == "D3":
            t[n] = ((cx, cy), SEAT + 47.5 - 9.5 + 3.0, "top lid of 5")
        elif n == "D4":
            t[n] = ((cx, cy), zs_shaker + 3.0, "plate on the shaker (lid off)")
        elif n in FRAME_TIP:
            t[n] = ((cx, cy), h - DKK.WAFER_PLATE["t"] - 1.0, "refill wafer lips on the frame")
        elif n in ("F1", "F3", "F4", "A2"):
            t[n] = ((cx, cy), h + 9.5 - 6.5, "tube-rack lid TRL-1 on the rack")
    for slot, c in plan["slots"].items():
        x, y = SLOTS[slot]
        if c["kind"] == "handoff":
            t[c["name"]] = ((x, y), SHELF_SEAT + 3.0 + 2.0, "plate at the edge hand-off")
        elif c["kind"].startswith(("tower", "spent")):
            t[slot] = ((x, y), SHELF_SEAT + c["height_mm"] - DKK.WAFER_PLATE["t"] - 1.0, f"top wafer of {c['kind']}")
    return t


def grip_s(z_fingertip):
    return (ZB - 194.0) - z_fingertip                 # gripper fingertips at s 0: head-local -194 (head_l z_axis_moving)


def in_travel(xc, yf):
    return XC0 - 1e-6 <= xc <= XC1 + 1e-6 and YF0 - 1e-6 <= yf <= YF1 + 1e-6


# =====================================================================================================
# 7. checks
# =====================================================================================================
def run_checks(scene, head, zs_shaker, plan):
    res = {"transit": [], "lowered": [], "gripper": [], "dock": [], "eject": [], "reach_fail": [], "reach_limit_body_in_labware": [], "configs": 0}
    by_place_tool = {}
    ax_x = P.L_AXIS_X

    def run(hp, label, skip=lambda hn, it: False):
        res["configs"] += 1
        return check_config(scene, head, hp, label, skip)
    # ---- transit: every axis over every place, shelf slot, hand-off and bay; three tool pairs with tips; corners of travel
    pairs = [("M300", "S1000"), ("SYR", "M10"), ("M50", "S10")]
    pts = [(n, pl["c"]) for n, pl in PLACES.items()] + [(s_, SLOTS[s_]) for s_ in SLOTS]
    for n, (x, y) in pts:
        for ax in ("carriage_A", "carriage_B", "gripper"):
            xc, yf = x - ax_x[ax], y - (GRIP_Y if ax == "gripper" else AY)
            xc, yf = min(max(xc, XC0), XC1), min(max(yf, YF0), YF1)
            for tA, tB in pairs:
                res["transit"] += run(head.parts(xc, yf, toolA=tA, toolB=tB), f"transit {ax} over {n}, A {tA} B {tB} (tips on)")
    for xc in (XC0, XC1):
        for yf in (YF0, YF1):
            for tA, tB in pairs:
                res["transit"] += run(head.parts(xc, yf, toolA=tA, toolB=tB, tipsA=yf < 400, tipsB=yf < 400), f"travel corner xc {xc:g} yf {yf:g}, A {tA} B {tB}")
    # dock entry lane: the head enters the dock row at its right end (xc max) and travels along it at s 0, no tips
    for yf in (420.0, 480.0, 520.0, 560.0, YF1):
        res["transit"] += run(head.parts(XC1, yf, toolA="M300", toolB="S1000", tipsA=False, tipsB=False), f"dock entry lane xc {XC1:g} yf {yf:g} (no tips)")
    for xc in np.linspace(XC0, XC1, 12):
        res["transit"] += run(head.parts(float(xc), DOCK_Y - AY, toolA="M300", toolB="S1000", tipsA=False, tipsB=False),
                              f"along the dock row xc {xc:.0f} (no tips)")
    log("transit configs done", res["configs"])
    # ---- pipettes lowered at every place they serve, on each carriage; the other carriage holds the 30-300 eight-channel with tips
    reach_lim = {}
    for n, key, ptsxy, s_t, tips, what, excl in pipette_targets():
        for ax, other in (("carriage_A", "carriage_B"), ("carriage_B", "carriage_A")):
            ok_all, worst = True, []
            for (x, y) in ptsxy:
                xc, yf = x - ax_x[ax], y - AY
                if not in_travel(xc, yf):
                    res["reach_fail"].append({"place": n, "tool": key, "carriage": ax, "point": [x, y], "xc": round(xc, 1), "yf": round(yf, 1)})
                    ok_all = False
                    continue
                s_use = min(s_t, P.L_Z_TRAVEL)
                if s_t > P.L_Z_TRAVEL + 1e-6:
                    res["reach_fail"].append({"place": n, "tool": key, "carriage": ax, "depth_short_mm": round(s_t - P.L_Z_TRAVEL, 1)})
                    ok_all = False
                for frac in (0.5, 0.85, 1.0):
                    s = s_use * frac
                    kw = dict(sA=s, sB=0.0, toolA=key, toolB="M300", tipsA=tips) if ax == "carriage_A" else \
                        dict(sA=0.0, sB=s, toolA="M300", toolB=key, tipsB=tips)

                    def skip(hn, it, n=n, ax=ax, excl=excl, frac=frac):
                        return (it.get("place") == n and it["cls"] == "labware" and hn.startswith(ax + ":tool:") and hn.split(":")[-1] in excl)
                    h = run(head.parts(xc, yf, **kw), f"{key} on {ax} lowered at {n} ({what}) s {s:.0f} at ({x:.0f}, {y:.0f})", skip)
                    lim = [q for q in h if q["place"] == n and q["cls"] == "labware" and q["head_part"].startswith(ax + ":tool:")]
                    h = [q for q in h if q not in lim]
                    res["reach_limit_body_in_labware"] += lim
                    res["lowered"] += h
                    worst += h
                    if lim:
                        reach_lim.setdefault((n, key), set()).add(round(frac, 2))
            rl = reach_lim.get((n, key))
            by_place_tool.setdefault(n, {})[f"{key}@{ax}"] = {"reached": ok_all and not rl, "interferences": len(worst), "s_target": round(s_t, 1),
                                                              "what": what, "reach_limit": (f"tool body meets the target labware at {min(rl) * 100:.0f} % of the "
                                                                                            "stroke to the floor: cannot reach the floor") if rl else None}
    log("pipette configs done", res["configs"])
    # ---- gripper at every place it serves (fingers open, other carriages up with tools and tips)
    grip = gripper_targets(zs_shaker, plan)
    for n, ((x, y), zf, what) in grip.items():
        xc, yf = x - ax_x["gripper"], y - GRIP_Y
        s_t = grip_s(zf)
        if not in_travel(xc, yf) or s_t > P.L_Z_TRAVEL + 1e-6 or s_t < 0:
            res["reach_fail"].append({"place": n, "tool": "gripper", "xc": round(xc, 1), "yf": round(yf, 1), "s": round(s_t, 1)})
        ok = 0
        for frac in (0.5, 0.85, 1.0):
            h = run(head.parts(min(max(xc, XC0), XC1), min(max(yf, YF0), YF1), sG=min(max(s_t, 0), P.L_Z_TRAVEL) * frac, toolA="M300", toolB="S1000"),
                    f"gripper at {n} ({what}) s {s_t * frac:.0f}")
            res["gripper"] += h
            ok += len(h)
        by_place_tool.setdefault(n, {})["gripper"] = {"reached": in_travel(xc, yf) and 0 <= s_t <= P.L_Z_TRAVEL, "interferences": ok,
                                                      "s_target": round(s_t, 1), "what": what}
    log("gripper configs done", res["configs"])
    # ---- dock: drop-off into an empty bay and pick-up from an occupied bay, each bay, each carriage; all other bays full
    sd = P.L_DOCK_S
    for i in range(DOCK_N):
        key = DOCK_TOOLS_FULL[i]
        bay = f"BAY{i + 1}"
        for ax, other in (("carriage_A", "carriage_B"), ("carriage_B", "carriage_A")):
            xc, yf = DOCK_X[i] - ax_x[ax], DOCK_Y - AY
            if not in_travel(xc, yf):
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
                            return True                       # (the bay is empty for a drop-off: the scene's parked tool there is ignored)
                        if "release_post" in it["name"] and leaf in ("roller", "latch") and s >= sd - 7.0:
                            return True                       # the ramp opens the latch by design over the last 7 mm
                        if mode == "pick-up" and it["group"] == "parked_tool" and leaf in ("pins", "bracket", "latch", "spring", "pusher_rod") and \
                                it.get("leaf") in ("balls", "stud", "push_rod", "tool_plate", "magnets", "eject_pin", "slider") and s >= sd - 7.0:
                            return True                       # coupling halves meeting (balls into pin pairs, stud through the open latch)
                        return False
                    h = run(head.parts(xc, yf, **kw), f"dock {mode} {bay} by {ax} ({key}) s {s:.0f}", skip)
                    res["dock"] += h
    log("dock configs done", res["configs"])
    # ---- eject over-travel over A1, every tool on both carriages
    cx, cy = PLACES["A1"]["c"]
    for key in PIPETTES:
        for ax in ("carriage_A", "carriage_B"):
            xc, yf = cx - ax_x[ax], cy - AY
            kw = dict(sA=-P.Z_EJECT_TRAVEL, toolA=key, toolB="M300") if ax == "carriage_A" else dict(sB=-P.Z_EJECT_TRAVEL, toolA="M300", toolB=key)
            res["eject"] += run(head.parts(xc, yf, **kw), f"eject over-travel {key} on {ax} at A1")
    log("eject configs done", res["configs"])
    return res, by_place_tool


def static_check(items):
    """cross-group overlaps of the static scene (each group's own internal fit is its module's check)"""
    sc = Scene(items)
    hits = []
    designed = [("deck", "station"), ("deck", "frame")]
    for i, a in enumerate(items):
        idx = sc.candidates(sc.bb[i])
        for j in idx:
            if j <= i:
                continue
            b = items[j]
            if a["group"] == b["group"]:
                continue
            v = overlap_volume(a["solid"], b["solid"])
            if v > 1.0 or v < 0:
                hits.append({"a": a["name"], "b": b["name"], "mm3": round(v, 1)})
    return hits


def dock_clearance(head):
    """the pass 2 requirement: with a tool seated in its bay, everything the head carries at s 0 (no tips) clears the parked tools'
    tops and the release posts.  Evaluated for the pass 1 numbers (gantry 430, stroke 230) and for these."""
    out = {}
    pk = [it for it in dock_items(DOCK_TOOLS_FULL) if it["group"] in ("parked_tool",) or "release_post" in it["name"]]
    top_tool = max(it["solid"].BoundingBox().zmax for it in pk if it["group"] == "parked_tool")
    top_post = max(it["solid"].BoundingBox().zmax for it in pk if "release_post" in it["name"])
    hp = head.parts(0.0, 0.0, toolA="M300", toolB="S1000", tipsA=False, tipsB=False)
    low_noz, _ = lowest_point(head, [h for h in hp if h[0].startswith(("carriage_A:tool", "carriage_B:tool"))])
    low_g, _ = lowest_point(head, hp, "gripper")
    for nm, zb, trav in (("pass 1 (gantry 430, stroke 230)", 430.0, 230.0), ("pass 2 (this model)", ZB, P.L_Z_TRAVEL)):
        d = ZB - zb
        sd = trav - 3.0
        lift = (P.L_DOCK_S - sd)                          # parked tools sit this much higher with the shorter stroke
        out[nm] = {"parked_tool_top_z": round(top_tool - d + lift, 1), "release_post_top_z": round(top_post - d + lift, 1),
                   "head_nozzles_at_s0_z (no tips)": round(low_noz - d, 1), "gripper_fingertips_at_s0_z": round(low_g - d, 1),
                   "clearance_mm": round(min(low_noz, low_g) - d - max(top_tool, top_post) + d - lift, 1)}
    out["rule"] = ("a tool parked at the bottom of the stroke hangs 289 mm (Hn) below its plate; while one carriage works in a bay, the "
                   "other carriage (tool up, NO TIPS) and the gripper pass over the neighbouring bays: their lowest points must clear the "
                   "parked tools and the release posts by >= 8 mm")
    out["consequence"] = ("pass 1 fails by about 90 mm: the other carriage's tool and the gripper fingers run into the neighbouring parked "
                          "tools and posts.  Pass 2: Z stroke 345 and gantry +115 (beam mid z 545): nothing on the deck changes (same reach, "
                          "same lowest fixed point z 190), the machine is 115 mm taller, the Z axes 115 mm longer, M1 grows (lever) and an "
                          "exchange takes about 51 s instead of 26 s")
    out["planner_rules"] = ["no tips on EITHER carriage while the head is over the dock row (eject or park the idle carriage's tips first)",
                            "the dock row is entered and left at s 0 on all three axes; inside it only the exchanging carriage moves in Z"]
    return out


# =====================================================================================================
# 8. reports
# =====================================================================================================
EXT_KG_M = {"2040": 0.49, "2020": 0.26}      # kg/m, slot-6 profile class (ASSUMED catalogue value)


def mass_of(items):
    m = {}
    for it in items:
        if it["cls"] in ("labware", "reserved") or it["group"] in ("parked_tool", "station") or "canister" in it["name"] or "crate" in it["name"]:
            continue
        bb = it["solid"].BoundingBox()
        if it["name"].startswith("EXT-"):
            dims = sorted([bb.xlen, bb.ylen, bb.zlen])
            mm = EXT_KG_M["2040" if dims[1] > 30 else "2020"] * dims[2] / 1000.0
        else:
            mm = it["solid"].Volume() * RHO.get(it.get("mat", "al"), RHO["al"])
        m[it["group"]] = m.get(it["group"], 0.0) + mm
    return {k: round(v, 2) for k, v in m.items()}


def deck_positions(by_place_tool, plan):
    std = DKK.module_standard()
    tags = {n: k + 1 for k, n in enumerate(sorted(PLACES, key=lambda q: ("FABCDG".index(q[0]), int(q[1]))))}
    rules = DKK.k2_rules()
    lim = {}
    for n, pl in PLACES.items():
        r, c = n[0], int(n[1])
        rows = list(ROWS)
        front = f"{rows[rows.index(r) - 1]}{c}" if rows.index(r) > 0 else None
        l1 = 150.0
        if front and PLACES[front]["work"]:
            l1 = 51.0 if PLACES[front]["work"].startswith("deep") else 72.0
        for cc in (c - 1, c + 1):
            nb = f"{r}{cc}"
            if nb in PLACES and PLACES[nb]["work"] == "deep":
                l1 = min(l1, 64.0)
        lim[n] = l1
    weigh = {"B1": "WN-1 (labware)", "F2": "balance (reagents by weight)", "B2": "reserved (WN-M on a 3 kg cell, rev K)"}
    out = {}
    for n, pl in PLACES.items():
        bt = by_place_tool.get(n, {})
        reach = sorted({k.split("@")[0] for k, v in bt.items() if v["reached"] and v["interferences"] == 0})
        blocked = sorted({k for k, v in bt.items() if v["interferences"]})
        out[n] = {"kind": pl["kind"], "centre_xy": list(pl["c"]), "work": pl["work"], "height_mm": pl["h"],
                  "height_limit_mm (K2 rules R1/R2, rev K caps)": lim[n], "within_limit": pl["h"] <= lim[n] + 1e-6,
                  "reaching_tools (zero interference)": reach, "tools_with_interference": blocked,
                  "weighable": weigh.get(n, False), "deck_tag": tags[n],
                  "connector": "fitted" if (n in P.L_CONNECTOR_PLACES or (n == "F4")) else "cable route (notch) only"}
    out["_rules"] = rules["rules"]
    return out


def tube_positions():
    rep = HL.reach_into_tubes()
    racks = {"F1": ["15mL (CTB-2 F4, ATR-1 F1; recessed 60)", "50mL (CTB-2 F4, ATR-1 F1; recessed 60)"],
             "F3": ["1.5mL (CTB-1 F3, ATR-2 A2)"], "F4": ["5mL (CTB-2 F4, ATR-2 A2)", "15mL (CTB-2 F4, ATR-1 F1; recessed 60)", "50mL (CTB-2 F4, ATR-1 F1; recessed 60)"],
             "A2": ["1.5mL (CTB-1 F3, ATR-2 A2)", "5mL (CTB-2 F4, ATR-2 A2)"]}
    out = {}
    for n, rk in racks.items():
        out[n] = {"rack": PLACES[n]["kind"], "per_tool": {}}
        for k, t in rep["tools"].items():
            if HL.TOOLS[k]["eight"]:
                continue
            out[n]["per_tool"][k] = {r: t["racks"][r] for r in rk}
    out["_basis"] = rep["basis"]
    return out


def pipettes_report(by_place_tool):
    rng = {"M10": (0.5, 10), "M50": (5, 50), "M300": (30, 300), "S10": (0.5, 10), "S100": (10, 100), "S1000": (100, 1000)}
    hr = json.loads((LH1 / "head_l" / "out" / "report.json").read_text())
    out = {}
    for k, t in HL.TOOLS.items():
        places = sorted({n for n, d in by_place_tool.items() for kk, v in d.items() if kk.startswith(k + "@") and v["reached"] and v["interferences"] == 0})
        out[k] = {"name": t["name"], "range_uL": rng.get(k), "channels": 8 if t["eight"] else 1, "tip": t["tip"], "Hn_mm": t["Hn"],
                  "nozzle_z_at_top": round(nozzle_top(k), 1), "nozzle_z_at_full_stroke": round(nozzle_top(k) - P.L_Z_TRAVEL, 1),
                  "tip_end_z_at_full_stroke": round(nozzle_top(k) - P.L_Z_TRAVEL - (P.TIP_BELOW_NOZZLE[t["tip"]] if t["tip"] else 0.0), 1),
                  "places_reached_clean": places, "first_build": k not in ("S100",), "mass": hr["tool_masses"][k],
                  "dock_bay": (DOCK_TOOLS.index(k) + 1) if k in DOCK_TOOLS else "7 (reserved)" if k == "S100" else None}
    return out


def headroom_l(res, zc, plan, travel, masses):
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
                     "ratio": round(prov / req, 2) if req and prov else None, "spare": round(prov - req, 2) if req is not None and prov is not None else None,
                     "rule": rule, "meets_rule": ok, "note": note})
    k = json.loads((LH1 / "deck_k" / "out" / "report.json").read_text())["headroom"]["table"]
    keep = [r for r in k if r["kind"] == "consumable" and r["capacity"] not in ("tips held on deck and shelf", "pipette dock bays", "tip waste",
                                                                               "edge hand-off positions")]
    for r in keep:
        add(r["kind"], r["capacity"], r["required_largest_known"], r["provided"], r["unit"], r["note"] + " (rev K K2 figure, unchanged)")
    tip_req = sum(TIP_DEMAND_L.values())
    tip_prov = sum(r["capacity_tips"] for r in plan["per_type"].values())
    add("consumable", "tips held on deck and shelf (wafers, per type +2)", tip_req, tip_prov if plan["fits"] else
        tip_prov - sum(96 * int(o["what"].split(" x ")[1].split()[0]) for o in plan["does_not_fit"] if o["kind"].startswith("tower")) if plan["fits"] else 0, "tips",
        "item 4 demand; " + ("fits" if plan["fits"] else f"DOES NOT FIT: {len(plan['does_not_fit'])} slot(s) short (tip_supply)"))
    w = plan["tip_waste"]
    add("fixed", "tip waste crate (tips)", w["requirement_tips"], w["chosen_holds_tips"], "tips",
        f"1.5 x rule applied (waste capacity); 20 L crate of rev K holds {w['rev_K_20L_crate_holds_tips']} = {w['rev_K_20L_ratio']} x: replaced")
    add("consumable", "pipette dock bays", 6, 8, "bays", "five pipettes + syringe tool, S100 in bay 7, bay 8 cable bay")
    add("consumable", "edge hand-off positions", 2, 2, "places", "LOWER than +2: ADR-010 minimum; a third would cost a shelf slot")
    add("fixed", "Z clearance over the tallest deck item (head lowest fixed point)", 33.0, round(zc["deck_clearance_mm"], 1), "mm",
        f"head lowest fixed z {zc['head_lowest_fixed_z']:.0f} vs tallest deck item z {zc['tallest_deck_item_z']:.0f} ({zc['tallest_deck_item']})")
    add("fixed", "Z clearance over the shelf towers", 33.0, round(zc["shelf_clearance_mm"], 1), "mm",
        "LOWER: towers at the rev K 150 mm cap; served by the gripper only; named lower figure (a 127 mm cap costs 2 more tower slots)")
    add("fixed", "Z clearance over the dock (no tips, release posts)", 8.0, round(zc["dock_clearance_mm"], 1), "mm",
        "requirement 8 mm set here (pass 2 dock rule); 1.5 x = 12")
    for ax, d in travel.items():
        add("fixed", f"{ax} travel end margin beyond the outermost target (min of both ends)", 10.0, d["min_end_margin_mm"], "mm",
            "required 10 mm (ASSUMED: overshoot + end switch); the rule cannot mean 1.5 x the length (rev K note)")
    hr = json.loads((LH1 / "head_l" / "out" / "report.json").read_text())
    m1 = hr["M1_150N"]
    f_lim = 150.0 * 0.4 / m1["sum_horizontal_mm"]
    add("fixed", "frame lateral load: pick-up force at which M1 (horizontal) reaches the 0.4 mm pass line", 120.0, round(f_lim), "N",
        f"M1 horizontal {m1['sum_horizontal_mm']} mm at 150 N (lever-corrected for the raised gantry); 1.5 x 120 = 180 N gives "
        f"{m1['sum_horizontal_mm'] * 180 / 150:.3f} mm")
    add("fixed", "tool mass vs carriage limit (heaviest tool)", max(v["total_kg"] for v in hr["tool_masses"].values()), P.TOOL_MAX_MASS, "kg",
        "LOWER than 1.5 x for the eight-channel tools (1.30 / 1.00 = 1.3): the limit is the study's, set by M1")
    add("fixed", "extraction air (ideal mixing, all open)", 2.12, 30.0, "m3/h", "rev K figure; the K2 volume grows 115 mm in height (+11 %)")
    add("fixed", "24 V and TEC supplies", None, None, "W", "electrical owner: 1.5 x the fitted load")
    return {"rule": "ADR-009 as adjusted (ADR-010): fixed capacities (travel, clearance, frame load, power, cooling) >= 1.5 x; consumable "
                    "places >= largest need + 2 spare of each kind; lower figures named", "table": rows,
            "fails_or_lower": [r["capacity"] for r in rows if r["meets_rule"] is False]}


def travel_report(res):
    """target ranges vs travel; clearance to the frame at the travel ends (head at each end, swept edge to the nearest frame part)"""
    xs, ys = [], []
    ax_x = P.L_AXIS_X
    for n, key, pts, s_t, tips, what, excl in pipette_targets():
        for x, y in pts:
            xs += [x - ax_x["carriage_A"], x - ax_x["carriage_B"]]
            ys.append(y - AY)
    xs_g = [x - ax_x["gripper"] for n, ((x, y), zf, w) in gripper_targets(0.0, tip_supply_plan_l()).items()]
    ys_g = [y - GRIP_Y for n, ((x, y), zf, w) in gripper_targets(0.0, tip_supply_plan_l()).items()]
    xs_d = [x - ax_x[a] for x in DOCK_X for a in ("carriage_A", "carriage_B")]
    ys_d = [DOCK_Y - AY]
    need_x = (min(xs + xs_g + xs_d), max(xs + xs_g + xs_d))
    need_y = (min(ys + ys_g + ys_d), max(ys + ys_g + ys_d))
    # note: a pipette target need only be reached by ONE carriage; the both-carriage need is reported separately
    one_x = []
    for n, key, pts, s_t, tips, what, excl in pipette_targets():
        for x, y in pts:
            c = [x - ax_x["carriage_A"], x - ax_x["carriage_B"]]
            one_x.append(c)
    x_lo_one = max(min(c) for c in one_x)
    out = {"X": {"travel_xc": [XC0, XC1], "travel_mm": XC1 - XC0, "needed_both_carriages_every_target": [round(v, 1) for v in need_x],
                 "end_margins_mm": [round(need_x[0] - XC0, 1), round(XC1 - need_x[1], 1)],
                 "min_end_margin_mm": round(min(need_x[0] - XC0, XC1 - need_x[1]), 1),
                 "driving_targets": "left: carriage B at bay 1 / gripper at column 1; right: gripper at the outer shelf column x 540, carriage A at bay 8",
                 "clearance_at_ends_mm": X_MARGIN, "clearance_basis": "swept head edge to the Y-carriage / rail-block inner faces (rev B rule)"},
           "Y": {"travel_yf": [YF0, YF1], "travel_mm": YF1 - YF0, "needed": [round(v, 1) for v in need_y],
                 "end_margins_mm": [round(need_y[0] - YF0, 1), round(YF1 - need_y[1], 1)],
                 "min_end_margin_mm": round(min(need_y[0] - YF0, YF1 - need_y[1]), 1),
                 "driving_targets": "front: single channels at the front tubes of row F; back: the dock bays and the shelf slot behind G",
                 "clearance_front_mm (head front to the door)": round((YF0 + HEAD_Y[0]) - (FY0 - 5), 1),
                 "clearance_back_mm (Y carriage to the back cross member)": round((FY1 - 40) - (YF1 + YC["dy"][1]), 1)},
           "Z": {"travel_mm": P.L_Z_TRAVEL, "eject_over_travel_mm": P.Z_EJECT_TRAVEL, "dock_seat_s": P.L_DOCK_S}}
    return out


def z_clearance(head, static):
    hp = head.parts(0.0, 0.0, toolA="M300", toolB="S1000")
    low_fixed, who = lowest_point(head, hp, "fixed")
    low_tips, who_t = lowest_point(head, hp)
    deck = [it for it in static if it["group"] in ("station",) and it["solid"].BoundingBox().zmax > 0]
    tall = max(deck, key=lambda it: it["solid"].BoundingBox().zmax)
    shelf = [it for it in static if it["group"] == "shelf"]
    tall_s = max(shelf, key=lambda it: it["solid"].BoundingBox().zmax)
    dock = [it for it in static if it["group"] in ("parked_tool", "dock")]
    tall_d = max(dock, key=lambda it: it["solid"].BoundingBox().zmax)
    hp0 = head.parts(0.0, 0.0, toolA="M300", toolB="S1000", tipsA=False, tipsB=False)
    low_notips, _ = lowest_point(head, [h for h in hp0 if not h[0].startswith("fixed")])
    return {"head_lowest_fixed_z": round(low_fixed, 1), "lowest_fixed_part": who, "head_lowest_with_tips_z": round(low_tips, 1), "lowest_part": who_t,
            "tallest_deck_item_z": round(tall["solid"].BoundingBox().zmax, 1), "tallest_deck_item": tall["name"],
            "deck_clearance_mm": round(low_fixed - tall["solid"].BoundingBox().zmax, 1),
            "tallest_shelf_item_z": round(tall_s["solid"].BoundingBox().zmax, 1), "tallest_shelf_item": tall_s["name"],
            "shelf_clearance_mm": round(low_fixed - tall_s["solid"].BoundingBox().zmax, 1),
            "tallest_dock_item_z": round(tall_d["solid"].BoundingBox().zmax, 1), "tallest_dock_item": tall_d["name"],
            "head_lowest_moving_part_no_tips_z": round(low_notips, 1),
            "dock_clearance_mm": round(low_notips - tall_d["solid"].BoundingBox().zmax, 1),
            "tips_at_transit": "lowest tip end at s 0 is z %.0f: tips clear every deck and shelf item; over the dock row tips are not allowed" % low_tips}


def envelope(static):
    bb = cq.Compound.makeCompound([it["solid"] for it in static if it["group"] in ("frame", "enclosure", "drawer")]).BoundingBox()
    return {"x": [round(bb.xmin, 1), round(bb.xmax, 1)], "y": [round(bb.ymin, 1), round(bb.ymax, 1)], "z": [round(bb.zmin, 1), round(bb.zmax, 1)],
            "W_D_H_mm": [round(bb.xlen), round(bb.ylen), round(bb.zlen)],
            "note": "frame, enclosure panels, feet and drawer; the extraction fan behind the back panel adds %.0f mm of depth" % 150.0}


def scene_static(plan, tools=DOCK_TOOLS_FULL):
    st, zs = station_items()
    items = (frame_items() + y_drive_items() + enclosure_items() + drawer_items() + deck_items() + st + shelf_items(plan) + dock_items(tools)
             + control_bay_items())
    return items, zs


def write_parts(static, gantry, head_rep):
    rows = []
    seen = {}
    for it in static + gantry:
        if it["cls"] in ("labware", "reserved") or it["group"] == "parked_tool":
            continue
        nm = it.get("part") or it["name"]
        base = nm.split("_y")[0] if nm.startswith(("DSB-L", "EXT-deck_post", "EXT-base_cross", "EXT-dock")) else nm
        bb = it["solid"].BoundingBox()
        key = (base, round(bb.xlen), round(bb.ylen), round(bb.zlen))
        if key in seen:
            seen[key]["qty"] += 1
            continue
        rho = RHO.get(it.get("mat", "al"), RHO["al"])
        proc = ("bought" if it["cls"] == "bought" else "cut_to_length" if base.startswith(("EXT", "SQT", "DSB", "TUBE")) else
                "sheet_laser" if base.startswith(("DK-L", "ENC", "TSS", "DRW", "CB-L")) else "cnc_or_print")
        r = {"module": "machine_l", "part": base, "group": it["group"], "kind": "bought" if it["cls"] == "bought" else "custom",
             "process": proc, "material": it.get("mat", "al"), "envelope_mm": f"{bb.xlen:.0f} x {bb.ylen:.0f} x {bb.zlen:.0f}", "qty": 1,
             "mass_kg_each": round((EXT_KG_M["2040" if sorted([bb.xlen, bb.ylen, bb.zlen])[1] > 30 else "2020"] * max(bb.xlen, bb.ylen, bb.zlen) / 1000.0)
                                   if base.startswith("EXT-") else it["solid"].Volume() * rho, 3)}
        seen[key] = r
        rows.append(r)
    with open(OUT / "machine_l_parts.csv", "w", newline="") as fh:
        w = csv.DictWriter(fh, fieldnames=list(rows[0]))
        w.writeheader()
        w.writerows(rows)
    return rows


def export_steps(static, gantry, head, pose):
    for it in static:
        if it["name"] in ("DK-L1_deck_front", "DK-L2_deck_back", "TSS-L_tip_supply_shelf", "ENC-L-SR_side_right", "ENC-L-TOP_roof_panel",
                          "ENC-L-RB_roof_blank_FFU", "DOCK_base", "TW-L_tip_chute_100x100", "DRW-L_under_base_drawer") or it["name"].startswith(("DOCK_bay0_", "A1_TW-L")):
            cq.exporters.export(cq.Workplane().add(it["solid"]), str(OUT / f"{it['name']}.step"))
            if it["name"].startswith(("DK-L", "ENC-L-SR", "ENC-L-TOP", "TSS-L")):
                cq.exporters.export(cq.Workplane().add(it["solid"]).faces("<Z" if not it["name"].startswith("ENC-L-SR") else "<X"),
                                    str(OUT / f"{it['name']}.dxf"))
    asm = cq.Assembly(name="lh1_machine_rev_l")
    for it in static + gantry:
        asm.add(it["solid"], name=it["name"].replace(":", "__"))
    for hn, v, off in head.parts(**pose):
        asm.add(v.translate(V(*off)), name="head__" + hn.replace(":", "__"))
    asm.save(str(OUT / "machine_l_assembly.step"))


# =====================================================================================================
# 9. main
# =====================================================================================================
def main(quick=False):
    OUT.mkdir(parents=True, exist_ok=True)
    plan = tip_supply_plan_l()
    static, zs = scene_static(plan)
    log("static items", len(static))
    valid = {it["name"]: it["solid"].isValid() for it in static}
    head = Head()
    log("head cached")
    st_hits = static_check(static)
    log("static check", len(st_hits))
    # gantry (Y group) vs static at the front, middle and back of travel; head vs gantry once
    g_hits = []
    for yf in (YF0, (YF0 + YF1) / 2, YF1):
        g = gantry_items(yf)
        sc = Scene(static)
        for it in g:
            for i in sc.candidates(np.array(sc.bb[0]) * 0 + np.array([*(lambda b: (b.xmin, b.xmax, b.ymin, b.ymax, b.zmin, b.zmax))(it["solid"].BoundingBox())])):
                o = static[i]
                if ("Y_block" in it["name"] and "Y_rail" in o["name"]):
                    continue
                v = overlap_volume(it["solid"], o["solid"])
                if v > 1.0:
                    g_hits.append({"yf": yf, "a": it["name"], "b": o["name"], "mm3": round(v, 1)})
    gsc = Scene(gantry_items(0.0))
    hg = []
    for xc in (XC0, 0.0, XC1):
        for s in (0.0, P.L_Z_TRAVEL, -P.Z_EJECT_TRAVEL):
            hg += check_config(gsc, head, head.parts(xc, 0.0, sA=s, sB=s, sG=max(s, 0.0)), f"head vs gantry xc {xc:g} s {s:g}",
                               skip=lambda hn, it: hn.split(":")[-1].startswith("xblock") and "X_rail" in it["name"])
    log("gantry checks", len(g_hits), len(hg))
    scene = Scene(static)
    res, by_place_tool = run_checks(scene, head, zs, plan)
    log("checks done", res["configs"])
    zc = z_clearance(head, static)
    dclr = dock_clearance(head)
    trav = travel_report(res)
    env = envelope(static)
    gm = gantry_items((YF0 + YF1) / 2)
    masses = mass_of(static + gm)
    hrep = json.loads((LH1 / "head_l" / "out" / "report.json").read_text())
    m_head = hrep["mass"]["head_kg"]
    masses_total = round(sum(masses.values()) + m_head + 6 * 0.9, 1)
    head_room = headroom_l(res, zc, plan, {"X carriage": trav["X"], "Y gantry": trav["Y"]}, masses)
    real = res["transit"] + res["lowered"] + res["gripper"] + res["dock"] + res["eject"]
    # group the hits by (head part leaf, item) for the list of fixes
    groups = {}
    for h in real:
        k = (h["head_part"].split(":")[0] + ":" + h["head_part"].split(":")[-1], h["item"])
        g = groups.setdefault(k, {"head_part": k[0], "item": k[1], "n_configs": 0, "max_mm3": 0.0, "example": h["config"]})
        g["n_configs"] += 1
        g["max_mm3"] = max(g["max_mm3"], h["mm3"])
    rep = {"module": "LH-1 revision L pass 2: K2 machine with the exchange head", "status": "CAD and calculation only; nothing built or measured",
           "frame": "global deck frame; head at carriage centre xc, beam front face yf, z = local + %g" % ZB,
           "parts_valid": all(valid.values()), "invalid": [k for k, v in valid.items() if not v],
           "frame_geometry": {"Y_rails_x": [round(XR_L, 1), round(XR_R, 1)], "X_beam_mm": round(BEAM["x1"] - BEAM["x0"], 1),
                              "X_beam": P.L_X_BEAM.replace("about 1170 mm (K2)", f"{BEAM['x1'] - BEAM['x0']:.0f} mm"),
                              "Y_rails_mm": round(Y_RAIL[1] - Y_RAIL[0], 1), "Y_rail_y": [round(v, 1) for v in Y_RAIL],
                              "side_beam_top_z": SB["z1"], "beam_mid_z": ZB, "base_top_z": BASE["z1"], "foot_h": FOOT_H, "roof_frame_z": ROOF_Z,
                              "frame_y": [FY0, FY1], "frame_x": [round(FX0, 1), round(FX1, 1)], "deck_x": list(DECK_X), "deck_y": list(DECK_Y),
                              "deck_supports": "40 x 10 Al flat bars on edge at y %s (row boundaries + 5 mm, clear of the connector notches), on 2020 posts" %
                                               [round(v) for v in DECK_BAR_Y],
                              "dock": {"bays_x": DOCK_X, "axis_y": DOCK_Y, "pitch": P.L_DOCK_PITCH, "seat_stroke": P.L_DOCK_S,
                                       "parked_plate_top_z": round(HL.dock_seat_top() + ZB, 1)},
                              "shelf": {"x": [SHELF["x0"], SHELF["x1"]], "y": [SHELF["y0"], SHELF["y1"]], "columns_x": P.L_SHELF_X},
                              "drawer": DRAWER, "chute": CHUTE},
           "envelope_mm": env, "mass_kg": {"by_group_modelled (extrusions at catalogue kg/m; deck stations, labware, crate and canisters excluded)": masses, "head_kg": m_head, "tools_6_kg (ASSUMED 0.9 each)": 5.4,
                                           "total_modelled_kg": masses_total,
                                           "note": "modelled solids at Al/steel/PC/PA12 densities; NOT included: deck modules (TC-1, shaker, thermal "
                                                   "modules, balance, about 15 kg), fasteners, cables, electronics and supplies (about 15-20 kg): ESTIMATE"},
           "interference": {"static_cross_group": st_hits, "gantry_vs_static": g_hits, "head_vs_gantry": hg,
                            "head_configs_checked": res["configs"], "count_real": len(real),
                            "by_kind": {k: len(res[k]) for k in ("transit", "lowered", "gripper", "dock", "eject")},
                            "grouped": sorted(groups.values(), key=lambda g: -g["n_configs"]), "all": real[:400],
                            "method": "bounding-box prefilter, OCC boolean common > 1 mm3; designed contacts excluded: tips/nozzles in the "
                                      "target place's labware, single-channel shafts in the target tube rack, the coupling halves meeting in "
                                      "a bay (balls/pins, stud/latch), the latch roller on the release-post ramp over its last 7 mm; Z "
                                      "sampled at 50, 85 and 100 % of each working stroke"},
           "reach_failures": res["reach_fail"], "reach_limits_body_in_target_labware": sorted({(q["config"].split(" lowered at ")[0], q["place"]) for q in res["reach_limit_body_in_labware"]}),
           "z_clearance": zc, "dock_clearance": dclr, "travel": trav, "tip_supply": plan}
    rep["deck_positions"] = deck_positions(by_place_tool, plan)
    rep["by_place_and_tool"] = by_place_tool
    rep["tube_positions"] = tube_positions()
    rep["pipettes"] = pipettes_report(by_place_tool)
    rep["headroom"] = head_room
    rep["plate_44mm_check"] = deep_plate_check(zs, head)
    rep["runtime_s"] = round(time.time() - T0, 1)
    (OUT / "report.json").write_text(json.dumps(rep, indent=1, default=str))
    parts = write_parts(static, gm, hrep)
    rep["parts_csv_rows"] = len(parts)
    pose = dict(xc=60.0, yf=(YF0 + YF1) / 2 - 60, sA=0.0, sB=120.0, sG=0.0, toolA="M300", toolB="S10")
    export_steps(static, gm, head, pose)
    log("exports done")
    rep["assembly_outputs"] = write_assembly_outputs(rep)
    (OUT / "report.json").write_text(json.dumps(rep, indent=1, default=str))
    return rep


# =====================================================================================================
# 11. assembly-level outputs (rev_l key, changes, proposed BOM and build parts)
# =====================================================================================================
def rev_l_block(rep):
    hr = json.loads((LH1 / "head_l" / "out" / "report.json").read_text())
    i = rep["interference"]
    return {
        "status": "PASS 2: the K2 machine in 3D with the exchange head (cad/lh1/machine_l) and the cradles (cad/lh1/head_l); CAD and calculation "
                  "only, nothing built or measured; every figure CALCULATED or ASSUMED as labelled in the module reports",
        "interference": {"head_configurations_checked": i["head_configs_checked"], "real_interferences": i["count_real"], "by_kind": i["by_kind"],
                         "static_cross_group": len(i["static_cross_group"]), "gantry_vs_static": len(i["gantry_vs_static"]),
                         "head_vs_gantry": len(i["head_vs_gantry"]), "reach_failures": len(rep["reach_failures"]),
                         "reach_limits (tool body meets the target labware before the floor)": rep["reach_limits_body_in_target_labware"],
                         "method": i["method"],
                         "found_and_fixed_in_pass_2": [
                             "dock: with a 230 mm stroke the other carriage's tool and the gripper fingers hit the parked tools and release posts "
                             "(clearance -104 mm): Z stroke 345, gantry +115 (beam mid z 545)",
                             "dock: the carriage plate (lowest fixed point z 190) sweeps any bay it crosses in Y: the dock row is entered at its "
                             "right end (bays x -296..250, carriage travel to 465) and travelled along X; bays moved behind G so the plate clears "
                             "the posts when the head works row G (axis y 472)",
                             "dock: bay pitch 78 (72 mm tool plates touched at 72); tools rest on cradle lugs, not on the tool plate (cradles are "
                             "wider than the plate); release-post column moved in front of the eight-channel manifolds; latch tail +6 mm",
                             "TC-1 lid guide strip 15 mm further out (single-channel cradles at B1/C1 column 1 reach x -208.5); deck to x -270",
                             "tip parking TPK-1 swapped with the D2 plate stack (the 86.6 mm TPK-1 stood where eight-channel cradles pass at C1 "
                             "column 12; K2 had assumed 50 mm)",
                             "G2/G3 frames recessed 7, G4 p1000 rack 75 (was 50), A2 rack 20 (was 15): eight-channel manifold cheeks at the next column",
                             "shaker at D4 sunk 40 mm on a tray (lid top was z 150: 40 mm under the carriage plate, below the 1.5 x rule; also takes "
                             "the 44 mm wash plate with a lid)",
                             "gripper fingers checked open (gap 92); the head_l model shows them part-closed"]},
        "deck_positions": rep["deck_positions"], "tube_positions": rep["tube_positions"], "pipettes": rep["pipettes"],
        "dock": {"bays": [{"bay": k + 1, "x": DOCK_X[k], "y": DOCK_Y, "first_build_tool": DOCK_TOOLS[k],
                           "note": "cable bay: reserved route + retractor space (pressure-sensing channel)" if k == 7 else ("S100 later" if k == 6 else "")}
                          for k in range(DOCK_N)],
                 "design": hr["dock_bay"], "seat_stroke_s": P.L_DOCK_S, "parked_plate_top_z": rep["frame_geometry"]["dock"]["parked_plate_top_z"],
                 "clearance": rep["dock_clearance"], "entry_rule": "enter and leave the dock row at its right end (xc >= 465), no tips on either carriage"},
        "tool_carriages": {"axes": P.L_AXIS_X, "coupling": hr["coupling"]["geometry"], "preload_N": hr["coupling"]["preload_N"],
                           "interface": hr["tool_interface"], "head_mass": hr["mass"], "head_envelope_mm": hr["envelope_mm"],
                           "interference_in_head": hr["interference"]["count"], "tool_masses": hr["tool_masses"], "cradles": hr["cradles"]},
        "exchange_time": hr["exchange"],
        "stiffness": {"M1_120N": {k: hr["M1_120N"][k] for k in ("sum_horizontal_mm", "sum_vertical_mm", "total_mm", "total_without_axial_screw_mm", "limit_mm", "horizontal_pass")},
                      "M1_150N": {k: hr["M1_150N"][k] for k in ("sum_horizontal_mm", "sum_vertical_mm", "total_mm", "total_without_axial_screw_mm", "limit_mm", "horizontal_pass")},
                      "rows_150N": hr["M1_150N"]["rows"], "coupling_cases": hr["coupling"]["cases"]},
        "dynamics": hr["dynamics"], "z_clearance": rep["z_clearance"], "travel": rep["travel"], "headroom": rep["headroom"],
        "frame": rep["frame_geometry"], "envelope_mm": rep["envelope_mm"], "mass_kg": rep["mass_kg"], "tip_supply": rep["tip_supply"],
        "plate_44mm_wash_plate": rep["plate_44mm_check"], "z_clearance_rule": "head lowest fixed point z 190 at every stroke",
        "planner_rules": rep["dock_clearance"]["planner_rules"] + [
            "single-channel 0.5-10 uL cannot reach the floor of a 44 mm deep-well plate (shaft wider than the well): use the eight-channel tools there",
            "single-channel 10-100 uL does not reach the bottom of 15/50 mL tubes (body in the tube): 100-1,000 uL tool",
            "K2 layout changes: tip parking at D2, plate stack at C2; recesses as listed"],
        "reports": {"machine": "cad/lh1/machine_l/out/report.json", "head": "cad/lh1/head_l/out/report.json"}}


def write_assembly_outputs(rep):
    import yaml
    ar = json.loads((AOUT / "report.json").read_text())
    before = {k: json.dumps(ar.get(k), sort_keys=True, default=str) for k in ("rev_h", "rev_k")}
    ar["rev_l"] = rev_l_block(rep)
    assert all(json.dumps(ar.get(k), sort_keys=True, default=str) == v for k, v in before.items())
    (AOUT / "report.json").write_text(json.dumps(ar, indent=1, default=str))
    (OUT / "rev_l.json").write_text(json.dumps(ar["rev_l"], indent=1, default=str))
    # ---- proposed BOM: rev K proposal minus the superseded frame and head rows, plus head_l and machine_l
    rows = list(csv.DictReader(open(AOUT / "lh1_bom_k_proposed.csv")))
    sup_f = ("EXT-", "TUBE", "ENC-", "B-MGN12", "YC_", "YCT_", "DP_", "NS-1", "NB-1", "RJ", "B-GT2_belt", "CBP", "VBP", "CBC", "TEP", "CEP", "PSD",
             "PSUP", "FT_", "YCS_", "YCHM", "ESY", "ESX", "XMB", "XIB", "YMB", "YIB", "B-levelling_foot", "B-cable_chain")
    keep_h = ("GR-", "B-MGN9", "B-TR8x2_LR", "B-TR8x2_flange", "B-NEMA11_28HS32", "B-GT2_16T", "B-silicone_pad", "B-grip_spring", "CAM-1")
    out, dropped = [], []
    for r in rows:
        m, pid = r["module"], r["part_id"]
        if m in ("head", "frame_gantry+head") and not pid.startswith(keep_h):
            dropped.append(pid)
            continue
        if m == "frame_gantry" and pid.startswith(sup_f):
            dropped.append(pid)
            continue
        if m == "frame_gantry":
            r["notes"] = (r["notes"] + "; " if r["notes"] else "") + "rev L: kept from rev K, quantity and size to be re-derived for the K2 frame"
        out.append(r)
    fields = list(rows[0])
    for r in csv.DictReader(open(LH1 / "head_l" / "out" / "head_l_parts.csv")):
        out.append({"module": "head_l", "part_id": r["part"], "name": r["part"], "kind": r["kind"], "process": r["process"], "material": r["material_or_spec"],
                    "thickness_or_stock": "", "envelope_mm": "", "qty": r["qty"], "mass_kg": "", "spec": "", "file": "cad/lh1/head_l/out", "notes": "rev L head"})
    for k, n in (("hand pipette, eight-channel 0.5-10 uL", 1), ("hand pipette, eight-channel 5-50 uL", 1), ("hand pipette, eight-channel 30-300 uL", 1),
                 ("hand pipette, single 0.5-10 uL", 1), ("hand pipette, single 100-1,000 uL", 1), ("hand pipette, single 10-100 uL (when samples come in tubes)", 0),
                 ("syringe dispense tool parts (if the rig test passes)", 1)):
        out.append({"module": "head_l", "part_id": "B-" + k.replace(" ", "_").replace(",", ""), "name": k, "kind": "bought", "process": "bought",
                    "material": "", "thickness_or_stock": "", "envelope_mm": "", "qty": n, "mass_kg": "", "spec": "", "file": "",
                    "notes": "ADR-010 first build"})
    for r in csv.DictReader(open(OUT / "machine_l_parts.csv")):
        out.append({"module": "machine_l", "part_id": r["part"], "name": r["part"], "kind": r["kind"], "process": r["process"], "material": r["material"],
                    "thickness_or_stock": "", "envelope_mm": r["envelope_mm"], "qty": r["qty"], "mass_kg": r["mass_kg_each"], "spec": "",
                    "file": "cad/lh1/machine_l/out", "notes": "rev L K2 frame/deck/dock/enclosure/drawer (" + r["group"] + ")"})
    with open(AOUT / "lh1_bom_l_proposed.csv", "w", newline="") as fh:
        w = csv.DictWriter(fh, fieldnames=fields)
        w.writeheader()
        w.writerows(out)
    # ---- build parts (machine steps added in front of the rig steps already there)
    old = yaml.safe_load((AOUT / "proposed_build_parts_l.yaml").read_text())
    rig_steps = {k: v for k, v in old["steps"].items() if k.startswith("BR-")}
    mrows = list(csv.DictReader(open(OUT / "machine_l_parts.csv")))
    steps = {}
    for code, grp in (("LH-L-B01_frame_and_feet", "frame"), ("LH-L-B02_gantry", "gantry"), ("LH-L-B03_deck_and_connectors", "deck"),
                      ("LH-L-B04_stations", "station"), ("LH-L-B05_shelf_and_hand-offs", "shelf"), ("LH-L-B06_dock", "dock"),
                      ("LH-L-B07_drawer_tip_crate_waste", "drawer"), ("LH-L-B08_enclosure", "enclosure")):
        steps[code] = [{"part": r["part"], "qty": int(r["qty"])} for r in mrows if r["group"] == grp]
    steps["LH-L-B09_head"] = [{"part": r["part"], "qty": int(r["qty"])} for r in csv.DictReader(open(LH1 / "head_l" / "out" / "head_l_parts.csv"))
                              if not r["part"].startswith(("CBL", "TPL", "PS-L"))]
    steps["LH-L-B10_tools"] = [{"part": r["part"], "qty": int(r["qty"])} for r in csv.DictReader(open(LH1 / "head_l" / "out" / "head_l_parts.csv"))
                               if r["part"].startswith(("CBL", "TPL", "PS-L"))]
    doc = {"source": "generated by cad/lh1/machine_l/build.py from machine_l_parts.csv and head_l_parts.csv (machine, rev L pass 2) and the rig steps "
                     "of the previous proposal (BR-*, unchanged); quantities equal the CAD lists",
           "machine_steps_note": "proposal for the build-document owner: one step per module group; the rev K machine steps are superseded",
           "steps": {**steps, **rig_steps}}
    (AOUT / "proposed_build_parts_l.yaml").write_text(yaml.safe_dump(doc, sort_keys=False, width=140))
    # ---- changes with counts before and after
    kr = json.loads((LH1 / "deck_k" / "out" / "report.json").read_text())
    krow = list(csv.DictReader(open(AOUT / "lh1_bom_k_proposed.csv")))
    hr = json.loads((LH1 / "head_l" / "out" / "report.json").read_text())
    counts = {
        "deck places (standard)": [15, len(PLACES)], "labware places (stacks of 4 count 4)": [kr["headroom"]["table"][0]["provided"], kr["headroom"]["table"][0]["provided"]],
        "pipettes on the machine": [3, 6], "syringe dispense tool": [0, 1], "Z axes on the head": [4, 3], "tool carriages": ["4 fixed tools", "2 universal"],
        "dock bays": [0, DOCK_N], "tip types / tip frames": [3, 4], "edge hand-offs": [0, 2], "shelf slots (gripper)": [5, len(SLOTS)],
        "X carriage travel mm": [P.TRAVEL_X, XC1 - XC0], "Y travel mm": [P.TRAVEL_Y, YF1 - YF0], "Z travel mm": [P.TRAVEL_Z, P.L_Z_TRAVEL],
        "X beam": ["80x40x4, 980 mm", f"100x50x5, {BEAM['x1'] - BEAM['x0']:.0f} mm"], "beam mid z": [P.X_BEAM_MID_Z, ZB],
        "envelope W x D x H mm": [kr["k2_layout"]["frame"]["envelope_was_mm"], rep["envelope_mm"]["W_D_H_mm"]],
        "head mass kg": [13.55, hr["mass"]["head_kg"]], "machine mass kg (modelled, see note)": ["about 88 (rev K estimate)", rep["mass_kg"]["total_modelled_kg"]],
        "BOM lines": [len(krow), len(out)], "BOM lines dropped (superseded frame/head)": [0, len(dropped)],
        "custom lines": [sum(1 for r in krow if r["kind"] == "custom"), sum(1 for r in out if r["kind"] == "custom")],
        "bought lines": [sum(1 for r in krow if r["kind"] == "bought"), sum(1 for r in out if r["kind"] == "bought")],
        "tip waste crate L": [20, round(CRATE["L"] * CRATE["W"] * CRATE["H"] / 1e6, 1)],
        "interferences in the whole-machine run": ["rev K run (old head)", rep["interference"]["count_real"]]}
    chg = {"revision": "L, pass 2 (brief items 1-5: K2 machine in 3D, head placed and checked, cradles, sizing, outputs)",
           "date": "2026-11-14", "status": "CAD and calculation only; nothing built or measured",
           "params_py": {"changed (rev L only; revisions H-K unaffected)": {"L_X_BEAM_MID_Z": "430 -> 545", "L_Z_TRAVEL": "230 -> 345",
                                                                           "L_NOZZLE_TOP_Z": "245/275 -> 360/390"},
                         "new": ["L_DOCK_S", "L_DOCK_PITCH", "L_DOCK_X0", "L_DOCK_Y", "L_XC_RANGE", "L_YF_RANGE", "L_COLS", "L_ROWS", "L_DECK_X",
                                 "L_DECK_Y", "L_SHELF_X", "L_HANDOFF", "L_MERGED_WINDOWS", "L_CONNECTOR_PLACES", "L_CONNECTOR_NOTCH"],
                         "revisions_H_K": "unchanged: no H-K module reads an L_ name; proved by rebuilding frame_gantry and deck_k with REV_L False "
                                          "and True and comparing their report hashes with the committed ones (machine_l report: hk_reproducibility)"},
           "new_module": "cad/lh1/machine_l", "head_l_changes": ["Z axes and carriage plate follow the stroke (CP 10 mm, rails 479 mm)",
                                                                  "real cradle bodies CBL-<tool> (+ straps, cheeks, slider, eject plunger, force collar) "
                                                                  "with dock lugs", "bay design: lugs on pins, post in front, latch tail +6 mm",
                                                                  "M1 lever-corrected for the raised gantry; dynamics with the K2 beam and rails",
                                                                  "exchange time with the dock at the bottom of the stroke"],
           "layout_changes_vs_K2": rev_l_block(rep)["interference"]["found_and_fixed_in_pass_2"],
           "counts_before_after (rev K -> rev L)": counts}
    try:                                              # keep the pass 1 record (committed file) under its own key
        import subprocess
        p1 = subprocess.run(["git", "show", "HEAD:cad/lh1/assembly/out/changes_rev_l.yaml"], capture_output=True, text=True, cwd=str(LH1.parent.parent)).stdout
        d1 = yaml.safe_load(p1) if p1 else None
        chg["pass_1 (committed record)"] = d1.get("pass_1 (committed record)", d1) if isinstance(d1, dict) else None
    except Exception:  # noqa: BLE001
        chg["pass_1 (committed record)"] = None
    (AOUT / "changes_rev_l.yaml").write_text(yaml.safe_dump(chg, sort_keys=False, width=140, allow_unicode=True))
    return {"bom_rows": len(out), "dropped": len(dropped), "build_steps": len(steps), "rig_steps": len(rig_steps)}


# =====================================================================================================
# 10. the taller wash plate (coordinator update: primary case for wash steps, then transfer to the 0.2 mL plate)
# =====================================================================================================
DEEP = {"square_2mL_44": dict(H=44.0, floor=2.5, depth=39.0, side=8.2, pyr=4.0, round=False, kg_empty=0.06),
        "round_1.2mL_44": dict(H=44.0, floor=2.5, depth=39.0, side=7.0, pyr=3.5, round=True, kg_empty=0.05)}   # ASSUMED classes: MEASURE


def _well_vol(w, h):
    a, hb = w["side"], w["pyr"]
    if w["round"]:
        r = a / 2
        if h <= hb:                                   # hemispherical-ish U bottom approximated as a cone
            return math.pi * (r * h / hb) ** 2 * h / 3
        return math.pi * r * r * hb / 3 + math.pi * r * r * (h - hb)
    if h <= hb:
        return (a * h / hb) ** 2 * h / 3
    return a * a * hb / 3 + a * a * (h - hb)


def deep_plate_check(zs_shaker, head):
    acc, g = 2.0, 9.81
    fills = {}
    for k, w in DEEP.items():
        rise = (acc / g) * w["side"] / 2
        fills[k] = {"well_ASSUMED": {kk: w[kk] for kk in ("depth", "side", "pyr", "round")},
                    "brim_uL": round(_well_vol(w, w["depth"])),
                    "max_fill_moved_by_gripper_uL": int(_well_vol(w, w["depth"] - 2.0 - rise)),
                    "max_fill_shaken_uL": int(_well_vol(w, w["depth"] - 6.0)),
                    "liquid_height_at_200uL_mm": round(next(h / 10 for h in range(1, 400) if _well_vol(w, h / 10) >= 200), 1),
                    "basis": "same rules as the 0.2 mL plate (rev K Y16): moved at 2 m/s2 = 2 mm + tilt rise; shaken = 6 mm freeboard"}
    pcr = DKK.plate_working_volume()
    H = DEEP["square_2mL_44"]["H"]
    top = SEAT + H
    # eight-channel reach to the floor at a plain place: the manifold (6 mm above the nozzle end) must stay above the plate top
    reach = {}
    for k in EIGHT:
        L = P.TIP_BELOW_NOZZLE[HL.TOOLS[k]["tip"]]
        nz = SEAT + DEEP["square_2mL_44"]["floor"] + 0.5 + L
        s = nozzle_top(k) - nz
        man = nz + HL.PIP[k]["nl"]
        reach[k] = {"tip": HL.TOOLS[k]["tip"], "tip_length_mm": L, "nozzle_z_at_floor": round(nz, 1), "manifold_bottom_z": round(man, 1),
                    "plate_top_z": top, "stroke_s": round(s, 1), "within_travel": s <= P.L_Z_TRAVEL,
                    "reaches_floor": man >= top + 1.0, "basis": "manifold bottom = nozzle end + nozzle cone length (rev J class: 14 / 18 mm, ASSUMED)",
                    "short_by_mm": round(max(0.0, top + 1.0 - man), 1)}
    m = DK.mb1_geometry()
    car = (P.GRIPPER_FINGERTIP_Z_AT_TOP, )
    fz0 = ZB - 194.0
    fill_max = max(f["max_fill_moved_by_gripper_uL"] for f in fills.values())
    m_plate = DEEP["square_2mL_44"]["kg_empty"] + 96 * fill_max * 1e-6
    f_grip = m_plate * (g + acc) / (2 * 0.3)
    return {
        "case": "wash steps in a 96-well plate about 44 mm tall (0.5-2 mL deep-well class), then transfer to the 0.2 mL plate for thermal "
                "steps (coordinator: primary case)",
        "fill_limits": fills, "fill_limits_0.2mL_plate (rev K)": {k: pcr[k] for k in ("brim_volume_uL", "max_fill_moved_by_gripper_uL", "max_fill_shaken_uL")},
        "gripper": {"grip_height": "fingertips 2 mm above the seat on the full-height side wall: same as the 0.2 mL plate; the 113 mm fingers "
                                   "leave the gripper body %.0f mm above the plate top" % (98.0 - H),
                    "force": f"plate {DEEP['square_2mL_44']['kg_empty']} kg + 96 x {fill_max} uL = {m_plate:.2f} kg: needs about {f_grip:.1f} N "
                             f"(mu 0.3 per pad, 2 m/s2, ASSUMED) within GR-1's {P.GRIP_FORCE_N[0]:g}-{P.GRIP_FORCE_N[1]:g} N; payload limit {P.GRIP_MAX_PAYLOAD} kg: PASS",
                    "carried_clearance": f"carried at s 0 the plate bottom is at z {fz0 - 2:.0f}: {fz0 - 2 - 340:.0f} mm over the release posts, "
                                         f"{fz0 - 2 - 155:.0f} mm over the shelf towers: PASS"},
        "lid": "LD-1 (universal SBS lid, 9.5 mm, ceiling 1 mm above the rim) sits on the top frame if the deep-well plate has an SBS rim "
               "(most do; CONFIRM with the plate bought); lid top at z %.1f.  A deep-well plate is often sealed with a mat instead: the "
               "gripper cannot place a mat: stay with lids" % (top + 2.5),
        "magnet": {"MB-1": f"does NOT suit: MB-1 holds 2 mm magnets in rows between the 0.2 mL wells (5.4 mm cones at 9 mm pitch) with the "
                           f"magnet top {DK.MB['pellet_top']:g} mm above the well floor; square 8.2 mm (or round 7 mm) deep wells leave 0.8-2 mm "
                           "between wells: no room for the rows",
                   "needed": "a bought 96 deep-well magnet plate (ring or bar magnets around/under each well, SBS footprint, about 20-25 mm "
                             "tall, the plate sitting in it) on the standard place: B2 gets it as a swap insert (4 screws), or B4 holds it and "
                             "the low-elution magnet moves to a C-row place",
                   "pellet": "with the wash volumes the liquid stands %.1f mm (200 uL) above the floor in the square well: the magnet must pull "
                             "the pellet within the lowest 5 mm of the side wall (or to the floor ring) so beads are not lost high in the well; "
                             "bench rig: bead capture at 200 uL and at the elution volume" % fills["square_2mL_44"]["liquid_height_at_200uL_mm"]},
        "shaker": {"clamp": "the bought shaker clamps the SBS skirt (bottom 6-7 mm): deep-well plates share the footprint and skirt: PASS if "
                            "the maker lists deep-well plates (CONFIRM); its maximum load and speed for a 0.2 kg plate (CONFIRM)",
                   "height": f"plate seat z {zs_shaker:.0f}: plate top {zs_shaker + H:.0f}, with lid {zs_shaker + H + 2.5:.1f}: the head's lowest fixed "
                             f"point is z 190, so {190 - zs_shaker - H - 2.5:.1f} mm in transit over D4: "
                             + ("PASSES the 1.5 x clearance rule (<= 140) with the shaker sunk %.0f mm (pass 2 fix)" % SHAKER_RECESS
                                if zs_shaker + H + 2.5 <= 140.5 else "FAILS the 1.5 x clearance rule (needs <= 140)"),
                   "what_it_takes": "done in pass 2: shaker sunk 40 mm (unsunk it would top at z 177.5); otherwise recess the shaker about 40 mm (D4 is an edge place: a 150 x 108 cut-out in the deck and an under-deck "
                                    "frame at z -45; the shaker's own clamp stays above the deck) so the lidded deep-well plate tops at "
                                    "about z 140; or a lower bought heater-shaker class (about 80 mm body)"},
        "weighing_nest": "WN-1 (1 kg cell) takes a filled deep-well plate (%.2f kg) within its range; the K2 B1 entry already assumes this plate "
                         "(44 mm): PASS" % m_plate,
        "stacking": "the 51 mm stack cap (C4, D2: head rear parts behind deep work) takes ONE deep-well plate per place, not two: fresh "
                    "deep-well plates need their own places or the shelf outer column (gripper only, no cap below 150: 3 per slot)",
        "cold_plate": "CPB-1 (B3) is a block matched to the 0.2 mL wells: a deep-well plate does not seat in it.  Washes run at room "
                      "temperature, so nothing is needed; a cold deep-well step would need a flat-bottom adapter block (custom, CPB-2)",
        "tip_reach_eight_channel": reach,
        "z_clearance_transit": f"on any plain place the lidded plate tops at z {top + 2.5:.1f}, under the 51 mm R1 cap behind deep work only "
                               f"if not lidded ({top:.0f} - 5 = {H:.0f} mm above the seat <= 51): PASS; head lowest fixed point 190",
        "what_must_change": ["a deep-well magnet plate (bought) for the wash steps; MB-1 stays for the 0.2 mL plate",
                             "the 0.5-10 uL eight-channel cannot reach the floor of a 44 mm plate with standard 10 uL tips (manifold hits "
                             "the plate top): use the 5-50 uL tool for washes, or long 10 uL tips (about 45 mm, a fifth tip type)",
                             "the shaker sunk 40 mm on a tray at D4 (modelled in pass 2; the maker's clamp and cable must suit)",
                             "planner: fill limits per plate class above; one deep-well plate per stack place; washes not on CPB-1",
                             "tip saving: washes in the taller plate need no tip-to-well-top margin: the extra ~1,480 tips at 96 samples go away"]}


if __name__ == "__main__":
    r = main()
    print(json.dumps({"valid": r["parts_valid"], "configs": r["interference"]["head_configs_checked"], "real": r["interference"]["count_real"],
                      "by_kind": r["interference"]["by_kind"], "static": len(r["interference"]["static_cross_group"]),
                      "gantry": len(r["interference"]["gantry_vs_static"]), "head_gantry": len(r["interference"]["head_vs_gantry"]),
                      "reach_fail": len(r["reach_failures"]), "env": r["envelope_mm"]["W_D_H_mm"], "z": r["z_clearance"]}, indent=1))
