"""LH-1 HEAD: X carriage plate, three Z axes, gripper tool, pipette adapters (CadQuery 2.5.2).

Run from the project root:   .venv/bin/python cad/lh1/head/build.py
Out: cad/lh1/head/out/  STEP per custom part, DXF for laser-cut parts (flat pattern), STL for printed
     parts, head_assembly*.step (simplified bought parts), report.json (parts, bought parts, checks).

Frames
  carriage-local (from params): origin on the X-beam front face at carriage centre / beam mid-height;
      +x = global X, -y = toward the operator, +z up; global z = local z + X_BEAM_MID_Z.
  tool frame (per Z axis): origin at the centre of the TOOL PLATE FRONT FACE in x, y = 0 on that face
      (tools live at y < 0), z = 0 at the tool plate LOWER EDGE.  Moves with the Z axis.

Status: FABRICATION CANDIDATE for a first prototype.  Nothing here is validated.  Bought-part
dimensions are catalogue nominals; everything tagged "measure on the part" must be measured (gate G0).
"""
from __future__ import annotations

import itertools
import json
import math
import sys
from pathlib import Path

import cadquery as cq

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parent))
import params as P  # noqa: E402
sys.path.insert(0, str(HERE.parent / "assembly"))      # integration rev D: in-memory params overrides
import params_override  # noqa: E402
params_override.apply(P)

OUT = HERE / "out"

# ======================================================================================
# Bought-part nominals (catalogue; confirm on the parts at G0)
# ======================================================================================
MGN12_RAIL_W, MGN12_RAIL_H, MGN12_PITCH, MGN12_END = 12.0, 8.0, 25.0, 12.5
BLK_L, BLK_W, BLK_H = P.MGN12H["block_l"], P.MGN12H["block_w"], P.MGN12H["height"]
BLK_B, BLK_C = P.MGN12H["hole_pattern"]
# rev G5: Z motors are 48 mm NEMA17 (17HS8401 class): the 40 mm motor could not give 1.1 x Z_MIN_THRUST_N inside the TMC2209
# short-time current (electronics/lh1/system.yaml z_drive).  ALL values catalogue-typical, ASSUMED: confirm on the motor bought.
N17, N17_L, N17_HOLES, N17_BOSS, N17_SHAFT_L = 42.3, 48.0, 31.0, 22.0, 14.0   # rev G5: body 48 (was 40); shaft 14 (cut / short-shaft)
N17_HOLD_NM, N17_USABLE_NM, N17_DETENT_NM = 0.52, 0.325, 0.02   # rev G5 (was 0.40 / 0.25 / 0.015); usable = holding x 0.625 as before
N17_MASS_KG = 0.36                                              # rev G5 (40 mm: 0.28)
N17_ID = "NEMA17_48"
N11, N11_HOLES = 28.2, 23.0
T8_D, T8_D2, T8_LEAD = 8.0, 7.0, 2.0          # Tr8x2 single start: pitch diameter 7.0
NUT_FL_D, NUT_FL_T, NUT_BODY_D, NUT_BODY_L, NUT_PCD = 22.0, 3.5, 10.2, 21.5, 16.0   # anti-backlash POM nut
COUPLER_D, COUPLER_L = 20.0, 20.0             # 5 -> 8 mm flexible coupler; rev G: L20 (was 30) for the eject overtravel
EJ = P.Z_EJECT_TRAVEL                          # rev G: method-B tip ejection, Z travel above the normal top
BRG_688 = (8.0, 16.0, 5.0)                    # bore, OD, width (gripper centre bearings)
BRG_695 = (5.0, 13.0, 4.0)                    # (G3 and earlier) Z screw fixed bearing on a D5 journal
# rev G4: fixed end of the Z lead screw = two F6-14M thrust bearings, one on each face of the ZBH-1 web, clamped between the
# screw shoulder (D7 thread blank -> D6 journal) and an M6 lock nut; MR126ZZ in the web pocket is RADIAL ONLY (inner ring free on
# the journal, no axial contact).  Both axial directions go through a thrust bearing.
THRUST = (6.0, 14.0, 5.0)                     # F6-14M: bore, OD, height
BRG_RAD = (6.0, 12.0, 4.0)                    # MR126ZZ radial bearing
SHIM = (6.5, 14.0, 2.0)                       # hardened backing washer under the TOP thrust housing washer (bridges the D12 pocket)
SPOT = (14.2, 0.5)                            # spot faces centring the thrust housing washer / shim on both web faces
JOURNAL_D, BLANK_D = 6.0, 7.0                 # machined screw end: D6 h6 journal, shoulder against the D7 thread blank
NUT_M6 = (10.0, 6.0, 1.0)                     # ISO 10511 M6 lock nut AF10 (modelled D10) x 6 + DIN 125 M6 washer 1.6 (modelled 1.0)
COLLAR = (5.0, 10.0, 6.0)                     # (G3 and earlier) D5 collar / M5 lock nut under the Z bearing housing
MGN9_RAIL_W, MGN9_RAIL_H, MGN9C_L, MGN9C_W, MGN9C_H = 9.0, 6.5, 28.9, 20.0, 10.0
EXTR_2040_A, EXTR_2040_I_WEAK = 290.0, 1.4e4   # mm^2, mm^4 (approx. catalogue; verify for the profile bought)
# integration rev F (M1): carriage stiffeners are steel rectangular tube 40 x 20 x 2 (same 20 x 40 envelope as the 2040 they
# replace), E 210 GPa: section transformed to aluminium (n = 3) in deflection_check.  +0.18 kg per stiffener.
STIFF_A, STIFF_I_WEAK, STIFF_N = 224.0, 14379.0, 210.0 / 70.0
E_AL = 70000.0
RHO = {"al": 2.70e-6, "pa12": 1.01e-6}        # kg/mm^3 (MJF PA12 printed solid)

# ======================================================================================
# HEAD design parameters (derived from params.py where possible)
# ======================================================================================
# integration rev D: 2 mm XSP-1 spacers on the X blocks -> plate back face at -15 (belt gets 3 mm each side)
PL_BACK_Y = getattr(P, "CARRIAGE_PLATE_BACK_LOCAL_Y", -BLK_H)
XSP_T = -BLK_H - PL_BACK_Y                     # spacer thickness (2)
PL_FRONT_Y = PL_BACK_Y - P.CARRIAGE_PLATE_T    # -19
ZBLK_TOP_Y = PL_FRONT_Y - BLK_H                # Z block top face (-32)
ZC_T = 10.0                                    # Z carriage (nut-carrying slide, CNC) thickness
Y_TF = ZBLK_TOP_Y - ZC_T - P.TOOL_PLATE_T      # tool plate front face, carriage-local y (-47)
AXES_X = list(P.TOOL_X_OFFSETS)               # -111, -37, 37, 111 (rev B)
SX, SY = 22.5, PL_FRONT_Y - 12.0               # integration rev D: screw axis 12 mm in front of the plate (was fixed -31)
ZB_TOP = P.TOOL_PLATE_BOTTOM_Z_AT_TOP - P.X_BEAM_MID_Z      # tool plate lower edge at top, local (-70)
RAIL_TOP = P.Z_RAIL_TOP_LOCAL_Z
RAIL_BOT = RAIL_TOP - P.Z_RAIL_LEN
BLK_OFFSET = 2.0                               # Z block centre sits this far below the tool plate centre
LEDGE_T = 8.0                                  # nut ledge thickness (top of Z carriage)
SHELF_Z0, SHELF_T = 98.0, 6.0                  # Z motor shelf (local z of underside); rev G: +2 (eject overtravel)
LEG_Z0 = P.Z_RAIL_TOP_LOCAL_Z                  # rev F/G: legs start at the rail top
HOUSING_Z = (-194.0, -182.0)                   # bottom bearing housing body (local z)
SCREW_Z = (HOUSING_Z[0] - 15.0, SHELF_Z0 - COUPLER_L + 11.0)   # lead screw ends (local z); rev G4: M6 end thread 15 mm below the web
Z_SHOULDER = HOUSING_Z[1] - SPOT[1] + SHIM[2] + THRUST[2]   # rev G4: screw shoulder (top of the upper thrust bearing)
SW_TRIGGER_GAP = 0.3                           # switch lever above the Z carriage top at the top position

# pipette reach parameters: the tool decides these, the checks below bound them
NOZZLE_BELOW_TP = P.TOOL_PLATE_BOTTOM_Z_AT_TOP - P.PIPETTE_NOZZLE_Z_AT_TOP   # 87: nozzle end below tool plate
GRIP_TIP_ABOVE_SEAT = 2.0  # gripper finger tip above LABWARE_SEAT_Z at the bottom of travel
# labware / rack assumptions used only for reach checks -- "measure on the part"
PCR_TOP_ABOVE_SEAT, PCR_WELL_DEPTH = 21.0, 17.0
RACK200_COLLAR_ABOVE_SEAT = 62.0   # measure on the part: top of 200 uL tip collars in a full rack
TIP_ENGAGE = 8.0                   # measure on the part: nozzle press-in depth into a tip collar
SBS_PLATE_H = 14.35

# ---------------- gripper (tool frame) ----------------
G_YC = -P.TOOL_ENVELOPE[1] / 2                 # labware centre in front of the tool plate (-55)
G_FINGER_L = P.TOOL_PLATE_BOTTOM_Z_AT_TOP - P.GRIPPER_FINGERTIP_Z_AT_TOP     # 115
G_BLADE_T, G_PAD_T = 4.0, P.GRIP_FINGER_T - 4.0   # 4 mm 5052 blade + 2 mm silicone pad = GRIP_FINGER_T
G_CARRIER_L = 28.0
G_SCREW_X, G_SCREW_Z = -12.0, -16.0            # L/R Tr8x2 screw axis
G_MOTOR_Z = 22.5                               # NEMA11 axis above the arm
G_RAIL_X = 12.0
G_PULLEY_RATIO = 20.0 / 16.0                   # GT2 16T motor -> 20T screw
N11_USABLE_NM = 0.04                           # 28HS32 (0.06 N m holding), derated
SCREW_MU = (0.10, 0.15, 0.20)                  # POM nut on rolled steel, greased .. dry

# ---------------- generic adapter / module ----------------
MOD_D = 40.0          # measure on the part: bought module body diameter (generic adapter, max 46)
MOD_CONE_L = 15.0     # measure on the part: nozzle cone length below the module body
MOD_TOP = 129.0       # measure on the part: module top, tool frame z

# ---------------- hand pipettes (tool frame) ----------------
# One parametric design, three parameter sets.  EVERY value below depends on the bought pipette:
# "measure on the part".  Nominals: a typical 0.5-10 uL single-channel and typical 8-channel bodies.
#   body_d handle diameter at the clamps | shaft_l nozzle end -> handle bottom (single) | body_l handle length
#   plunger_above button top above handle top at rest | stroke rest -> blow-out | force at blow-out (N)
#   button_d | m_w, m_t, m_h 8-ch manifold width along Y, thickness along X, height | nozzle_l nozzle cone length
HP_SETS = {
    "pipette_small": dict(eight=False, body_d=28.0, shaft_l=72.0, body_l=110.0, plunger_above=40.0,
                          stroke=15.0, force=25.0, button_d=14.0, m_w=0.0, m_t=0.0, m_h=0.0, nozzle_l=0.0),
    "pipette_8ch_50": dict(eight=True, body_d=30.0, shaft_l=0.0, body_l=105.0, plunger_above=42.0,
                           stroke=18.0, force=40.0, button_d=16.0, m_w=90.0, m_t=32.0, m_h=50.0, nozzle_l=14.0),
    "pipette_8ch_300": dict(eight=True, body_d=32.0, shaft_l=0.0, body_l=105.0, plunger_above=45.0,
                            stroke=22.0, force=40.0, button_d=16.0, m_w=95.0, m_t=36.0, m_h=55.0, nozzle_l=18.0),
}
HP_ACT_STEP_LEAD = 1.0             # NEMA11 captive linear stepper, lead 1 mm
HP_ACT_USABLE_NM = 0.03


# ======================================================================================
# geometry helpers
# ======================================================================================
def bx(x0, x1, y0, y1, z0, z1) -> cq.Workplane:
    xa, xb, ya, yb, za, zb = min(x0, x1), max(x0, x1), min(y0, y1), max(y0, y1), min(z0, z1), max(z0, z1)
    return cq.Workplane("XY").box(xb - xa, yb - ya, zb - za, centered=False).translate((xa, ya, za))


_AX = {"x": cq.Vector(1, 0, 0), "y": cq.Vector(0, 1, 0), "z": cq.Vector(0, 0, 1)}


def cyl(axis: str, p: tuple, d: float, a0: float, a1: float) -> cq.Workplane:
    """Cylinder along `axis` through point p (the axis coordinate of p is ignored) from a0 to a1."""
    i = "xyz".index(axis)
    q = list(p)
    q[i] = min(a0, a1)
    return cq.Workplane(obj=cq.Solid.makeCylinder(d / 2, abs(a1 - a0), cq.Vector(*q), _AX[axis]))


def cone(axis: str, p: tuple, d0: float, d1: float, a0: float, a1: float) -> cq.Workplane:
    i = "xyz".index(axis)
    q = list(p)
    q[i] = a0
    v = _AX[axis] * (1 if a1 > a0 else -1)
    return cq.Workplane(obj=cq.Solid.makeCone(d0 / 2, d1 / 2, abs(a1 - a0), cq.Vector(*q), v))


def cut_all(w: cq.Workplane, tools: list) -> cq.Workplane:
    for t in tools:
        w = w.cut(t)
    return w


def flat_plate(outline: cq.Workplane, t: float) -> cq.Workplane:
    return outline.extrude(t)


def flat_to_xz(flat: cq.Workplane, y_back: float) -> cq.Workplane:
    """Flat part (sketch x,y; thickness +z) -> plate in the X-Z plane, flat y -> z, thickness toward -y,
    back face at y = y_back."""
    return flat.rotate((0, 0, 0), (1, 0, 0), 90).translate((0, y_back, 0))


def holes_flat(w: cq.Workplane, pts, d) -> cq.Workplane:
    if not pts:
        return w
    return w.faces(">Z").workplane(centerOption="ProjectedOrigin", origin=(0, 0, 0)).pushPoints(pts).hole(d)


def slots_flat(w: cq.Workplane, pts, w_slot, length, angle=0.0, t=10.0) -> cq.Workplane:
    for (x, y) in pts:
        s = cq.Workplane("XY").center(x, y).slot2D(length, w_slot, angle).extrude(t).translate((0, 0, -1))
        w = w.cut(s)
    return w


def csk_flat(w: cq.Workplane, pts, d, dk, top_z) -> cq.Workplane:
    """90 deg countersunk through holes, countersink on the +z face (z = top_z)."""
    for (x, y) in pts:
        w = w.cut(cyl("z", (x, y, 0), d, -1, top_z + 1))
        w = w.cut(cone("z", (x, y, 0), dk, d, top_z, top_z - (dk - d) / 2))
    return w


def csk_flat_back(w: cq.Workplane, pts, d, dk, top_z) -> cq.Workplane:
    """rev G3: 90 deg countersunk through holes, countersink on the z = 0 face (the BACK face after flat_to_xz)."""
    for (x, y) in pts:
        w = w.cut(cyl("z", (x, y, 0), d, -1, top_z + 1))
        w = w.cut(cone("z", (x, y, 0), dk, d, 0.0, (dk - d) / 2))
    return w


# ======================================================================================
# 1. carriage plate CP-1 (laser, 5052, CARRIAGE_PLATE_T) and its static furniture
# ======================================================================================
X_BLOCKS = [(bx_, bz) for bx_ in P.X_BLOCKS_LOCAL_X for bz in P.X_RAILS_LOCAL_Z]
CLAMP_Z = (P.X_BELT_LOWER_Z - 6.0, P.X_BELT_LOWER_Z + 6.0)   # integration rev D: +/-6 so XBC-1 stays 3 mm under the free upper run
LEG_HOLE_Z = (65.0, 75.0)
LEGS_X = ((2.5, 11.5), (35.0, 43.0))           # rel. to the Z rail
SWB_X, SWB_HOLES = (-25.0, -3.0), (-20.0, -8.0)
TRAY_HOLES = [(-146.0, 50.0), (-146.0, 70.0)]
ANCHOR_HOLES = [(13.0, 72.0), (31.0, 72.0)]
ANCHOR_X = (9.0, 35.0)
STIFF_Z = (-238.0, -45.0)                     # below the 80 mm X beam (local z -40)
STIFF_BOLT_Z = (-60.0, -120.0, -180.0)
HOUSING_TAB = (14.5, 36.0, -226.0)
HOUSING_HOLES = (19.5, 31.0, -212.0)


# ---- integration rev D: new carriage furniture ----
MBB_X = (-105.0, 3.0)                          # MMB board bracket (CR-06), left of the chain anchor
MBB_HOLES = [(-90.0, 72.0), (-12.0, 72.0)]     # M3 clearance in CP-1 -> heat-set inserts in the MBB-1 flange
MMB = (108.3, 59.0, 25.0)                      # BTT MMB CAN V2.0 board incl. EZ drivers (electronics/lh1/decisions.yaml)
MMB_Y = (-9.5, 49.5)
XFL = (140.0, P.X_ENDSTOP_FLAG_TIP_LOCAL_X)    # integration rev E (CR-12b): tip at local x 155.5, lever trips at carriage x 295
XFL_HOLES = [(143.0, -13.0), (151.0, -13.0)]


def x_spacer_flat() -> cq.Workplane:
    """XSP-1 (laser 5052, 2 mm): spacer under CP-1 on each X block (belt clearance); flat x, y = local z."""
    f = cq.Workplane("XY").rect(BLK_L, BLK_W).extrude(XSP_T)
    return holes_flat(f, [(sx * BLK_C / 2, sz * BLK_B / 2) for sx in (-1, 1) for sz in (-1, 1)], 3.4)


def x_spacer(x, z) -> cq.Workplane:
    return x_spacer_flat().rotate((0, 0, 0), (1, 0, 0), 90).translate((x, -BLK_H, z))


def mmb_bracket() -> cq.Workplane:
    """MBB-1 (MJF PA12): flange behind CP-1 above the belt (z 64..80), shelf over the beam for the two MMB boards."""
    w = bx(MBB_X[0], MBB_X[1], PL_BACK_Y, PL_BACK_Y + 4.0, 64.0, P.CARRIAGE_PLATE_Z[1])
    w = w.union(bx(MBB_X[0], MBB_X[1], PL_BACK_Y, MMB_Y[1], P.CARRIAGE_PLATE_Z[1], P.CARRIAGE_PLATE_Z[1] + 3.0))
    for x, z in MBB_HOLES:
        w = w.cut(cyl("y", (x, 0, z), 4.0, PL_BACK_Y - 1, PL_BACK_Y + 5))     # M3 heat-set inserts
    for x in (MBB_X[0] + 4.0, MBB_X[1] - 4.0):
        for y in (MMB_Y[0] + 4.0, MMB_Y[1] - 4.0):
            w = w.cut(cyl("z", (x, y, 0), 3.6, 79, 84))                       # M2.5 heat-set inserts (standoffs)
    return w


MMB_PCB = 1.6
MMB_Z0 = (P.CARRIAGE_PLATE_Z[1] + 3.0 + 5.0, P.CARRIAGE_PLATE_Z[1] + 3.0 + 5.0 + MMB[2] + 1.0)   # rev E: PCB undersides 88, 114
MMB_HOLES = [(MBB_X[0] + 4.0, MMB_Y[0] + 4.0), (MBB_X[0] + MMB[0] - 4.0, MMB_Y[0] + 4.0),
             (MBB_X[0] + 4.0, MMB_Y[1] - 4.0), (MBB_X[0] + MMB[0] - 4.0, MMB_Y[1] - 4.0)]


def b_mmb(k: int) -> cq.Workplane:
    """integration rev E: PCB 1.6 + component envelope inset 6 mm from the edges (mounting corners free)"""
    z0 = MMB_Z0[k]
    w = bx(MBB_X[0], MBB_X[0] + MMB[0], MMB_Y[0], MMB_Y[1], z0, z0 + MMB_PCB)
    return w.union(bx(MBB_X[0] + 6.0, MBB_X[0] + MMB[0] - 6.0, MMB_Y[0] + 6.0, MMB_Y[1] - 6.0, z0 + MMB_PCB, z0 + MMB[2]))


def b_standoffs() -> cq.Workplane:
    """integration rev E: M2.5 hex standoffs: HIB-1 4 x 5 mm on EB-1; MMB_a 4 x 5 mm on MBB-1; MMB_b 4 x 24.4 mm on MMB_a"""
    w = None
    pts = [(x, y, P.CARRIAGE_PLATE_Z[1] + 3.0, MMB_Z0[0]) for x, y in MMB_HOLES]
    pts += [(x, y, MMB_Z0[0] + MMB_PCB, MMB_Z0[1]) for x, y in MMB_HOLES]
    if not getattr(P, "REV_J", False):     # rev J: no HIB-1 standoffs
        pts += [(x, y, 85.0, 90.0) for x in (BOARD_X[0] + 4.0, BOARD_X[1] - 4.0) for y in (BOARD_Y[0] + 4.0, BOARD_Y[1] - 4.0)]
    for x, y, z0, z1 in pts:
        c = cyl("z", (x, y, 0), 5.0, z0, z1)
        w = c if w is None else w.union(c)
    return w


CAM = (-149.0, PL_FRONT_Y - 6.0, -80.0, -40.0, 10.0)    # integration rev E: down-looking D10 carriage camera (x, y, z0, z1, d)
if getattr(P, "REV_K", False):   # rev K (ADR-004 d1): VC-1 head camera, lens at global z 165 (working distance 160 to the deck seat)
    CAM = (-149.0, PL_FRONT_Y - 6.0, 165.0 - P.X_BEAM_MID_Z, 165.0 - P.X_BEAM_MID_Z + 40.0, 10.0)


def cam_clip() -> cq.Workplane:
    """CAM-1 (MJF PA12): clip for a D10 endoscope-class camera on the CP-1 front face, outboard of the left tool"""
    x, y, z0, z1, d = CAM
    w = bx(x - 6.0, x + 6.0, y - 6.0, PL_FRONT_Y, z1 - 12.0, z1)
    w = w.cut(cyl("z", (x, y, 0), d + 0.3, z1 - 13.0, z1 + 1.0))
    return w.cut(cyl("y", (x, 0, z1 - 6.0), 3.4, PL_FRONT_Y - 1.0, PL_FRONT_Y - 20.0))


def b_camera() -> cq.Workplane:
    x, y, z0, z1, d = CAM
    return cyl("z", (x, y, 0), d, z0, z1)


def x_flag() -> cq.Workplane:
    """XFL-1 (laser + 1 bend, 5052 1.5 mm): X end-stop flag on the CP-1 front face, right edge."""
    w = bx(XFL[0], XFL[1], PL_FRONT_Y - 5.0, PL_FRONT_Y - 3.5, -18.0, -8.0)
    w = w.union(bx(XFL[0], 155.0, PL_FRONT_Y - 3.5, PL_FRONT_Y, -18.0, -16.5))
    w = w.union(bx(XFL[0], 155.0, PL_FRONT_Y - 1.5, PL_FRONT_Y, -18.0, -8.0))
    for x, z in XFL_HOLES:
        w = w.cut(cyl("y", (x, 0, z), 3.4, PL_FRONT_Y - 6, PL_FRONT_Y + 1))
    return w


# ---- integration rev E2: Z-axis cabling = one vertical ROLLING LOOP of 10-way 1.0 mm-pitch high-flex FFC per tool,
# in the free gap left of each Z block between the carriage-plate front (y -21) and the Z-carriage back (y -34).
# Fixed run on the plate front, moving run on the Z-carriage back, 180-degree bend at the bottom moving at half the Z speed.
FFC = dict(x=(-26.5, -15.5), w_ways=10, pitch=1.0, t=0.5,          # t modelled 0.5 (real FFC about 0.15)
           z_fix=0.0,                                              # fixed clamp exit (local z), cable runs down from here
           z_mov_tf=110.0,                                         # moving clamp exit on the Z-carriage back (tool-frame z)
           free=175.0,                                             # straight length (excl. the bend) between the clamps
           slot_z=(57.0, 59.5))                                    # CP-1 slot: fixed tail to the plate back (static)


def ffc_geometry(s):
    """(z_fixed_exit, z_moving_exit, z_bend) in local z for stroke s"""
    zf = FFC["z_fix"]
    zm = (ZB_TOP - s) + FFC["z_mov_tf"]
    zb = (zf + zm - FFC["free"]) / 2.0
    return zf, zm, zb


def ffc_cable(s) -> cq.Workplane:
    """loop envelope at stroke s, axis at x = 0 (fixed run, bend, moving run)"""
    x0, x1 = FFC["x"]
    t = FFC["t"]
    zf, zm, zb = ffc_geometry(s)
    yf, ym = PL_FRONT_Y, ZBLK_TOP_Y                               # -21 (plate face), -34 (Z-carriage back)
    fixed = bx(x0, x1, yf - t, yf, zb, zf)
    moving = bx(x0, x1, ym, ym + t, zb, zm)
    rc, yc = (yf - ym) / 2.0, (yf + ym) / 2.0
    ring = cyl("x", (0, yc, zb), 2 * rc, x0, x1).cut(cyl("x", (0, yc, zb), 2 * (rc - t), x0 - 1, x1 + 1))
    ring = ring.cut(bx(x0 - 1, x1 + 1, ym - 1, yf + 1, zb, zb + rc + 1))
    return fixed.union(moving).union(ring)


FFCF_HOLE = (-29.5, 5.0)     # rev G4: FFC-F fixing (x rel. axis, local z), M2 tapped in the plate


def ffc_fixed_parts() -> cq.Workplane:
    """FFC-F (MJF PA12) clamp + 2 tie points on the plate front, and the static tail up to the CP-1 slot and behind it"""
    x0, x1 = FFC["x"]
    t = FFC["t"]
    # rev G4: the plate screw (x -29.5) was outside the clamp; the clamp now reaches x -31.5 and takes 1 x M2 there
    clamp = bx(FFCF_HOLE[0] - 2.0, x1 + 1.0, PL_FRONT_Y - t - 3.0, PL_FRONT_Y - t, FFC["z_fix"], FFC["z_fix"] + 10.0)
    clamp = clamp.cut(cyl("y", (FFCF_HOLE[0], 0, FFCF_HOLE[1]), 2.4, PL_FRONT_Y - 5.0, PL_FRONT_Y + 1.0))
    tie = bx(x0 - 1.0, x1 + 1.0, PL_FRONT_Y - t - 2.0, PL_FRONT_Y - t, FFC["z_fix"] + 25.0, FFC["z_fix"] + 30.0)
    return clamp.union(tie)


def ffc_tail() -> cq.Workplane:
    x0, x1 = FFC["x"]
    t = FFC["t"]
    front = bx(x0, x1, PL_FRONT_Y - t, PL_FRONT_Y, FFC["z_fix"], FFC["slot_z"][1])
    back = bx(x0, x1, PL_BACK_Y, PL_BACK_Y + t, FFC["slot_z"][0], FFC["slot_z"][0] + 6.0)
    return front.union(back)


def ffc_moving_clamp() -> cq.Workplane:
    """FFC-M (MJF PA12) on the Z-carriage back top (tool frame), tie point on top of the carriage"""
    x0, x1 = FFC["x"]
    t = FFC["t"]
    yb = TF_ZC[1]                                                   # Z-carriage back face, tool frame y (15)
    # rev G4: widened to the ZC-1 -X edge for its fixing: 2 x M2 x 8 along Y into ZC-1 (tapped), clear of the cable (x -26.5..-15.5)
    w = bx(ZC_X[0], x1 + 1.0, yb + t, yb + t + 3.0, FFC["z_mov_tf"], FFC["z_mov_tf"] + 10.0)
    for x, z in FFCM_HOLES:
        w = w.cut(cyl("y", (x, 0, z), 2.4, yb, yb + t + 4.0))
    return w


FFCM_HOLES = [(-29.5, 112.5), (-29.5, 117.5)]   # rev G4: FFC-M fixing, tool frame (x, z)


def carriage_plate_flat(back_csk: bool = True) -> cq.Workplane:
    x0, x1 = P.CARRIAGE_PLATE_X
    z0, z1 = P.CARRIAGE_PLATE_Z
    t = P.CARRIAGE_PLATE_T
    f = cq.Workplane("XY").center((x0 + x1) / 2, (z0 + z1) / 2).rect(x1 - x0, z1 - z0).extrude(t)
    f = f.edges("|Z").fillet(4.0)
    clear_m3 = [(bx_ + sx * BLK_C / 2, bz + sz * BLK_B / 2) for bx_, bz in X_BLOCKS for sx in (-1, 1) for sz in (-1, 1)]
    tap_m3 = [(x, z) for x in P.X_BELT_CLAMP_X for z in CLAMP_Z]
    tap_m4 = []
    csk_back_m3 = []
    for i, ax in enumerate(AXES_X):
        h = axis_plate_holes(i, ax)
        tap_m3 += h["tap_m3"]
        tap_m4 += h["tap_m4"]
        csk_back_m3 += h["csk_back_m3"]
    tap_m3 += TRAY_HOLES + ANCHOR_HOLES
    # integration rev D: the 16 X-block screws are ISO 10642 M3 countersunk from the front (rails/Z blocks pass over them);
    # the XBC-1 clamp screws come from the front into the tapped clamp (belt service without removing the head)
    clamp_pts = [(x, z) for x in P.X_BELT_CLAMP_X for z in CLAMP_Z]
    tap_m3 = [q for q in tap_m3 if q not in clamp_pts]
    f = holes_flat(f, tap_m3, 2.5)          # M3 tap drill (6 mm 5052: tapped allowed)
    f = holes_flat(f, MBB_HOLES + XFL_HOLES + [(CAM[0], CAM[3] - 6.0)], 3.4)   # rev D: MBB, XFL; rev E: camera clip
    f = holes_flat(f, tap_m4, 3.3)          # M4 tap drill
    csk = [(ax + s * 10.0, z) for ax in AXES_X for s in (-1, 1) for z in STIFF_BOLT_Z]
    for ax in AXES_X:                                    # integration rev E2: FFC tail slot per axis + 2 M3 for FFC-F
        f = f.cut(cq.Workplane("XY").center(ax + sum(FFC["x"]) / 2, sum(FFC["slot_z"]) / 2)
                  .rect(FFC["x"][1] - FFC["x"][0] + 2.0, FFC["slot_z"][1] - FFC["slot_z"][0]).extrude(t + 2).translate((0, 0, -1)))
    f = holes_flat(f, [(ax + FFCF_HOLE[0], FFCF_HOLE[1]) for ax in AXES_X], 1.6)   # rev G4: FFC-F, M2 tapped
    f = csk_flat(f, clear_m3 + clamp_pts, 3.4, 6.6, t)   # integration rev D: M3 csk from the front
    f = csk_flat_back(f, csk_back_m3, 3.4, 6.6, t) if back_csk else holes_flat(f, csk_back_m3, 3.4)   # rev G3: EJ-1 root, ISO 10642 M3 from BEHIND (flush: X blocks behind)
    return csk_flat(f, csk, 5.5, 10.4, t)   # M5 countersunk from the FRONT face into 2040 T-nuts


def axis_plate_holes(i: int, ax: float) -> dict:
    """rev G3: every hole one Z axis needs in the plate it hangs on (CP-1 on the machine, BR-2 on the rig), flat coords
    (x, z).  The fastener schedule (axis_joints) counts its screws from these same lists."""
    n = int((P.Z_RAIL_LEN - 2 * MGN12_END) // MGN12_PITCH) + 1
    h = {"rail_m3": [(ax, RAIL_TOP - MGN12_END - k * MGN12_PITCH) for k in range(n)],
         "zsw_m3": [(ax + x, 75.0) for x in SWB_HOLES],
         "zmb_m4": [(ax + (a + b) / 2, z) for a, b in LEGS_X for z in LEG_HOLE_Z],
         "zbh_m4": [(ax + HOUSING_HOLES[0], HOUSING_HOLES[2]), (ax + HOUSING_HOLES[1], HOUSING_HOLES[2])],
         "zhs_m4": [(ax + x, z) for x, z in ZHS["m4"]] if i in ZHS_AXES else [],
         "ej_m3": [(ax + x, z) for x, z in ej_root_holes(i)] if P.TOOL_NAMES[i] in getattr(P, "EJECTOR", {}) else [],
         "ffcf_m2": [(ax + FFCF_HOLE[0], FFCF_HOLE[1])]}
    h["tap_m3"] = h["rail_m3"] + h["zsw_m3"]
    h["tap_m4"] = h["zmb_m4"] + h["zbh_m4"] + h["zhs_m4"]
    h["csk_back_m3"] = h["ej_m3"]
    return h


def belt_clamp() -> cq.Workplane:
    """XBC-1: clamps both cut ends of the lower X belt run in a 1.3 mm groove against the plate back."""
    # integration rev D: 3 mm to the beam face (y -3) and to the free upper run (top z 54); groove open to the BACK so the
    # belt end slides in from behind; 4 x M3 tapped blind from the front, screws through CP-1 from the front.
    z_lo, z_hi = 33.0, P.X_BELT_UPPER_Z - 0.69 - 3.0
    y_back = -3.0
    c = bx(-46, 46, PL_BACK_Y, y_back, z_lo, z_hi).edges("|Y").fillet(2.0)
    zc = P.X_BELT_LOWER_Z
    c = c.cut(bx(-47, 47, P.X_BELT_LOCAL_Y - 4.5 - 0.2, y_back + 1, zc - 0.65, zc + 0.65))
    for x in P.X_BELT_CLAMP_X:
        for z in CLAMP_Z:
            c = c.cut(cyl("y", (x, 0, z), 2.5, PL_BACK_Y - 1, PL_BACK_Y + 2.8))
    return c


def motor_bracket() -> cq.Workplane:
    """ZMB-1 (CNC 6061), at rail x = 0: two legs on the plate front, shelf above the plate top edge."""
    w = bx(SX - N17 / 2, SX + N17 / 2, SY - N17 / 2, SY + N17 / 2, SHELF_Z0, SHELF_Z0 + SHELF_T)
    for a, b in LEGS_X:
        w = w.union(bx(a, b, PL_FRONT_Y - 6.0, PL_FRONT_Y, LEG_Z0, SHELF_Z0 + 0.5))
        for z in LEG_HOLE_Z:
            w = w.cut(cyl("y", ((a + b) / 2, 0, z), 4.5, PL_FRONT_Y - 7, PL_FRONT_Y + 1))
    w = w.cut(cyl("z", (SX, SY, 0), N17_BOSS + 0.5, SHELF_Z0 - 1, SHELF_Z0 + SHELF_T + 1))
    for sx in (-1, 1):
        for sy in (-1, 1):
            w = w.cut(cyl("z", (SX + sx * N17_HOLES / 2, SY + sy * N17_HOLES / 2, 0), 3.4, SHELF_Z0 - 1, SHELF_Z0 + 8))
    return w


def bearing_housing() -> cq.Workplane:
    """ZBH-1 (CNC 6061): fixed end of the lead screw (rev G4): F6-14M thrust bearing on each web face (top one on a hardened
    backing washer), MR126ZZ radial in the web, screw shoulder above, M6 lock nut + washer below: both axial directions
    through a thrust bearing, preloaded (no end float)."""
    xa, xb, ztab = HOUSING_TAB
    w = bx(xa, xb, SY - 8.5, PL_FRONT_Y, HOUSING_Z[0], HOUSING_Z[1])
    w = w.union(bx(xa, xb, PL_FRONT_Y - 4.0, PL_FRONT_Y, ztab, HOUSING_Z[0] + 0.5))
    # rev G4: web 12 mm: spot face D14.2 x 0.5 top and bottom (thrust washer / shim), D12 pocket 8 deep from the top for the
    # MR126ZZ (sits on the pocket floor, 4 mm below the shim: no axial contact), D6.5 clearance bore through the rest
    w = w.cut(cyl("z", (SX, SY, 0), JOURNAL_D + 0.5, HOUSING_Z[0] - 1, HOUSING_Z[1] + 1))
    w = w.cut(cyl("z", (SX, SY, 0), BRG_RAD[1], HOUSING_Z[1] - 8.0, HOUSING_Z[1] + 1))
    w = w.cut(cyl("z", (SX, SY, 0), SPOT[0], HOUSING_Z[1] - SPOT[1], HOUSING_Z[1] + 1))
    w = w.cut(cyl("z", (SX, SY, 0), SPOT[0], HOUSING_Z[0] - 1, HOUSING_Z[0] + SPOT[1]))
    for x in HOUSING_HOLES[:2]:
        w = w.cut(cyl("y", (x, 0, HOUSING_HOLES[2]), 4.5, PL_FRONT_Y - 5, PL_FRONT_Y + 1))
    return w


def switch_bracket() -> cq.Workplane:
    """ZSW-1 (MJF PA12): holds a KW10 micro switch whose lever the Z carriage top presses at home."""
    z_sw = ZB_TOP + P.TOOL_PLATE_H + SW_TRIGGER_GAP + 1.2 + EJ   # rev G: switches at the EJECT top
    w = bx(SWB_X[0], SWB_X[1], PL_FRONT_Y - 14.0, PL_FRONT_Y, z_sw, P.CARRIAGE_PLATE_Z[1])
    # integration rev E (review A12): arm carrying the second, 'Z up' D2F switch of the hardware all-tools-up chain
    w = w.union(bx(SWB_X[1] - 0.5, 10.0, PL_FRONT_Y - 14.0, PL_FRONT_Y - 7.0, z_sw + 6.5, z_sw + 16.0))
    w = w.cut(cyl("z", (0.0, 0, 0), 1.6, z_sw + 6.0, z_sw + 17.0).translate((0, PL_FRONT_Y - 10.5, 0)))
    w = w.cut(cyl("z", (6.5, 0, 0), 1.6, z_sw + 6.0, z_sw + 17.0).translate((0, PL_FRONT_Y - 10.5, 0)))
    for x in SWB_HOLES:
        w = w.cut(cyl("y", (x, 0, 75.0), 3.4, PL_FRONT_Y - 15, PL_FRONT_Y + 1))
        w = w.cut(cyl("y", (x, 0, 75.0), 6.0, PL_FRONT_Y - 15, PL_FRONT_Y - 4))
    for x in (-18.75, -9.25):
        w = w.cut(cyl("y", (x, 0, z_sw + 5.0), 1.8, PL_FRONT_Y - 15, PL_FRONT_Y - 4))
    return w


def zsw_ffc_bracket() -> cq.Workplane:
    """ZSF-1 (MJF PA12, rev J, ADR-003 #4): the ZSW-1 home/Z-up switch bracket and the FFC-F fixed cable clamp + tie as ONE print
    per axis, joined by a 4 x 3 mm web that runs IN FRONT of the static FFC tail (0.5 mm off it, x -24..-20) from the clamp to the
    ZSW-1 underside.  (A web beside the cable at x -31.5..-27 hit EB-1 on axis 0.)  The web is slender PA12, so the clamp's own
    M2 still draws the clamp onto the cable.  Fixings unchanged: 2 x M3 (switch end, counterbored) + 1 x M2 (clamp end)."""
    z_sw = ZB_TOP + P.TOOL_PLATE_H + SW_TRIGGER_GAP + 1.2 + EJ
    t = FFC["t"]
    w = switch_bracket().union(ffc_fixed_parts())
    w = w.union(bx(-24.0, -20.0, PL_FRONT_Y - t - 3.5, PL_FRONT_Y - t - 0.5, FFC["z_fix"] + 5.0, z_sw + 0.01))
    return w.cut(cyl("y", (FFCF_HOLE[0], 0, FFCF_HOLE[1]), 2.4, PL_FRONT_Y - 5.0, PL_FRONT_Y + 1.0))


BOARD_X = (P.CARRIAGE_PLATE_X[0] + 0.5, P.CARRIAGE_PLATE_X[0] + 45.5)
BOARD_Y = (-6.0, 39.0)                         # behind the plate top edge (Z motors occupy y < -9.85)


XT30_CLIP_Z = (50.0, 66.0)        # rev J: lead clips on the EB-1 web front (local z)
SPLICE = dict(n=4, w=13.1, l=18.4, h=8.1, pitch_x=17.0, pitch_y=23.0, x0=P.CARRIAGE_PLATE_X[0] + 6.0, y0=-14.0)   # rev J
EB2 = dict(wall=1.6, top=1.6, h=12.0, screws=[(P.CARRIAGE_PLATE_X[0] + 3.5, 37.0), (P.CARRIAGE_PLATE_X[0] + 42.5, 37.0)])


def splice_envelopes() -> cq.Workplane:
    """rev J: 4 lever splice connectors (3-way, 221-413 class, about 18.4 x 13.1 x 8.1) on EB-1 in place of HIB-1: V+, 0 V, CAN-H,
    CAN-L from the chain cable to the two head boards (keyed pre-made leads; electrical owner)"""
    top = P.CARRIAGE_PLATE_Z[1] + 5.0 - 2.0                       # seated in the 2 mm EB-1 pocket
    w = None
    for k in range(SPLICE["n"]):
        x = SPLICE["x0"] + (k % 2) * SPLICE["pitch_x"]
        y = SPLICE["y0"] + (k // 2) * SPLICE["pitch_y"]
        b = bx(x, x + SPLICE["w"], y, y + SPLICE["l"], top, top + SPLICE["h"])
        w = b if w is None else w.union(b)
    return w


def splice_cover() -> cq.Workplane:
    """EB-2 (MJF PA12, rev J): strain-relief cover over the lever splices on EB-1: open-bottom box, a 7 x 5 lead slot at each
    splice end (front and back), two cable-tie bars per side, 2 x M3 into heat-set inserts in EB-1.  Leads are tied to the bars,
    so a pull on a lead loads the cover, not the lever clamp."""
    top = P.CARRIAGE_PLATE_Z[1] + 5.0
    x0, x1 = P.CARRIAGE_PLATE_X[0], P.CARRIAGE_PLATE_X[0] + 46.0
    y0, y1 = PL_FRONT_Y + 1.0, 40.0
    wl, h = EB2["wall"], EB2["h"]
    c = bx(x0, x1, y0, y1, top, top + h).cut(bx(x0 + wl, x1 - wl, y0 + wl, y1 - wl, top - 1.0, top + h - EB2["top"]))
    for k in range(2):
        xc = SPLICE["x0"] + k * SPLICE["pitch_x"] + SPLICE["w"] / 2
        for yy in (y0, y1):
            c = c.cut(bx(xc - 3.5, xc + 3.5, yy - wl - 1, yy + wl + 1, top - 1.0, top + 5.0))       # lead slots
    for x, y in EB2["screws"]:
        c = c.union(bx(x - 3.5, x + 3.5, y - 3.0, y + 3.0, top, top + h)).cut(cyl("z", (x, y, 0), 3.4, top - 1, top + h + 1))
    return c


def board_tray() -> cq.Workplane:
    """EB-1 (MJF PA12): tray for the 45 x 45 x 15 tool-head board, on the plate top edge at the left end,
    reaching back over the X beam (board sits behind the left Z motor).  rev J: no HIB-1 board: the tray carries the lever
    splices under the EB-2 cover (2 x M3 heat-set inserts; the four M2.5 board-standoff holes are gone)."""
    x0 = P.CARRIAGE_PLATE_X[0]
    w = bx(x0, x0 + 15.0, PL_FRONT_Y - 5.0, PL_FRONT_Y, 40.0, P.CARRIAGE_PLATE_Z[1])   # rev E2: web 15 wide (axis-0 FFC channel)
    # rev J fix: the shelf now overlaps the web top (they only shared an edge: EB-1 exported as two loose solids)
    w = w.union(bx(x0, x0 + 46.0, PL_FRONT_Y - 5.0, 40.0, P.CARRIAGE_PLATE_Z[1], P.CARRIAGE_PLATE_Z[1] + 5.0))
    for x, z in TRAY_HOLES:
        w = w.cut(cyl("y", (x, 0, z), 3.4, PL_FRONT_Y - 6, PL_FRONT_Y + 1))
    if getattr(P, "REV_J", False):
        for x, y in EB2["screws"]:
            w = w.cut(cyl("z", (x, y, 0), 4.0, 79, 86))   # rev J: M3 heat-set inserts for the EB-2 cover
        top = P.CARRIAGE_PLATE_Z[1] + 5.0
        x0s, y0s = SPLICE["x0"] - 0.3, SPLICE["y0"] - 0.3                  # pocket: the 4-way lever block sits 2 mm deep, one way
        w = w.cut(bx(x0s, x0s + SPLICE["pitch_x"] + SPLICE["w"] + 0.6, y0s, y0s + SPLICE["pitch_y"] + SPLICE["l"] + 0.6, top - 2.0, top + 1.0))
        x0 = P.CARRIAGE_PLATE_X[0]
        for zc in XT30_CLIP_Z:                                              # two clips for the pre-made XT30(2+2) power + CAN leads
            clip = bx(x0 + 2.0, x0 + 13.0, PL_FRONT_Y - 5.0 - 8.0, PL_FRONT_Y - 5.0 + 0.01, zc - 5.0, zc + 5.0)
            clip = clip.cut(cyl("z", (x0 + 7.5, PL_FRONT_Y - 5.0 - 6.0, 0), 6.5, zc - 6.0, zc + 6.0))          # lead channel D6.5
            clip = clip.cut(bx(x0 + 1.0, x0 + 14.0, PL_FRONT_Y - 5.0 - 2.5, PL_FRONT_Y - 5.0 - 1.0, zc - 2.0, zc + 2.0))   # tie slot
            w = w.union(clip)
        return w
    for x in (BOARD_X[0] + 4.0, BOARD_X[1] - 4.0):
        for y in (BOARD_Y[0] + 4.0, BOARD_Y[1] - 4.0):
            w = w.cut(cyl("z", (x, y, 0), 2.8, 79, 86))   # M2.5 board standoffs, heat-set inserts
    return w


def chain_anchor() -> cq.Workplane:
    """CCA-1 (MJF PA12): fixed end of the head cable chain, behind/above the plate top, between motors."""
    top = P.CARRIAGE_PLATE_Z[1]
    # underside at local z 64: 5.6 mm above the free upper run of the X belt (z 57.7); at 60 it was 1.6 mm (frame report)
    w = bx(ANCHOR_X[0], ANCHOR_X[1], PL_BACK_Y, PL_BACK_Y + 5.0, 64.0, top)
    # integration rev D (CR-01): chain attach face raised to local z 90 -> 50 mm bend height over the beam top (z 40)
    w = w.union(bx(ANCHOR_X[0], ANCHOR_X[1], PL_BACK_Y, PL_BACK_Y + 5.0, top, top + 4.0))
    w = w.union(bx(ANCHOR_X[0], ANCHOR_X[1], PL_FRONT_Y, 25.0, top + 4.0, top + 10.0))
    for x, z in ANCHOR_HOLES:
        w = w.cut(cyl("y", (x, 0, z), 3.4, PL_BACK_Y - 1, PL_BACK_Y + 6))
    for y in (8.0, 20.0):
        w = w.cut(cyl("z", (sum(ANCHOR_X) / 2, y, 0), 3.4, top + 3, top + 11))
    return w

# ======================================================================================
# 2. Z axis moving parts (tool frame)
# ======================================================================================
TF_ZC = (P.TOOL_PLATE_T, P.TOOL_PLATE_T + ZC_T)          # Z carriage y range in the tool frame (5..15)
TF_BLK_Y = (TF_ZC[1], TF_ZC[1] + BLK_H)                  # Z block (15..28)
TF_SY = SY - Y_TF                                        # screw axis y in the tool frame (16)
TP_ZC = P.TOOL_PLATE_H / 2                               # tool plate centre height (65)
ZBLK_C = 81.0                                            # rev F: centre of the TWO-block pair in the tool frame (was one block at 63)
ZBLK_DZ = BLK_L / 2 + 0.5                                 # rev F: block centres at ZBLK_C +/- 23.2
TP_CSK = [(-15.0, TP_ZC - 28.0), (-15.0, TP_ZC + 28.0), (5.0, TP_ZC - 28.0), (5.0, TP_ZC + 28.0)]
ZC_X = (-P.TOOL_PLATE_W / 2, BLK_W / 2)                  # Z carriage body x range (-31..13.5)


def tool_plate_flat() -> cq.Workplane:
    """TP-1 (laser, 5052, TOOL_PLATE_T): flat x, y = tool-frame z, thickness +z (front face = +z)."""
    t = P.TOOL_PLATE_T
    f = cq.Workplane("XY").center(0, TP_ZC).rect(P.TOOL_PLATE_W, P.TOOL_PLATE_H).extrude(t)
    f = f.edges("|Z").fillet(3.0)
    f = holes_flat(f, [(x, TP_ZC + z) for x, z in P.TOOL_BOLTS], 3.3)       # M4 tapped (5 mm plate)
    f = holes_flat(f, [(x, TP_ZC + z) for x, z in P.TOOL_DOWELS], 3.0)      # D3 dowels, ream after cutting
    return csk_flat(f, TP_CSK, 3.4, 6.5, t)                                # M3 csk into the Z carriage


def tool_plate() -> cq.Workplane:
    return flat_to_xz(tool_plate_flat(), P.TOOL_PLATE_T)


def z_carriage() -> cq.Workplane:
    """ZC-1 (CNC 6061): bolts to the Z block, carries the tool plate and, on its top ledge, the nut."""
    top = P.TOOL_PLATE_H
    w = bx(ZC_X[0], ZC_X[1], TF_ZC[0], TF_ZC[1], 0, top)
    ledge_x1 = SX + NUT_FL_D / 2 + 0.5
    w = w.union(bx(ZC_X[1] - 0.5, ledge_x1, TF_ZC[0], TF_ZC[1], top - LEDGE_T, top))
    w = w.union(bx(ZC_X[1] + 0.5, ledge_x1, TF_ZC[1] - 0.5, TF_SY + NUT_FL_D / 2 - 0.5, top - LEDGE_T, top))
    w = w.cut(cyl("z", (SX, TF_SY, 0), NUT_BODY_D + 0.3, top - LEDGE_T - 1, top + 1))
    for a in (45, 135, 225, 315):
        r = NUT_PCD / 2
        p = (SX + r * math.cos(math.radians(a)), TF_SY + r * math.sin(math.radians(a)), 0)
        w = w.cut(cyl("z", p, 2.5, top - LEDGE_T - 1, top + 1))              # M3 tapped, nut flange
    for sx, sz, kz in itertools.product((-1, 1), (-1, 1), (-1, 1)):
        if True:
            p = (sx * BLK_B / 2, 0, ZBLK_C + kz * ZBLK_DZ + sz * BLK_C / 2)   # rev F: both blocks
            w = w.cut(cyl("y", p, 3.4, TF_ZC[0] - 1, TF_ZC[1] + 1))
            w = w.cut(cyl("y", p, 6.5, TF_ZC[0] - 1, TF_ZC[0] + 3.5))       # M3 SHCS head under tool plate
    for x, z in TP_CSK:
        w = w.cut(cyl("y", (x, 0, z), 2.5, TF_ZC[0] - 1, TF_ZC[0] + 9))     # M3 tapped
    for x, z in P.TOOL_BOLTS:
        if ZC_X[0] < x < ZC_X[1]:
            w = w.cut(cyl("y", (x, 0, TP_ZC + z), 4.5, TF_ZC[0] - 1, TF_ZC[1] + 1))   # M4 tip relief
    for x, z in P.TOOL_DOWELS:
        w = w.cut(cyl("y", (x, 0, TP_ZC + z), 3.5, TF_ZC[0] - 1, TF_ZC[0] + 6))
    for x, z in FFCM_HOLES:                                                  # rev G4: FFC-M, M2 tapped from the back face
        w = w.cut(cyl("y", (x, 0, z), 1.6, TF_ZC[1] - 6.0, TF_ZC[1] + 1))
    return w.union(cam_bar())                                                 # rev G4: ZUC-1 cam bars machined on ZC-1 (no joint)


# ---------------- simplified bought parts ----------------
def b_z_block() -> cq.Workplane:            # tool frame
    return bx(-BLK_W / 2, BLK_W / 2, TF_BLK_Y[0], TF_BLK_Y[1] - 3.0, ZBLK_C - ZBLK_DZ - BLK_L / 2, ZBLK_C - ZBLK_DZ + BLK_L / 2).union(
        bx(-BLK_W / 2, BLK_W / 2, TF_BLK_Y[0], TF_BLK_Y[1] - 3.0, ZBLK_C + ZBLK_DZ - BLK_L / 2, ZBLK_C + ZBLK_DZ + BLK_L / 2))   # rev F: 2 blocks


def b_nut() -> cq.Workplane:                # tool frame, flange on the ledge
    top = P.TOOL_PLATE_H
    return cyl("z", (SX, TF_SY, 0), NUT_FL_D, top, top + NUT_FL_T).union(
        cyl("z", (SX, TF_SY, 0), NUT_BODY_D, top - NUT_BODY_L, top))


def b_z_rail() -> cq.Workplane:             # local, at rail x = 0
    return bx(-MGN12_RAIL_W / 2, MGN12_RAIL_W / 2, PL_FRONT_Y - MGN12_RAIL_H, PL_FRONT_Y, RAIL_BOT, RAIL_TOP)


def b_motor17() -> cq.Workplane:
    z0 = SHELF_Z0 + SHELF_T
    m = bx(SX - N17 / 2, SX + N17 / 2, SY - N17 / 2, SY + N17 / 2, z0, z0 + N17_L).edges("|Z").chamfer(4.0)
    m = m.union(cyl("z", (SX, SY, 0), N17_BOSS, z0 - 2.0, z0))
    return m.union(cyl("z", (SX, SY, 0), 5.0, z0 - N17_SHAFT_L, z0))


def b_coupler() -> cq.Workplane:
    return cyl("z", (SX, SY, 0), COUPLER_D, SHELF_Z0 - COUPLER_L, SHELF_Z0 - 0.2)


def b_screw() -> cq.Workplane:
    s = cyl("z", (SX, SY, 0), T8_D, Z_SHOULDER, SCREW_Z[1])                 # rev G4: blank/thread above the shoulder
    return s.union(cyl("z", (SX, SY, 0), JOURNAL_D, SCREW_Z[0], Z_SHOULDER))  # D6 journal + M6 end thread


def _ring(d_in, d_out, z0, z1):
    return cyl("z", (SX, SY, 0), d_out, z0, z1).cut(cyl("z", (SX, SY, 0), d_in, z0 - 1, z1 + 1))


def b_bearing_stack() -> cq.Workplane:
    """rev G4: shim + upper F6-14M on the web top, MR126ZZ in the pocket, lower F6-14M under the web"""
    zt = HOUSING_Z[1] - SPOT[1]
    w = _ring(SHIM[0], SHIM[1], zt, zt + SHIM[2]).union(_ring(THRUST[0], THRUST[1], zt + SHIM[2], zt + SHIM[2] + THRUST[2]))
    w = w.union(_ring(BRG_RAD[0], BRG_RAD[1], HOUSING_Z[1] - 8.0, HOUSING_Z[1] - 8.0 + BRG_RAD[2]))
    zb = HOUSING_Z[0] + SPOT[1]
    return w.union(_ring(THRUST[0], THRUST[1], zb - THRUST[2], zb))


def b_collars() -> cq.Workplane:
    """rev G4: DIN 125 M6 washer + ISO 10511 M6 lock nut under the lower thrust bearing"""
    z0 = HOUSING_Z[0] + SPOT[1] - THRUST[2]
    return cyl("z", (SX, SY, 0), NUT_M6[0], z0 - NUT_M6[2] - NUT_M6[1], z0)


def ejector_button(name: str) -> cq.Workplane:
    """rev G: the pipette's own ejector button (reference solid, tool frame), NOMINAL position from params EJECTOR"""
    g, e = hp_geom(name), P.EJECTOR[name]
    top = g["h_z1"] + e["top_above_handle"]
    return cyl("z", (g["hx"] + e["dx"], g["y"] + e["dy"], 0), e["d"], top - 8.0, top)


def eject_finger(i: int) -> cq.Workplane:
    """EJ-1 (CNC 6061, rev G): fixed on CP-1.  Root block (6 x 3 x 29, 2 x M3 from behind CP-1) in the 9 mm band between
    the neighbouring Z carriages, arm (4 wide, 24 deep) along -Y in the -X gap beside tool i, finger (8 x 6) over the
    ejector button.  Underside of the finger Z_EJECT_GAP above the button top at the normal top.  Carriage-local."""
    name = P.TOOL_NAMES[i]
    g, e = hp_geom(name), P.EJECTOR[name]
    ax = AXES_X[i]
    zt = ZB_TOP + g["h_z1"] + e["top_above_handle"] + P.Z_EJECT_GAP     # finger underside, local
    yb = Y_TF + g["y"] + e["dy"]                                          # button centre y, local
    xb = ax + g["hx"] + e["dx"]
    xa0, xa1 = ax - 36.5, ax - 32.5
    root = bx(ax - 38.5, xa1, PL_FRONT_Y - 3.0, PL_FRONT_Y, zt - 25.0, zt + 4.0)
    arm = bx(xa0, xa1, yb - 4.0, PL_FRONT_Y - 2.99, zt - 20.0, zt + 4.0)
    finger = bx(xa0, xb + e["d"] / 2 - 1.0, yb - 4.0, yb + 4.0, zt, zt + 6.0)    # ends 1 mm inside the button edge
    w = root.union(arm).union(finger)
    for x, z in ej_root_holes(i):                                          # rev G3: the 2 x M3 were not modelled
        w = w.cut(cyl("y", (ax + x, 0, z), 2.5, PL_FRONT_Y - 4.0, PL_FRONT_Y + 1.0))
    return w


def _ej_zt(name: str) -> float:
    g, e = hp_geom(name), P.EJECTOR[name]
    return ZB_TOP + g["h_z1"] + e["top_above_handle"] + P.Z_EJECT_GAP


def ej_root_holes(i: int):
    """rev G3: EJ-1 root fixing, 2 x ISO 10642 M3 from behind the plate into the tapped root; (x rel. axis, local z).
    ONE hole pair common to every EJ-1 variant (each root spans zt-25..zt+4; holes >= 3 mm inside every root), so the rig's
    single BR-2 takes any of the three fingers and the machine plate pattern is the same on every axis."""
    zts = [_ej_zt(n) for n in P.EJECTOR if n in P.TOOL_NAMES]
    lo, hi = max(zts) - 22.0, min(zts) + 1.0
    assert hi - lo >= 8.0, (lo, hi)
    return [(-35.5, lo), (-35.5, hi)]


# ---------------- rev G3 (R4): ZHS-1 adjustable mechanical Z hard stop ----------------
# The Z-carriage nut ledge (ZC-1, underside at tool-frame z TOOL_PLATE_H - LEDGE_T) lands on the top of an M5 set screw
# (ISO 4026, hex socket UP for adjustment, locked by an ISO 4035 thin nut) in an Al block that SITS ON the ZBH-1 bearing
# housing top and is located on the plate by 2 x M4.  Force loop at a stall: nut -> ledge -> set screw -> block -> ZBH-1
# housing <- lead screw (axial support in ZBH-1): the plate and the M4 screws carry no stall load while the block is seated.
ZHS = dict(x=(27.8, 42.0), y=(PL_FRONT_Y - 9.0, PL_FRONT_Y), chamfer=3.0,   # G4: 3.0 (clear of the upper thrust bearing)
           scr=(31.5, PL_FRONT_Y - 5.0), scr_d=5.0, scr_l=30.0, tap=4.2, nut_af=8.0, nut_t=2.7, h_min=4.0,
           m4=[(38.5, -160.0), (38.5, -176.0)], cbore=(8.0, 4.4))
ZHS_AXES = [i for i, n in enumerate(P.TOOL_NAMES) if n in P.Z_HARD_STOP_TOOLS]


def ledge_under_z(s: float) -> float:
    """local z of the ZC-1 nut-ledge underside at stroke s"""
    return zb_of(s) + P.TOOL_PLATE_H - LEDGE_T


ZHS["z"] = (HOUSING_Z[1], ZB_TOP - P.Z_HARD_STOP_RANGE[1] + P.TOOL_PLATE_H - LEDGE_T - ZHS["h_min"])   # block on ZBH-1 top


def z_stop_block() -> cq.Workplane:
    """ZHS-1 (CNC 6061-T6), carriage-local at rail x = 0"""
    (x0, x1), (y0, y1), (z0, z1) = ZHS["x"], ZHS["y"], ZHS["z"]
    w = bx(x0, x1, y0, y1, z0, z1)
    c = ZHS["chamfer"]                                       # corner toward the nut body: clearance to the Tr8 nut at full stroke
    w = w.cut(cq.Workplane(obj=cq.Solid.extrudeLinear(cq.Wire.makePolygon(
        [cq.Vector(x0 - 1, y0 + c + 1, z0 - 1), cq.Vector(x0 - 1, y0 - 1, z0 - 1), cq.Vector(x0 + c + 1, y0 - 1, z0 - 1)], close=True),
        [], cq.Vector(0, 0, z1 - z0 + 2))))
    sx, sy = ZHS["scr"]
    w = w.cut(cyl("z", (sx, sy, 0), ZHS["tap"], z0 - 1, z1 + 1))              # M5 tapped through (tap drill 4.2)
    for x, z in ZHS["m4"]:
        w = w.cut(cyl("y", (x, 0, z), 4.5, y0 - 1, y1 + 1))                   # M4 clearance
        w = w.cut(cyl("y", (x, 0, z), ZHS["cbore"][0], y0 - 1, y0 + ZHS["cbore"][1]))   # counterbore from the front
    return w


def zhs_protrusion(s_set: float) -> float:
    return ledge_under_z(s_set) - ZHS["z"][1]


def z_stop_screw(s_set: float) -> cq.Workplane:
    """ISO 4026 M5 x 30 set screw + ISO 4035 M5 thin lock nut, screw top at the ledge underside for stroke s_set"""
    sx, sy = ZHS["scr"]
    top = ledge_under_z(s_set)
    s = cyl("z", (sx, sy, 0), ZHS["tap"], top - ZHS["scr_l"], top)            # modelled at the tap-drill diameter in the block
    s = s.union(cyl("z", (sx, sy, 0), ZHS["scr_d"], ZHS["z"][1], top))
    return s.union(cyl("z", (sx, sy, 0), ZHS["nut_af"], ZHS["z"][1], ZHS["z"][1] + ZHS["nut_t"]))


def cam_bar() -> cq.Workplane:
    """ZUC-1 (rev G): bar on the Z-carriage top (tool frame) that holds the side-actuated Z-up roller switch made from the
    normal top up to the eject top; it slides past the roller during the eject overtravel"""
    top = P.TOOL_PLATE_H
    # rev G1: two bars (y 5.5..8.5 in the tool frame, clear of the nut flange and of both switch bodies): +X bar for the Z-up
    # roller switch, -X bar for the HOME roller switch; both made from the NORMAL top up to the eject top
    zup = bx(8.5, 11.0, TF_ZC[0] + 0.5, TF_ZC[0] + 3.5, top, top + EJ + 9.0)
    home = bx(-28.0, -25.5, TF_ZC[0] + 0.5, TF_ZC[0] + 3.5, top, top + EJ + 11.0)
    return zup.union(home)


def b_zup() -> cq.Workplane:
    """integration rev E: D2F 'Z up' switch (made only at the top of travel; lever on the Z-carriage top face), x -3..9.8"""
    z_sw = ZB_TOP + P.TOOL_PLATE_H + SW_TRIGGER_GAP + 1.2 + EJ   # rev G: switches at the EJECT top
    y0 = PL_FRONT_Y - 14.0
    return bx(-3.0, 9.8, y0 - 6.5, y0 - 0.7, z_sw, z_sw + 6.5)


def b_switch() -> cq.Workplane:
    z_sw = ZB_TOP + P.TOOL_PLATE_H + SW_TRIGGER_GAP + 1.2 + EJ   # rev G: switches at the EJECT top
    y0 = PL_FRONT_Y - 14.0
    s = bx(-24, -4, y0 - 6.5, y0, z_sw, z_sw + 10.0)
    return s.union(bx(-16, -12, y0 - 5.0, y0 - 1.5, z_sw - 1.2, z_sw))


STIFF_RIVNUT_D = 7.0      # rev G2: M5 steel rivnut, 7.0 mm hole in the 2 mm front wall


def b_stiffener() -> cq.Workplane:
    """steel tube 40 x 20 x 2 (rev F); rev G2: the six M5 rivnut holes (x +/-10 at STIFF_BOLT_Z) in the wall on CP-1"""
    w = bx(-20, 20, PL_BACK_Y, PL_BACK_Y + 20.0, STIFF_Z[0], STIFF_Z[1])
    w = w.cut(bx(-18, 18, PL_BACK_Y + 2.0, PL_BACK_Y + 18.0, STIFF_Z[0] - 1, STIFF_Z[1] + 1))
    for sx in (-1, 1):
        for z in STIFF_BOLT_Z:
            w = w.cut(cyl("y", (sx * 10.0, 0, z), STIFF_RIVNUT_D, PL_BACK_Y - 1, PL_BACK_Y + 3))
    return w


# ======================================================================================
# 3. gripper tool (tool frame; centre tool plate)
# ======================================================================================
def gr_bracket() -> cq.Workplane:
    """GR-1 (CNC 6061): back plate on the tool plate, arm over the labware, motor flange, and two
    bearing cheeks that carry the L/R screw at its centre (screw driven at the centre by a GT2 belt)."""
    w = bx(-31, 31, -8, 0, 0, P.TOOL_PLATE_H)
    w = w.union(bx(-30, 30, G_YC - 52.0, -8, 0, 8))      # integration rev D: arm ends flush with the open front finger (-107)
    w = w.union(bx(-28, 4, G_YC + 4.0, G_YC + 8.0, 8, 40))                       # motor flange
    for s in (-1, 1):
        w = w.union(bx(-28, 2, G_YC + s * 4.0, G_YC + s * 9.0, -26, 0.5))          # bearing cheeks
        w = w.cut(cyl("y", (G_SCREW_X, 0, G_SCREW_Z), BRG_688[1], G_YC + s * 3.9, G_YC + s * 9.1))
    w = w.cut(bx(G_SCREW_X - 7, G_SCREW_X + 7, G_YC - 3.9, G_YC + 3.9, -1, 9))     # belt slot
    w = w.cut(cyl("y", (G_SCREW_X, 0, G_MOTOR_Z), 22.5, G_YC + 3, G_YC + 9))       # NEMA11 boss
    for sx in (-1, 1):
        for sz in (-1, 1):
            w = w.cut(cyl("y", (G_SCREW_X + sx * N11_HOLES / 2, 0, G_MOTOR_Z + sz * N11_HOLES / 2), 2.9, G_YC + 3, G_YC + 9))
    for x, z in P.TOOL_BOLTS:
        w = w.cut(cyl("y", (x, 0, TP_ZC + z), 4.5, -9, 1))
        w = w.cut(cyl("y", (x, 0, TP_ZC + z), 8.0, -9, -4.5))
    for x, z in P.TOOL_DOWELS:
        w = w.cut(cyl("y", (x, 0, TP_ZC + z), 3.0, -9, 1))
    for k in range(4):                                                             # MGN9 rail M3 tapped
        w = w.cut(cyl("z", (G_RAIL_X, G_YC - 30 + 20 * k, 0), 2.5, -1, 8))
    return w


def gr_switch_bracket() -> cq.Workplane:
    """GR-4 (MJF PA12, integration rev D, CR-04): jaws-open D2F switch under the GR-1 arm, beside the rear carrier."""
    w = bx(24.0, 32.0, -21.0, -8.0, -10.0, 0.0)
    return w.cut(cyl("x", (0, -11.0, -5.0), 1.6, 23, 33)).cut(cyl("x", (0, -17.5, -5.0), 1.6, 23, 33))


def b_d2f(x0, y0, z0, along="y") -> cq.Workplane:
    """D2F-class subminiature switch envelope 12.8 x 5.8 x 6.5 (lever not modelled)"""
    if along == "y":
        return bx(x0, x0 + 6.5, y0, y0 + 12.8, z0, z0 + 5.8)
    return bx(x0, x0 + 12.8, y0, y0 + 6.5, z0, z0 + 5.8)


def gr_carrier(s: int, gap: float) -> cq.Workplane:
    """GR-2 (CNC 6061) jaw carrier; s = +1 rear (toward the tool plate), -1 front."""
    o = gap / 2 + G_PAD_T
    y_in, y_out = G_YC + s * (o - G_CARRIER_L), G_YC + s * o
    w = bx(-28, 22, y_in, y_out, -26, -MGN9C_H)
    w = w.cut(cyl("y", (G_SCREW_X, 0, G_SCREW_Z), 10.5, y_in - 1 * s, y_out + 1 * s))
    yc = (y_in + y_out) / 2
    for sx in (-1, 1):
        for sy in (-1, 1):
            w = w.cut(cyl("z", (G_RAIL_X + sx * 7.5, yc + sy * 5.0, 0), 3.4, -27, -9))
    for x in (-12.0, 12.0):
        w = w.cut(cyl("y", (x, 0, -18.0), 2.5, y_out - 8 * s, y_out + s))         # finger M3 tapped
    return w


def gr_finger_flat() -> cq.Workplane:
    """GR-3 (laser 5052, 4 mm): x across, flat y = tool-frame z (top at -MGN9C_H)."""
    f = cq.Workplane("XY").center(0, (-G_FINGER_L - MGN9C_H) / 2).rect(40.0, G_FINGER_L - MGN9C_H).extrude(G_BLADE_T)
    f = f.edges("|Z").fillet(3.0)
    return holes_flat(f, [(-12.0, -18.0), (12.0, -18.0)], 3.4)


def gr_finger(s: int, gap: float) -> cq.Workplane:
    o = gap / 2 + G_PAD_T
    f = gr_finger_flat().rotate((0, 0, 0), (1, 0, 0), 90)       # thickness toward -y, y in [-4, 0]
    return f.translate((0, G_YC + s * o + (G_BLADE_T if s > 0 else 0.0), 0))


def b_gr_pad(s: int, gap: float) -> cq.Workplane:
    return bx(-18, 18, G_YC + s * gap / 2, G_YC + s * (gap / 2 + G_PAD_T), -G_FINGER_L + 0.5, -G_FINGER_L + 16.5)


def b_gr_rail() -> cq.Workplane:
    return bx(G_RAIL_X - MGN9_RAIL_W / 2, G_RAIL_X + MGN9_RAIL_W / 2, G_YC - 50, G_YC + 50, -MGN9_RAIL_H, 0)


def b_gr_block(s: int, gap: float) -> cq.Workplane:
    o = gap / 2 + G_PAD_T
    yc = G_YC + s * (o - G_CARRIER_L / 2)
    return bx(G_RAIL_X - MGN9C_W / 2, G_RAIL_X + MGN9C_W / 2, yc - MGN9C_L / 2, yc + MGN9C_L / 2, -MGN9C_H, -2.0)


def b_gr_nut(s: int, gap: float) -> cq.Workplane:
    o = gap / 2 + G_PAD_T
    y_in = G_YC + s * (o - G_CARRIER_L)
    fl = cyl("y", (G_SCREW_X, 0, G_SCREW_Z), NUT_FL_D, y_in, y_in - s * 3.5)
    return fl.union(cyl("y", (G_SCREW_X, 0, G_SCREW_Z), NUT_BODY_D, y_in, y_in + s * 11.5))


def b_gr_screw() -> cq.Workplane:
    return cyl("y", (G_SCREW_X, 0, G_SCREW_Z), T8_D, G_YC - 40, G_YC + 40)


def b_gr_motor() -> cq.Workplane:
    m = bx(G_SCREW_X - N11 / 2, G_SCREW_X + N11 / 2, G_YC + 8.0, G_YC + 40.0, G_MOTOR_Z - N11 / 2, G_MOTOR_Z + N11 / 2)
    return m.union(cyl("y", (G_SCREW_X, 0, G_MOTOR_Z), 5.0, G_YC - 6.0, G_YC + 8.0))

# ======================================================================================
# 4a. generic pipette-module adapter (tool frame; left tool plate)
# ======================================================================================
ADJ = 2.0                          # +/- slotted adjustment in X (back plate) and Y (V-blocks)
PA_SHELF_Z = ((20.0, 26.0), (104.0, 110.0))
PA_VB_Y = (-10.0, -40.0)


def mod_centre_y() -> float:
    y_v = -19.7                     # V vertex (90 deg) of PA-2
    return y_v - MOD_D / 2 * math.sqrt(2)


def slot_y(x, z, w, travel, y0, y1) -> cq.Workplane:
    """Through slot along y (plate in X-Z), long in x by `travel`."""
    s = bx(x - travel / 2, x + travel / 2, y0, y1, z - w / 2, z + w / 2)
    return s.union(cyl("y", (x - travel / 2, 0, z), w, y0, y1)).union(cyl("y", (x + travel / 2, 0, z), w, y0, y1))


def _xslots(w, y0, y1):
    """X-adjust slots for the tool bolts (M4) and the dowels (D3 sliding fit) through a plate y0..y1."""
    for x, z in P.TOOL_BOLTS:
        w = w.cut(slot_y(x, TP_ZC + z, 4.5, 2 * ADJ, y0 - 1, y1 + 1))
    for x, z in P.TOOL_DOWELS:
        w = w.cut(slot_y(x, TP_ZC + z, 3.0, 2 * ADJ, y0 - 1, y1 + 1))
    return w


def pa_base() -> cq.Workplane:
    """PA-1 (CNC 6061): back plate with X slots, two shelves with a front U-cut for the module."""
    w = bx(-31, 31, -6, 0, 0, P.TOOL_PLATE_H)
    for z0, z1 in PA_SHELF_Z:
        sh = bx(-35, 35, -64, -6, z0, z1)
        sh = sh.cut(bx(-24, 24, -65, -26, z0 - 1, z1 + 1))
        for x in (-30.0, 30.0):
            for y in (-14.0, -32.0):
                sh = sh.cut(cyl("z", (x, y, 0), 3.3, z0 - 1, z1 + 1))        # M4 tapped
        w = w.union(sh)
    return _xslots(w, -6, 0)


def _v_notch(y_vertex: float, opens: int, half_w: float, z0: float, z1: float) -> cq.Workplane:
    """90-degree V notch prism, vertex at y_vertex, opening toward sign(opens) in y."""
    pts = [(0.0, y_vertex), (-half_w, y_vertex + opens * half_w), (half_w, y_vertex + opens * half_w)]
    return cq.Workplane("XY").polyline(pts).close().extrude(z1 - z0 + 2).translate((0, 0, z0 - 1))


def pa_vblock(z0: float) -> cq.Workplane:
    """PA-2 (CNC 6061): 12 mm V-block on a shelf; Y slots give +/-2 mm; strap bolts tapped M4."""
    z1 = z0 + 12.0
    w = bx(-35, 35, PA_VB_Y[1], PA_VB_Y[0], z0, z1)
    w = w.cut(_v_notch(-19.7, -1, 40.0, z0, z1))
    for x in (-30.0, 30.0):
        for y in (-14.0, -32.0):
            w = w.cut(cq.Workplane("XY").center(x, y).slot2D(4.5 + 2 * ADJ, 4.5, 90).extrude(14).translate((0, 0, z0 - 1)))
    for x in (-28.0, 28.0):
        w = w.cut(cyl("y", (x, 0, z0 + 6.0), 3.3, PA_VB_Y[1] - 1, PA_VB_Y[1] + 10))
    return w


def pa_strap(z0: float) -> cq.Workplane:
    """PA-3 (CNC 6061): opposing V strap, pulled onto the module by 2 x M4 into the V-block."""
    z1 = z0 + 12.0
    yv = mod_centre_y() - MOD_D / 2 * math.sqrt(2)
    w = bx(-35, 35, -82.0, -58.0, z0, z1)
    w = w.cut(_v_notch(yv, 1, 40.0, z0, z1))
    for x in (-28.0, 28.0):
        w = w.cut(cyl("y", (x, 0, z0 + 6.0), 4.5, -83, -57))
    return w


def b_module() -> cq.Workplane:
    yc = mod_centre_y()
    m = cyl("z", (0, yc, 0), MOD_D, -NOZZLE_BELOW_TP + MOD_CONE_L, MOD_TOP)
    return m.union(cyl("z", (0, yc, 0), 8.0, -NOZZLE_BELOW_TP, -NOZZLE_BELOW_TP + MOD_CONE_L))


# ======================================================================================
# 4b. hand-pipette tool (tool frame) -- the pipetting tool for all three pipette positions
# ======================================================================================
HP_X = -5.0                        # pipette handle axis x (layout)
HP_BACK_Z0 = -70.0                 # HP-1 back plate extends below the tool plate to reach the clamps (8-channel)
# integration rev D: per-tool handle x and (single channel) nozzle y so the nozzle sits on the tool reference
HP_XS = {"pipette_small": 1.5}     # others keep -5
HP_YOKE_T = 6.0
HP_ACT_DY, HP_ACT_X = 30.0, 18.0   # plunger linear stepper axis: in front of the handle by 30 mm, at x = 18
HP_VB_H = 14.0
HP_EJ_RESERVED = "x -35..-12, y = handle - 30 +/- 11, z >= 60: kept free for an optional NEMA8 ejector actuator"
# rev J (ease of build, ADR-003 #5): ONE machined cradle body CB-1 per pipette replaces the HP-1 back plate, the HP-4 plunger-
# actuator bracket, the HP-8 collar stop and the HP-6 plunger-switch bracket.  The actuator axis, the collar-stop face and the
# switch shelf are machined in (no by-feel alignment); the body is located on the two TP-1 dowels (round hole on the upper
# dowel, Z slot on the lower one: no X adjustment, one way only) and carries an engraved tool tag.  Kept separate: V-blocks
# HP-2 (shimmable), straps HP-3, eight-channel clamp HP-2B/HP-3B, collar HP-7, yoke HP-5 (HP-5R on the rig), EJ-1.
# Machining: TWO set-ups, both 3-axis with standard cutters: OP1 tool axis -Y (from the front, back face = datum A on
# parallels): every face, the stop U-slot, every Y hole; OP2 tool axis -Z (from the top, datum A on the fixed jaw, side face B
# on a stop): the actuator bore + 4 holes and the 2 vertical M2 switch holes.  The back face has no feature (the upper V-block
# screws now come from the FRONT through counterbored ears on HP-2, the lower clamp screws from behind into plain holes below
# TP-1), so it can be the face of cast tooling plate.
CB_BACK_T = 6.0                                    # back plate thickness: as HP-1 (the M1 budget term is unchanged)
CB_VB_EAR = dict(h=10.0, dx=(12.0, 22.0), screw_dx=17.0, cb_d=8.0)   # upper V-block ears with front screws
CB_ACCESS_D = 8.0                                  # hex-key / head access bore in front of every TP-1 bolt head
CB_STOCK = "cast aluminium tooling plate (5083-class, E 70 GPa, both faces milled flat 0.1 mm), or 6061-T6 with the back face skimmed in OP1"


def hp_geom(name: str) -> dict:
    g = dict(HP_SETS[name])
    noz = -NOZZLE_BELOW_TP
    if g["eight"]:
        g["y"] = -6.0 - g["m_w"] / 2                        # manifold back face on the back plate front face
        g["m_z"] = (noz + g["nozzle_l"], noz + g["nozzle_l"] + g["m_h"])
        g["h_z0"] = g["m_z"][1]
        g["clamp_lo_z"] = (g["m_z"][0] + g["m_h"] / 2) - HP_VB_H / 2
    else:
        g["y"] = getattr(P, "TOOL_REFERENCE_LOCAL_Y", -92.0) - Y_TF   # integration rev D: nozzle at the tool reference (was -45)
        g["h_z0"] = noz + g["shaft_l"]
        g["clamp_lo_z"] = g["h_z0"] + 4.0
    g["h_z1"] = g["h_z0"] + g["body_l"]
    g["button_top"] = g["h_z1"] + g["plunger_above"]
    g["clamp_hi_z"] = g["h_z0"] + 0.55 * g["body_l"]
    g["act"] = (HP_ACT_X, g["y"] - HP_ACT_DY)
    g["plate_z"] = g["button_top"] + 1.0 - g["stroke"] - 4.0 - 6.0   # underside of the actuator head plate
    g["hx"] = HP_XS.get(name, HP_X)                                  # integration rev D
    g["back_z0"] = HP_BACK_Z0 if g["eight"] else g["clamp_lo_z"] - 3.0   # integration rev D: slim single-channel cradle
    return g


def hp_back_flat(name: str) -> cq.Workplane:
    """HP-1 (laser 5052, 6 mm): cradle back plate, X slots (+/-2 mm) on the tool bolts and dowels."""
    g = hp_geom(name)
    z0, z1 = g["back_z0"], P.TOOL_PLATE_H
    f = cq.Workplane("XY").center(0, (z0 + z1) / 2).rect(P.TOOL_PLATE_W, z1 - z0).extrude(6.0).edges("|Z").fillet(3.0)
    for x, z in P.TOOL_BOLTS:
        f = f.cut(cq.Workplane("XY").center(x, TP_ZC + z).slot2D(4.5 + 2 * ADJ, 4.5).extrude(8).translate((0, 0, -1)))
    for x, z in P.TOOL_DOWELS:
        f = f.cut(cq.Workplane("XY").center(x, TP_ZC + z).slot2D(3.0 + 2 * ADJ, 3.0).extrude(8).translate((0, 0, -1)))
    pts = [(g["hx"] + sx * 16.0, g["clamp_hi_z"] + HP_VB_H / 2) for sx in (-1, 1)]
    if g["eight"]:
        pts += [(g["hx"] + sx * (g["m_t"] / 2 + 3.0), g["clamp_lo_z"] + HP_VB_H / 2) for sx in (-1, 1)]
    else:
        pts += [(g["hx"] + sx * 16.0, g["clamp_lo_z"] + HP_VB_H / 2) for sx in (-1, 1)]
    pts += [(23.0, g["plate_z"] - 22.0), (29.0, g["plate_z"] - 8.0)]
    pts += hp8_holes(name)                                       # rev G3: HP-8 collar stop, 2 x M4 (were not modelled)
    # rev G3: holes that lie behind TP-1 (tool-frame z 0..TOOL_PLATE_H) take ISO 10642 heads, countersunk in the HP-1 BACK face
    # (a socket head there would sit between HP-1 and TP-1); the others are plain clearance holes for ISO 4762
    under_tp = [q for q in pts if 0.0 <= q[1] <= P.TOOL_PLATE_H]
    f = holes_flat(f, [q for q in pts if q not in under_tp], 4.5)
    f = csk_flat_back(f, under_tp, 4.5, 9.0, 6.0)
    return holes_flat(f, force_switch_holes(name), 1.6)          # rev G2: M2 tapped, force switch


def hp_back_screws(name: str) -> dict:
    """rev G3: HP-1 screw positions (flat x, z) by joint, for the fastener schedule"""
    g = hp_geom(name)
    hi = [(g["hx"] + sx * 16.0, g["clamp_hi_z"] + HP_VB_H / 2) for sx in (-1, 1)]
    lo = ([(g["hx"] + sx * (g["m_t"] / 2 + 3.0), g["clamp_lo_z"] + HP_VB_H / 2) for sx in (-1, 1)] if g["eight"]
          else [(g["hx"] + sx * 16.0, g["clamp_lo_z"] + HP_VB_H / 2) for sx in (-1, 1)])
    act = [(23.0, g["plate_z"] - 22.0), (29.0, g["plate_z"] - 8.0)]
    return {"vblock_hi": hi, "lower": lo, "hp4": act, "hp8": hp8_holes(name)}


def hp8_holes(name: str):
    g = hp_geom(name)
    z0, _ = _collar_z(g)
    return [(g["hx"] + sx * 18.0, z0 - 3.0) for sx in (-1, 1)]


HP_RIB = {"left": 13.0, "right": 5.0, "t": 6.0, "enabled": False}   # rev F: ribs removed (TC-1 clash), see hp_back


def hp_back(name: str) -> cq.Workplane:
    g = hp_geom(name)
    w = hp_back_flat(name).rotate((0, 0, 0), (1, 0, 0), 90)
    t = HP_RIB["t"]
    x0 = -P.TOOL_PLATE_W / 2
    # integration rev F: rear ribs REMOVED after the whole-machine check (they reached 13 mm behind the tool plate and hit
    # the TC-1 bridge/presser/clamp motor with a tool lowered at B1); the combined budget shows the HP-1 part below the tool
    # plate carries no moment (the axial pick-up load enters at the upper V-block, inside the tool-plate/Z-carriage stack)
    if not HP_RIB.get("enabled"):
        return w
    w = w.union(bx(x0, x0 + t, 0.0, HP_RIB["left"], g["back_z0"], -0.2))
    return w.union(bx(-x0 - t, -x0, 0.0, HP_RIB["right"], g["back_z0"], -0.2))


def _collar_z(g):
    zt = g["clamp_hi_z"] - 2.0          # 2 mm disc-spring stack between collar and the upper V-block
    return zt - 8.0, zt


def hp_collar(name: str) -> cq.Workplane:
    """HP-7 (CNC 6061, split clamp collar): axial datum on the handle (rev F, M13).  Bears DOWN on the HP-8 stop (strip load)
    and UP through a preloaded disc-spring stack on the upper V-block (pick-up load); ID = measured handle diameter."""
    g = hp_geom(name)
    z0, z1 = _collar_z(g)
    r = g["body_d"] / 2
    c = cyl("z", (g["hx"], g["y"], 0), g["body_d"] + 14.0, z0, z1).cut(cyl("z", (g["hx"], g["y"], 0), g["body_d"], z0 - 1, z1 + 1))
    c = c.cut(bx(g["hx"] - 0.5, g["hx"] + 0.5, g["y"] - r - 8, g["y"] - r + 1, z0 - 1, z1 + 1))   # split
    # rev G4: tangential M3 clamp screw across the split (clearance on -X, tapped on +X, spot face for the head)
    ys, zs = g["y"] - r - 3.5, (z0 + z1) / 2
    c = c.cut(cyl("x", (0, ys, zs), 3.4, g["hx"] - 20.0, g["hx"]))
    c = c.cut(cyl("x", (0, ys, zs), 2.5, g["hx"], g["hx"] + 20.0))
    return c.cut(cyl("x", (0, ys, zs), 6.5, g["hx"] - 20.0, g["hx"] - 8.0))


def hp_stop(name: str) -> cq.Workplane:
    """HP-8 (CNC 6061): hard stop under the collar, bolted to HP-1 (2 x M4)"""
    g = hp_geom(name)
    z0, _ = _collar_z(g)
    w = bx(g["hx"] - 22, g["hx"] + 22, g["y"], -6.0, z0 - 6.0, z0)
    w = w.cut(cyl("z", (g["hx"], g["y"], 0), g["body_d"] + 1.0, z0 - 7, z0 + 1))
    for x, z in hp8_holes(name):                                  # rev G3: 2 x M4 tapped from HP-1 (were not modelled)
        w = w.cut(cyl("y", (x, 0, z), 3.3, -16.0, -5.0))
    return w


def hp_springs(name: str) -> cq.Workplane:
    g = hp_geom(name)
    _, z1 = _collar_z(g)
    return cyl("z", (g["hx"], g["y"], 0), g["body_d"] + 10.0, z1, z1 + 2.0).cut(cyl("z", (g["hx"], g["y"], 0), g["body_d"] + 0.5, z1 - 1, z1 + 3))


def hp_force_switch(name: str) -> cq.Workplane:
    """D2F 'pick-up force reached' switch beside the collar: trips when the collar lifts 0.5 mm against the springs (M14)"""
    g = hp_geom(name)
    z0, z1 = _collar_z(g)
    x0 = g["hx"] + g["body_d"] / 2 + 7.0 + 1.0
    return bx(x0, x0 + 12.8, -11.8, -6.0, z1 - 3.0, z1 + 3.5)   # rev G2: body ON the HP-1 front face (was 0.2 mm off), 2 x M2 into HP-1


def force_switch_holes(name: str):
    """rev G2: the two M2 tapped holes in HP-1 for the force switch (flat coords x, z)"""
    g = hp_geom(name)
    _, z1 = _collar_z(g)
    x0 = g["hx"] + g["body_d"] / 2 + 8.0
    return [(x0 + 3.15, z1 + 0.25), (x0 + 3.15 + 6.5, z1 + 0.25)]


def vb_ear_front_y(name: str) -> float:
    """rev J: front face of the upper V-block ears: 1 mm behind the handle surface at |dx| = 12"""
    g = hp_geom(name)
    r = g["body_d"] / 2
    return g["y"] + math.sqrt(max(r * r - CB_VB_EAR["dx"][0] ** 2, 0.0)) + 1.0


def vb_ear_screws(name: str):
    """rev J: the two upper V-block screws (x, z), from the front into CB-1 (tapped M4)"""
    g = hp_geom(name)
    z = g["clamp_hi_z"] + HP_VB_H + CB_VB_EAR["h"] / 2
    return [(g["hx"] + sx * CB_VB_EAR["screw_dx"], z) for sx in (-1, 1)]


ISO4762_L = (8, 10, 12, 16, 20, 25, 30, 35, 40, 45, 50)


def vb_ear_screw_len(name: str) -> float:
    """rev J: ISO 4762 M4 length for the upper V-block: the longest standard length that leaves >= 4 mm of counterbore for the
    head; the tip stops 0.5 mm short of the CB-1 back face (engagement in CB-1 = CB_BACK_T - 0.5 = 5.5 mm)"""
    room = (0.0 - 0.5) - vb_ear_front_y(name) - 4.0
    return float(max(L for L in ISO4762_L if L <= room))


def vb_ear_cb_floor(name: str) -> float:
    return -0.5 - vb_ear_screw_len(name)


def hp_vblock(name: str, z0: float) -> cq.Workplane:
    """HP-2 (CNC 6061): V-block on the handle; V vertex from body_d (measure on the part).
    rev J: the UPPER V-block carries two ears above the V (|dx| 12..22) with counterbored M4 screws from the FRONT into CB-1;
    the lower one (single channel) keeps its tapped holes for screws from behind CB-1 (below TP-1)."""
    g = hp_geom(name)
    r = g["body_d"] / 2
    z1 = z0 + HP_VB_H
    upper = getattr(P, "REV_J", False) and abs(z0 - g["clamp_hi_z"]) < 1e-6
    w = bx(g["hx"] - 22, g["hx"] + 22, g["y"] + 3.0, -6.0, z0, z1)
    w = w.cut(_v_notch(g["y"] + r * math.sqrt(2), -1, 30.0, z0, z1).translate((g["hx"], 0, 0)))
    for sx in (-1, 1):
        if not upper:
            w = w.cut(cyl("y", (g["hx"] + sx * 16.0, 0, z0 + 7.0), 3.3, -14, -5))
        w = w.cut(cyl("y", (g["hx"] + sx * 18.5, 0, z0 + 7.0), 3.3, g["y"] + 2.0, g["y"] + 12.0))
    if upper:
        ye = vb_ear_front_y(name)
        a, b = CB_VB_EAR["dx"]
        for sx in (-1, 1):
            x0, x1 = sorted((g["hx"] + sx * a, g["hx"] + sx * b))
            w = w.union(bx(x0, x1, ye, -6.0, z1 - 0.01, z1 + CB_VB_EAR["h"]))
        for x, z in vb_ear_screws(name):
            w = w.cut(cyl("y", (x, 0, z), 4.5, ye - 1.0, -5.0))
            w = w.cut(cyl("y", (x, 0, z), CB_VB_EAR["cb_d"], ye - 1.0, vb_ear_cb_floor(name)))
    return w


def hp_strap(name: str, z0: float) -> cq.Workplane:
    """HP-3 (CNC 6061): opposing V strap, 2 x M4 into the V-block."""
    g = hp_geom(name)
    r = g["body_d"] / 2
    z1 = z0 + HP_VB_H
    w = bx(g["hx"] - 22, g["hx"] + 22, g["y"] - 3.0, g["y"] - 25.0, z0, z1)
    w = w.cut(_v_notch(g["y"] - r * math.sqrt(2), 1, 30.0, z0, z1).translate((g["hx"], 0, 0)))
    for sx in (-1, 1):
        w = w.cut(cyl("y", (g["hx"] + sx * 18.5, 0, z0 + 7.0), 4.5, g["y"] - 26, g["y"] - 2))
    return w


def hp8_clamp(name: str) -> cq.Workplane:
    """HP-2B (CNC 6061): 8-channel manifold clamp, U open to the front.  The manifold's back face bears on
    the HP-1 front face; the pocket (m_t x m_w, measure on the part) fixes the nozzle row along Y."""
    g = hp_geom(name)
    z0 = g["clamp_lo_z"]
    xi, yb, yf = g["m_t"] / 2 + 0.2, -6.0, -6.0 - g["m_w"] - 0.4
    w = bx(g["hx"] - xi - 6, g["hx"] + xi + 6, yf, yb, z0, z0 + HP_VB_H)
    w = w.cut(bx(g["hx"] - xi, g["hx"] + xi, yf - 1, yb + 1, z0 - 1, z0 + HP_VB_H + 1))
    for sx in (-1, 1):
        x = g["hx"] + sx * (xi + 3.0)
        w = w.cut(cyl("y", (x, 0, z0 + 7.0), 3.3, yb - 10, yb + 1))     # M4 tapped, from HP-1
        w = w.cut(cyl("y", (x, 0, z0 + 7.0), 3.3, yf - 1, yf + 10))     # M4 tapped, front bar
    return w


def hp8_bar(name: str) -> cq.Workplane:
    """HP-3B (CNC 6061): front bar of the 8-channel clamp; 2 x M4 pull the manifold onto HP-1."""
    g = hp_geom(name)
    z0 = g["clamp_lo_z"]
    xi, yf = g["m_t"] / 2 + 0.2, -6.0 - g["m_w"] - 0.4
    w = bx(g["hx"] - xi - 6, g["hx"] + xi + 6, yf - 6.0, yf, z0, z0 + HP_VB_H)
    for sx in (-1, 1):
        w = w.cut(cyl("y", (g["hx"] + sx * (xi + 3.0), 0, z0 + 7.0), 4.5, yf - 7, yf + 1))
    return w


def hp_act_bracket(name: str) -> cq.Workplane:
    """HP-4 (CNC 6061): plunger linear stepper mount.  NEMA11 captive actuator hangs below the head
    plate in front of the handle, rod up; web on HP-1, strip beside the handle."""
    g = hp_geom(name)
    pz = g["plate_z"]
    z1 = pz + 6.0
    ax_, ay = g["act"]
    w = bx(11, 35, -12, -6, pz - 30.0, z1)
    w = w.union(bx(11, 35, ay + 15, -12, pz, z1))
    w = w.union(bx(ax_ - 15, ax_ + 15, ay - 15, ay + 15.5, pz, z1))
    w = w.cut(cyl("z", (ax_, ay, 0), 22.5, pz - 1, z1 + 1))
    for sx in (-1, 1):
        for sy in (-1, 1):
            w = w.cut(cyl("z", (ax_ + sx * N11_HOLES / 2, ay + sy * N11_HOLES / 2, 0), 2.9, pz - 1, z1 + 1))
    for x, z in ((23.0, pz - 22.0), (29.0, pz - 8.0)):
        w = w.cut(cyl("y", (x, 0, z), 3.3, -13, -5))
    x, y = hp6_foot_hole(name)
    return w.cut(cyl("z", (x, y, 0), 2.4, pz - 1, z1 + 1))                   # rev G4: HP-6 fixing (M2 from below)


def hp_plunger_yoke(name: str) -> cq.Workplane:
    """HP-5 (CNC 6061): yoke from the actuator rod to the plunger button (rests 1 mm above it)."""
    g = hp_geom(name)
    z0 = g["button_top"] + 1.0
    ax_, ay = g["act"]
    w = bx(g["hx"] - g["button_d"] / 2, ax_ + 7, g["y"] + 6, ay - 6, z0, z0 + HP_YOKE_T)
    return w.cut(cyl("z", (ax_, ay, 0), 2.5, z0 - 1, z0 + 4))


def b_pipette(name: str) -> cq.Workplane:
    g = hp_geom(name)
    noz = -NOZZLE_BELOW_TP
    x, y = g["hx"], g["y"]
    if g["eight"]:
        p = bx(x - g["m_t"] / 2, x + g["m_t"] / 2, y - g["m_w"] / 2, y + g["m_w"] / 2, g["m_z"][0], g["m_z"][1])
        for k in range(8):
            p = p.union(cyl("z", (x, y + (k - 3.5) * P.WELL_PITCH_96, 0), 5.0, noz, g["m_z"][0] + 0.5))
    else:
        # integration rev D: lowest 20 mm modelled as the 4.5 mm tip cone (strip-comb slot 5.4 passes it), 7 mm shaft above
        p = cyl("z", (x, y, 0), 4.5, noz, noz + 20.0).union(cyl("z", (x, y, 0), 7.0, noz + 20.0, g["h_z0"]))
    p = p.union(cyl("z", (x, y, 0), g["body_d"], g["h_z0"] - 0.01, g["h_z1"]))
    p = p.union(cyl("z", (x, y, 0), 8.0, g["h_z1"], g["button_top"] - 5.0))
    return p.union(cyl("z", (x, y, 0), g["button_d"], g["button_top"] - 5.0, g["button_top"]))


def _hp_sw_box(name):
    g = hp_geom(name)
    y0 = g["y"] + 6.0 + 0.5
    zt = g["button_top"] + 1.0 - 0.5                 # 0.5 mm under the yoke at rest (released)
    return g, y0, zt


def hp_switch(name: str) -> cq.Workplane:
    g, y0, zt = _hp_sw_box(name)
    return bx(18.0, 30.8, y0, y0 + 6.5, zt - 5.8, zt)


def hp_switch_bracket(name: str) -> cq.Workplane:
    """HP-6 (MJF PA12): post on the HP-4 strip carrying the plunger 'released' D2F switch, lever under the yoke edge."""
    g, y0, zt = _hp_sw_box(name)
    w = bx(30.8, 35.0, y0, y0 + 6.5, g["plate_z"] + 6.0, zt - 5.8)
    # rev G4: shelf under the switch, 2 x M2 VERTICAL through the switch body (the D2F holes go through its 5.8 mm thickness)
    w = w.union(bx(18.0, 35.0, y0, y0 + 6.5, zt - 5.8 - 3.0, zt - 5.8))
    for x, y in hp6_switch_holes(name):
        w = w.cut(cyl("z", (x, y, 0), 1.8, zt - 10.0, zt))                      # M2 thread-forming pilot in PA12
    x, y = hp6_foot_hole(name)
    return w.cut(cyl("z", (x, y, 0), 1.6, g["plate_z"] + 5.0, g["plate_z"] + 14.0))   # rev G4: M2 from below through HP-4


def hp6_switch_holes(name):
    g, y0, zt = _hp_sw_box(name)
    return [(24.4 - 3.25, y0 + 3.25), (24.4 + 3.25, y0 + 3.25)]


def hp6_foot_hole(name):
    g, y0, zt = _hp_sw_box(name)
    return (32.9, y0 + 3.25)


def cb_bolts(name: str) -> dict:
    """rev J: TP-1 bolts used by CB-1.  A bolt is used only if a straight hex-key corridor from its head to the front is clear of
    the body (the stop block gets access bores; a head under the plunger-switch shelf cannot be reached and is left out)."""
    g = hp_geom(name)
    _, y0, zt = _hp_sw_box(name)
    shelf = (18.0, 35.0, zt - 5.8 - 3.0, zt - 5.8)            # x0, x1, z0, z1 of the switch shelf (+ the post above x 30.8)
    used, dropped = [], []
    for x, z in P.TOOL_BOLTS:
        zc = TP_ZC + z
        r = CB_ACCESS_D / 2
        blocked = (x + r > shelf[0] and x - r < shelf[1] and zc + 1.5 > shelf[2] - 0.01 and zc - 1.5 < shelf[3] + 0.01)
        (dropped if blocked else used).append((x, zc))
    return {"used": used, "dropped": dropped}


def hp_body(name: str) -> cq.Workplane:
    """CB-1 (CNC, one per pipette, rev J): back plate + plunger-actuator shelf + collar stop + plunger-switch shelf in ONE body"""
    g = hp_geom(name)
    hx, t = g["hx"], CB_BACK_T
    z0b, z1b = g["back_z0"], P.TOOL_PLATE_H
    w = bx(-P.TOOL_PLATE_W / 2, P.TOOL_PLATE_W / 2, -t, 0.0, z0b, z1b).edges("|Y").fillet(3.0)
    # collar stop (was HP-8): U slot open to the front (machinable from the front; the collar bears on the top face)
    zc0, _ = _collar_z(g)
    rb = (g["body_d"] + 1.0) / 2
    stop = bx(hx - 22, hx + 22, g["y"], -t + 0.01, zc0 - 6.0, zc0)
    stop = stop.cut(bx(hx - rb, hx + rb, g["y"] - 1.0, g["y"] + rb, zc0 - 7.0, zc0 + 1.0))
    w = w.union(stop)
    # plunger actuator shelf (was HP-4): web on the plate, bridge, actuator flange plate; flange face = shelf underside
    pz = g["plate_z"]
    z1 = pz + 6.0
    ax_, ay = g["act"]
    w = w.union(bx(11, 35, -12, -t + 0.01, pz - 30.0, z1))
    w = w.union(bx(11, 35, ay + 15, -12 + 0.01, pz, z1))
    w = w.union(bx(ax_ - 15, ax_ + 15, ay - 15, ay + 15.5, pz, z1))
    # plunger-released switch post + shelf (was HP-6, printed): same switch position, M2 tapped from the top (OP2)
    _, y0, zt = _hp_sw_box(name)
    w = w.union(bx(30.8, 35.0, y0, y0 + 6.5, z1 - 0.01, zt - 5.8))
    w = w.union(bx(18.0, 35.0, y0, y0 + 6.5, zt - 5.8 - 3.0, zt - 5.8))
    cuts = [cyl("z", (ax_, ay, 0), 22.5, pz - 1, z1 + 1)]                                      # OP2
    cuts += [cyl("z", (ax_ + sx * N11_HOLES / 2, ay + sy * N11_HOLES / 2, 0), 2.9, pz - 1, z1 + 1)
             for sx in (-1, 1) for sy in (-1, 1)]                                                 # OP2: M2.5 clearance
    cuts += [cyl("z", (x, y, 0), 1.6, zt - 5.8 - 3.5, zt - 5.8 + 0.5) for x, y in hp6_switch_holes(name)]   # OP2: M2 tapped through
    # OP1 (front): TP-1 bolts (plain clearance; access bore through anything in front of the head), dowels, V-block, clamp, switch
    for x, zc in cb_bolts(name)["used"]:
        cuts.append(cyl("y", (x, 0, zc), 4.5, -t - 1, 1.0))
        cuts.append(cyl("y", (x, 0, zc), CB_ACCESS_D, -150.0, -t))
    dz = sorted(TP_ZC + z for _, z in P.TOOL_DOWELS)
    for x, z in P.TOOL_DOWELS:
        zc = TP_ZC + z
        if zc == dz[-1]:
            cuts.append(cyl("y", (x, 0, zc), 3.0, -t - 1, 1.0))                                 # round, reamed 3 H7
        else:
            cuts.append(cyl("y", (x, 0, zc - 1.0), 3.0, -t - 1, 1.0))
            cuts.append(cyl("y", (x, 0, zc + 1.0), 3.0, -t - 1, 1.0))
            cuts.append(bx(x - 1.5, x + 1.5, -t - 1, 1.0, zc - 1.0, zc + 1.0))                  # slot along Z
    cuts += [cyl("y", (x, 0, z), 3.3, -t - 1, -0.0) for x, z in vb_ear_screws(name)]               # M4 tapped through
    cuts += [cyl("y", (x, 0, z), 4.5, -t - 1, 1.0) for x, z in hp_back_screws(name)["lower"]]       # clearance, heads behind
    cuts += [cyl("y", (x, 0, z), 1.6, -t - 1, 1.0) for x, z in force_switch_holes(name)]            # M2 tapped through
    return cut_all(w, cuts)


def cb_machining(name: str) -> dict:
    """rev J: set-ups, tool access and the deepest cuts of CB-1 (from its own geometry)"""
    g = hp_geom(name)
    ax_, ay = g["act"]
    y_front = ay - 15.0
    reach = -CB_BACK_T - y_front                              # stock front face to the back-plate front face
    rb = (g["body_d"] + 1.0) / 2
    stock = (P.TOOL_PLATE_W, round(-y_front, 1), round(max(P.TOOL_PLATE_H, _hp_sw_box(name)[2] - 5.8) - g["back_z0"], 1))
    return {"stock_xyz_mm": stock, "stock": CB_STOCK, "setups": 2,
            "OP1": "tool axis -Y, back face (datum A) on parallels, clamped on the side faces: all faces, the collar-stop U slot, "
                   "the actuator flange face (shelf underside, side-milled), the switch shelf, every hole along Y (TP-1 bolts + "
                   "access bores, dowel hole 3 H7 reamed + dowel slot, V-block M4 taps, lower clamp holes, force-switch M2 taps), "
                   "engraved tool tag",
            "OP2": "tool axis -Z, datum A against the fixed jaw, side face B (x = +31) on a stop: actuator bore D22.5, 4 x D2.9, "
                   "2 x M2 tapped switch holes",
            "deepest_open_reach_mm": round(reach, 1),
            "cutter_for_the_deep_reach": f"D20 square end mill, long series, >= {math.ceil(reach / 5) * 5:g} mm reach (L/D {reach / 20:.1f})",
            "min_internal_radius_mm": {"edges along Y within 25 mm of the stock front (D10 cutter)": 5.0,
                                       "edges along Y deeper than 25 mm (D20 long series)": 10.0,
                                       "floor corners (square end mill)": 0.2},
            "deepest_pocket_ratio": {"collar-stop U slot depth/width": round((g["y"] + rb - y_front) / (2 * rb), 2),
                                     "M2 tapped holes depth/diameter": round(6.0 / 2.0, 1),
                                     "dowel hole depth/diameter": round(CB_BACK_T / 3.0, 1)},
            "tp1_bolts_used": cb_bolts(name)["used"], "tp1_bolts_left_out": cb_bolts(name)["dropped"],
            "one_way_fit": "round hole on the upper TP-1 dowel + Z slot on the lower one; engraved tag 'CB-1 " + HP_TAG.get(name, name) +
                           " / axis " + str(P.TOOL_NAMES.index(name)) + "' (the two eight-channel bodies look alike)"}


# ---------------- rev J (ADR-003 #7): printed setting gauges in place of feeler-gauge settings ----------------
def gauge_zhs() -> cq.Workplane:
    """GG-1 (MJF PA12): ZHS-1 setting block.  Laid flat on the tip-rack top under the eight-channel clamp HP-2B: jog down at 30 %
    current until HP-2B touches the step (the force switch or a visible stall shows it), then run the ZHS-1 stop screw up to the
    nut ledge and lock it.  Step 'M' 3.5 mm (machine set gap), step 'R' 3.0 mm (rig); 60 x 30 footprint covers the HP-2B face."""
    g = P.Z_HARD_STOP_SET_GAP
    w = bx(0, 30, 0, 30, 0, g["machine"]).union(bx(30, 60, 0, 30, 0, g["rig"]))
    w = w.union(bx(60, 90, 10, 20, 0, 3.0))                                          # handle
    w = w.cut(cyl("z", (82.0, 15.0, 0), 5.0, -1, 4))                                 # lanyard hole
    for x0, txt in ((4.0, "M3.5"), (34.0, "R3.0")):
        w = w.cut(bx(x0, x0 + 1.0, 4.0, 26.0, g["rig"] - 0.4, 10.0))                  # engraved index line per step
    return w


def gauge_eject_gap() -> cq.Workplane:
    """GG-2 (MJF PA12): eject-finger gap go/no-go.  With the axis at the normal top and the pipette fitted, the GO blade (0.5)
    must slide between the EJ-1 finger and the ejector button, the NO-GO blade (1.5) must not: proves 0.5 < gap < 1.5 (design
    1.0, P.Z_EJECT_GAP)."""
    w = bx(0, 70, 0, 14, 0, 3.0)
    w = w.union(bx(-14, 0.01, 2, 12, 0, 0.5)).union(bx(69.99, 84, 2, 12, 0, 1.5))
    return w


def gauge_overload_index() -> cq.Workplane:
    """GG-3 (MJF PA12): index T-handle for the 2 mm hex key of the M4 overload-stop set screws (rig LC-1, LC-2, LC-4): six numbered
    notches.  Turn the screw in until the load-cell reading moves by 1 N (contact, read on the HX711), then back it off TWO notches
    (120 deg = 0.23 mm on M4 x 0.7) and lock the nut: the gap is set by the instrument and the count, not by feel."""
    w = cyl("z", (0, 0, 0), 24.0, 0, 8.0).union(bx(-30, 30, -5, 5, 0, 8.0))
    w = w.cut(cq.Workplane("XY").polygon(6, 2.0 / math.cos(math.pi / 6)).extrude(10).translate((0, 0, -1)))
    for k in range(6):
        a = math.radians(60 * k)
        w = w.cut(bx(-0.6, 0.6, 9.0, 13.0, 6.0, 9.0).rotate((0, 0, 0), (0, 0, 1), 60 * k))
    return w


GAUGES = {"GG-1_zhs_setting_gauge": (gauge_zhs, "ZHS-1 set gap: clamp-to-rack 3.5 mm (machine) / 3.0 mm (rig), as the two step heights",
                                     "replaces the 0.5 mm feeler + jog of BR-B11 / the machine ZHS-1 setting"),
          "GG-2_eject_gap_go_nogo": (gauge_eject_gap, "EJ-1 finger-to-button gap at the normal top: 0.5 < gap < 1.5 (design 1.0)",
                                     "replaces the feeler check of the finger gap (BR-B05 / HD-07)"),
          "GG-3_overload_stop_index": (gauge_overload_index, "load-cell overload-stop gap 0.23 mm = 2 of 6 notches on M4 x 0.7 after "
                                                             "instrumented contact", "replaces the 0.2 mm feeler on the rig overload stops")}


def b_hp_actuator(name: str) -> cq.Workplane:
    g = hp_geom(name)
    ax_, ay = g["act"]
    a = bx(ax_ - N11 / 2, ax_ + N11 / 2, ay - N11 / 2, ay + N11 / 2, g["plate_z"] - 34.0, g["plate_z"])
    return a.union(cyl("z", (ax_, ay, 0), 6.0, g["plate_z"], g["button_top"] + 1.0 + 4.0))


# ======================================================================================
# 5. assembly model
# ======================================================================================
# custom part catalogue: id -> (process, material, thickness or None, qty, density key)
CUSTOM = {
    "CP-1_carriage_plate": ("sheet_laser", "Al 5052", P.CARRIAGE_PLATE_T, 1, "al"),
    "XBC-1_x_belt_clamp": ("cnc", "Al 6061-T6", None, 1, "al"),
    "ZMB-1_z_motor_bracket": ("cnc", "Al 6061-T6", None, P.N_TOOLS, "al"),
    "ZC-1_z_carriage": ("cnc", "Al 6061-T6", None, P.N_TOOLS, "al"),
    "TP-1_tool_plate": ("sheet_laser", "Al 5052", P.TOOL_PLATE_T, P.N_TOOLS, "al"),
    "ZBH-1_screw_bearing_housing": ("cnc", "Al 6061-T6", None, P.N_TOOLS, "al"),
    "ZSW-1_home_switch_bracket": ("print_mjf_pa12", "PA12", None, 0 if getattr(P, "REV_J", False) else P.N_TOOLS, "pa12"),
    "ZSF-1_switch_and_ffc_bracket": ("print_mjf_pa12", "PA12", None, P.N_TOOLS if getattr(P, "REV_J", False) else 0, "pa12"),   # rev J
    "EB-1_board_tray": ("print_mjf_pa12", "PA12", None, 1, "pa12"),
    "EB-2_splice_cover": ("print_mjf_pa12", "PA12", None, 1 if getattr(P, "REV_J", False) else 0, "pa12"),     # rev J
    "CCA-1_cable_chain_anchor": ("print_mjf_pa12", "PA12", None, 1, "pa12"),
    "GR-1_gripper_bracket": ("cnc", "Al 6061-T6", None, 1, "al"),
    "GR-2_jaw_carrier": ("cnc", "Al 6061-T6", None, 2, "al"),
    "GR-3_finger": ("sheet_laser", "Al 5052", G_BLADE_T, 2, "al"),
    # integration rev D
    "XSP-1_x_block_spacer": ("sheet_laser", "Al 5052", 2.0, 4, "al"),
    "XFL-1_x_endstop_flag": ("sheet_laser_bent", "Al 5052", 1.5, 1, "al"),
    "MBB-1_mmb_bracket": ("print_mjf_pa12", "PA12", None, 1, "pa12"),
    "CAM-1_camera_clip": ("print_mjf_pa12", "PA12", None, 1, "pa12"),              # integration rev E
    "FFC-F_fixed_clamp": ("print_mjf_pa12", "PA12", None, 0 if getattr(P, "REV_J", False) else P.N_TOOLS, "pa12"),       # rev E2
    "FFC-M_moving_clamp": ("print_mjf_pa12", "PA12", None, P.N_TOOLS, "pa12"),
    "ZUC-1_z_up_cam_bar": ("print_mjf_pa12", "PA12", None, 0, "pa12"),        # rev G; rev G4: qty 0, the bars are machined on ZC-1
    "EJ-1_eject_finger_small": ("cnc", "Al 6061-T6", None, 1, "al"),                  # rev G: method-B tip ejection
    "EJ-1_eject_finger_8ch_50": ("cnc", "Al 6061-T6", None, 1, "al"),
    "EJ-1_eject_finger_8ch_300": ("cnc", "Al 6061-T6", None, 1, "al"),
    "GR-4_jaw_switch_bracket": ("print_mjf_pa12", "PA12", None, 1, "pa12"),
    "HP-6_plunger_switch_bracket": ("print_mjf_pa12", "PA12", None, 0 if getattr(P, "REV_J", False) else 3, "pa12"),
    "PA-1_adapter_base": ("cnc", "Al 6061-T6", None, 0, "al"),
    "PA-2_adapter_vblock": ("cnc", "Al 6061-T6", None, 0, "al"),
    "PA-3_adapter_strap": ("cnc", "Al 6061-T6", None, 0, "al"),
    "ZHS-1_z_hard_stop": ("cnc", "Al 6061-T6", None, len(ZHS_AXES), "al"),          # rev G3 (R4): one per eight-channel axis
}
FLAT = {"XSP-1_x_block_spacer": x_spacer_flat, "CP-1_carriage_plate": carriage_plate_flat, "TP-1_tool_plate": tool_plate_flat, "GR-3_finger": gr_finger_flat}
HP_TAG = {"pipette_small": "small", "pipette_8ch_50": "8ch50", "pipette_8ch_300": "8ch300"}
for _n, _t in HP_TAG.items():
    if _n not in P.TOOL_NAMES:
        continue
    _e = HP_SETS[_n]["eight"]
    CUSTOM[f"HP-1_cradle_back_plate_{_t}"] = ("sheet_laser", "Al 5052", 6.0, 1, "al")
    CUSTOM[f"HP-2_pipette_vblock_{_t}"] = ("cnc", "Al 6061-T6", None, 1 if _e else 2, "al")
    CUSTOM[f"HP-3_pipette_strap_{_t}"] = ("cnc", "Al 6061-T6", None, 1 if _e else 2, "al")
    if _e:
        CUSTOM[f"HP-2B_8ch_manifold_clamp_{_t}"] = ("cnc", "Al 6061-T6", None, 1, "al")
        CUSTOM[f"HP-3B_8ch_clamp_bar_{_t}"] = ("sheet_laser", "Al 5052-H32", 6.0, 1, "al")   # integration rev E (DFM)
        FLAT[f"HP-3B_8ch_clamp_bar_{_t}"] = (lambda n=_n: hp8_bar(n).rotate((0, 0, 0), (1, 0, 0), 90))
    CUSTOM[f"HP-4_plunger_actuator_bracket_{_t}"] = ("cnc", "Al 6061-T6", None, 0 if getattr(P, "REV_J", False) else 1, "al")
    CUSTOM[f"HP-7_axial_collar_{_t}"] = ("cnc", "Al 6061-T6", None, 1, "al")      # rev F
    CUSTOM[f"HP-8_collar_stop_{_t}"] = ("cnc", "Al 6061-T6", None, 0 if getattr(P, "REV_J", False) else 1, "al")
    CUSTOM[f"HP-1_cradle_back_plate_{_t}"] = ("cnc", "Al 6061-T6", None, 0 if getattr(P, "REV_J", False) else 1, "al")   # rev F (M1)
    if getattr(P, "REV_J", False):      # rev J: HP-1 + HP-4 + HP-8 + HP-6 -> one machined body
        CUSTOM[f"CB-1_cradle_body_{_t}"] = ("cnc", "cast Al tooling plate 5083-class (or 6061-T6)", None, 1, "al")
    CUSTOM[f"HP-5_plunger_yoke_{_t}"] = ("sheet_laser", "Al 5052-H32", 6.0, 1, "al")   # integration rev E (DFM): laser + tap
    # rev F: HP-1 is now a machined part (rear ribs): no flat pattern

# bought: id -> (description, unit mass kg, qty)
N_HP = sum(1 for n in P.TOOL_NAMES if n in HP_SETS)
STIFF_LEN = STIFF_Z[1] - STIFF_Z[0]
BOUGHT = {
    "MGN12H_block": (f"MGN12H carriage (4 on the X rails + {2 * P.N_TOOLS} on the Z rails, two per Z rail; rev G2)", 0.054, 4 + 2 * P.N_TOOLS),
    "MGN12_rail_Z": (f"MGN12 rail, {P.Z_RAIL_LEN:.0f} mm, M3 x 8 screws on 25 mm pitch", 0.65 * P.Z_RAIL_LEN / 1000, P.N_TOOLS),
    N17_ID: ("NEMA17 1.8 deg, 48 mm body (17HS8401 class), >= 0.52 N m holding at <= 1.7 A rated, plain 5 mm shaft 14 mm long "
             "(cut or short-shaft option), standard NEMA17 flange (rev G5: was 40 mm 17HS4401 class)", N17_MASS_KG, P.N_TOOLS),
    "coupler_5_8": ("flexible JAW coupler 5 -> 8 mm, D20 x 20, hubs set 0.5 mm apart on the spider (axial float: the motor bearings "
                    "carry no screw thrust; rev G4)", 0.015, P.N_TOOLS),
    "ejector_button_ref": ("REFERENCE ONLY: the pipette's own tip-ejector button (part of the bought pipette)", 0.0, 0),
    "D2F_roller_lever": ("Omron D2F-01FL3 class hinge-roller-lever subminiature switch (rev G: Z-up switch, side-actuated)", 0.003, 0),
    "T8x2_screw": (f"Tr8x2 single-start lead screw {SCREW_Z[1] - SCREW_Z[0]:.0f} mm with the LOWER END MACHINED (rev G4): thread blank "
                   f"(D7, measure) >= 4 mm, square shoulder (runout <= 0.01) at the D7 -> D6 step, D6 h6 journal "
                   f"{Z_SHOULDER - (HOUSING_Z[0] + SPOT[1] - THRUST[2]):.0f} mm, M6 x 1 thread {HOUSING_Z[0] + SPOT[1] - THRUST[2] - SCREW_Z[0]:.1f} mm",
                   0.395 * (SCREW_Z[1] - SCREW_Z[0]) / 1000, P.N_TOOLS),
    "T8_antibacklash_nut": ("Tr8x2 POM anti-backlash nut, flange D22, 4 x M3 on PCD16", 0.02, P.N_TOOLS),
    "MR126ZZ": ("MR126ZZ bearing 6 x 12 x 4: RADIAL ONLY in the ZBH-1 web pocket (outer ring light press, inner ring slip on the D6 "
                "journal, 4 mm clear of the shim: no axial load) (rev G4: replaces 695ZZ)", 0.002, P.N_TOOLS),
    "ZBH_backing_washer_6.5x14x2": ("hardened steel backing washer 6.5 x 14 x 2 (DIN 988 class or turned, >= 45 HRC) under the "
                                    "upper thrust housing washer: bridges the D12 pocket (rev G4)", 0.002, P.N_TOOLS),
    # rev F (M2): pick-up reaction through a THRUST bearing at the fixed screw end, not the 695 deep-groove bearing
    # rev G4 (M2 corrected): the pick-up / ZHS-1 stop / downward-stall reaction pushes the screw UP into the LOWER thrust bearing;
    # strip, method-B eject and an upward stall push it DOWN into the UPPER one.  C0a ~1.59 kN (distributor tables, F6-14M class:
    # VERIFY ON THE DATASHEET OF THE PART BOUGHT); static SF >= 2 against the 755 N worst stall (z_hard_stop_rev_g3 / g4 report)
    "thrust_bearing_F6-14M": ("miniature thrust ball bearing 6 x 14 x 5 (F6-14M class, grooved races), TWO per axis, one on each face "
                              "of the ZBH-1 web; REQUIRED static axial rating C0a >= 1.5 kN (datasheet)", 0.004, 2 * P.N_TOOLS),
    "disc_spring_stack": ("disc springs (DIN 2093 class) under the upper V-block, preload = PICKUP_FORCE_N of the tool (20 / 120 / 120 N), "
                          "0.5 mm travel to the force switch", 0.005, N_HP),
    "688ZZ": ("688ZZ bearing 8 x 16 x 5 (gripper screw centre support)", 0.008, 2),
    "screw_end_locknut_M6": ("ISO 10511 M6 lock nut + DIN 125 M6 washer on the screw end under ZBH-1: preloads the thrust pair "
                             "(rev G4: was M5)", 0.004, P.N_TOOLS),
    "KW10_switch": (f"KW10 micro switch with roller lever ({P.N_TOOLS} Z homes)", 0.003, P.N_TOOLS),
    "steel_tube_40x20x2_stiffener": (f"steel rectangular tube 40 x 20 x 2 (S235/S355, zinc plated) cut to {STIFF_LEN:.0f} mm, carriage "
                                     "stiffener; M5 steel rivnuts in the front wall, M5 csk through CP-1 (integration rev F, M1: was 2040)",
                                     1.76 * STIFF_LEN / 1000, P.N_TOOLS),
    # integration rev D: boards are in electronics/lh1/electrical_bom.csv (qty 0 here; masses kept for the head mass)
    "HIB-1_board": ("HIB-1 head interconnect board 45 x 45 x 15 (electrical BOM; rev J: NOT BUILT, ADR-003 #1)", 0.0 if getattr(P, "REV_J", False) else 0.03, 0),
    "lever_splice_3way": ("lever splice connector 3-way, 221-413 class (about 18.4 x 13.1 x 8.1), 4 on EB-1 under EB-2 (rev J; ordered "
                          "with the pre-made leads by the electrical owner)", 0.004, 0),
    "M3_heatset_EB2": ("M3 heat-set insert + ISO 4762 M3x16 for the EB-2 cover (rev J)", 0.001, 2 if getattr(P, "REV_J", False) else 0),
    "MMB_CAN_board": ("BTT MMB CAN V2.0 with 4 EZ2209, 108.3 x 59 x 25 (electrical BOM)", 0.05, 0),
    # integration rev E: D2F switches are counted in electronics/lh1/electrical_bom.csv (qty 0 here; mass kept)
    "D2F_switch": ("D2F-class subminiature micro switch with lever (1 jaws-open + 3 plunger released + 4 Z-up): electrical BOM", 0.002, 0),
    "ISO10642_M3x12": ("ISO 10642 M3x12 A2: CP-1 + XSP-1 to the X blocks (confirm block thread depth)", 0.001, 16),
    "ISO10642_M3x10": ("ISO 10642 M3x10 A2: XBC-1 clamp from the front", 0.001, 4),
    "FFC_10way_1mm_highflex": ("10-way 1.0 mm pitch high-flex FFC, 0.1 mm tinned Cu, about 0.15 mm thick, length 300 mm, contacts same side; "
                               "REQUIRED: dynamic bend radius <= 6 mm for >= 1e6 cycles and >= 0.7 A per conductor (verify on the "
                               "datasheet of the part bought) + 2 x 10-way 1.0 mm ZIF-to-wire adapters per tool. rev G2: ORDERED FROM "
                               "electronics/lh1/electrical_bom.csv (FFC 10-way 300 mm, 6 pcs); listed here for reference only", 0.003, 0),
    "D2F_force_switch": ("D2F-01 class subminiature switch, SPDT changeover, plunger, on HP-1 (2 x M2x8 into HP-1): cradle "
                         "pick-up-force switch, one per pipette (rev G2: was in neither parts list)", 0.002, 3),
    "M2.5_standoffs": ("M2.5 hex standoffs F-F: 4 x 5 mm (HIB-1; none in rev J), 4 x 5 mm + 4 x 24.4 mm (MMB stack)", 0.001,
                       8 if getattr(P, "REV_J", False) else 12),
    "carriage_camera_D10": (("VC-1 head camera: 10 mm UVC module with integrated 6-LED ring, focus 50-300 mm (ASSUMED class), lens at "
                             "global z 165 (rev K): electrical BOM") if getattr(P, "REV_K", False) else
                            "down-looking D10 USB endoscope camera (stage 4 WM-1 imaging): electrical BOM", 0.01, 0),
    "M2x10_KW10": ("M2x10 or self-tapping 2.2 x 9.5 screws for the KW10 Z home switches", 0.0005, 8),
    "M2_D2F": ("M2x8 screws: four Z-up D2F (8) + three cradle force D2F on HP-1 (6; rev G2)", 0.0004, 14),
    "M2.5_heatset_M2.5x6": ("M2.5 heat-set insert + M2.5x6 screw for the MMB boards on MBB-1 (8 + 8) and HIB-1 (4 + 4; none in rev J)",
                            0.001, 8 if getattr(P, "REV_J", False) else 12),
    "M3x6_flag": ("ISO 4762 M3x6 for XFL-1", 0.001, 2),
    "MGN9_rail_100": ("MGN9 rail 100 mm (gripper)", 0.038, 1),
    "MGN9C_block": ("MGN9C carriage (gripper jaws)", 0.016, 2),
    "TR8x2_LR_screw_80": ("Tr8x2 left/right-hand screw 80 mm, centre journal (gripper)", 0.032, 1),
    "TR8x2_flange_nut": ("Tr8x2 brass flange nut, one LH one RH (gripper)", 0.015, 2),
    "NEMA11_28HS32": ("NEMA11 28 x 32 mm stepper, 0.06 N m (gripper)", 0.11, 1),
    "GT2_16T_20T_belt": ("GT2 6 mm: 16T + 20T pulleys, ~100 mm closed belt (gripper)", 0.02, 1),
    "silicone_pad_2mm": ("silicone sheet 2 mm, Shore A 40, 36 x 16 mm, adhesive-backed", 0.002, 2),
    "grip_spring_switch": ("2 x D4 guide pins + 2 springs k ~ 4 N/mm + D2F switch on one carrier (gripped)", 0.01, 1),
    "NEMA11_captive_linear": ("NEMA11 captive linear stepper, lead 1 mm, stroke >= 30 mm, thrust >= 40 N at the working speed (plunger; rev F M6)", 0.15, N_HP),
    "hand_pipette_small": ("single-channel 0.5-10 uL hand pipette (bought, tips stripped by the waste-nest plate)", 0.08, 1),
    "hand_pipette_8ch50": ("eight-channel 5-50 uL hand pipette, nozzle row along Y", 0.18, 1),
    "hand_pipette_8ch300": ("eight-channel 30-300 uL hand pipette, nozzle row along Y", 0.20, 1),
    "fasteners": ("ISO 4762 / 10642 M3, M4, M5 screws, M5 T-nuts, D3 x 10 dowels, washers", 0.18, 1),
    # rev G3 (R4): ZHS-1 hard stop hardware, one set per stop axis (listed explicitly, not in the 'fasteners' lot)
    "ZHS_set_screw_M5x30_locknut": ("ISO 4026 M5 x 30 set screw, flat point, hex socket, steel 45H (stop contact, socket end UP) "
                                    "+ ISO 4035 M5 thin lock nut (ZHS-1)", 0.006, len(ZHS_AXES)),
    "ZHS_ISO4762_M4x10": ("ISO 4762 M4 x 10 A2-70: ZHS-1 to CP-1 (tapped M4), counterbored in the block", 0.002, 2 * len(ZHS_AXES)),
}


def _loc(w: cq.Workplane, ax: float, zb: float) -> cq.Workplane:
    return w.translate((ax, Y_TF, zb))


class Item:
    def __init__(self, name, part, w, kind, pos=None):
        self.name, self.part, self.w, self.kind, self.pos = name, part, w, kind, pos   # kind: custom|bought|ctx


def zb_of(s: float) -> float:
    return ZB_TOP - s


def static_items() -> list[Item]:
    it = [Item("CP-1", "CP-1_carriage_plate", flat_to_xz(carriage_plate_flat(), PL_BACK_Y), "custom"),
          Item("XBC-1", "XBC-1_x_belt_clamp", belt_clamp(), "custom"),
          Item("EB-1", "EB-1_board_tray", board_tray(), "custom"),
          Item("CCA-1", "CCA-1_cable_chain_anchor", chain_anchor(), "custom"),
          *([Item("splices", "lever_splice_3way", splice_envelopes(), "bought"), Item("EB-2", "EB-2_splice_cover", splice_cover(), "custom")]
            if getattr(P, "REV_J", False) else
            [Item("board", "HIB-1_board", bx(BOARD_X[0], BOARD_X[1], BOARD_Y[0], BOARD_Y[1], 90.0, 105.0), "bought")])]   # rev D; rev J
    beam = bx(-200, 200, 0, 40, -40, 40)          # 4080 X beam (rev B), local portion
    for z in P.X_RAILS_LOCAL_Z:
        beam = beam.union(bx(-200, 200, -8, 0, z - 6, z + 6))
    it.append(Item("xbeam", "context", beam, "ctx"))
    for k, (x, z) in enumerate(X_BLOCKS):
        it.append(Item(f"xblk{k}", "MGN12H_block", bx(x - BLK_L / 2, x + BLK_L / 2, -BLK_H, -3.0, z - BLK_W / 2, z + BLK_W / 2), "bought"))
        it.append(Item(f"XSP{k}", "XSP-1_x_block_spacer", x_spacer(x, z), "custom"))      # integration rev D
    it += [Item("MBB-1", "MBB-1_mmb_bracket", mmb_bracket(), "custom"),                   # integration rev D (CR-06)
           Item("mmb_a", "MMB_CAN_board", b_mmb(0), "bought"), Item("mmb_b", "MMB_CAN_board", b_mmb(1), "bought"),
           Item("XFL-1", "XFL-1_x_endstop_flag", x_flag(), "custom"),
           Item("stdoffs", "M2.5_standoffs", b_standoffs(), "bought"),          # integration rev E
           Item("CAM-1", "CAM-1_camera_clip", cam_clip(), "custom"), Item("cam", "carriage_camera_D10", b_camera(), "bought")]
    for i, ax in enumerate(AXES_X):
        t = lambda w: w.translate((ax, 0, 0))   # noqa: E731
        it += [Item(f"zrail{i}", "MGN12_rail_Z", t(b_z_rail()), "bought"),
               Item(f"ZMB{i}", "ZMB-1_z_motor_bracket", t(motor_bracket()), "custom"),
               Item(f"motor{i}", N17_ID, t(b_motor17()), "bought"),
               Item(f"coupler{i}", "coupler_5_8", t(b_coupler()), "bought"),
               Item(f"screw{i}", "T8x2_screw", t(b_screw()), "bought"),
               Item(f"collars{i}", "screw_end_locknut_M6", t(b_collars()), "bought"),          # rev G4
               Item(f"zbrg{i}", "thrust_bearing_F6-14M", t(b_bearing_stack()), "bought"),      # rev G4: 2 x F6-14M + MR126ZZ + shim
               Item(f"ZBH{i}", "ZBH-1_screw_bearing_housing", t(bearing_housing()), "custom"),
               *([] if getattr(P, "REV_J", False) else [Item(f"ZSW{i}", "ZSW-1_home_switch_bracket", t(switch_bracket()), "custom")]),
               Item(f"switch{i}", "KW10_switch", t(b_switch()), "bought"),
               Item(f"zup{i}", "D2F_roller_lever", t(b_zup()), "bought"),    # rev E chain; rev G: roller lever on ZUC-1
               (Item(f"FFCF{i}", "ZSF-1_switch_and_ffc_bracket", t(zsw_ffc_bracket()), "custom") if getattr(P, "REV_J", False) else
                Item(f"FFCF{i}", "FFC-F_fixed_clamp", t(ffc_fixed_parts()), "custom")),   # integration rev E2; rev J: merged with ZSW-1
               Item(f"ffctail{i}", "FFC_10way_1mm_highflex", t(ffc_tail()), "bought"),
               Item(f"stiff{i}", "steel_tube_40x20x2_stiffener", t(b_stiffener()), "bought")]
        if P.TOOL_NAMES[i] in getattr(P, "EJECTOR", {}):                # rev G: method-B eject finger on CP-1
            it.append(Item(f"EJ{i}", f"EJ-1_eject_finger_{P.TOOL_NAMES[i].replace('pipette_', '')}", eject_finger(i), "custom"))
        if i in ZHS_AXES:                                                # rev G3 (R4): mechanical Z hard stop
            s_set = zhs_setting(P.TOOL_NAMES[i])["stop_setting_s"]
            it += [Item(f"ZHS{i}", "ZHS-1_z_hard_stop", t(z_stop_block()), "custom"),
                   Item(f"zhsscr{i}", "ZHS_set_screw_M5x30_locknut", t(z_stop_screw(s_set)), "bought")]
    return it


def tool_items(i: int, gap: float = P.GRIP_OPEN, name: str | None = None) -> list[tuple[str, str, cq.Workplane, str]]:
    """Tool-frame solids of the tool on axis i (name, part, workplane, kind)."""
    name = name or P.TOOL_NAMES[i]
    if name == "gripper":
        r = [("GR1", "GR-1_gripper_bracket", gr_bracket(), "custom"),
             ("g_rail", "MGN9_rail_100", b_gr_rail(), "bought"),
             ("g_screw", "TR8x2_LR_screw_80", b_gr_screw(), "bought"),
             ("g_motor", "NEMA11_28HS32", b_gr_motor(), "bought"),
             ("GR4", "GR-4_jaw_switch_bracket", gr_switch_bracket(), "custom"),          # integration rev D
             ("jaw_sw", "D2F_switch", b_d2f(24.0, -20.8, -15.8), "bought")]
        for s, tag in ((1, "r"), (-1, "f")):
            r += [(f"GR2{tag}", "GR-2_jaw_carrier", gr_carrier(s, gap), "custom"),
                  (f"GR3{tag}", "GR-3_finger", gr_finger(s, gap), "custom"),
                  (f"pad{tag}", "silicone_pad_2mm", b_gr_pad(s, gap), "bought"),
                  (f"g_blk{tag}", "MGN9C_block", b_gr_block(s, gap), "bought"),
                  (f"g_nut{tag}", "TR8x2_flange_nut", b_gr_nut(s, gap), "bought")]
        return r
    if name in HP_SETS:
        t, g = HP_TAG[name], hp_geom(name)
        if getattr(P, "REV_J", False):     # rev J: one machined cradle body
            r = [("CB1", f"CB-1_cradle_body_{t}", hp_body(name), "custom")]
        else:
            r = [("HP1", f"HP-1_cradle_back_plate_{t}", hp_back(name), "custom"),
                 ("HP4", f"HP-4_plunger_actuator_bracket_{t}", hp_act_bracket(name), "custom")]
        r += [("HP5", f"HP-5_plunger_yoke_{t}", hp_plunger_yoke(name), "custom"),
             ("pipette", f"hand_pipette_{t}", b_pipette(name), "bought"),
             ("hp_act", "NEMA11_captive_linear", b_hp_actuator(name), "bought"),
             ("HP2_hi", f"HP-2_pipette_vblock_{t}", hp_vblock(name, g["clamp_hi_z"]), "custom"),
             ("HP3_hi", f"HP-3_pipette_strap_{t}", hp_strap(name, g["clamp_hi_z"]), "custom"),
             ("HP7", f"HP-7_axial_collar_{t}", hp_collar(name), "custom"),            # rev F (M13)
             ("hp_spr", "disc_spring_stack", hp_springs(name), "bought"),
             ("f_sw", "D2F_force_switch", hp_force_switch(name), "bought"),             # rev F (M14); rev G2 part id
             ("pl_sw", "D2F_switch", hp_switch(name), "bought")]
        if not getattr(P, "REV_J", False):
            r += [("HP8", f"HP-8_collar_stop_{t}", hp_stop(name), "custom"),
                  ("HP6", "HP-6_plunger_switch_bracket", hp_switch_bracket(name), "custom")]      # integration rev D (CR-05)
        if g["eight"]:
            r += [("HP2B", f"HP-2B_8ch_manifold_clamp_{t}", hp8_clamp(name), "custom"),
                  ("HP3B", f"HP-3B_8ch_clamp_bar_{t}", hp8_bar(name), "custom")]
        else:
            r += [("HP2_lo", f"HP-2_pipette_vblock_{t}", hp_vblock(name, g["clamp_lo_z"]), "custom"),
                  ("HP3_lo", f"HP-3_pipette_strap_{t}", hp_strap(name, g["clamp_lo_z"]), "custom")]
        return r
    r = [("PA1", "PA-1_adapter_base", pa_base(), "custom"), ("module", "pipette_module", b_module(), "bought")]
    for k, (z0, z1) in enumerate(PA_SHELF_Z):
        r += [(f"PA2_{k}", "PA-2_adapter_vblock", pa_vblock(z1), "custom"),
              (f"PA3_{k}", "PA-3_adapter_strap", pa_strap(z1), "custom")]
    return r


def axis_items(i: int, s: float, gap: float = P.GRIP_OPEN) -> list[Item]:
    ax, zb = AXES_X[i], zb_of(s)
    r = [Item(f"zblk{i}", "MGN12H_block", _loc(b_z_block(), ax, zb), "bought", (s, gap)),
         Item(f"ffc{i}", "FFC_10way_1mm_highflex", ffc_cable(s).translate((ax, 0, 0)), "bought", (s, gap)),        # rev E2 loop
         Item(f"FFCM{i}", "FFC-M_moving_clamp", _loc(ffc_moving_clamp(), ax, zb), "custom", (s, gap)),
         Item(f"ZC{i}", "ZC-1_z_carriage", _loc(z_carriage(), ax, zb), "custom", (s, gap)),
         Item(f"TP{i}", "TP-1_tool_plate", _loc(tool_plate(), ax, zb), "custom", (s, gap)),
         Item(f"nut{i}", "T8_antibacklash_nut", _loc(b_nut(), ax, zb), "bought", (s, gap)),
]   # rev G4: ZUC-1 merged into ZC-1
    if P.TOOL_NAMES[i] in getattr(P, "EJECTOR", {}):                   # rev G: the pipette's ejector button (reference)
        r.append(Item(f"ejb{i}", "ejector_button_ref", _loc(ejector_button(P.TOOL_NAMES[i]), ax, zb), "bought", (s, gap)))
    for n, part, w, kind in tool_items(i, gap):
        r.append(Item(f"{n}@{i}", part, _loc(w, ax, zb), kind, (s, gap)))
    return r


# ---------------- rev G3 (R4): first clamp-to-rack contact and the hard-stop setting ----------------
SEAT_Z = P.LABWARE_SEAT_Z


def clamp_contact_s(name: str, rack_h: float | None = None) -> dict:
    """Smallest Z stroke at which any NON-NOZZLE part of the tool (cradle, clamp, actuator...) meets a tip rack of height
    rack_h (default: the tool's own tip type) centred under its nozzle row.  Bare nozzles and the pipette body enter the
    rack by design and are excluded.  Analytic from the solids' lowest faces over the rack footprint, then confirmed with
    an exact boolean 0.05 mm either side."""
    rack_h = P.TIP_RACK_H[P.TOOL_TIP[name]] if rack_h is None else rack_h
    g = hp_geom(name)
    xr, yr = g["hx"], Y_TF + g["y"]
    top_local = SEAT_Z + rack_h - P.X_BEAM_MID_Z
    hx_, hy_ = P.SBS_L / 2 + P.Z_HARD_STOP_LATERAL_BAND, P.SBS_W / 2 + P.Z_HARD_STOP_LATERAL_BAND   # rev G3: lateral band
    rack = bx(xr - hx_, xr + hx_, yr - hy_, yr + hy_, top_local - 40.0, top_local)
    best = (1e9, None)
    items = [(n, w) for n, part, w, kind in tool_items(P.TOOL_NAMES.index(name)) if n not in ("pipette",)]
    band = bx(xr - hx_, xr + hx_, g["y"] - hy_, g["y"] + hy_, -2000.0, 2000.0)      # rack footprint + band, tool frame
    for n, w in items:
        b = w.val().BoundingBox()
        if b.xmax <= xr - hx_ or b.xmin >= xr + hx_ or b.ymax + Y_TF <= yr - hy_ or b.ymin + Y_TF >= yr + hy_:
            continue
        # rev J: lowest point of the part INSIDE the band (a bounding box is too coarse for the merged CB-1 body, whose back
        # plate lies outside the band but whose box reaches over it)
        sol = w.intersect(band).solids().vals()
        if not sol:
            continue
        s = min(x.BoundingBox().zmin for x in sol) + ZB_TOP - top_local
        if s < best[0]:
            best = (s, n)
    s_c, who = best
    w = [w for n, w in items if n == who][0]

    def vol(s):
        inter = _loc(w, 0.0, zb_of(s)).intersect(rack)
        return sum(x.Volume() for x in inter.solids().vals()) if inter.solids().vals() else 0.0
    return {"tool": name, "rack_h": rack_h, "first_contact_s": round(s_c, 2), "part": who,
            "lateral_band_mm": P.Z_HARD_STOP_LATERAL_BAND,
            "exact_check": {"overlap_mm3_at_s_minus_0.05": round(vol(s_c - 0.05), 4), "overlap_mm3_at_s_plus_0.05": round(vol(s_c + 0.05), 4)}}


_ZHS_CACHE: dict = {}


def zhs_setting(name: str, where: str = "machine") -> dict:
    """nominal stop setting = first contact - set gap; firmware limit = setting - margin; checks against the strip stroke"""
    key = (name, where)
    if key in _ZHS_CACHE:
        return _ZHS_CACHE[key]
    c = clamp_contact_s(name)
    gap = P.Z_HARD_STOP_SET_GAP[where]
    s_set = math.floor((c["first_contact_s"] - gap) * 10.0) / 10.0
    tol = (P.Z_HARD_STOP_TOL_REV_J if getattr(P, "REV_J", False) else P.Z_HARD_STOP_TOL)[where]   # rev J: GG-1 gauge term
    worst, rss = sum(tol.values()), math.sqrt(sum(v * v for v in tol.values()))
    clr = c["first_contact_s"] - s_set
    r = {**c, "where": where, "set_gap_mm": gap, "stop_setting_s": s_set,
         "firmware_depth_limit_s": round(s_set - P.Z_HARD_STOP_FIRMWARE_MARGIN, 1),
         "clearance_at_stop_nominal_mm": round(clr, 2),
         "tolerance_stack_mm": tol, "tolerance_worst_case_mm": round(worst, 2), "tolerance_rss_mm": round(rss, 2),
         "clearance_after_worst_case_mm": round(clr - worst, 2), "clearance_after_rss_mm": round(clr - rss, 2),
         "pass_min_gap_after_tolerances": clr - worst >= P.Z_HARD_STOP_MIN_GAP_AFTER_TOL - 1e-9,
         "within_adjustment_range": P.Z_HARD_STOP_RANGE[0] <= s_set <= P.Z_HARD_STOP_RANGE[1],
         "screw_protrusion_above_block_mm": round(zhs_protrusion(s_set), 2),
         "screw_engagement_in_block_mm": round(ZHS["scr_l"] - zhs_protrusion(s_set), 2),
         # what the stop does NOT bound: the pipette's own manifold/nozzles over a FULL rack (they meet the tips first; the
         # pick-up stroke is ended by the cradle force switch, not by this stop)
         "pipette_manifold_bottom_minus_rack_top_at_stop_mm": round(
             (P.TOOL_PLATE_BOTTOM_Z_AT_TOP - NOZZLE_BELOW_TP + HP_SETS[name]["nozzle_l"] - s_set) - (SEAT_Z + c["rack_h"]), 2)}
    _ZHS_CACHE[key] = r
    return r


def s_max(i: int) -> float:
    """rev G3: deepest stroke axis i can physically reach (its ZHS-1 setting) - TRAVEL_Z for axes without a stop"""
    return zhs_setting(P.TOOL_NAMES[i])["stop_setting_s"] if i in ZHS_AXES else P.TRAVEL_Z


# pairs that overlap by design (shaft in coupler, screw in nut, rail in block, rod in tapped yoke ...)
def _excluded(a: str, b: str) -> bool:
    def base(n: str) -> str:
        return n.split("@")[0] if "@" in n else n.rstrip("0123456789")
    pair = {base(a), base(b)}
    same_axis = a[-1] == b[-1] or "@" in a or "@" in b
    rules = [{"zrail", "zblk"}, {"motor", "coupler"}, {"screw", "coupler"}, {"screw", "nut"}, {"screw", "collars"},
             {"g_rail", "g_blkr"}, {"g_rail", "g_blkf"}, {"g_screw", "g_nutr"}, {"g_screw", "g_nutf"},
             {"hp_act", "HP5"}, {"ffc", "ffctail"},   # rev E2: loop continues into its static tail
             {"ejb", "pipette"}, {"ZUC", "ZC"},   # rev G: button is part of the pipette; bar on ZC; finger flange on CP-1
             {"ZHS", "zhsscr"},                   # rev G3: stop screw in its tapped hole (modelled at the tap-drill diameter)
             {"zbrg", "ZBH"}, {"zbrg", "screw"}, {"zbrg", "collars"}]   # rev G4: bearings seated in the housing / on the journal                   # rev G3: stop screw in its tapped hole (modelled at the tap-drill diameter)
    if any(pair == r for r in rules) and same_axis:
        return True
    if pair == {"EJ", "ejb"} and a[-1] == b[-1]:
        return True      # rev G: the finger presses its own pipette's ejector button at the eject position (by design)
    if pair == {"EJ", "CP-"} or pair in ({"ZUC", "zup"}, {"ZC", "zup"}) and a[-1] == b[-1]:   # rev G4: the bar is part of ZC-1
        return True      # rev G: finger flange bolted on CP-1; the Z-up roller rides on its own cam bar
    return ("xbeam" in pair) and any(p.startswith("xblk") for p in pair)


def _bb_overlap(a: cq.BoundBox, b: cq.BoundBox, tol=0.01) -> bool:
    return not (a.xmax < b.xmin + tol or b.xmax < a.xmin + tol or a.ymax < b.ymin + tol or b.ymax < a.ymin + tol
                or a.zmax < b.zmin + tol or b.zmax < a.zmin + tol)


_CACHE: dict = {}


def interference(items: list[Item]) -> list[dict]:
    hits = []
    bbs = [it.w.val().BoundingBox() for it in items]
    for p in range(len(items)):
        for q in range(p + 1, len(items)):
            a, b = items[p], items[q]
            if _excluded(a.name, b.name) or not _bb_overlap(bbs[p], bbs[q]):
                continue
            key = tuple(sorted([(a.name, a.pos), (b.name, b.pos)], key=str))
            if key not in _CACHE:
                inter = a.w.intersect(b.w)
                _CACHE[key] = sum(s.Volume() for s in inter.solids().vals()) if inter.solids().vals() else 0.0
            v = _CACHE[key]
            if v > 0.01:
                hits.append({"a": a.name, "b": b.name, "mm3": round(v, 3)})
    return hits

# ======================================================================================
# 6. engineering checks
# ======================================================================================
G = 9.81


def lead_angle() -> float:
    return math.atan(T8_LEAD / (math.pi * T8_D2))


def screw_eff(mu: float) -> float:
    lam, phi = lead_angle(), math.atan(mu / math.cos(math.radians(15)))
    return math.tan(lam) / math.tan(lam + phi)


def thrust_check(moving_kg: float) -> dict:
    r = {"motor": f"NEMA17 48 mm (rev G5), {N17_HOLD_NM} N m holding; usable torque assumed {N17_USABLE_NM} N m at <= 5 rev/s (10 mm/s), 24 V, rated current",
         "screw": "Tr8x2 single start, d2 = 7.0 mm, lead angle %.2f deg, 30 deg trapezoidal thread" % math.degrees(lead_angle()),
         "required_N": P.Z_MIN_THRUST_N}
    for mu in SCREW_MU:
        eta = screw_eff(mu)
        f = 2 * math.pi * N17_USABLE_NM * eta / (T8_LEAD / 1000)
        r[f"mu_{mu:.2f}"] = {"efficiency": round(eta, 3), "thrust_N": round(f, 0),
                             "push_down_N": round(f + moving_kg * G, 0), "lift_N": round(f - moving_kg * G, 0)}
    worst = min(v["thrust_N"] for k, v in r.items() if k.startswith("mu_"))
    r["worst_thrust_N"] = worst
    r["pass"] = worst >= P.Z_MIN_THRUST_N
    return r


# ---------------- rev G3: fastener schedule derived from the modelled holes (used by the BR-1 rig parts list) ----------------
FASTENER_SPEC = {   # id -> (kind, specification)
    "ISO4762_M2x8": "ISO 4762 (or ISO 7045) M2 x 8 A2", "ISO4762_M2x10": "M2 x 10 A2 (or self-tapping 2.2 x 9.5)",
    "ISO4762_M2.5x8": "ISO 4762 M2.5 x 8 A2", "ISO4762_M3x8": "ISO 4762 M3 x 8 A2-70", "ISO4762_M3x10": "ISO 4762 M3 x 10 A2-70",
    "ISO10642_M3x8": "ISO 10642 M3 x 8 A2-70 countersunk", "ISO10642_M3x10": "ISO 10642 M3 x 10 A2-70 countersunk",
    "ISO4762_M4x10": "ISO 4762 M4 x 10 A2-70", "ISO4762_M4x12": "ISO 4762 M4 x 12 A2-70", "ISO4762_M4x16": "ISO 4762 M4 x 16 A2-70",
    "ISO4762_M4x30": "ISO 4762 M4 x 30 A2-70", "ISO10642_M4x12": "ISO 10642 M4 x 12 A2-70 countersunk",
    "ISO8734_D3x10": "dowel pin ISO 8734 D3 m6 x 10, hardened", "ISO4035_M3": "ISO 4035 M3 thin nut A2 (actuator rod jam nut)",
    "ISO4762_M2x6": "ISO 4762 M2 x 6 A2 (FFC-M into the tapped ZC-1 back face)", "ISO4762_M3x16": "ISO 4762 M3 x 16 A2-70 (HP-7 collar clamp)",
    "PT_M2x10": "M2 x 10 thread-forming screw for plastics (DIN 7500 / PT class), pan head (HP-6 post from below HP-4)",
    "ISO4762_M4x16": "ISO 4762 M4 x 16 A2-70 (rev J: upper V-block to CB-1 from the front, small pipette)",
    "ISO4762_M4x35": "ISO 4762 M4 x 35 A2-70 (rev J: upper V-block to CB-1 from the front, eight-channel)",
    "ISO4762_M4x20": "ISO 4762 M4 x 20 A2-70", "ISO4762_M4x25": "ISO 4762 M4 x 25 A2-70", "ISO4762_M4x30_": "ISO 4762 M4 x 30 A2-70",
}


def axis_joints(i: int, with_stop: bool | None = None) -> list[dict]:
    """Every screwed joint of Z axis i with its count taken from the hole lists that cut the parts (CP-1/BR-2, ZC-1, TP-1,
    ZMB-1, ZBH-1, ZSW-1, EJ-1, ZHS-1).  Joints whose holes are NOT in the model are returned with count 0 and 'open'."""
    h = axis_plate_holes(i, 0.0)
    stop = (i in ZHS_AXES) if with_stop is None else with_stop
    n_blk = sum(1 for _ in itertools.product((-1, 1), (-1, 1), (-1, 1)))
    J = [("Z rail to plate (tapped M3, 25 mm pitch)", "ISO4762_M3x8", len(h["rail_m3"]), "axis_plate_holes rail_m3"),
         ("two Z blocks to ZC-1 (counterbored under TP-1)", "ISO4762_M3x10", n_blk, "z_carriage block holes"),
         ("TP-1 to ZC-1 (countersunk)", "ISO10642_M3x10", len(TP_CSK), "TP_CSK"),
         ("Tr8 nut flange to the ZC-1 ledge (tips must NOT pass the ledge underside: ZHS-1 contact face)", "ISO4762_M3x10", 4,
          "z_carriage nut-flange holes (4 on PCD16)"),
         ("ZMB-1 legs to plate (tapped M4)", "ISO4762_M4x12", len(h["zmb_m4"]), "axis_plate_holes zmb_m4"),
         ("NEMA17 to ZMB-1 shelf", "ISO4762_M3x10", 4, "motor_bracket 4 x 3.4"),
         ("ZBH-1 tab to plate (tapped M4)", "ISO4762_M4x10", len(h["zbh_m4"]), "axis_plate_holes zbh_m4"),
         ("ZSW-1 (rev J: ZSF-1) to plate (tapped M3, counterbored)", "ISO4762_M3x10", len(h["zsw_m3"]), "axis_plate_holes zsw_m3"),
         ("KW10 home switch to ZSW-1", "ISO4762_M2x10", 2, "switch_bracket 2 x 1.8"),
         ("Z-up D2F roller switch to the ZSW-1 arm", "ISO4762_M2x8", 2, "switch_bracket arm 2 x 1.6"),
         ("FFC-F (rev J: ZSF-1 clamp end) to plate (tapped M2)", "ISO4762_M2x8", len(h["ffcf_m2"]), "axis_plate_holes ffcf_m2"),
         ("TP-1 dowels to the cradle (pressed into TP-1)", "ISO8734_D3x10", len(P.TOOL_DOWELS), "TOOL_DOWELS")]
    if h["ej_m3"]:
        J.append(("EJ-1 root to plate, from BEHIND (countersunk, flush)", "ISO10642_M3x8", len(h["ej_m3"]), "axis_plate_holes ej_m3"))
    if stop:
        J.append(("ZHS-1 to plate (tapped M4, counterbored)", "ZHS_ISO4762_M4x10", len(ZHS["m4"]), "ZHS m4"))
        J.append(("ZHS-1 stop screw + lock nut", "ZHS_set_screw_M5x30_locknut", 1, "ZHS scr"))
    J.append(("FFC-M moving clamp to the ZC-1 back face (tapped M2)", "ISO4762_M2x6", len(FFCM_HOLES), "FFCM_HOLES"))
    # rev G4: the ZUC-1 cam bars are machined on ZC-1 (no joint)
    return [dict(joint=j, fastener=f, count=c, source=s) for j, f, c, s in J]


def cradle_joints(name: str) -> list[dict]:
    """screwed joints of one hand-pipette cradle set (HP-1..HP-8 + force/plunger switches), counts from its hole lists"""
    g = hp_geom(name)
    hs = hp_back_screws(name)

    def head(pts, length="12"):
        under = [q for q in pts if 0.0 <= q[1] <= P.TOOL_PLATE_H]
        out = []
        if under:
            out.append(("ISO10642_M4x" + length, len(under)))
        if len(pts) - len(under):
            out.append(("ISO4762_M4x" + length, len(pts) - len(under)))
        return out
    if getattr(P, "REV_J", False):
        J = [("CB-1 to TP-1 (tapped M4 in TP-1, plain holes in CB-1; key through the access bores)", "ISO4762_M4x12",
              len(cb_bolts(name)["used"]), "cb_bolts used"),
             ("upper V-block HP-2 to CB-1, from the FRONT through the counterbored ears (CB-1 tapped M4)",
              f"ISO4762_M4x{vb_ear_screw_len(name):g}", len(vb_ear_screws(name)), "vb_ear_screws")]
        for f, c in head(hs["lower"]):
            J.append(("lower clamp (HP-2B or lower HP-2) to CB-1, from behind CB-1 below TP-1", f, c, "hp_back_screws lower"))
    else:
        J = [("HP-1 to TP-1 (tapped M4 in TP-1, X slots in HP-1)", "ISO4762_M4x12", len(P.TOOL_BOLTS), "TOOL_BOLTS")]
        for lab, key in (("upper V-block HP-2 to HP-1", "vblock_hi"), ("lower clamp (HP-2B or lower HP-2) to HP-1", "lower"),
                         ("HP-4 actuator bracket to HP-1", "hp4"), ("HP-8 collar stop to HP-1", "hp8")):
            for f, c in head(hs[key]):
                J.append((lab + (" (countersunk in the HP-1 back face: behind TP-1)" if f.startswith("ISO10642") else ""), f, c,
                          f"hp_back_screws {key}"))
    n_straps = 1 if g["eight"] else 2
    J.append(("HP-3 strap to its V-block", "ISO4762_M4x30", 2 * n_straps, "hp_strap 2 x 4.5"))
    if g["eight"]:
        J.append(("HP-3B clamp bar to HP-2B", "ISO4762_M4x16", 2, "hp8_bar 2 x 4.5"))
    if getattr(P, "REV_J", False):
        J += [("NEMA11 captive actuator to the CB-1 shelf", "ISO4762_M2.5x8", 4, "hp_body 4 x 2.9"),
              ("actuator rod in the HP-5 yoke (rod thread to confirm on the actuator bought)", "ISO4035_M3", 1, "hp_plunger_yoke 2.5 tapped"),
              ("force switch D2F to CB-1 (tapped M2)", "ISO4762_M2x8", len(force_switch_holes(name)), "force_switch_holes"),
              ("plunger-released D2F to the CB-1 shelf (vertical, tapped M2 in aluminium)", "ISO4762_M2x8", len(hp6_switch_holes(name)),
               "hp6_switch_holes"),
              ("HP-7 split collar clamp (tangential)", "ISO4762_M3x16", 1, "hp_collar clamp hole")]
        return [dict(joint=j, fastener=f, count=c, source=s) for j, f, c, s in J]
    J += [("NEMA11 captive actuator to HP-4", "ISO4762_M2.5x8", 4, "hp_act_bracket 4 x 2.9"),
          ("actuator rod in the HP-5 yoke (rod thread to confirm on the actuator bought)", "ISO4035_M3", 1, "hp_plunger_yoke 2.5 tapped"),
          ("force switch D2F to HP-1 (tapped M2)", "ISO4762_M2x8", len(force_switch_holes(name)), "force_switch_holes"),
          ("plunger-released D2F to the HP-6 shelf (vertical, thread-forming into PA12)", "ISO4762_M2x8", len(hp6_switch_holes(name)),
           "hp6_switch_holes"),
          ("HP-6 post to HP-4 (from below)", "PT_M2x10", 1, "hp6_foot_hole"),
          ("HP-7 split collar clamp (tangential)", "ISO4762_M3x16", 1, "hp_collar clamp hole")]
    return [dict(joint=j, fastener=f, count=c, source=s) for j, f, c, s in J]


N17_ROTOR_J = 68e-7         # kg m^2: rev G5 48 mm NEMA17 rotor inertia (catalogue-typical 68 g cm^2; ASSUMED; 40 mm was 54)
ZHS_IMPACT_V = 10.0          # mm/s: Z speed assumed when a firmware fault drives the axis into the stop (thrust_check basis)
ZHS_PATH_K = 10.0            # N/um: axial stiffness nut + screw + bearing in series (ASSUMED; POM nut dominates)
AL6061_YIELD, AL5052_YIELD = 240.0, 160.0    # MPa, minimum tensile yield (EN 573/485 class values for T6 / H32)


def stall_thrust() -> dict:
    """rev G3: largest axial force the Z motor can put through the nut (holding torque, least friction in SCREW_MU)"""
    mu = SCREW_MU[0]
    eta = screw_eff(mu)
    f = 2 * math.pi * N17_HOLD_NM * eta / (T8_LEAD / 1000)
    w = 2 * math.pi * ZHS_IMPACT_V / T8_LEAD                       # rad/s
    e_rot = 0.5 * N17_ROTOR_J * w ** 2                               # J
    f_dyn = math.sqrt(2 * eta * e_rot * ZHS_PATH_K * 1e6)            # N (energy into the stiffness of the path)
    return {"basis": f"NEMA17 holding torque {N17_HOLD_NM} N m at rated current, Tr8x2 (lead {T8_LEAD} mm, d2 {T8_D2}), "
                     f"least friction mu = {mu} -> efficiency {eta:.3f}: F = 2 pi T eta / L",
            "static_stall_N": round(f, 0), "efficiency": round(eta, 3),
            "dynamic_addition_N": round(f_dyn, 0),
            "dynamic_basis": f"rotor energy at {ZHS_IMPACT_V:g} mm/s (J = {N17_ROTOR_J * 1e7:.0f} g cm^2 ASSUMED) x efficiency into a "
                             f"path stiffness of {ZHS_PATH_K:g} N/um ASSUMED: F = sqrt(2 eta E k)",
            "design_force_N": round(f + f_dyn, 0)}


N17_RATED_A = 1.7          # A: 48 mm NEMA17 rated phase current (ASSUMED: catalogue-typical, confirm on the motor bought)
THRUST_C0A = 1588.0        # N: F6-14M static axial rating (distributor tables 357 lbf; ASSUMED until the datasheet of the part bought)
MR126_C0R = 330.0          # N: MR126ZZ static radial rating (catalogue-typical; ASSUMED)
STACK_K = 20.0             # N/um: axial stiffness of the preloaded thrust pair + shoulder + nut (ASSUMED, light-preload ball contacts)
PRELOAD_N = 100.0          # N: stack preload set by the lock nut (procedure in the report)
FW_Z_RUN_A = 0.9           # A rms: firmware/lh1/printer.cfg [tmc2209 *z*] run_current (read, not owned here)
FW_Z_FORCE_A_NOW = 1.4     # A rms: printer.cfg _LH_ZCUR force_current at G5 (force strokes only; read, not owned here)
FW_Z_FORCE_A = 1.3         # A rms: force_current the firmware owner sets for the 48 mm motor (G5 decision)
DRIVER_BURST_A = 1.4       # A rms: TMC2209 short-time limit used by the firmware owner (system.yaml z_drive)


def screw_support_report(z_moving: float) -> dict:
    """rev G4: every axial load case of the Z lead screw, its direction, the bearing that carries it and the static SF"""
    st = stall_thrust()
    w = z_moving * G
    eta20 = screw_eff(SCREW_MU[-1])
    full_usable_thrust = 2 * math.pi * N17_USABLE_NM * eta20 / (T8_LEAD / 1000)        # worst friction, at speed, full current

    def stall_at(i_a):
        return 2 * math.pi * N17_HOLD_NM * (i_a / N17_RATED_A) * st["efficiency"] / (T8_LEAD / 1000)
    up, dn = "UP on the screw -> LOWER F6-14M (web bottom face, lock nut side)", "DOWN on the screw -> UPPER F6-14M (shoulder side, on the shim)"
    cases = [
        ("tip pick-up, design load (DESIGN_LOAD_N)", up, P.DESIGN_LOAD_N),
        ("tip pick-up, force-ended (PICKUP_FORCE_N max)", up, max(P.PICKUP_FORCE_N.values())),
        ("comb strip (STRIP_FORCE_N max) + carriage weight lifted", dn, max(P.STRIP_FORCE_N.values()) + w),
        ("method-B eject at the top (EJECTOR force_N max) + carriage weight lifted", dn, max(e["force_N"] for e in P.EJECTOR.values()) + w),
        ("stall onto ZHS-1 at rated current (static + dynamic + weight)", up, st["design_force_N"] + w),
        ("stall at the top at rated current (nut flange on the coupler; static + dynamic)", dn, st["design_force_N"]),
        # rev G5: at the currents the firmware can actually set (force strokes only; TMC2209 short-time limit 1.4 A)
        (f"stall onto ZHS-1 at the firmware force current now ({FW_Z_FORCE_A_NOW} A)", up, stall_at(FW_Z_FORCE_A_NOW) + st["dynamic_addition_N"] + w),
        (f"stall onto ZHS-1 at the planned force current ({FW_Z_FORCE_A} A)", up, stall_at(FW_Z_FORCE_A) + st["dynamic_addition_N"] + w),
        (f"stall at the top at the planned force current ({FW_Z_FORCE_A} A)", dn, stall_at(FW_Z_FORCE_A) + st["dynamic_addition_N"]),
        (f"stall onto ZHS-1 at the run current ({FW_Z_RUN_A} A)", up, stall_at(FW_Z_RUN_A) + st["dynamic_addition_N"] + w),
    ]
    rows = [{"case": c, "direction": d, "force_N": round(f, 0), "bearing": "F6-14M", "C0a_N (ASSUMED)": THRUST_C0A,
             "static_safety_factor": round(THRUST_C0A / f, 2)} for c, d, f in cases]
    sf_min = min(r["static_safety_factor"] for r in rows)
    # run-current window: floor keeps Z_MIN_THRUST_N x 1.1 at the worst friction (at speed); ceiling keeps SF >= 2 on the stall
    i_floor = 1.1 * P.Z_MIN_THRUST_N / full_usable_thrust * N17_RATED_A          # rev G5: force-current floor
    i_ceil_sf2 = (THRUST_C0A / 2 - st["dynamic_addition_N"] - w) / stall_at(1.0)
    i_ceil = min(N17_RATED_A, i_ceil_sf2)
    i_rec = round(min(i_ceil, max(i_floor, 1.6)), 2)
    shoulder_area = math.pi / 4 * (BLANK_D ** 2 - JOURNAL_D ** 2)
    f_max = max(r["force_N"] for r in rows)
    therm = (23.0e-6 - 11.5e-6) * 12.0 * 20.0 * 1000.0                          # um over the 12 mm web for 20 K
    return {
        "arrangement": "F6-14M on each face of the 12 mm ZBH-1 web (upper on a hardened 6.5 x 14 x 2 backing washer over the D12 "
                       "pocket, lower in a 0.5 mm spot face), clamped between the screw shoulder (D7 blank -> D6 journal) and an "
                       "ISO 10511 M6 lock nut + washer; MR126ZZ in the web pocket is radial only (inner ring free on the journal, "
                       "4 mm clear of the shim): no case loads it axially",
        "load_cases": rows, "min_static_safety_factor_at_rated_current": sf_min,
        "radial_MR126ZZ": {"C0r_N (ASSUMED)": MR126_C0R, "loads": "radial only: nut side load and coupler misalignment, estimated "
                                                                   "< 20 N (ASSUMED)", "safety_factor": round(MR126_C0R / 20.0, 1)},
        "screw_end": {"shoulder_bearing_stress_MPa": round(f_max / shoulder_area, 0), "shoulder_area_mm2": round(shoulder_area, 1),
                      "screw_steel_yield_MPa (C45 class, ASSUMED)": 350,
                      "lock_nut_M6_thread_shear_MPa": round(f_max / (0.5 * math.pi * 6.0 * 5.0), 1),
                      "machined_features_specified": "thread blank D7 (rolled Tr8: measure) >= 4 mm, square shoulder, D6 h6 journal, "
                                                     "M6 x 1 end thread: specified on the T8x2_screw line (custom end machining; a stock "
                                                     "screw does not have them)"},
        "preload_and_end_float": {"procedure": "tighten the M6 lock nut until the axial play read on a dial indicator at the carriage "
                                               "is zero under +/-20 N, then a further 30 deg; check the screw turns by hand without "
                                               "notching", "preload_N (ASSUMED from the procedure)": PRELOAD_N,
                                  "end_float_mm": 0.0,
                                  "elastic_axial_shift_at_150N_um (stiffness ASSUMED)": round(P.DESIGN_LOAD_N / STACK_K, 1),
                                  "unloading_of_the_opposite_bearing_N (~2 x preload)": 2 * PRELOAD_N,
                                  "effect": "no free play: pipetting height error from the support is the elastic shift only "
                                            f"(~{P.DESIGN_LOAD_N / STACK_K:.0f} um at 150 N), well inside the 0.5 mm disc-spring travel "
                                            "that ends the force-ended pick-up"},
        "thermal": {"web_vs_journal_length_change_um_per_20K": round(therm, 1),
                    "preload_change_N_per_20K": round(therm * STACK_K, 0),
                    "screw_vs_plate_growth_over_the_screw_mm_per_20K": round((23.0e-6 - 11.5e-6) * (SCREW_Z[1] - SCREW_Z[0]) * 20, 3),
                    "taken_by": "the jaw coupler's axial float (hubs 0.5 mm apart): the screw is fixed only at ZBH-1"},
        "motor_end": "plain-shaft NEMA17 (not an integrated-screw motor) on a jaw coupler with 0.5 mm axial float: the motor bearings "
                     "see no screw thrust, and the screw's thermal growth relative to the plate (row above) is far below the float",
        "run_current": {
            "motor": f"{N17_ID}: holding {N17_HOLD_NM} N m, usable {N17_USABLE_NM} N m, rated {N17_RATED_A} A, rotor {N17_ROTOR_J * 1e7:.0f} g cm^2 "
                     "(catalogue-typical, ASSUMED); torque proportional to current",
            "run_current_A (all ordinary moves)": FW_Z_RUN_A,
            "force_current_A (force strokes only, planned)": FW_Z_FORCE_A,
            "force_current_A_now_in_firmware": FW_Z_FORCE_A_NOW, "driver_short_time_limit_A": DRIVER_BURST_A,
            "required_N (1.1 x Z_MIN_THRUST_N)": 1.1 * P.Z_MIN_THRUST_N,
            "worst_thrust_N (mu 0.20, at speed)": {f"{a:g} A": round(full_usable_thrust * a / N17_RATED_A, 0)
                                                   for a in (FW_Z_RUN_A, 1.2, FW_Z_FORCE_A, FW_Z_FORCE_A_NOW, N17_RATED_A)},
            "force_current_floor_A (1.1 x Z_MIN_THRUST_N at mu 0.20)": round(i_floor, 2),
            "force_current_ceiling_A (bearing SF 2 incl. dynamic + weight)": round(i_ceil_sf2, 2),
            "force_current_ceiling_A (driver)": DRIVER_BURST_A,
            "verdict": ("PASS: 1.3 A gives %.0f N >= %.0f N; stall total at 1.3 A %.0f N, F6-14M SF %.2f; at 1.4 A (firmware now) SF %.2f"
                        % (full_usable_thrust * FW_Z_FORCE_A / N17_RATED_A, 1.1 * P.Z_MIN_THRUST_N,
                           stall_at(FW_Z_FORCE_A) + st["dynamic_addition_N"] + w, THRUST_C0A / (stall_at(FW_Z_FORCE_A) + st["dynamic_addition_N"] + w),
                           THRUST_C0A / (stall_at(FW_Z_FORCE_A_NOW) + st["dynamic_addition_N"] + w))),
            "basis": f"rated {N17_RATED_A} A (ASSUMED); usable {N17_USABLE_NM} N m and holding {N17_HOLD_NM} N m at rated current"},
        "M2_corrected": "G2/G3 put the F5-10M in the UPPER pocket, where it carried only screw-DOWN loads; the pick-up reaction "
                        "(screw UP) went axially through the 695ZZ. G4: the pick-up load is carried by the LOWER F6-14M "
                        f"(C0a {THRUST_C0A:g} N ASSUMED): SF {THRUST_C0A / P.DESIGN_LOAD_N:.1f} at 150 N"}


def z_hard_stop_report(z_moving: float, static: list) -> dict:
    """rev G3 (R4): ZHS-1 per stop axis: setting, clearance after tolerances, geometric checks, stall-load margins"""
    st = stall_thrust()
    # rev G5: design force at the highest current the Z driver can be set to (TMC2209 short-time limit), not at rated current
    f_static = 2 * math.pi * N17_HOLD_NM * (DRIVER_BURST_A / N17_RATED_A) * st["efficiency"] / (T8_LEAD / 1000)
    F = f_static + st["dynamic_addition_N"] + z_moving * G   # carriage weight adds when driving down
    sx, sy = ZHS["scr"]
    # load path elements
    a_s = 14.2                                            # M5 tensile stress area, mm^2
    d_flat, socket_ac = 3.9, 2.5 / math.cos(math.radians(30))   # set-screw end face (chamfered) and hex-socket across corners
    a_contact = math.pi / 4 * (d_flat ** 2 - socket_ac ** 2)
    e_min = min(ZHS["scr_l"] - zhs_protrusion(s) for s in P.Z_HARD_STOP_RANGE)
    a_thread = 0.5 * math.pi * 5.0 * e_min                # internal-thread shear area (about 0.5 pi d Le) at the least engagement
    blk = z_stop_block()
    hous = bearing_housing()
    top_slab = hous.intersect(bx(-50, 60, -60, 0, HOUSING_Z[1] - 0.1, HOUSING_Z[1])).translate((0, 0, 0.1))
    a_bear = blk.intersect(top_slab).val().Volume() / 0.1      # block underside area that sits on the ZBH-1 top face
    preload, mu_al = 2000.0, 0.2                          # N per M4 (about 1.6 N m into tapped 5052), Al-Al friction (ASSUMED)
    m4_shear = 0.6 * 700.0 * 8.78                          # A2-70, shear on the stress area
    loads = {
        "design_force_N": round(F, 0),
        "design_force_basis": f"stall at {DRIVER_BURST_A} A (driver short-time limit; firmware force current planned {FW_Z_FORCE_A} A) + dynamic + weight",
        "set_screw_compression_MPa": round(F / a_s, 0), "set_screw_proof_MPa (45H, ISO 898-5 class)": 450,
        "ledge_contact_stress_MPa": round(F / a_contact, 0), "ledge_contact_area_mm2": round(a_contact, 2),
        "ledge_6061_yield_MPa": AL6061_YIELD, "ledge_contact_safety_factor": round(AL6061_YIELD / (F / a_contact), 2),
        "block_thread_shear_MPa": round(F / a_thread, 1), "block_thread_least_engagement_mm": round(e_min, 1),
        "block_thread_safety_factor (6061 shear yield 0.58 x yield)": round(0.58 * AL6061_YIELD / (F / a_thread), 1),
        "block_on_ZBH1_bearing_area_mm2": round(a_bear, 1), "block_on_ZBH1_bearing_MPa": round(F / a_bear, 1),
        "block_bearing_safety_factor": round(AL6061_YIELD / (F / a_bear), 1),
        "M4_to_plate_only_if_block_NOT_seated": {
            "friction_capacity_N (2 x preload x mu)": round(2 * preload * mu_al, 0), "friction_safety_factor": round(2 * preload * mu_al / F, 2),
            "bolt_shear_capacity_N": round(2 * m4_shear, 0), "bolt_shear_safety_factor": round(2 * m4_shear / F, 1),
            "assumptions": f"preload {preload:g} N per screw, mu {mu_al} (ASSUMED); the block is seated on ZBH-1 at build, so the "
                           "M4 screws only locate it"},
        "screw_axial_support_in_ZBH1": "rev G4: two F6-14M thrust bearings, one each side of the ZBH-1 web (see checks.z_screw_axial_support_rev_g4)",
    }
    axes = {}
    for i in ZHS_AXES:
        name = P.TOOL_NAMES[i]
        r = dict(zhs_setting(name))
        s_set = r["stop_setting_s"]
        zc = [it for it in axis_items(i, s_set - 0.5) if it.name == f"ZC{i}"][0].w.val()
        scr = [it for it in static if it.name == f"zhsscr{i}"][0].w.val()
        blkw = [it for it in static if it.name == f"ZHS{i}"][0].w.val()
        nut = [it for it in axis_items(i, P.Z_HARD_STOP_RANGE[1]) if it.name == f"nut{i}"][0].w.val()
        zc_deep = [it for it in axis_items(i, s_set) if it.name == f"ZC{i}"][0].w
        inter = zc_deep.intersect(cq.Workplane(obj=scr))
        r["geometry"] = {"ledge_to_screw_gap_at_setting_minus_0.5_mm": round(zc.distance(scr), 3),
                         "ledge_screw_overlap_at_setting_mm3": round(sum(x.Volume() for x in inter.solids().vals()), 4),
                         "tr8_nut_to_block_clearance_at_deepest_setting_mm": round(nut.distance(blkw), 2),
                         "lead_screw_to_block_clearance_mm": round([it for it in static if it.name == f"screw{i}"][0].w.val().distance(blkw), 2)}
        axes[name] = r
    return {"part": "ZHS-1_z_hard_stop (CNC 6061-T6) + ISO 4026 M5x30 set screw + ISO 4035 M5 lock nut + 2 x ISO 4762 M4x10",
            "axes": axes, "stall": st, "load_path": loads,
            "not_obstructed": "the stop is at the BOTTOM of travel behind the Z carriage ledge: homing and the method-B eject "
                              "over-travel (Z -12) are at the top; the FFC loop runs on the -X side of the axis (x -26.5..-15.5); "
                              "interference configs use s_max (= the stop setting) for these axes",
            "protects_against": "the tool's OWN rack type under its clamp (tallest rack the cradle meets in the model).  A TALLER rack "
                                "beside the tool (e.g. a 72 mm p300 rack next to the 50 uL eight-channel, contact at s 185.0) is below the "
                                "strip stroke (185.25) and stays a software rule (keepouts K17)"}


def backdrive_check(moving_kg: float) -> dict:
    lam = lead_angle()
    mu_crit = math.tan(lam) * math.cos(math.radians(15))
    w = moving_kg * G
    t_hold_frictionless = w * (T8_LEAD / 1000) / (2 * math.pi)
    return {"lead_angle_deg": round(math.degrees(lam), 2),
            "self_locking_if_mu_above": round(mu_crit, 4),
            "mu_range_assumed": list(SCREW_MU),
            "self_locking_at_lowest_mu": SCREW_MU[0] > mu_crit,
            "margin_at_lowest_mu": round(SCREW_MU[0] - mu_crit, 4),
            "gravity_load_N": round(w, 1),
            "torque_to_hold_with_zero_friction_Nm": round(t_hold_frictionless, 4),
            "nema17_detent_torque_Nm_typ": N17_DETENT_NM,
            "detent_alone_holds": N17_DETENT_NM > t_hold_frictionless,
            "verdict": "no counterbalance: the screw is self-locking for mu >= %.3f, and even at mu = 0 the "
                       "unpowered motor detent (~%.3f N m) exceeds the %.4f N m gravity torque; "
                       "verify by power-off drop test with 1.3 kg on each tool (vibration creep)."
                       % (mu_crit, N17_DETENT_NM, t_hold_frictionless)}


def deflection_check(load_y: float, block_z: float, F: float = 150.0) -> dict:
    """200 N upward on an outer tool: the moment about X is carried by the carriage plate (+2040 stiffener)
    as a cantilever from the lower X-block line down to the Z block."""
    z_root = P.X_RAILS_LOCAL_Z[0] - BLK_W / 2
    L = z_root - block_z
    b = 2 * (P.TOOL_PITCH_X - P.TOOL_PLATE_W / 2) + 2 * 20.0   # effective plate width, assumption ~ 126 mm
    t = P.CARRIAGE_PLATE_T
    ap, yp, ip = b * t, PL_BACK_Y - t / 2, b * t ** 3 / 12
    as_, ys, is_ = STIFF_A * STIFF_N, PL_BACK_Y + 10.0, STIFF_I_WEAK * STIFF_N     # rev F: steel tube, transformed section
    yna = (ap * yp + as_ * ys) / (ap + as_)
    i_c = ip + ap * (yp - yna) ** 2 + is_ + as_ * (ys - yna) ** 2
    out = {}
    for tag, I, yn in (("plate_only", ip, yp), ("plate_plus_2040", i_c, yna)):
        M = F * (yn - load_y)
        delta = M * L ** 2 / (2 * E_AL * I)
        theta = M * L / (E_AL * I)
        out[tag] = {"I_mm4": round(I, 0), "moment_Nmm": round(M, 0), "deflection_at_block_mm": round(delta, 3),
                    "rotation_mrad": round(theta * 1000, 3),
                    "tip_shift_y_mm_200mm_below_block": round(delta + theta * 200.0, 3)}
    out["assumptions"] = (f"F = {F:.0f} N up on the outer tool (x = {AXES_X[-1]:.0f}) at local y = {load_y:.0f} (tool centre), Z block at bottom of travel "
                          f"(local z {block_z:.0f}), cantilever L = {L:.0f} mm from the lower X-block edge, effective "
                          f"plate width {b:.0f} mm, one 2040 behind the loaded rail (I_weak {EXTR_2040_I_WEAK:.0f} mm^4, "
                          f"approx.), E = 70 GPa; X-block and Z-block compliance not included")
    out["z_block_pitch_moment_Nm"] = round(F * (Y_TF + G_YC - (ZBLK_TOP_Y)) / -1000, 1)
    out["plate_share_of_MAX_TOOL_TIP_DEFLECTION_mm"] = P.MAX_TOOL_TIP_DEFLECTION_UNDER_PICKUP
    return out


def item_mass(it: Item) -> float:
    if it.kind == "custom":
        return it.w.val().Volume() * RHO[CUSTOM[it.part][4]]
    if it.part == "thrust_bearing_F6-14M":
        return 2 * BOUGHT["thrust_bearing_F6-14M"][1] + BOUGHT["MR126ZZ"][1] + BOUGHT["ZBH_backing_washer_6.5x14x2"][1]
    return BOUGHT.get(it.part, ("", 0.0))[1]


def mass_props(items: list[Item]) -> dict:
    tot, mx, my, mz = 0.0, 0.0, 0.0, 0.0
    tools_model = 0.0
    for it in items:
        if it.kind == "ctx":
            continue
        m = item_mass(it)
        if "@" in it.name:
            tools_model += m
            continue
        c = cq.Shape.centerOfMass(it.w.val())
        tot += m; mx += m * c.x; my += m * c.y; mz += m * c.z
    for i, ax in enumerate(AXES_X):                 # tools at TOOL_MAX_MASS, CoM assumed 55 mm in front, 30 up
        m = P.TOOL_MAX_MASS
        tot += m; mx += m * ax; my += m * (Y_TF + G_YC); mz += m * (ZB_TOP + 30.0)
    for m, (x, y, z) in ((BOUGHT["fasteners"][1], (0, -25, -60)), (0.20, (0, -20, 20))):   # fasteners, cables
        tot += m; mx += m * x; my += m * y; mz += m * z
    return {"total_kg": round(tot, 2), "com_local_mm": [round(mx / tot, 1), round(my / tot, 1), round(mz / tot, 1)],
            "com_in_front_of_x_rail_face_mm": round(-my / tot, 1),
            "com_global_z_mm": round(mz / tot + P.X_BEAM_MID_Z, 1),
            "modelled_tool_hardware_kg (excluded; replaced by N x TOOL_MAX_MASS)": round(tools_model, 3),
            "total_with_modelled_tools_kg": round(tot - P.N_TOOLS * P.TOOL_MAX_MASS + tools_model, 2),
            "gantry_assumption_kg": P.HEAD_MOVING_MASS_ASSUMED, "exceeds_assumption": tot > P.HEAD_MOVING_MASS_ASSUMED,
            "includes": "carriage plate, clamp, brackets, 2040 stiffeners, 4 X blocks, %d Z rails/blocks/motors/screws/nuts/"
                        "couplers/bearings, Z carriages, tool plates, board, %d tools x %.1f kg, fasteners, 0.20 kg cables"
                        % (P.N_TOOLS, P.N_TOOLS, P.TOOL_MAX_MASS)}


def gripper_force() -> dict:
    out = {"motor": "NEMA11 28HS32, usable 0.04 N m (0.06 holding, derated)", "drive": "GT2 16T -> 20T (x1.25) to Tr8x2 L/R screw"}
    for mu in SCREW_MU:
        eta = screw_eff(mu)
        t = N11_USABLE_NM * G_PULLEY_RATIO
        f = math.pi * t * eta / (T8_LEAD / 1000)          # each nut: T = 2 F L / (2 pi eta)
        out[f"mu_{mu:.2f}"] = {"efficiency": round(eta, 3), "max_clamp_force_N": round(f, 1)}
    fmin = min(v["max_clamp_force_N"] for k, v in out.items() if k.startswith("mu_"))
    out["max_clamp_force_worst_N"] = fmin
    out["required_range_N"] = list(P.GRIP_FORCE_N)
    out["pass"] = fmin >= P.GRIP_FORCE_N[1]
    out["force_setting"] = "driver current limit; 'gripped' = spring-mounted front finger moves 1 mm and trips a D2F switch"
    for mu_pad in (0.5, 1.0):
        need = P.GRIP_MAX_PAYLOAD * (G + P.MAX_ACCEL_XY / 1000) / (2 * mu_pad)
        out[f"clamp_needed_for_payload_mu_pad_{mu_pad}"] = round(need, 1)
    out["payload_safety_factor_at_20N_mu0.5"] = round(P.GRIP_FORCE_N[1] / out["clamp_needed_for_payload_mu_pad_0.5"], 2)
    out["self_locking_on_power_loss"] = SCREW_MU[0] > math.tan(lead_angle()) * math.cos(math.radians(15))
    return out

# ======================================================================================
# 7. export + report
# ======================================================================================
COLORS = {"custom": cq.Color(0.75, 0.78, 0.82), "bought": cq.Color(0.25, 0.25, 0.3), "ctx": cq.Color(0.6, 0.6, 0.6, 0.3)}
PRINT_COLOR = cq.Color(0.9, 0.5, 0.1)


def save_assembly(items: list[Item], path: Path, name: str) -> None:
    asm = cq.Assembly(name=name)
    for it in items:
        col = PRINT_COLOR if it.kind == "custom" and CUSTOM[it.part][0].startswith("print") else COLORS[it.kind]
        asm.add(it.w, name=it.name.replace("@", "_at_ax"), loc=cq.Location((0, 0, P.X_BEAM_MID_Z)), color=col)
    asm.save(str(path))


def relayer_dxf(path: Path) -> None:
    """integration rev E (DFM): circles smaller than the laser minimum go to TAPPED (M3 2.5 / M4 3.3 tap drills) or REAM
    (D3 dowels cut 2.8, ream D3 H7) layers with a note; everything else stays on CUT."""
    import ezdxf
    doc = ezdxf.readfile(str(path))
    for name, col in (("CUT", 7), ("TAPPED", 5), ("REAM", 3), ("NOTE", 8)):
        if name not in doc.layers:
            doc.layers.add(name, color=col)
    msp = doc.modelspace()
    for e in list(msp):
        e.dxf.layer = "CUT"
        if e.dxftype() == "CIRCLE":
            d = 2 * e.dxf.radius
            if abs(d - 1.6) < 0.05 or abs(d - 2.5) < 0.05 or abs(d - 3.3) < 0.05:   # rev G4: + M2 tap drill
                e.dxf.layer = "TAPPED"
            elif abs(d - 3.0) < 0.05:
                e.dxf.radius = 1.4
                e.dxf.layer = "REAM"
    msp.add_text("TAPPED: drill + tap (M3 2.5 / M4 3.3) as a secondary op; REAM: cut D2.8, ream D3 H7; CP-1: the 3.4 holes of the EJ-1 roots (local x -35.5 per EJ axis) are 90 deg countersunk to 6.6 on the BACK face (see STEP)", height=3.0,
                 dxfattribs={"layer": "NOTE"}).set_placement((0, -20))
    doc.saveas(str(path))


def bbox(w: cq.Workplane) -> list[float]:
    b = w.val().BoundingBox()
    return [round(b.xlen, 1), round(b.ylen, 1), round(b.zlen, 1)]


def config(s_by_axis: list[float], gap: float = P.GRIP_OPEN, static=None) -> list[Item]:
    items = list(static) if static is not None else static_items()
    for i, s in enumerate(s_by_axis):
        items += axis_items(i, s, gap)
    return items


def main() -> dict:
    OUT.mkdir(parents=True, exist_ok=True)
    for old in OUT.glob("*"):
        old.unlink()
    rep: dict = {"status": "FABRICATION CANDIDATE for a first prototype -- not validated; bought-part, pipette and labware "
                           "dimensions nominal (gate G0); values tagged 'measure on the part' in build.py",
                 "revision": "B (params rev B: four tools), 12 Nov 2026; mechanical G3 (13 Nov 2026: ZHS-1 Z hard stop, EJ-1/HP-8 holes)",
                 "frame": "carriage-local (params.py); global z = local z + %.0f" % P.X_BEAM_MID_Z,
                 "tools": {n: AXES_X[i] for i, n in enumerate(P.TOOL_NAMES)},
                 "parts": [], "bought_parts": [], "checks": {}, "params_change_requests": []}
    static = static_items()
    top = config([0.0] * P.N_TOOLS, static=static)
    gi = P.TOOL_NAMES.index("gripper")

    # ---- per-part export (one instance each) + part table ----
    seen: dict[str, Item] = {}
    for it in top:
        if it.kind == "custom" and it.part not in seen:
            seen[it.part] = it
    for n, part, w, kind in tool_items(0, name="module_adapter"):                 # generic module adapter: alternative, not fitted
        if kind == "custom" and part not in seen:
            seen[part] = Item(n, part, w, kind)
    all_valid = True
    for part, it in seen.items():
        proc, mat, t, qty, dens = CUSTOM[part]
        w = FLAT[part]() if part in FLAT else it.w
        valid = bool(w.val().isValid())
        all_valid &= valid
        cq.exporters.export(w, str(OUT / f"{part}.step"))
        if proc == "sheet_laser":
            # rev G3: CP-1 carries countersinks on its BACK face (EJ-1 roots): the drawing is cut from the through-hole variant
            wd = carriage_plate_flat(back_csk=False) if part == "CP-1_carriage_plate" else w
            cq.exporters.export(wd.faces("<Z"), str(OUT / f"{part}.dxf"))
            relayer_dxf(OUT / f"{part}.dxf")              # integration rev E (DFM): TAPPED / REAM layers
        if proc.startswith("print"):
            cq.exporters.export(w, str(OUT / f"{part}.stl"), tolerance=0.02, angularTolerance=0.2)
        env = bbox(w)
        rep["parts"].append({"name": part, "process": proc, "material": mat, "thickness": t, "qty": qty,
                             "envelope_mm": env, "mass_kg": round(w.val().Volume() * RHO[dens], 3), "valid_solid": valid,
                             "within_size_limit": (max(env) <= 250.0) if proc != "sheet_laser" else
                             (sorted(env)[-1] <= 600.0 and sorted(env)[-2] <= 400.0),
                             **({"note": "alternative: generic adapter for a bought pipetting module (PIPETTE_UPGRADE); qty 0 on the first build"}
                                if part.startswith("PA-") else {})})
    for k, (desc, m, q) in BOUGHT.items():
        rep["bought_parts"].append({"name": k, "description": desc, "unit_mass_kg": round(m, 3), "qty": q})
    rep["part_count_by_process"] = {}
    for p_ in rep["parts"]:
        rep["part_count_by_process"].setdefault(p_["process"], {"designs": 0, "pieces": 0})
        if p_["qty"]:
            rep["part_count_by_process"][p_["process"]]["designs"] += 1
            rep["part_count_by_process"][p_["process"]]["pieces"] += p_["qty"]
    rep["part_count_by_process"]["bought"] = {"lines": len(BOUGHT), "pieces": sum(q for _, _, q in BOUGHT.values())}
    if getattr(P, "REV_J", False):      # rev J: printed setting gauges (STEP + STL), listed with the head parts
        rep["checks"]["setting_gauges_rev_j"] = {}
        for gname, (fn, proves, replaces) in GAUGES.items():
            w = fn()
            cq.exporters.export(w, str(OUT / f"{gname}.step"))
            cq.exporters.export(w, str(OUT / f"{gname}.stl"), tolerance=0.02, angularTolerance=0.2)
            env = bbox(w)
            rep["parts"].append({"name": gname, "process": "print_mjf_pa12", "material": "PA12 (MJF)", "thickness": None, "qty": 1,
                                 "envelope_mm": env, "mass_kg": round(w.val().Volume() * RHO["pa12"], 4), "valid_solid": bool(w.val().isValid()),
                                 "within_size_limit": max(env) <= 250.0, "note": "rev J setting gauge (a tool, shared with the BR-1 rig)"})
            rep["checks"]["setting_gauges_rev_j"][gname] = {
                "proves": proves, "replaces": replaces, "print_tolerance_mm": P.MJF_PRINT_TOL,
                "acceptance": f"caliper-measured on receipt, used only within +/-{P.GAUGE_ACCEPT_TOL} mm of nominal; the measured value is recorded"}
        zs = {n: zhs_setting(n) for n in P.Z_HARD_STOP_TOOLS if n in P.TOOL_NAMES}
        rep["checks"]["setting_gauges_rev_j"]["GG-1_zhs_setting_gauge"]["effect_on_the_stop_stack"] = {
            n: {"tolerance_worst_case_mm": z["tolerance_worst_case_mm"], "clearance_after_worst_case_mm": z["clearance_after_worst_case_mm"],
                "pass_min_gap_after_tolerances": z["pass_min_gap_after_tolerances"]} for n, z in zs.items()}
    if getattr(P, "REV_J", False):      # rev J: machinability and fit of the merged cradle bodies
        rep["checks"]["cradle_body_rev_j"] = {n: {**cb_machining(n), "mass_kg": round(hp_body(n).val().Volume() * RHO["al"], 3),
                                                  "replaces": ["HP-1", "HP-4", "HP-8", "HP-6"],
                                                  "upper_vblock_screw": f"ISO 4762 M4 x {vb_ear_screw_len(n):g} from the front, "
                                                                        f"engagement {CB_BACK_T - 0.5:g} mm in CB-1"}
                                              for n in HP_SETS if n in P.TOOL_NAMES}

    # ---- assemblies ----
    save_assembly(top, OUT / "head_assembly.step", "LH1_head")
    # rev G2: stiffener drawing = STEP with the rivnut holes + hole table (datum: tube end at STIFF_Z[0], centre of the 40 face)
    cq.exporters.export(b_stiffener(), str(OUT / "STF-1_head_stiffener_steel_tube.step"))
    with open(OUT / "STF-1_head_stiffener_hole_table.csv", "w") as fh:
        fh.write("feature,face,x_mm_from_tube_centreline,z_mm_from_lower_end,diameter_mm\n")
        for sx in (-1, 1):
            for z in STIFF_BOLT_Z:
                fh.write(f"M5 steel rivnut hole (drill 7.0 +0.1/-0),40 mm face against CP-1,{sx * 10.0:.1f},{z - STIFF_Z[0]:.1f},{STIFF_RIVNUT_D}\n")
        fh.write(f"# steel rectangular tube 40 x 20 x 2, length {STIFF_LEN:.0f} mm, square cut +/-0.2, zinc plated after drilling; "
                 "rivnuts M5 steel, 0.5-3.0 grip; 4 pieces\n")
    save_assembly(top, OUT / "head_assembly_Ztop.step", "LH1_head_Ztop")
    for i, tag in ((gi, "gripper"), (P.N_TOOLS - 1, P.TOOL_NAMES[-1])):
        ss = [s_max(i) if j == i else 0.0 for j in range(P.N_TOOLS)]          # rev G3: an axis with a stop ends on it
        save_assembly(config(ss, static=static), OUT / f"head_assembly_Zbottom_{tag}.step", f"LH1_head_Zbottom_{tag}")

    c = rep["checks"]
    # ---- interference ----
    inter = {}
    n = P.N_TOOLS
    cfgs = [("all_top_grip_open", [0.0] * n, P.GRIP_OPEN), ("all_top_grip_closed", [0.0] * n, P.GRIP_CLOSED),
            ("all_bottom", [s_max(i) for i in range(n)], P.GRIP_OPEN)]          # rev G3: bottom = hard stop where fitted
    for i in range(n):
        for s, lab in ((P.TRAVEL_Z / 2, "mid"), (s_max(i), "bottom")):
            cfgs.append((f"{P.TOOL_NAMES[i]}_{lab}_others_top", [s if j == i else 0.0 for j in range(n)], P.GRIP_OPEN))
    cfgs.append(("gripper_bottom_closed", [P.TRAVEL_Z if j == gi else 0.0 for j in range(n)], P.GRIP_CLOSED))
    for i in range(n):                                       # rev G: each axis at the method-B eject position
        cfgs.append((f"{P.TOOL_NAMES[i]}_eject_top_others_top", [-EJ if j == i else 0.0 for j in range(n)], P.GRIP_OPEN))
    total = 0.0
    for name, ss, gap in cfgs:
        hits = interference(config(ss, gap, static=static))
        inter[name] = hits
        total += sum(h["mm3"] for h in hits)
    c["interference"] = {"configs": inter, "total_overlap_mm3": round(total, 3), "pass": total == 0.0,
                         "scope": "all custom AND simplified bought solids (+ X beam/rail context), pairwise, each axis "
                                  "top/mid/bottom with the others at top, all bottom, gripper open/closed; designed overlaps "
                                  "excluded: shaft/screw in coupler, screw in nuts/collar, rail in block, actuator rod in yoke"}

    # ---- Z travel ----
    blk_top_at_top = ZB_TOP + ZBLK_C + ZBLK_DZ + BLK_L / 2          # rev F: upper block
    blk_bot_at_bot = zb_of(P.TRAVEL_Z) + ZBLK_C - ZBLK_DZ - BLK_L / 2   # rev F: lower block
    s_mech = P.TRAVEL_Z + blk_bot_at_bot - RAIL_BOT
    c["z_travel"] = {"required_mm": P.TRAVEL_Z, "usable_between_home_and_soft_limit_mm": P.TRAVEL_Z,
                     "rail_limited_block_stroke_mm": round(P.Z_RAIL_LEN - BLK_L - 2 * ZBLK_DZ, 1),
                     "block_to_rail_top_margin_at_home_mm": round(RAIL_TOP - blk_top_at_top, 2),
                     "block_to_rail_bottom_margin_at_full_travel_mm": round(blk_bot_at_bot - RAIL_BOT, 2),
                     "nut_flange_to_coupler_gap_at_home_mm": round((SHELF_Z0 - COUPLER_L) - (ZB_TOP + P.TOOL_PLATE_H + NUT_FL_T), 2),
                     "nut_flange_to_coupler_gap_at_eject_top_mm": round((SHELF_Z0 - COUPLER_L) - (ZB_TOP + EJ + P.TOOL_PLATE_H + NUT_FL_T), 2),
                     "block_to_rail_top_margin_at_eject_top_mm": round(RAIL_TOP - blk_top_at_top - EJ, 2),
                     "nut_to_bearing_housing_gap_at_max_mech_travel_mm": round(
                         (zb_of(s_mech) + P.TOOL_PLATE_H - NUT_BODY_L) - Z_SHOULDER, 2),   # rev G4: to the upper thrust bearing
                     "lead_screw_length_mm": SCREW_Z[1] - SCREW_Z[0],
                     "pass": (P.Z_RAIL_LEN - BLK_L - 2 * ZBLK_DZ) >= P.TRAVEL_Z}
    c["tool_plate_lower_edge_at_top"] = {"global_z": ZB_TOP + P.X_BEAM_MID_Z, "required": P.TOOL_PLATE_BOTTOM_Z_AT_TOP,
                                         "pass": abs(ZB_TOP + P.X_BEAM_MID_Z - P.TOOL_PLATE_BOTTOM_Z_AT_TOP) < 1e-6}

    # ---- nozzle / fingertip heights achieved, lowest points, reach ----
    tp_top = P.TOOL_PLATE_BOTTOM_Z_AT_TOP
    tp_bot = tp_top - P.TRAVEL_Z
    noz_top, noz_bot = tp_top - NOZZLE_BELOW_TP, tp_bot - NOZZLE_BELOW_TP
    achieved, low = {}, {}
    for i, tname in enumerate(P.TOOL_NAMES):
        its = axis_items(i, 0.0)
        zmin = min(it.w.val().BoundingBox().zmin for it in its) + P.X_BEAM_MID_Z
        low[tname] = {"geometry_min_z_no_tip": round(zmin, 2)}
        if tname in HP_SETS:
            achieved[tname] = {"nozzle_end_z_at_top": round(zmin, 2), "required": P.PIPETTE_NOZZLE_Z_AT_TOP}
            low[tname].update({f"with_{k}_tip": round(zmin - v, 1) for k, v in P.TIP_BELOW_NOZZLE.items()})
        elif tname == "gripper":
            achieved[tname] = {"fingertip_z_at_top": round(zmin, 2), "required": P.GRIPPER_FINGERTIP_Z_AT_TOP}
    c["tool_heights_achieved"] = achieved
    worst_used = min(min(v for k, v in d.items() if "p1000" not in k) for d in low.values())
    c["lowest_point_at_top_vs_safe_transit"] = {
        "tools": low, "SAFE_TRANSIT_Z": P.SAFE_TRANSIT_Z, "MAX_LABWARE_H": P.MAX_LABWARE_H,
        "worst_without_p1000_tips": worst_used, "pass_without_p1000": worst_used >= P.SAFE_TRANSIT_Z,
        "p1000_tip_low_point": noz_top - P.TIP_BELOW_NOZZLE["p1000"],
        "p1000_vs_SAFE_TRANSIT_Z_mm": noz_top - P.TIP_BELOW_NOZZLE["p1000"] - P.SAFE_TRANSIT_Z,
        "note": "no 1000 uL tool in rev B (10 / 50 / 300 uL); a p1000 tip at the fixed 243 mm nozzle height would be "
                "15 mm below SAFE_TRANSIT_Z (5 mm above MAX_LABWARE_H, as params states)",
        "carriage_plate_bottom_global": P.CARRIAGE_PLATE_Z[0] + P.X_BEAM_MID_Z,
        "stiffener_bottom_global": STIFF_Z[0] + P.X_BEAM_MID_Z}
    well_bottom = P.LABWARE_SEAT_Z + PCR_TOP_ABOVE_SEAT - PCR_WELL_DEPTH
    pick_target = P.LABWARE_SEAT_Z + RACK200_COLLAR_ABOVE_SEAT - TIP_ENGAGE
    reach = {"nozzle_z_at_bottom_of_travel": noz_bot, "pcr_well_bottom_z (seat + 21 - 17)": well_bottom,
             "p200_rack_pickup_nozzle_target_z (assumed 62 mm collar, 8 mm engage)": pick_target,
             "pickup_margin_mm": round(pick_target - noz_bot, 1)}
    for k, L in P.TIP_BELOW_NOZZLE.items():
        end = noz_bot - L
        reach[f"{k}_tip_end_at_bottom"] = end
        reach[f"{k}_reaches_pcr_well_bottom"] = end <= well_bottom
        reach[f"{k}_tip_end_below_well_bottom_mm (travel to spare)"] = round(well_bottom - end, 1)
        reach[f"{k}_reaches_bottom_of_a_tip_in_its_rack"] = end <= P.LABWARE_SEAT_Z + RACK200_COLLAR_ABOVE_SEAT - L
    c["pipette_reach"] = reach
    fing_tip_bot = tp_bot - G_FINGER_L
    c["gripper_reach"] = {"finger_length_below_tool_plate_mm": G_FINGER_L, "finger_tip_z_at_bottom": fing_tip_bot,
                          "pad_z_range_at_bottom": [fing_tip_bot + 0.5, fing_tip_bot + 16.5],
                          "labware_side_z_range": [P.LABWARE_SEAT_Z, P.LABWARE_SEAT_Z + SBS_PLATE_H],
                          "pad_overlap_with_96well_plate_side_mm": round(P.LABWARE_SEAT_Z + SBS_PLATE_H - fing_tip_bot - 0.5, 2),
                          "pipettes_fully_raised_nozzle_z": noz_top}
    row_gap = P.NEST_PITCH_Y - P.SBS_W
    neighbour = P.NEST_PITCH_Y - P.SBS_W / 2
    c["gripper_finger_clearance"] = {"row_gap_mm": round(row_gap, 2),
                                     "finger_outer_face_from_plate_centre_open_mm": P.GRIP_OPEN / 2 + P.GRIP_FINGER_T,
                                     "neighbour_labware_edge_from_plate_centre_mm": round(neighbour, 2),
                                     "clearance_open_mm": round(neighbour - (P.GRIP_OPEN / 2 + P.GRIP_FINGER_T), 2),
                                     "gripper_body_half_depth_mm": P.TOOL_ENVELOPE[1] / 2,
                                     "body_clear_of_neighbour_row": P.TOOL_ENVELOPE[1] / 2 < neighbour,
                                     "pass": neighbour - (P.GRIP_OPEN / 2 + P.GRIP_FINGER_T) > 2.0}
    # 8-channel Y extent vs neighbours
    ext = {}
    for i, tname in enumerate(P.TOOL_NAMES):
        bbs = [it.w.val().BoundingBox() for it in axis_items(i, 0.0) if "@" in it.name]
        ext[tname] = {"x_local": [round(min(b.xmin for b in bbs), 1), round(max(b.xmax for b in bbs), 1)],
                      "y_local": [round(min(b.ymin for b in bbs), 1), round(max(b.ymax for b in bbs), 1)],
                      "y_in_front_of_tool_plate_mm": round(Y_TF - min(b.ymin for b in bbs), 1)}
    xgaps = [round(ext[P.TOOL_NAMES[k + 1]]["x_local"][0] - ext[P.TOOL_NAMES[k]]["x_local"][1], 1) for k in range(n - 1)]
    c["tool_extents"] = {"tools": ext, "x_gap_between_neighbouring_tools_mm": xgaps,
                         "TOOL_ENVELOPE_D": P.TOOL_ENVELOPE[1],
                         "eight_channel_nozzle_row_y_span_mm": 7 * P.WELL_PITCH_96,
                         "pass": min(xgaps) > 2.0 and all(v["y_in_front_of_tool_plate_mm"] <= P.TOOL_ENVELOPE[1] + 0.01 for v in ext.values()),
                         "note": "tools sit side by side in X at 74 mm pitch, all within D = 110 mm in Y; the 8-channel "
                                 "manifolds (along Y) never overlap a neighbour's X band"}
    c["gripper_force"] = gripper_force()
    plunger = {}
    for tname in HP_SETS:
        if tname in P.TOOL_NAMES:
            g = hp_geom(tname)
            f = {mu: round(2 * math.pi * HP_ACT_USABLE_NM * 0.35 / (HP_ACT_STEP_LEAD / 1000), 0) for mu in (0.15,)}
            plunger[tname] = {"stroke_needed_mm": g["stroke"], "rod_travel_available_mm": round(g["button_top"] + 1.0 - (g["plate_z"] + 6.0), 1),
                              "force_needed_N": g["force"], "force_available_N (0.03 N m, eta 0.35, lead 1 mm)": f[0.15],
                              "pass": f[0.15] >= g["force"] and g["button_top"] + 1.0 - (g["plate_z"] + 6.0) >= g["stroke"]}
    c["plunger_actuators"] = plunger

    # ---- motors, thrust, backdrive ----
    mb = sorted([it.w.val().BoundingBox() for it in static if it.part == N17_ID], key=lambda b: b.xmin)
    gaps = [round(mb[k + 1].xmin - mb[k].xmax, 2) for k in range(len(mb) - 1)]
    c["z_motor_clearance"] = {"motor_to_motor_gap_mm": gaps, "nominal": round(P.TOOL_PITCH_X - N17, 2),
                              "outer_motor_x_extent_mm": [round(mb[0].xmin, 2), round(mb[-1].xmax, 2)],
                              "carriage_plate_x": list(P.CARRIAGE_PLATE_X),
                              "pass": min(gaps) > 5.0 and mb[-1].xmax <= P.CARRIAGE_PLATE_X[1]}
    zc_tp = [it for it in axis_items(0, 0.0) if it.part in ("ZC-1_z_carriage", "TP-1_tool_plate", "T8_antibacklash_nut", "MGN12H_block")]
    z_moving = sum(item_mass(it) for it in zc_tp) + P.TOOL_MAX_MASS
    c["z_thrust"] = thrust_check(z_moving)
    c["z_backdrive"] = backdrive_check(z_moving)
    c["z_moving_mass_per_axis_kg"] = round(z_moving, 3)
    c["z_hard_stop_rev_g3"] = z_hard_stop_report(z_moving, static)
    c["z_screw_axial_support_rev_g4"] = screw_support_report(z_moving)
    # ---- rev G: method-B ejection: finger engagement, finger clearance to every moving part at every stroke ----
    ejc = {}
    for i in range(n):
        tname = P.TOOL_NAMES[i]
        if tname not in P.EJECTOR:
            continue
        e = P.EJECTOR[tname]
        fing = [it for it in static if it.name == f"EJ{i}"][0].w.val()
        worst = (1e9, None)
        for j in range(n):
            for sv in [-EJ, -EJ / 2] + [float(v) for v in range(0, int(P.TRAVEL_Z) + 1, 10)]:
                if j != i and sv < 0:
                    continue
                for it in axis_items(j, sv):
                    if it.name == f"ejb{i}":
                        continue
                    b1, b2 = it.w.val().BoundingBox(), fing.BoundingBox()
                    if (b1.xmin > b2.xmax + 6 or b2.xmin > b1.xmax + 6 or b1.ymin > b2.ymax + 6 or b2.ymin > b1.ymax + 6
                            or b1.zmin > b2.zmax + 6 or b2.zmin > b1.zmax + 6):
                        continue
                    d = fing.distance(it.w.val())
                    if d < worst[0]:
                        worst = (round(d, 2), f"{it.name} s={sv:g}")
        btn = [it for it in axis_items(i, -EJ) if it.name == f"ejb{i}"][0].w.val()
        z_under = ZB_TOP + hp_geom(tname)["h_z1"] + e["top_above_handle"] + P.Z_EJECT_GAP
        pen = round(btn.BoundingBox().zmax - z_under, 2)          # button pushed down by the finger at eject top
        ejc[tname] = {"button_nominal": e, "finger_min_clearance_mm": worst[0], "nearest": worst[1],
                      "button_travel_at_eject_top_mm": pen, "ejector_stroke_mm": e["stroke"],
                      "full_stroke_reached": pen >= e["stroke"] - 1e-6,
                      "z_force_needed_N": e["force_N"], "z_lift_force_worst_N": c["z_thrust"]["mu_0.20"]["lift_N"],
                      "thrust_margin": round(c["z_thrust"]["mu_0.20"]["lift_N"] / (e["force_N"] + z_moving * 9.81), 2)}
    c["eject_method_b"] = {"z_eject_travel_mm": EJ, "per_tool": ejc,
                           "home": "home = NORMAL top (axis 0): KW10 roller lever raised by Z_EJECT_TRAVEL, side-actuated by the ZUC-1 -X bar, "
                                   "made from the normal top up to the eject top; homing stops at the switch edge and never enters the over-travel",
                           "z_up_chain": "D2F roller-lever switch raised by Z_EJECT_TRAVEL, side-actuated by ZUC-1: made from the normal top up to the eject top",
                           "coupler": "D20 x 20 (was x 30); motor shelf +2 mm (local z 98)",
                           "values": "button position, stroke and force are NOMINAL (params EJECTOR): measure each pipette"}


    # ---- head mass / CoM / deflection ----
    c["head_mass"] = mass_props(top)
    yb = Y_TF + G_YC
    zbk = zb_of(P.TRAVEL_Z) + ZBLK_C
    c["carriage_deflection_150N_outer_tool"] = deflection_check(yb, zbk, 150.0)
    c["carriage_deflection_200N_outer_tool"] = deflection_check(yb, zbk, 200.0)

    allb = [it.w.val().BoundingBox() for it in top if it.kind != "ctx"]
    c["head_envelope"] = {"x": [round(min(b.xmin for b in allb), 1), round(max(b.xmax for b in allb), 1)],
                          "y_local": [round(min(b.ymin for b in allb), 1), round(max(b.ymax for b in allb), 1)],
                          "top_global_z": round(max(b.zmax for b in allb) + P.X_BEAM_MID_Z, 1),
                          "ENCLOSURE_H": P.ENCLOSURE_H, "CARRIAGE_W": P.CARRIAGE_W}
    c["all_solids_valid"] = all_valid
    c["dfm"] = {"sheet_thicknesses_used": sorted({p["thickness"] for p in rep["parts"] if p["process"] == "sheet_laser"}),
                "tapped_holes_only_in_ge_4mm_al": True, "printed_parts_low_load_only": True,
                "all_within_size_limits": all(p["within_size_limit"] for p in rep["parts"])}
    d150 = c["carriage_deflection_150N_outer_tool"]
    rep["params_change_requests"] = [
        {"param": "Z_RAIL_LEN", "current": P.Z_RAIL_LEN, "requested": 260.0,
         "why": "250 mm rail - 45.4 mm block = 204.6 mm stroke leaves %.1f / %.1f mm (top / bottom) beyond TRAVEL_Z; a homing "
                "overshoot larger than that runs the Z block off its rail. 260 mm (top unchanged, bottom 10 mm lower, still "
                "above the plate bottom) gives 7 mm per end." % (c["z_travel"]["block_to_rail_top_margin_at_home_mm"],
                                                                c["z_travel"]["block_to_rail_bottom_margin_at_full_travel_mm"])},
        {"param": "GRIPPER_FINGERTIP_Z_AT_TOP", "current": P.GRIPPER_FINGERTIP_Z_AT_TOP, "requested": 207.0,
         "why": "with 200 mm travel the fingertip reaches z = %.0f: the pads overlap only the top %.1f mm of a 14.35 mm SBS "
                "plate side. 207 puts the tip 2 mm above the seat and keeps 37 mm above SAFE_TRANSIT_Z."
                % (fing_tip_bot, c["gripper_reach"]["pad_overlap_with_96well_plate_side_mm"])},
        {"param": "SAFE_TRANSIT_Z vs PIPETTE_NOZZLE_Z_AT_TOP", "current": [P.SAFE_TRANSIT_Z, P.PIPETTE_NOZZLE_Z_AT_TOP],
         "requested": "state SAFE_TRANSIT_Z applies to the tips actually fitted (10/50/300 uL: lowest %.0f)" % worst_used,
         "why": "a 1000 uL tip on a 243 mm nozzle is at 155 < 170"},
        {"param": "TOOL_ENVELOPE", "current": list(P.TOOL_ENVELOPE),
         "requested": "add: nothing above the tool plate top within 7 mm of the plate front (local y > -54)",
         "why": "Z motors (42.3 mm square on the screw axis, local y -31) overhang to local y -51.7 from local z 102; at the "
                "top of travel a tool part above the tool plate at y > -54 would hit them"},
        {"param": "carriage stiffening (new interface)", "current": "6 mm plate only",
         "requested": "%d x 2040 x %.0f mm behind the plate below the X beam (local y -13..+7, global z %.0f..%.0f)"
                      % (P.N_TOOLS, STIFF_LEN, STIFF_Z[0] + P.X_BEAM_MID_Z, STIFF_Z[1] + P.X_BEAM_MID_Z),
         "why": "150 N on the outer tool: plate alone %.2f mm at the Z block, %.2f mm 200 mm lower at the tip; with stiffener "
                "%.3f / %.3f mm. Gantry must confirm nothing passes behind the carriage there."
                % (d150["plate_only"]["deflection_at_block_mm"], d150["plate_only"]["tip_shift_y_mm_200mm_below_block"],
                   d150["plate_plus_2040"]["deflection_at_block_mm"], d150["plate_plus_2040"]["tip_shift_y_mm_200mm_below_block"])},
        {"param": "board / chain anchor (new interface)", "current": "-",
         "requested": "board tray occupies local x -155..-109, y -19..+40, z 80..105; chain anchor x 9..35, y -19..+25, z 64..86",
         "why": "both reach back over the X beam top; the gantry cable chain must land on the anchor"},
    ]
    if c["head_mass"]["exceeds_assumption"]:
        rep["params_change_requests"].append({"param": "HEAD_MOVING_MASS_ASSUMED", "current": P.HEAD_MOVING_MASS_ASSUMED,
                                              "requested": c["head_mass"]["total_kg"], "why": "computed head mass exceeds the assumption"})
    rep["first_assembly_risks"] = [
        "Z block runs within ~2.3 mm of each rail end: a homing overshoot or wrong soft limit drops balls out of the block",
        "lead-screw to rail parallelism: screw fixed only at the bottom housing and floating in a flexible coupler; housing and "
        "motor bracket are located by tapped holes only -- align against the Z carriage at both ends before tightening",
        "0.5-1 mm static clearances: nut flange to plate front, bearing housing to Z carriage / tool plate, handle to plunger actuator",
        "hand-pipette dimensions are nominal: HP-1 hole heights, V vertices and the 8-ch pocket must be re-derived from the measured pipettes",
    ]
    rep["ejector_space_reserved"] = HP_EJ_RESERVED
    # rev G3: fastener schedule derived from the modelled holes (machine: still ordered as the 'fasteners' lot + explicit lines)
    rep["fastener_schedule_rev_g3"] = {
        "axes": {P.TOOL_NAMES[i]: axis_joints(i) for i in range(P.N_TOOLS)},
        "cradles": {n: cradle_joints(n) for n in HP_SETS if n in P.TOOL_NAMES},
        "spec": {**FASTENER_SPEC, **{k: BOUGHT[k][0] for k in ("ZHS_set_screw_M5x30_locknut", "ZHS_ISO4762_M4x10")}},
        "note": "counts come from the same point lists that cut the holes; OPEN rows are joints the model has no holes for"}
    (OUT / "report.json").write_text(json.dumps(rep, indent=1, default=str))
    return rep


if __name__ == "__main__":
    r = main()
    summary = {k: r["checks"][k] for k in ("z_travel", "tool_heights_achieved", "lowest_point_at_top_vs_safe_transit",
                                            "pipette_reach", "z_thrust", "head_mass", "carriage_deflection_150N_outer_tool",
                                            "gripper_force", "gripper_reach", "gripper_finger_clearance", "z_motor_clearance",
                                            "tool_extents", "plunger_actuators", "head_envelope")}
    summary["interference_total_mm3"] = r["checks"]["interference"]["total_overlap_mm3"]
    summary["interference_hits"] = {k: v for k, v in r["checks"]["interference"]["configs"].items() if v}
    summary["all_solids_valid"] = r["checks"]["all_solids_valid"]
    summary["part_count_by_process"] = r["part_count_by_process"]
    print(json.dumps(summary, indent=1, default=str))
