"""LH-1 revision L: exchange head (ADR-010), its stiffness and dynamics, and the bench-rig parts that prove it (CT-1..CT-9).

Run from the project root:  .venv/bin/python cad/lh1/head_l/build.py
Out: cad/lh1/head_l/out/{report.json, head_l_assembly.step, rig_l_assembly.step, <part>.step/.stl/.dxf, head_l_parts.csv}

Status: CAD and calculation only.  Nothing built or measured.  Every number is CALCULATED here from the model or from an
ASSUMED input named beside it; figures taken from the concept study (evidence/external/exchange_head_study_2026-11-14.txt)
are marked STUDY.

Coordinates: carriage-local, as the present head (cad/lh1/head): x along the X beam, y negative toward the operator,
z = 0 at the X beam mid-height, which revision L raises to global z P.L_X_BEAM_MID_Z (430).  s = Z stroke from the top
(0 .. L_Z_TRAVEL; negative = eject over-travel).
"""
from __future__ import annotations

import csv
import importlib.util
import json
import math
import sys
from pathlib import Path

import cadquery as cq

HERE = Path(__file__).resolve().parent
LH1 = HERE.parent
sys.path.insert(0, str(LH1))
sys.path.insert(0, str(HERE))
import params as P  # noqa: E402
import latch as LT  # noqa: E402  (V1: toggle latch, dock lift; calculations only)

OUT = HERE / "out"


def _load(name, rel):
    spec = importlib.util.spec_from_file_location(name, str(LH1 / rel))
    m = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(m)
    return m


DK = _load("deck_build_l", "deck_modules/build.py")
B, cyl, mv, fuse_all, cut_all = DK.B, DK.cyl, DK.mv, DK.fuse_all, DK.cut_all
V = cq.Vector
RHO = {"al": 2.70e-6, "steel": 7.85e-6, "pa12": 1.01e-6}
ZB = P.L_X_BEAM_MID_Z                     # global z of local z 0


def cyl_dir(p, d, r, h):
    return cq.Solid.makeCylinder(r, h, V(*p), V(*d))


# =====================================================================================================
# 1. geometry (carriage-local)
# =====================================================================================================
Z_LOW = 110.0 - P.L_Z_TRAVEL - 120.0                                       # rev L pass 2: CP / rail bottom follows the stroke (lower Z block on the rail)
DZT = -(P.L_Z_TRAVEL - 230.0)                                               # parts hung from the CP bottom keep their global height (camera)
CP = dict(x=(-135.0, 135.0), y=(-25.0, -15.0), z=(Z_LOW, 124.0), t=10.0)    # carriage plate CP-L, Al 6061-T6 10 mm (was 6), lightened
RAIL = dict(w=15.0, h=10.0, z=(Z_LOW, 124.0))                               # MGN15 rail on CP front face (y -25..-35)
BLK = dict(w=34.0, l=58.8, h=6.0, pitch=60.0)                              # MGN15H block (outer body beyond the rail), 2 per axis
ZC = dict(hx=32.0, y=(-51.0, -41.0), h=160.0)                              # Z carriage ZC-L, Al 10 mm; top at z_top(s)
ZTOP0 = 110.0                                                               # ZC top at s = 0 (= coupling plane); lower block stays on the rail at full stroke
SCREW = dict(dx=30.0, y=-33.0, d=8.0)                                       # Tr8x2 beside the rail, behind the ZC
MOTOR = dict(a=42.3, h=48.0, z0=126.0)                                       # NEMA17 on top, above the CP top edge
BR = dict(x=(-27.0, 45.0), y=(-146.0, -55.0), t=20.0, web=(-61.0, -51.0), web_h=60.0)   # coupling bracket CBR-L (Al 6061-T6)
C = P.L_COUPLING
AXY = (0.0, -102.5)                                                         # coupling axis = nozzle (row centre) in x, y rel. axis x
R = C["R"]
BALLS = [(-R / 2, AXY[1] + R * math.sqrt(3) / 2), (-R / 2, AXY[1] - R * math.sqrt(3) / 2), (R, AXY[1])]  # edge along Y, single ball +X
ZOFF = (C["ball_d"] / 2 + C["pin_d"] / 2) * math.cos(math.radians(45))      # ball centre below the pin centres (contact at 45 deg)
PIN_SEP = 2 * (C["ball_d"] / 2 + C["pin_d"] / 2) * math.sin(math.radians(45))
STUD = dict(x=14.0, d=10.0, head_d=16.0, head_h=6.0)                         # bought retention knob (45 deg cone under a D16 head), off-axis 14 mm
# V1 (21 Nov 2026): latch LTG-1 = keyhole jaw + 4-disc A20 cartridge + link + crank on needle bearings + arm with roller + two constant-
# force springs (cad/lh1/head_l/latch.py).  Carriage-local at s = 0: jaw plane z 124..130 in a channel in the bracket top, stud head
# above it (z 130..136), crank pivot P at (y -140, z 127); the arm leaves the bracket's front face; the dock lift bar pushes its roller up.
LATCH = dict(x=(6.5, 21.5), jaw_y=(-126.0, -90.0), t=6.0, travel_open=LT.G["jaw_clear_mm"], P=LT.G["P_yz"], cart_d=14.0,
             chan=(6.0, 22.0, -146.5, -77.0, 118.0, 131.0))
# latch sensing (integration 23 Nov 2026, latch.SENSE): flag LSF-L on the jaw, LOCK and EMPTY sealed switches on bracket LSB-L on the
# coupling-bracket top (x 23..38 beside the stud, behind the flag), ladder LLB-L (lever splices + 4 resistors) on top of the switches
# toggle planes (x relative to the stud line xs = axis + 14): see latch_parts
TOGGLE = dict(link_hw=3.0, crank_x=((-6.0, -4.0), (4.0, 6.0)), pin_x=((-9.0, -4.0), (4.0, 12.0)), cheek_x=((-9.0, -7.0), (7.0, 9.0)),
              arm_x=(9.0, 12.0), roller_x=(12.0, 20.0), pad_x=(12.2, 21.7), drum_x=((-18.5, -9.5), (23.0, 32.0)), drum_y=-155.0,
              drum_lugs=((-20.0, -18.5), (32.0, 33.5)), anchor_x=(-20.0, 33.5), slot_x=(-7.0, 13.0))
SW = dict(flag_y0=-94.0, riser_x=(18.0, 21.5), arm_z=(14.5, 17.5), arm_x1=38.0,          # arm z relative to the jaw top
          body=(20.0, 6.4, 10.2), lock_x=24.0, empty_x=31.0, body_y1=-95.0, plunger_d=2.0,
          bracket=(23.0, 38.0, -119.0, -115.0), ladder=(23.0, 43.0, -119.0, -95.0, 21.5, 37.5))   # ladder z relative to the bracket top
PUSHER = dict(xy=(0.0, AXY[1]), a=LT.LGA35["flange_mm"], h=LT.LGA35["motor_len_mm"], guide_d=LT.LGA35["guide_d_mm"], guide_len=LT.LGA35["guide_len_mm"],
              ge_above_bracket=24.6, stroke=LT.LGA35["stroke_mm"], rod_d=6.0, collar=(12.0, 6.0))
# V6: Nanotec LGA351S12-A-UIAP-038 (sourcing file first choice), guide end 24.6 mm above the bracket top so the rod's set collar (D12 x 6,
# the RETRACTED switch flag) can travel the full 38.1 mm stroke inside a D13 bore in the bracket and stop 0.5 mm above its underside
VEE = dict(L=16.0, W=14.0, H=8.0)                                              # bought hardened 90 deg vee seat (radial), 16 x 14 x 8
EJECT_XY = (-22.0, AXY[1])                                                   # eject pad / finger axis, 22 mm from the axis (clear of the pusher)
PUSHROD_XY = (28.0, -128.0)                                                  # force-switch push-rod (clear of pins, latch)
HALL_XY = [(30.0, -68.0), (30.0, -77.0), (30.0, -86.0), (39.0, -77.0)]       # 4 magnet / Hall positions (tool code)
TPL = dict(t=10.0)                                                           # tool plate TPL-L, Al 7075-T6 10 mm (+ web for the cradle)
GRIP = dict(env=(62.0, 107.0, 156.0), finger_l=113.0, drop=50.0)                        # GR-1 kept (present gripper), envelope from head report

TOOLS = {   # ASSUMED hand-pipette geometry in its cradle: plunger-button top below the tool plate, Hn = tool-plate top -> nozzle end
    "M50": dict(name="eight-channel 5-50 uL", tip="p200", eight=True, stroke=18.0, plunger_N=40.0, mass=1.25, Hn=None, w=66.0),
    "M300": dict(name="eight-channel 30-300 uL", tip="p300", eight=True, stroke=22.0, plunger_N=50.0, mass=1.30, Hn=None, w=66.0),
    "S10": dict(name="single 0.5-10 uL", tip="p10", eight=False, stroke=15.0, plunger_N=25.0, mass=0.70, Hn=None, w=44.0),
    "S100": dict(name="single 10-100 uL", tip="p200", eight=False, stroke=18.0, plunger_N=30.0, mass=0.75, Hn=None, w=44.0),
    "S1000": dict(name="single 100-1,000 uL", tip="p1000", eight=False, stroke=28.0, plunger_N=60.0, mass=0.85, Hn=None, w=48.0),
    "M10": dict(name="eight-channel 0.5-10 uL", tip="p10", eight=True, stroke=15.0, plunger_N=35.0, mass=1.2, Hn=None, w=66.0),
    "SYR": dict(name="syringe dispense tool (10 mL, direct drive)", tip=None, eight=False, stroke=32.0, plunger_N=80.0, mass=1.0, Hn=None, w=60.0)}
TIP_L = dict(P.TIP_BELOW_NOZZLE)
SHAFT = {"S10": [(0.0, 4.5), (20.0, 7.0), (60.0, 28.0)], "S100": [(0.0, 5.0), (25.0, 8.0), (60.0, 28.0)],
         "S1000": [(0.0, 8.0), (30.0, 11.0), (60.0, 26.0)]}     # ASSUMED shaft OD vs height above the nozzle end (deck_k PIPETTES_K class)


def z_top(s):
    return ZTOP0 - s


def tpl_top(s):
    return z_top(s) - ZOFF                     # tool-plate top: ball centres lie on it (balls half sunk)


for _k, _t in TOOLS.items():
    _nz = P.L_NOZZLE_TOP_Z["p1000" if _t["tip"] == "p1000" else "standard"]
    _t["Hn"] = round(tpl_top(0) + ZB - _nz, 1)      # tool-plate top to nozzle end, standard heights (study 3.2)


def z_axis_fixed(x):
    parts = {}
    parts["rail"] = B(x - RAIL["w"] / 2, x + RAIL["w"] / 2, -35.0, -25.0, *RAIL["z"])
    parts["motor"] = B(x + SCREW["dx"] - MOTOR["a"] / 2, x + SCREW["dx"] + MOTOR["a"] / 2, SCREW["y"] - MOTOR["a"] / 2,
                       SCREW["y"] + MOTOR["a"] / 2, MOTOR["z0"], MOTOR["z0"] + MOTOR["h"])
    parts["screw"] = cyl(x + SCREW["dx"], SCREW["y"], Z_LOW + 8.0, MOTOR["z0"], SCREW["d"])
    return parts


def latch_switch_parts(x, s, split=False):
    """LOCK and EMPTY sealed switches (pin plungers at their operating points, latch.SENSE), their bracket LSB-L and the ladder block LLB-L
    (envelope of 2 x 5-way + 2 x 2-way lever splices and the four resistors) on the coupling-bracket top; one solid unless split"""
    zt = z_top(s) + BR["t"]                                            # bracket top = jaw top
    L, W, H = SW["body"]
    zc = zt + (SW["arm_z"][0] + SW["arm_z"][1]) / 2.0                  # plunger axis at the flag arm's mid-height
    parts = {}
    for nm, x0, op in (("lock", SW["lock_x"], LT.SENSE["lock_op_mm"]), ("empty", SW["empty_x"], LT.SENSE["empty_op_mm"])):
        body = B(x + x0, x + x0 + W, SW["body_y1"] - L, SW["body_y1"], zc - H / 2.0, zc + H / 2.0)
        tip_y = SW["flag_y0"] + op                                     # plunger end where the flag's face is when the switch operates
        plunger = cq.Solid.makeCylinder(SW["plunger_d"] / 2.0, tip_y - SW["body_y1"] + 0.01, V(x + x0 + W / 2.0, SW["body_y1"] - 0.01, zc), V(0, 1, 0))
        parts[f"latch_switch_{nm}"] = body.fuse(plunger)
    bx0, bx1, by0, by1 = SW["bracket"]
    parts["latch_switch_bracket"] = fuse_all([B(x + bx0, x + bx1, by0, by1, zt, zc + H / 2.0),
                                              B(x + bx0, x + bx1, by1 - 0.01, SW["body_y1"], zc - H / 2.0 - 2.0, zc - H / 2.0)])
    lx0, lx1, ly0, ly1, lz0, lz1 = SW["ladder"]
    parts["latch_ladder"] = B(x + lx0, x + lx1, ly0, ly1, zt + lz0, zt + lz1)
    return parts if split else fuse_all(list(parts.values()))


def z_axis_moving(x, s, tool=True, gripper=False, phi=None):
    zt = z_top(s)
    p = {}
    for k in (0, 1):
        zc = zt - 30.0 - k * BLK["pitch"]
        p[f"block{k}"] = B(x - BLK["w"] / 2, x + BLK["w"] / 2, -41.0, -35.0, zc - BLK["l"] / 2, zc + BLK["l"] / 2)
    p["zc"] = B(x - ZC["hx"], x + ZC["hx"], ZC["y"][0], ZC["y"][1], zt - ZC["h"], zt).cut(
        B(x - 20.0, x + 20.0, ZC["y"][0] - 1, ZC["y"][1] + 1, zt - ZC["h"] + 20.0, zt - 100.0))   # lightening window below the blocks
    p["nut"] = B(x + SCREW["dx"] - 11.0, x + SCREW["dx"] + 11.0, -41.0, -27.0, zt - 40.0, zt - 25.0).cut(
        cyl(x + SCREW["dx"], SCREW["y"], zt - 41.0, zt - 24.0, SCREW["d"] + 1.0))
    if gripper:
        gx, gy, gz = GRIP["env"]
        p["gripper_hanger"] = B(x - gx / 2, x + gx / 2, -61.0, -51.0, zt - GRIP["drop"] - 20.0, zt)
        zt = zt - GRIP["drop"]                                   # GR-1 hung lower so the fingertips reach the deck at full stroke
        p["gripper_body"] = B(x - gx / 2, x + gx / 2, -61.0 - gy, -61.0, zt - gz, zt)
        for sy in (-1, 1):
            p[f"finger{sy}"] = B(x - 20.0, x + 20.0, -104.0 + sy * 44.0 - 2.0, -104.0 + sy * 44.0 + 2.0, zt - gz - GRIP["finger_l"] + 15.0, zt - gz + 10.0)
        return p
    # coupling bracket: horizontal arm on top of the ZC + a vertical web bolted to the ZC front face
    arm = B(x + BR["x"][0], x + BR["x"][1], BR["y"][0], BR["y"][1], zt, zt + BR["t"])
    web = B(x + BR["x"][0], x + BR["x"][1], BR["web"][0], BR["web"][1], zt - BR["web_h"], zt)
    pc, ph = PUSHER["collar"]
    holes = [cyl(x + PUSHER["xy"][0], PUSHER["xy"][1], zt - 1, zt + BR["t"] + 1, pc + 1.0),          # rod + RETRACTED collar bore (D13)
             cyl(x + EJECT_XY[0], EJECT_XY[1], zt - 1, zt + BR["t"] + 1, 9.0),
             cyl(x + PUSHROD_XY[0], PUSHROD_XY[1], zt - 1, zt + BR["t"] + 1, 4.0),
             cyl(x + STUD["x"], AXY[1], zt - 1, zt + BR["t"] + 1, STUD["head_d"] + 2.0)]
    # V18: three bought hardened vee seats in shallow pockets on the bracket underside (no reamed grooves, no pressed pins)
    vees, pockets_v = [], []
    for bx, by in BALLS:
        ang = math.degrees(math.atan2(by - AXY[1], bx))
        v, pk = vee_seat(x + bx, by, ang, zt)
        vees.append(v)
        pockets_v.append(pk)
    cx0, cx1, cy0, cy1, cz0, cz1 = LATCH["chan"]
    chan = B(x + cx0, x + cx1, cy0, cy1, zt + (cz0 - ZTOP0), zt + BR["t"] + 1)          # latch channel / toggle pocket from the top
    front_slot = B(x + 9.0, x + 19.0, BR["y"][0] - 1.0, cy0 + 2.0, zt - 1, zt + BR["t"] + 1)   # the arm leaves through the front face
    pockets = [B(x - 22.0, x - 4.0, -92.0, -66.0, zt + 6.0, zt + BR["t"] + 1)]           # lightening pocket (clear of holes)
    p["bracket"] = cut_all(fuse_all([arm, web]), holes + pockets_v + pockets + [chan, front_slot])
    p["vees"] = fuse_all(vees)
    p.update(latch_parts(x, s, phi))
    p["latch_switch"] = latch_switch_parts(x, s)
    p["pushrod_switch"] = B(x + PUSHROD_XY[0] - 6.5, x + PUSHROD_XY[0] + 6.5, PUSHROD_XY[1] - 3.0, PUSHROD_XY[1] + 3.0, zt + BR["t"] + 2.0, zt + BR["t"] + 22.0)
    p["hall_board"] = B(x + 25.0, x + 44.0, -91.0, -63.0, zt + BR["t"], zt + BR["t"] + 1.6)
    p.update(pusher_parts(x, s))
    return p


def vee_seat(cx, cy, ang_deg, zt):
    """bought hardened 90 deg vee seat, groove radial (toward the ball-triangle centroid); ball D10 on it with contacts at 45 deg.
    Returns (seat solid, bracket pocket).  Vertex line 7.07 mm above the ball centre (ball centre on the tool-plate top plane)."""
    zc = zt - ZOFF                                   # ball centre
    zv = zc + (C["ball_d"] / 2) / math.sin(math.radians(45))
    L, W, H = VEE["L"], VEE["W"], VEE["H"]
    z0 = zt - 3.0
    blk = cq.Workplane("XY").box(L, W, H, centered=(True, True, False)).translate((0, 0, z0)).val()
    d = 12.0
    groove = cq.Workplane("YZ").polyline([(0.0, zv), (-d, zv - d), (d, zv - d)]).close().extrude(L + 2).translate((-L / 2 - 1, 0, 0)).val()
    seat = blk.cut(groove)
    pocket = cq.Workplane("XY").box(L + 0.2, W + 0.2, 5.0 + 3.0 + 0.1, centered=(True, True, False)).translate((0, 0, z0)).val()
    rot = lambda sh: sh.rotate(V(0, 0, 0), V(0, 0, 1), ang_deg).translate(V(cx, cy, 0))  # noqa: E731
    return rot(seat), rot(pocket)


def _rod_yz(y0, z0, y1, z1, x0, x1, w):
    """flat bar of width w (in the YZ plane) from (y0, z0) to (y1, z1), extruded x0..x1"""
    dy, dz = y1 - y0, z1 - z0
    n = math.hypot(dy, dz)
    oy, oz = -dz / n * w / 2, dy / n * w / 2
    return (cq.Workplane("YZ").polyline([(y0 + oy, z0 + oz), (y1 + oy, z1 + oz), (y1 - oy, z1 - oz), (y0 - oy, z0 - oz)]).close()
            .extrude(x1 - x0).translate((x0, 0, 0)).val())


def latch_parts(x, s, phi=None):
    """LTG-1 latch at opening angle phi (deg; None = locked, 4 deg past dead centre): jaw, cartridge, link, crank, arm, roller, pivot pin,
    constant-force spring drums and their anchor plate.  Carriage-local; the toggle plane is the jaw mid-plane z 127 (at s 0)."""
    ph = -LT.G["over_deg"] if phi is None else phi
    dz = -s
    zt = z_top(s)
    Py, Pz = LATCH["P"][0], LATCH["P"][1] + dz
    xs = x + STUD["x"]
    jaw_dy = LT.jaw_disp(ph)
    jy0, jy1 = LATCH["jaw_y"]
    out = {}
    jaw = B(x + LATCH["x"][0], x + LATCH["x"][1], jy0 + jaw_dy, jy1 + jaw_dy, Pz - 3.0, Pz + 3.0)
    hole_y = AXY[1] - LT.G["jaw_clear_mm"] + jaw_dy
    jaw = cut_all(jaw, [cyl(xs, hole_y, Pz - 4, Pz + 4, STUD["head_d"] + 1.0), B(xs - 5.5, xs + 5.5, hole_y, AXY[1] + 5.5 + jaw_dy, Pz - 4, Pz + 4),
                        cyl(xs, AXY[1] + 5.5 + jaw_dy, Pz - 4, Pz + 4, 11.0)])
    # sensing flag LSF-L (integration 23 Nov 2026): 3 mm steel strip screwed to the jaw top at its +Y end, rising in the gap between the
    # pusher U-standoff (x <= 17.5) and the Hall clamp plate (x >= 24), then running +X above the Hall switch tops to press the LOCK and
    # EMPTY switches' plungers (its -Y face at y FLAG_Y0 when engaged)
    fy0 = SW["flag_y0"] + jaw_dy
    flag = fuse_all([B(x + SW["riser_x"][0], x + SW["riser_x"][1], fy0, fy0 + 3.0, Pz + 3.0 - 0.01, Pz + 3.0 + SW["arm_z"][1]),
                     B(x + SW["riser_x"][0], x + SW["arm_x1"], fy0, fy0 + 3.0, Pz + 3.0 + SW["arm_z"][0], Pz + 3.0 + SW["arm_z"][1])])
    out["jaw"] = jaw.fuse(flag)
    sl = LT.slider(ph)                                              # cartridge pin, distance from P along +Y
    cart_y0 = Py + sl
    out["cartridge"] = cyl_dir((xs, cart_y0, Pz), (0, 1, 0), LATCH["cart_d"] / 2, max(1.0, jy0 + jaw_dy - cart_y0 + 0.5))
    th = math.radians(180.0 - ph)                                     # crank angle (from +Y)
    cpy, cpz = Py + LT.G["r"] * math.cos(th), Pz + LT.G["r"] * math.sin(th)
    # integration 23 Nov 2026 (stroke check): the toggle is laid out in planes so nothing passes through anything over the 128 deg swing:
    # link (centre plane, it crosses the pivot axis at dead centre) between two crank plates on STUB pivot pins (no pin across the
    # centre), the cheeks outside the crank plates, the arm outside the +X cheek, the roller outboard of the arm, its bar pad beyond
    # every toggle part in X (the pad rises past the pivot when the latch opens)
    tg = TOGGLE
    out["link"] = _rod_yz(cpy, cpz, cart_y0, Pz, xs - tg["link_hw"], xs + tg["link_hw"], 6.0)
    out["crank"] = fuse_all([cyl_dir((xs + a, Py, Pz), (1, 0, 0), 6.0, b - a) for a, b in tg["crank_x"]] +
                            [cyl_dir((xs + tg["crank_x"][0][0], cpy, cpz), (1, 0, 0), LT.PIN["d"] / 2, tg["crank_x"][1][1] - tg["crank_x"][0][0])])
    ry, rz = LT.roller_yz(ph, s)
    out["arm"] = _rod_yz(Py, Pz, ry, rz, xs + tg["arm_x"][0], xs + tg["arm_x"][1], 7.0)
    out["roller"] = cyl_dir((xs + tg["roller_x"][0], ry, rz), (1, 0, 0), LT.G["roller_d"] / 2, tg["roller_x"][1] - tg["roller_x"][0])
    out["pivot_pin"] = fuse_all([cyl_dir((xs + a, Py, Pz), (1, 0, 0), LT.PIN["d"] / 2, b - a) for a, b in tg["pin_x"]])
    # two constant-force springs: drums behind the lift bar (y -154, clear of the bar's path), one outboard of each side of the toggle, on
    # an anchor plate hanging from the bracket front face; the strips (not modelled) run up to the arm tip / roller pin
    ry0, rz0 = LT.roller_yz(-LT.G["over_deg"], s)
    zd = rz0 - 22.0
    out["cf_spring_drums"] = fuse_all([cyl_dir((xs + a, tg["drum_y"], zd), (1, 0, 0), 7.0, b - a) for a, b in tg["drum_x"]])
    ax0, ax1 = tg["anchor_x"]
    plate = B(xs + ax0, xs + ax1, BR["y"][0] - 3.0, BR["y"][0], zd - 10.0, zt + BR["t"]).cut(
        B(xs + tg["slot_x"][0], xs + tg["slot_x"][1], BR["y"][0] - 4.0, BR["y"][0] + 1.0, Pz - 26.0, zt + BR["t"] + 1.0))   # toggle slot
    out["spring_anchor"] = fuse_all([plate] + [B(xs + a, xs + b, tg["drum_y"] - 2.0, BR["y"][0] - 2.9, zd - 10.0, zd + 3.0) for a, b in tg["drum_lugs"]] +
                                    [B(xs + a, xs + b, Py - 7.0, BR["y"][0] - 2.9, Pz - 7.0, Pz + 7.0) for a, b in tg["cheek_x"]])   # pivot cheeks
    return out


def pusher_parts(x, s):
    """V6: Nanotec LGA351S12-A-UIAP-038 on a U standoff PB-L (top plate on its 26 mm M3 square, walls in front of and behind the stud
    head); rod extension D6 on the M4 screw tip; set collar D12 x 6 (RETRACTED switch flag) travelling in a D13 bore in the bracket"""
    zt = z_top(s)
    a = PUSHER["a"]
    xb, yb = x + PUSHER["xy"][0], PUSHER["xy"][1]
    zg = zt + BR["t"] + PUSHER["ge_above_bracket"]                  # guide end (screw tip when retracted)
    zf = zg + PUSHER["guide_len"]                                    # flange face
    out = {"pusher": fuse_all([cyl(xb, yb, zg, zf + 0.01, PUSHER["guide_d"]), B(xb - a / 2, xb + a / 2, yb - a / 2, yb + a / 2, zf, zf + PUSHER["h"])])}
    top = B(xb - a / 2, xb + a / 2, yb - a / 2, yb + a / 2, zf - 6.0, zf).cut(cyl(xb, yb, zf - 7, zf + 1, PUSHER["guide_d"] + 0.4))
    walls = [B(xb - a / 2, xb + a / 2, yb + sy * a / 2 - (3.0 if sy > 0 else 0.0), yb + sy * a / 2 + (0.0 if sy > 0 else 3.0), zt + BR["t"], zf - 6.0 + 0.01)
             for sy in (-1, 1)]
    # notch in the -Y wall over the latch channel: the cartridge (D14, 4 mm proud of the bracket top) passes under it as the jaw opens
    out["pusher_bracket"] = fuse_all([top] + walls).cut(B(x + LATCH["chan"][0], x + LATCH["chan"][1], yb - a / 2 - 1.0, yb - a / 2 + 4.0,
                                                         zt + BR["t"] - 1.0, zt + BR["t"] + 8.0))
    pc, ph = PUSHER["collar"]
    out["pusher_collar"] = cyl(xb, yb, zg - ph, zg, pc)
    out["pusher_rod"] = cyl(xb, yb, tpl_top(s) - 2.0 + 3.0, zg, PUSHER["rod_d"])      # retracted: rod end 3 mm above the slider pad
    out["pusher_retracted_switch"] = B(xb - 14.5, xb - 8.0, yb - 5.0, yb + 5.0, zg - ph - 2.0, zg - 0.5)
    return out


# rev L pass 2: real cradle bodies (CB-1 pattern turned horizontal).  ASSUMED bought-pipette envelopes: handle diameter hd, mass pip_kg
# (class values for mechanical hand pipettes: WEIGH and MEASURE the ones bought; the cradle V-block heights are set from the measured
# handle so that the nozzle sits Hn below the tool-plate top).
# eight-channel manifold class from the rev J head (HP_SETS): m_t thickness along X, m_w length along Y (the row), m_h height, nl nozzle cones
PIP = {"M50": dict(hd=32.0, pip_kg=0.24, m_t=32.0, m_w=90.0, m_h=50.0, nl=14.0), "M300": dict(hd=32.0, pip_kg=0.26, m_t=36.0, m_w=95.0, m_h=55.0, nl=18.0),
       "M10": dict(hd=32.0, pip_kg=0.22, m_t=32.0, m_w=90.0, m_h=50.0, nl=14.0),
       "S10": dict(hd=30.0, pip_kg=0.11), "S100": dict(hd=30.0, pip_kg=0.12), "S1000": dict(hd=32.0, pip_kg=0.14)}
CR = dict(fl_t=8.0, bp_y=(-77.0, -69.0), pad_h=12.0, pad_y0=-96.0, strap_t=8.0, lug_x=22.0, lug_y=(-69.0, -57.0), lug_dz=(-172.0, -162.0),
          lug_hole=6.0, upper_dz=(-62.0, -50.0), notch_hx=10.5, cheek=(9.5, 15.5))
# the dock interface every tool shares (bay saddle pins under the two lugs): lug z relative to the tool-plate top, x +/-20, y -69..-57
CR_SCREWS = [(-20.0, -82.0), (-20.0, -124.0), (8.0, -72.0), (10.0, -134.0)]     # 4 x M4 from the TPL top into the flange (clear of balls,
CR_DOWELS = {"round": (-6.0, -70.0), "slot": (36.0, -120.0)}                     # magnets, stud, eject and push-rod); dowels asymmetric: ONE WAY


def _vcut(x, y_v, z0, z1, opening=-1.0, depth=60.0):
    """90 deg V prism along Z, vertex at (x, y_v), opening toward sign(opening) in Y"""
    pts = [(x, y_v), (x - depth, y_v + opening * depth), (x + depth, y_v + opening * depth)]
    return cq.Workplane("XY").workplane(offset=z0).polyline(pts).close().extrude(z1 - z0).val()


def cradle_parts(x, s, key):
    """CBL-<tool>: one machined cradle body (flange under the tool plate + back plate + integral V-pads + dock lugs) and its small
    parts.  Two set-ups: OP1 tool axis -Y (front): back-plate face, V-grooves (chamfer mill), notch, cheek, strap holes (Y);
    OP2 tool axis -Z (top): flange face, 4 tapped M4, 2 dowel holes (round + slot), slider, eject and push-rod bores, lug holes."""
    t, g = TOOLS[key], PIP[key]
    top = tpl_top(s)
    zf1 = top - TPL["t"]
    zf0 = zf1 - CR["fl_t"]
    nz = top - t["Hn"]
    ay = AXY[1]
    w, r = t["w"], g["hd"] / 2
    yv = ay + r * math.sqrt(2.0)                 # V vertex (contacts at 45 deg)
    eight = t["eight"]
    mt, nl, mh = (g["m_t"], g["nl"], g["m_h"]) if eight else (0.0, 0.0, 0.0)
    bp0 = nz + nl + 5.0 if eight else nz + 58.0  # back plate bottom
    flange = B(x + BR["x"][0], x + BR["x"][1], -142.0, -64.0, zf0, zf1)
    xb0 = max(-w / 2, BR["x"][0])                # never wider than the tool plate on the -X side (bay pitch: tool width = plate width 72)
    back = B(x + xb0, x + w / 2, CR["bp_y"][0], CR["bp_y"][1], bp0, zf0 + 0.01)
    pads = []
    z_hi = (top + CR["upper_dz"][0], top + CR["upper_dz"][1])
    z_lo = (nz + nl + 5.0, nz + nl + mh - 5.0) if eight else (nz + 62.0, nz + 74.0)
    hx = min(w / 2, 25.0)
    pads.append(B(x - hx, x + hx, CR["pad_y0"], CR["bp_y"][0] + 0.01, *z_hi))
    if not eight:
        pads.append(B(x - hx, x + hx, CR["pad_y0"], CR["bp_y"][0] + 0.01, *z_lo))
    lugs = [B(x + sx * CR["lug_x"] - 6.0, x + sx * CR["lug_x"] + 6.0, CR["lug_y"][0] - 0.01, CR["lug_y"][1], top + CR["lug_dz"][0], top + CR["lug_dz"][1])
            for sx in (-1, 1)]
    body = fuse_all([flange, back] + pads + lugs)
    cuts = [_vcut(x, yv, z_hi[0] - 1, z_hi[1] + 1)]
    if not eight:
        sv = ay + 5.0 * math.sqrt(2.0) + 9.0     # lower V on the handle base (same handle diameter, ASSUMED)
        cuts.append(_vcut(x, yv, z_lo[0] - 1, z_lo[1] + 1))
    else:
        cuts.append(B(x - mt / 2 - 1.5, x + mt / 2 + 1.5, CR["bp_y"][0] - 1, CR["bp_y"][1] + 1, bp0 - 1, nz + nl + mh + 2.0))   # manifold notch
    cuts += [cyl(x, ay, zf0 - 1, zf1 + 1, 10.4),                                  # slider bush bore
             cyl(x - 20.0, ay, zf0 - 1, zf1 + 1, 8.4),                            # eject plunger bush bore
             cyl(x + PUSHROD_XY[0], PUSHROD_XY[1], zf0 - 1, zf1 + 1, 4.2)]        # push-rod
    cuts += [cyl(x + a, b, zf0 - 1, zf1 + 1, 3.3) for a, b in CR_SCREWS]          # M4 tapping
    cuts += [cyl(x + CR_DOWELS["round"][0], CR_DOWELS["round"][1], zf1 - 6, zf1 + 1, 4.0),
             B(x + CR_DOWELS["slot"][0] - 2.0, x + CR_DOWELS["slot"][0] + 2.0, CR_DOWELS["slot"][1] - 3.0, CR_DOWELS["slot"][1] + 3.0, zf1 - 6, zf1 + 1)]
    cuts += [cyl(x - CR["lug_x"], -63.0, top + CR["lug_dz"][0] - 1, top + CR["lug_dz"][1] + 1, CR["lug_hole"]),        # round (bay pin)
             B(x + CR["lug_x"] - 5.0, x + CR["lug_x"] + 5.0, -66.0, -60.0, top + CR["lug_dz"][0] - 1, top + CR["lug_dz"][1] + 1)]   # slot along X
    # lightening windows in the back plate between the pads (from the front, OP1)
    zw0, zw1 = (nz + nl + mh + 10.0) if eight else z_lo[1] + 12.0, z_hi[0] - 12.0
    if zw1 - zw0 > 30.0:
        cuts.append(B(x + xb0 + 7.0, x + w / 2 - 7.0, CR["bp_y"][0] - 1, CR["bp_y"][1] + 1, zw0, zw1))
    out = {"cradle": cut_all(body, cuts)}
    # straps (front opposing V, 2 x M4 along Y into the pad) and, for the eight-channel, the loose manifold cheek
    strap_y = (ay - r - 6.0, CR["pad_y0"] - 1.0)
    st = []
    for z0_, z1_ in ([z_hi] if eight else [z_hi, z_lo]):
        sb = B(x - hx, x + hx, ay - r * math.sqrt(2.0) - 6.0, strap_y[1], z0_, z1_)
        st.append(sb.cut(_vcut(x, ay - r * math.sqrt(2.0), z0_ - 1, z1_ + 1, opening=1.0)))
    out["straps"] = fuse_all(st)
    if eight:
        c0, c1 = mt / 2 + 0.5, mt / 2 + 6.5
        yc0 = AXY[1] - g["m_w"] / 2 + 4.0
        out["cheek_fixed"] = B(x + c0, x + c1, yc0, CR["bp_y"][0] + 0.01, *z_lo)
        out["cheek_loose"] = B(x - c1, x - c0, yc0, CR["bp_y"][0] - 0.5, *z_lo)
    out["slider"] = cyl(x, ay, zf1 - 13.0, top - 2.0, 8.0)
    out["eject_plunger"] = cyl(x - 20.0, ay, zf0 - 7.0, top - TPL["t"] + 0.5, 8.0)
    ca = (z_hi[0] - 12.0, z_hi[0] - 4.0)          # force collar + arm to the push-rod (disc-spring stack under the upper pad)
    out["force_collar"] = fuse_all([cyl(x, ay, ca[0], ca[1], g["hd"] + 12.0).cut(cyl(x, ay, ca[0] - 1, ca[1] + 1, g["hd"])),
                                    B(x + r + 2.0, x + PUSHROD_XY[0] + 4.0, PUSHROD_XY[1] - 4.0, ay - 4.0, ca[0], ca[1])])
    return out


def pipette_envelope(x, s, key):
    t = TOOLS[key]
    top = tpl_top(s)
    nz = top - t["Hn"]
    htop = top - TPL["t"] - CR["fl_t"] - 2.0
    if t["eight"]:
        g = PIP[key]
        m0, m1 = nz + g["nl"], nz + g["nl"] + g["m_h"]
        return {"pipette": fuse_all([cyl(x, AXY[1], m1 - 0.01, htop, g["hd"]),
                                     B(x - g["m_t"] / 2, x + g["m_t"] / 2, AXY[1] - g["m_w"] / 2, AXY[1] + g["m_w"] / 2, m0, m1)]),
                "nozzles": fuse_all([cyl(x, AXY[1] + (k - 3.5) * 9.0, nz, m0 + 0.01, 5.2) for k in range(8)])}
    return {"pipette": fuse_all([cyl(x, AXY[1], nz + 60.0, htop, PIP[key]["hd"]), cyl(x, AXY[1], nz + 6.0, nz + 60.0 + 0.01, 10.0)]),
            "nozzles": cyl(x, AXY[1], nz, nz + 6.0, 5.0)}


# SYR (integration 23 Nov 2026): a 10 mL luer-lock syringe DRIVEN DIRECTLY by the carriage pusher (family cr_answers; the 2:1 lever of
# revision L is withdrawn) with a stiffer bought return spring over the plunger rod.  Syringe class dimensions ASSUMED (UNVERIFIED part):
# barrel OD 16.0 / bore 14.5, 62 mm barrel below a 2 mm finger flange, thumb rest D20, plunger travel used 32 mm (5.3 mL), blunt jetting
# needle from the luer tip to the standard nozzle height.
SYR_T = dict(barrel_d=16.0, bore_d=14.5, barrel_l=62.0, flange=(30.0, 22.0, 2.0), thumb_d=20.0, thumb_t=3.0, rod_w=12.0, luer_l=8.0,
             travel=32.0, window=3.0, kg=0.55)
# return spring (bought compression spring, stainless 302 class; ASSUMED catalogue class, UNVERIFIED part): chosen so that the force
# with the plunger OUT (end of a refill, the weakest point) is >= 1.5 x the seal friction + check-valve cracking force, also at the
# spring's -10 % rate tolerance; installed between the thumb rest and a spring seat on the finger flange
SYR_SPRING = dict(wire_d=1.5, od=20.0, n_active=14, n_total=16, free_l=75.0, installed_l=60.0, G_MPa=69000.0, rate_tol=0.10,
                  uts_MPa=1900.0, seal_friction_N=3.5, check_valve_N=0.5, need_basis="seal friction 3.5 N for a 10 mL syringe plunger (ASSUMED, "
                  "UNVERIFIED: CT-SYR measures it) + inlet check-valve cracking 0.5 N (ASSUMED)")


def syringe_spring():
    """return spring force and margin (CALCULATED: rate = G d^4 / (8 D^3 n), Wahl-corrected shear stress at full stroke)"""
    sp = SYR_SPRING
    D = sp["od"] - sp["wire_d"]
    rate = sp["G_MPa"] * sp["wire_d"] ** 4 / (8.0 * D ** 3 * sp["n_active"])
    pre = sp["free_l"] - sp["installed_l"]
    full = pre + SYR_T["travel"]
    f_out, f_full = rate * pre, rate * full
    need = sp["seal_friction_N"] + sp["check_valve_N"]
    C_ = D / sp["wire_d"]
    K = (4 * C_ - 1) / (4 * C_ - 4) + 0.615 / C_
    tau = 8.0 * f_full * D * K / (math.pi * sp["wire_d"] ** 3)
    solid = sp["n_total"] * sp["wire_d"]
    return {"spring": "compression spring, wire %.1f mm stainless, OD %.0f, %d active / %d total coils, free length %.0f mm, installed %.0f mm "
                      "(plunger out) (ASSUMED catalogue class, UNVERIFIED part)" % (sp["wire_d"], sp["od"], sp["n_active"], sp["n_total"], sp["free_l"], sp["installed_l"]),
            "rate_N_per_mm": round(rate, 3), "force_plunger_out_N": round(f_out, 2), "force_full_stroke_N": round(f_full, 2),
            "force_plunger_out_low_tolerance_N": round(f_out * (1 - sp["rate_tol"]), 2),
            "need_N": need, "need_basis": sp["need_basis"], "margin": round(f_out / need, 2), "margin_low_tolerance": round(f_out * (1 - sp["rate_tol"]) / need, 2),
            "length_at_full_stroke_mm": round(sp["installed_l"] - SYR_T["travel"], 1), "solid_length_mm": round(solid, 1),
            "solid_margin_mm": round(sp["installed_l"] - SYR_T["travel"] - solid, 1),
            "shear_stress_full_stroke_MPa": round(tau), "shear_allowed_MPa (0.45 UTS, static)": round(0.45 * sp["uts_MPa"]),
            "stress_pass": tau <= 0.45 * sp["uts_MPa"],
            "replaces": "the 0.35 N/mm spring (5.2 N plunger-out, margin 1.3: failed the audit)"}


def syringe_parts(x, s):
    """SYR tool, direct drive (modelled in 3D): tool plate as every tool; a body (back plate on the tool plate's underside + a clamp block
    holding the finger flange); the syringe barrel, luer tip and blunt needle on the axis with the needle end at the standard nozzle
    height (Hn); the plunger rod and thumb rest under the tool plate, the thumb rest's top pad 2 mm below the tool-plate top on the axis
    (the standard slider interface the carriage pusher presses); the return spring (coil envelope) between the thumb rest and the
    flange seat.  Plunger OUT (start of a dispense)."""
    t = TOOLS["SYR"]
    top = tpl_top(s)
    nz = top - t["Hn"]
    zf1 = top - TPL["t"]
    ay = AXY[1]
    thumb_top = zf1 - 0.5
    thumb_bot = thumb_top - SYR_T["thumb_t"]
    flange_top = thumb_bot - SYR_SPRING["installed_l"]
    flange_bot = flange_top - SYR_T["flange"][2]
    barrel_bot = flange_bot - SYR_T["barrel_l"]
    out = {}
    body = fuse_all([B(x - 27.0, x + 30.0, -77.0, -69.0, flange_bot - 25.0, zf1), B(x - 27.0, x + 33.0, -142.0, -64.0, zf1 - 8.0, zf1)])
    out["body"] = cut_all(body, [cyl(x + EJECT_XY[0], EJECT_XY[1], zf1 - 9, zf1 + 1, 6.0), cyl(x + PUSHROD_XY[0], PUSHROD_XY[1], zf1 - 9, zf1 + 1, 4.2),
                                 cyl(x, ay, zf1 - 9, zf1 + 1, SYR_T["thumb_d"] + 3.0)])
    fl_w, fl_d, _ = SYR_T["flange"]
    out["syringe"] = fuse_all([cyl(x, ay, barrel_bot, flange_bot + 0.01, SYR_T["barrel_d"]),
                               B(x - fl_w / 2, x + fl_w / 2, ay - fl_d / 2, ay + fl_d / 2, flange_bot, flange_top),
                               cyl(x, ay, barrel_bot - SYR_T["luer_l"], barrel_bot + 0.01, 6.0),
                               cyl(x, ay, nz, barrel_bot - SYR_T["luer_l"] + 0.01, 1.2)])
    rod = fuse_all([B(x - SYR_T["rod_w"] / 2, x + SYR_T["rod_w"] / 2, ay - 0.75, ay + 0.75, flange_bot - 1.0, thumb_bot + 0.01),
                    B(x - 0.75, x + 0.75, ay - SYR_T["rod_w"] / 2, ay + SYR_T["rod_w"] / 2, flange_bot - 1.0, thumb_bot + 0.01)])
    out["plunger"] = fuse_all([rod, cyl(x, ay, thumb_bot, thumb_top, SYR_T["thumb_d"]), cyl(x, ay, thumb_top - 0.01, top - 2.0, 8.0)])
    od = SYR_SPRING["od"]
    out["return_spring"] = cyl(x, ay, flange_top, thumb_bot, od).cut(cyl(x, ay, flange_top - 1, thumb_bot + 1, od - 2 * SYR_SPRING["wire_d"]))
    out["clamp"] = B(x - 16.0, x + 16.0, -116.0, -77.0 + 0.01, flange_bot - 12.0, flange_bot).cut(cyl(x, ay, flange_bot - 13.0, flange_bot + 1.0, SYR_T["barrel_d"] + 0.4))
    return out


def syringe_spring_helix(x=0.0, y=0.0, z0=0.0):
    """the return spring as a true helix (exported as its own STEP; the head model uses the coil envelope)"""
    sp = SYR_SPRING
    D = sp["od"] - sp["wire_d"]
    pitch = (sp["installed_l"] - 2 * sp["wire_d"]) / sp["n_active"]
    h = pitch * sp["n_active"]
    path = cq.Wire.makeHelix(pitch, h, D / 2.0)
    prof = cq.Workplane("XZ").center(D / 2.0, 0).circle(sp["wire_d"] / 2.0)
    coil = prof.sweep(cq.Workplane().add(path), isFrenet=True).val()
    return coil.translate(V(x, y, z0 + sp["wire_d"]))


def tool_module(x, s, key):
    """passive tool: tool plate TPL-L (balls, stud, magnets, transfer pins), cradle body CBL-<tool> and its parts, bought pipette envelope."""
    t = TOOLS[key]
    top = tpl_top(s)
    p = {}
    plate = B(x + BR["x"][0], x + BR["x"][1], BR["y"][0], -63.0, top - TPL["t"], top)
    seats = [cyl(x + bx, by, top - C["ball_d"] / 2 - 0.5, top + 1, C["ball_d"]) for bx, by in BALLS]
    mags = [cyl(x + hx, hy, top - 2.0, top + 0.01, 4.0) for hx, hy in HALL_XY]
    p["tool_plate"] = cut_all(plate, seats + mags + [cyl(x + EJECT_XY[0], EJECT_XY[1], top - TPL["t"] - 1, top + 1, 4.2),
                                                     cyl(x + PUSHROD_XY[0], PUSHROD_XY[1], top - TPL["t"] - 1, top + 1, 3.2),
                                                     cyl(x, AXY[1], top - TPL["t"] - 1, top + 1, 8.4)]
                          + [cyl(x + a, b, top - TPL["t"] - 1, top + 1, 4.5) for a, b in CR_SCREWS]
                          + [cyl(x + a, b, top - TPL["t"] - 1, top - TPL["t"] + 6, 4.0) for a, b in CR_DOWELS.values()])
    p["balls"] = fuse_all([cq.Solid.makeSphere(C["ball_d"] / 2, V(x + bx, by, top), angleDegrees1=-90, angleDegrees2=90)
                           for bx, by in BALLS])
    zj = z_top(s) + BR["t"]                    # jaw top = bracket top: the bought retention knob's cone sits on the jaw's slot edges
    p["stud"] = fuse_all([cyl(x + STUD["x"], AXY[1], top - 8.0, zj + 0.01, STUD["d"]), cyl(x + STUD["x"], AXY[1], zj, zj + STUD["head_h"], STUD["head_d"])])
    p["magnets"] = fuse_all(mags)
    p["eject_pin"] = cyl(x + EJECT_XY[0], EJECT_XY[1], top - TPL["t"] - 1.0, top + 0.5, 4.0)
    p["push_rod"] = cyl(x + PUSHROD_XY[0], PUSHROD_XY[1], top + CR["upper_dz"][0] - 4.0, z_top(s) + BR["t"] + 2.0, 3.0)
    if t["tip"] is None:                       # syringe tool: envelope + drive
        p.update(syringe_parts(x, s))
        return p
    p.update(cradle_parts(x, s, key))
    p.update(pipette_envelope(x, s, key))
    return p


def tool_mass(key):
    """CALCULATED from the model (Al 6061 cradle parts, 7075 tool plate, steel small parts) + the ASSUMED bought pipette."""
    parts = tool_module(0.0, 0.0, key)
    steel = ("balls", "stud", "eject_pin", "push_rod", "slider", "eject_plunger", "straps")
    skip = ("pipette", "nozzles", "syringe", "plunger", "return_spring")
    m = {k: round(v.Volume() * RHO["steel" if k in steel else "al"], 4) for k, v in parts.items() if k not in skip}
    if key == "SYR":
        m["syringe_and_needle (ASSUMED)"] = 0.03
        m["return_spring_fasteners (ASSUMED)"] = 0.03
    else:
        m["bought_pipette (ASSUMED class value: WEIGH)"] = PIP[key]["pip_kg"]
        m["fasteners_bushes_disc_springs (ASSUMED)"] = 0.04
    tot = sum(m.values())
    cmx = sum(v.Center().x * v.Volume() for k, v in parts.items() if k not in skip) / sum(v.Volume() for k, v in parts.items() if k not in skip)
    return {"parts_kg": m, "total_kg": round(tot, 3), "limit_kg": P.TOOL_MAX_MASS, "pass": tot <= P.TOOL_MAX_MASS,
            "margin_kg": round(P.TOOL_MAX_MASS - tot, 3), "metal_parts_centre_x_from_axis_mm": round(cmx, 1),
            "width_x_mm": round(cq.Compound.makeCompound([v for k, v in parts.items()]).BoundingBox().xlen, 1)}


def cradle_report():
    out = {"pattern": "CB-1 (rev J) turned horizontal: ONE machined body per pipette = flange under the tool plate (4 x M4 from the tool-"
                      "plate top, two D4 dowels: round at (-6, -70), slot at (36, -120), asymmetric so it fits one way only) + back plate "
                      "with integral V-pads + two dock lugs (D6 round hole at x -20, slot at x +20, z -172..-162 below the tool-plate top: "
                      "the standard bay interface); separate opposing-V straps (2 x M4 each); eight-channel tools add two 6 mm manifold "
                      "cheeks below a notch in the back plate",
           "setups": {"OP1 (tool axis -Y, back face on parallels)": "back-plate face, V-grooves with a 90 deg chamfer mill, manifold notch, "
                                                                    "lightening window, strap tapped holes",
                      "OP2 (tool axis -Z, flange face up, back face on the fixed jaw)": "flange top face, 4 x M4 tapped, dowel hole + slot, "
                                                                                       "slider / eject / push-rod bores, lug hole + slot"},
           "assumed": "pipette handle diameters, lengths and masses are class values (PIP, TOOLS): MEASURE and WEIGH each pipette, then "
                      "set the V-pad heights so the nozzle is Hn below the tool-plate top", "tools": {}}
    for k in TOOLS:
        if k == "SYR":
            continue
        cp_ = cradle_parts(0.0, 0.0, k)
        bb = cp_["cradle"].BoundingBox()
        out["tools"][k] = {"name": TOOLS[k]["name"], "valid": cp_["cradle"].isValid(), "envelope_mm": [round(bb.xlen, 1), round(bb.ylen, 1), round(bb.zlen, 1)],
                           "body_kg": round(cp_["cradle"].Volume() * RHO["al"], 3), "Hn_mm": TOOLS[k]["Hn"],
                           "first_build": k != "S100", "one_way": "dowel pair asymmetric about the axis; lugs hole/slot; flange outline follows "
                                                                   "the tool plate (x -27..45)"}
    return out


def head_fixed():
    cp = B(*CP["x"], *CP["y"], *CP["z"])
    lw = [B(xx - 20.0, xx + 20.0, CP["y"][0] - 1, CP["y"][1] + 1, -200.0, -60.0) for xx in (-45.0, 45.0)]   # lightening windows between axes
    p = {"cp": cut_all(cp, lw)}
    for ax, x in P.L_AXIS_X.items():
        for k, s_ in z_axis_fixed(x).items():
            p[f"{ax}:{k}"] = s_
    # X blocks (MGN15H x 4) behind the plate, X belt clamp, camera VC-1 + ring light, eject fingers (fixed)
    for xb in (-95.0, 95.0):
        for zb in (-20.0, 20.0):
            p[f"xblock{xb:+.0f}{zb:+.0f}"] = B(xb - 17.0, xb + 17.0, -15.0, -5.0, zb - 15.0, zb + 15.0)
    p["camera"] = B(-152.0, -140.0, -60.0, -30.0, -210.0 + DZT, -190.0 + DZT)
    p["camera_arm"] = B(-152.0, -135.0, -30.0, -25.0, -215.0 + DZT, -185.0 + DZT)
    p["ring_light"] = cyl(-146.0, -45.0, -214.0 + DZT, -210.0 + DZT, 28.0)
    for ax in ("carriage_A", "carriage_B"):
        x = P.L_AXIS_X[ax]
        fx, fy = x + EJECT_XY[0], EJECT_XY[1]
        # finger: a steel pin hanging from an arm on a post at the CP top edge; enters the bracket hole only at over-travel
        zarm = ZTOP0 + BR["t"] + 20.0
        p[f"{ax}:eject_arm"] = B(fx - 4.0, fx + 4.0, fy - 4.0, -25.0, zarm, zarm + 8.0)
        p[f"{ax}:eject_post"] = B(fx - 4.0, fx + 4.0, -33.0, -25.0, CP["z"][1], zarm + 8.0)
        p[f"{ax}:eject_pin"] = cyl(fx, fy, ZTOP0 + BR["t"] + Z_EJ_PIN_ABOVE, zarm, 6.0)
    return p


Z_EJ_PIN_ABOVE = 1.0         # finger end this far above the bracket top at s = 0; it enters the bracket hole at over-travel


def head_parts(sA=0.0, sB=0.0, sG=0.0, toolA="M300", toolB="S10"):
    parts = {f"fixed:{k}": v for k, v in head_fixed().items()}
    for ax, s, tool in (("carriage_A", sA, toolA), ("carriage_B", sB, toolB)):
        x = P.L_AXIS_X[ax]
        for k, v in z_axis_moving(x, s).items():
            parts[f"{ax}:{k}"] = v
        if tool:
            for k, v in tool_module(x, s, tool).items():
                parts[f"{ax}:tool:{k}"] = v
    for k, v in z_axis_moving(P.L_AXIS_X["gripper"], sG, gripper=True).items():
        parts[f"gripper:{k}"] = v
    return parts


# pairs that touch by design (kinematic contacts, nested parts) are not interferences
def _expected(a, b):
    ga, gb = a.split(":")[0], b.split(":")[0]
    if ga != gb and not (a.startswith("fixed") or b.startswith("fixed")):
        return False
    n = {a.split(":")[-1], b.split(":")[-1]}
    ok = [{"balls", "vees"}, {"balls", "tool_plate"}, {"vees", "bracket"}, {"stud", "tool_plate"}, {"stud", "jaw"}, {"jaw", "bracket"},
          {"cartridge", "bracket"}, {"cartridge", "jaw"}, {"link", "cartridge"}, {"link", "crank"}, {"link", "bracket"}, {"crank", "bracket"},
          {"crank", "pivot_pin"}, {"crank", "arm"}, {"arm", "roller"}, {"arm", "pivot_pin"}, {"arm", "bracket"}, {"pivot_pin", "bracket"},
          {"link", "pivot_pin"}, {"link", "arm"}, {"cf_spring_drums", "spring_anchor"}, {"spring_anchor", "bracket"},
          {"pivot_pin", "spring_anchor"}, {"crank", "spring_anchor"}, {"arm", "spring_anchor"}, {"link", "spring_anchor"}, {"roller", "spring_anchor"},
          {"pusher_collar", "pusher_rod"}, {"pusher_collar", "pusher"}, {"pusher_retracted_switch", "pusher_bracket"}, {"pusher_collar", "bracket"},
          {"magnets", "tool_plate"}, {"eject_pin", "tool_plate"}, {"push_rod", "tool_plate"}, {"push_rod", "bracket"},
          {"pusher_rod", "pusher"}, {"pusher_rod", "bracket"}, {"pusher_rod", "pusher_bracket"}, {"zc", "bracket"}, {"zc", "block0"},
          {"zc", "block1"}, {"nut", "screw"}, {"nut", "zc"}, {"rail", "cp"}, {"block0", "rail"}, {"block1", "rail"}, {"screw", "motor"},
          {"pusher", "pusher_bracket"}, {"pusher_bracket", "bracket"}, {"latch_switch", "bracket"}, {"pushrod_switch", "push_rod"},
          {"hall_board", "bracket"}, {"cradle", "tool_plate"}, {"pipette", "cradle"}, {"pipette", "nozzles"}, {"pipette", "tool_plate"},
          {"camera", "camera_arm"}, {"camera_arm", "cp"}, {"ring_light", "camera"}, {"eject_arm", "eject_post"}, {"eject_pin", "eject_arm"},
          {"eject_post", "cp"}, {"xblock-95-20", "cp"}, {"gripper_body", "zc"}, {"finger-1", "gripper_body"}, {"finger1", "gripper_body"}, {"gripper_hanger", "zc"},
          {"gripper_hanger", "gripper_body"},
          {"body", "tool_plate"}, {"stud", "bracket"}, {"vees", "tool_plate"},
          {"cradle", "straps"}, {"cradle", "cheek_fixed"}, {"cradle", "cheek_loose"}, {"pipette", "cheek_fixed"}, {"pipette", "cheek_loose"},
          {"pipette", "straps"}, {"cradle", "slider"}, {"tool_plate", "slider"}, {"cradle", "eject_plunger"}, {"tool_plate", "eject_plunger"},
          {"eject_pin", "eject_plunger"}, {"cradle", "push_rod"}, {"force_collar", "push_rod"}, {"force_collar", "pipette"},
          {"slider", "pusher_rod"}, {"pipette", "slider"}, {"pipette", "eject_plunger"}, {"cradle", "force_collar"},
          {"body", "clamp"}, {"syringe", "clamp"}, {"body", "syringe"}, {"plunger", "syringe"}, {"plunger", "tool_plate"}, {"plunger", "body"},
          {"return_spring", "syringe"}, {"return_spring", "plunger"}, {"plunger", "pusher_rod"}, {"latch_switch", "jaw"},
          {"tool_plate", "cradle"}]
    if any(n == o for o in ok):
        return True
    if any(k.startswith("xblock") for k in n) and "cp" in n:
        return True
    return False


def interference(parts, label, tol=1.0):
    names = list(parts)
    bbs = {k: v.BoundingBox() for k, v in parts.items()}
    hits = []
    for i, a in enumerate(names):
        for b in names[i + 1:]:
            ba, bb = bbs[a], bbs[b]
            if ba.xmax < bb.xmin or bb.xmax < ba.xmin or ba.ymax < bb.ymin or bb.ymax < ba.ymin or ba.zmax < bb.zmin or bb.zmax < ba.zmin:
                continue
            if _expected(a, b):
                continue
            v = parts[a].intersect(parts[b]).Volume()
            if v > tol:
                hits.append({"config": label, "a": a, "b": b, "mm3": round(v, 2)})
    return hits


# =====================================================================================================
# 2. coupling calculations (study 2.3-2.5, recomputed for the geometry here)
# =====================================================================================================
Z_THRUST_RUN_N = None          # set by callers that read electronics/lh1/system.yaml (machine_family/mechanisms.py); None = not compared


def coupling_calcs(z_thrust=None, lift_thrust=None):
    """V1/V18: loads that try to pull the tool off (with the 1.5 margin), the preload the LTG-1 latch gives, contact stress, repeatability"""
    lt = LT.calcs(z_thrust, lift_thrust)
    Pmin, Pmax = lt["holding"]["preload_min_N"], lt["holding"]["preload_max_N"]
    Rm = R / 1000.0
    strip = max(P.STRIP_FORCE_N.values())
    pick = max(P.PICKUP_FORCE_N.values())
    plunger = max(t["plunger_N"] for t in TOOLS.values())
    eject = 90.0
    cases = {   # (pull-off T N, tipping moment M N m, note); weight + accelerations 19 N / 0.31 N m (study) added where they add
        "tip pick-up, 8 tips, %g N design load (%g x 1.25) half on an end tip 31.5 mm off (pushes the tool INTO the coupling; tips it)" %
        (P.DESIGN_LOAD_N, pick): (0.0, P.DESIGN_LOAD_N / 2 * 0.0315, "compressive: only the moment matters"),
        "tip eject %g N at the eject pad %.0f mm off axis" % (eject, abs(EJECT_XY[0]) + 4.0): (eject + 19.0, 0.31 + eject * (abs(EJECT_XY[0]) + 4.0) / 1000.0, "eject line at x -26 (family)"),
        "comb strip %g N, half on an end tip 31.5 mm off" % strip: (strip + 19.0 + 12.0, strip / 2 * 0.0315 + 0.01, "STRIP_FORCE_N, eight-channel"),
        "plunger %g N on axis (largest tool, SYR), pusher reaction" % plunger: (plunger + 19.0, 0.31, "TOOLS plunger_N"),
        "comb strip 250 N (sensitivity: real strip force unmeasured)": (250.0 + 31.0, 3.9, "CT-9 measures it")}
    rows = {}
    for k, (T, M, note) in cases.items():
        sep = T + 2 * M / Rm                            # preload at which a ball starts to unload
        rows[k] = {"pull_off_N": round(T, 1), "moment_Nm": round(M, 3), "preload_to_just_hold_N": round(sep), "needed_with_1.5_N": round(1.5 * sep),
                   "latch_preload_min_N": Pmin, "margin": round(Pmin / sep, 2), "pass_1.5": Pmin >= 1.5 * sep, "note": note}
    inside = STUD["x"] < R / 2
    Es = 115e3                                       # MPa, E* steel on steel
    rb = C["ball_d"] / 2

    def hertz(Ftot):
        Q = Ftot / (6 * math.cos(math.radians(45)))
        a = (3 * Q * rb / (4 * Es)) ** (1 / 3)
        return Q, a, 3 * Q / (2 * math.pi * a * a)
    h_work = hertz(Pmax + P.DESIGN_LOAD_N)
    h_crash = hertz(949.0)                          # Z stall force onto the hard stop (STUDY / head report z_thrust)
    h_lever = TOOLS["M300"]["Hn"] + ZOFF
    lever = 1 + h_lever / (1.5 * R)
    db = 0.10 / lever
    Q = (Pmin + P.DESIGN_LOAD_N) / 6 / math.cos(math.radians(45))
    delta = (9 * Q * Q / (16 * rb * Es * Es)) ** (1 / 3)
    k_c = 1.5 * Q / delta
    k_theta = 3 * k_c * 1.5 * R * R / 3
    th = (P.DESIGN_LOAD_N / 2 * 31.5) / k_theta
    return {"geometry": {"ball_circle_R_mm": R, "orientation": "two balls on an edge along Y (the nozzle row), one ball at +X",
                         "balls": BALLS, "ball_centre_below_vee_contacts_mm": round(ZOFF, 2),
                         "stud": f"bought retention knob at x +{STUD['x']:g} mm from the axis (the plunger pusher needs the axis): inside R/2 = "
                                 f"{R / 2:g}: {'all three balls stay loaded' if inside else 'a ball unloads'}"},
            "components (V18: bought, nothing bonded, pressed or reamed)": {
                "balls": "3 x tooling ball D10 with threaded shank (hardened, grade 25 class), screwed into tapped holes in the tool plate until the "
                         "ball's shoulder seats on a spot face: the shoulder sets the ball height (catalogue class, UNVERIFIED part)",
                "seats": "3 x hardened 90 deg vee seat (%g x %g x %g, kinematic-coupling component class, UNVERIFIED part) in shallow pockets on "
                         "the bracket underside, groove pointing at the ball-triangle centre, 2 x M3 each" % (VEE["L"], VEE["W"], VEE["H"]),
                "stud": "bought retention knob (machine-tool pull stud class, 45 deg cone under a D16 head, M10 shank thread; UNVERIFIED part) "
                        "screwed to its shoulder in the tool plate",
                "why_no_precision_placement": "a kinematic coupling repeats to the quality of its contacts, not of their placement: each vee only "
                                              "has to point roughly at the centre (+/-1 deg) and sit within +/-0.2 mm; the absolute offset of each "
                                              "tool is measured once at commissioning (head camera / tip gate) and kept in the tool table"},
            "loads_and_holding": rows, "latch_preload_min_N": Pmin, "latch_preload_max_N": Pmax,
            "hertz_working": {"load_N": round(Pmax + P.DESIGN_LOAD_N), "Q_per_contact_N": round(h_work[0], 1),
                              "contact_radius_mm": round(h_work[1], 3), "p0_GPa": round(h_work[2] / 1000, 2),
                              "basis": "ball on a flat (conservative for ball on a vee flank), largest preload + 150 N"},
            "hertz_crash_949N": {"Q_per_contact_N": round(h_crash[0], 1), "p0_GPa": round(h_crash[2] / 1000, 2),
                                 "limit": "about 4 GPa static for through-hardened bearing steel (ISO 76 basis, ASSUMED): crash uses "
                                          f"{h_crash[2] / 4000 * 100:.0f} %: inspect the seats after any crash (no brinell marks)"},
            "repeatability": {"needed_at_nozzle_mm": 0.10, "lever_nozzle_per_seat": round(lever, 2), "seat_repeatability_needed_mm": round(db, 4),
                              "basis": "0.10 mm at the nozzle end, 3 sigma: the exchange study's tip-entry budget (2.4) and ADR-010 gate CT-1; "
                                       "h = tool-plate to nozzle end of the eight-channel tool; a ball-and-vee coupling of hardened parts is "
                                       "expected to repeat to a few micrometres (literature class, UNVERIFIED): CT-1 measures it"},
            "latch": lt,
            "stiffness": {"per_contact_N_per_mm": round(k_c), "rotational_N_mm_per_rad": round(k_theta),
                          "uneven_pickup_tilt_mrad": round(th * 1000, 3), "nozzle_shift_mm": round(th * h_lever, 4)},
            "contact_seat_rule": "balls and vee seats bought hardened: no printed, aluminium or plastic seats (study 2.3)"}


# =====================================================================================================
# 3. stiffness budget M1 with the new head (rows of rev F/K scaled; each factor named)
# =====================================================================================================
def beam_length():
    """K2 X beam (machine_l frame): the Y rails sit 8.5 mm + half an MGN15H block outside the head's swept X range; the beam ends over
    the outer block edges + 4.5 mm (rev B rule)"""
    bb = cq.Compound.makeCompound(list(head_fixed().values()) + list(z_axis_moving(P.L_AXIS_X["gripper"], 0.0, gripper=True).values())).BoundingBox()
    xl = P.L_XC_RANGE[0] + min(bb.xmin, -160.0) - 8.5 - 17.0
    xr = P.L_XC_RANGE[1] + max(bb.xmax, 141.2) + 8.5 + 17.0
    return (xr - xl) + 2 * (17.0 + 4.5)


def m1_budget(F=150.0):
    rk = json.loads((LH1 / "assembly" / "out" / "report.json").read_text())["rev_f"]["M1_deflection_budget"]
    beamL = beam_length() / 980.0
    J0 = 4 * (76 * 36) ** 2 * 4 / 224.0                    # Bredt, 80 x 40 x 4 (mid-line)
    J1 = 4 * (95 * 45) ** 2 * 5 / 280.0                    # 100 x 50 x 5
    I0 = (40 * 80 ** 3 - 32 * 72 ** 3) / 12.0
    I1 = (50 * 100 ** 3 - 40 * 90 ** 3) / 12.0
    yc = (15.0 / P.L_YC_T) ** 3
    cc = coupling_calcs()
    bracket_I = (BR["x"][1] - BR["x"][0]) * BR["t"] ** 3 / 12.0
    L_arm = abs(BR["y"][0] - BR["web"][0]) / 2              # load acts about half-way along the arm (nozzle under the triangle)
    th_br = F * (abs(AXY[1] - BR["web"][0])) ** 2 / (2 * 70e3 * bracket_I)
    h_noz = TOOLS["M300"]["Hn"] + ZOFF
    scale = {"X beam torsion": (beamL * J0 / J1, f"beam 980 -> {beam_length():.0f} mm (x{beamL:.2f}); 100x50x5 J {J1:.0f} vs {J0:.0f} mm4"),
             "beam-to-Y-carriage joints": (1.0, "unchanged (rev K value)"),
             "Y blocks (pitch)": (1.0, "unchanged"),
             "Y carriage plates": (yc, f"YC plates 15 -> {P.L_YC_T:g} mm (rotation ~ 1/t^3)"),
             "X blocks (radial, 4 x MGN12H)": (1 / 1.5, "MGN15H: radial rigidity about 1.5 x MGN12H (ASSUMED from catalogue class)"),
             "X beam bending": (beamL ** 3 * I0 / I1, f"L^3 x{beamL ** 3:.2f}; I {I1:.0f} vs {I0:.0f} mm4"),
             "side (Y) beam bending": (1.0, "K2 side beams with intermediate uprights (rev H method)"),
             "roll about Y (tool x offset)": (1.0, "unchanged"),
             "carriage plate CP-1 + 2040 stiffeners": (0.5, "CP-L 10 mm (was 6) + the same stiffeners: about half (ASSUMED; plate term "
                                                            "falls with t^3 but the stiffeners carry most)"),
             "Z blocks pitch (2 x MGN12H, 46.4 mm apart)": ((46.4 / BLK["pitch"]) ** 2 / 1.5, f"MGN15H blocks {BLK['pitch']:g} mm apart"),
             "Z blocks roll (nozzle 5 mm off the rail axis)": (0.4, "nozzle on the rail plane within 2 mm (was 5)"),
             "pipette in cradle, uneven 8-tip seating": (1.0, "unchanged (inside the tool module)"),
             "lead screw + thrust bearing + nut (axial)": (1.0, "unchanged; vertical, taken up by the force-ended pick-up")}
    # pass 2: lever corrections for the raised gantry (rev F rows were taken at beam mid z 400 with the nozzle at z 69, pick-up at a rack):
    # rotations at the beam / Y carriage / X blocks move the nozzle sideways in proportion to the beam-to-nozzle lever; the carriage
    # plate is a cantilever from the X blocks to the Z blocks, which sit lower on it by the same amount (rev F: 150 mm, rotation x 181)
    lev = (ZB - 69.0) / 331.0
    lev_cp = (ZB - 69.0 - 181.0) / 150.0
    rot_rows = ("X beam torsion", "beam-to-Y-carriage joints", "Y blocks (pitch)", "Y carriage plates", "X blocks (radial, 4 x MGN12H)")
    beamL = beam_length() / 980.0
    rows, H, Vv = [], 0.0, 0.0
    for r in rk["rows"]:
        c = r["contributor"]
        if c.startswith("tool plate TP-1"):
            continue
        f, why = scale.get(c, (1.0, "unchanged"))
        fh = f * (lev if c in rot_rows else lev_cp if c.startswith("carriage plate") else 1.0)
        fv = f * (lev_cp if c.startswith("carriage plate") else 1.0)
        if c in rot_rows or c.startswith("carriage plate"):
            why += f"; lever x{fh / f:.2f} (beam mid z {ZB:g})"
        h, v = r["horizontal_mm"] * fh * F / 150.0, r["vertical_mm"] * fv * F / 150.0
        f = fh
        rows.append({"contributor": c, "horizontal_mm": round(h, 4), "vertical_mm": round(v, 4), "factor": round(f, 3), "why": why})
        H += h
        Vv += v
    h_br = th_br * h_noz
    rows.append({"contributor": "coupling bracket CBR-L (20 mm Al 6061, cantilever from the web to the nozzle line)",
                 "horizontal_mm": round(h_br, 4), "vertical_mm": round(th_br * abs(AXY[1] - BR["web"][0]), 4), "factor": None,
                 "why": f"theta = F L^2 / 2EI, L {abs(AXY[1] - BR['web'][0]):.1f}, I {bracket_I:.0f} mm4, lever {h_noz:.0f} mm"})
    H += h_br
    Vv += th_br * abs(AXY[1] - BR["web"][0])
    cpl = cc["stiffness"]["nozzle_shift_mm"] * F / 150.0 + 0.002 * F / 150.0
    rows.append({"contributor": "three-ball coupling (contacts, uneven pick-up) + axial", "horizontal_mm": round(cpl, 4), "vertical_mm": 0.002,
                 "factor": None, "why": "coupling_calcs stiffness"})
    H += cpl
    Vv += 0.002
    v_no_axial = Vv - next(r["vertical_mm"] for r in rows if r["contributor"].startswith("lead screw"))
    lim = rk["limit_mm"]
    return {"load_N": F, "rows": rows, "sum_horizontal_mm": round(H, 3), "sum_vertical_mm": round(Vv, 3),
            "total_mm": round(math.hypot(H, Vv), 3), "total_without_axial_screw_mm": round(math.hypot(H, v_no_axial), 3),
            "limit_mm": lim, "pass_rule_mm (limit/1.25)": round(lim / 1.25, 3),
            "horizontal_pass": H <= lim / 1.25, "total_pass_rule": math.hypot(H, Vv) <= lim / 1.25,
            "total_without_axial_pass_rule": math.hypot(H, v_no_axial) <= lim / 1.25,
            "note": "the axial screw term is vertical and is taken up by the force-ended pick-up (Z drives until the cradle force "
                    "switch trips), so it does not move the tip laterally: the lateral (horizontal) sum is what the tip-entry "
                    "budget sees"}


# =====================================================================================================
# 4. dynamics: belt modes on the K2 frame with this head; Z clearance; travel
# =====================================================================================================
def dynamics(m_head):
    rf = json.loads((LH1 / "assembly" / "out" / "report.json").read_text())["rev_f"]["M9_belts_and_settling"]
    m0x, m0y = rf["masses_kg"]["x"], rf["masses_kg"]["y"]
    bl = beam_length()
    beam_extra = (1400 * bl - 896 * 980) * 2.70e-6            # kg: 100x50x5 at the K2 length vs 80x40x4 at 980
    m_y = m0y - m0x + m_head + beam_extra + 0.4                # + MGN15 X rails/blocks vs MGN12 (ASSUMED +0.4 kg)
    span_ratio = bl / 980.0                                    # worst X belt span grows with the beam (rev K method)
    y_rail = (P.L_YF_RANGE[1] - P.L_YF_RANGE[0]) + 85.0 + 58.8 + 10.0
    loop_ratio = (2 * y_rail + 44.0) / 1364.0                  # Y belt loop (rev F 1364 mm for a 660 mm rail)
    zeta, dwell = 0.03, rf["dwell_before_slot_entry_or_tip_pick_s"]
    out = {"head_mass_kg": round(m_head, 2), "Y_moving_mass_kg": round(m_y, 2), "dwell_s": dwell}
    for ea, key in ((3e4, "X_EA3e+04_x+295"), (6e4, "X_EA6e+04_x+295"), (1.0e5, None)):
        base = rf["modes"]["X_EA6e+04_x+295"] if key is None else rf["modes"][key]
        kfac = (ea / 6e4) if key is None else 1.0
        k = base["k_N_per_mm"] * kfac / span_ratio
        f = math.sqrt(k * 1000 / m_head) / (2 * math.pi)
        amp = m_head * 2.0 / (k * 1000) * 1000                 # mm stretch at 2 m/s2
        t = math.log(amp / 0.05) / (zeta * 2 * math.pi * f) if amp > 0.05 else 0.0
        out[f"X_EA{ea:.0e}"] = {"f_Hz": round(f, 1), "stretch_mm": round(amp, 3), "settle_s_to_0.05mm": round(t, 2), "within_dwell": t <= dwell}
    for ea, key in ((3e4, "Y_EA3e+04"), (6e4, "Y_EA6e+04")):
        base = rf["modes"][key]
        k = base["k_N_per_mm"] / loop_ratio
        f = math.sqrt(k * 1000 / m_y) / (2 * math.pi)
        amp = m_y * 2.0 / (k * 1000) * 1000
        t = math.log(amp / 0.05) / (zeta * 2 * math.pi * f)
        out[f"Y_EA{ea:.0e}"] = {"f_Hz": round(f, 1), "stretch_mm": round(amp, 3), "settle_s_to_0.05mm": round(t, 2), "within_dwell": t <= dwell}
    out["X_belt_specification"] = ("EA >= 6e4 N per belt run (a 15 mm GT2 glass-fibre belt, or a 9 mm steel-cord belt; the present 9 mm "
                                   "glass-fibre belt is the EA 3e4 class): CALCULATED requirement; measure the belt on the rig")
    out["basis"] = ("rev F/K belt model (damping 0.03, residual = static stretch at 2 m/s2); X span x%.2f (beam %.0f mm) and Y loop 1364 -> "
                    "%.0f mm for the K2 frame (Y rail %.0f mm); X beam 100x50x5 adds %.2f kg to the Y mass" % (span_ratio, bl, 2 * y_rail + 44.0, y_rail, beam_extra))
    return out


def z_clearance(parts):
    low_fixed = min(v.BoundingBox().zmin for k, v in parts.items() if k.startswith("fixed")) + ZB
    low_all = min(v.BoundingBox().zmin for k, v in parts.items()) + ZB
    tips = {}
    for k, t in TOOLS.items():
        if t["tip"]:
            nz = tpl_top(0) - t["Hn"] + ZB
            tips[k] = {"nozzle_z_at_top": round(nz, 1), "tip_end_z_at_top": round(nz - TIP_L[t["tip"]], 1)}
    grip = min(v.BoundingBox().zmin for k, v in parts.items() if k.startswith("gripper")) + ZB
    lowest_tip = min(v["tip_end_z_at_top"] for v in tips.values())
    req = 177.0
    lo_blk = z_top(P.L_Z_TRAVEL) - 30.0 - BLK["pitch"] - BLK["l"] / 2
    hi_blk = z_top(-P.Z_EJECT_TRAVEL) - 30.0 + BLK["l"] / 2
    blocks = {"lower_block_bottom_at_full_stroke": round(lo_blk, 1), "upper_block_top_at_eject": round(hi_blk, 1), "rail": list(RAIL["z"]),
              "on_rail": lo_blk >= RAIL["z"][0] and hi_blk <= RAIL["z"][1]}
    return {"requirement_z": req, "z_blocks_on_rail": blocks, "head_fixed_lowest_z": round(low_fixed, 1), "lowest_any_part_no_tips_z": round(low_all, 1),
            "gripper_lowest_z": round(grip, 1), "tools": tips, "lowest_tip_end_z": lowest_tip,
            "gripper_fingertip_z_at_full_stroke": round(grip - P.L_Z_TRAVEL, 1),
            "pass": min(low_fixed, low_all, grip, lowest_tip) >= req,
            "reach_down": {k: {"nozzle_z_at_full_stroke": round(v["nozzle_z_at_top"] - P.L_Z_TRAVEL, 1),
                               "tip_end_z_at_full_stroke": round(v["tip_end_z_at_top"] - P.L_Z_TRAVEL, 1)} for k, v in tips.items()},
            "note": f"X beam mid raised to z {ZB:g} (K2 frame +30 mm), Z travel {P.L_Z_TRAVEL:g}"}


def reach_into_tubes():
    """tip end at full stroke against tube bottoms on the K2 racks (rims from rev K deck_k K2: recessed racks)."""
    seat = P.LABWARE_SEAT_Z
    racks = {   # tube class: (rim z ASSUMED from the K2 rack heights, tube length, inner diameter at the top)
        "1.5mL (CTB-1 F3, ATR-2 A2)": (seat + 45.0, 39.0, 8.7), "5mL (CTB-2 F4, ATR-2 A2)": (seat + 55.0, 57.0, 15.5),
        "15mL (CTB-2 F4, ATR-1 F1; recessed 60)": (seat + 100.0 - 0.0, 120.0, 14.5), "50mL (CTB-2 F4, ATR-1 F1; recessed 60)": (seat + 100.0, 115.0, 27.5),
        "PCR plate well": (seat + 16.1, 15.1, 5.4), "deep-well 2 mL": (seat + 44.0, 40.0, 8.0)}
    out = {}
    for k, t in TOOLS.items():
        if not t["tip"]:
            continue
        nz_low = tpl_top(0) - t["Hn"] + ZB - P.L_Z_TRAVEL
        tip_low = nz_low - TIP_L[t["tip"]]
        row = {}
        prof = SHAFT.get(k)
        for rk_, (rim, L, d) in racks.items():
            bottom = rim - L
            # body entry: how far above the nozzle end the shaft first gets wider than the bore (linear between profile points)
            entry = 0.0
            if prof:
                entry = prof[-1][0]
                for (h0, d0), (h1, d1) in zip(prof, prof[1:]):
                    if d1 > d - 0.5:
                        entry = h0 + (max(d - 0.5, d0) - d0) / (d1 - d0) * (h1 - h0) if d0 <= d - 0.5 else h0
                        break
            body_floor = rim - entry
            deepest = max(tip_low, bottom + 0.5, body_floor - TIP_L[t["tip"]] if prof else rim - TIP_L[t["tip"]] if d < 12.0 else -1e9)
            if t["eight"] and not rk_.startswith(("PCR", "deep")):
                row[rk_] = "not used (eight-channel into tubes: tube pitch is not 9 mm)"
                continue
            row[rk_] = {"tip_end_lowest_z": round(deepest, 1), "tube_bottom_z": round(bottom, 1),
                        "bottom_reached": deepest <= bottom + 0.5 + 1e-6, "short_of_bottom_mm": round(max(0.0, deepest - bottom - 0.5), 1)}
        out[k] = {"tip": t["tip"], "nozzle_lowest_z": round(nz_low, 1), "tip_end_lowest_z": round(tip_low, 1), "racks": row}
    return {"basis": "K2 racks with the rev K recess; tube geometry from deck_k TUBES (ASSUMED); single-channel shafts enter a bore until their "
                     "OD (SHAFT profiles, ASSUMED) exceeds the bore - 0.5 mm; minimum usable volume = tube volume below (short + 3 mm)",
            "tools": out}


# =====================================================================================================
# 5. exchange motion and time (study 4.2 recomputed from assumed speeds)
# =====================================================================================================
def exchange_time():
    v_xy, a_xy = 300.0, 2000.0          # mm/s, mm/s2 (ASSUMED rule values, as the rev F dynamics)
    v_z_fast, v_z_slow = 20.0, 5.0      # mm/s: Tr8x2 at 600 rpm; slow for the last 10 mm onto the bay and for latching (ASSUMED)
    dwell = 1.04

    def move(d):
        da = v_xy ** 2 / a_xy
        return 2 * math.sqrt(d / a_xy) if d < da else d / v_xy + v_xy / a_xy
    d_to_dock = 450.0                   # mm, a typical deck place to the dock row (K2, ASSUMED)
    sd = P.L_DOCK_S                     # pass 2: the bays sit at the bottom of the stroke (dock_clearance in machine_l)
    lift = IN_ROW_LIFT                  # empty carriage lifts this far to pass over the parked tools between bays (inrow_clearance)
    row = 300.0                         # dock row entered at its right end and travelled along X (machine_l dock rule), mean run
    steps = [("XY to the dock row's right end (Z up, no tips), then along the row to the empty bay", move(d_to_dock) + move(row)), ("settle", dwell),
             (f"Z down {sd - 15:.0f} mm fast + 15 slow (lugs onto the bay pins)", (sd - 15) / v_z_fast + 15 / v_z_slow),
             ("dock lift bar up (opens the latch) at 5 mm/s", LT.lift_stroke()["needed_mm"] / 5.0 - 2.6),
             ("Z and bar up 13 mm together in 1 mm steps (stud head leaves the jaw)", 13 / v_z_slow * 2),
             ("bar down to home", LT.lift_stroke()["needed_mm"] / 10.0),
             (f"Z up {lift - 13:.0f} mm more to the in-row stroke s {P.L_DOCK_S - lift:.0f} (tool stays)", (lift - 13) / v_z_fast),
             ("XY to the next bay (one 78 mm pitch to 7 pitches, mean 3)", move(3 * P.L_DOCK_PITCH)), ("settle", dwell),
             (f"Z down {lift - 13:.0f} mm (stud head just below the jaw)", (lift - 28) / v_z_fast + 15 / v_z_slow),
             ("bar up (latch open), then Z and bar down 13 mm together in 1 mm steps (balls seat)", LT.lift_stroke()["needed_mm"] / 5.0 + 13 / v_z_slow * 2),
             ("bar down: the springs close the toggle and preload the coupling", LT.lift_stroke()["needed_mm"] / 5.0),
             ("verify: code, latch, bay switch", 0.5),
             (f"Z up {sd:.0f} mm", sd / v_z_fast), ("X back along the row to its end, then XY back", move(row) + move(d_to_dock)), ("settle", dwell)]
    tot = sum(t for _, t in steps)
    return {"assumed": {"v_xy_mm_s": v_xy, "a_xy_mm_s2": a_xy, "v_z_fast": v_z_fast, "v_z_slow": v_z_slow, "dwell_s": dwell},
            "steps_s": {k: round(t, 2) for k, t in steps}, "total_s": round(tot, 1),
            "plus_first_pickup_nozzle_check_s": 10.0, "per_run_two_carriages": f"9-22 exchanges (study 4.1): {9 * tot / 60:.0f}-{22 * tot / 60:.0f} min; "
            f"30-36 with the full tool set (ADR-010): {30 * tot / 60:.0f}-{36 * tot / 60:.0f} min",
            "note": "V1: the dock lift adds about 40 s per exchange (bar strokes at 5 mm/s and the 13 mm follow steps).  pass 2: the dock seat at the bottom of the stroke costs about 2 x 330 mm of Z per exchange at the Tr8x2 speed: "
                    "the Z travel dominates; a Tr8x4 screw would halve it if the pick-up thrust allows (rig CT-9)"}


# =====================================================================================================
# 6. dock bay (machine; the rig builds two bays of the same design)
# =====================================================================================================
BAY = dict(pitch=P.L_DOCK_PITCH, h=300.0, pin_d=6.0, base_t=6.0, pad_hx=4.75, pad_y=(-173.0, -150.0), pad_h=30.0, bar_y=(-176.0, -163.0), bar_t=12.0)
# pass 2: the tool rests on two D6 pins in the cradle's dock lugs (round hole + slot, the standard interface of every cradle), not on
# its tool plate.  V1 (21 Nov 2026): the fixed release posts are gone; one dock lift DLB-1 (a bar under all bays with a hardened pad under
# each bay's latch roller, raised by one lead-screw stepper at the row's left end) opens the latch only when the firmware asks, with the
# tool already on the bay pins (drop-off) or the carriage empty (pick-up).  Bar DOWN: the pads stand 3 mm under a locked carriage's roller.


def dock_seat_top():
    """carriage-local tool-plate top of a parked tool (seated at stroke L_DOCK_S)"""
    return tpl_top(P.L_DOCK_S)


def bay_parts(x0=0.0, y0=0.0, z0=0.0, n=2, base_z=None):
    """Bays in carriage-local coordinates of a carriage axis at x = bay centre, coupling at its dock stroke: x0 first bay centre,
    y0 offset in y (0 = carriage-local y), z0 offset in z.  base_z: local z of the dock base top (default: tray under the nozzles)."""
    top = dock_seat_top()
    zt = z_top(P.L_DOCK_S)
    lug_b = top + CR["lug_dz"][0]
    noz_low = top - max(t["Hn"] for t in TOOLS.values())
    zb = (noz_low - 18.0) if base_z is None else base_z
    parts = {}
    parts["base"] = mv(B(x0 - 45.0, x0 + (n - 1) * BAY["pitch"] + 45.0, -170.0, -45.0, zb - BAY["base_t"], zb), 0, y0, z0)
    # dock lift DLB-1, bar DOWN: pad tops 3 mm under the locked roller of a carriage seated at L_DOCK_S
    zpad = LT.roller_yz(-LT.G["over_deg"], P.L_DOCK_S)[1] - LT.G["roller_d"] / 2 - 3.0
    zbar = zpad - BAY["pad_h"]
    xa, xb = x0 - 40.0, x0 + (n - 1) * BAY["pitch"] + 50.0
    by0, by1 = BAY["bar_y"]
    yc = (by0 + by1) / 2
    # actuator and guides at the row's LEFT end (the head enters the dock row at its right end, where nothing may stand)
    zb0 = zbar - BAY["bar_t"]                                   # bar underside at DOWN
    stroke = LT.DOCK_LIFT["stroke_mm"]
    gx = (xa - 56.0, xb + 8.0)                                 # right guide beyond the last bay's spring drums (stroke check)
    parts["lift_bar"] = mv(fuse_all([B(xa - 66.0, xb + 18.0, by0, by1, zb0, zbar)] +
                                    # LM8UU bushing blocks hung UNDER the bar at each guide (24 mm bushing in a 26 mm block)
                                    [B(g - 9.0, g + 9.0, yc - 9.0, yc + 9.0, zb0 - 26.0, zb0 + 0.01) for g in gx]).cut(
        fuse_all([cyl(g, yc, zb0 - 27.0, zbar + 1.0, 8.2) for g in gx])), 0, y0, z0)
    # integration 23 Nov 2026: shaft tops 2 mm above the bushing blocks at full stroke (were bar top + stroke + 15: the tallest dock item)
    parts["lift_guides"] = mv(fuse_all([cyl(g, yc, zb, zb0 + stroke + 2.0, 8.0) for g in gx]), 0, y0, z0)
    zm = zb + 60.0
    parts["lift_actuator"] = mv(fuse_all([B(xa - 46.3, xa - 4.0, yc - 21.15, yc + 21.15, zm - 48.0, zm),
                                          cyl(xa - 25.15, yc, zm, zb0 + 0.01, 8.0)]), 0, y0, z0)
    # DOWN: optical slot sensor (bought, EE-SX67x class, U body open at the top) behind the bar at the left end; flag F1 on the bar's back
    # face sits in its slot only at DOWN (it leaves after 10 mm)
    zh = zb0 - 4.0
    parts["lift_home_switch"] = mv(B(xa - 14.0, xa - 4.0, by1 + 2.0, by1 + 8.4, zh - 18.0, zh).cut(B(xa - 12.0, xa - 6.0, by1, by1 + 11.0, zh - 12.0, zh + 1.0)),
                                   0, y0, z0)
    # OPEN height (integration 23 Nov 2026, firmware request): sealed roller-lever switch beside the bar's path on a post at the left end;
    # flag F2 (18 mm tall, on the bar's back face) presses its roller from open_height_mm of bar stroke (latch.open_height_mm) to beyond
    # the full 50 mm stroke (a lever pressed sideways, so the over-travel is not used up by the 14 mm the bar still rises)
    hf = 15.0
    zl = zbar + hf + LT.open_height_mm()                       # roller line: the flag top reaches it at the open height
    parts["lift_open_switch"] = mv(fuse_all([B(xa - 44.0, xa - 24.0, by1 + 3.5, by1 + 9.9, zl - 5.0, zl + 5.2),
                                             B(xa - 48.0, xa - 44.0 + 0.01, by1 + 3.5, by1 + 12.0, zb + 60.0, zl + 5.2)]), 0, y0, z0)   # switch + post on the motor's top
    parts["lift_flags"] = mv(fuse_all([B(xa - 11.0, xa - 7.0, by1 - 0.01, by1 + 10.0, zh - 10.0, zbar),          # F1 DOWN flag
                                       B(xa - 40.0, xa - 28.0, by1 - 0.01, by1 + 3.0, zbar - 3.0, zbar + hf)]), 0, y0, z0)   # F2 OPEN flag
    for i in range(n):
        cx = x0 + BAY["pitch"] * i
        sad = [B(cx + sx * CR["lug_x"] - 3.0, cx + sx * CR["lug_x"] + 3.0, -69.0, -55.0, zb, lug_b - 0.3) for sx in (-1, 1)]   # clear of the manifolds
        parts[f"bay{i}_saddle"] = mv(fuse_all(sad), 0, y0, z0)
        parts[f"bay{i}_pins"] = mv(fuse_all([cyl(cx + sx * CR["lug_x"], -63.0, lug_b - 0.3, lug_b + 5.0, BAY["pin_d"] - 0.02) for sx in (-1, 1)]), 0, y0, z0)
        px0, px1 = cx + STUD["x"] + TOGGLE["pad_x"][0], cx + STUD["x"] + TOGGLE["pad_x"][1]     # under the outboard roller
        parts[f"bay{i}_lift_pad"] = mv(B(px0, px1, BAY["pad_y"][0], BAY["pad_y"][1], zpad - BAY["pad_h"], zpad), 0, y0, z0)
        parts[f"bay{i}_switch"] = mv(B(cx + 25.5, cx + 32.0, -66.0, -58.0, lug_b - 20.0, lug_b - 1.0), 0, y0, z0)
        parts[f"bay{i}_drip_tray"] = mv(B(cx - 34.0, cx + 34.0, -140.0, -70.5, zb, zb + 15.0).cut(
            B(cx - 32.0, cx + 32.0, -138.0, -72.5, zb + 2.0, zb + 16.0)), 0, y0, z0)
    return parts


# =====================================================================================================
# 6b. latch through its stroke (integration 23 Nov 2026): open position modelled, jaw / toggle / lift bar checked for interference
# =====================================================================================================
LATCH_JOINTS = [{"crank", "pivot_pin"}, {"arm", "pivot_pin"}, {"crank", "arm"}, {"link", "crank"}, {"link", "cartridge"}, {"cartridge", "jaw"},
                {"arm", "roller"}, {"pivot_pin", "spring_anchor"}, {"cf_spring_drums", "spring_anchor"}, {"spring_anchor", "bracket"},
                {"balls", "vees"}, {"balls", "tool_plate"}, {"vees", "bracket"}, {"stud", "tool_plate"}, {"magnets", "tool_plate"},
                {"eject_pin", "tool_plate"}, {"push_rod", "tool_plate"}, {"push_rod", "bracket"}, {"zc", "bracket"},
                {"pusher", "pusher_bracket"}, {"pusher_bracket", "bracket"}, {"pusher_collar", "pusher_rod"}, {"pusher_collar", "pusher"},
                {"pusher_rod", "pusher"}, {"pusher_retracted_switch", "pusher_bracket"}, {"latch_switch", "bracket"}, {"pushrod_switch", "push_rod"},
                {"hall_board", "bracket"}, {"stud", "bracket"}, {"pusher_collar", "bracket"}, {"pusher_rod", "bracket"}, {"pusher_rod", "pusher_bracket"},
                {"block0", "zc"}, {"block1", "zc"}, {"nut", "zc"}, {"lift_bar", "lift_flags"}, {"lift_bar", "lift_actuator"}, {"lift_bar", "lift_guides"},
                {"lift_bar", "lift_pad"}, {"cradle", "tool_plate"}, {"pipette", "cradle"}, {"pipette", "nozzles"}, {"pipette", "tool_plate"},
                {"cradle", "straps"}, {"cradle", "cheek_fixed"}, {"cradle", "cheek_loose"}, {"pipette", "cheek_fixed"}, {"pipette", "cheek_loose"},
                {"pipette", "straps"}, {"cradle", "slider"}, {"tool_plate", "slider"}, {"cradle", "eject_plunger"}, {"tool_plate", "eject_plunger"},
                {"eject_pin", "eject_plunger"}, {"cradle", "push_rod"}, {"force_collar", "push_rod"}, {"force_collar", "pipette"}, {"pipette", "slider"},
                {"pipette", "eject_plunger"}, {"cradle", "force_collar"}, {"lugs", "pins"}, {"cradle", "pins"}]


def _strict_hits(parts, label, allowed, tol=1.0):
    names = list(parts)
    bbs = {k: v.BoundingBox() for k, v in parts.items()}
    hits = []
    for i, a in enumerate(names):
        for b in names[i + 1:]:
            ba, bb = bbs[a], bbs[b]
            if ba.xmax < bb.xmin or bb.xmax < ba.xmin or ba.ymax < bb.ymin or bb.ymax < ba.ymin or ba.zmax < bb.zmin or bb.zmax < ba.zmin:
                continue
            la, lb = a.split(":")[-1], b.split(":")[-1]
            la = "lift_pad" if la.endswith("lift_pad") else ("pins" if la.endswith("_pins") else la)
            lb = "lift_pad" if lb.endswith("lift_pad") else ("pins" if lb.endswith("_pins") else lb)
            if {la, lb} in allowed:
                continue
            v = parts[a].intersect(parts[b]).Volume()
            if v > tol:
                hits.append({"config": label, "a": a, "b": b, "mm3": round(v, 2)})
    return hits


def latch_stroke_check(tool="M300"):
    """the carriage seated in a bay at L_DOCK_S with a tool on the bay pins; the dock lift bar raised so its pad carries the roller
    (3 mm gap first), the latch at its kinematic angle for that bar height; every part of the carriage, the tool and the bay against
    every other, with only the pin joints and seats of the design allowed to touch.  Then the 13 mm follow (drop-off: carriage and bar
    up together, jaw held open, the stud head leaving the keyhole; pick-up: the reverse) at full open.  Plus the two sensing switches
    against the flag through the whole stroke (made / released as latch.SENSE says)."""
    sd = P.L_DOCK_S
    rz0 = LT.roller_yz(-LT.G["over_deg"], sd)[1]
    dock0 = bay_parts(0.0, 0.0, 0.0, 1)
    moving = ("lift_bar", "lift_flags", "bay0_lift_pad")
    allowed = LATCH_JOINTS + [{"roller", "lift_pad"}, {"stud", "jaw"}, {"latch_switch", "jaw"}]
    rows, hits = [], []
    phis = [-LT.G["over_deg"], 0.0, 4.0, 10.0, 20.0, 30.0, 45.0, 60.0, 75.0, 90.0, 105.0, LT.G["open_deg"]]

    def scene(phi, s_car, bar_dz):
        parts = {f"carriage:{k}": v for k, v in z_axis_moving(0.0, s_car, phi=phi).items()}
        parts.update({f"tool:{k}": v for k, v in tool_module(0.0, sd, tool).items()})
        for k, v in dock0.items():
            if k in ("base",) or k.endswith("drip_tray"):
                continue
            parts[f"dock:{k}"] = v.translate(V(0, 0, bar_dz)) if k in moving else v
        return parts
    for ph in phis:
        dz = 3.0 + (LT.roller_yz(ph, sd)[1] - rz0)                    # pad top under the roller bottom
        al = allowed if ph <= 0.0 else [a for a in allowed if a != {"stud", "jaw"}]
        h = _strict_hits(scene(ph, sd, dz), f"phi {ph:+.0f} deg, bar +{dz:.1f} mm", al)
        sens = LT.jaw_disp(ph)
        rows.append({"phi_deg": ph, "bar_raised_mm": round(dz, 1), "jaw_open_mm": round(sens, 2),
                     "LOCK": sens <= LT.SENSE["lock_op_mm"], "EMPTY": sens <= LT.SENSE["empty_op_mm"], "hits": len(h)})
        hits += h
    dz_open = 3.0 + (LT.roller_yz(LT.G["open_deg"], sd)[1] - rz0)
    follow = []
    for k in range(0, int(LT.DOCK_LIFT["interleave_mm"]) + 1, 2):
        lab = f"follow: carriage {k} mm above the seat, bar with it, jaw open"
        h = _strict_hits(scene(LT.G["open_deg"], sd - k, dz_open + k), lab, [a for a in allowed if a != {"stud", "jaw"}])
        follow.append({"carriage_up_mm": k, "hits": len(h)})
        hits += h
    # stud head against the open keyhole: radial clearance (model: D17 hole, D16 head)
    return {"what": "latch LTG-1 from locked (phi -4) to full open (phi %g) with the dock lift bar under the roller, carriage seated at s %g "
                    "with an %s on the bay pins; then the 13 mm follow at full open" % (LT.G["open_deg"], sd, tool),
            "stroke": rows, "follow": follow, "hits": hits, "count": len(hits), "pass": not hits,
            "allowed_contacts": "pin joints (crank/arm/pivot, link/crank, link/cartridge, cartridge/jaw, arm/roller), the roller on its pad, seats "
                                "(balls/vees, stud/tool plate), the stud cone in the jaw only while locked, the switch plungers on the flag",
            "keyhole_radial_clearance_mm": round((STUD["head_d"] + 1.0 - STUD["head_d"]) / 2.0, 2),
            "open_position_step": "out/LTG-L_latch_open_assembly.step (jaw, flag, cartridge, link, crank, arm, roller at phi %g, bar raised %.1f mm)" % (LT.G["open_deg"], dz_open)}


IN_ROW_LIFT = 40.0          # mm above the dock seat stroke for the empty carriage's in-row move (inrow_clearance; was 35)
DOCK_RULE_MM = 8.0          # machine_l pass 2 dock rule: >= 8 mm over parked tools and the dock parts while moving along the row


def inrow_clearance(lift=None, step=6.0):
    """the in-row move of an exchange: the EMPTY carriage (just parked its tool in bay 1) moves along the row to bay 2 at stroke
    L_DOCK_S - lift over the parked tools (M300 in bay 1, S1000 in bay 2) and the bar DOWN.  Vertical gap between every carriage part and every
    parked-tool / dock part whose plan boxes overlap, at every step; and the exact solid intersections.  The other carriage and the
    gripper are at s 0 (machine dock_clearance covers them)."""
    lift = IN_ROW_LIFT if lift is None else lift
    s_in = P.L_DOCK_S - lift
    dock = bay_parts(0.0, 0.0, 0.0, 2)
    stat = {f"dock:{k}": v for k, v in dock.items() if k not in ("base",)}
    for i in range(2):
        stat.update({f"bay{i}_tool:{k}": v for k, v in tool_module(i * BAY["pitch"], P.L_DOCK_S, ("M300", "S1000")[i]).items()})
    sb = {k: v.BoundingBox() for k, v in stat.items()}
    worst, side, hits = (1e9, None), (1e9, None), []
    n = int(BAY["pitch"] / step) + 1
    for j in range(n):
        x = min(j * step, BAY["pitch"])
        car = z_axis_moving(x, s_in)
        zt = z_top(s_in)
        car.pop("bracket")                                          # split into its arm and its web (solid boxes: conservative)
        car["bracket_arm"] = B(x + BR["x"][0], x + BR["x"][1], BR["y"][0], BR["y"][1], zt, zt + BR["t"])
        car["bracket_web"] = B(x + BR["x"][0], x + BR["x"][1], BR["web"][0], BR["web"][1], zt - BR["web_h"], zt)
        for ck, cv in car.items():
            cb = cv.BoundingBox()
            for sk, b in sb.items():
                if (cb.xmax + 20 < b.xmin or b.xmax + 20 < cb.xmin or cb.ymax + 20 < b.ymin or b.ymax + 20 < cb.ymin
                        or cb.zmax + 20 < b.zmin or b.zmax + 20 < cb.zmin):
                    continue
                d = cv.distance(stat[sk])                           # exact minimum distance (BRepExtrema)
                d_up = cv.translate(V(0, 0, 5.0)).distance(stat[sk])
                if d_up > d + 2.5:                                  # the gap grows as the carriage rises: it passes OVER the part
                    if d < worst[0]:
                        worst = (d, f"carriage {ck} over {sk} at x {x:.0f}")
                elif d < side[0]:                                   # beside it (the gap does not depend on the stroke)
                    side = (d, f"carriage {ck} beside {sk} at x {x:.0f}")
                if d < 1e-6 and cv.intersect(stat[sk]).Volume() > 1.0:
                    hits.append({"x": x, "a": ck, "b": sk})
    return {"in_row_stroke_s": s_in, "lift_above_seat_mm": lift, "min_clearance_over_mm": round(worst[0], 1), "where": worst[1], "rule_mm": DOCK_RULE_MM,
            "min_side_gap_mm (no Y motion during the move)": round(side[0], 1), "side_where": side[1],
            "pass": worst[0] >= DOCK_RULE_MM and not hits, "intersections": hits,
            "note": "exact minimum distance between solids (pairs within 20 mm by bounding box); the firmware's in-row stroke is "
                    "_LH_GEOM dock_s - park_lift_mm"}


# =====================================================================================================
# 7. safety functions and firmware inputs
# =====================================================================================================
def safety():
    per_carriage = ["Z home (top) switch", "Z-up switch (tools-up chain)", "latch LOCK + EMPTY switches on one resistor-ladder way: LOCKED / CLOSED-EMPTY / NOT CLOSED (latch.SENSE)",
                    "4 Hall switches (tool code; 0000 = no tool)", "cradle force switch via the push-rod (NC)", "pusher contact switch",
                    "plunger pusher motor (Nanotec LGA35 captive with guide, V6)", "Z motor"]
    return {"inputs_per_tool_carriage (x2)": per_carriage,
            "gripper_axis": ["Z home", "Z-up switch", "jaw switch", "gripper motor", "Z motor"],
            "dock": ["8 bay-occupied switches", "dock lift motor (NEMA17 48 + Tr8x2, one per dock row, LEFT end)", "dock lift home sensor (bar DOWN, slot sensor)",
                     "dock lift OPEN-height switch (bar at or above %.1f mm, NO)" % LT.open_height_mm()],
            "count": {"digital_inputs_new": 2 * 8 + 8 - 2 * 2, "motors": "3 Z + 2 pusher + gripper (was 4 Z + 3 plunger + gripper)"},
            "guards": [
                "TOOLS-UP chain: XY motion only with all three Z-up switches made (carriage A, B, gripper), as now with three axes",
                "TOOL PRESENT AND LOCKED: before any XY move each carriage reads either (code 0000 AND latch closed-empty) or (code == expected "
                "AND latch switch in window); any other combination = HOLD, Z stays where it is",
                "HALF-LOCK: code present + latch out of window = HOLD; never move XY; the run asks for a person",
                "DROPPED TOOL: code goes to 0000 or latch leaves the window while not in a bay = emergency stop of XY, Z hold",
                "WRONG TOOL: code != planner's expected tool for that bay/carriage = HOLD before leaving the bay",
                "BAY CHECK: after a drop-off the bay switch must close; after a pick-up it must open; otherwise HOLD",
                "POWER-UP: read both codes and latches before homing; a tool on a carriage is homed with Z up only (no Z-down homing), "
                "and its nozzle position is re-checked at the tip gate before any work",
                "TIP CHECK PER TOOL: after every pick-up the tip gate checks presence and VC-2 the tip length (tip type) for that tool",
                "EJECT: the eject over-travel is allowed only for a carriage whose code says a tool with tips is fitted; the fixed finger "
                "enters the bracket hole only above s = 0",
                "LATCH OPEN only inside a dock bay, by the dock lift: bar UP only with a carriage at a bay centre at the dock stroke and either the "
                "bay switch made (drop-off) or Hall code 0000 (pick-up); X/Y never move unless the bar home switch is made",
                "PUSHER: homes UP to its RETRACTED switch at <= 0.5 A and <= 2 mm/s; it must never be driven into the actuator's internal end "
                "stop above 0.57 A (maker's limit for this thread); soft limits 0.5 .. 37.5 mm"],
            "power_loss": "over-centre latch held by geometry (needs no power, no spring) and closed by two constant-force springs: a tool cannot drop; the dock lift is a lead screw and stays where it is; Z screws self-locking; pusher self-locking",
            "sequences": LT.SEQUENCES}


# =====================================================================================================
# 8. bench rig parts for CT-1..CT-9 (designed here; the rig module lists them under REV_L)
# =====================================================================================================
def rig_parts():
    """Rig-local coordinates: the rig Z carriage's tool plate face (TOOL_BOLTS pattern) at y 0, bolts at (+/-22, +/-52) in x/z."""
    p = {}
    # RCB-L: L-shaped adapter: vertical web on the rig TP-1 bolt pattern + the standard horizontal coupling arm on top
    web = B(-31.0, 31.0, -12.0, 0.0, -65.0, 65.0)
    arm = B(-27.0, 45.0, -117.0, -12.0, 65.0, 85.0)
    bolts = [DK.cylx if False else cq.Solid.makeCylinder(2.25, 14.0, V(x, -13.0, z), V(0, 1, 0)) for x, z in P.TOOL_BOLTS]
    p["RCB-L_rig_coupling_bracket"] = cut_all(fuse_all([web, arm]), bolts)
    # TPL-L tool plates (rig: two, for S10 and M300; same part as the machine)
    p["TPL-L_tool_plate"] = B(-27.0, 45.0, -105.0, 0.0, 0.0, 10.0)
    # CRA-L: cradle adapter: hangs the existing CB-1 cradle bodies (their TP-1 pattern) from TPL-L
    p["CRA-L_cradle_adapter"] = fuse_all([B(-31.0, 31.0, -14.0, -2.0, -140.0, 0.0), B(-27.0, 45.0, -60.0, 0.0, -2.0, 0.0 + 0.01)])
    # two-bay dock (same bay design as the machine), on a base plate that bolts to the rig base
    p["DB-L2_dock_base"] = B(0.0, 164.0, 0.0, 150.0, 0.0, 6.0)
    p["BC-L_bay_ledge"] = B(0.0, 60.0, 0.0, 8.0, 0.0, 300.0)
    p["RP-L_release_post"] = B(0.0, 12.0, 0.0, 8.0, 0.0, 346.0)
    p["DT-L_drip_tray"] = B(0.0, 68.0, 0.0, 90.0, 0.0, 15.0).cut(B(2.0, 66.0, 2.0, 88.0, 2.0, 16.0))
    # test fixtures
    p["NRP-L_nozzle_reference_pin_holder"] = cyl(0.0, 0.0, 0.0, 30.0, 12.0).cut(cyl(0.0, 0.0, 10.0, 31.0, 6.0))   # ground D6 pin on a nozzle
    p["IDH-L_indicator_holder"] = B(0.0, 80.0, 0.0, 20.0, 0.0, 12.0).cut(cyl(70.0, 10.0, -1.0, 13.0, 8.0))       # clamps a D8-stem indicator
    p["LJF-L_lateral_load_jack_frame"] = fuse_all([B(0.0, 120.0, 0.0, 60.0, 0.0, 10.0), B(0.0, 10.0, 0.0, 60.0, 0.0, 90.0)])
    p["PLF-L_pull_fixture"] = fuse_all([B(0.0, 80.0, 0.0, 80.0, 0.0, 10.0), B(30.0, 50.0, 30.0, 50.0, 10.0, 60.0)])
    p["SHM-L_half_lock_shim_set"] = B(0.0, 30.0, 0.0, 10.0, 0.0, 0.3)
    p["SDM-L_syringe_drive_plate"] = fuse_all([B(-31.0, 31.0, -10.0, 0.0, -65.0, 65.0), B(-31.0, 31.0, -60.0, -10.0, 40.0, 50.0),
                                               B(-31.0, 31.0, -60.0, -10.0, -65.0, -55.0)])
    p["SCB-L_syringe_clamp_block"] = B(0.0, 60.0, 0.0, 24.0, 0.0, 20.0).cut(cyl(30.0, 12.0, -1.0, 21.0, 16.0))
    return p


RIG_ROWS = [   # part, kind, process, material/spec, qty, test(s)
    ("RCB-L_rig_coupling_bracket", "custom", "cnc", "Al 6061-T6, 20 mm arm; ground pin pockets reamed D6 H7; bolts to the rig TP-1 pattern", 1, "CT-1..CT-9"),
    ("TPL-L_tool_plate", "custom", "cnc", "Al 7075-T6 10 mm; three D10 ball seats reamed, 4 magnet pockets, stud M10 thread", 2, "CT-1..CT-9"),
    ("CRA-L_cradle_adapter", "custom", "cnc", "Al 6061-T6: hangs the existing CB-1 small and CB-1 8ch300 cradle bodies from TPL-L", 2, "CT-1..CT-9"),
    ("LJ-L_latch_jaw + LL/LC/LA-L link, crank, arm + LCA-L cartridge + SPA-L anchor", "custom", "cnc / laser", "the machine's LTG-1 latch, one set", 1, "CT-0a, CT-0b, CT-3, CT-4"),
    ("DLB-L_dock_lift_rig (bar for two bays, two DLP-L pads)", "custom", "cnc", "Al bar 20 x 12 + hardened pads, on two D8 shafts", 1, "CT-0a, CT-0b, CT-5"),
    ("DB-L2_dock_base", "custom", "sheet_laser", "Al 5052-H32 6 mm, two bays", 1, "CT-1, CT-5"),
    ("BC-L_bay_ledge", "custom", "sheet_laser", "Al 5052-H32 8 mm, with two hardened D6 locating pins (hole + slot)", 4, "CT-1, CT-5"),
    ("DT-L_drip_tray", "custom", "print_mjf_pa12", "PA12, under each parked pipette", 2, "CT-5"),
    ("NRP-L_nozzle_reference_pin_holder", "custom", "turned", "Al, carries a ground D6 pin in place of a tip (the indicators touch it)", 2, "CT-1, CT-2"),
    ("IDH-L_indicator_holder", "custom", "sheet_laser", "Al 6 mm, clamps a D8-stem dial indicator; two on magnetic bases (X and Y)", 2, "CT-1, CT-2, CT-3"),
    ("LJF-L_lateral_load_jack_frame", "custom", "cnc", "Al: holds a hand screw jack and the S-type cell pushing the nozzle reference sideways", 1, "CT-2"),
    ("PLF-L_pull_fixture", "custom", "cnc", "Al: hook under the tool plate, pulled down through the S-type cell by an M10 jack screw", 1, "CT-3, CT-4"),
    ("SHM-L_half_lock_shim_set", "custom", "sheet_laser", "stainless shim 0.1 / 0.2 / 0.3 mm tabs (one under a ball)", 1, "CT-4"),
    ("SDM-L_syringe_drive_plate", "custom", "cnc", "Al 6061: on the rig TP-1 bolts; NEMA17 + T8 pusher and two 8 mm rods", 1, "syringe prototype"),
    ("SCB-L_syringe_clamp_block", "custom", "print_mjf_pa12", "PA12: clamps 1-4 luer-lock syringe barrels (1-5 mL)", 1, "syringe prototype"),
    ("tooling_ball_D10_threaded_shank", "bought", "bought", "hardened tooling ball D10 with threaded shank and shoulder (UNVERIFIED part)", 6, "coupling"),
    ("vee_seat_hardened_90deg", "bought", "bought", "hardened 90 deg vee seat 16 x 14 x 8 (kinematic-coupling component class, UNVERIFIED part)", 3, "coupling"),
    ("retention_knob_45deg_D16", "bought", "bought", "machine-tool retention knob class, 45 deg cone, D16 head (UNVERIFIED part)", 2, "latch"),
    ("dowel_ISO8734_6x20", "bought", "bought", "ISO 8734 hardened ground dowel pin 6 m6 x 20 (bay locators)", 4, "dock"),
    ("disc_spring_DIN2093_A20", "bought", "bought", "DIN 2093 A20 disc spring 20 x 10.2 x 1.1 (3 per latch + spares)", 6, "latch"),
    ("needle_roller_bearing_HK0408 + D4 pins", "bought", "bought", "drawn-cup needle bearings on hardened D4 pins (toggle)", 3, "latch"),
    ("constant_force_spring_46N", "bought", "bought", "constant-force spring 46 N class on a drum (UNVERIFIED part)", 2, "latch"),
    ("NEMA17_48_Tr8x2_lead_screw_motor", "bought", "bought", "dock lift motor with integrated Tr8x2 screw + anti-backlash nut", 1, "CT-0a, CT-0b"),
    ("Nanotec_LGA351S12-A-UIAP-038", "bought", "bought", "plunger pusher (V6), captive with guide, 0.6096 mm lead, 38.1 mm stroke", 1, "CT-0d"),
    ("load_cell_button_500N", "bought", "bought", "button cell under a dock lift pad (latch release/re-latch force)", 1, "CT-0a, CT-0b"),
    ("sealed_subminiature_pin_plunger_switch", "bought", "bought", "LOCK and EMPTY latch switches + LSF-L flag, LSB-L bracket, ladder resistors 1.0 k / 22 k / 4.7 k / 2.0 k in lever splices (the machine's latch sensing, latch.SENSE)", 2, "CT-0a, CT-0b, CT-4"),
    ("hall_switch_DRV5032_class", "bought", "bought", "omnipolar/unipolar Hall switch on a 30 x 10 board (4 per carriage)", 4, "CT-8"),
    ("magnet_N52_D4x2", "bought", "bought", "N52 disc D4 x 2 (tool code, up to 4 per tool)", 8, "CT-8"),
    ("D2F_switch_rev_L", "bought", "bought", "D2F class: bay-occupied (2) + push-rod force switch on the carriage (1)", 3, "CT-5, CT-7"),
    ("dial_indicator_0.001mm", "bought", "bought", "dial or digital indicator 0.001 mm, D8 stem, 12.5 mm range", 2, "CT-1, CT-2, CT-3"),
    ("magnetic_base", "bought", "bought", "magnetic indicator base with fine adjustment", 2, "CT-1, CT-2"),
    ("load_cell_S_500N", "bought", "bought", "S-type load cell 500 N with HX711-class amplifier", 1, "CT-2, CT-3, CT-4, CT-9"),
    ("load_cell_button_200N", "bought", "bought", "button load cell 200 N (20 kg) class in the pusher line, read through the rig HX711 (CT-0d pusher force to 120 N at 60 % of range; CT-9 1,000 uL plunger force); UNVERIFIED part", 1, "CT-0d, CT-9"),
    ("die_spring_CT-0d", "bought", "bought", "die spring about 12.5 N/mm, 12 mm working travel (about 150 N), in series with the 200 N cell in the pusher line: the pusher works against a rising force so its speed holds while the force is read; not modelled in 3D (bought, on the rig's pusher rod extension)", 1, "CT-0d"),
    ("HX711_amplifier_rev_L", "bought", "bought", "24-bit amplifier for the two new cells", 2, "CT-2..CT-9"),
    ("hand_screw_jack_M10", "bought", "bought", "M10 fine screw with hand wheel (load application)", 2, "CT-2, CT-3"),
    ("NEMA17_48_syringe", "bought", "bought", "NEMA17 48 mm: syringe pusher motor (rig prototype)", 1, "syringe prototype"),
    ("T8x2_screw_200_and_nut", "bought", "bought", "Tr8x2 lead screw 200 mm + anti-backlash nut", 1, "syringe prototype"),
    ("linear_rod_8x150_and_LM8UU", "bought", "bought", "2 rods + 2 bearings for the syringe pusher plate", 2, "syringe prototype"),
    ("luer_lock_syringe_1-5mL", "bought", "bought", "disposable luer-lock syringes + blunt needles / jetting tips (consumable)", 10, "syringe prototype")]

RIG_TESTS = {
    "FIRST - CT-0a latch release force": "the rig carriage with the LTG-1 latch, a tool latched on a seated two-bay dock; the dock lift pushes the "
                                         "roller through the 500 N button cell under the pad: log force vs bar stroke; pass: peak <= lift thrust / 1.5 "
                                         "and within 20 % of the calculated profile (head report coupling.latch.table)",
    "FIRST - CT-0b re-latch force and preload": "bar lowered slowly with the stud seated: the springs must close the toggle 100 of 100 times with "
                                                "0.67 x spring force (a third of the springs' force taken off by a hanging weight); preload measured "
                                                "with PLF-L and the S-cell (separation at the tool plate edge) for each of 6 knobs: pass >= 520 N",
    "FIRST - CT-1 repeatability, 1,000 exchanges": "two indicators (X, Y) on magnetic bases touch the ground D6 pin held on the nozzle (NRP-L); "
                                                   "exchange tool in/out of the two-bay dock with the dock lift; pass <= 0.10 mm 3 sigma at the nozzle",
    "FIRST - CT-0c comb entry with real tips": "one TPK-S group (S E S) cut for the rig M300's measured nozzle row: 500 park/re-pick cycles of "
                                               "real p300 tips in random order: no strike, no collar left out of its pocket, no tip left on a strip",
    "FIRST - CT-0d pusher force at 0.7 A": "LGA35 at 0.7 A, 24 V, 2 / 5 / 8 mm/s against the 200 N button cell with the die spring in series in "
                                           "the pusher line: pass >= 120 N at 5 mm/s (80 N x 1.5) without lost steps; also log the homing stall "
                                           "current (<= 0.57 A)",
    "CT-2 added deflection at 120 / 150 N": "LJF-L pushes the nozzle pin sideways through the 500 N S-cell; pass <= 0.05 mm",
    "CT-3 no lift-off at 2 x the working force": "PLF-L pulls the tool plate down through the S-cell to 2 x eject/strip force; separation < 0.005 mm",
    "CT-4 power-off pull and half-locks": "motor power off, pull at 2 x; shim under one ball, wrong bay, debris, a constant-force spring removed: "
                                          "the latch must never read LOCKED with a bad seat it can detect (latch.SENSE: stud 0.12 mm or more off), "
                                          "and the tool never drops; record which shims read CLOSED-EMPTY / NOT CLOSED",
    "CT-SYR syringe return spring": "10 mL syringe with the 1.5 x 20 x 75 return spring: plunger seal friction (pull-out force at 1 mm/s, wet), the "
                                    "plunger follows the pusher up 100 of 100 refills at 5 mm/s; pass: spring force out / measured friction + valve >= 1.5",
    "CT-5 3,000 exchanges": "automatic cycling between the two bays with the dock lift; no undetected failure, <= 1 detected per 1,000",
    "CT-6 pipetting after exchange": "gravimetric, S10 and M300, against the same pipette bolted directly",
    "CT-7 force switch through the push-rod": "load cell under the nozzle: trip within 15 % of the cradle's set force",
    "CT-8 tool code": "Hall switches against 1,000 insertions: no misread",
    "CT-9 real forces": "eight-tip pick-up, comb strip, ejector, 1,000 uL plunger"}


# =====================================================================================================
def main():
    OUT.mkdir(parents=True, exist_ok=True)
    rep = {"module": "LH-1 revision L exchange head (ADR-010)", "status": "CAD and calculation only; nothing built or measured",
           "frame": "carriage-local; global z = local + %g (K2 beam raised 145 mm over revision K: 30 in pass 1, 115 more in pass 2 for the "
                    "dock)" % ZB}
    # -------- configurations and interference
    configs = [("top, both tools up", dict(sA=0.0, sB=0.0, sG=0.0)),
               ("A full stroke, B up", dict(sA=P.L_Z_TRAVEL, sB=0.0, sG=0.0)),
               ("B full stroke, A up", dict(sA=0.0, sB=P.L_Z_TRAVEL, sG=0.0)),
               ("A eject over-travel", dict(sA=-P.Z_EJECT_TRAVEL, sB=0.0, sG=0.0)),
               ("B eject over-travel", dict(sA=0.0, sB=-P.Z_EJECT_TRAVEL, sG=0.0)),
               ("gripper full stroke", dict(sA=0.0, sB=0.0, sG=P.L_Z_TRAVEL)),
               ("A 1,000 uL tool, B 8ch 5-50", dict(sA=115.0, sB=0.0, sG=0.0, toolA="S1000", toolB="M50")),
               ("A 10-100 single, B 8ch 30-300 full", dict(sA=0.0, sB=P.L_Z_TRAVEL, sG=0.0, toolA="S100", toolB="M300")),
               ("A 8ch 0.5-10 eject, B syringe tool", dict(sA=-P.Z_EJECT_TRAVEL, sB=0.0, sG=0.0, toolA="M10", toolB="SYR")),
               ("A syringe full, B 1,000 uL eject", dict(sA=P.L_Z_TRAVEL, sB=-P.Z_EJECT_TRAVEL, sG=0.0, toolA="SYR", toolB="S1000"))]
    hits = []
    for lab, kw in configs:
        hits += interference(head_parts(**kw), lab)
    rep["interference"] = {"configs": [c[0] for c in configs], "pairs_over_1mm3": hits, "count": len(hits),
                           "expected_contacts_excluded": "balls on pins, stud in latch/bracket, nested switch/rod/pin holes, rails in "
                                                         "blocks (listed in _expected)"}
    parts0 = head_parts()
    rep["all_solids_valid"] = all(v.isValid() for v in parts0.values())
    rep["tool_masses"] = {k: tool_mass(k) for k in TOOLS}
    rep["cradles"] = cradle_report()
    # -------- mass
    dens = {"vees": "steel", "balls": "steel", "stud": "steel", "jaw": "steel", "cartridge": "steel", "link": "steel", "crank": "steel",
            "pivot_pin": "steel", "arm": "steel", "screw": "steel", "eject_pin": "steel",
            "push_rod": "steel", "roller": "steel", "pusher_rod": "steel"}
    bought_mass = {"motor": 0.36, "block0": 0.09, "block1": 0.09, "rail": 0.36, "pusher": 0.14, "spring": 0.04,
                   "gripper_body": 0.75}     # bought / real masses: GR-1 parts 0.52 (head report) + its drive about 0.23 (ASSUMED)
    m, mx = 0.0, {}
    for k, v in parts0.items():
        leaf = k.split(":")[-1]
        if ":tool:" in k:
            continue
        bm = bought_mass.get(leaf, 0.09 if leaf.startswith("xblock") else None)
        mm = bm if bm is not None else v.Volume() * RHO[dens.get(leaf, "al")]
        m += mm
        mx[leaf] = round(mx.get(leaf, 0.0) + mm, 3)
    tools_kg = 2 * 1.3
    extras = 0.6 + 0.8                                   # boards, cables, fasteners 0.6; X belt clamp + 2040 stiffeners 0.8 (ASSUMED from the head report)
    m_head = m + tools_kg + extras
    rep["mass"] = {"modelled_head_without_tools_kg": round(m, 2), "tools_kg (2 x TOOL_MAX_MASS)": tools_kg, "boards_cables_fasteners_stiffeners_kg": extras,
                   "head_kg": round(m_head, 2), "study_estimate_kg": 11.0, "present_head_kg": 13.55,
                   "breakdown_kg": dict(sorted(mx.items(), key=lambda kv: -kv[1]))}
    bb = cq.Compound.makeCompound(list(parts0.values())).BoundingBox()
    rep["envelope_mm"] = {"x": [round(bb.xmin, 1), round(bb.xmax, 1)], "y": [round(bb.ymin, 1), round(bb.ymax, 1)],
                          "z_global": [round(bb.zmin + ZB, 1), round(bb.zmax + ZB, 1)], "width_mm": round(bb.xlen, 1)}
    # -------- calculations
    rep["coupling"] = coupling_calcs()
    rep["coupling"]["eject_force_N"] = 90.0                  # the tip-eject case's force (loads_and_holding 'tip eject ...'): a stable key for the firmware check
    rep["M1_120N"] = m1_budget(120.0)
    rep["M1_150N"] = m1_budget(150.0)
    rep["recovery"] = {"what_it_takes": [P.L_X_BEAM, f"Y carriage plates {P.L_YC_T:g} mm (was 15)", P.L_X_BLOCKS,
                                         "carriage plate CP-L 10 mm (was 6) with the present 2040 stiffeners",
                                         "Z axes on MGN15 rails, two MGN15H blocks 60 mm apart (was MGN12H 46.4)",
                                         "coupling bracket 20 mm Al 6061 (study: 20 mm Al or 12 mm steel)",
                                         "nozzle on the rail plane (within 2 mm)"],
                       "carriage_spacing": "90 mm: set by two 72 mm brackets side by side; it does not enter M1 (each carriage carries its own load)"}
    rep["dynamics"] = dynamics(m_head)
    rep["z_clearance"] = z_clearance(parts0)
    rep["reach"] = reach_into_tubes()
    rep["tools"] = {k: {kk: vv for kk, vv in t.items()} for k, t in TOOLS.items()}
    rep["tool_interface"] = {
        "nozzle": "single nozzle or eight-nozzle row centre on the coupling axis (x 0, y -102.5 rel. the carriage axis) within 2 mm, row along Y",
        "nozzle_heights": f"two standard heights: tool-plate top to nozzle end Hn = {TOOLS['M300']['Hn']} mm (p10/p200/p300 tools: nozzle at "
                          f"z {P.L_NOZZLE_TOP_Z['standard']:g} at the top) and {TOOLS['S1000']['Hn']} mm (p1000: z {P.L_NOZZLE_TOP_Z['p1000']:g})",
        "plunger": "each cradle brings its pipette's plunger button under a guided slider whose top pad sits 2 mm below the tool-plate top on the "
                   "axis; the carriage pusher (0-35 mm stroke, >= 80 N) presses it through the bracket; different strokes are absorbed by "
                   "the pusher's software stroke table and the contact switch (found per exchange); different pipette heights by the cradle's "
                   "own V-block height, set so the slider pad and Hn are standard",
        "eject": "cradle lever brings the pipette's ejector button to a pad under the eject pin (22 mm from the axis); the fixed finger "
                 "on the head enters the bracket hole only in the eject over-travel (12 mm above s 0)",
        "force_switch": "the cradle's disc-spring stack (set to that pipette's pick-up force) lifts the D3 push-rod through the tool plate to "
                        "the D2F on the carriage",
        "code": "four D4x2 magnets in the tool plate top, read through 1 mm of bracket by four Hall switches",
        "mass_and_width": "<= 1.3 kg, <= 70 mm in X, centre of mass within 15 mm of the axis (study 3.2)"}
    rep["exchange"] = exchange_time()
    rep["latch_stroke_check"] = latch_stroke_check()
    rep["inrow_clearance"] = inrow_clearance()
    _old = inrow_clearance(35.0)
    rep["inrow_clearance"]["previous_stroke_307"] = {"in_row_stroke_s": _old["in_row_stroke_s"], "min_clearance_over_mm": _old["min_clearance_over_mm"],
                                                    "where": _old["where"], "pass": _old["pass"]}
    rep["syringe_return_spring"] = syringe_spring()
    rep["dock_bay"] = {"pitch_mm": BAY["pitch"], "bays": 8, "bay_height_mm": BAY["h"],
                       "height_note": "the parked tool's plate top sits at stroke L_DOCK_S (z %.0f global); bays stand on the dock base at deck level" % (dock_seat_top() + ZB),
                       "how_held": "the cradle's two dock lugs (round hole and slot, D6, the same on every cradle) sit on two hardened D6 pins on "
                                   "the bay saddle; the pipette hangs over a drip tray",
                       "release": "V1: no fixed post.  Dock lift DLB-1: one bar under all eight bays with a hardened pad under each bay's latch "
                                  "roller, raised by one NEMA17 48 + Tr8x2 lead-screw stepper (Z-axis class) at the row's LEFT end on two D8 "
                                  "shafts; bar DOWN the pads stand 3 mm under a locked roller.  Sequences in safety.sequences",
                       "dock_lift": dict(LT.DOCK_LIFT, stroke_check=LT.lift_stroke(), open_height_mm=LT.open_height_mm(),
                                         switches={"DOWN": "optical slot sensor behind the bar at the left end, flag F1 in its slot only at DOWN",
                                                   "OPEN": "sealed roller-lever switch on a post on the motor's top flange, flag F2 presses it from "
                                                           "%.1f mm of bar stroke to beyond the full stroke" % LT.open_height_mm()},
                                         thrust_basis="system.yaml dock_lift.recomputed.net_thrust_N_at (gross Z-drive thrust minus the bar's own load)"),
                       "in_row_stroke_s": P.L_DOCK_S - IN_ROW_LIFT, "in_row_lift_mm": IN_ROW_LIFT,
                       "occupied": "D2F switch pressed by the tool plate", "drip": "PA12 tray with an absorbent pad under each bay",
                       "clearances": "bay pitch 78 unchanged (tools up to 70 mm wide, 72 mm tool plates); the pads stand in front of the tool "
                                     "plates (y -173..-150 carriage-local), 3 mm under the roller",
                       "machine_place": "behind row G, bays x %.0f..%.0f at coupling-axis y %.0f (machine_l)" % (P.L_DOCK_X0, P.L_DOCK_X0 + 7 * P.L_DOCK_PITCH, P.L_DOCK_Y),
                       "cable_bay": "bay 8"}
    rep["safety"] = safety()
    rep["rig"] = {"tests": RIG_TESTS, "parts": [dict(zip(("part", "kind", "process", "spec", "qty", "tests"), r)) for r in RIG_ROWS]}
    rep["assembly_steps_and_gauges"] = [
        "1 vee seats: screw the three bought hardened vee seats into their bracket pockets (2 x M3 each) with the grooves pointing at the "
        "drilled centre mark; alignment needed +/-1 deg, +/-0.2 mm: checked by eye against the scribed radial line and with calipers to the "
        "centre mark (a kinematic coupling repeats on its contacts, not its placement)",
        "2 tooling balls: screw the three bought shank balls into the tool plate until each shoulder seats on its spot face (finger-tight + "
        "thread locker); check: the three ball tops within 0.03 mm, dial indicator on a magnetic base, plate face down on a flat surface",
        "3 retention knob: screw the bought knob to its shoulder (torque per its sheet); nothing to set",
        "4 latch: fit jaw, 3-disc cartridge (discs unloaded at assembly: no compression fixture), link, crank and arm on the needle-bearing "
        "pins, the two constant-force springs (hook on, no tool); with a tool latched, adjust the cartridge nut until a 0.50 mm feeler "
        "enters the cartridge gap window and a 0.60 mm feeler does not (stack compression 0.55 mm); lock the nut",
        "5 latch switches: with a tool latched and a 0.12 mm feeler between the flag and the LOCK switch's plunger, move the LOCK switch on "
        "its slots until it just operates; with no tool, the flag closed, set the EMPTY switch the same way 0.42 mm (0.54 - 0.12) short of the "
        "flag (feelers 0.40 / 0.45 bracket it); check: tool latched reads LOCKED (4.9 k at the ladder), no tool CLOSED-EMPTY (2.3 k), bar up "
        "NOT CLOSED (23 k); with a 0.3 mm feeler under the +X ball it must not read LOCKED (CT-4 at build)",
        "6 bracket to the Z carriage: 4 x M5 with an engineer's square against the carriage plate: coupling plane square within 0.1 mm over "
        "100 mm, checked with a 0.10 mm feeler under the square",
        "7 per-tool offset: at commissioning, measure each tool's nozzle position at the tip gate / head camera and store it (no gauge)",
        "8 dock lift: bar level within 0.5 mm over the row (spirit level and calipers from the dock base); home switch flag set so the pads "
        "stand 3 +/- 0.5 mm under a locked roller (feeler stack)",
        "dedicated gauges: none (was GG-L1..GG-L5); ordinary tools: feeler gauges, dial indicator on a magnetic base, calipers, square, level"]
    rep["requirements_disagreements_with_study"] = {
        "ball_circle": "R 36 not R 50 (width at 90 mm carriage pitch): preload 580 N; eject pad 22 mm from the axis (case B')",
        "stud_offset": "the plunger pusher needs the coupling axis, so the pull stud sits 14 mm off it, inside R/2: all balls stay loaded",
        "head_width": f"about {rep['envelope_mm']['width_mm']:.0f} mm, not 250 (90 mm pitch, camera outboard)",
        "Z_travel": f"{P.L_Z_TRAVEL:g} mm (study: probably 230-250)",
        "gantry_height": f"X beam mid raised to z {ZB:g} so the head's lowest point clears z 177 with tools up"}
    rep["remaining (not done in pass 1; done in pass 2 by cad/lh1/machine_l and the cradles here)"] = [
        "item 1: the K2 frame and deck in 3D with the full interference run (the frame values exist as a layout; the head is checked "
        "here on its own)", "item 3: cradle bodies for each pipette in CAD (envelopes and the interface only here)",
        "item 4: the eight-bay machine dock in the K2 frame (the bay design is here; the rig builds two)",
        "item 8: rev_l in the assembly report with K2 deck_positions, lh1_bom_l_proposed.csv, proposed_build_parts_l.yaml and the "
        "renders-ready full assembly STEP"]
    # -------- exports
    def export(name, s, stl=False):
        cq.exporters.export(cq.Workplane().add(s), str(OUT / f"{name}.step"))
        if stl:
            cq.exporters.export(cq.Workplane().add(s), str(OUT / f"{name}.stl"), tolerance=0.05, angularTolerance=0.2)
    xa = P.L_AXIS_X["carriage_A"]
    mvg = z_axis_moving(xa, 0.0)
    export("CBR-L_coupling_bracket", mv(mvg["bracket"], -xa, 0, -ZTOP0))
    for k_ in ("jaw", "link", "crank", "arm", "spring_anchor"):
        export({"jaw": "LJ-L_latch_jaw", "link": "LL-L_latch_link", "crank": "LC-L_latch_crank", "arm": "LA-L_latch_arm",
                "spring_anchor": "SPA-L_spring_anchor_and_pivot_cheeks"}[k_], mv(mvg[k_], -xa, 0, -ZTOP0))
    export("ZC-L_z_carriage", mv(mvg["zc"], -xa, 0, -ZTOP0))
    for k_, v_ in latch_switch_parts(xa, 0.0, split=True).items():
        if k_ in ("latch_switch_bracket",):
            export("LSB-L_latch_switch_bracket", mv(v_, -xa, 0, -ZTOP0))
    # integration 23 Nov 2026: the latch in its OPEN position (bar raised under the roller) as an assembly, and the latch at five angles
    sd_ = P.L_DOCK_S
    rz0_ = LT.roller_yz(-LT.G["over_deg"], sd_)[1]
    for nm_, ph_ in (("open", LT.G["open_deg"]), ("dead_centre", 0.0), ("half", 60.0)):
        asm_l = cq.Assembly(name=f"ltg_latch_{nm_}")
        dz_ = 3.0 + LT.roller_yz(ph_, sd_)[1] - rz0_
        car_ = z_axis_moving(0.0, sd_, phi=ph_)
        for k_ in ("bracket", "jaw", "cartridge", "link", "crank", "arm", "roller", "pivot_pin", "cf_spring_drums", "spring_anchor", "latch_switch", "vees"):
            asm_l.add(car_[k_], name=k_)
        for k_, v_ in tool_module(0.0, sd_, "M300").items():
            if k_ in ("tool_plate", "balls", "stud"):
                asm_l.add(v_, name="tool_" + k_)
        for k_, v_ in bay_parts(0.0, 0.0, 0.0, 1).items():
            if k_ in ("lift_bar", "bay0_lift_pad", "lift_flags"):
                asm_l.add(v_.translate(V(0, 0, dz_)), name=k_)
            elif k_ in ("lift_guides", "lift_open_switch", "lift_home_switch"):
                asm_l.add(v_, name=k_)
        asm_l.save(str(OUT / f"LTG-L_latch_{nm_}_assembly.step"))
    # SYR tool (direct drive) and its return spring as a real helix
    asm_s = cq.Assembly(name="syr_tool")
    for k_, v_ in tool_module(0.0, 0.0, "SYR").items():
        asm_s.add(v_, name=k_)
    try:
        t_ = TOOLS["SYR"]
        fl_top = tpl_top(0.0) - TPL["t"] - 0.5 - SYR_T["thumb_t"] - SYR_SPRING["installed_l"]
        asm_s.add(syringe_spring_helix(0.0, AXY[1], fl_top), name="return_spring_helix")
        export("SRS-L_syringe_return_spring_helix", syringe_spring_helix(0.0, 0.0, 0.0))
    except Exception as e:  # noqa: BLE001
        print("helix spring export failed:", e)
    asm_s.save(str(OUT / "SYR-L_syringe_tool_assembly.step"))
    export("PB-L_pusher_bracket", mv(mvg["pusher_bracket"], -xa, 0, -ZTOP0))
    tm = tool_module(xa, 0.0, "M300")
    export("TPL-L_tool_plate", mv(tm["tool_plate"], -xa, 0, -tpl_top(0)))
    for k in TOOLS:
        if k == "SYR":
            continue
        cp_ = cradle_parts(0.0, 0.0, k)
        export(f"CBL-{k}_cradle_body", mv(cp_["cradle"], 0, 0, -tpl_top(0)))
        export(f"CBL-{k}_straps", mv(cp_["straps"], 0, 0, -tpl_top(0)))
    export("CP-L_carriage_plate", head_fixed()["cp"])
    for k, v in rig_parts().items():
        export(k, v, stl=k.startswith(("DT-L", "SCB-L")))
    asm = cq.Assembly(name="head_l")
    for k, v in parts0.items():
        asm.add(v, name=k.replace(":", "__"))
    asm.save(str(OUT / "head_l_assembly.step"))
    dock = bay_parts(0.0, 0.0, 0.0, 2)
    export("BS-L_bay_saddle", dock["bay0_saddle"])
    export("DLP-L_dock_lift_pad", dock["bay0_lift_pad"])
    export("DLB-L_dock_lift_bar_rig", dock["lift_bar"])
    asm2 = cq.Assembly(name="rig_l_dock")
    for k, v in dock.items():
        asm2.add(v, name=k)
    asm2.save(str(OUT / "rig_l_two_bay_dock.step"))
    custom = [("CP-L_carriage_plate", "cnc", "Al 6061-T6 10 mm", 1), ("ZC-L_z_carriage", "cnc", "Al 6061-T6 10 mm", 3),
              ("CBR-L_coupling_bracket", "cnc", "Al 6061-T6, 20 mm arm + web; three shallow vee-seat pockets, latch channel", 2),
              ("LJ-L_latch_jaw", "cnc", "tool steel hardened 58 HRC, keyhole D11/D17, 45 deg slot-edge chamfers", 2),
              ("LL-L/LC-L/LA-L_latch_link_crank_arm", "sheet_laser + ream", "steel S355 6 mm (link, crank, arm) for D4 needle bearings", 2),
              ("SPA-L_spring_anchor_and_pivot_cheeks", "cnc", "Al 6061-T6", 2),
              ("LCA-L_latch_cartridge", "turned", "steel cup + M8 draw bolt + nut for 3 x A20 discs", 2),
              ("PB-L_pusher_bracket", "cnc", "Al 6061-T6 U standoff for the LGA35 (26 mm M3 square)", 2),
              ("PRX-L_pusher_rod_extension", "turned", "D6 steel rod, M4 female end", 2),
              ("TPL-L_tool_plate", "cnc", "Al 7075-T6 10 mm (one per tool, 6 in the first build incl. the syringe tool)", 6),
              ("CBL-M10/M50/M300/S10/S1000_cradle_body", "cnc (2 set-ups)", "Al 6061-T6 or cast tooling plate; one per pipette", 5),
              ("CBL-S100_cradle_body", "cnc (2 set-ups)", "Al 6061-T6: reserved, made when the 10-100 uL pipette is fitted", 0),
              ("CBL-straps", "cnc", "Al 6061-T6 opposing-V straps, 1 per eight-channel / 2 per single", 7),
              ("CBL-C_manifold_cheeks", "sheet_laser", "Al 5052 6 mm, a fixed and a loose cheek per eight-channel", 6),
              ("EJ-L_eject_finger", "cnc", "steel arm + D6 pin", 2),
              ("LSF-L_latch_sensing_flag", "sheet_laser + bend", "stainless 3 mm strip, 2 x M3 into the jaw top (latch.SENSE)", 2),
              ("LSB-L_latch_switch_bracket", "cnc", "Al 6061-T6: plate + shelf for the LOCK and EMPTY switches, slotted holes to set them", 2),
              ("SYR-L_syringe_body_clamp", "cnc", "Al 6061-T6 back plate + flange clamp for a 10 mL luer-lock syringe (direct drive)", 1)]
    bought = [(f"MGN15_rail_{RAIL['z'][1] - RAIL['z'][0]:.0f}_Z", 3), ("MGN15H_block", 6 + 4), (f"MGN15_rail_{beam_length() - 40:.0f}_X", 2), ("NEMA17_48", 3), ("Nanotec_LGA351S12-A-UIAP-038_pusher", 2),
              (f"T8x2_screw_{MOTOR['z0'] - Z_LOW - 8:.0f}_and_nut", 3), ("tooling_ball_D10_threaded_shank", 18), ("vee_seat_hardened_90deg_16x14x8", 6), ("retention_knob_45deg_D16_M10", 6),
              ("disc_spring_DIN2093_A20", 6 + 4), ("needle_roller_bearing_HK0408", 4 + 2), ("pin_D4_hardened_ground", 6), ("constant_force_spring_46N", 4),
              ("roller_D10_bearing_on_pin", 2), ("shaft_collar_D6_bore", 2), ("sealed_subminiature_pin_plunger_switch_latch_LOCK_EMPTY", 4),
              ("ladder_resistors_1k0_22k_4k7_2k0_1pct", 8), ("lever_splice_5way_and_2way_ladder", 8), ("hall_switch", 8), ("magnet_N52_D4x2", 24), ("D2F_switch", 2 + 8),
              ("syringe_10mL_luer_lock", 2), ("SYR_return_spring_1.5x20x75_stainless", 2)]
    with open(OUT / "head_l_parts.csv", "w", newline="") as fh:
        w = csv.writer(fh)
        w.writerow(["part", "kind", "process", "material_or_spec", "qty"])
        for n, pr, mt, q in custom:
            w.writerow([n, "custom", pr, mt, q])
        for n, q in bought:
            w.writerow([n, "bought", "bought", n, q])
    rep["parts_list"] = {"file": "out/head_l_parts.csv", "custom_lines": len(custom), "bought_lines": len(bought)}
    (OUT / "report.json").write_text(json.dumps(rep, indent=1, default=str))
    return rep


if __name__ == "__main__":
    r = main()
    print(json.dumps({"interference": r["interference"]["count"], "valid": r["all_solids_valid"], "mass": r["mass"],
                      "M1_150N": {k: r["M1_150N"][k] for k in ("sum_horizontal_mm", "sum_vertical_mm", "total_mm", "total_without_axial_screw_mm",
                                                              "horizontal_pass", "total_without_axial_pass_rule")},
                      "M1_120N_H": r["M1_120N"]["sum_horizontal_mm"], "z": {k: r["z_clearance"][k] for k in ("head_fixed_lowest_z", "lowest_tip_end_z", "gripper_lowest_z", "pass")},
                      "dyn": {k: v for k, v in r["dynamics"].items() if k.startswith(("X_", "Y_"))}, "exchange_s": r["exchange"]["total_s"],
                      "coupling": {k: [v["margin"], v["pass_1.5"]] for k, v in r["coupling"]["loads_and_holding"].items()},
                      "latch": {k: r["coupling"]["latch"][k] for k in ("release", "relatch", "stroke", "z_only_comparison") if k in r["coupling"]["latch"]}}, indent=1, default=str))
