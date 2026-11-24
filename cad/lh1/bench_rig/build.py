"""BR-1 bench rig (integration rev F, review "before any fabrication order" item 5; mechanical revision G3).

One LH-1 Z axis (rail, two blocks, Tr8 screw, thrust bearing, force-switch cradle, ZHS-1 hard stop) on a BR-2 mount plate,
carried by a bought belt linear module (X) on two 2040 posts (each on a 2040 bench foot) over a BR-3 base; stations under the
line: tip-rack nest (IN-P on the BR-4 carrier on the LC-1 platform load cell), TW-1/TW-4D comb station (on the BR-5 carrier on
the LC-2 platform load cell, TW-2 funnel and TW-3 bin below), a plate nest (IN-P on BR-3) and a balance with an evaporation-
trap vial (on its own table).  Each pipette is fitted in turn (its cradle set bolts to the same TP-1 tool plate, as
on the machine; rev J: one CB-1 cradle body + V-blocks/straps/clamp/collar/yoke per pipette); BR-3 is slid in Y on its feet so
the nest line sits under that pipette's nozzle row (the rig has no Y axis; rev J: located by the GG-4 index bar / GG-5 spacer).
Driven by the head's own bought controller (BTT MMB CAN V2.0) on a Raspberry Pi running the LH-1 firmware stack.

Supports: plunger force/travel (load cell in the HP-5 yoke + step count), gravimetric dispense at 0.5, 1, 2, 5, 10, 50 and
300 uL, single- and eight-tip pick-up and strip force, and thousands of unattended pick-up/strip cycles.

Geometry: real LH-1 parts are imported from cad/lh1/head and cad/lh1/deck_modules (nothing re-modelled); bought rig items
(linear module, posts, balance, load cells) are envelopes (approximations).  Rev G3: BR-2..BR-5 carry every hole the rig
needs (STEP + DXF); the parts list (fasteners included) is generated from the modelled joints.
Run:  .venv/bin/python cad/lh1/bench_rig/build.py
Out:  cad/lh1/bench_rig/out/{report.json, br1_assembly.step, BR1_parts_list.csv, BR-2..BR-5 .step/.dxf}
"""
from __future__ import annotations

import csv
import importlib.util
import json
import math
import sys
from collections import OrderedDict
from pathlib import Path

import cadquery as cq
from OCP.BRepAlgoAPI import BRepAlgoAPI_Common
from OCP.BRepGProp import BRepGProp
from OCP.GProp import GProp_GProps

HERE = Path(__file__).resolve().parent
LH1 = HERE.parent
ROOT = LH1.parent.parent
OUT = HERE / "out"
sys.path.insert(0, str(LH1))
import params as P  # noqa: E402


def _load(name, path):
    spec = importlib.util.spec_from_file_location(name, path)
    m = importlib.util.module_from_spec(spec)
    sys.modules[name] = m
    spec.loader.exec_module(m)
    return m


HD = _load("br_head", LH1 / "head" / "build.py")
DK = _load("br_deck", LH1 / "deck_modules" / "build.py")

Z_OFF = P.X_BEAM_MID_Z          # carriage-local z -> rig z (rig z 0 = machine deck top: same labware heights as LH-1)
ST = P.BR1_STATION_X            # rev G3: fixed station layout (params)
COMB_X = ST["comb_nest_centre"]
EJ = P.Z_EJECT_TRAVEL
RIG_TOOLS = ("pipette_small", "pipette_8ch_50", "pipette_8ch_300")


def station_x(tname: str) -> dict:
    """nozzle x of each station for tool tname (the comb nest is fixed: each tool strips at its own slot x)"""
    return {"tip_rack": ST["tip_rack"], "strip_comb": COMB_X + DK.TW["comb"]["moves"][tname]["x1"],
            "plate_nest": ST["plate_nest"], "balance": ST["balance"], "eject_B": COMB_X + DK.TW["hopper"]["eject_x"]}


# report key kept scalar for the firmware cross-check (electronics/lh1/check.py): the eight-channel strip x
STATIONS = dict(station_x("pipette_8ch_300"))
# rev G3: module envelope lowered so the carriage hole pattern lands on BR-2 and the FFC tail clears the carriage (z 55 < 57)
MODULE = dict(stroke=500.0, body=(-120.0, 640.0), body_y=(15.0, 75.0), body_z=(-40.0, 60.0),
              carriage_x=(-27.0, 63.0), carriage_y=(HD.PL_BACK_Y, 15.0), carriage_z=(-35.0, 55.0))   # G4: +3 in x (LC-4 window)
CARR_HOLES = [(-12.0, -25.0), (48.0, -25.0), (-12.0, 45.0), (48.0, 45.0)]   # BR-2 to carriage (60 x 70, x rel. rail; MEASURE)
BR2 = dict(x=(-128.0, 60.0), z=P.CARRIAGE_PLATE_Z, t=P.CARRIAGE_PLATE_T)   # G4: extended to -128 for the LC-4 bar
# rev G4: method-B force path.  EJ-1 finger -> BR-6 carrier (6 mm, floating in a BR-2 window, 2 x M3 csk from behind as on CP-1)
# -> BR-6 lug -> LC-4 single-point micro bar (free end) -> bar fixed end on BR-7 -> BR-2.  Overload: M4 set screw down through
# BR-2's top edge, 0.2 mm above the carrier top (feeler at build).
WIN4 = (-45.0, -31.5, -5.0, 60.5)   # G4: z from -5 so every EJ-1 root (z -3..39 over the three fingers) is clear of BR-2                       # BR-2 window (x0, x1, z0, z1), x rel. rail, local z
CAR6 = dict(x=(-44.5, -32.0), z=(-4.0, 60.0), lug_x=(-66.0, -32.0), lug_z=(54.0, 60.0), lug_y=(-34.2, HD.PL_FRONT_Y - 0.5))
LC4 = dict(L=80.0, W=12.7, H=12.7, x_free=-44.0, holes=(6.0, 18.0), shim=1.6, y=(-34.2, HD.PL_FRONT_Y - 0.5),
           spec="single-point micro bar load cell 20 kg (200 N), about 80 x 12.7 x 12.7, 2 x M4 at each end 12 mm apart, deflection "
                "about 0.3 mm at rated load (NOMINAL: measure the cell bought); safe overload 150 % (ASSUMED)")
BR7 = dict(x=(-126.0, -100.0), z=(40.0, 60.0), m4_x=(-121.0, -103.0), m4_z=46.0)
OVL4 = (-37.75, 60.5, 80.0)                             # LC-4 overload set screw axis x, from the window top to the BR-2 top edge
LC_STOP_GAP = 0.2                                       # set-screw gap (mm) of every load-cell overload stop, set with a feeler
# rev J: set by instrument + GG-3 count (0.23 mm), see report checks overload_stops_rev_j; rig plates laser-cut + secondary ops
RIG_PLATE_PROC = "sheet_laser" if getattr(P, "REV_J", False) else "cnc"
RIG_PLATE_NOTE = ("; rev J: LASER-CUT 12 mm, then tapped / countersunk / reamed (hole table)" if getattr(P, "REV_J", False) else "")
BENCH_Z = -150.0
LC4["z"] = (CAR6["lug_z"][1] + LC4["shim"], CAR6["lug_z"][1] + LC4["shim"] + LC4["H"])
BR3 = dict(x=(-85.0, 360.0), y=(-87.5, 172.5), t=12.0)   # NEST-LINE coordinates (y 0 = nest centres); placed at the tool's row y
CAR_T = 12.0                                             # BR-4 / BR-5 carrier thickness (top at z 0 = BR-3 top)
WIN = {"rack": (-67.0, 67.0, -56.0, 56.0), "comb": (COMB_X - 67.0, COMB_X + 67.0, -56.0, 74.0)}
BR4 = dict(x=(-65.0, 65.0), y=(-54.0, 54.0))
BR5 = dict(x=(-65.0, 65.0), y=(-54.0, 72.0), opening=(-60.0, 60.0, -40.0, 40.0))
LC = dict(L=130.0, W=30.0, H=22.0, holes=(7.5, 22.5), spacer=3.2,          # single-point platform cell bar along +Y (NOMINAL)
          spec="single-point platform load cell 50 kg (500 N), bar about 130 x 30 x 22, 2 x M6 at each end 15 mm apart "
               "(hole pattern NOMINAL: MEASURE the cell bought before machining BR-3/BR-4/BR-5)")
LC_AT = {"LC-1": (0.0, -15.0), "LC-2": (COMB_X + 30.0, 40.0)}                # (bar centre x, bar start y) nest-line coords
FEET = [(-58.0, -75.0), (337.0, -75.0), (-58.0, 160.0), (337.0, 160.0)]      # 2040 feet under BR-3, 40 along X, 20 along Y
POST_X = (MODULE["body"][0] + 20.0, MODULE["body"][1] - 20.0)
POST_Y = (HD.PL_BACK_Y + 40.0, HD.PL_BACK_Y + 80.0)
POST_FOOT_L = 200.0
POST_LEN = (MODULE["body_z"][0] + Z_OFF) - (BENCH_Z + 20.0)


def box(x0, x1, y0, y1, z0, z1):
    return cq.Solid.makeBox(x1 - x0, y1 - y0, z1 - z0, cq.Vector(x0, y0, z0))


def shp(w):
    if isinstance(w, cq.Workplane):
        v = w.vals()
        return v[0] if len(v) == 1 else cq.Compound.makeCompound(v)
    return w


def vol_common(a, b):
    op = BRepAlgoAPI_Common(a.wrapped, b.wrapped)
    op.Build()
    pr = GProp_GProps()
    BRepGProp.VolumeProperties_s(op.Shape(), pr)
    return abs(pr.Mass())


# =====================================================================================================
# rig plates: every hole the rig needs (rev G3).  Hole tables are the single source for the fastener schedule.
# =====================================================================================================
def _plate(x0, x1, y0, y1, t, windows=(), feats=(), csk=True):
    """flat plate z 0..t; feats: (kind, x, y, d, dk) kinds 'thru', 'tap', 'ream', 'csk' (from +z)"""
    s = box(x0, x1, y0, y1, 0.0, t)
    for wx0, wx1, wy0, wy1 in windows:
        s = s.cut(box(wx0, wx1, wy0, wy1, -1.0, t + 1.0))
    for kind, x, y, d, dk in feats:
        s = s.cut(DK.cyl(x, y, -1.0, t + 1.0, d))
        if kind == "csk" and csk:
            s = s.cut(DK.csk(x, y, t, 0.0, d, dk))
    return s


def br3_feats():
    f = []
    for fx, fy in FEET:                                                      # 2 x M5 csk per foot into the tapped core holes
        f += [("csk", fx + sx * 10.0, fy, 5.5, 10.4, "BR-3 to 2040 foot", "ISO10642_M5x16") for sx in (-1, 1)]
    xp = ST["plate_nest"]
    f += [("tap", xp + x, y, 3.3, 0, "plate-nest IN-P to BR-3 (M4 tapped)", "ISO10642_M4x10") for x, y in P.INSERT_SCREWS]
    f += [("ream", xp + x, y, P.DOWEL_D, 0, "plate-nest dowels (pressed in BR-3)", "DOWEL_D4x10") for x, y in P.INSERT_DOWELS]
    for lc, (bx_, by0) in LC_AT.items():                                    # fixed end of each load-cell bar
        f += [("thru", bx_, by0 + LC["L"] - h, 6.6, 0, f"{lc} fixed end to BR-3 (from above, 3.2 mm washer spacer)", "ISO4762_M6x25")
              for h in LC["holes"]]
    for lc, (x, y, sy) in STOP9.items():                                    # rev G4: overload-stop brackets BR-9
        f += [("tap", x + dx, y + sy * 6.0, 3.3, 0, f"BR-9 {lc} overload-stop bracket to BR-3 (M4 tapped, 1.6 mm washer spacer)",
               "ISO4762_M4x16") for dx in (-10.0, 10.0)]
    return f


def br4_feats():
    f = [("tap", x, y, 3.3, 0, "rack-nest IN-P to BR-4 (M4 tapped)", "ISO10642_M4x10") for x, y in P.INSERT_SCREWS]
    f += [("ream", x, y, P.DOWEL_D, 0, "rack-nest dowels (pressed in BR-4)", "DOWEL_D4x10") for x, y in P.INSERT_DOWELS]
    bx_, by0 = LC_AT["LC-1"]
    f += [("csk", bx_, by0 + h, 6.6, 12.4, "LC-1 free end to BR-4 (countersunk, under IN-P)", "ISO10642_M6x25") for h in LC["holes"]]
    return f


def br5_feats():
    f = [("tap", x, y, 3.3, 0, "TW-1 to BR-5 (M4 tapped)", "ISO10642_M4x10") for x, y in P.INSERT_SCREWS]
    f += [("ream", x, y, P.DOWEL_D, 0, "TW-1 dowels (pressed in BR-5)", "DOWEL_D4x10") for x, y in P.INSERT_DOWELS]
    bx_, by0 = LC_AT["LC-2"]
    f += [("csk", bx_ - COMB_X, by0 + h, 6.6, 12.4, "LC-2 free end to BR-5 (countersunk, under TW-1)", "ISO10642_M6x25")
          for h in LC["holes"]]
    return f


def br2_feats():
    """BR-2 flat coords (x rel. rail, y = carriage-local z): the Z-axis hole set of the machine's CP-1 + carriage holes"""
    i8 = P.TOOL_NAMES.index("pipette_8ch_50")            # an axis that carries ZHS-1 on the machine (same hole set)
    h = HD.axis_plate_holes(i8, 0.0)
    f = [("tap", x, z, 2.5, 0, "Z rail to BR-2 (M3 tapped, 25 mm pitch)", None) for x, z in h["rail_m3"]]
    f += [("tap", x, z, 2.5, 0, "ZSW-1 to BR-2", None) for x, z in h["zsw_m3"]]
    f += [("tap", x, z, 1.6, 0, "FFC-F to BR-2 (M2)", None) for x, z in h["ffcf_m2"]]
    f += [("tap", x, z, 3.3, 0, "ZMB-1 legs to BR-2 (M4)", None) for x, z in h["zmb_m4"]]
    f += [("tap", x, z, 3.3, 0, "ZBH-1 to BR-2 (M4)", None) for x, z in h["zbh_m4"]]
    f += [("tap", x, z, 3.3, 0, "ZHS-1 to BR-2 (M4)", None) for x, z in h["zhs_m4"]]
    # rev G4: the EJ-1 root holes are in the BR-6 carrier (LC-4 force path), not in BR-2
    f += [("tap", x, BR7["m4_z"], 3.3, 0, "BR-7 LC-4 standoff to BR-2 (M4 tapped)", "ISO4762_M4x10") for x in BR7["m4_x"]]
    f += [("csk", x, z, 5.5, 10.4, "BR-2 to the module carriage (ISO 10642 from the front)", "ISO10642_M5x12") for x, z in CARR_HOLES]
    return f


# rev G4: BR-9 overload-stop brackets (6 mm Al flat 40 x 30: 2 x M4 clearance at local y 6, M4 tapped at local y 27 for the stop
# screw).  (x, y of the bracket's local y 0, direction of local +y) in nest-line coords: LC-1 under BR-3's front strip reaching under
# BR-4 (stop screw UP, gap 0.2 to BR-4: pick-up overload); LC-2 on BR-3's top reaching over BR-5's back strip (screw DOWN: strip overload)
STOP9 = {"LC-1": (0.0, -79.0, 1.0), "LC-2": (COMB_X - 45.0, 96.0, -1.0)}
BR9 = dict(x=(-20.0, 20.0), L=30.0, t=6.0, clr=[(-10.0, 6.0), (10.0, 6.0)], tap=(0.0, 27.0))


def br9_flat(csk=True):
    return _plate(*BR9["x"], 0.0, BR9["L"], BR9["t"], feats=[("thru", x, y, 4.5, 0) for x, y in BR9["clr"]] + [("tap", *BR9["tap"], 3.3, 0)])


def br9_placed(lc, yn):
    x, y0, sy = STOP9[lc]
    sol = br9_flat()
    if sy < 0:
        sol = sol.mirror("XZ")
    z = -BR3["t"] - LC["spacer"] / 2 - BR9["t"] if lc == "LC-1" else LC["spacer"] / 2      # 1.6 mm washer spacer
    return sol.moved(cq.Location(cq.Vector(x, yn + y0, z)))


def br9_screw(lc, yn):
    """ISO 4026 M4 set screw + nut in the stop bracket, tip LC_STOP_GAP from the carrier face"""
    x, y0, sy = STOP9[lc]
    y = yn + y0 + sy * BR9["tap"][1]
    if lc == "LC-1":
        zb = -BR3["t"] - LC["spacer"] / 2 - BR9["t"]
        return DK.cyl(x, y, zb - 8.0, -CAR_T - LC_STOP_GAP, 3.3)       # modelled at the tap drill (thread engaged)
    zt = LC["spacer"] / 2
    return DK.cyl(x, y, LC_STOP_GAP, zt + BR9["t"] + 8.0, 3.3)


# rev G4: BR-8 electronics plate (3 mm Al, laser) on the +Y face of the left post: board hole patterns NOMINAL (measure the boards)
BR8 = dict(u=(-230.0, 0.0), v=(100.0, 300.0), t=3.0)
BR8_HOLES = {"post (2 x ISO 7380 M5 + slot-6 T-nut)": ("thru", 5.5, [(-100.0, 120.0), (-100.0, 280.0)]),
             "MMB CAN V2.0 (100.3 x 51, M3 standoffs)": ("thru", 3.2, [(-221.0, 154.0), (-120.7, 154.0), (-221.0, 205.0), (-120.7, 205.0)]),
             "Raspberry Pi 4 (58 x 49, M2.5 standoffs)": ("thru", 2.7, [(-86.0, 235.0), (-28.0, 235.0), (-86.0, 284.0), (-28.0, 284.0)]),
             "HX711 x 4 (2 holes 29 apart each, M3 standoffs)": ("thru", 3.2, [(x, v) for v in (115.0, 145.0, 175.0, 205.0) for x in (-75.0, -46.0)])}


def br8_flat(csk=True):
    feats = [(k, x, y, d, 0) for k, d, pts in BR8_HOLES.values() for x, y in pts]
    return _plate(*BR8["u"], *BR8["v"], BR8["t"], feats=feats)


def br8_placed():
    """on the +Y face of the left post (y POST_Y[1]), plate thickness toward +Y; u = rig x, v = rig z"""
    return br8_flat().rotate(cq.Vector(0, 0, 0), cq.Vector(1, 0, 0), 90).moved(cq.Location(cq.Vector(0, POST_Y[1] + BR8["t"], 0)))


def br6_carrier():
    """BR-6 (CNC 6061): EJ-1 carrier in the BR-2 window + lug under the LC-4 free end; carriage-local, rail at x 0"""
    yb, yf = HD.PL_BACK_Y, HD.PL_FRONT_Y
    w = box(*CAR6["x"], yf, yb, *CAR6["z"])
    w = w.fuse(box(CAR6["x"][0], CAR6["lug_x"][1], CAR6["lug_y"][0], yb, *CAR6["lug_z"]))
    w = w.fuse(box(CAR6["lug_x"][0], CAR6["x"][0], CAR6["lug_y"][0], CAR6["lug_y"][1], *CAR6["lug_z"])).clean()
    for x, z in HD.ej_root_holes(P.TOOL_NAMES.index("pipette_8ch_50")):   # same pair for every finger; ISO 4762 from behind
        w = w.cut(cq.Solid.makeCylinder(1.7, 10.0, cq.Vector(x, yf - 2.0, z), cq.Vector(0, 1, 0)))   # (free space behind BR-6)
    yc = sum(LC4["y"]) / 2
    for h in LC4["holes"]:
        w = w.cut(DK.cyl(LC4["x_free"] - h, yc, CAR6["lug_z"][0] - 1.0, CAR6["lug_z"][1] + 1.0, 3.3))    # M4 tapped through the lug
    return w


def br7_standoff():
    w = box(*BR7["x"], LC4["y"][0], HD.PL_FRONT_Y, *BR7["z"])
    for x in BR7["m4_x"]:
        w = w.cut(cq.Solid.makeCylinder(2.25, 20.0, cq.Vector(x, LC4["y"][0] - 1.0, BR7["m4_z"]), cq.Vector(0, 1, 0)))
        w = w.cut(cq.Solid.makeCylinder(4.0, 4.4 + 1.0, cq.Vector(x, LC4["y"][0] - 1.0, BR7["m4_z"]), cq.Vector(0, 1, 0)))
    yc = sum(LC4["y"]) / 2
    x0 = LC4["x_free"] - LC4["L"]
    for h in LC4["holes"]:
        w = w.cut(DK.cyl(x0 + h, yc, BR7["z"][1] - 9.0, BR7["z"][1] + 1.0, 3.3))          # M4 tapped, 9 deep
    return w


def lc4_items(xc):
    """BR-6, BR-7, the LC-4 bar and the overload set screw, moved to the rig axis at x = xc"""
    loc = cq.Location(cq.Vector(xc, 0, Z_OFF))
    bar = box(LC4["x_free"] - LC4["L"], LC4["x_free"], *LC4["y"], *LC4["z"])
    ovl = cq.Solid.makeCylinder(2.0, OVL4[2] + 8.0 - (CAR6["z"][1] + LC_STOP_GAP),
                                cq.Vector(OVL4[0], (HD.PL_FRONT_Y + HD.PL_BACK_Y) / 2, CAR6["z"][1] + LC_STOP_GAP), cq.Vector(0, 0, 1))
    return [("BR-6", "BR-6_ej_carrier_lc4", "custom", br6_carrier().moved(loc)),
            ("BR-7", "BR-7_lc4_standoff", "custom", br7_standoff().moved(loc)),
            ("LC-4", "load_cell_200N_bar", "bought", bar.moved(loc)),
            ("ovl4", "ISO4026_M4x20_stop", "bought", ovl.moved(loc))]


def hp5r(name):
    """HP-5R (rig only, CNC 6061): plunger yoke 10 mm thick with a pocket for LC-3 (tool frame).  Same footprint as HP-5."""
    g = HD.hp_geom(name)
    z0 = g["button_top"] + 1.0
    ax_, ay = g["act"]
    w = box(g["hx"] - g["button_d"] / 2, ax_ + 7, ay - 6, g["y"] + 6, z0, z0 + 10.0)
    w = w.cut(DK.cyl(ax_, ay, z0 - 1, z0 + 6, 2.5))                                   # actuator rod, tapped
    w = w.cut(DK.cyl(g["hx"], g["y"], z0 - 1, z0 + 5.5, 10.5))                        # LC-3 pocket (cell 5 mm: 0.5 recessed)
    return w.cut(DK.cyl(g["hx"], g["y"], z0 + 5.4, z0 + 11, 3.4))                     # M3 into the cell's tapped back


def lc3_cell(name):
    g = HD.hp_geom(name)
    z0 = g["button_top"] + 1.0
    return DK.cyl(g["hx"], g["y"], z0 + 0.5, z0 + 5.5, 10.0).fuse(DK.cyl(g["hx"], g["y"], z0 - 0.5, z0 + 0.5, 3.0))


def br2_flat(csk=True):
    x0, x1 = BR2["x"]
    z0, z1 = BR2["z"]
    t = BR2["t"]
    s = box(x0, x1, z0, z1, 0.0, t)
    fx0, fx1 = HD.FFC["x"]
    sz0, sz1 = HD.FFC["slot_z"]
    s = s.cut(box(fx0 - 1.0, fx1 + 1.0, sz0, sz1, -1.0, t + 1.0))           # FFC tail slot (as CP-1)
    s = s.cut(box(WIN4[0], WIN4[1], WIN4[2], WIN4[3], -1.0, t + 1.0))         # rev G4: window for the BR-6 carrier (LC-4)
    s = s.cut(cq.Solid.makeCylinder(3.3 / 2, OVL4[2] - OVL4[1] + 1, cq.Vector(OVL4[0], OVL4[1], t / 2), cq.Vector(0, 1, 0)))   # M4 tap, edge
    for kind, x, y, d, dk, *_ in br2_feats():
        s = s.cut(DK.cyl(x, y, -1.0, t + 1.0, d))
        if csk and kind == "csk":
            s = s.cut(DK.csk(x, y, t, 0.0, d, dk))                           # front face (+z) of the flat
        if csk and kind == "csk_back":
            s = s.cut(cq.Solid.makeCone(dk / 2 + 0.01, d / 2, (dk - d) / 2 + 0.01, cq.Vector(x, y, -0.01), cq.Vector(0, 0, 1)))
    return s


def mount_plate(xc):
    """BR-2 (CNC Al 6061-T6, 6 mm) in rig coordinates, rail at x = xc"""
    return shp(HD.flat_to_xz(cq.Workplane(obj=br2_flat()), HD.PL_BACK_Y)).moved(cq.Location(cq.Vector(xc, 0, Z_OFF)))


def br3_flat(csk=True):
    feats = [(k, x, y, d, dk) for k, x, y, d, dk, *_ in br3_feats()]
    return _plate(BR3["x"][0], BR3["x"][1], BR3["y"][0], BR3["y"][1], BR3["t"], windows=WIN.values(), feats=feats, csk=csk)


def br4_flat(csk=True):
    return _plate(*BR4["x"], *BR4["y"], CAR_T, feats=[(k, x, y, d, dk) for k, x, y, d, dk, *_ in br4_feats()], csk=csk)


def br5_flat(csk=True):
    return _plate(*BR5["x"], *BR5["y"], CAR_T, windows=[BR5["opening"]],
                  feats=[(k, x, y, d, dk) for k, x, y, d, dk, *_ in br5_feats()], csk=csk)


def br3_plate(yn):
    return br3_flat().moved(cq.Location(cq.Vector(0, yn, -BR3["t"])))


def carriage(xc):
    m = MODULE
    return box(xc + m["carriage_x"][0], xc + m["carriage_x"][1], *m["carriage_y"], m["carriage_z"][0] + Z_OFF, m["carriage_z"][1] + Z_OFF)


def rig_stop(xc, s_set):
    """ZHS-1 block + set screw (rail at x = xc), screw top at the ledge underside for stroke s_set"""
    loc = cq.Location(cq.Vector(xc, 0, Z_OFF))
    return [("ZHS", "ZHS-1_z_hard_stop", "custom", shp(HD.z_stop_block()).moved(loc)),
            ("zhsscr", "ZHS_set_screw_M5x30_locknut", "bought", shp(HD.z_stop_screw(s_set)).moved(loc))]


def axis_parts(i, s, xc):
    """Z axis i (fixed + moving) with its axis at rig x = xc; carriage-local y kept, z + Z_OFF"""
    dx = xc - HD.AXES_X[i]
    keep = ("zrail", "ZMB", "motor", "coupler", "screw", "collars", "zbrg", "ZBH", "ZSW", "switch", "zup", "FFCF", "ffctail", "EJ")
    out = []
    for it in HD.static_items():
        if it.kind != "ctx" and it.name.startswith(keep) and it.name.endswith(str(i)) and not it.name.startswith(("xblk", "XSP", "stiff")):
            out.append((it.name, it.part, it.kind, shp(it.w).moved(cq.Location(cq.Vector(dx, 0, Z_OFF)))))
    name = P.TOOL_NAMES[i]
    tag = HD.HP_TAG.get(name)
    for it in HD.axis_items(i, s):
        w = it.w
        part = it.part
        if it.name == f"HP5@{i}":                                   # rev G4: rig yoke with LC-3
            w, part = HD._loc(cq.Workplane(obj=hp5r(name)), HD.AXES_X[i], HD.zb_of(s)), f"HP-5R_plunger_yoke_lc3_{tag}"
        out.append((it.name, part, it.kind, shp(w).moved(cq.Location(cq.Vector(dx, 0, Z_OFF)))))
    if tag:
        out.append((f"LC3@{i}", "load_cell_200N_button", "bought",
                    shp(HD._loc(cq.Workplane(obj=lc3_cell(name)), HD.AXES_X[i], HD.zb_of(s))).moved(cq.Location(cq.Vector(dx, 0, Z_OFF)))))
    return out


# rev J (ADR-003 #7): BR-3 is no longer slid in Y "by eye".  GG-4 index bar (PA12) on the left post foot's top slot (2 x M5 T-nuts)
# reaches forward along the bench; the back face of BR-3's front-left foot is pushed against its end for the small pipette, and
# against GG-5 (a two-step spacer, 20.0 / 22.5 mm, engraved '8ch50' / '8ch300') for the eight-channels.
def br3_foot_back_y(tname):
    return HD.Y_TF + HD.hp_geom(tname)["y"] + FEET[0][1] + 10.0


GG4 = dict(x=(-79.5, -62.0), base_x=(-115.0, -79.5), z=(BENCH_Z, BENCH_Z + 15.0))


def gg4_bar():
    """arm on the bench between the left post foot (x <= -80) and BR-3's front-left foot; base on the post foot's top slot"""
    yb = br3_foot_back_y("pipette_small")
    pyc = sum(POST_Y) / 2
    y_end_foot = pyc - POST_FOOT_L / 2                       # front end of the left post foot
    arm = box(GG4["x"][0], GG4["x"][1], yb, y_end_foot + 30.0, *GG4["z"])
    riser = box(GG4["x"][0], GG4["x"][1], y_end_foot, y_end_foot + 30.0, BENCH_Z, BENCH_Z + 25.0)
    base = box(GG4["base_x"][0], GG4["x"][0] + 0.01, y_end_foot, y_end_foot + 30.0, BENCH_Z + 20.0, BENCH_Z + 25.0)
    s = arm.fuse(riser).fuse(base)
    for yy in (y_end_foot + 8.0, y_end_foot + 22.0):
        s = s.cut(DK.cyl(POST_X[0], yy, BENCH_Z + 19.0, BENCH_Z + 26.0, 5.5))
    return s


def gg5_spacer():
    d1 = br3_foot_back_y("pipette_small") - br3_foot_back_y("pipette_8ch_50")
    d2 = br3_foot_back_y("pipette_small") - br3_foot_back_y("pipette_8ch_300")
    return box(0, 24, 0, d1, 0, 12).fuse(box(24, 48, 0, d2, 0, 12))


def rig_static():
    """items that do not move with X and do not follow BR-3 (module body, posts and their bench feet)"""
    m = MODULE
    items = [("LM_linear_module_body", "belt_linear_module_500", "bought",
              box(m["body"][0], m["body"][1], *m["body_y"], m["body_z"][0] + Z_OFF, m["body_z"][1] + Z_OFF))]
    pyc = sum(POST_Y) / 2
    for k, x in enumerate(POST_X):
        items.append((f"POST_{k}", f"2040_post_{POST_LEN:.0f}", "bought", box(x - 10.0, x + 10.0, *POST_Y, BENCH_Z + 20.0, m["body_z"][0] + Z_OFF)))
        items.append((f"POSTFOOT_{k}", f"2040_post_foot_{POST_FOOT_L:.0f}", "bought",
                      box(x - 20.0, x + 20.0, pyc - POST_FOOT_L / 2, pyc + POST_FOOT_L / 2, BENCH_Z, BENCH_Z + 20.0)))
    items.append(("BR-8_electronics_plate", "BR-8_electronics_plate", "custom", br8_placed()))      # rev G4
    if getattr(P, "REV_J", False):
        items.append(("GG-4_br3_y_index_bar", "GG-4_br3_y_index_bar", "custom", gg4_bar()))
    return items


def stations(i):
    """BR-3 and everything on/under it, with the nest line at the nozzle-row y of tool i (BR-3 slid in Y per pipette)"""
    tname = P.TOOL_NAMES[i]
    xn, yn = HD.AXES_X[i] + HD.hp_geom(tname)["hx"], HD.Y_TF + HD.hp_geom(tname)["y"]
    out = []
    plain, _ = DK.insert_plate(tap_m3=DK.locator_tap_points())
    locs = DK.locator_set()
    # TW-1 exactly as the deck builds it (comb feet, tip-check and TW-5 hopper taps; funnel screw clearances)
    tw_plate, _ = DK.insert_plate(opening=DK.TW["opening"], tap_m3=DK.TW["comb"]["feet_m3"] + [(sx * x, y) for x, y in DK.TW["tipcheck"]["m3"]
                                                                                               for sx in (-1, 1)] + DK.TW["hopper"]["m3"],
                                  clr_m3=DK.TW["screws"])
    rack = DK.B(-DK.SBS_HX, DK.SBS_HX, -DK.SBS_HY, DK.SBS_HY, DK.SEAT, DK.SEAT + P.TIP_RACK_H[P.TOOL_TIP[tname]])

    def put(name, part, kind, s, x, y=0.0, z=0.0):
        out.append((name, part, kind, s.moved(cq.Location(cq.Vector(x, yn + y, z)))))
    put("BR-3_base", "BR-3_base_plate", "custom", br3_flat(), 0.0, z=-BR3["t"])
    for k, (fx, fy) in enumerate(FEET):
        put(f"FOOT_{k}", "2040_foot_138", "bought", box(fx - 20, fx + 20, fy - 10, fy + 10, BENCH_Z, -BR3["t"]), 0.0)
    for lc, (bx_, by0) in LC_AT.items():
        part = "load_cell_500N_platform"
        put(f"{lc}_load_cell", part, "bought", box(bx_ - LC["W"] / 2, bx_ + LC["W"] / 2, by0, by0 + LC["L"],
                                                    -CAR_T - LC["spacer"] - LC["H"], -CAR_T - LC["spacer"]), 0.0)
    x = ST["tip_rack"]
    put("BR-4_rack_carrier", "BR-4_rack_nest_carrier", "custom", br4_flat(), x, z=-CAR_T)
    for lc in STOP9:                                                                   # rev G4: load-cell overload stops
        out.append((f"BR-9_{lc}", "BR-9_lc_overload_stop", "custom", br9_placed(lc, yn)))
        out.append((f"stop_{lc}", "ISO4026_M4x16_stop", "bought", br9_screw(lc, yn)))
    put("rack_insert", "IN-P_plain_insert_plate", "custom", plain, x)
    for k, l in enumerate(locs):
        put(f"rack_loc{k}", "LOC-1_corner_locator", "custom", l, x)
    put("tip_rack", "labware_tiprack", "labware", rack, x)
    x = COMB_X
    put("BR-5_comb_carrier", "BR-5_comb_station_carrier", "custom", br5_flat(), x, z=-CAR_T)
    put("comb_insert", "TW-1_tip_waste_insert_plate", "custom", tw_plate, x)
    put("comb", "TW-4D_strip_comb", "custom", DK.strip_comb(), x)
    put("eject_hopper", "TW-5_eject_hopper", "custom", DK.tip_hopper(), x)        # rev G1
    put("tip_funnel", "TW-2_tip_funnel", "custom", DK.tip_funnel(), x)            # rev G2: carries the bin on its rails
    put("tip_bin", "TW-3_tip_bin", "custom", DK.tip_bin(), x)
    x = ST["plate_nest"]
    put("plate_insert", "IN-P_plain_insert_plate", "custom", plain, x)
    for k, l in enumerate(locs):
        put(f"plate_loc{k}", "LOC-1_corner_locator", "custom", l, x)
    put("pcr_plate", "labware_pcr_plate", "labware", DK.pcr_plate(False).moved(cq.Location(cq.Vector(0, 0, DK.SEAT))), x)
    x = ST["balance"]
    put("BAL_balance_env", "balance_6place_env", "bought", DK.B(-110, 110, -60, 260, BENCH_Z, -6.0), x)      # on its own table
    put("BAL_vial_trap", "evaporation_trap_vial", "bought", DK.B(-12, 12, -12, 12, -6.0, 40.0), x)
    return out, (xn, yn)


def overlaps(mov, fixed, sname, s, designed_names=("tip_rack", "comb", "pcr_plate", "BAL_vial_trap")):
    res = []
    for mn, _, _, ms in mov:
        mb = ms.BoundingBox()
        for fn, _, fk, fs in fixed:
            fb = fs.BoundingBox()
            if mb.xmax < fb.xmin or fb.xmax < mb.xmin or mb.ymax < fb.ymin or fb.ymax < mb.ymin or mb.zmax < fb.zmin or fb.zmax < mb.zmin:
                continue
            v = vol_common(ms, fs)
            if v > 1.0:
                designed = mn.startswith("pipette") and fn in designed_names
                res.append({"moving": mn, "fixed": fn, "station": sname, "s": round(s, 2), "mm3": round(v, 1),
                            "class": "working contact (nozzle in tip/slot/well)" if designed else "real"})
    return res


def stop_setting(tname):
    if tname in P.Z_HARD_STOP_TOOLS:
        return HD.zhs_setting(tname, "rig")
    return {"tool": tname, "where": "rig", "stop_setting_s": P.Z_HARD_STOP_RAIL_END_S,
            "basis": "no clamp contact with any station at full stroke: the stop is set past TRAVEL_Z to keep the lower Z "
                     "block on its rail (lower block to rail end at this setting: "
                     f"{round(HD.zb_of(P.Z_HARD_STOP_RAIL_END_S) + HD.ZBLK_C - HD.ZBLK_DZ - HD.BLK_L / 2 - HD.RAIL_BOT, 2)} mm)",
            "firmware_depth_limit_s": P.TRAVEL_Z}


# =====================================================================================================
# fastener schedule (rev G3): from the plates' hole tables, the head's axis/cradle joint lists and the deck parts' holes
# =====================================================================================================
RIG_SPEC = {
    "ISO10642_M5x16": "ISO 10642 M5 x 16 A2-70 countersunk (BR-3 into the tapped 2040 foot core holes)",
    "ISO10642_M5x12": "ISO 10642 M5 x 12 A2-70 countersunk (BR-2 to the module carriage; length to the carriage thread: MEASURE)",
    "ISO4762_M5x16": "ISO 4762 M5 x 16 A2-70 (module end to the tapped post core holes; module hole pattern MEASURE)",
    "ISO7380_M5x10": "ISO 7380 M5 x 10 A2 (2040 corner brackets)", "TNUT_slot6_M5": "slot-6 drop-in T-nut M5",
    "corner_bracket_2040": "2040/slot-6 cast corner bracket 20 x 20 with 2 x M5 (post to its bench foot)",
    "ISO10642_M4x10": "ISO 10642 M4 x 10 A2-70 countersunk (insert / TW-1 to their plate, as on the deck)",
    "DOWEL_D4x10": "dowel pin ISO 8734 D4 m6 x 10 (pressed into the plate, as on the deck)",
    "ISO10642_M3x6": "ISO 10642 M3 x 6 A2 (TW-4D comb feet into TW-1)", "ISO4762_M3x6": "ISO 4762 M3 x 6 A2 (TW-5 hopper tabs into TW-1)",
    "ISO10642_M3x12": "ISO 10642 M3 x 12 A2 countersunk (TW-1 to the TW-2 funnel inserts)", "HEATSET_M3": "M3 brass heat-set insert (TW-2 funnel bosses)",
    "ISO4762_M6x25": "ISO 4762 M6 x 25 A2-70 (load-cell fixed ends, through BR-3)", "ISO10642_M6x25": "ISO 10642 M6 x 25 A2-70 countersunk (load-cell free ends, through BR-4/BR-5)",
    "DIN125_M6_washer": "DIN 125 M6 washer A2, 1.6 mm (two per load-cell screw = 3.2 mm bar clearance)",
    "bench_clamp_F150": "F-clamp 150 mm (each post foot to the bench)",
    # rev G4
    "ISO4762_M4x16": "ISO 4762 M4 x 16 A2-70", "ISO4762_M4x20": "ISO 4762 M4 x 20 A2-70 (LC-4 bar free end into the BR-6 lug)",
    "ISO4762_M4x25": "ISO 4762 M4 x 25 A2-70 (LC-4 bar fixed end into BR-7)", "DIN125_M4_washer": "DIN 125 M4 washer A2, 0.8 mm (spacers)",
    "ISO4026_M4x16": "ISO 4026 M4 x 16 set screw, flat point (LC-1 / LC-2 overload stops)",
    "ISO4026_M4x20": "ISO 4026 M4 x 20 set screw, flat point (LC-4 overload stop, through the BR-2 top edge)",
    "ISO4035_M4": "ISO 4035 M4 thin lock nut (overload-stop set screws)", "ISO4762_M3x8_LC3": "ISO 4762 M3 x 8 A2 (HP-5R yoke into the LC-3 back)",
    "ISO7380_M5x10": "ISO 7380 M5 x 10 A2", "STANDOFF_M3x10": "M3 x 10 nylon hex standoff F-F (MMB, HX711; NOMINAL board holes)",
    "ISO7045_M3x6": "ISO 7045 M3 x 6 A2 pan head (board standoffs)", "STANDOFF_M2.5x10": "M2.5 x 10 nylon hex standoff F-F (Pi 4)",
    "ISO7045_M2.5x6": "ISO 7045 M2.5 x 6 A2 pan head (Pi 4 standoffs)",
}


def fastener_schedule():
    """joint rows: (group, joint, fastener id, count, hole source)"""
    rows = []
    for fe in br3_feats() + br4_feats() + br5_feats() + br2_feats():
        pass
    agg = OrderedDict()
    for plate, feats in (("BR-3", br3_feats()), ("BR-4", br4_feats()), ("BR-5", br5_feats()), ("BR-2", br2_feats())):
        for kind, x, y, d, dk, joint, fid in feats:
            if fid is None:
                continue                                     # Z-axis holes: counted from the head joint list below
            agg.setdefault((plate, joint, fid), 0)
            agg[(plate, joint, fid)] += 1
    for (plate, joint, fid), n in agg.items():
        rows.append(("rig plates", joint, fid, n, f"{plate} hole table"))
    # washers: two per load-cell screw
    n_lc = sum(n for (pl, j, f), n in agg.items() if f in ("ISO4762_M6x25", "ISO10642_M6x25"))
    rows.append(("rig plates", "load-cell spacers (2 per M6)", "DIN125_M6_washer", 2 * n_lc, "LC spacer 3.2 mm"))
    # posts / feet (not plates)
    rows += [("frame", "module ends to the two posts (2 core holes per post)", "ISO4762_M5x16", 2 * len(POST_X), "POST_X x 2 core holes"),
             ("frame", "post to bench foot: 2 corner brackets per post", "corner_bracket_2040", 2 * len(POST_X), "POST_X x 2"),
             ("frame", "corner bracket screws", "ISO7380_M5x10", 4 * len(POST_X), "2 brackets x 2 screws per post"),
             ("frame", "corner bracket T-nuts", "TNUT_slot6_M5", 4 * len(POST_X), "2 brackets x 2 per post"),
             ("frame", "post feet to the bench", "bench_clamp_F150", len(POST_X), "one per post foot")]
    # rev G4: load-cell overload stops, LC-4 path, LC-3, electronics plate
    n9 = sum(n for (pl, j, f), n in agg.items() if j.startswith("BR-9"))
    rows += [("rig plates", "BR-9 spacer washers (2 per M4)", "DIN125_M4_washer", 2 * n9, "BR-3 hole table BR-9 rows"),
             ("load cells", "LC-1 / LC-2 overload-stop set screws", "ISO4026_M4x16", len(STOP9), "STOP9"),
             ("load cells", "LC-4 bar fixed end into BR-7", "ISO4762_M4x25", len(LC4["holes"]), "br7 tapped holes"),
             ("load cells", "LC-4 bar free end into the BR-6 lug", "ISO4762_M4x20", len(LC4["holes"]), "br6 lug tapped holes"),
             ("load cells", "LC-4 bar spacer washers (2 per M4)", "DIN125_M4_washer", 4 * len(LC4["holes"]), "LC4 shim 1.6"),
             ("load cells", "LC-4 overload-stop set screw (BR-2 top edge)", "ISO4026_M4x20", 1, "OVL4"),
             ("load cells", "overload-stop lock nuts", "ISO4035_M4", len(STOP9) + 1, "STOP9 + OVL4"),
             ("load cells", "EJ-1 root to BR-6 (ISO 4762 from behind, heads in free space behind BR-6)", "ISO4762_M3x8",
              len(HD.ej_root_holes(P.TOOL_NAMES.index("pipette_8ch_50"))), "head ej_root_holes"),
             ("load cells", "HP-5R yoke to the LC-3 back (one per yoke)", "ISO4762_M3x8_LC3", len(RIG_TOOLS), "hp5r M3 hole")]
    bh = {k: len(v[2]) for k, v in BR8_HOLES.items()}
    n_post = [n for k, n in bh.items() if k.startswith("post")][0]
    n_m3 = sum(n for k, n in bh.items() if k.startswith(("MMB", "HX711")))
    n_pi = [n for k, n in bh.items() if k.startswith("Raspberry")][0]
    rows += [("electronics", "BR-8 to the left post", "ISO7380_M5x10", n_post, "BR8_HOLES post"),
             ("electronics", "BR-8 post T-nuts", "TNUT_slot6_M5", n_post, "BR8_HOLES post"),
             ("electronics", "MMB + HX711 standoffs", "STANDOFF_M3x10", n_m3, "BR8_HOLES"),
             ("electronics", "MMB + HX711 standoff screws (2 per standoff)", "ISO7045_M3x6", 2 * n_m3, "BR8_HOLES"),
             ("electronics", "Pi 4 standoffs", "STANDOFF_M2.5x10", n_pi, "BR8_HOLES"),
             ("electronics", "Pi 4 standoff screws (2 per standoff)", "ISO7045_M2.5x6", 2 * n_pi, "BR8_HOLES")]
    # deck parts used on the rig: counts from their own hole lists
    n_loc = 2 * 4                                                         # two nests x 4 locators
    rows += [("stations", "LOC-1 locators to the IN-P inserts", "ISO10642_M3x8", n_loc * len(DK.LOC["m3"]), "LOC m3 x 8 locators"),
             ("stations", "TW-4D comb feet to TW-1", "ISO10642_M3x6", len(DK.TW["comb"]["feet_m3"]), "TW comb feet_m3"),
             ("stations", "TW-5 hopper tabs to TW-1", "ISO4762_M3x6", len(DK.TW["hopper"]["m3"]), "TW hopper m3"),
             ("stations", "TW-1 to TW-2 funnel", "ISO10642_M3x12", len(DK.TW["screws"]), "TW screws"),
             ("stations", "TW-2 funnel inserts", "HEATSET_M3", len(DK.TW["screws"]), "TW screws (funnel bosses)")]
    # the Z axis (machine joint list for one stop axis; EJ-1 rows replaced by the three fingers in br2_feats above)
    i8 = P.TOOL_NAMES.index("pipette_8ch_300")
    for j in HD.axis_joints(i8, with_stop=True):
        if j["fastener"] and not j["joint"].startswith("EJ-1"):
            rows.append(("Z axis", j["joint"], j["fastener"], j["count"], j["source"]))
        elif not j["fastener"]:
            rows.append(("Z axis", j["joint"], None, 0, j["source"]))
    for tn in RIG_TOOLS:
        for j in HD.cradle_joints(tn):
            rows.append((f"cradle {tn}", j["joint"], j["fastener"], j["count"], j["source"]))
    return rows


def plate_hole_check(name, solid_flat, feats):
    """count the circular through holes in the plate's bottom face (STEP geometry) vs its hole table"""
    f = sorted([fc for fc in solid_flat.Faces() if fc.geomType() == "PLANE"], key=lambda fc: fc.Center().z)[0]
    n = 0
    for w in f.innerWires():
        es = w.Edges()
        if all(e.geomType() == "CIRCLE" for e in es):
            n += 1
    return {"plate": name, "holes_in_model_bottom_face": n, "holes_in_table": len(feats), "match": n == len(feats)}


def relayer_dxf(path: Path, note: str) -> None:
    """layers: TAPPED (2.5 / 3.3 tap drills), REAM (D4 dowel holes: cut 3.8, ream D4 H7), CUT (everything else) + NOTE"""
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
            elif abs(d - 4.0) < 0.05:
                e.dxf.radius = 1.9
                e.dxf.layer = "REAM"
    msp.add_text(note, height=3.0, dxfattribs={"layer": "NOTE"}).set_placement((0, -30))
    doc.saveas(str(path))


def main():
    OUT.mkdir(parents=True, exist_ok=True)
    rep = {"status": "bench rig concept for first measurements; NOT built; envelopes for bought rig items",
           "revision": "mechanical G5 (13 Nov 2026): G4 + 48 mm NEMA17 Z motor; wiring statements agree with bench_rig.cfg / wiring table",
           "eject_methods": {"A": "TW-4D comb (strip by lifting under the comb), LC-2 measures strip force",
                             "B": "EJ-1 finger on BR-2 presses the pipette's own ejector when Z goes "
                                  f"{P.Z_EJECT_TRAVEL:g} mm above the normal top; the finger is carried by LC-4 (BR-6 carrier, rev G4)",
                             "decision": "run both for >= 1000 tips per pipette: success rate (tip-check gate), peak force, "
                                         "tips missing the funnel; choose per pipette"},
           "purpose": ["plunger force vs travel per pipette (LC-3 in the HP-5 yoke + actuator steps)",
                       "gravimetric dispense 0.5, 1, 2, 5, 10, 50, 300 uL (ISO 8655-6 method; 6-place balance below 10 uL)",
                       "one-tip and eight-tip pick-up force (LC-1 under the rack) and strip force (LC-2 under the comb)",
                       "thousands of unattended pick-up/strip cycles: comb mode with a bin"],
           "stations_nozzle_x_mm": STATIONS, "stations_nozzle_x_per_tool_mm": {t: station_x(t) for t in RIG_TOOLS},
           "checks": {}, "per_tool": {}}
    static = rig_static()
    asm = cq.Assembly(name="BR1")
    for tname in RIG_TOOLS:
        i = P.TOOL_NAMES.index(tname)
        st, (xn, yn) = stations(i)
        stop = stop_setting(tname)
        s_set = stop["stop_setting_s"]
        s_end = min(s_set, P.TRAVEL_Z)
        res = {"bench_y_of_nest_line": round(yn, 2), "overlaps_unconstrained_mm3": [], "overlaps_mm3": [], "station_reach": {}}
        fixed = static + st
        for sname, sx in station_x(tname).items():
            xc = sx - (xn - HD.AXES_X[i])                       # axis x so the nozzle is over the station
            res["station_reach"][sname] = 0.0 <= sx <= MODULE["stroke"]
            for s in (-EJ, 0.0, 50.0, 100.0, 150.0, s_end, P.TRAVEL_Z):
                if s == P.TRAVEL_Z and s_end >= P.TRAVEL_Z and s != s_end:
                    continue
                mov = axis_parts(i, s, xc) + [("BR-2", "BR-2_z_mount_plate", "custom", mount_plate(xc)),
                                              ("carriage", "belt_linear_module_500", "bought", carriage(xc))] + rig_stop(xc, s_set) + lc4_items(xc)
                hits = overlaps(mov, fixed, sname, s)
                if s > s_end + 1e-9:
                    res["overlaps_unconstrained_mm3"] += hits     # beyond the stop: the axis cannot get here (G2 R4 figures)
                else:
                    res["overlaps_mm3"] += hits
        res["n_real"] = sum(1 for o in res["overlaps_mm3"] if o["class"] == "real")
        # first contact of every real overlap seen at full stroke (unconstrained), by bisection
        first = {}
        for o in [o for o in res["overlaps_unconstrained_mm3"] if o["class"] == "real"] + \
                 [o for o in res["overlaps_mm3"] if o["class"] == "real" and o["s"] >= P.TRAVEL_Z]:
            sx = station_x(tname)[o["station"]]
            xc = sx - (xn - HD.AXES_X[i])
            fs = [f for f in fixed if f[0] == o["fixed"]][0][3]
            lo, hi = 0.0, P.TRAVEL_Z
            while hi - lo > 0.05:
                mid = (lo + hi) / 2
                ms = [m_ for m_ in axis_parts(i, mid, xc) if m_[0] == o["moving"]][0][3]
                lo, hi = (lo, mid) if vol_common(ms, fs) > 1e-3 else (mid, hi)
            k = f"{o['moving']} x {o['fixed']} @ {o['station']}"
            first[k] = round(lo, 2)
        res["first_contact_s_unconstrained"] = first
        # R4 at the hard stop: overlap and clearance of every non-nozzle tool part to the tip rack
        xc_r = station_x(tname)["tip_rack"] - (xn - HD.AXES_X[i])
        rack_s = [f for f in fixed if f[0] == "tip_rack"][0][3]
        parts_at = [m_ for m_ in axis_parts(i, s_end, xc_r) if "@" in m_[0] and not m_[0].startswith(("pipette@", "ejb"))]
        dmin, who = 1e9, None
        for n_, _, _, sh in parts_at:
            d = sh.distance(rack_s)
            if d < dmin:
                dmin, who = d, n_
        r4 = {"stop_setting_s": s_set, "axis_end_s": s_end,
              "overlap_mm3_non_nozzle_parts_vs_tip_rack_at_stop": round(sum(vol_common(sh, rack_s) for *_, sh in parts_at), 3),
              "min_clearance_mm_at_stop": round(dmin, 2), "nearest_part": who}
        if tname in P.Z_HARD_STOP_TOOLS:
            r4.update({"governing_part (over the rack within the lateral band)": stop["part"],
                       "vertical_gap_of_governing_part_mm": stop["clearance_at_stop_nominal_mm"],
                       "clearance_after_worst_case_tolerance_mm": stop["clearance_after_worst_case_mm"],
                       "clearance_after_rss_tolerance_mm": stop["clearance_after_rss_mm"],
                       "note": "min_clearance_mm_at_stop is the 3-D distance at the nominal BR-3 Y setting; the tolerance stack is "
                               "vertical and is applied to the governing part's vertical gap",
                       "unconstrained_overlap_at_200_mm3 (G2 R4 figure)": sum(o["mm3"] for o in res["overlaps_unconstrained_mm3"]
                                                                              if o["class"] == "real" and o["fixed"] == "tip_rack")})
            res["hard_stop_z_stroke_max"] = {"tip_rack": {"s_max": s_set, "working_end_z_min": round(P.PIPETTE_NOZZLE_Z_AT_TOP - s_set, 1),
                                                          "rule": "ZHS-1 MECHANICAL stop setting (rev G3); firmware depth limit "
                                                                  f"<= s_max - {P.Z_HARD_STOP_FIRMWARE_MARGIN:g}"}}
        res["z_hard_stop"] = stop
        res["R4_at_hard_stop"] = r4
        res["note"] = ("bare nozzles: they enter the tip collars / the comb slots / the wells by design at the working stroke; "
                       "overlaps of nozzle/tip with rack, comb or plate at the deepest strokes are the working contacts")
        rep["per_tool"][tname] = res
        if tname == "pipette_8ch_300":
            xc0 = STATIONS["tip_rack"] - (xn - HD.AXES_X[i])
            for n, _, _, s in static + st + axis_parts(i, 0.0, xc0) + rig_stop(xc0, s_set) + lc4_items(xc0) + \
                    [("BR-2", "", "", mount_plate(xc0)), ("carriage", "", "", carriage(xc0))]:
                asm.add(s, name=n)
    asm.save(str(OUT / "br1_assembly.step"))
    # ---- rig plates: STEP (with countersinks) + DXF (through-hole outline, bottom face) ----
    br8f = lambda: [("thru", x, y, d, 0, k, None) for k, (kk, d, pts) in BR8_HOLES.items() for x, y in pts]   # noqa: E731
    br9f = lambda: [("thru", x, y, 4.5, 0, "BR-9 to BR-3", None) for x, y in BR9["clr"]] + [("tap", *BR9["tap"], 3.3, 0, "stop screw", None)]   # noqa: E731
    plates = {"BR-2_z_mount_plate": (br2_flat, br2_feats, "Al 6061-T6 6 mm", BR2["t"], "cnc"),
              "BR-3_base_plate": (br3_flat, br3_feats, "Al 6082-T6 12 mm cast/milled plate", BR3["t"], RIG_PLATE_PROC),
              "BR-4_rack_nest_carrier": (br4_flat, br4_feats, "Al 6082-T6 12 mm", CAR_T, RIG_PLATE_PROC),
              "BR-5_comb_station_carrier": (br5_flat, br5_feats, "Al 6082-T6 12 mm", CAR_T, RIG_PLATE_PROC),
              "BR-8_electronics_plate": (br8_flat, br8f, "Al 5052-H32 3 mm", BR8["t"], "sheet_laser"),
              "BR-9_lc_overload_stop": (br9_flat, br9f, "Al 5052-H32 6 mm", BR9["t"], "sheet_laser")}
    rep["parts"], rep["checks"]["plate_hole_counts"] = [], []
    for name, (fn, ff, mat, t, proc) in plates.items():
        sol = fn()
        cq.exporters.export(cq.Workplane().add(sol), str(OUT / f"{name}.step"))
        cq.exporters.export(cq.Workplane().add(fn(csk=False)).faces("<Z"), str(OUT / f"{name}.dxf"))
        kinds = {}
        for f_ in ff():
            kinds[f_[0]] = kinds.get(f_[0], 0) + 1
        relayer_dxf(OUT / f"{name}.dxf", f"{name}: {mat}; holes {kinds}; csk = 90 deg countersink on the face stated in the "
                                         "report hole table; TAPPED: drill + tap; REAM: cut 3.8, ream D4 H7")
        bb = sol.BoundingBox()
        rep["parts"].append({"name": name, "process": proc, "material": mat, "thickness": t, "qty": 2 if name.startswith("BR-9") else 1,
                             "envelope_mm": [round(bb.xlen, 1), round(bb.ylen, 1), round(bb.zlen, 1)], "valid_solid": sol.isValid(),
                             "hole_table": [{"kind": k, "x": round(x, 2), "y": round(y, 2), "d": d, "csk_d": dk or None, "joint": j,
                                             "fastener": fid} for k, x, y, d, dk, j, fid in ff()]})
        rep["checks"]["plate_hole_counts"].append(plate_hole_check(name, sol, ff()))
    if getattr(P, "REV_J", False):     # rev J: laser-cut rig plates (secondary ops) and gauges
        sec = {}
        for name in ("BR-3_base_plate", "BR-4_rack_nest_carrier", "BR-5_comb_station_carrier", "BR-8_electronics_plate"):
            fn, ff, mat, t, proc = plates[name]
            k = {}
            for f_ in ff():
                key = {"tap": f"tapped M{3 if f_[3] < 3.0 else 4 if f_[3] < 4.0 else 5}" if f_[0] == "tap" else None,
                       "csk": "countersunk (90 deg)", "ream": "reamed D4 H7 (laser pilot 3.8)", "thru": "plain through"}.get(f_[0]) or f_[0]
                k[key] = k.get(key, 0) + 1
            sec[name] = {"process": proc, "thickness_mm": t, "material": mat, "holes": k}
        rep["checks"]["rig_plates_rev_j"] = {
            "plates": sec, "kept_machined": ["BR-2 (Z mount: CP-1 hole set, tight hole positions)", "BR-6 / BR-7 (LC-4 eject force path)",
                                             "LOC-1 locators", "HP-5R yokes"],
            "secondary_ops": "tap, countersink and ream after laser cutting (supplier option if offered; otherwise drill press + hand tap "
                             "+ D4 H7 machine reamer in the laser pilots). NOT CHECKED: that the sheet supplier laser-cuts 12 mm aluminium "
                             "(common limit 10-15 mm) and taps it"}
        for gname, fn in (("GG-4_br3_y_index_bar", gg4_bar), ("GG-5_br3_y_spacer", gg5_spacer)):
            sol = fn()
            cq.exporters.export(cq.Workplane().add(sol), str(OUT / f"{gname}.step"))
            cq.exporters.export(cq.Workplane().add(sol), str(OUT / f"{gname}.stl"), tolerance=0.02, angularTolerance=0.2)
        lc_t = {"BR-3 / carrier plate thickness (12 mm sheet, EN 485-4 class)": 0.25, "load-cell bar height (NOMINAL, measure)": 0.1,
                "spacer washers DIN 125 (1.6 / 3.2 stack)": 0.2, "BR-9 6 mm sheet thickness": 0.2, "fixed stop pin length (machined)": 0.02}
        wc, rss = sum(lc_t.values()), math.sqrt(sum(v * v for v in lc_t.values()))
        rep["checks"]["overload_stops_rev_j"] = {
            "question": "can the 0.2 mm overload-stop gaps be fixed-height (no adjustment)?",
            "stack_mm (+/-)": lc_t, "worst_case_mm": round(wc, 2), "rss_mm": round(rss, 2),
            "allowed_band_mm": "0.1..0.3 (above the working deflection, below the cell's safe overload deflection: NOMINAL)",
            "answer": f"no: the stack (worst {wc:.2f}, RSS {rss:.2f}) is 3-8 x the +/-0.1 band, even with machined spacers",
            "replacement": "keep the M4 set screw, set it by instrument: screw in until the HX711 reading moves by 1 N (contact), back off two "
                           "GG-3 notches (120 deg = 0.23 mm on M4 x 0.7), lock the nut, then confirm the reading returns to its zero",
            "gauge": "GG-3_overload_stop_index"}
        rep["checks"]["br3_y_index_rev_j"] = {
            "foot_back_face_y_by_tool": {t: round(br3_foot_back_y(t), 2) for t in RIG_TOOLS},
            "GG-5 steps_mm": [round(br3_foot_back_y("pipette_small") - br3_foot_back_y(t), 2) for t in RIG_TOOLS if t != "pipette_small"],
            "print_tolerance_mm": P.MJF_PRINT_TOL, "effect": "nest line within +/-0.2 mm of the nozzle row in Y (the rack pitch tolerance is larger)"}
    # ---- rev G4: machined rig blocks (STEP only) ----
    blocks = {"BR-6_ej_carrier_lc4": br6_carrier(), "BR-7_lc4_standoff": br7_standoff()}
    for tn in RIG_TOOLS:
        blocks[f"HP-5R_plunger_yoke_lc3_{HD.HP_TAG[tn]}"] = hp5r(tn)
    for name, sol in blocks.items():
        cq.exporters.export(cq.Workplane().add(sol), str(OUT / f"{name}.step"))
        bb = sol.BoundingBox()
        rep["parts"].append({"name": name, "process": "cnc", "material": "Al 6061-T6", "thickness": None, "qty": 1,
                             "envelope_mm": [round(bb.xlen, 1), round(bb.ylen, 1), round(bb.zlen, 1)], "valid_solid": sol.isValid()})
    # ---- rev G4: force paths through the load cells (gaps that must stay open, stops that must be set) ----
    i8 = P.TOOL_NAMES.index("pipette_8ch_300")
    xc8 = STATIONS["eject_B"] - (HD.AXES_X[i8] + HD.hp_geom("pipette_8ch_300")["hx"] - HD.AXES_X[i8])
    li = {n: sh for n, _, _, sh in lc4_items(xc8)}
    br2s = mount_plate(xc8)
    ej = [sh for n, _, _, sh in axis_parts(i8, 0.0, xc8) if n.startswith("EJ")][0]
    st8_, (xn8, yn8) = stations(i8)
    sd = {n: sh for n, _, _, sh in st8_}
    fp = {"LC-4 (method-B eject)": {
              "path": "EJ-1 finger -> 2 x M3 -> BR-6 carrier (in a BR-2 window) -> lug -> 2 x M4 -> LC-4 bar free end -> bar -> fixed "
                      "end -> 2 x M4 -> BR-7 -> 2 x M4 -> BR-2",
              "BR-6_to_BR-2_min_gap_mm (must be > 0: carrier floats)": round(li["BR-6"].distance(br2s), 2),
              "LC-4_bar_to_BR-2_gap_mm": round(li["LC-4"].distance(br2s), 2),
              "EJ-1_root_on_BR-6 (contact 0)": round(ej.distance(li["BR-6"]), 3),
              "EJ-1_to_BR-2_gap_mm": round(ej.distance(br2s), 2),
              "overload_screw_gap_to_carrier_mm": round(li["ovl4"].distance(li["BR-6"]), 2),
              "rated_N": 200, "safe_overload_N (150 % ASSUMED)": 300, "working_N (EJECTOR force_N)": max(e["force_N"] for e in P.EJECTOR.values()),
              "stop_engages_at_N (0.2 mm of the ASSUMED 0.3 mm rated deflection)": round(200 * LC_STOP_GAP / 0.3, 0),
              "stall_case": "an upward Z stall at the eject top (head report z_screw_axial_support_rev_g4: about 754 N at 1.3 A) ends on the overload screw, not the cell"},
          "LC-1 (pick-up)": {"path": "tips -> rack -> IN-P -> BR-4 -> 2 x M6 -> LC-1 free end -> fixed end -> 2 x M6 -> BR-3",
                             "BR-4_to_BR-3_gap_mm": round(sd["BR-4_rack_carrier"].distance(sd["BR-3_base"]), 2),
                             "overload_stop_gap_mm": round(sd["stop_LC-1"].distance(sd["BR-4_rack_carrier"]), 2),
                             "rated_N": 500, "working_N": max(P.PICKUP_FORCE_N.values()),
                             "stop_engages_at_N (ASSUMED 0.3 mm at rated)": round(500 * LC_STOP_GAP / 0.3, 0),
                             "stall_case": "a downward Z stall onto the rack (about 767 N at 1.3 A) ends on the BR-9 stop"},
          "LC-2 (strip)": {"path": "tips -> TW-4D comb -> TW-1 -> BR-5 -> 2 x M6 -> LC-2 free end -> fixed end -> 2 x M6 -> BR-3",
                           "BR-5_to_BR-3_gap_mm": round(sd["BR-5_comb_carrier"].distance(sd["BR-3_base"]), 2),
                           "overload_stop_gap_mm": round(sd["stop_LC-2"].distance(sd["BR-5_comb_carrier"]), 2),
                           "rated_N": 500, "working_N": max(P.STRIP_FORCE_N.values()),
                           "stop_engages_at_N (ASSUMED 0.3 mm at rated)": round(500 * LC_STOP_GAP / 0.3, 0),
                           "stall_case": "an upward Z stall with tips caught under the comb ends on the BR-9 stop"},
          "LC-3 (plunger)": {"path": "LGA35 actuator rod (V6; was NEMA11) -> HP-5R yoke -> LC-3 button cell (in the yoke pocket, M3 into its back) -> "
                                     "pipette plunger button",
                             "cell_button_to_plunger_gap_at_rest_mm": 0.5, "rated_N": 200, "working_N": max(g["force"] for g in HD.HP_SETS.values()),
                             "actuator_stall_N (holding 0.06 N m ASSUMED, eta 0.35, lead 1 mm)": round(2 * math.pi * 0.06 * 0.35 / 0.001, 0),
                             "overload": "no mechanical stop: the LGA35 at 0.7 A (about 140 N at 2 mm/s, INFERENCE) stays inside the 200 N rating; "
                                         "CT-0d (ADR-014) needs the 120 N pass on this cell: 60 % of its range (integration 23 Nov 2026: was a "
                                         "100 N cell, which could not show 120 N); for CT-0d a die spring (about 12.5 N/mm) goes in series "
                                         "between the actuator rod and the yoke"}}
    rep["checks"]["force_paths_rev_g4"] = fp
    # ---- rev G4: plain numbers for firmware/lh1/bench_rig.cfg ----
    fw = {"note": "stroke s in mm down from the normal top; x in mm along the rig X axis (nozzle over the station)",
          "per_pipette": {}, "station_x": {t: station_x(t) for t in RIG_TOOLS},
          "z_run_current_A": "same window as the machine Z axes (head report checks.z_screw_axial_support_rev_g4.run_current)"}
    for tn in RIG_TOOLS:
        stp = rep["per_tool"][tn]["z_hard_stop"]
        strip_nom = P.PIPETTE_NOZZLE_Z_AT_TOP - DK.TW["comb"]["moves"][tn]["z"]
        tol = sum(P.Z_STRIP_STROKE_TOL["rig"].values())
        lim = stp["firmware_depth_limit_s"]
        fw["per_pipette"][tn] = {"zhs1_stop_s": stp["stop_setting_s"], "depth_max": lim,
                                 "comb_strip_stroke_nominal_s": round(strip_nom, 2), "comb_strip_stroke_worst_s": round(strip_nom + tol, 2),
                                 "limit_minus_worst_strip_mm": round(lim - strip_nom - tol, 2),
                                 "strip_fits": lim - strip_nom - tol >= 0.25}
    fw["rig_z_position_max"] = max(v["depth_max"] for v in fw["per_pipette"].values())
    fw["rig_z_position_max_note"] = ("axis limit = the small pipette's; the mounted eight-channel's depth_max is the binding limit "
                                     "(_BR depth_max_<pipette>); electronics/lh1/check.py also compares position_max with the lowest stop - 1")
    rep["firmware_numbers_rev_g4"] = fw
    # ---- static rig parts checked against each other (8ch_300 layout) ----
    st8, _ = stations(P.TOOL_NAMES.index("pipette_8ch_300"))
    allst = static + st8
    sp = []
    for a in range(len(allst)):
        for b in range(a + 1, len(allst)):
            ba, bb_ = allst[a][3].BoundingBox(), allst[b][3].BoundingBox()
            if ba.xmax < bb_.xmin or bb_.xmax < ba.xmin or ba.ymax < bb_.ymin or bb_.ymax < ba.ymin or ba.zmax < bb_.zmin or bb_.zmax < ba.zmin:
                continue
            v = vol_common(allst[a][3], allst[b][3])
            if v > 1.0:
                sp.append({"a": allst[a][0], "b": allst[b][0], "mm3": round(v, 1)})
    rep["checks"]["rig_static_overlaps_mm3"] = sp
    rep["layout_rev_g3"] = {
        "BR-3": {"size_mm": [BR3["x"][1] - BR3["x"][0], BR3["y"][1] - BR3["y"][0], BR3["t"]],
                 "why": "G2 model 475 x 180 was too short for the plate-nest insert at x 320 (insert to x 385, BR-3 to 355) and the "
                        "parts-list text 760 x 250 was stale (G1): G3 moves the plate nest to x 290 and the balance to x 480 (clear of "
                        "BR-3 by 10 mm), keeps the rack at 0 and the comb nest at 151; BR-3 = x -85..360, 87.5 in front of (feet clear "
                        "of the windows) and 172.5 behind the nest line (load-cell fixed ends)",
                 "y_setting": "BR-3 stands on its four feet and is slid in Y so the nest line sits under the mounted pipette's nozzle "
                              f"row: rig y {', '.join(f'{t} {HD.Y_TF + HD.hp_geom(t)['y']:.1f}' for t in RIG_TOOLS)} (no Y axis)"},
        "load_cells": {"LC-1": "single-point bar under the rack nest: fixed end to BR-3, free end carries BR-4 + IN-P",
                       "LC-2": "rev G3: single-point PLATFORM cell (was S-type: a single S-type hanger takes the off-centre comb "
                               "load as a moment): fixed end to BR-3, free end carries BR-5 + TW-1 + TW-4D + TW-2 + TW-3",
                       "spec": LC["spec"]},
        "module": {"carriage_holes_rel_rail": CARR_HOLES, "note": "carriage envelope 90 x 90 moved +15 mm in x so the EJ-1 root screws "
                                                                  "(from behind BR-2) stay accessible; module hole pattern MEASURE"},
        "posts": {"length_mm": POST_LEN, "foot": f"2040 x {POST_FOOT_L:.0f} along Y under each post, 2 corner brackets, F-clamped"}}
    # ---- parts list (rev G3): every row with kind, process, material/spec, qty; fasteners from the joint schedule ----
    lh1 = {}
    bp = ROOT / "bom" / "lh1_bom.csv"
    if bp.exists():
        with open(bp) as fh:
            for r in csv.DictReader(fh):
                lh1[r["part_id"]] = r
    deck = {p_["name"]: p_ for p_ in json.loads((LH1 / "deck_modules" / "out" / "report.json").read_text())["parts"]}
    machine = {}
    for tname in RIG_TOOLS:
        i = P.TOOL_NAMES.index(tname)
        for n, part, kind, _ in axis_parts(i, 0.0, 0.0):
            if n.startswith(("pipette", "ejb")):
                continue
            if part == "MGN12H_block":
                machine[part] = 2                            # rev G2: two blocks on the rig's Z rail, as on the machine
                continue
            q = 1 if (n.startswith(("HP", "CB", "hp_", "f_sw", "pl_sw", "EJ")) or tname == "pipette_small") else 0
            machine[part] = max(machine.get(part, 0), 0) + q
    machine.update({"IN-P_plain_insert_plate": 2, "LOC-1_corner_locator": 8, "TW-1_tip_waste_insert_plate": 1, "TW-4D_strip_comb": 1,
                    "TW-2_tip_funnel": 1, "TW-3_tip_bin": 1, "TW-5_eject_hopper": 1, "MMB_CAN_board": 1,
                    "ZHS-1_z_hard_stop": 1, "thrust_bearing_F6-14M": 2, "MR126ZZ": 1, "ZBH_backing_washer_6.5x14x2": 1})   # rev G4

    def spec_of(part):
        if part.startswith("HP-5R"):
            return "custom", "cnc", "Al 6061-T6, 10 mm yoke with the LC-3 pocket (rig only; replaces HP-5 on the rig) (STEP)"
        if part == "load_cell_200N_button":
            return "bought", "bought", ("LC-3 button compression load cell 200 N (20 kg) class, body D10 x 5 + 1 mm load button, M3 tapped back "
                                        "(NOMINAL class, UNVERIFIED part: confirm the body fits the D10.5 x 5.5 yoke pocket before ordering; "
                                        "safe overload 150 % ASSUMED); the same cell serves CT-0d (120 N pass) and CT-9 (plunger force)")
        if part in HD.CUSTOM:
            c = HD.CUSTOM[part]
            return "custom", c[0], c[1] + (f" {c[2]:g} mm" if c[2] else "")
        if part in HD.BOUGHT:
            return "bought", "bought", HD.BOUGHT[part][0]
        if part in deck:
            return "custom", deck[part].get("process", ""), str(deck[part].get("material", ""))
        return "", "", ""
    rows_ = []
    for part, q in sorted(machine.items()):
        bid = next((c for c in (part, "B-" + part, "REF-" + part) if c in lh1), None)   # REF-: listed for reference, ordered elsewhere
        k, pr, sp_ = spec_of(part)
        if part == "MMB_CAN_board":
            sp_ = HD.BOUGHT[part][0] + "; machine units come from electronics/lh1/electrical_bom.csv"
        if part in ("D2F_switch",):
            sp_ = ("D2F-class subminiature switch with lever: plunger 'released' switch on " + ("the CB-1 shelf" if getattr(P, "REV_J", False) else "HP-6") +
                   ", one per cradle (NOT the force switch)")
        if part in ("D2F_roller_lever",):
            sp_ = HD.BOUGHT[part][0] + "; 2 x M2x8 on the " + ("ZSF-1" if getattr(P, "REV_J", False) else "ZSW-1") + " arm"
        if part.startswith("HP-5R") or part == "load_cell_200N_button":
            rows_.append([part, k, pr, sp_, q, "no", "rig only (rev G4)"])
            continue
        note = bid or ("machine part (G3/G4 id; bom/lh1_bom.csv not yet regenerated)" if part.startswith(("ZHS-1", "MR126", "ZBH_backing", "thrust_bearing_F6", "NEMA17_48", "screw_end_locknut"))
                       else "machine part counted in the electrical BOM (head qty 0)" if part in HD.BOUGHT and HD.BOUGHT[part][2] == 0
                       else "not in bom/lh1_bom.csv")
        rows_.append([part, k or (lh1[bid]["kind"] if bid else ""), pr or (lh1[bid]["process"] if bid else ""), sp_, q,
                      "yes" if (bid or part.startswith(("ZHS-1", "MR126", "ZBH_backing", "thrust_bearing_F6", "NEMA17_48", "screw_end_locknut"))) else "no", note])
    rig_only = [
        ("BR-2_z_mount_plate", "custom", "cnc", f"Al 6061-T6 6 mm, {BR2['x'][1] - BR2['x'][0]:.0f} x {BR2['z'][1] - BR2['z'][0]:.0f}: CP-1 Z-axis "
                                                "hole set + ZHS-1 + 3 EJ-1 fingers + 4 carriage csk (STEP + DXF)", 1),
        ("BR-3_base_plate", "custom", RIG_PLATE_PROC, f"Al 6082-T6 12 mm, {BR3['x'][1] - BR3['x'][0]:.0f} x {BR3['y'][1] - BR3['y'][0]:.0f}: windows for "
                                             "BR-4/BR-5, plate-nest pattern, feet and load-cell holes (STEP + DXF)" + RIG_PLATE_NOTE, 1),
        ("BR-4_rack_nest_carrier", "custom", RIG_PLATE_PROC, "Al 6082-T6 12 mm, 130 x 108: IN-P pattern (M4 + D4) on the LC-1 free end (STEP + DXF)"
                                                             + RIG_PLATE_NOTE, 1),
        ("BR-5_comb_station_carrier", "custom", RIG_PLATE_PROC, "Al 6082-T6 12 mm, 130 x 126, 120 x 80 opening: TW-1 pattern on the LC-2 free end "
                                                       "(STEP + DXF)" + RIG_PLATE_NOTE, 1),
        ("belt_linear_module_500", "bought", "bought", "500 mm stroke, belt, NEMA17, guided, carriage >= 80 x 80 with 4 x M5 "
                                                       "(BR-2 drilled 60 x 70: confirm), NC home switch", 1),
        (f"2040_post_{POST_LEN:.0f}", "bought", "extrusion_cut", f"2040 slot 6, {POST_LEN:.0f} mm, both core holes tapped M5 at the top end "
                                                                   "(post foot to module)", 2),
        (f"2040_post_foot_{POST_FOOT_L:.0f}", "bought", "extrusion_cut", f"2040 slot 6, {POST_FOOT_L:.0f} mm, laid along Y under each post", 2),
        ("2040_foot_138", "bought", "extrusion_cut", "2040 slot 6, 138 mm, both core holes tapped M5 at the top end (BR-3 feet)", 4),
        ("load_cell_500N_platform", "bought", "bought", "LC-1 (rack nest) and LC-2 (comb station, rev G3: was S-type): " + LC["spec"], 2),
        ("BR-6_ej_carrier_lc4", "custom", "cnc", "Al 6061-T6: EJ-1 carrier in the BR-2 window + lug on the LC-4 free end (STEP)", 1),
        ("BR-7_lc4_standoff", "custom", "cnc", "Al 6061-T6 26 x 13 x 20: LC-4 fixed end on BR-2 (STEP)", 1),
        ("BR-8_electronics_plate", "custom", "sheet_laser", "Al 5052-H32 3 mm, 230 x 200, on the left post: MMB, Pi 4, 4 x HX711 hole "
                                                            "patterns NOMINAL (STEP + DXF)", 1),
        ("BR-9_lc_overload_stop", "custom", "sheet_laser", "Al 5052-H32 6 mm, 40 x 30: LC-1 and LC-2 overload-stop brackets (STEP + DXF)", 2),
        ("load_cell_200N_bar", "bought", "bought", "LC-4 method-B ejection force (rev G4: was S-type 200 N): " + LC4["spec"], 1),
        ("HX711_amplifier", "bought", "bought", "24-bit, one per load cell", 4),
        ("raspberry_pi_4", "bought", "bought", "4 GB rig host", 1),
        ("balance_6place", "bought", "bought", "0.001 mg (ISO 8655-6 below 10 uL); builder's own if available", 1),
        ("evaporation_trap_vial", "bought", "bought", "ISO 8655-6 trap", 2),
        ("PSU_24V_150W", "bought", "bought", "rig supply (electrical BOM: 24 V desktop supply)", 1),
        ("FFC_ZIF_adapter_10p_1mm", "bought", "bought", "10-pin 1.0 mm pitch FFC ZIF-to-wire adapter board, one at each end of the "
                                                         "Z-axis FFC loop (wiring BR-M-02, BR-S-03, BR-S-04)", 2),
        ("hand_pipette_small / 8ch50 / 8ch300", "bought", "bought", "the builder's own pipettes (also used on LH-1, not in its parts list)", 3)]
    if getattr(P, "REV_J", False):     # rev J: gauges (GG-1..GG-3 shared with LH-1 setting; GG-4/GG-5 rig only)
        rig_only += [("GG-4_br3_y_index_bar", "custom", "print_mjf_pa12", "PA12: BR-3 Y index bar on the left post foot (2 x M5 T-nut) (STEP + STL)", 1),
                     ("GG-5_br3_y_spacer", "custom", "print_mjf_pa12", "PA12: two-step spacer 20.0 / 22.5 for the eight-channel BR-3 positions (STEP + STL)", 1)]
        for gname, (fn, proves, _) in HD.GAUGES.items():
            rows_.append([gname, "custom", "print_mjf_pa12", "PA12 setting gauge: " + proves, 1, "yes", "rev J gauge, also used on LH-1"])
    for n, k, pr, m, q in rig_only:
        rows_.append([n, k, pr, m, q, "no", "rig only"])
    if getattr(P, "REV_L", False) and (HERE.parent / "head_l" / "out" / "report.json").exists():   # rev L: CT-1..CT-9 rig parts
        _hl = json.loads((HERE.parent / "head_l" / "out" / "report.json").read_text())
        rep["rev_l_additions"] = {"tests": _hl["rig"]["tests"], "parts": "cad/lh1/head_l/out (RCB-L, TPL-L, CRA-L, DB-L2, BC-L, RP-L, "
                                  "DT-L, NRP-L, IDH-L, LJF-L, PLF-L, SHM-L, SDM-L, SCB-L STEP; two-bay dock rig_l_two_bay_dock.step)",
                                  "status": "designed as parts and fixtures (revision L; latch LTG-1 and dock lift 23 Nov 2026); nothing built or measured",
                                  "first_tests (in this order, before any head, coupling or dock part is bought for the machine)": [k for k in _hl["rig"]["tests"] if k.startswith("FIRST")],
                                  "why_first": "they decide the design: latch release and re-latch force (V1), coupling repeatability over repeated exchanges (CT-1), comb entry with real tips (V4/V5), pusher force at 0.7 A (V6)",
                                  "mounting": "RCB-L bolts to the rig Z carriage's TP-1 pattern in place of a cradle; the two-bay dock "
                                              "base bolts to the rig base beside the tip-rack station; SDM-L bolts to the same TP-1 pattern"}
        for r_ in _hl["rig"]["parts"]:
            rows_.append([r_["part"], r_["kind"], r_["process"], r_["spec"], r_["qty"], "no", "rev L rig (" + r_["tests"] + ")"])
    sched = fastener_schedule()
    fq = OrderedDict()
    for grp, joint, fid, n, src in sched:
        if fid:
            fq[fid] = fq.get(fid, 0) + n
    fspec = {**HD.FASTENER_SPEC, **{k: HD.BOUGHT[k][0] for k in ("ZHS_set_screw_M5x30_locknut", "ZHS_ISO4762_M4x10")}, **RIG_SPEC}
    for fid, n in fq.items():
        kind = "bought"
        rows_.append([fid, kind, "bought", fspec[fid], n, "no", "fastener (rev G3 schedule: report fastener_schedule)"])
    with open(OUT / "BR1_parts_list.csv", "w", newline="") as fh:
        w = csv.writer(fh)
        w.writerow(["part", "kind", "process", "material_or_spec", "qty", "shared_with_lh1", "lh1_bom_part_id_or_note"])
        w.writerows(rows_)
    rep["fastener_schedule"] = [{"group": g, "joint": j, "fastener": f, "count": n, "source": s} for g, j, f, n, s in sched]
    rep["parts_list"] = {"file": "out/BR1_parts_list.csv",
                         "shared_lines": sum(1 for r in rows_ if r[5] == "yes"), "shared_pieces": sum(r[4] for r in rows_ if r[5] == "yes"),
                         "not_recovered_lines": sum(1 for r in rows_ if r[5] == "no"),
                         "not_recovered_pieces": sum(r[4] for r in rows_ if r[5] == "no"),
                         "rows_missing_kind_process_or_spec": [r[0] for r in rows_ if not (r[1] and r[2] and r[3])],
                         "duplicates_resolved": {"D2F_switch": "plunger-released switch on HP-6 (3, one per cradle): a different "
                                                                "function from D2F_force_switch on HP-1 (3); both are needed",
                                                 "D2F_roller_lever": "the Z-up switch on the ZSW-1 arm (1)",
                                                 "MMB_CAN_board": "controller (machine units in the electrical BOM)"}}
    rep["open_items"] = [
        "bought-part hole patterns are NOMINAL (module carriage and ends, load-cell ends, boards on BR-8): measure before machining",
        "load-cell deflections at rated load (0.3 mm) and safe overloads (150 %) are ASSUMED: set the 0.2 mm stop gaps after measuring",
        "the 24 V desktop supply stands on the bench (no fixing needed)"]
    rep["wiring"] = {
        "controller": "BTT MMB CAN V2.0 (the head's board type) on CAN/USB to a Raspberry Pi running the LH-1 firmware stack",
        "drivers": {"M1": "Z NEMA17_48 (rev G5)", "M2": "plunger pusher Nanotec LGA351S12-A-UIAP-038 at 0.7 A (V6; homing <= 0.50 A, never into "
                   "its internal stop above 0.57 A)", "M3": "X linear-module NEMA17", "M4": "dock lift (NEMA17 48 + Tr8x2) for the two-bay dock (V1)"},
        "summary_rev_j": "Z-up switch: normally-open (made at the top).  Cradle force switch: normally-closed.  Load cells: read by the "
                         "head board (MMB CAN, Klipper [load_cell] through the HX711s), not by the Pi.",
        "inputs": {"Z home": "KW10 NC (broken wire = triggered)",
                   "Z up": "D2F roller lever, normally-open, wired NO: CLOSED from the normal top up to the eject top (bench_rig.cfg '^!'); a broken "
                           "wire reads 'not up' and blocks X moves (fail-safe; rev G5 correction: was 'NC')",
                   "tip force": "cradle force switch, normally-closed: D2F changeover on the cradle body (CB-1; HP-1 before rev J) wired NC: a broken wire reads 'force reached' and ends the stroke (rev G5 "
                                "correction: was 'NO')",
                   "plunger home": "D2F NC", "X home": "module limit switch NC"},
        "load_cells": "LC-1 (rack), LC-2 (comb), LC-3 (plunger yoke), LC-4 (eject): one HX711 each on the BR-8 plate, read by the head "
                      "board (MMB CAN) through Klipper [load_cell] (rev G5 correction: not the Pi GPIO)",
        "balance": "RS232/USB to the Pi; stable-reading command per weighing; draft shield top door left open only for the nozzle",
        "power": "24 V PSU -> MMB; Pi on its own 5 V supply; E-stop in the 24 V motor line",
        "note": "same FFC 10-way loop to the tool plate as LH-1 (plunger motor 4 + switch 2 + force switch 2 conductors), ZIF adapters at both ends"}
    if getattr(P, "REV_K", False):     # rev K (ADR-004..007): bench rig additions, concept level
        rep["rev_k_additions"] = {
            "gripper_trial": {"mount": "the machine gripper GR-1 bolts to the rig's TP-1 tool plate in place of a cradle (same TOOL_BOLTS "
                                       "and dowels): no new rig part",
                              "tests": ["grip force 5-20 N with a force gauge between the fingers (GRIP_FORCE_N)",
                                        "slip: plate + 0.6 kg (GRIP_MAX_PAYLOAD) hung from a clamped plate, X move at 2 m/s2: no slip on the "
                                        "BR-3 plate-nest station",
                                        "200 pick-and-place cycles of a real full-skirt PCR plate between the plate nest and the rack nest: "
                                        "seat error by the VC-1 marker method / dial indicator",
                                        "LD-1 lid placement on a plate and on a parked lid (LIFO D3 case), 100 cycles",
                                        "stackable tip-rack top-frame grip (band 55-60 mm, ASSUMED) and a 2-high rack stack"]},
            "exchange_coupling_reservation (ADR-010 CT-1..CT-9)": {
                "reserved": "one tool coupling on the rig's Z carriage in place of a cradle (the TP-1 TOOL_BOLTS pattern and the 2 dowels "
                            "stay free for a coupling adapter plate up to 20 mm thick, Al or steel) and a two-bay dock on the rig base "
                            "beside the tip-rack station (2 x 72 mm bays, 110 deep, 200 tall, ASSUMED): the space is kept free; its fixing is chosen with the coupling in revision L",
                "tests": "CT-1 repeatability over 1,000 exchanges; CT-2 added deflection at 120/150 N; CT-3 no lift-off at twice the working "
                         "force; CT-4 power-off pull and half-locks; CT-5 3,000 exchanges; CT-6 pipetting after exchange; CT-7 force switch "
                         "through the push-rod; CT-8 tool code; CT-9 the real forces",
                "status": "space and mounting reserved only; the coupling and dock are designed in revision L"},
            "syringe_dispense_prototype (ADR-010 market adjustments)": {
                "concept": "one to four disposable luer-lock syringes (1-5 mL) in a PA12 clamp block on a lead-screw plunger drive (NEMA17 + "
                           "T8 screw + a pusher plate on two 8 mm rods, home switch), bolted to the rig tool plate (TP-1 TOOL_BOLTS and "
                           "dowels) like a cradle; a blunt luer needle or a jetting nozzle on each syringe",
                "use_on_the_rig": "draw from a reagent tube at the rig's tube position, then jet into wells from above at the plate nest",
                "tests": "smallest clean non-contact volume, drop placement, bead settling in the syringe over time, ethanol dripping, "
                         "carry-over between fills (all by weight on the rig balance)",
                "status": "concept and mounting only; nothing designed in detail"},
            "low_elution_magnet_check (item 10e / X7)": {
                "what": "elute 8, 10 and 16 uL from MB-1 and from a bought low-elution (spring-cushioned) magnet plate at the BR-3 plate-nest "
                        "station; draw 1 uL less than loaded with the rig's p10 tool offset 0.4 mm away from the pellet side",
                "measure": "bead carry-over in the eluate (count or absorbance), DNA yield against a hand-pipetted control, pellet height "
                           "photographed through a clear-walled plate",
                "pass": "MB-1 kept for elution only if carry-over and yield match the low-elution magnet; otherwise the low-elution magnet is "
                        "the elution position (B4 in the K2 layout)",
                "status": "test plan only, nothing measured"},
            "single_tip_and_tip_types (Y14)": {"what": "wide-bore p200 tips in the rig tip rack and through the TW-4D comb and tip-sensor "
                                                       "pair: pick-up, eject and detection; VC-2 tip-length read for p10/p200/p300/wide-bore",
                                               "status": "test plan only"},
            "weighing_nest": {"mount": "WN-1 (WNB-1, cell, WNP-1, IN-W) on BR-3 at the plate-nest station (BR-3 gets the window and 2 x M4 "
                                       "of the deck web, CAD later)", "against": "the microbalance at the balance station x 480",
                              "tests": "ADR-005: bias, repeatability, settle time, 1-minute drift, gantry-motion and fan effects per volume "
                                       "class; overload-stop setting by the GG-3 method"},
            "bead_retention_fixture": {"plate": "clear-walled full-skirt PCR plate (2024 evidence)", "magnet": "MB-1 passive magnet block on "
                                       "an IN-P at the plate-nest station", "shaker": "the bought B3 shaker on its SK-1 adapter on the bench "
                                       "beside BR-3 (the gripper trial moves plates between them), or an orbital stand-in at 300-1000 rpm",
                                       "measure": "bead and DNA recovery per transfer against the hand method (first biological test, ADR-006)"},
            "tipless_prototype (concept only)": {
                "bulk_dispense_nozzle": "an 8-nozzle non-contact head (PA12 manifold, 0.5 mm stainless needles at 9 mm pitch) on the PA-1 "
                                        "module adapter pattern on TP-1; small pump and reagent bottle on BR-8; dispense over the balance",
                "vacuum_needle_manifold": "DROPPED by the owner (14 Nov 2026): no mounting point"}}
    rep["procedures"] = {
        "plunger": "step the plunger at 0.1 mm, log LC-3 force and step count to first and second stop, per pipette",
        "gravimetric": "dispense 0.5, 1, 2, 5, 10, 50, 300 uL into the trap vial on the balance, 20-100 replicates each, ISO 8655-6 "
                       "corrections (Z factor, evaporation blank); dial at the build's fixed setting and at the volume, to test M11",
        "pickup_strip": "pick one and eight tips over LC-1 with the force switch ending the stroke; strip on TW-4D over LC-2; log peak forces",
        "hard_stop": "per cradle (BR-B11): stop screw fully down; jog with a 0.5 mm feeler on the rack until pinched -> s_pinch; contact "
                     "s_c = s_pinch + 0.5; jog to s_c - set gap (3.0 rig / 3.5 machine), raise the stop screw to the ledge, lock the nut; "
                     "verify >= 2 mm gap with the axis on the stop; firmware limit = stop - 1"}
    rep["checks"]["rig_interference_note"] = ("overlaps_mm3 per tool are at strokes the axis can reach (up to its ZHS-1 setting); "
                                              "overlaps_unconstrained_mm3 are the G2 figures at s 200, which the stop now prevents")
    (OUT / "report.json").write_text(json.dumps(rep, indent=1, default=str))
    print(json.dumps({t: {"n_real": v["n_real"], "R4": v["R4_at_hard_stop"]} for t, v in rep["per_tool"].items()}, indent=1))
    print("static", rep["checks"]["rig_static_overlaps_mm3"], rep["checks"]["plate_hole_counts"], rep["parts_list"])


if __name__ == "__main__":
    main()
