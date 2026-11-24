"""TC-1 geometry: every solid in GLOBAL deck coordinates (params.py frame), lid CLOSED and CLAMPED.

The open state is produced in build.py by rigid moves of three groups:
  carriage group  (moves +Y by the lid travel)
  lid group       (moves +Y by the travel and +Z by the lift stroke)
  presser         (moves +Y by the travel and +Z by lift + spring stroke)
Bought parts are modelled as envelopes (marked 'bought') so that clearances include them.
"""
from __future__ import annotations

import math
import sys
from pathlib import Path

import cadquery as cq

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import params as PR  # noqa: E402
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "assembly"))   # integration rev D: params overrides
import params_override  # noqa: E402
params_override.apply(PR)

X0, Y0 = PR.nest_centre("C1")             # plate centre
YD = PR.nest_centre("D1")[1]              # lid park centre (Y)
TRAVEL = YD - Y0
WIN_L = PR.WINDOW_L                       # merged window: WINDOW_L in X ...
WIN_Y0 = Y0 - PR.WINDOW_W / 2             # ... from the front edge of the C1 window
WIN_Y1 = YD + PR.WINDOW_W / 2             # to the back edge of the D1 window
NEST_X0 = X0 - PR.INSERT_L / 2            # -X edge of the C1/D1 nests
DECK_X0 = -PR.DECK_L / 2

P = {   # ---------------- module parameters (mm) ----------------
    # bought 96-well block: MEASURE ON THE PART
    "block_l": 112.0, "block_w": 76.0, "block_h": 22.0, "block_top_below_plate_top": 1.5,
    "well_d_top": 5.5, "well_d_bot": 3.0, "well_depth": 14.0,
    # plate (nominal full-skirted 96 x 0.2 mL; MEASURE, semi-skirted differs)
    "plate_h": 16.0, "plate_skirt_wall": 1.0,
    # stack
    "if_t": 0.25, "tec": 50.0, "tec_t": 4.0, "tec_dx": 27.0,
    "top_t": 5.0, "cut_gap": 2.0,
    "sink_w": 160.0, "sink_base_t": 8.0, "fin_h": 50.0, "fin_t": 1.5, "fin_pitch": 4.0,
    "sink_y0_gap": 3.0, "sink_y1": None, "fin_y1": 140.0,
    "clamp_xy": [(-40.0, -32.0), (0.0, -32.0), (40.0, -32.0), (-40.0, 32.0), (0.0, 32.0), (40.0, 32.0)],
    "spacer_xy": [(-45.0, 135.0), (45.0, 135.0), (-45.0, 175.0), (45.0, 175.0)],   # (dx from X0, absolute y)
    "cable_hole": (-52.0, 192.0, 12.0),
    "duct_wall": 2.0, "plenum_h": 8.0, "duct_floor": 3.0, "fan": 80.0, "fan_t": 25.0, "fan_dx": 41.0,
    "fan_y": 57.0, "rod_xy": [(-70.0, 150.0), (70.0, 150.0), (-70.0, 176.0), (70.0, 176.0)],
    "tray_t": 2.0, "tray_y0": 100.0, "tray_flange_h": 20.0,
    # lid
    "pad_t": 4.0, "pad_comp": 0.3,   # integration rev F (M16): liner/spring equilibrium (was 0.8)
    "pad_l": 120.0, "pad_w": 82.0, "lidp_l": 122.0, "lidp_w": 82.0, "lidp_t": 6.0,
    # integration rev E: 'pad' = gripper-placed SEALING LID (liner 1.5 + top 2.5)
    "heater_t": 1.5, "peek_d": 8.0, "hood_t": 1.5, "hood_clear": 0.6, "skirt_below_plate_top": 0.5,   # rev E: hood skirt clears the sealing lid by 2.3 mm when raised (was 4.0)
   
    "lift": 6.0, "spring_stroke": 5.0,
    "arm_t": 10.0, "arm_x1": -80.0, "arm_root_w": 64.0, "arm_beam_y": (-40.0, 30.0),
    # side guide (strip on the deck margin -X of the C1/D1 nests)
    "rail_w": 15.0, "rail_h": 10.0, "blk_w": 32.0, "blk_l": 59.0, "blk_h_total": 16.0, "blk_gap": 4.0,
    "carr_off": -22.0, "carr_t": 6.0, "pin_d": 8.0, "pin_dy": 20.0, "pin_top": 93.0,   # integration rev D: 96 -> 93 with the bridge
    "presser_t": 8.0, "spring_len_comp": 20.0, "bridge_t": 6.0,
    "chain_w": 9.5, "chain_h": 12.0, "chain_r": 18.0, "chain_l_free": 125.0,   # integration rev D (CR-11): inner 6 x 8, outer <= 9.5 x 12
    "pulley_z": 28.5, "pulley_pd": 12.73, "belt_w": 6.0, "belt_t": 1.4,
}
P["sink_y1"] = WIN_Y1 - 2.0

# ---------------- derived heights ----------------
SEAT_Z = PR.LABWARE_SEAT_Z
PLATE_TOP = SEAT_Z + P["plate_h"]
BLOCK_TOP = PLATE_TOP - P["block_top_below_plate_top"]
BLOCK_BOT = BLOCK_TOP - P["block_h"]
TEC_TOP = BLOCK_BOT - P["if_t"]
TEC_BOT = TEC_TOP - P["tec_t"]
SINK_TOP = TEC_BOT - P["if_t"]
SINK_BOT = SINK_TOP - P["sink_base_t"]
FIN_BOT = SINK_BOT - P["fin_h"]
PAD_BOT = PLATE_TOP - P["pad_comp"]
LIDP_BOT = PAD_BOT + P["pad_t"]
LIDP_TOP = LIDP_BOT + P["lidp_t"]
HOOD_TOP_IN = LIDP_TOP + P["heater_t"] + 4.3          # 4.3 mm insulation / PEEK spacer zone above the heater
HOOD_TOP = HOOD_TOP_IN + P["hood_t"]
SKIRT_BOT = PLATE_TOP - P["skirt_below_plate_top"]
ARM_BOT, ARM_TOP = HOOD_TOP, HOOD_TOP + P["arm_t"]
PRESS_BOT = ARM_TOP + P["spring_len_comp"]
BRIDGE_BOT = 87.0                                      # integration rev D: was 90 (presser raised top 85.5)
CMOTOR_L = 34.0                                        # integration rev D: NEMA17 x 34 (was x 40): strip parts top z 127
CMOTOR_TOP = BRIDGE_BOT + P["bridge_t"] + CMOTOR_L

# ---------------- side-guide X layout (strip between deck edge and nest edge) ----------------
STRIP_X0, STRIP_X1 = DECK_X0 + 1.0, NEST_X0 - 1.0     # -249 .. -198
CHAIN_X = (STRIP_X0, STRIP_X0 + P["chain_w"])
BELT_X = -235.0
BLK_X1 = STRIP_X1 - 0.5
BLK_X0 = BLK_X1 - P["blk_w"]
RAIL_X = 0.5 * (BLK_X0 + BLK_X1)
CARR_Y_CLOSED = Y0 + P["carr_off"]
RAIL_Y = (2.0, 176.0)
YMOT_Y = (177.0, 219.0)


def box(x0, x1, y0, y1, z0, z1) -> cq.Workplane:
    return cq.Workplane("XY").box(x1 - x0, y1 - y0, z1 - z0, centered=False).translate((x0, y0, z0))


def cyl_z(x, y, d, z0, z1) -> cq.Workplane:
    return cq.Workplane("XY").circle(d / 2).extrude(z1 - z0).translate((x, y, z0))


def cyl_x(x0, x1, y, z, d) -> cq.Workplane:
    return cq.Workplane("YZ").circle(d / 2).extrude(x1 - x0).translate((x0, y, z))


def holes(w, pts, d, z0, z1):
    for x, y in pts:
        w = w.cut(cyl_z(x, y, d, z0, z1))
    return w


# ======================================================== thermal stack
def wells() -> list[tuple[float, float]]:
    a1x = X0 - PR.SBS_L / 2 + PR.A1_OFFSET_X
    a1y = Y0 + PR.SBS_W / 2 - PR.A1_OFFSET_Y        # row A at the back (+Y)
    return [(a1x + c * PR.WELL_PITCH_96, a1y - r * PR.WELL_PITCH_96) for r in range(8) for c in range(12)]


def pcr_block() -> cq.Workplane:
    b = box(X0 - P["block_l"] / 2, X0 + P["block_l"] / 2, Y0 - P["block_w"] / 2, Y0 + P["block_w"] / 2, BLOCK_BOT, BLOCK_TOP)
    cone = cq.Workplane("XY").circle(P["well_d_bot"] / 2).workplane(offset=P["well_depth"]) \
        .circle(P["well_d_top"] / 2).loft(combine=True)
    cuts = None
    for x, y in wells():
        c = cone.translate((x, y, BLOCK_TOP - P["well_depth"]))
        cuts = c if cuts is None else cuts.union(c)
    b = b.cut(cuts)
    # blind tapped M3 holes from below (drill/tap on receipt) for the clamp screws; thermistor bores
    b = holes(b, [(X0 + dx, Y0 + dy) for dx, dy in P["clamp_xy"]], 2.5, BLOCK_BOT, BLOCK_BOT + 7.0)
    return b


def tecs() -> list[cq.Workplane]:
    h = P["tec"] / 2
    return [box(X0 + s * P["tec_dx"] - h, X0 + s * P["tec_dx"] + h, Y0 - h, Y0 + h, TEC_BOT, TEC_TOP) for s in (-1, 1)]


def tec_interfaces(side: str) -> list[cq.Workplane]:
    """integration rev E: 0.25 mm interface layers over each TEC footprint (block side: graphite sheet; sink side: grease)"""
    h = P["tec"] / 2
    z0, z1 = (TEC_TOP, BLOCK_BOT) if side == "block" else (SINK_TOP, TEC_BOT)
    return [box(X0 + s * P["tec_dx"] - h, X0 + s * P["tec_dx"] + h, Y0 - h, Y0 + h, z0, z1) for s in (-1, 1)]


def insulation_sheet() -> cq.Workplane:
    s = box(X0 - P["block_l"] / 2, X0 + P["block_l"] / 2, Y0 - P["block_w"] / 2, Y0 + P["block_w"] / 2, SINK_TOP, BLOCK_BOT)
    h = P["tec"] / 2 + 1.0
    for sx in (-1, 1):
        s = s.cut(box(X0 + sx * P["tec_dx"] - h, X0 + sx * P["tec_dx"] + h, Y0 - h, Y0 + h, SINK_TOP, BLOCK_BOT))
    return holes(s, [(X0 + dx, Y0 + dy) for dx, dy in P["clamp_xy"]], 6.0, SINK_TOP, BLOCK_BOT)


def sink_y0() -> float:
    return WIN_Y0 + P["sink_y0_gap"]


def heat_sink() -> cq.Workplane:
    w2 = P["sink_w"] / 2
    y0, y1 = sink_y0(), P["sink_y1"]
    s = box(X0 - w2, X0 + w2, y0, y1, SINK_BOT, SINK_TOP)
    n = int(P["sink_w"] / P["fin_pitch"])
    for i in range(n):
        xc = X0 - w2 + P["fin_pitch"] * (i + 0.5)
        s = s.union(box(xc - P["fin_t"] / 2, xc + P["fin_t"] / 2, y0, P["fin_y1"], FIN_BOT, SINK_BOT))
    cx, cy, _ = P["cable_hole"]
    s = s.cut(box(X0 + cx - 6.0 - 20.0, X0 + cx + 6.0, cy - 10.0, y1, SINK_BOT, SINK_TOP))    # cable notch
    for dx, dy in P["clamp_xy"]:                                    # spot-faces through the fins + M3 clearance
        s = s.cut(cyl_z(X0 + dx, Y0 + dy, 11.0, FIN_BOT - 1, SINK_BOT))
        s = s.cut(cyl_z(X0 + dx, Y0 + dy, 7.0, SINK_BOT, SINK_TOP))  # PEEK top-hat bushing bore
    s = holes(s, [(X0 + dx, y) for dx, y in P["spacer_xy"]], 3.3, SINK_BOT, SINK_TOP)   # M4 tapped
    s = holes(s, [(X0 + dx, y) for dx, y in P["rod_xy"]], 3.3, SINK_BOT, SINK_BOT + 6)  # M4 tapped, blind
    # integration rev D: 1 mm relief on the base top outside the deck window (was 1 mm under the deck, now 2)
    for sx in (-1, 1):
        x_in, x_out = X0 + sx * (WIN_L / 2 - 2.0), X0 + sx * (w2 + 1.0)   # relief starts 2 mm inside the window edge
        s = s.cut(box(min(x_in, x_out), max(x_in, x_out), y0 - 1, y1 + 1, SINK_TOP - 1.0, SINK_TOP + 0.1))
    return s


def spacers() -> list[cq.Workplane]:
    return [cyl_z(X0 + dx, y, 8.0, SINK_TOP, 0.0).cut(cyl_z(X0 + dx, y, 4.3, SINK_TOP, 0.0)) for dx, y in P["spacer_xy"]]


SUBPLATE_FRAME_SCREWS = [(-230.0, 55.0), (-230.0, 165.0)]   # integration rev F: deck DK-2 csk screws to the x=-230 side beam


def top_plate() -> cq.Workplane:
    y0, y1 = Y0 - PR.INSERT_W / 2 + 1.0, YD + PR.INSERT_W / 2      # 1 .. 219
    t = box(STRIP_X0, X0 + PR.INSERT_L / 2 - 0.5, y0, y1, 0.0, P["top_t"])
    g = P["cut_gap"]
    t = t.cut(box(X0 - P["block_l"] / 2 - g, X0 + P["block_l"] / 2 + g, Y0 - P["block_w"] / 2 - g,
                  Y0 + P["block_w"] / 2 + g, 0.0, P["top_t"]))
    scr = [(X0 + s[0], Y0 + s[1]) for s in PR.INSERT_SCREWS if s[1] < 0] + \
          [(X0 + s[0], YD + s[1]) for s in PR.INSERT_SCREWS if s[1] > 0]
    t = holes(t, scr, 4.5, 0.0, P["top_t"])
    dw = [(X0 + d[0], Y0 + d[1]) for d in PR.INSERT_DOWELS if d[1] < 0]
    t = holes(t, dw, PR.DOWEL_D, 0.0, P["top_t"])
    dr = [(X0 + d[0], YD + d[1]) for d in PR.INSERT_DOWELS if d[1] > 0]
    for x, y in dr:                                                  # rear dowel: slot along Y
        t = t.cut(box(x - PR.DOWEL_D / 2, x + PR.DOWEL_D / 2, y - 2.0, y + 2.0, 0.0, P["top_t"]))
        t = t.cut(cyl_z(x, y - 2.0, PR.DOWEL_D, 0, P["top_t"])).cut(cyl_z(x, y + 2.0, PR.DOWEL_D, 0, P["top_t"]))
    t = holes(t, [(X0 + dx, y) for dx, y in P["spacer_xy"]], 4.5, 0.0, P["top_t"])
    cx, cy, cd = P["cable_hole"]
    t = t.cut(cyl_z(X0 + cx, cy, cd, 0.0, P["top_t"]))
    rail_holes = [(RAIL_X, y) for y in range(int(RAIL_Y[0]) + 18, int(RAIL_Y[1]), 40)]
    t = holes(t, rail_holes, 2.5, 0.0, P["top_t"])                   # M3 tapped (5 mm aluminium)
    # integration: M4 clearance holes for the 2 strip screws into the deck's new tapped holes (params TC1_STRIP_TAPPED_HOLES)
    t = holes(t, list(PR.TC1_STRIP_TAPPED_HOLES), 4.5, 0.0, P["top_t"])
    # integration rev F (M7): D12 access holes over the two DK-2 frame screws so DK-2 + TC-1 lift out together
    t = holes(t, [h for h in SUBPLATE_FRAME_SCREWS if STRIP_X0 < h[0] < X0 + PR.INSERT_L / 2 and y0 < h[1] < y1], 12.0, 0.0, P["top_t"])
    return t


def duct() -> cq.Workplane:
    w2, wl = P["sink_w"] / 2 + 0.5, P["duct_wall"]
    y0 = sink_y0() - 0.5 - wl
    y1 = P["sink_y1"] + 1.5
    z_floor_top = FIN_BOT - 0.5 - P["plenum_h"]
    zb = z_floor_top - P["duct_floor"]
    d = box(X0 - w2 - wl, X0 + w2 + wl, y0, y1, zb, SINK_TOP - 1.0)   # integration rev D: top 1 mm lower (2 mm to the deck)
    d = d.cut(box(X0 - w2, X0 + w2, y0 + wl, y1, FIN_BOT - 0.5, SINK_TOP))                   # sink + exhaust
    d = d.cut(box(X0 - w2, X0 + w2, y0 + wl, P["fan_y"] + P["fan"] / 2, z_floor_top, FIN_BOT - 0.5))   # plenum
    for s in (-1, 1):
        d = d.cut(cyl_z(X0 + s * P["fan_dx"], P["fan_y"], 76.0, zb, z_floor_top))
        for hx in (-35.75, 35.75):                                                           # fan screws
            for hy in (-35.75, 35.75):
                d = d.cut(cyl_z(X0 + s * P["fan_dx"] + hx, P["fan_y"] + hy, 4.3, zb, z_floor_top))
    d = holes(d, [(X0 + dx, y) for dx, y in P["rod_xy"]], 4.5, zb, FIN_BOT - 0.5)
    cx, cy, _ = P["cable_hole"]
    d = d.cut(cyl_z(X0 + cx - 8.0, cy, 14.0, zb, z_floor_top))
    return d


def duct_floor_z() -> float:
    return FIN_BOT - 0.5 - P["plenum_h"] - P["duct_floor"]


def fans() -> list[cq.Workplane]:
    z1 = duct_floor_z()
    f2 = P["fan"] / 2
    return [box(X0 + s * P["fan_dx"] - f2, X0 + s * P["fan_dx"] + f2, P["fan_y"] - f2, P["fan_y"] + f2, z1 - P["fan_t"], z1)
            for s in (-1, 1)]


def tray_z() -> float:
    return -(PR.DECK_T + PR.UNDER_DECK_CLEAR) + 3.0


def tray() -> cq.Workplane:
    z0, t = tray_z(), P["tray_t"]
    w2 = P["sink_w"] / 2
    y0, y1 = P["tray_y0"], P["sink_y1"] + 1.0
    tr = box(X0 - w2, X0 + w2, y0, y1, z0, z0 + t)
    for s in (-1, 1):
        x = X0 + s * (w2 - t / 2)
        tr = tr.union(box(x - t / 2, x + t / 2, y0, y1, z0 + t, z0 + t + P["tray_flange_h"]))
    tr = holes(tr, [(X0 + dx, y) for dx, y in P["rod_xy"]], 4.5, z0, z0 + t)
    if getattr(PR, "REV_J", False):     # rev J: TCB-1 retired (ADR-003 #2): plain 25 mm grid of D3.4 holes on the old board area
        grid = [(x, y) for x in [TCB1["x"][0] + 10.0 + 25.0 * k for k in range(5)] for y in [TCB1["y"][0] + 9.0 + 25.0 * k for k in range(3)]]
        return holes(tr, grid, 3.4, z0, z0 + t)
    return holes(tr, TCB1["holes"], 3.4, z0, z0 + t)      # integration rev D: 4 x M3 for the TCB1 standoffs


def rods() -> list[cq.Workplane]:
    return [cyl_z(X0 + dx, y, 4.0, tray_z(), SINK_BOT) for dx, y in P["rod_xy"]]


# integration rev D: TCB1 controller board 120 x 88 on 6 mm standoffs (electronics/tcb1/layout_brief.yaml), 22 mm parts
TCB1 = dict(x=(-192.0, -72.0), y=(107.0, 195.0), standoff=6.0, pcb_t=1.6, parts_h=22.0,
            holes=[(-192.0 + 4.0, 107.0 + 4.0), (-192.0 + 116.0, 107.0 + 4.0), (-192.0 + 4.0, 107.0 + 84.0), (-192.0 + 116.0, 107.0 + 84.0)])


def electronics() -> cq.Workplane:
    z0 = tray_z() + P["tray_t"] + TCB1["standoff"]
    return box(TCB1["x"][0], TCB1["x"][1], TCB1["y"][0], TCB1["y"][1], z0, z0 + TCB1["pcb_t"] + TCB1["parts_h"])


def tcb1_standoffs() -> list[cq.Workplane]:
    z0 = tray_z() + P["tray_t"]
    return [cyl_z(x, y, 5.5, z0, z0 + TCB1["standoff"]) for x, y in TCB1["holes"]]


# ======================================================== labware reference
def pcr_plate() -> cq.Workplane:
    o = box(X0 - PR.SBS_L / 2, X0 + PR.SBS_L / 2, Y0 - PR.SBS_W / 2, Y0 + PR.SBS_W / 2, SEAT_Z, PLATE_TOP)
    w = P["plate_skirt_wall"]
    return o.cut(box(X0 - PR.SBS_L / 2 + w, X0 + PR.SBS_L / 2 - w, Y0 - PR.SBS_W / 2 + w, Y0 + PR.SBS_W / 2 - w,
                     SEAT_Z, BLOCK_TOP + 0.1))


# ======================================================== lid group (closed, clamped)
# integration rev E: sealing lid for full-skirted PCR plates (replaces the heated lid's own pad: the pad must never touch open
# wells).  Rigid frame inside the SBS footprint (127.36 x 85.08) resting on the plate deck (z 19.6), replaceable 1.5 mm silicone
# liner on the rims (0.8 mm compression), 2.5 mm top; grip band z 19.6 .. 24.2 (gripper fingertips >= z 19.8).  NOMINAL: specify
# by the bought lid chosen at stage 0.
LID_FRAME_Z0 = SEAT_Z + 14.6
LID_OUT = (PR.SBS_L - 0.4, PR.SBS_W - 0.4)


def sealing_lid() -> cq.Workplane:
    lid = box(X0 - LID_OUT[0] / 2, X0 + LID_OUT[0] / 2, Y0 - LID_OUT[1] / 2, Y0 + LID_OUT[1] / 2, LID_FRAME_Z0, LIDP_BOT)
    return lid.cut(box(X0 - LID_OUT[0] / 2 + 1.5, X0 + LID_OUT[0] / 2 - 1.5, Y0 - LID_OUT[1] / 2 + 1.5, Y0 + LID_OUT[1] / 2 - 1.5,
                       LID_FRAME_Z0 - 0.1, PAD_BOT))


def pad() -> cq.Workplane:
    return box(X0 - P["pad_l"] / 2, X0 + P["pad_l"] / 2, Y0 - P["pad_w"] / 2, Y0 + P["pad_w"] / 2, PAD_BOT, LIDP_BOT)


def lid_plate() -> cq.Workplane:
    lp = box(X0 - P["lidp_l"] / 2, X0 + P["lidp_l"] / 2, Y0 - P["lidp_w"] / 2, Y0 + P["lidp_w"] / 2, LIDP_BOT, LIDP_TOP)
    pts = [(X0 + sx * 52.0, Y0 + sy * 32.0) for sx in (-1, 1) for sy in (-1, 1)]
    lp = holes(lp, pts, 2.5, LIDP_TOP - 6.0, LIDP_TOP)              # M3 tapped for the PEEK standoffs
    return lp.cut(cyl_x(X0 - P["lidp_l"] / 2, X0 - P["lidp_l"] / 2 + 15, Y0 + 20.0, LIDP_BOT + 3.0, 2.0))  # NTC bore


def heater() -> cq.Workplane:
    return box(X0 - 45, X0 + 45, Y0 - 27, Y0 + 27, LIDP_TOP, LIDP_TOP + P["heater_t"])


def peek_standoffs() -> list[cq.Workplane]:
    return [cyl_z(X0 + sx * 52.0, Y0 + sy * 32.0, P["peek_d"], LIDP_TOP, HOOD_TOP_IN).cut(
        cyl_z(X0 + sx * 52.0, Y0 + sy * 32.0, 3.2, LIDP_TOP, HOOD_TOP_IN)) for sx in (-1, 1) for sy in (-1, 1)]


def hood_dims():
    xi = PR.SBS_L / 2 + P["hood_clear"]
    yi = PR.SBS_W / 2 + P["hood_clear"]
    return xi, yi, xi + P["hood_t"], yi + P["hood_t"]


# rev J (CR-21): subminiature through-beam pair (E3T-ST class, about 31 x 11 x 3.5 per head, 2 x M2: NOMINAL, check the part
# bought) in place of the 3 mm IR pair: one head on the outside of the front skirt, one on the back skirt, beam along Y through two
# D4 holes in the skirts, in the plane just above the plate top (lens 3 mm above the skirt edge: NOMINAL).  Heads move with the hood.
BEAM_HEAD = dict(l=31.0, h=11.0, t=3.5, holes_dx=12.5, lens_up=3.0, hole_d=4.0, m2_d=2.4)


def beam_heads() -> list[cq.Workplane]:
    xi, yi, xo, yo = hood_dims()
    b = BEAM_HEAD
    return [box(X0 - b["l"] / 2, X0 + b["l"] / 2, Y0 - yo - b["t"], Y0 - yo, SKIRT_BOT, SKIRT_BOT + b["h"]),
            box(X0 - b["l"] / 2, X0 + b["l"] / 2, Y0 + yo, Y0 + yo + b["t"], SKIRT_BOT, SKIRT_BOT + b["h"])]


def hood() -> cq.Workplane:
    xi, yi, xo, yo = hood_dims()
    h = box(X0 - xo, X0 + xo, Y0 - yo, Y0 + yo, SKIRT_BOT, HOOD_TOP)
    h = h.cut(box(X0 - xi, X0 + xi, Y0 - yi, Y0 + yi, SKIRT_BOT, HOOD_TOP_IN))
    pts = [(X0 + sx * 52.0, Y0 + sy * 32.0) for sx in (-1, 1) for sy in (-1, 1)]
    h = holes(h, pts, 3.4, HOOD_TOP_IN, HOOD_TOP)
    if getattr(PR, "REV_J", False):
        b = BEAM_HEAD
        zl, zm = SKIRT_BOT + b["lens_up"], SKIRT_BOT + b["h"] / 2
        for y0, y1 in ((Y0 - yo - 1, Y0 - yi + 1), (Y0 + yi - 1, Y0 + yo + 1)):
            h = h.cut(cq.Workplane("XZ").center(X0, zl).circle(b["hole_d"] / 2).extrude(-(y1 - y0)).translate((0, y0, 0)))
            for dx in (-b["holes_dx"], b["holes_dx"]):
                h = h.cut(cq.Workplane("XZ").center(X0 + dx, zm).circle(b["m2_d"] / 2).extrude(-(y1 - y0)).translate((0, y0, 0)))
    return h


def arm(cy=CARR_Y_CLOSED) -> cq.Workplane:
    lc = cy - P["carr_off"]
    r2 = P["arm_root_w"] / 2
    a = box(BLK_X0 + 2.5, NEST_X0 - 1.0, cy - r2, cy + r2, ARM_BOT, ARM_TOP)
    a = a.union(box(NEST_X0 - 1.0, P["arm_x1"], lc + P["arm_beam_y"][0], lc + P["arm_beam_y"][1], ARM_BOT, ARM_TOP))
    a = holes(a, [(RAIL_X, cy - P["pin_dy"]), (RAIL_X, cy + P["pin_dy"])], 12.0, ARM_BOT, ARM_TOP)   # flanged bushings
    a = a.cut(cyl_z(RAIL_X, cy, 10.0, ARM_BOT, ARM_TOP))
    pts = [(X0 + sx * 52.0, Y0 + sy * 32.0) for sx in (-1, 1) for sy in (-1, 1)]
    a = holes(a, [p for p in pts if lc + P["arm_beam_y"][0] < p[1] < lc + P["arm_beam_y"][1]], 3.4, ARM_BOT, ARM_TOP)
    if getattr(PR, "REV_K", False):     # rev K (ADR-004 d3): tag pocket + 2 blind holes on the lid arm top (moves with the lid)
        a = a.cut(box(X0 + 15.8, X0 + 28.2, lc - 6.2, lc + 6.2, ARM_TOP - 0.3, ARM_TOP + 1))
        for dx in (9.0, 35.0):
            a = a.cut(cyl_z(X0 + dx, lc, 2.0, ARM_TOP - 1.5, ARM_TOP + 1))
    return a


# ======================================================== carriage group (closed)
def carriage_plate(cy=CARR_Y_CLOSED) -> cq.Workplane:
    l2 = P["blk_l"] / 2
    zt = PR.LABWARE_SEAT_Z * 0 + P["top_t"] + P["blk_h_total"]       # rail seat on the top plate
    c = box(BELT_X + 4.0, BLK_X1, cy - l2, cy + l2, zt, zt + P["carr_t"])
    c = c.union(box(CHAIN_X[1] + 0.5, BELT_X + 4.0, cy + 8.0, cy + 28.0, zt - 4.0, zt + P["carr_t"]))   # belt clamp
    c = holes(c, [(RAIL_X, cy - P["pin_dy"]), (RAIL_X, cy + P["pin_dy"])], P["pin_d"] + 0.05, zt, zt + P["carr_t"])
    return c


def carriage_z() -> float:
    return P["top_t"] + P["blk_h_total"]


def rail() -> cq.Workplane:
    return box(RAIL_X - P["rail_w"] / 2, RAIL_X + P["rail_w"] / 2, RAIL_Y[0], RAIL_Y[1], P["top_t"], P["top_t"] + P["rail_h"])


def rail_block(cy=CARR_Y_CLOSED) -> cq.Workplane:
    l2 = P["blk_l"] / 2
    b = box(BLK_X0, BLK_X1, cy - l2, cy + l2, P["top_t"] + P["blk_gap"], carriage_z())
    return b.cut(box(RAIL_X - P["rail_w"] / 2 - 0.25, RAIL_X + P["rail_w"] / 2 + 0.25, cy - l2, cy + l2,
                     P["top_t"] + P["blk_gap"], P["top_t"] + P["rail_h"] + 0.5))


def pins(cy=CARR_Y_CLOSED) -> list[cq.Workplane]:
    return [cyl_z(RAIL_X, cy + s * P["pin_dy"], P["pin_d"], carriage_z(), P["pin_top"]) for s in (-1, 1)]


def presser(cy=CARR_Y_CLOSED) -> cq.Workplane:
    p = box(BLK_X0 + 2.5, NEST_X0 + 4.0 - 7.0, cy - 28.0, cy + 28.0, PRESS_BOT, PRESS_BOT + P["presser_t"])
    return holes(p, [(RAIL_X, cy - P["pin_dy"]), (RAIL_X, cy + P["pin_dy"]), (RAIL_X, cy)], 8.2, PRESS_BOT,
                 PRESS_BOT + P["presser_t"])


def bridge(cy=CARR_Y_CLOSED) -> cq.Workplane:
    b = box(RAIL_X - 21.5, RAIL_X + 21.5, cy - 30.0, cy + 30.0, BRIDGE_BOT, BRIDGE_BOT + P["bridge_t"])
    b = holes(b, [(RAIL_X, cy - P["pin_dy"]), (RAIL_X, cy + P["pin_dy"])], P["pin_d"] + 0.05, BRIDGE_BOT, BRIDGE_BOT + 6)
    b = b.cut(cyl_z(RAIL_X, cy, 22.5, BRIDGE_BOT, BRIDGE_BOT + 6))                       # motor pilot
    return holes(b, [(RAIL_X + sx * 15.5, cy + sy * 15.5) for sx in (-1, 1) for sy in (-1, 1)], 3.4, BRIDGE_BOT,
                 BRIDGE_BOT + 6)


def clamp_motor(cy=CARR_Y_CLOSED) -> list[cq.Workplane]:
    z0 = BRIDGE_BOT + P["bridge_t"]
    m = box(RAIL_X - 21.0, RAIL_X + 21.0, cy - 21.0, cy + 21.0, z0, z0 + CMOTOR_L)
    s = cyl_z(RAIL_X, cy, 8.0, ARM_TOP + P["lift"] + 1.0, BRIDGE_BOT)        # Tr8x2 integrated lead screw
    return [m, s]


def seal_switch_bracket(cy=CARR_Y_CLOSED) -> cq.Workplane:
    """TC-25 (MJF PA12, integration rev D, item 16): block under the bridge carrying the 'clamp SEALED' switch beside the
    presser; the lever drops onto the presser's top edge when the presser reaches its sealed (lowest) position."""
    w = box(-200.0, -193.0, cy - 6.0, cy + 6.0, BRIDGE_BOT - 6.0, BRIDGE_BOT)     # hangs under the bridge, beside the presser
    return w.cut(cyl_z(-196.5, cy - 3.0, 1.6, BRIDGE_BOT - 7, BRIDGE_BOT + 1)).cut(cyl_z(-196.5, cy + 3.0, 1.6, BRIDGE_BOT - 7, BRIDGE_BOT + 1))


def seal_switch(cy=CARR_Y_CLOSED) -> cq.Workplane:
    # body 5.8 (x) x 12.8 (y) x 6.5 (z); lever (not modelled) touches the presser top edge (x -200) at its sealed height
    return box(-199.0, -193.2, cy - 6.4, cy + 6.4, BRIDGE_BOT - 12.5, BRIDGE_BOT - 6.0)


def chain_envelope(cy: float) -> cq.Workplane:
    """Mini drag chain lying in the Y-Z plane: fixed end at the rear, moving end at the carriage rear."""
    f = RAIL_Y[1]
    m = cy + P["blk_l"] / 2
    b = (f + m - P["chain_l_free"]) / 2
    z0 = P["top_t"]
    return box(CHAIN_X[0], CHAIN_X[1], b - P["chain_r"], f, z0, z0 + 2 * P["chain_r"] + P["chain_h"])


# ======================================================== fixed drive parts on the strip
def ymotor() -> cq.Workplane:
    zc = P["pulley_z"]
    return box(-227.0, -204.0, YMOT_Y[0], YMOT_Y[1], zc - 21.0, zc + 21.0)


def pulleys() -> list[cq.Workplane]:
    zc = P["pulley_z"]
    return [cyl_x(BELT_X - 3.5, BELT_X + 3.5, 198.0, zc, 16.0), cyl_x(BELT_X - 3.5, BELT_X + 3.5, 11.0, zc, 18.0)]


def belt() -> list[cq.Workplane]:
    zc, r = P["pulley_z"], P["pulley_pd"] / 2
    return [box(BELT_X - 3, BELT_X + 3, 11.0, 198.0, zc + s * r - P["belt_t"] / 2, zc + s * r + P["belt_t"] / 2) for s in (-1, 1)]


def bent_bracket(web_x0, y0, y1, z_top, foot_x0, foot_x1, t=3.0) -> cq.Workplane:
    web = box(web_x0, web_x0 + t, y0, y1, P["top_t"], z_top)
    foot = box(foot_x0, foot_x1, y0, y1, P["top_t"], P["top_t"] + t)
    return web.union(foot)


TC23_FOOT_X0 = -238.0     # integration rev D: was -242; 2.5 mm to the M4 strip screw head at (-244, 190)


def ymotor_bracket() -> cq.Workplane:
    b = bent_bracket(-230.0, YMOT_Y[0], YMOT_Y[1], P["pulley_z"] + 22.0, TC23_FOOT_X0, -227.0)   # integration rev D: foot from -238
    b = b.cut(cyl_x(-230.0, -227.0, 198.0, P["pulley_z"], 22.5))
    for sy in (-1, 1):
        for sz in (-1, 1):
            b = b.cut(cyl_x(-230.0, -227.0, 198.0 + sy * 15.5, P["pulley_z"] + sz * 15.5, 3.4))
    return b


def idler_bracket() -> cq.Workplane:
    b = bent_bracket(-242.5, 2.0, 20.0, P["pulley_z"] + 10.0, CHAIN_X[0], -242.5)
    return b.cut(cyl_x(-242.5, -239.5, 11.0, P["pulley_z"], 5.2))
