"""LH-1 machine family: size geometry (S, M, L from one parameter, the row list) and the family head.

Imported by cad/lh1/machine_family/build.py.  machine_l, head_l and deck_k are imported read-only (their committed outputs are not
touched); the family's own values live here.  Every number is CALCULATED from the stack-up rules of machine_l (which reproduce
revision L) or ASSUMED as marked.  Nothing is built or measured.
"""
from __future__ import annotations

import importlib.util
import json
import math
import sys
from pathlib import Path

import cadquery as cq
import yaml

HERE = Path(__file__).resolve().parent
LH1 = HERE.parent
ROOT = LH1.parent.parent
sys.path.insert(0, str(LH1))
sys.path.insert(0, str(HERE))
import params as P  # noqa: E402


def _load(name, rel):
    spec = importlib.util.spec_from_file_location(name, str(LH1 / rel))
    m = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(m)
    return m


M = _load("machine_l_for_family", "machine_l/build.py")
HL, DKK, DK = M.HL, M.DKK, M.DK
B, cyl, mv, fuse_all, cut_all = M.B, M.cyl, M.mv, M.fuse_all, M.cut_all
V = cq.Vector
SEAT = P.LABWARE_SEAT_Z
ZB = P.L_X_BEAM_MID_Z
AY = M.AY                    # coupling axis (nozzle row centre), carriage-local y -102.5
GRIP_Y = M.GRIP_Y            # gripper finger centre, carriage-local y -104
WIN = M.WIN

STUDY = LH1 / "size_family" / "out"
LAYOUTS = yaml.safe_load((STUDY / "layouts.yaml").read_text())
SF_JSON = json.loads((STUDY / "size_family.json").read_text())

COLX = {0: -264.0, 1: -132.0, 2: 0.0, 3: 132.0, 4: 264.0}
ROW_F_Y = -275.0
PITCH_X, PITCH_Y = 132.0, 110.0
DOCK_BEHIND_BACK_ROW = 197.0           # rev L: dock axis y 472 behind row G centre 275
END_MARGIN = 15.0                      # travel end margin beyond the outermost target (X both ends, Y front; Y back 16 as rev L)
SHAKER_OUT = 8.0                       # family fix (N5): shaker 8 mm outward of its place centre, shelf cut to x 350
SHELF_CUT_X = 350.0
SIZES = ("S", "M", "L")
ROWS = {"S": ["F", "A", "B", "G"], "M": ["F", "A", "B", "C", "G"], "L": ["F", "A", "B", "C", "D", "G"]}
FRONT_ROWS = ("F", "A", "B")           # never move
BAY_ROWS = {"S": [], "M": ["C"], "L": ["C", "D"]}   # inserted rows (one 110 mm bay of panels, floor and deck each)

# tip parking TPK-S on the starter (ADR-013 small runs): a narrow p300 comb on a half insert at B0, beside the tip waste B1, in the
# left part of the place that VC-2 leaves free.  Same slot law as TPK-1 (deck_k: lower plate holds the collar, upper plate strips it,
# LIFO per row); 7 X positions at 8.6 mm = the entry + 6 eight-channel sets = 48 p300 tips.
#   capacity: the wash series (repeated wash series) park ONE tip per sample at a time and
#   never overlap; ADR-013 upper case 32 reactions -> 32 tips x 1.5 (ADR-009) = 48 = 6 sets.
#   X: the entry is the column-0 eight-channel target x -313.5 (keeps the 15 mm X end margin); the last spot -261.9 keeps the pipette
#   body (nozzle line +18) 8 mm clear of the VC-2 bracket (x -236) and, with the lower plate top at z 88, the cradle (nozzle +23 .. +45 in
#   X) passes 6 mm (5-50 tool) to 10 mm (30-300 tool) over the VC-2 camera top (z 102).  Y: nozzle-row centre on the row B centre; the tool (+/-47.5) stays 8.5 mm behind the
#   TC-1 lid guide strip end (y -111).
# V4/V5 (23 Nov 2026): TPK-S made RANDOM ACCESS.  Three groups [spot ENTRY spot]: a set is lowered through its group's entry hole, slid
# 8.0 mm to either spot and dropped 1.0 mm into a pocket (X/Y detent), stripped by the upper plate; any set can be re-picked without moving
# another.  6 spots = 4 columns (32 reactions) x 1.5 for the M300 (the only tool that parks: removal tips of repeated wash series).  The groups need
# 65.6 mm of X, B0's half place had 51.6: the X travel's left end, the left Y rail, the frame and the deck's left edge move 10 mm left
# (every size; machine 10 mm wider); the deepest spot moves 4 mm right and keeps the M300 >= 5 mm from VC-2.  Plates are cut to the
# MEASURED nozzle row of the M300 pipette (per-channel slot offsets), so channel scatter drops to the measurement error.
X_LEFT_EXT = 10.0
ROOF_RAISE = 0.0           # V6: machine_l.ROOF_Z follows the head top (LGA35 + retracted collar): the roof rises 50 mm by itself
TPKS = dict(p=8.0, pg=8.8, groups=3, zl1=88.0, t_lower=8.0, t_upper=6.0, collar_h=8.0, half=(45.0, 38.5), post=(41.0, 28.0, 8.0), tray_h=4.0,
            insert_half=(46.0, 47.0), tip="p300", place="B0", pocket_d=7.8, pocket_depth=1.0, ws=6.2, wn=6.0, entry_d=8.0, ucb_depth=0.5,
            chamfer_entry=1.0, chamfer_slot=0.5, chamfer_pocket=0.3)
_xs, _x = [], COLX[0] - 49.5 - X_LEFT_EXT
for _g in range(TPKS["groups"]):
    _xs += [("S", _x), ("E", _x + TPKS["p"]), ("S", _x + 2 * TPKS["p"])]
    _x += 2 * TPKS["p"] + TPKS["pg"]
TPKS["cx_abs"] = round((_xs[0][1] + _xs[-1][1]) / 2, 2)
TPKS["positions"] = [(k, round(v - TPKS["cx_abs"], 2)) for k, v in _xs]          # local x
TPKS["x_local"] = [v for _, v in TPKS["positions"]]
TPKS["n_pos"] = len(TPKS["positions"])
TPKS["spots_local"] = [v for k, v in TPKS["positions"] if k == "S"]
TPKS["entries_local"] = [v for k, v in TPKS["positions"] if k == "E"]
TPKS["height"] = round(TPKS["zl1"] + TPKS["collar_h"] + 0.6 + TPKS["t_upper"] - 5.0, 1)                          # above the seat (z 5)

PANEL_T = dict(M.PANEL_T)
FOOT_H = M.FOOT_H
BASE = dict(M.BASE)
MG15 = dict(M.MG15)
YC = dict(M.YC)
YB_REL = M.YB_REL
X_MARGIN = M.X_MARGIN


def tip_need_from_planner(n=32, margin=1.5):
    """build/lh1/small_runs.json, the small-batch plan ('mode') at n reactions, both phases (day 1): tips by type x margin -> wafers, towers"""
    import math as _m
    r = json.loads((HERE / "data" / f"tip_need_{n}.json").read_text())["phases"]   # frozen tip counts (public export)
    need = {}
    for ph in r.values():
        for t, v in ph.get("tips_by_type", {}).items():
            need[t] = need.get(t, 0) + v
    per, towers = {}, []
    for tip in ("p300", "p200", "p10"):
        req = _m.ceil(need.get(tip, 0) * margin)
        wafers = _m.ceil(req / 96.0)
        on_shelf = max(0, wafers - 1)                    # one wafer on the deck frame
        tw = []
        while on_shelf > 0:
            k = min(8 if tip != "p10" else 8, on_shelf)
            tw.append([tip, k])
            on_shelf -= k
        towers += tw
        per[tip] = {"need": need.get(tip, 0), "need_x1.5": req, "wafers_needed": wafers, "towers": len(tw), "held": 96 * (1 + sum(k for _, k in tw))}
    p1 = need.get("p1000", 0)
    per["p1000"] = {"need": p1, "need_x1.5": _m.ceil(p1 * margin), "wafers_or_racks_needed": 1, "held": 96, "rule_met": p1 * margin <= 96,
                    "how": "one person-loaded rack on the shelf at IG"}
    return {"samples": n, "basis": "planner build/lh1/small_runs.json, %d reactions, small-batch plan ('mode'), phases a + b (day 1), x %.1f "
                                   "(ADR-009); one wafer of each type on its deck frame" % (n, margin),
            "need": need, "need_total": sum(need.values()), "per_type": per, "towers": towers, "n_towers": len(towers),
            "held_total": sum(v["held"] for v in per.values())}


class Size:
    """every size-dependent number of one size, from its row list (CALCULATED with the machine_l stack-up rules)"""

    def __init__(self, name):
        self.name = name
        self.rows = ROWS[name]
        lay = LAYOUTS["sizes"][name]
        self.layout = {p: v["station"] for p, v in lay["places"].items()}
        self.layout.setdefault("A2", "ATR3")                # family addition (planner, 24-sample day 1): ambient small tubes on A2, every size
        # ADR-013 / ADR-014 #6: random-access tip parking TPK-S at B0 in EVERY size (integration 23 Nov 2026).  M and L had TPK-1 (LIFO,
        # 81.6 mm) at C0, which R1 does not allow behind a shallow-worked B0 (72 mm), so the firmware refused parking there (no spot
        # geometry).  TPK-1 leaves C0 (left EMPTY); the same TPK-S serves all sizes, 48 tips >= 32 x 1.5 (the parking need does not grow
        # with the size: one removal tip per sample at a time, ADR-013 upper case 32)
        self.layout["B0"] = "TPKS"
        if self.layout.get("C0") == "TPK":
            self.layout["C0"] = "EMPTY"
        if name == "S":
            # V13: the 12-column reservoir at F2 (the run sheet's choice, right for 8-32 reactions: reservoir_plan() in build.py)
            self.layout["F2"] = "RES2"
        self.shelf_plan = dict(lay["shelf"])
        self.tips_override = None
        if name == "S":
            # V15: tip supply for 32 reactions x 1.5 as the planner now counts tips: five towers; the spent-wafer stack moves to OF
            # (the H1 edge hand-off is closed on S: no S protocol uses it) and OK becomes the third p300 tower
            self.tips_override = tip_need_from_planner()
            self.shelf_plan["OF"] = "spent wafer stack"
            self.shelf_plan["OK"] = "tower"
        self.ry = {r: ROW_F_Y + i * PITCH_Y for i, r in enumerate(self.rows)}
        self.back_row = self.rows[-1]
        self.dock_y = self.ry["G"] + DOCK_BEHIND_BACK_ROW
        # travel: front = single channels at the row F front tubes (y - 28); back = gripper at the shelf slot at the dock row (+16 margin
        # as rev L); X left = carriage B at column 0 minus the margin; X right = gripper at the outer shelf column (rev L)
        self.YF0 = ROW_F_Y - 31.5 - AY - END_MARGIN          # single channels at the front wells of a deep place in row F (y -306.5)
        self.YF1 = self.dock_y - GRIP_Y + 16.0
        self.XC0 = COLX[0] - 49.5 - END_MARGIN - X_LEFT_EXT
        self.XC1 = M.XC1
        hx0, hx1 = M.HEAD_X
        self.XR_L = self.XC0 + hx0 - X_MARGIN - MG15["W"] / 2
        self.XR_R = self.XC1 + hx1 + X_MARGIN + MG15["W"] / 2
        self.BEAM = dict(M.BEAM, x0=self.XR_L - MG15["W"] / 2 - 4.5, x1=self.XR_R + MG15["W"] / 2 + 4.5)
        self.SB = dict(M.SB)
        self.Y_RAIL = (self.YF0 + YB_REL[1] - MG15["L"] / 2 - 5.0, self.YF1 + YB_REL[0] + MG15["L"] / 2 + 5.0)
        self.FY0 = (ROW_F_Y - 28.0 - AY - END_MARGIN) + M.HEAD_Y[0] - 27.0   # frame front as the study / ADR-011 (head front 28.5 mm behind the door)
        self.FY1 = self.YF1 + YC["dy"][1] + 45.0
        self.FX0, self.FX1 = self.XR_L - 20.0, self.XR_R + 20.0
        self.EX0, self.EX1 = self.XR_L - 70.0, self.XR_R + 70.0
        self.ROOF_Z = M.ROOF_Z + ROOF_RAISE
        self.DECK_X = (COLX[0] - 66.0 - X_LEFT_EXT, 335.0)
        self.DECK_Y = (ROW_F_Y - 85.0, self.ry["G"] + 70.0)
        self.DOCK_X = list(M.DOCK_X)
        self.SHELF = dict(x0=self.DECK_X[1], x1=self.XR_R - 12.0, y0=-345.0, y1=self.dock_y + 60.0, t=4.0)
        self.SLOTS = {}
        for r, y in self.ry.items():
            self.SLOTS[f"I{r}"] = (P.L_SHELF_X["inner"], y)
            self.SLOTS[f"O{r}"] = (P.L_SHELF_X["outer"], y)
        self.SLOTS["OK"] = (P.L_SHELF_X["outer"], self.dock_y)
        self.IG_RACK_X = P.L_SHELF_X["inner"] - 5.0             # p1000 rack / pocket at IG 5 mm inboard: carriage B keeps a 10 mm X end margin
        self.DOCK_RAIL_Y = (self.dock_y - 53.0, self.dock_y + 42.0)
        # the back section (row G strip, dock, back uprights) separates at the boundary in front of row G; inserted rows are bays
        self.joint_y = self.ry["G"] - 55.0
        self.bay_joints = [self.ry[r] - 55.0 for r in BAY_ROWS[name]] + [self.joint_y]     # panel / floor / roof joints, front to back
        # deck plate strips (one per row) jointed over the row-boundary bars at boundary + 7 (bar 40 x 12 on edge, clear of the
        # connector brackets under the notches, which reach boundary - 3)
        self.deck_joints = [self.ry[r] + 55.0 + 7.0 for r in self.rows[:-1]]
        self.DECK_BAR_Y = [self.DECK_Y[0] + 9.0] + self.deck_joints + [self.DECK_Y[1] - 9.0]
        # mid-side uprights: one at the back-section joint (panel strap and the extension joint), then spans <= 405 mm (rev L's longest);
        # none in the hand-off openings (y -355..-85, right side)
        y0, y1 = self.FY0 + 20.0, self.FY1 - 20.0
        ups = [self.joint_y] + [self.ry[r] - 55.0 for r in BAY_ROWS[name]]
        ups = sorted(set(round(u, 1) for u in ups))
        while True:
            pts = [y0] + ups + [y1]
            spans = [(pts[i + 1] - pts[i], i) for i in range(len(pts) - 1)]
            sp, i = max(spans)
            if sp <= 405.0:
                break
            ups = sorted(ups + [round((pts[i] + pts[i + 1]) / 2, 1)])
        self.mid_uprights = ups
        self.max_side_span = max(pts[i + 1] - pts[i] for i in range(len(pts) - 1))
        self.feet_y = [self.FY0 + 20.0] + ups + [self.FY1 - 20.0]
        self.XC, self.YF = (self.XC0, self.XC1), (self.YF0, self.YF1)
        # the study's geometry for the same size (envelope, volumes): must agree with this stack-up
        self.study = SF_JSON["geometry"][name]

    def geo_dict(self):
        return {"XC": [self.XC0, self.XC1], "YF": [self.YF0, self.YF1], "frame_y": [self.FY0, self.FY1], "dock_axis_y": self.dock_y,
                "inner_volume_gross_m3": self.study["inner_volume_gross_m3"], "inner_volume_above_deck_m3": self.study["inner_volume_above_deck_m3"],
                "enclosure_x": [self.EX0, self.EX1]}

    def summary(self):
        return {"size": self.name, "rows": self.rows, "row_y": self.ry, "dock_axis_y": self.dock_y,
                "travel_xc": [self.XC0, self.XC1], "travel_yf": [self.YF0, self.YF1], "travel_X_mm": round(self.XC1 - self.XC0, 1),
                "travel_Y_mm": round(self.YF1 - self.YF0, 1), "Y_rails_x": [round(self.XR_L, 1), round(self.XR_R, 1)],
                "X_beam_mm": round(self.BEAM["x1"] - self.BEAM["x0"], 1), "Y_rail_mm": round(self.Y_RAIL[1] - self.Y_RAIL[0], 1),
                "Y_rail_y": [round(v, 1) for v in self.Y_RAIL], "frame_x": [round(self.FX0, 1), round(self.FX1, 1)],
                "frame_y": [round(self.FY0, 1), round(self.FY1, 1)], "enclosure_x": [round(self.EX0, 1), round(self.EX1, 1)],
                "deck_x": list(self.DECK_X), "deck_y": list(self.DECK_Y), "deck_strip_joints_y": self.deck_joints,
                "deck_bars_y": self.DECK_BAR_Y, "back_section_joint_y": self.joint_y, "panel_joints_y": self.bay_joints,
                "mid_uprights_y": self.mid_uprights, "max_side_beam_span_mm": round(self.max_side_span, 1), "roof_frame_z": self.ROOF_Z,
                "beam_mid_z": ZB, "base_top_z": BASE["z1"], "foot_h": FOOT_H, "dock_bays_x": self.DOCK_X,
                "shelf": {"x": [self.SHELF["x0"], round(self.SHELF["x1"], 1)], "y": [self.SHELF["y0"], self.SHELF["y1"]],
                          "columns_x": P.L_SHELF_X, "cut_beside_the_shaker_to_x": SHELF_CUT_X}}


# =====================================================================================================
# the family head: head_l (unchanged module) + the changes the electrical owner asked for (CR-L2, CR-L5, CR-L7) and the gripper's
# second switch.  Same head in every size.  CALCULATED envelopes of ASSUMED bought parts (classes, not part numbers).
# =====================================================================================================
AX = P.L_AXIS_X
ZT0 = HL.z_top(0.0)                                   # 110: ZC top at s = 0 (carriage-local z)
BRT = HL.BR["t"]
NEMA14 = dict(a=HL.PUSHER["a"], h=HL.PUSHER["h"])     # V6: Nanotec LGA35 (head_l pusher_parts models it: guide, motor, U standoff, collar, switch)
EJECT_DX_NEW = -26.0                                  # eject finger / pin line moved from axis x -22 to -26 (4 mm) so the NEMA14 clears it
SEALED_SW = dict(x=10.2, y=6.4, z=20.0)               # sealed subminiature switch with factory leads, IP67 class (ASSUMED envelope)
HALL_D = 6.5                                          # cylindrical leaded Hall switch, 6.5 mm smooth body (ASSUMED class)
FFC = {"carriage_A": dict(w=21.0, pitch=1.0, ways=20, slot=(-160.0, -136.0), y_runs=(-125.0, -75.0), arm_from="pusher"),
       "carriage_B": dict(w=10.5, pitch=0.5, ways=20, slot=(-45.0, -32.0), y_runs=(-125.0, -75.0), arm_from="pusher"),
       "gripper": dict(w=6.5, pitch=0.5, ways=12, slot=(45.0, 58.0), y_runs=(-77.0, -27.0), arm_from="hanger")}
LOOP_R = 25.0
STROKE = P.L_Z_TRAVEL


def loop_geometry(ax):
    """rolling loop: moving clamp on the moving assembly, fixed clamp at the middle of its range, free length = stroke/2 + pi R"""
    f = FFC[ax]
    x = AX[ax]
    m_top = (ZT0 + BRT + HL.PUSHER["ge_above_bracket"] + HL.PUSHER["guide_len"] + NEMA14["h"] - 5.0) if f["arm_from"] == "pusher" else (ZT0 - 5.0)
    m0, m1 = m_top + P.Z_EJECT_TRAVEL, m_top - STROKE           # moving clamp z over the full range (eject over-travel to full stroke)
    F = (m0 + m1) / 2.0
    L = (m0 - m1) / 2.0 + math.pi * LOOP_R
    return dict(x=x, m_top=m_top, F=F, L=L, m_range=(m1, m0))


def loop_envelope_z(ax, s):
    g = loop_geometry(ax)
    m = g["m_top"] - s
    zc = (g["F"] + m + math.pi * LOOP_R - g["L"]) / 2.0
    zc = min(zc, g["F"], m)
    return zc - LOOP_R - 1.0, max(g["F"], m) + 1.0


def loop_solid(ax, s):
    f = FFC[ax]
    z0, z1 = loop_envelope_z(ax, s)
    return B(f["slot"][0] + 1.0, f["slot"][1] - 1.0, f["y_runs"][0] - 1.0, f["y_runs"][1] + 1.0, z0, z1)


def head_moving_additions(ax):
    """carriage-local solids at s = 0 that move with the axis"""
    x = AX[ax]
    zt = ZT0
    out = {}
    if ax in ("carriage_A", "carriage_B"):
        a = NEMA14["a"]
        h = HL.PUSHER["ge_above_bracket"] + HL.PUSHER["guide_len"] + NEMA14["h"] - 24.0      # so that top = zt + BRT + 24 + h is the motor top
        # latch sensing: head_l's LOCK + EMPTY switches, bracket and ladder block (latch_switch, integration 23 Nov 2026) replace the
        # single window switch the family drew here
        px, py = HL.PUSHROD_XY
        out["pushrod_switch"] = B(x + px - SEALED_SW["x"] / 2, x + px + SEALED_SW["x"] / 2, py - SEALED_SW["y"] / 2, py + SEALED_SW["y"] / 2,
                                  zt + BRT + 2.0, zt + BRT + 2.0 + SEALED_SW["z"])
        out["hall_board"] = fuse_all([cyl(x + hx, hy, zt + 1.0, zt + BRT + 14.0, HALL_D) for hx, hy in HL.HALL_XY] +
                                     [B(x + 24.0, x + 45.0, -92.0, -62.0, zt + BRT, zt + BRT + 4.0)])      # 4 leaded Hall switches + clamp plate
        top = zt + BRT + 24.0 + h
        f = FFC[ax]
        out["ffc_arm"] = B(f["slot"][0] + 1.0, x - a / 2 + 0.01, f["y_runs"][0] - 2.0, f["y_runs"][0] + 14.0, top - 10.0, top)
    else:
        gx = HL.GRIP["env"][0]
        zb = zt - HL.GRIP["drop"]
        for k, (y0_, y1_) in enumerate(((-150.0, -130.0), (-120.0, -100.0))):
            out[f"jaw_switch_{('open', 'gripped')[k]}"] = B(x + gx / 2, x + gx / 2 + SEALED_SW["y"], y0_, y1_, zb - 40.0, zb - 20.0)
        f = FFC[ax]
        out["ffc_arm"] = B(f["slot"][0] + 1.0, x - gx / 2 + 0.01, -61.0, -51.0, zt - 10.0, zt)
    return out


def head_fixed_additions():
    out = {}
    # eject fingers moved 4 mm to x axis -26 (arm, post, pin) so the NEMA14 pusher clears them
    for ax in ("carriage_A", "carriage_B"):
        x = AX[ax]
        fx, fy = x + EJECT_DX_NEW, AY
        zarm = ZT0 + BRT + 20.0
        out[f"{ax}:eject_arm"] = B(fx - 4.0, fx + 4.0, fy - 4.0, -25.0, zarm, zarm + 8.0)
        out[f"{ax}:eject_post"] = B(fx - 4.0, fx + 4.0, -33.0, -25.0, HL.CP["z"][1], zarm + 8.0)
        out[f"{ax}:eject_pin"] = cyl(fx, fy, ZT0 + BRT + HL.Z_EJ_PIN_ABOVE, zarm, 6.0)
    # FFC fixed runs: guide plates with the fixed clamp at mid-range; A beyond the CP's left end, B in the slot between A and B,
    # gripper on the CP face between B and the gripper axis
    for ax, f in FFC.items():
        g = loop_geometry(ax)
        zlo = loop_envelope_z(ax, STROKE)[0] - 5.0
        if ax == "gripper":
            out[f"ffc_guide_{ax}"] = B(f["slot"][0], f["slot"][1], -27.0, -25.0, zlo, g["F"] + 10.0)
        else:
            y = f["y_runs"][1]
            gp = B(f["slot"][0], f["slot"][1], y + 1.0, y + 4.0, zlo, g["F"] + 10.0)
            if ax == "carriage_A":
                tab = B(f["slot"][1] - 0.01, -135.0 + 0.01, y + 1.0, -25.0 + 0.01, zlo, zlo + 20.0)    # to the CP's left end
            else:
                tab = B(f["slot"][0], f["slot"][1], y + 1.0, -25.0 + 0.01, zlo - 6.0, zlo)             # to the CP face, below the loop
            out[f"ffc_guide_{ax}"] = gp.fuse(tab)
    # X cable chain moving end bracket on the carriage plate top, behind the beam's front face (CR-L1)
    out["x_chain_end_bracket"] = B(60.0, 80.0, -14.0, -11.0, 60.0, 185.0).fuse(B(60.0, 80.0, -14.0, 30.0, 182.0, 185.0))   # 3 mm bent sheet
    return out


DROP_FIXED = ("carriage_A:eject_arm", "carriage_A:eject_post", "carriage_A:eject_pin", "carriage_B:eject_arm", "carriage_B:eject_post",
              "carriage_B:eject_pin")


class FamilyHead(M.Head):
    """machine_l's cached head with the family changes; same interface (parts / bbox)"""

    def __init__(self):
        super().__init__()
        fx = head_fixed_additions()
        for k in DROP_FIXED:
            self.fixed.pop(f"fixed:{k}", None)
        for k, v in fx.items():
            self.fixed[f"fixed:{k}"] = v
        for ax in ("carriage_A", "carriage_B", "gripper"):
            for k, v in head_moving_additions(ax).items():
                self.axes[ax][f"{ax}:{k}"] = v
        for ax in ("carriage_A", "carriage_B"):            # bracket hole for the moved eject finger
            k = f"{ax}:bracket"
            self.axes[ax][k] = self.axes[ax][k].cut(cyl(AX[ax] + EJECT_DX_NEW, AY, ZT0 - 1.0, ZT0 + BRT + 1.0, 9.0))
        # tools: eject pin and cradle eject plunger follow the moved eject line (-4 mm in x)
        for (ax, key), d in self.tools.items():
            for k in list(d):
                if k.endswith(":eject_pin") or k.endswith(":eject_plunger"):
                    d[k] = d[k].translate(V(-4.0, 0, 0))
        self._loop = {}
        self.bb = {}
        for dd in [self.fixed] + list(self.axes.values()) + list(self.tools.values()):
            for k, v in dd.items():
                self.bb[id(v)] = v.BoundingBox()
        for v in self.tips.values():
            self.bb[id(v)] = v.BoundingBox()

    def loop(self, ax, s):
        key = (ax, round(s))
        if key not in self._loop:
            sol = loop_solid(ax, round(s))
            self._loop[key] = sol
            self.bb[id(sol)] = sol.BoundingBox()
        return self._loop[key]

    def parts(self, xc, yf, sA=0.0, sB=0.0, sG=0.0, toolA="M300", toolB="S1000", tipsA=True, tipsB=True):
        out = super().parts(xc, yf, sA=sA, sB=sB, sG=sG, toolA=toolA, toolB=toolB, tipsA=tipsA, tipsB=tipsB)
        for ax, s in (("carriage_A", sA), ("carriage_B", sB), ("gripper", sG)):
            out.append((f"{ax}:ffc_loop", self.loop(ax, s), (xc, yf, ZB)))
        return out

    def bbox(self, v, off):
        if id(v) not in self.bb:
            self.bb[id(v)] = v.BoundingBox()
        return super().bbox(v, off)


def head_internal_check(head):
    """the family additions against the rest of the head, every axis at several strokes (designed contacts listed)"""
    designed = {("pusher_bracket", "bracket"), ("pusher", "pusher_bracket"), ("pusher_collar", "pusher_rod"), ("pusher_rod", "pusher_bracket"),
                ("hall_board", "bracket"), ("latch_switch", "bracket"), ("latch_switch", "jaw"), ("pushrod_switch", "push_rod"), ("ffc_arm", "pusher"),
                ("ffc_arm", "gripper_hanger"), ("pusher_retracted_switch", "pusher_bracket"), ("ffc_loop", "ffc_guide_carriage_A"),
                ("ffc_loop", "ffc_guide_carriage_B"), ("ffc_loop", "ffc_guide_gripper"), ("ffc_loop", "ffc_arm"), ("jaw_switch_open", "gripper_body"), ("pusher_collar", "bracket"), ("pusher_collar", "pusher"),
                ("jaw_switch_gripped", "gripper_body"), ("pusher_rod", "pusher"), ("eject_pin", "bracket"), ("pusher_collar", "pusher_bracket")}
    new_leaves = {"pusher", "pusher_bracket", "pusher_collar", "pusher_retracted_switch", "latch_switch", "pushrod_switch", "hall_board",
                  "ffc_arm", "ffc_loop", "jaw_switch_open", "jaw_switch_gripped", "eject_arm", "eject_post", "eject_pin", "ffc_guide_carriage_A",
                  "ffc_guide_carriage_B", "ffc_guide_gripper", "x_chain_end_bracket"}
    hits, n = [], 0
    for sA, sB, sG in ((0.0, 0.0, 0.0), (-12.0, -12.0, 0.0), (120.0, 0.0, 0.0), (0.0, 120.0, 0.0), (345.0, 0.0, 0.0), (0.0, 345.0, 0.0),
                       (0.0, 0.0, 200.0), (342.0, 0.0, 300.0), (200.0, 60.0, 345.0)):
        for tA, tB in (("M300", "S1000"), ("M50", "SYR")):
            pp = head.parts(0.0, 0.0, sA=sA, sB=sB, sG=sG, toolA=tA, toolB=tB, tipsA=False, tipsB=False)
            sols = [(k, v.translate(V(*off))) for k, v, off in pp]
            n += 1
            for i, (ka, a) in enumerate(sols):
                la = ka.split(":")[-1]
                for kb, b in sols[i + 1:]:
                    lb = kb.split(":")[-1]
                    if la not in new_leaves and lb not in new_leaves:
                        continue
                    same_axis = ka.split(":")[0] == kb.split(":")[0]
                    if {la, lb} == {"eject_arm", "eject_post"} and ka.split(":")[1] == kb.split(":")[1]:
                        continue                              # one bent part (arm on its post), as in head_l
                    if la == "ffc_loop" and lb == f"ffc_guide_{ka.split(':')[0]}" or lb == "ffc_loop" and la == f"ffc_guide_{kb.split(':')[0]}":
                        continue                              # the loop's fixed run lies on its guide plate
                    if same_axis and ((la, lb) in designed or (lb, la) in designed):
                        continue
                    if ":tool:" in ka and ":tool:" in kb and same_axis:
                        continue
                    ba, bb = a.BoundingBox(), b.BoundingBox()
                    if ba.xmax < bb.xmin or bb.xmax < ba.xmin or ba.ymax < bb.ymin or bb.ymax < ba.ymin or ba.zmax < bb.zmin or bb.zmax < ba.zmin:
                        continue
                    v = M.overlap_volume(a, b)
                    if v > 1.0 or v < 0:
                        hits.append({"pose": f"sA {sA:g} sB {sB:g} sG {sG:g} tools {tA}/{tB}", "a": ka, "b": kb, "mm3": round(v, 1)})
    return {"poses": n, "hits": hits}
