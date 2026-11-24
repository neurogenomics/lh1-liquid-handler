"""LH-1 liquid handler: the single source of interface dimensions.

Every CAD module, the electronics and the firmware configuration import from here.
Do not hard-code any of these numbers elsewhere.  Units: mm, kg, N.

Coordinate system: origin at the centre of the deck's TOP surface; +X to the operator's
right (long axis), +Y away from the operator (depth), +Z up.  Deck top is z = 0.

Status: interface specification, revision B (12 Nov 2026): FOUR tool positions (a dedicated 0.5-10 uL
pipette was added so that sub-2 uL additions are done by the machine), wider gantry to suit.  Nominal dimensions of bought
parts (rails, extrusion, motors, labware) must be confirmed on the physical parts (gate G0).
"""

# ---------------- labware ----------------
SBS_L, SBS_W = 127.76, 85.48          # ANSI/SLAS 1-2004 footprint
WELL_PITCH_96 = 9.0
A1_OFFSET_X, A1_OFFSET_Y = 14.38, 11.24   # well A1 centre from the plate's top-left corner

# ---------------- deck ----------------
# 3 columns x 4 rows of SBS nests.  Plates sit landscape (long side along X); an 8-channel tool
# lies along Y (plate rows A-H).  The gripper closes along Y on the labware's long sides, so the
# finger room is in the Y gaps and the X pitch can be tight.
DECK_L, DECK_W, DECK_T = 500.0, 500.0, 6.0     # laser-cut aluminium plate; dowel holes reamed after cutting
NEST_COLS, NEST_ROWS = 3, 4
NEST_PITCH_X, NEST_PITCH_Y = 132.0, 110.0
NEST_X = [(-(NEST_COLS - 1) / 2 + i) * NEST_PITCH_X for i in range(NEST_COLS)]      # -132, 0, 132
NEST_Y = [(-(NEST_ROWS - 1) / 2 + j) * NEST_PITCH_Y for j in range(NEST_ROWS)]      # -165, -55, 55, 165
NEST_NAMES = [[f"{'ABCD'[j]}{i + 1}" for i in range(NEST_COLS)] for j in range(NEST_ROWS)]   # row A = front
# Every nest is a window in the deck closed by an INSERT.  A plain insert carries SBS corner
# locators; a module (magnet lift, thermal block, shaker) is an insert with a body hanging below.
WINDOW_L, WINDOW_W = 118.0, 78.0               # deck cut-out, centred on the nest
INSERT_L, INSERT_W, INSERT_T = 130.0, 108.0, 5.0
INSERT_SCREWS = [(sx * 57.0, sy * 47.0) for sx in (-1, 1) for sy in (-1, 1)]   # 4 x M4, relative to nest centre
INSERT_DOWELS = [(0.0, -47.0), (0.0, 47.0)]    # 2 x D4 dowel; the +Y one sits in a slot in the insert
DOWEL_D = 4.0
LABWARE_SEAT_Z = INSERT_T                      # underside of labware sits on the insert top, z = +5
UNDER_DECK_CLEAR = 110.0                       # free height below the deck for module bodies and electronics
# Tip waste and the hand-off position are NESTS, not side zones: with tool offsets of up to +/-111 mm
# and +/-295 mm of carriage travel, nothing outside the deck is reachable by all three tools.
DEFAULT_TRASH_NEST = "A1"                      # chute insert through the window to a bin under the deck
DEFAULT_HANDOFF_NEST = "D3"                    # plain insert at an edge nest
SIDE_ZONE_LEFT = SIDE_ZONE_RIGHT = "unused (carriage over-travel only)"
# On-deck thermal-cycling module TC-1 (cad/lh1/thermocycler): plate at the centre of nest C1, sliding lid
# parks over D1.  Those two windows are merged into one 118 x 188 mm window; both nests' screw and dowel
# holes are kept for mounting.  Compiling the bulk protocol shows this module is what separates LH-1 from a
# pipetting-only robot: second-day manual steps 6 with it, 33 without (configs/lh1_tc.yaml vs configs/lh1.yaml).
THERMOCYCLER_NESTS = ("C1", "D1")
MERGED_WINDOWS = [("C1", "D1")]
# TC-1 (cad/lh1/thermocycler/out/report.json).  The plate sits in C1; the lid parks over D1, so D1 holds no labware.
TC1_NEST, TC1_PARK_NEST = "C1", "D1"
TC1_PLATE_TOP_Z = 21.0                         # full-skirted 16.0 mm plate on the block; set per plate type after measurement
TC1_LID_STRIP = {"x": (-249.0, -198.0), "y": (2.0, 219.0), "z": (0.0, 140.0)}   # deck-top volume reserved for the lid guide rail
TC1_STRIP_TAPPED_HOLES = [(-244.0, 30.0), (-244.0, 190.0)]   # M4; frame report: no conflict (2 mm outside the x = -230 beam). NOT YET in the deck plate drawing
DECK_BEAM_Y110_END_X = -47.0                   # the y = +110 deck support beam stops here on a 12 x 20 post (TC-1 body is at x -214.5..-49.5)
TC1_LID_SKIRT_X = -66.0                        # labware in C2 must not overhang its -X edge (1 mm clearance to the lid)
TC1_REAR_EXHAUST = {"min_w": 120.0, "min_h": 50.0}   # grille needed in the rear panel behind D1, below the deck

# ---------------- frame ----------------
EXTRUSION = "2040 aluminium T-slot (slot 6), 20 x 40 mm"
FRAME_L, FRAME_W = 970.0, 600.0                # outer footprint of the base rectangle (frame report, revision B)
OVERALL_DEPTH = 722.0   # rev D: with the enclosure, door handle (y -359) to rear panel (y 363); was 614 without panels  #                         # the X beam overhangs the frame back by 14 mm at the back end of travel
BASE_Z = -(DECK_T + UNDER_DECK_CLEAR + 20.0)   # z of the top face of the base extrusions
GANTRY_RAIL_Z = 332.0                          # top face of the Y-rail support beams: beam bottom 360 - 15 mm Y carriage plate - 13 mm MGN12H block
OVERALL_WIDTH = 1136.0  # rev D: with the enclosure cage and side panels; was 1076 without  #                        # Y motors, belts and chain tray outboard of the Y rails at x = +/-472
ENCLOSURE_H = 560.0                            # overall height above the deck top

# ---------------- motion ----------------
TRAVEL_X, TRAVEL_Y, TRAVEL_Z = 590.0, 400.0, 200.0
RAIL = "MGN12H linear rail"
X_RAIL_LEN, Y_RAIL_LEN, Z_RAIL_LEN = 850.0, 550.0, 300.0   # rev F: Z 300 (two blocks per Z rail)   # X: 590 travel + 235 block span = 825 used
Y_RAIL_X = (-472.0, 472.0)                     # the two Y rails (at 440 the side beams sat inside the swept carriage)
X_BEAM = "80 x 40 x 4 6063-T5 rectangular tube on edge (80 mm vertical), 980 mm, two MGN12 rails on its front face on tapped M3"   # closed section: about 4x the torsional stiffness of an open 4080 profile
MAX_TOOL_TIP_DEFLECTION_UNDER_PICKUP = 0.5     # mm at the tool tip under a 150 N upward tip-pickup load on an OUTER tool (beam bending + torsion + joints)
HEAD_MOVING_MASS_ASSUMED = 12.3                # kg: head design report, four Z axes with four tools at TOOL_MAX_MASS
BELT = "GT2, 9 mm, steel-core or glass-fibre"
PULLEY_TEETH = 20
XY_MOTOR = "NEMA17 closed-loop stepper, 1.8 deg; Y: >= 0.45 N m each; X: >= 0.65 N m (frame report: 1.05x margin at full speed with 0.45 N m and the 12.1 kg head)"
Z_SCREW = "T8 lead screw, lead 2 mm, anti-backlash nut"
Z_MOTOR = "NEMA17, 1.8 deg, >= 0.3 N m (through-shaft or coupled)"
Z_MIN_THRUST_N = 200.0                         # pickup of 8 tips needs up to ~120 N (to be measured, test T-P2-03)
MAX_SPEED_XY, MAX_ACCEL_XY = 400.0, 2000.0     # mm/s, mm/s^2 (targets)
CARRIAGE_W = 310.0                             # X carriage width along X (four tool plates)

# ---------------- X carriage interface (frame/gantry module <-> head module) ----------------
# Carriage-local frame: origin on the FRONT face of the X beam (the rail mounting face), at the
# carriage centre in X and at the beam's mid-height.  +x = global +X, +z = global +Z,
# -y = toward the operator (in front of the beam).
X_BEAM_MID_Z = 400.0                           # global z of the beam's mid-height (carriage-local z = 0)
X_RAILS_LOCAL_Z = (-20.0, 20.0)                # two MGN12 rails on the beam front face
X_BLOCKS_LOCAL_X = (-95.0, 95.0)               # two MGN12H blocks per rail (four in all)
MGN12H = {"height": 13.0, "block_w": 27.0, "block_l": 45.4, "hole_pattern": (20.0, 20.0), "thread": "M3"}
CARRIAGE_PLATE_T = 6.0                         # aluminium, laser cut; back face at local y = -13, front at y = -19
CARRIAGE_PLATE_X = (-155.0, 155.0)
CARRIAGE_PLATE_Z = (-240.0, 80.0)              # local; the bottom edge is at global z = 160, above MAX_LABWARE_H
# X belt: one GT2 loop in the X-Z plane between the beam face and the carriage plate, above the upper rail.
X_BELT_LOCAL_Y = -7.5                          # belt centre plane (9 mm belt occupies y = -13 .. -4)
X_BELT_LOWER_Z, X_BELT_UPPER_Z = 45.0, 57.7    # the carriage clamps the LOWER run; the upper run passes free
X_BELT_CLAMP_X = (-40.0, 40.0)                 # clamp screws (M3) into the carriage plate back at z = 45 +/- 8
# Z axes on the carriage plate front face: rail centre lines at local x = TOOL_X_OFFSETS; rail top at local z = +18
Z_RAIL_TOP_LOCAL_Z = 60.0   # rev F: rail top raised (was 18) so two Z blocks fit the 200 mm stroke; rail bottom stays at the plate bottom (-240)
Z_MOTORS_ABOVE_PLATE = True                    # Z motors sit above the carriage plate top edge, screws hang down

# ---------------- head ----------------
# The X carriage carries three independent Z axes side by side.  Each Z axis ends in a TOOL PLATE.
N_TOOLS = 4
TOOL_PITCH_X = 74.0                            # centre-to-centre of tool plates along X
TOOL_NAMES = ("pipette_small", "pipette_8ch_50", "gripper", "pipette_8ch_300")
TOOL_X_OFFSETS = tuple((i - (N_TOOLS - 1) / 2) * TOOL_PITCH_X for i in range(N_TOOLS))      # -111, -37, +37, +111
TOOL_PLATE_W, TOOL_PLATE_H, TOOL_PLATE_T = 62.0, 130.0, 5.0
TOOL_BOLTS = [(sx * 22.0, sy * 52.0) for sx in (-1, 1) for sy in (-1, 1)]      # 4 x M4 on the tool plate face (X, Z)
TOOL_DOWELS = [(0.0, 40.0), (0.0, -40.0)]      # 2 x D3
TOOL_MAX_MASS = 1.3                            # kg per tool
TOOL_ENVELOPE = (70.0, 110.0, 300.0)           # W (X) x D (toward -Y, in front of the plate) x H
TOOL_PLATE_BOTTOM_Z_AT_TOP = 330.0             # z of the tool plate's lower edge with the Z axis fully up
SAFE_TRANSIT_Z = 170.0                         # lowest point of any tool (incl. tip) during XY transit
# Z budget, revision C (head design report).  With TRAVEL_Z = 200 the pipette nozzle end sits at z = 230 with
# its axis fully up.  Then a 10 uL tip (32 mm) reaches z = -2, i.e. the bottom of a full-skirted PCR well
# (z = 5.8) with 7.8 mm to spare (at 243 it stopped 5 mm above that well bottom, too high to touch 10-15 uL);
# a 200/300 uL tip (51 mm) is at 179 in transit, 9 mm above SAFE_TRANSIT_Z; the bare nozzle reaches a tip
# seat at z = 47.  1000 uL tips (88 mm) are NOT supported: no tool takes them and they would hang at 142.
# SAFE_TRANSIT_Z applies to the tips actually fitted.  Any pipetting tool must be mounted to this height.
PIPETTE_NOZZLE_Z_AT_TOP = 230.0
# Fingertips 2 mm above the labware seat at full travel, so the pads cover 11.9 mm of a 14.35 mm SBS plate side
# (at 215 they covered 3.9 mm).  37 mm above SAFE_TRANSIT_Z with the axis up.
GRIPPER_FINGERTIP_Z_AT_TOP = 207.0
# Above the tool plate's top edge a tool must stay behind local y = -54 (the Z motors overhang to y = -51.7).
TOOL_KEEP_BEHIND_Y_ABOVE_PLATE = -56.0
# The head adds 2040 stiffeners behind the carriage plate below the X beam (local y -13..+7, global z 162..355),
# a board tray (local x -155..-109, y -19..+40, z 80..105) and a chain anchor (x 9..35, y -19..+25, z 64..86).
# The gantry must keep these volumes clear.

# Pipetting channels (decisions/ADR-002-pipetting-channel.md): air displacement with disposable tips,
# using factory-made hand pipettes pressed by a stepper on the tool plate.
PIPETTES = {
    "pipette_small":   {"type": "single-channel hand pipette", "range_ul": (0.5, 10.0), "channels": 1},
    "pipette_8ch_50":  {"type": "eight-channel hand pipette, along Y", "range_ul": (5.0, 50.0), "channels": 8},
    "pipette_8ch_300": {"type": "eight-channel hand pipette, along Y", "range_ul": (30.0, 300.0), "channels": 8},
}
PIPETTE_UPGRADE = "bought sensing modules (pressure + level detection) on the same tool plates and nozzle height"
MIN_TRANSFER_UL = 0.5

# ---------------- deck objects (envelopes above the deck top) ----------------
MAX_LABWARE_H = 150.0
TIP_BELOW_NOZZLE = {"p10": 32.0, "p200": 51.0, "p1000": 88.0}   # nominal; measure for the tips chosen

# ---------------- gripper ----------------
GRIP_AXIS = "Y"                                # fingers close on the labware's long sides (gap between rows: 24.5 mm)
GRIP_OPEN, GRIP_CLOSED = 92.0, 80.0            # finger gap range around SBS_W = 85.48
GRIP_FINGER_T = 6.0                            # finger thickness in Y; two fingers + clearance must fit 24.5 mm
GRIP_FORCE_N = (5.0, 20.0)
GRIP_MAX_PAYLOAD = 0.6                         # kg (carrier + filled labware)

# ---------------- electrical ----------------
SUPPLY_V = 24.0
CONTROLLER_CLASS = "32-bit 8-driver 3D-printer controller running Klipper, with a Raspberry-Pi-class host"
CAN_OR_USB_TOOLBOARD = True                    # head I/O through one small tool board on the carriage


def nest_centre(name: str) -> tuple[float, float]:
    row, col = "ABCD".index(name[0]), int(name[1:]) - 1
    return NEST_X[col], NEST_Y[row]


# ---------------- integration revision D (cad/lh1/assembly/out/params_requests.yaml, changes_rev_d.yaml) ----------------
# These values are also applied by cad/lh1/assembly/params_override.py; keeping them here makes that a no-op.
TOOL_REFERENCE_LOCAL_Y = -80.0                 # head-local y that sits at the gantry Y coordinate
CARRIAGE_PLATE_BACK_LOCAL_Y = -15.0            # 2 mm spacer XSP-1 between the X blocks and CP-1 (belt clearance)
# real nozzle / nozzle-row centre / finger centre, carriage-local (x, y); TOOL_X_OFFSETS are the Z-axis centres
TOOL_NOZZLE_XY_LOCAL = {"pipette_small": [-109.5, -80.0], "pipette_8ch_50": [-42.0, -100.0],
                        "gripper": [37.0, -104.0], "pipette_8ch_300": [106.0, -102.5]}
BEAM_FRONT_Y_WINDOW = (-120.0, 280.0)          # commanded beam-front-face travel with the real head (collision-free -123.9..282.2)
TC1_LID_TALLEST_Z = 127.0                      # strip parts after rev D (was 136); no tool below z 129 over C1 unless the lid is parked
TIP_STRIP_COMB_UNDERSIDE_Z = 55.0              # TW-4D comb at A1; works only if tip collars are wider than the slots (6.2 / 5.4 mm): MEASURE

# ---------------- integration revision E (cad/lh1/assembly/out/changes_rev_e.yaml) ----------------
# 300 uL tips for the 30-300 uL eight-channel (was given 200 uL tips).  NOMINAL: measure at stage 0 (tip length below the
# nozzle, collar OD, loaded-rack height) and update these three tables before ordering TW-4D or setting depth limits.
TIP_BELOW_NOZZLE["p300"] = 59.0
TIP_COLLAR_OD = {"p10": 5.9, "p200": 7.2, "p300": 7.4, "p1000": 9.0}          # mm, must exceed the comb slots (5.4 / 6.2)
TIP_RACK_H = {"p10": 50.0, "p200": 64.0, "p300": 72.0}                         # rack + tips above the labware seat
TOOL_TIP = {"pipette_small": "p10", "pipette_8ch_50": "p200", "pipette_8ch_300": "p300"}
# X end stop (CR-12b): flag tip at carriage-local x 155.5; the KW11 lever trips at carriage x 295.0 = position_max
X_ENDSTOP_FLAG_TIP_LOCAL_X = 155.5
X_ENDSTOP_TRIP_X = 295.0
# later-stage deck modules (cad/lh1/deck_modules2): optional, not in the base parts list
SHAKER_NEST, WASH_NEST = "B1", "B3"

# ---------------- integration revision F (cad/lh1/assembly/out/changes_rev_f.yaml) ----------------
Z_BLOCKS_PER_RAIL = 2                          # two MGN12H per Z rail, 46.4 mm apart (one-block pitch compliance dominated the budget)
# Tip pick-up and strip forces: ONE requirement per tool.  NOMINAL engineering estimates (no measurement yet): BR-1 bench rig
# measures them first (tests T-P2-03 / BR-03).  The pick-up stroke ends on the cradle force switch, set to PICKUP_FORCE_N.
PICKUP_FORCE_N = {"pipette_small": 20.0, "pipette_8ch_50": 120.0, "pipette_8ch_300": 120.0}
STRIP_FORCE_N = {"pipette_small": 15.0, "pipette_8ch_50": 120.0, "pipette_8ch_300": 120.0}
DESIGN_LOAD_N = 150.0                          # deflection and bearing design load (1.25 x the largest pick-up force)
STRIP_COMB_SAFETY = 3.0                        # comb designed for 3 x the largest strip force
# nozzle OD where the comb finger engages (just above the tip collar): NOMINAL, measure.  Comb slot = OD + 2 x 0.3 mm.
TIP_NOZZLE_OD_AT_COLLAR = {"p10": 4.5, "p200": 5.0, "p300": 5.2}
COMB_SLOT_CLEARANCE = 0.3                      # per side
COMB_MIN_CATCH = 1.0                           # collar must overlap each finger by >= 1 mm
TIP_STRIP_COMB_UNDERSIDE_Z = 54.25             # rev F: comb 3 mm thick (was 1.5): 1.5 mm to the 8ch-50 collar below and manifold above
COMB_T = 3.0                                   # rev F: TW-4D machined comb plate thickness (review M3: 3..4 mm; 8ch-50 nozzle leaves 6 mm)
COMB_RIB = (17.5, 21.5, 15.0)                  # rev F: x0, x1, depth of the rib under the comb behind the slot ends (mid-span support)
COMB_LEADIN = 0.8                              # rev F: 45 deg in-plane lead-in at each slot mouth, per side
HEAD_MOVING_MASS_ASSUMED = 13.1                # rev F: head report (two Z blocks per axis, steel stiffeners, cradle collars)

# ---------------- integration revision G (cad/lh1/assembly/out/changes_rev_g.yaml) ----------------
# M1 stated properly: lateral misalignment of the nozzle to the tip is the requirement; deflection along the stroke is taken
# up by the force-ended pick-up and is reported as compliance + the extra stroke it costs.
MAX_TIP_LATERAL_DEFLECTION = 0.5               # mm at the nozzle end, at the SPECIFIED pick-up / strip forces
MAX_TOOL_TIP_DEFLECTION_UNDER_PICKUP = MAX_TIP_LATERAL_DEFLECTION   # legacy name: now means LATERAL only (rev G)
# Tip ejection method B: a fixed finger on the carriage plate presses each pipette's own ejector button when the Z axis is
# driven above its normal top.  Button position / stroke / force are NOMINAL for this pipette class: MEASURE each pipette.
EJECTOR = {   # dx: from the handle axis (-X side, away from the plunger actuator); top: above the handle-body top (h_z1)
    "pipette_small": dict(dx=-18.0, dy=0.0, top_above_handle=22.0, d=10.0, stroke=8.0, force_N=20.0),
    "pipette_8ch_50": dict(dx=-18.0, dy=0.0, top_above_handle=22.0, d=12.0, stroke=10.0, force_N=45.0),
    "pipette_8ch_300": dict(dx=-18.0, dy=0.0, top_above_handle=22.0, d=12.0, stroke=10.0, force_N=45.0)}
Z_EJECT_GAP, Z_EJECT_OVER = 1.0, 0.5            # finger above the button at normal top; overtravel past the ejector stroke
Z_EJECT_TRAVEL = 12.0                           # >= gap + max stroke + over (11.5): Z travel above the normal top (method B)
Z_RAIL_TOP_LOCAL_Z = 60.0 + Z_EJECT_TRAVEL      # rail and blocks' top margin kept at the eject position
Z_RAIL_LEN = 300.0 + Z_EJECT_TRAVEL

# ---------------- revision G3 (cad/lh1/assembly/out/changes_rev_g3.yaml) ----------------
# R4: ADJUSTABLE MECHANICAL Z HARD STOP (part ZHS-1) on every axis that carries an eight-channel cradle (machine) and on the
# BR-1 rig Z axis.  The Z-carriage nut ledge (ZC-1) lands on an M5 stop screw in a block that sits on the ZBH-1 bearing
# housing and is located on the carriage plate by 2 x M4.  The stop is SET AT BUILD with a feeler from the measured
# clamp-to-rack contact; the CAD values below give the nominal setting.  All new names: no existing value changed.
Z_HARD_STOP_TOOLS = ("pipette_8ch_50", "pipette_8ch_300")   # machine axes fitted with ZHS-1 (R4: HP-2B over a tip rack)
Z_HARD_STOP_SET_GAP = {"machine": 3.5, "rig": 3.0}         # clamp-to-rack gap (mm) at stop contact, set at build
Z_HARD_STOP_MIN_GAP_AFTER_TOL = 2.0                         # rule: gap left after the worst-case tolerance stack
# tolerance stack between the build-time setting and any later contact (mm, +/-; ASSUMED, nothing measured).  The clamp
# position on the pipette and the stop height are captured by the build-time feeler setting, so they are not in the stack.
Z_HARD_STOP_TOL = {
    "machine": {"tip_rack_height_same_type": 0.5, "labware_seat_between_nests": 0.3, "gantry_height_over_xy_travel": 0.3,
                "setting_feeler_and_locknut": 0.1, "thermal_20K": 0.1},
    "rig": {"tip_rack_height_same_type": 0.5, "setting_feeler_and_locknut": 0.1, "thermal_20K": 0.1}}
Z_HARD_STOP_FIRMWARE_MARGIN = 1.0      # firmware depth limit this far ABOVE the stop: the stop is never touched in normal motion
Z_HARD_STOP_RANGE = (184.0, 202.0)     # stroke range over which the stop screw can be set (M5 x 30 set screw in the block)
Z_HARD_STOP_RAIL_END_S = 201.5         # setting for a tool with no clamp contact (rig small pipette): keeps the lower Z block on its rail
# BR-1 bench rig layout (rev G3: fixed comb station, plate nest and balance moved so the plate nest fits BR-3 and the balance
# stands clear of it).  Nozzle-x of each station along the rig X axis; the comb nest centre is fixed.
BR1_STATION_X = {"tip_rack": 0.0, "comb_nest_centre": 151.0, "plate_nest": 290.0, "balance": 480.0}
Z_HARD_STOP_LATERAL_BAND = 2.5         # a cradle part within this distance of the rack footprint (plan) counts as over the rack
                                       # (rack moulding/locator play, rig BR-3 Y setting by hand): it can govern the first contact
# rev G4: tolerance on the measured comb-strip stroke (mm, worst case, ASSUMED): comb/insert stack, deck at A1, gantry height at A1
Z_STRIP_STROKE_TOL = {"machine": {"TW-4D_legs": 0.05, "TW-1_sheet": 0.15, "deck_flatness_at_A1": 0.3, "gantry_height_at_A1": 0.3},
                      "rig": {"TW-4D_legs": 0.05, "TW-1_sheet": 0.15, "BR-5_carrier": 0.1}}
# ---------------- revision G5 (cad/lh1/assembly/out/changes_rev_g5.yaml) ----------------
# CHANGED VALUE: 48 mm NEMA17 Z motors (+0.08 kg each) on top of the G3/G4 additions: head report 13.56 kg (13.1 was exceeded since G3)
HEAD_MOVING_MASS_ASSUMED = 13.6

# ---------------- revision H (cad/lh1/assembly/out/changes_rev_h.yaml) ----------------
# Three more labware positions as a FRONT ROW F at y -275 (one nest pitch in front of row A): F1 pipette-only, F2 plain (all four
# tools), F3 second cold strip carrier.  The frame, Y rails, Y belts, deck and enclosure grow 110 mm toward the operator; the
# X beam, X belt and the lateral stiffness budget are unchanged.  Existing nest coordinates and names are unchanged.
FRONT_ROW = {"F1": (-132.0, -275.0), "F2": (0.0, -275.0), "F3": (132.0, -275.0)}   # nest centres (x, y)
Y_FRONT_EXT = 110.0                    # frame / Y rails / deck / enclosure extension toward the operator (new)
Y_TRAVEL_SHIFT = 55.0                  # the Y travel window moved this far toward the operator (new): beam front y -230..280
DECK_Y0 = -360.0                       # deck front edge (new); deck y -360..250
# CHANGED VALUES (rev H; every module importing params was rebuilt):
TRAVEL_Y = 510.0                       # was 400
Y_RAIL_LEN = 660.0                     # was 550
DECK_W = 610.0                         # was 500 (two-piece deck DK-1F 500 x 360 + DK-1B 500 x 250)
OVERALL_DEPTH = 722.0 + Y_FRONT_EXT    # was 722
BEAM_FRONT_Y_WINDOW = (-230.0, 280.0)  # was (-120, 280)
NEST_NAMES = NEST_NAMES + [["F1", "F2", "F3"]]   # was 4 rows A-D (row F appended; NEST_Y / NEST_ROWS keep the 4 grid rows)


def nest_centre(name: str) -> tuple[float, float]:   # CHANGED (rev H): handles the front row F
    if name in FRONT_ROW:
        return FRONT_ROW[name]
    row, col = "ABCD".index(name[0]), int(name[1:]) - 1
    return NEST_X[col], NEST_Y[row]

# ---------------------------------------------------------------------------------------------------------------------
# rev J (13 Nov 2026, ADR-003 ease of build): switches the CAD modules to the merged / bought / gauge-set variants.
# Only NEW values here; every module tests getattr(P, "REV_J", False).
REV_J = True
# rev J: ZHS-1 setting with the printed GG-1 gauge (measured with a caliper on receipt, accepted within +/-0.15 mm) in place of
# a feeler: the setting term grows from 0.1 to 0.25 mm (gauge 0.15 + lock nut 0.1); the other terms are unchanged.
Z_HARD_STOP_TOL_REV_J = {
    "machine": {"tip_rack_height_same_type": 0.5, "labware_seat_between_nests": 0.3, "gantry_height_over_xy_travel": 0.3,
                "setting_gauge_GG1_and_locknut": 0.25, "thermal_20K": 0.1},
    "rig": {"tip_rack_height_same_type": 0.5, "setting_gauge_GG1_and_locknut": 0.25, "thermal_20K": 0.1}}
MJF_PRINT_TOL = 0.2            # mm, +/- on a sintered PA12 dimension below 100 mm (supplier class value, ASSUMED); gauges are measured
GAUGE_ACCEPT_TOL = 0.15        # mm: a gauge is used only if its caliper-measured dimension is within this of nominal

# ---------------------------------------------------------------------------------------------------------------------
# rev K (14 Nov 2026, ADR-004..007): stations that remove manual actions and tips.  NEW values only.
REV_K = True
K_STATIONS = {   # nest -> station kind (rev K); nests not listed keep their rev H use
    "A2": "tip_parking", "A3": "tip_frame_p10", "B1": "weighing", "B2": "magnet_weighable", "B3": "cold",
    "C2": "tip_frame_p200", "D2": "tip_frame_p300", "D3": "lid_park_2", "F2": "balance", "F3": "shaker"}
# rev K layout notes: the shaker went to F3 (front-right corner: no tool works in front of it, its only X neighbour is the balance)
# because the head's rear parts (Z carriage, tool plate, CB-1) pass low over the position BEHIND the one being worked; the second cold
# block moved F3 -> B3.  Tip frames: p300 (72 mm) at D2 (no pipette works beside it), p200 (64) at C2 (K19 limit 64), p10 at A3.
# rev K item 6b (ADR-007 d11): tip supply shelf TSS-1 beside the deck (+X), served by the gripper only: wafer towers and a spent-wafer
# stack; the working tip frames at A3 / D2 / C2 take one refill wafer at a time.  Slot centres (global x, y).
K_TIP_SHELF_X = 312.0      # units x 251..373: clear of the 8-channel tool plates working column-3 col12 (to x 246)
K_TIP_SHELF_SLOTS = {"S1": ("tower_p300", -300.0), "S2": ("tower_p300", -195.0), "S3": ("tower_p200", -90.0), "S4": ("tower_p10", 15.0),
                     "S5": ("spent_wafers", 120.0)}
# units per stack used in rev K.  Stacks are capped at about 51 mm (4 full-skirt PCR plates): the assembly showed a 2-high tip-rack
# stack (133 mm) is hit by the eight-channel HP-2B / CB-1 and the Z carriages working deep at the neighbouring positions.
K_STACK_UNITS = {"tiprack_stackable": 1, "pcr_plate_full_skirt": 4, "deepwell_44": 1}
K_DECK_TAGS = {n: k + 1 for k, n in enumerate(["A1", "A2", "A3", "B1", "B2", "B3", "C1", "C2", "C3", "D1", "D2", "D3", "F1", "F2", "F3"])}

# ---------------------------------------------------------------------------------------------------------------------
# rev L (14 Nov 2026, ADR-010 exchange head; first pass = items 2, 5, 7 of the brief: head, stiffness, rig parts).
# Only NEW names so far: revisions H-K rebuild unchanged (no module reads REV_L yet except cad/lh1/head_l and the rig's
# REV_L block).  The K2 frame values (travel, rails, deck, envelope) will be introduced as L_* names read only when REV_L
# is true, so setting REV_L = False reproduces revision K exactly.
REV_L = True
# rev L second pass (machine_l): the dock check showed that with a 230 mm Z stroke a parked tool stands as high as the tools on
# the head (it hangs 289 mm below its tool plate), so while one carriage works in a bay the OTHER carriage's tool, the gripper
# fingers and the release posts collide.  The cure is a deeper dock seat: Z stroke 345 and the gantry raised by the same 115 mm,
# so every reach onto the deck and the head's lowest fixed point (z 190) are unchanged (cad/lh1/machine_l, report dock_clearance).
L_X_BEAM_MID_Z = 545.0          # CHANGED (rev L pass 2): was 430 (pass 1); 400 in revisions H-K (X_BEAM_MID_Z)
L_Z_TRAVEL = 345.0              # CHANGED (rev L pass 2): was 230 (pass 1); the dock seat is at stroke L_DOCK_S
L_AXIS_X = {"carriage_A": -90.0, "carriage_B": 0.0, "gripper": 90.0}   # carriage-local x of the three Z axes (study: ~80 pitch)
L_NOZZLE_TOP_Z = {"standard": 360.0, "p1000": 390.0}   # CHANGED (pass 2, +115 with the gantry): global nozzle-end z at the top of the stroke
L_COUPLING = dict(ball_d=10.0, pin_d=6.0, pin_l=20.0, R=36.0, preload_N=580.0, preload_tol=0.10)
L_X_BEAM = "100 x 50 x 5 6063-T6 rectangular tube on edge, about 1170 mm (K2): was 80 x 40 x 4 at 980"
L_YC_T = 20.0                   # Y carriage plates (was YC_T 15)
L_X_BLOCKS = "4 x MGN15H on two MGN15 rails (was MGN12H)"

# rev L pass 2: the K2 frame and deck in 3D (cad/lh1/machine_l).  NEW names only; read only by head_l and machine_l, so revisions
# H-K (frame_gantry, deck_modules, deck_k, assembly rev_h / rev_k) rebuild unchanged whatever REV_L is.
L_DOCK_S = 342.0                # Z stroke at which a tool sits in its dock bay (3 mm short of the end of travel)
L_DOCK_PITCH = 78.0             # bay pitch along X (was 72 in K2: a 72 mm tool plate would touch its neighbour; 6 mm between)
L_DOCK_X0 = -296.0              # first bay centre (8 bays: -296 .. +250): both carriages reach every bay AND the head can enter the
                                # dock row from its right end (carriage plate and camera clear of bay 8 at xc 465) and move along it
L_DOCK_Y = 472.0                # coupling-axis y of the bays: release posts 5 mm behind the carriage plate when it works at row G
L_XC_RANGE = (-311.0, 465.0)    # X carriage centre travel (gripper at column 1 needs -222; gripper at the outer shelf column +450)
L_YF_RANGE = (-230.0, 592.0)    # beam front face travel (row F front tubes -212.5; dock bays +545.5; shelf slot behind G +547)
L_COLS = {1: -132.0, 2: 0.0, 3: 132.0, 4: 264.0}                    # K2 place grid (deck_k K2_COLS / K2_ROWS)
L_ROWS = {"F": -275.0, "A": -165.0, "B": -55.0, "C": 55.0, "D": 165.0, "G": 275.0}
L_DECK_X = (-270.0, 335.0)      # deck plate (two pieces split at y 0); the TC-1 lid strip (moved 15 mm out) needs x <= -264
L_DECK_Y = (-360.0, 345.0)
L_SHELF_X = {"inner": 410.0, "outer": 540.0}                         # gripper-only side shelf columns (deck level)
L_HANDOFF = {"H1": (540.0, -275.0), "H2": (540.0, -165.0)}           # edge hand-off nests on the outer shelf column
L_MERGED_WINDOWS = [("C1", "D1"), ("F3", "F4")]                      # double places (TC-1; the module zone)
L_CONNECTOR_PLACES = ("A1", "A3", "B1", "B3", "C1", "C3", "D4", "F2", "F3")   # survey_adjustments.connectors_fitted_at (F3 = F3+F4)
L_CONNECTOR_NOTCH = (15.0, 45.0, 14.0)    # every window: notch in its +Y edge at place-local x +15..+45, 14 deep (module_standard FIX)
