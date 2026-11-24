"""LH-1 tool latch LTG-1 (verification V1, 21 Nov 2026): over-centre toggle latch on the carriage, opened and re-closed only at the
dock by one dock lift bar.  Pure calculation (no CAD); imported by cad/lh1/head_l/build.py and cad/lh1/machine_family/mechanisms.py.

Status: designed and calculated; nothing built or measured.  Bought-part figures are catalogue-class values marked UNVERIFIED unless the
source says they were read from a maker's page.

Why the old latch failed (V1): one 709 N disc-spring stack was both the holding spring and the thing the dock post had to compress a
further 7 mm through a 45 deg ramp: about 1.5 kN against a Z axis of about 160 N.  Holding force and release force were the same spring.

Principle here: HOLD by geometry, PRELOAD by a short stiff stack that is compressed only by the toggle closing, RELEASE through the
toggle's dead centre (where the stack force has almost no lever), RE-CLOSE by two light constant-force springs.
    jaw (keyhole slide, 45 deg flank under the stud cone) <- disc-stack cartridge (3 x DIN 2093 A20 in series) <- link <- crank (pivot P)
    the crank carries an arm with a roller; pushing the roller UP rotates the crank back through dead centre and opens the jaw;
    two constant-force springs pulling the arm DOWN close it; past dead centre (4 deg) the link tension holds the crank on its stop.
Pull on the tool pushes the jaw open, i.e. pulls the link, which drives the crank harder onto its stop: the lock cannot be back-driven
(fail locked: no power, no spring, no dock needed to stay locked).

Why a dock actuator and not the Z axis alone (CALCULATED argument, see `z_only_verdict`): the latch may only open after the tool rests
on the bay pins, and at that moment the carriage cannot move further down; a fixed post acts only while Z moves.  And a drop-off and a
pick-up are the same Z motion at the same bay, so a fixed post cannot tell them apart (the revision L post had this fault too: it
would re-latch the tool it had just parked).  One dock lift bar under all eight bays, raised by one bought lead-screw stepper, gives the
second motion; the Z axis only positions.
"""
from __future__ import annotations

import math

# ----------------------------------------------------------------------------------------------------------------------------------
# bought parts (catalogue class; UNVERIFIED unless stated)
# ----------------------------------------------------------------------------------------------------------------------------------
DISC = dict(name="disc spring DIN 2093 (DIN EN 16983) A20, 20 x 10.2 x 1.1, h0 0.45, l0 1.55", D=20.0, d=10.2, t=1.1, h0=0.45, l0=1.55,
            source="standard series A dimensions (UNVERIFIED: not read from a maker's page in this session); force from the Almen-Laszlo "
                   "formula below, which gives 1.52 kN at 75 % of h0 against the usual catalogue 1.5 kN class")
E_STEEL, NU = 206000.0, 0.3
LGA35 = dict(name="Nanotec LGA351S12-A-UIAP-038 captive NEMA 14 with integrated guide", lead_mm=0.6096, stroke_mm=38.1, winding_A=1.2,
             mass_kg=0.21, flange_mm=35.0, motor_len_mm=33.6, guide_d_mm=22.0, guide_len_mm=48.2, screw_tip="D5 g6 with M4 x 6",
             force_N={"1.2A_1.4mm_s": 240.8, "1.2A_5mm_s": 220.0, "0.7A_2mm_s": 140.0, "0.7A_5mm_s": 128.0, "0.7A_8mm_s": 115.0},
             end_stop_current_max_A=0.57, axial_play_mm=0.06,
             source="evidence/external/pusher_actuator_sourcing_2026-11-21.txt: lead, stroke, winding, mass, envelope and the 1.2 A 1.4 mm/s "
                    "force VERIFIED from the maker's datasheet / page; the other forces are that file's reading of the curve image and its "
                    "linear current scaling (INFERENCE, UNVERIFIED); rated (continuous) force NOT PUBLISHED: ask the maker")
CF_SPRING = dict(name="constant-force spring (coiled stainless strip on a drum), 46 N class, 10 mm strip, drum about D16 (catalogue class, "
                      "UNVERIFIED part), two per carriage, one each side of the arm", each_N=46.0, n=2, tol=0.10, extension_max_mm=150.0)
PIN = dict(d=4.0, bearing="drawn-cup needle roller bearing on a D4 hardened ground pin (HK0408 class, bought; UNVERIFIED part) at the crank "
                          "pivot and the crank pin; the cartridge pin rides in the jaw (plain)", mu=0.02)

# ----------------------------------------------------------------------------------------------------------------------------------
# geometry of the toggle (carriage-local Y/Z plane; P = crank pivot)
# ----------------------------------------------------------------------------------------------------------------------------------
G = dict(r=8.0, l=20.0, over_deg=4.0, open_deg=124.0, La=18.0, arm_closed_deg=228.0, roller_d=10.0,
         P_yz=(-150.0, 127.0),               # crank pivot (4 mm in front of the bracket face, in cheeks on the spring anchor) (carriage-local y, z at s = 0); the jaw plane mid-height is z 127
         jaw_clear_mm=10.8,                  # keyhole: D17 hole 10.8 mm from the engaged position = the jaw travel at full open (10.79), so the
                                             # hole is centred on the D16 head when the bar is at its open height (integration 23 Nov 2026: at
                                             # 10.0 the head met the hole edge during the 13 mm follow, latch stroke check); the D11 slot carries
                                             # the cone over 10.3 mm of its length on both edges
         cone_deg=45.0, mu_cone=(0.10, 0.15), mu_floor=0.10,
         n_disc=3, delta_set_mm=0.55,        # stack compression at dead centre with a seated stud (set at build with feeler gauges)
         knob_height_tol_mm=0.05)            # stud (retention knob) cone height above the tool plate, +/- (ASSUMED class, UNVERIFIED)


def disc_force(s, D=DISC["D"], d=DISC["d"], t=DISC["t"], h0=DISC["h0"]):
    """Almen-Laszlo (DIN EN 16979) force of one disc at deflection s (mm), N"""
    dl = D / d
    K1 = (1.0 / math.pi) * ((dl - 1.0) / dl) ** 2 / ((dl + 1.0) / (dl - 1.0) - 2.0 / math.log(dl))
    c = 4.0 * E_STEEL / (1.0 - NU ** 2) * t ** 4 / (K1 * D * D)
    return c * (s / t) * ((h0 / t - s / t) * (h0 / t - s / (2.0 * t)) + 1.0)


def stack_force(delta, n=G["n_disc"]):
    """n discs in series (same orientation alternating, one disc per layer): every disc deflects delta / n"""
    return disc_force(max(0.0, delta) / n)


def stack_limits(n=G["n_disc"]):
    s75 = 0.75 * DISC["h0"]
    return {"discs_in_series": n, "free_length_mm": round(n * DISC["l0"], 2), "flat_deflection_mm": round(n * DISC["h0"], 2),
            "catalogue_limit_75pct_h0_mm": round(n * s75, 3), "force_at_limit_N": round(disc_force(s75)),
            "force_flat_N": round(disc_force(DISC["h0"]))}


def slider(phi_deg):
    """slider (cartridge pin) distance from P along +Y; phi = opening angle from dead centre (crank pointing -Y at phi 0)"""
    r, l = G["r"], G["l"]
    p = math.radians(phi_deg)
    return -r * math.cos(p) + math.sqrt(l * l - (r * math.sin(p)) ** 2)


def ds_dphi(phi_deg):
    """mm per radian (virtual work: torque on the crank = link force x ds/dphi)"""
    r, l = G["r"], G["l"]
    p = math.radians(phi_deg)
    return r * math.sin(p) * (1.0 - r * math.cos(p) / math.sqrt(l * l - (r * math.sin(p)) ** 2))


def arm_angle(phi_deg):
    """arm direction (deg, Y right / Z up); the arm turns clockwise as the latch opens"""
    return G["arm_closed_deg"] - (phi_deg + G["over_deg"])


def roller_yz(phi_deg, s=0.0):
    a = math.radians(arm_angle(phi_deg))
    return (G["P_yz"][0] + G["La"] * math.cos(a), G["P_yz"][1] - s + G["La"] * math.sin(a))


def lever(phi_deg):
    """horizontal lever of a vertical force on the roller about P"""
    return G["La"] * abs(math.cos(math.radians(arm_angle(phi_deg))))


def jaw_disp(phi_deg):
    """jaw travel from its engaged position (stud seated) as the toggle opens: the stack relaxes first, then the jaw follows"""
    return max(0.0, slider(phi_deg) - slider(0.0) - G["delta_set_mm"])


def compression(phi_deg, dh=0.0):
    """stack compression with a seated stud; dh = stud cone higher than nominal (mm) adds dh/tan(cone) of compression"""
    return max(0.0, G["delta_set_mm"] + dh / math.tan(math.radians(G["cone_deg"])) - (slider(phi_deg) - slider(0.0)))


def preload_from_stack(F, mu):
    """stud preload P (pull on the tool plate) from the jaw force F through the 45 deg cone; closing (friction against) and relaxed"""
    a = math.radians(G["cone_deg"])
    close = F / ((math.sin(a) + mu * math.cos(a)) / (math.cos(a) - mu * math.sin(a)) + G["mu_floor"])
    return close, F / math.tan(a)


def spring_N(phi_deg, side="nominal"):
    """closing spring force at the roller pin (vertical, downwards): two constant-force springs, +/-10 % class tolerance"""
    f = CF_SPRING["each_N"] * CF_SPRING["n"]
    return f * (1.0 - CF_SPRING["tol"] if side == "low" else 1.0 + CF_SPRING["tol"] if side == "high" else 1.0)


def calcs(z_thrust_N=None, lift_thrust_N=None, margin=1.5):
    """forces through the whole stroke.  z_thrust_N: Z axis thrust (from electronics/lh1/system.yaml) for the Z-only comparison;
    lift_thrust_N: the dock lift's thrust at its run current (same source class)."""
    over, op = G["over_deg"], G["open_deg"]
    phis = [round(-over + k * 0.5, 2) for k in range(int((op + over) / 0.5) + 1)]
    mu_pin = PIN["mu"]
    rp = PIN["d"] / 2.0
    rows, worst_rel, worst_close = [], (0.0, None), (1e9, None)
    for ph in phis:
        F = stack_force(compression(ph))
        k = ds_dphi(ph)
        # toggle torque: + resists opening for phi < 0 (over-centre side), assists for phi > 0
        t_toggle = -F * k
        t_fric = mu_pin * rp * 2.0 * F                 # pivot + crank pin carry the link force; cartridge pin rides with the jaw
        sp = spring_N(ph)
        lev = lever(ph)
        # release: lift force at the roller so that F_lift * lev >= spring torque + toggle resistance + friction (spring at +10 %)
        f_rel = spring_N(ph, "high") + (max(0.0, t_toggle) + t_fric) / lev
        # re-latch: spring torque available (spring at -10 %) vs toggle torque to be overcome (closing compresses the stack) + friction
        need = (max(0.0, F * k) + t_fric)
        avail = spring_N(ph, "low") * lev
        ratio = avail / need if need > 1e-6 else None
        rows.append({"phi_deg": ph, "jaw_open_mm": round(jaw_disp(ph), 2), "stack_N": round(F), "lever_mm": round(lev, 1),
                     "spring_N": round(sp, 1), "release_force_at_roller_N": round(f_rel, 1),
                     "relatch_torque_need_Nmm": round(need), "relatch_torque_avail_Nmm": round(avail), "relatch_ratio": round(ratio, 2) if ratio else None})
        if f_rel > worst_rel[0]:
            worst_rel = (f_rel, ph)
        if ratio is not None and ratio < worst_close[0]:
            worst_close = (ratio, ph)
    # preload band
    F0 = stack_force(compression(-over))
    band = {}
    for dh in (-G["knob_height_tol_mm"], 0.0, G["knob_height_tol_mm"]):
        Fd = stack_force(compression(-over, dh))
        lo = min(preload_from_stack(Fd, mu)[0] for mu in G["mu_cone"])
        hi = preload_from_stack(Fd, 0.0)[1]
        band[f"stud {dh:+.2f} mm"] = {"stack_N": round(Fd), "preload_after_closing_N (mu 0.15..0.10)": round(lo), "preload_relaxed_N (no friction)": round(hi)}
    p_min = min(v["preload_after_closing_N (mu 0.15..0.10)"] for v in band.values())
    p_max = max(v["preload_relaxed_N (no friction)"] for v in band.values())
    lim = stack_limits()
    out = {
        "mechanism": "over-centre toggle (crank r %.1f, link %.0f, 4 deg past dead centre onto a stop) pulling the keyhole jaw through a 4-disc "
                     "A20 cartridge; arm %.0f mm with a D10 roller; closing by two constant-force springs; opened only by the dock lift bar" %
                     (G["r"], G["l"], G["La"]),
        "geometry": dict(G), "stack": lim, "pins": PIN, "closing_spring": CF_SPRING,
        "stack_at_locked_N": round(F0),
        "holding": {"preload_band_N": band, "preload_min_N": round(p_min), "preload_max_N": round(p_max),
                    "no_separation_up_to_N": round(p_min),
                    "locked_against_opening": "a pull on the tool pushes the jaw (+Y), which pulls the link and drives the crank harder onto its "
                                              "over-centre stop: it cannot open, whatever the pull, until a part yields",
                    "beyond_preload": "above the preload the balls lift and the stack compresses further; at the stack's catalogue limit (75 %% of "
                                      "flat, +%.2f mm) the stud carries %d N; flat at %d N; then the load passes into the hardened D4 pins (double "
                                      "shear, far above)" % (lim["catalogue_limit_75pct_h0_mm"] - G["delta_set_mm"], lim["force_at_limit_N"], lim["force_flat_N"]),
                    "power_loss": "nothing on the latch is powered; the dock lift is a lead screw (self-locking), so a power cut anywhere leaves the "
                                  "latch as it was: closed everywhere except inside a bay with the tool already on the bay pins"},
        "release": {"max_force_at_roller_N": round(worst_rel[0], 1), "at_phi_deg": worst_rel[1],
                    "what": "dock lift bar pushing the roller up: spring + toggle through dead centre + pin friction; the stack energy is given "
                            "back as the toggle passes centre"},
        "relatch": {"min_ratio_spring_to_need": round(worst_close[0], 2), "at_phi_deg": worst_close[1], "rule": margin,
                    "pass": worst_close[0] >= margin,
                    "what": "the constant-force springs close the toggle and compresses the stack when the bar is lowered with the stud seated"},
        "stroke": {"roller_rise_mm": round(roller_yz(G["open_deg"])[1] - roller_yz(-G["over_deg"])[1], 1),
                   "jaw_open_mm_at_full_open": round(jaw_disp(G["open_deg"]), 2), "jaw_needed_mm": G["jaw_clear_mm"],
                   "spring_travel_mm": round(roller_yz(G["open_deg"])[1] - roller_yz(-G["over_deg"])[1], 1),
                   "spring_limit_mm": CF_SPRING["extension_max_mm"],
                   "stack_compression_at_locked_mm": round(compression(-G["over_deg"]), 3),
                   "stack_compression_max_mm (stud +tol)": round(compression(-G["over_deg"], G["knob_height_tol_mm"]), 3),
                   "stack_catalogue_limit_mm": lim["catalogue_limit_75pct_h0_mm"]},
        "table": rows[::8] + [rows[-1]],
        "wear_points": ["jaw flank / stud cone: one sliding stroke under about 0.8 kN per latch: both hardened (jaw 58 HRC, bought knob), "
                        "grease; inspect at 1,000 exchanges (CT-5) for galling",
                        "toggle pins in DU-class bushings: replace the bushings if the latch switch window drifts",
                        "roller on the bar pad: rolling, hardened pad",
                        "balls and vee seats: Hertz contact (coupling.hertz_working); inspect after any crash",
                        "dock lift nut (anti-backlash, POM): 2 strokes per exchange"]}
    out["sensing"] = sense_table()
    if z_thrust_N:
        out["z_only_comparison"] = {"z_thrust_N": z_thrust_N, "force_needed_N": round(worst_rel[0], 1), "margin": round(z_thrust_N / worst_rel[0], 2),
                                    "force_pass": z_thrust_N / worst_rel[0] >= margin}
    if lift_thrust_N:
        out["release"]["lift_thrust_N"] = lift_thrust_N
        out["release"]["margin"] = round(lift_thrust_N / worst_rel[0], 2)
        out["release"]["pass"] = lift_thrust_N / worst_rel[0] >= margin
    out["z_only_verdict"] = (
        "NOT POSSIBLE with the Z axis alone, whatever the force: (1) the latch may only open once the tool rests on the bay pins, and then "
        "the carriage cannot move further down, so a fixed post has no motion to work with; opening it earlier drops the tool through the "
        "last millimetres; (2) a drop-off and a pick-up are the same Z motion at the same bay, so a fixed post cannot leave the tool in one "
        "case and take it in the other (revision L's post had this fault: rising after a drop-off its spring re-closed the jaw on the cone "
        "and lifted the tool again).  The force alone would now pass (z_only_comparison) because of the toggle.")
    return out


# ----------------------------------------------------------------------------------------------------------------------------------
# latch state sensing (ADR-014 integration, 23 Nov 2026): the firmware needs LOCKED / CLOSED-EMPTY / NOT CLOSED on ONE carriage-cable way
# ----------------------------------------------------------------------------------------------------------------------------------
# A locked latch and a closed empty latch have the SAME toggle angle (the crank sits on its over-centre stop in both): they differ only
# by the disc stack, which pushes the jaw a further compression(-4 deg) = 0.54 mm closed when no stud stops it.  So the state is read
# at the JAW.  Two single-edge sealed switches (no two-edge 'window' switch to set), both pressed by one flag on the jaw:
#   LOCK  switch: operates when the closing jaw reaches +0.12 mm (open side of the engaged position) and stays made all the way closed
#   EMPTY switch: operates at -0.12 mm (0.12 mm past the engaged position, toward closed)
# jaw position (0 = engaged on a nominal stud; + = open)   LOCK  EMPTY   reads
#   > +0.12 (open, opening, or closed on a cone 0.12+ high)   off   off    NOT CLOSED
#   -0.12 .. +0.12 (stud seated within its tolerance)          on    off    LOCKED
#   < -0.12 (no stud: -0.54; or a stud 0.12+ too low)          on    on     CLOSED-EMPTY (with a tool code present: a bad seat -> HOLD)
# Ladder (the firmware's one ADC way, analog bands unchanged): Rs 1.0 k in series, R0 22 k to return, LOCK switch adds 4.7 k in
# parallel with R0, EMPTY switch adds 2.0 k (was 1.5 k for the firmware's window arrangement: with single-edge switches EMPTY reads
# with LOCK also made).  Assembled tool-free: resistor and switch leads in lever splices on the switch bracket, no soldering.
SENSE = dict(lock_op_mm=0.12, empty_op_mm=-0.12, switch_class="sealed subminiature switch with pin plunger and factory lead (D2SW-P class, "
             "UNVERIFIED part): operating-point repeatability <= 0.02 mm class, over-travel >= 0.8 mm (the LOCK switch is driven 0.66 mm "
             "past its operating point when the jaw closes empty: PURCHASE CONDITION)",
             r_series=1000.0, r0=22000.0, r_lock=4700.0, r_empty=2000.0, setting="each switch is moved on its slotted holes until it just "
             "operates with the stated feeler between the flag and the jaw stop face (build step 5)")


def ladder_ohm(lock, empty, s=SENSE):
    par = [s["r0"]] + ([s["r_lock"]] if lock else []) + ([s["r_empty"]] if empty else [])
    return s["r_series"] + 1.0 / sum(1.0 / r for r in par)


def sense_table():
    """the three states as the switches and the ladder see them (CALCULATED from latch geometry)"""
    empty_jaw = -compression(-G["over_deg"])
    rows = {"LOCKED": (True, False), "CLOSED-EMPTY": (True, True), "NOT CLOSED": (False, False), "fault: EMPTY made, LOCK not (LOCK open-circuit)": (False, True)}
    return {"jaw_engaged_mm": 0.0, "jaw_closed_empty_mm": round(empty_jaw, 3), "jaw_full_open_mm": round(jaw_disp(G["open_deg"]), 2),
            "lock_switch_operates_at_mm": SENSE["lock_op_mm"], "empty_switch_operates_at_mm": SENSE["empty_op_mm"],
            "lock_overtravel_needed_mm": round(SENSE["lock_op_mm"] - empty_jaw, 2), "empty_overtravel_needed_mm": round(SENSE["empty_op_mm"] - empty_jaw, 2),
            "empty_margin_to_closed_empty_mm": round(SENSE["empty_op_mm"] - empty_jaw, 2),
            "stud_height_tolerance_mm": G["knob_height_tol_mm"],
            "ladder_ohm": {k: round(ladder_ohm(*v)) for k, v in rows.items()},
            "firmware_bands_ohm (printer.cfg, unchanged)": {"LOCKED": [3900, 6200], "CLOSED-EMPTY": [1800, 3100], "NOT CLOSED": [15000, 35000]},
            "detects": "an open or half-open jaw; a stud cone 0.12 mm or more off its nominal height (a 0.3 mm shim under the +X ball lowers the "
                       "stud 0.18 mm: CLOSED-EMPTY with a code = HOLD); NOT detected: a shim under one of the two -X balls (stud moves 0.06 mm, "
                       "inside the knob tolerance): CT-4 measures which shims the switch catches",
            "switches": SENSE["switch_class"], "set": SENSE["setting"]}


# ----------------------------------------------------------------------------------------------------------------------------------
# dock lift DLB-1 (one per dock row)
# ----------------------------------------------------------------------------------------------------------------------------------
DOCK_LIFT = dict(
    actuator="NEMA17 48 mm stepper with an integrated Tr8x2 lead screw and an anti-backlash nut (bought, the Z axes' motor and screw class; "
             "UNVERIFIED part), at the LEFT-hand end of the dock row, left of bay 1 (the head enters and leaves the row at its right end, "
             "where nothing may stand)",
    end="left",
    guides="two D8 h6 ground shafts (left of bay 1 and right of bay 8), each in one LM8UU-class bushing (24 mm) in a block hung UNDER the "
           "bar: the shaft tops stand only 2 mm above the bushing block at full stroke, 15 mm below the bar's top at bar UP (integration "
           "23 Nov 2026: the old shafts stood 15 mm above the bar at full stroke and set the dock clearance to 7.4 mm)",
    bar="DLB-F lift bar: Al 6061 20 x 12 along the whole dock row, with eight hardened steel pads (bought 8 x 16 ground flat stock, "
        "cut), one 14 mm right of each bay centre (under that bay's latch roller)",
    stroke_mm=50.0, run_current_A=1.3, speed_mm_s=5.0, home="optical slot switch at bar DOWN (NC, broken wire = not home), flag on the bar's left end",
    open_switch="sealed subminiature switch with factory lead on a post at the left end, made by a flag on the bar's front face from the "
                "bar-open height (roller rise + 2 mm) upwards; wired NO (broken wire = not at height)",
    bar_load_N=8.0,
    interleave_mm=13.0,
    why_13="after a drop-off the stud head (6 mm) must leave the 6 mm jaw downwards before the jaw may close: the bar follows the "
           "carriage up 12.5 mm (+0.5)")


def open_height_mm():
    """bar stroke (from DOWN) at which the OPEN-height switch operates: roller rise + 2 mm = 1 mm short of the full-open bar position
    (3 mm approach gap + roller rise); the firmware's DOCK_BAR_OPEN figure (system.yaml dock_lift.open_height_mm)"""
    return round(roller_yz(G["open_deg"])[1] - roller_yz(-G["over_deg"])[1] + 2.0, 1)


def lift_stroke():
    return {"roller_rise_mm": round(roller_yz(G["open_deg"])[1] - roller_yz(-G["over_deg"])[1], 1), "follow_mm": DOCK_LIFT["interleave_mm"],
            "approach_gap_mm": 3.0, "needed_mm": round(roller_yz(G["open_deg"])[1] - roller_yz(-G["over_deg"])[1] + DOCK_LIFT["interleave_mm"] + 3.0, 1),
            "stroke_mm": DOCK_LIFT["stroke_mm"]}


SEQUENCES = {
    "drop_off (carriage C carrying tool T to its bay k)": [
        "1 X/Y over bay k at the dock row (no tips), bar DOWN (home switch made); Z down to the seat stroke L_DOCK_S: the cradle lugs land "
        "on the bay pins; bay switch k must close",
        "2 bar UP by the roller rise (+3 mm approach) at 5 mm/s: the roller is lifted, the toggle passes dead centre, the stack relaxes, the "
        "jaw opens; the bar OPEN-height switch must be made and the latch must read NOT CLOSED (both switches off)",
        "3 Z up 13 mm and bar up 13 mm together, in 1 mm steps (bar first): the stud head leaves the jaw downwards with the jaw held open",
        "4 bar DOWN to home: the spring closes the empty jaw (latch reads CLOSED-EMPTY: LOCK and EMPTY switches made); Hall code must read 0000",
        "5 Z up to s 0 (single drop-off) or to the in-row stroke IN_ROW_S (exchange: the empty carriage moves along the row to the pick-up bay "
        "at that stroke, the other carriage and the gripper at s 0); leave the dock row at its right end"],
    "pick_up (empty carriage C takes tool T from bay k)": [
        "1 X/Y over bay k (no tips), bar DOWN; Z down to L_DOCK_S - 13 (the stud head is 0.5 mm below the jaw)",
        "2 bar UP by the roller rise + 3 + 13 mm: the jaw opens",
        "3 Z down 13 mm and bar down 13 mm together, in 1 mm steps (Z first): the balls seat on the vee seats, the stud head passes up "
        "through the open keyhole",
        "4 bar DOWN to home at 5 mm/s: the spring closes the toggle, the jaw draws the stud up through the cone and the stack takes the "
        "preload; latch must read LOCKED (LOCK switch made, EMPTY not); Hall code must equal T",
        "5 Z up: the tool leaves the bay pins (bay switch k must open); tip-gate nozzle check before work"],
    "guards (firmware)": [
        "bar UP only with a carriage at a bay centre (X within 0.5 mm), Z at the stroke of step 1, and either bay switch k made (drop-off) or "
        "Hall code 0000 on that carriage (pick-up); otherwise refuse",
        "the bar never moves while either carriage's X/Y moves; X/Y never move unless the bar home switch is made",
        "a latch switch outside its expected state after any step = HOLD, Z where it is"]}
