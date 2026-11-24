"""TC-1 on-deck thermal cycler with sliding, self-clamping heated lid -- parametric CAD + budgets.

Run from the project root:  .venv/bin/python cad/lh1/thermocycler/build.py
Out: cad/lh1/thermocycler/out/  STEP per custom part, DXF (laser parts, flat blanks), STL (printed),
     tc1_assembly_lid_open.step, tc1_assembly_lid_closed.step, report.json, tc1_protocol.yaml

Maturity: FABRICATION CANDIDATE FOR A FIRST PROTOTYPE.  Thermal, sealing and stiffness numbers are
model estimates; bought-part dimensions (block, plate, Peltiers, rail, motors) must be measured (gate G0).
"""
from __future__ import annotations

import itertools
import json
import math
import sys
from pathlib import Path

import cadquery as cq

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
sys.path.insert(0, str(HERE.parent))
import params as PR  # noqa: E402
import geometry as G  # noqa: E402
import thermal as T  # noqa: E402
from protocol import write_protocol  # noqa: E402

OUT = HERE / "out"
P = G.P
RHO = {"al": 2.70e-6, "ss": 7.9e-6, "peek": 1.32e-6, "pa12": 1.01e-6, "steel": 7.85e-6}   # kg/mm^3

# mechanics (ESTIMATES, see report)
MECH = {
    "seal_spring_rate_n_per_mm_each": 25.0, "seal_springs": 2,          # die springs around the two guide pins
    "return_spring_n_each_compressed": 5.0, "return_springs": 2,
    "clamp_motor_torque_nm": 0.30, "clamp_screw_lead_mm": 2.0, "clamp_screw_eff": 0.35,   # integration rev D: NEMA17 x 34
    "ymotor_stall_torque_nm": 0.15, "ymotor_pulley_teeth": PR.PULLEY_TEETH,
    "chain_drag_n": 3.0, "rail_friction_coeff": 0.01, "lid_accel_m_s2": 1.0,
    "well_rim_contact_mm2": 7.7,
    # integration rev F (M16): sealing-lid liner and plate-top heat path (ASSUMED values, see mechanics()['sealing_lid_rev_f'])
    "liner_e_mpa": 2.0, "liner_t_mm": 1.5, "k_silicone": 0.25, "k_al": 167.0, "k_pp": 0.12, "contact_r_m2k_w": 3e-3,
    "well_od_at_rim_mm": 6.1, "well_wall_t_mm": 0.25, "well_wall_rim_extra_mm": 1.5, "lid_set_c": 105.0,            # per well: ~pi x 6.1 mm x 0.4 mm rim land (ESTIMATE, measure)
    "vapour_dp_mpa_98c": 0.10,              # water vapour ~94 kPa + trapped-air expansion, relative to ambient
    "e_al_mpa": 69000.0,
    "mgn15h_static_roll_moment_nm_catalogue": 60.0,   # ORDER OF MAGNITUDE; verify on the chosen brand
}


def V(w) -> float:
    return sum(s.Volume() for s in w.vals()) if hasattr(w, "vals") else w.Volume()


def bb(w):
    return w.val().BoundingBox() if len(w.vals()) == 1 else cq.Compound.makeCompound(w.vals()).BoundingBox()


def env(w) -> list[float]:
    b = bb(w)
    return [round(b.xlen, 1), round(b.ylen, 1), round(b.zlen, 1)]


# ------------------------------------------------------------------ part registry
def build_parts():
    """name -> dict(w=list of solids (closed state), group, kind, process, material, t, qty, density, mass_kg)."""
    reg = {}

    def add(name, ws, group, kind, process="bought", material="", t=None, qty=None, dens=None, mass=None, check=True):
        ws = ws if isinstance(ws, list) else [ws]
        reg[name] = dict(w=ws, group=group, kind=kind, process=process, material=material, t=t,
                         qty=qty or len(ws), dens=dens, mass=mass, check=check)

    # fixed, custom
    add("TC-01_top_plate", G.top_plate(), "fixed", "custom", "sheet_laser", "aluminium 5052-H32", 5.0, dens="al")
    add("TC-02_heat_sink", G.heat_sink(), "fixed", "custom", "cnc",
        "aluminium 6063-T5 extruded fin profile (160 wide, 50 fins x 1.5 @ 4 pitch) or skived; machined", None, dens="al")
    add("B_spacer_M4_OD8x7_ss", G.spacers(), "fixed", "bought", dens="ss")      # integration rev E (DFM): bought, was TC-03
    add("TC-04_fan_duct", G.duct(), "fixed", "custom", "print_mjf_pa12", "PA12 (MJF), max 75 C at sink cut-out", 2.0, dens="pa12")
    add("TC-05_electronics_tray", G.tray(), "fixed", "custom", "sheet_laser_bent", "aluminium 5052-H32", 2.0, dens="al")
    add("TC-23_ymotor_bracket", G.ymotor_bracket(), "fixed", "custom", "sheet_laser_bent", "aluminium 5052-H32", 3.0, dens="al")
    add("TC-24_idler_bracket", G.idler_bracket(), "fixed", "custom", "sheet_laser_bent", "aluminium 5052-H32", 3.0, dens="al")
    # lid group, custom
    add("TC-10_lid_plate", G.lid_plate(), "lid", "custom", "cnc", "aluminium 6061-T6, heated to 105 C", 6.0, dens="al")
    add("TC-11_hood", G.hood(), "lid", "custom", "sheet_laser_bent", "stainless 304", 1.5, dens="ss")
    add("B_peek_spacer_M3_OD8", G.peek_standoffs(), "lid", "bought", dens="peek")   # integration rev E (DFM): bought, was TC-12
    add("TC-13_lid_arm", G.arm(), "lid", "custom", "cnc", "aluminium 6061-T6", P["arm_t"], dens="al")
    # carriage group, custom
    add("TC-20_carriage_plate", G.carriage_plate(), "carriage", "custom", "cnc", "aluminium 6061-T6", P["carr_t"], dens="al")
    add("TC-21_presser", G.presser(), "presser", "custom", "cnc", "aluminium 6061-T6 (Tr8x2 nut pressed in)", P["presser_t"], dens="al")
    add("TC-22_bridge", G.bridge(), "carriage", "custom", "sheet_laser", "aluminium 5052-H32", P["bridge_t"], dens="al")
    # bought envelopes
    add("B_pcr_block", G.pcr_block(), "fixed", "bought", dens="al")
    add("B_peltier", G.tecs(), "fixed", "bought", mass=0.030)
    # integration rev E: the 0.25 mm interfaces are now solids: graphite sheet on the block side, grease film on the sink side
    add("B_tec_interface_graphite", G.tec_interfaces("block"), "fixed", "bought", mass=0.001)
    add("B_tec_interface_grease", G.tec_interfaces("sink"), "fixed", "bought", mass=0.001)
    add("B_insulation_sheet", G.insulation_sheet(), "fixed", "bought", mass=0.010)
    add("B_fan_80x25", G.fans(), "fixed", "bought", mass=0.110)
    add("B_tie_rod_M4", G.rods(), "fixed", "bought", dens="ss")
    if not getattr(PR, "REV_J", False):     # rev J: TCB-1 retired; the TC-1 block controller TEC-1163 sits on the frame TEP plate
        add("B_electronics_envelope", G.electronics(), "fixed", "bought", mass=0.20)   # integration rev D: TCB1 board + parts envelope
        add("B_tcb1_standoffs", G.tcb1_standoffs(), "fixed", "bought", mass=0.002)
    else:
        add("B_lid_beam_sensor_pair", G.beam_heads(), "lid", "bought", mass=0.006)     # rev J (CR-21): E3T-ST class pair, NOMINAL
    add("B_rail_MGN15", G.rail(), "fixed", "bought", mass=1.1 * (G.RAIL_Y[1] - G.RAIL_Y[0]) / 1000)
    add("B_ymotor_pancake", G.ymotor(), "fixed", "bought", mass=0.17)
    add("B_pulleys", G.pulleys(), "fixed", "bought", mass=0.02)
    add("B_belt_GT2_6mm", G.belt(), "fixed", "bought", qty=1, mass=0.01, check=False)
    add("REF_sealing_lid", G.sealing_lid(), "fixed", "labware", mass=0.03)   # integration rev E: on the plate, not on the heated lid
    add("B_lid_heater", G.heater(), "lid", "bought", mass=0.02)
    add("B_rail_block_MGN15H", G.rail_block(), "carriage", "bought", mass=0.20)
    add("B_guide_pins_D8", G.pins(), "carriage", "bought", dens="steel")
    add("B_clamp_motor_Tr8x2", G.clamp_motor(), "carriage", "bought", qty=1, mass=0.28)   # integration rev D: x 34 body
    add("TC-25_seal_switch_bracket", G.seal_switch_bracket(), "carriage", "custom", "print_mjf_pa12", "PA12 (MJF)", None, dens="pa12")
    add("B_seal_switch_D2F", G.seal_switch(), "carriage", "bought", mass=0.002)
    add("B_drag_chain", G.chain_envelope(G.CARR_Y_CLOSED), "chain", "bought", mass=0.05)
    add("REF_pcr_plate", G.pcr_plate(), "fixed", "labware", mass=0.014 + 96 * 50e-6)
    for r in reg.values():                       # mass is PER ITEM; totals use qty
        if r["mass"] is None:
            r["mass"] = sum(V(w) for w in r["w"]) * RHO[r["dens"]] / len(r["w"])
    return reg


def placed(reg, state: str):
    """Return name -> list of solids for state in {'closed', 'raised', 'open', 'slide_k'}"""
    if state == "closed":
        dy, dz_lid, dz_pr = 0.0, 0.0, 0.0
    elif state == "raised":
        dy, dz_lid, dz_pr = 0.0, P["lift"], P["lift"] + P["spring_stroke"]
    elif state.startswith("slide_"):
        dy, dz_lid, dz_pr = float(state[6:]) * G.TRAVEL, P["lift"], P["lift"] + P["spring_stroke"]
    else:
        dy, dz_lid, dz_pr = G.TRAVEL, P["lift"], P["lift"] + P["spring_stroke"]
    out = {}
    for n, r in reg.items():
        g = r["group"]
        if g == "fixed":
            out[n] = r["w"]
        elif g == "lid":
            out[n] = [w.translate((0, dy, dz_lid)) for w in r["w"]]
        elif g == "carriage":
            out[n] = [w.translate((0, dy, 0)) for w in r["w"]]
        elif g == "presser":
            out[n] = [w.translate((0, dy, dz_pr)) for w in r["w"]]
        elif g == "chain":
            out[n] = [G.chain_envelope(G.CARR_Y_CLOSED + dy)]
    return out


# ------------------------------------------------------------------ checks
ALLOWED_CONTACT = {frozenset(("REF_sealing_lid", "REF_pcr_plate")), frozenset(("REF_sealing_lid", "TC-10_lid_plate"))}   # rev E: liner on the rims, heated plate on the lid


def inter_vol(a, b) -> float:
    ba, bbx = a.val().BoundingBox(), b.val().BoundingBox()
    if (ba.xmax <= bbx.xmin + 1e-6 or bbx.xmax <= ba.xmin + 1e-6 or ba.ymax <= bbx.ymin + 1e-6 or
            bbx.ymax <= ba.ymin + 1e-6 or ba.zmax <= bbx.zmin + 1e-6 or bbx.zmax <= ba.zmin + 1e-6):
        return 0.0
    try:
        i = a.intersect(b)
        return sum(s.Volume() for s in i.solids().vals())
    except Exception:          # noqa: BLE001
        return -1.0


def interference(reg, pl) -> dict:
    items = [(n, i, w) for n, ws in pl.items() if reg[n]["check"] for i, w in enumerate(ws)]
    bad, n_pairs = {}, 0
    for (na, ia, wa), (nb, ib, wb) in itertools.combinations(items, 2):
        if na == nb or frozenset((na, nb)) in ALLOWED_CONTACT:
            continue
        n_pairs += 1
        v = inter_vol(wa, wb)
        if abs(v) > 1e-3:
            bad[f"{na}[{ia}] x {nb}[{ib}]"] = round(v, 3)
    return {"pairs_checked": n_pairs, "interferences_mm3": bad, "total_mm3": round(sum(abs(v) for v in bad.values()), 3)}


def max_z(pl) -> tuple[float, str]:
    best = (-1e9, "")
    for n, ws in pl.items():
        for w in ws:
            z = w.val().BoundingBox().zmax
            if z > best[0]:
                best = (z, n)
    return round(best[0], 2), best[1]


def corridor_check(reg, pl) -> dict:
    x0, x1 = G.X0 - PR.SBS_L / 2, G.X0 + PR.SBS_L / 2
    fy = PR.GRIP_OPEN / 2 + PR.GRIP_FINGER_T
    above = G.box(x0, x1, G.Y0 - fy, G.Y0 + fy, G.PLATE_TOP + 0.01, PR.MAX_LABWARE_H)
    fingers = [G.box(x0, x1, G.Y0 + s * PR.SBS_W / 2, G.Y0 + s * fy, PR.LABWARE_SEAT_Z + 0.01, PR.MAX_LABWARE_H)
               if s > 0 else G.box(x0, x1, G.Y0 - fy, G.Y0 - PR.SBS_W / 2, PR.LABWARE_SEAT_Z + 0.01, PR.MAX_LABWARE_H)
               for s in (-1, 1)]
    vol, hits = 0.0, {}
    for n, ws in pl.items():
        if n in ("REF_pcr_plate", "REF_sealing_lid"):   # rev E: the sealing lid travels with the plate
            continue
        for w in ws:
            for c in [above] + fingers:
                v = inter_vol(w, c)
                if v > 1e-3:
                    hits[n] = round(hits.get(n, 0) + v, 2)
                    vol += v
    lid_front = min(w.val().BoundingBox().ymin for w in pl["TC-11_hood"])
    arm_front = min(w.val().BoundingBox().ymin for w in pl["TC-13_lid_arm"])
    return {"clear_rectangle_xy_mm": [round(x0, 2), round(x1, 2), round(G.Y0 - fy, 2), round(G.Y0 + fy, 2)],
            "clear_rectangle_size_mm": [round(x1 - x0, 2), round(2 * fy, 2)],
            "z_range_mm": [PR.LABWARE_SEAT_Z, PR.MAX_LABWARE_H],
            "intersection_mm3": round(vol, 3), "hits": hits,
            "parked_hood_front_edge_y": round(lid_front, 2), "parked_arm_root_front_y_outside_plate_x": round(arm_front, 2),
            "margin_lid_to_rear_finger_corridor_mm": round(lid_front - (G.Y0 + fy), 2)}


def below_deck_check(pl) -> dict:
    xlim = PR.WINDOW_L / 2 + 40.0
    zmin_allowed = -(PR.DECK_T + PR.UNDER_DECK_CLEAR)
    deck = G.box(G.X0 - 150, G.X0 + 150, G.WIN_Y0 - 60, G.WIN_Y1 + 60, -PR.DECK_T, 0.0).cut(
        G.box(G.X0 - PR.WINDOW_L / 2, G.X0 + PR.WINDOW_L / 2, G.WIN_Y0, G.WIN_Y1, -PR.DECK_T, 0.0))
    rows, ok, dvol, ext = {}, True, 0.0, 0.0
    for n, ws in pl.items():
        for w in ws:
            b = w.val().BoundingBox()
            dv = inter_vol(w, deck)
            dvol += max(dv, 0)
            if b.zmin < -0.01 and n != "TC-01_top_plate":
                within = (abs(b.xmin - G.X0) <= xlim + 1e-6 and abs(b.xmax - G.X0) <= xlim + 1e-6 and
                          b.ymin >= G.WIN_Y0 - 1e-6 and b.ymax <= G.WIN_Y1 + 1e-6 and b.zmin >= zmin_allowed - 1e-6)
                ext = max(ext, max(G.X0 - b.xmin, b.xmax - G.X0) - PR.WINDOW_L / 2)
                ok &= within
                rows[n] = {"x": [round(b.xmin, 1), round(b.xmax, 1)], "y": [round(b.ymin, 1), round(b.ymax, 1)],
                           "z": [round(b.zmin, 1), round(b.zmax, 1)], "within_envelope": within}
    return {"all_within": ok, "window_x": [G.X0 - PR.WINDOW_L / 2, G.X0 + PR.WINDOW_L / 2], "window_y": [G.WIN_Y0, G.WIN_Y1],
            "x_extension_used_beyond_window_each_side_mm": round(ext, 2), "x_extension_allowed_mm": 40.0,
            "lowest_z_allowed": zmin_allowed, "intersection_with_deck_plate_mm3": round(dvol, 3), "parts": rows}


def mechanics(reg) -> dict:
    m = MECH
    lid_mass = sum(reg[n]["mass"] * reg[n]["qty"] for n in reg if reg[n]["group"] == "lid")
    moving_y = lid_mass + sum(reg[n]["mass"] * reg[n]["qty"] for n in reg if reg[n]["group"] in ("carriage", "presser"))
    f_spring = m["seal_springs"] * m["seal_spring_rate_n_per_mm_each"] * P["spring_stroke"]
    f_return = m["return_springs"] * m["return_spring_n_each_compressed"]
    f_seal = f_spring - f_return + lid_mass * 9.81
    f_screw_avail = 2 * math.pi * m["clamp_motor_torque_nm"] * m["clamp_screw_eff"] / (m["clamp_screw_lead_mm"] / 1000)
    f_screw_need = f_spring + f_return
    rim = 96 * m["well_rim_contact_mm2"]
    p_rim = f_seal / rim
    r_pulley = PR.PULLEY_TEETH * 2.0 / (2 * math.pi) / 1000
    f_belt = m["ymotor_stall_torque_nm"] / r_pulley
    f_need = m["chain_drag_n"] + m["rail_friction_coeff"] * moving_y * 9.81 + moving_y * m["lid_accel_m_s2"] + 1.0
    lever = (G.X0 - G.RAIL_X) / 1000
    m_roll = f_seal * lever
    # arm deflection: cantilever from the guide pins (x = RAIL_X) to the plate centre
    w_beam = P["arm_beam_y"][1] - P["arm_beam_y"][0]
    i_arm = w_beam * P["arm_t"] ** 3 / 12
    L = G.X0 - G.RAIL_X
    d_tip = f_seal * L ** 3 / (3 * m["e_al_mpa"] * i_arm)
    slope = f_seal * L ** 2 / (2 * m["e_al_mpa"] * i_arm)
    tilt_across_pad = slope * P["pad_l"]
    # integration rev F (M16): sealing with the SL-1 sealing lid.  The liner is a spring in series with the die springs:
    # solve the force/compression equilibrium instead of assuming the 0.8 mm geometric compression.
    k_l = m["liner_e_mpa"] * rim / m["liner_t_mm"]
    k_s = m["seal_springs"] * m["seal_spring_rate_n_per_mm_each"]
    d0 = P["pad_comp"]
    d_eq = (f_seal + k_s * d0) / (k_l + k_s)
    f_eq = k_l * d_eq
    # plate-top (rim) temperature, block at 98 C, heated plate at lid_set: lid -> contact -> SL-1 Al roof -> liner on the
    # rims -> PP well walls (rim to block top) -> block.  1-D series network, conductances per the stated assumptions.
    a_lid = P["lidp_l"] * P["lidp_w"] * 1e-6
    r_contact = m["contact_r_m2k_w"] / a_lid
    r_roof = (P["pad_t"] - m["liner_t_mm"]) * 1e-3 / (m["k_al"] * a_lid)
    r_liner = (m["liner_t_mm"] - d_eq) * 1e-3 / (m["k_silicone"] * rim * 1e-6)
    a_wall = 96 * math.pi * m["well_od_at_rim_mm"] * m["well_wall_t_mm"] * 1e-6
    r_wall = (P["block_top_below_plate_top"] + m["well_wall_rim_extra_mm"]) * 1e-3 / (m["k_pp"] * a_wall)
    t_lid, t_blk = m["lid_set_c"], 98.0
    q = (t_lid - t_blk) / (r_contact + r_roof + r_liner + r_wall)
    t_rim = t_lid - q * (r_contact + r_roof + r_liner)
    seal = {"liner": f"{m['liner_t_mm']} mm silicone, E {m['liner_e_mpa']} MPa (~50 Shore A, ASSUMED), on {round(rim)} mm2 of rims",
            "liner_stiffness_n_per_mm": round(k_l), "spring_rate_total_n_per_mm": k_s,
            "liner_compression_equilibrium_mm": round(d_eq, 2), "liner_compression_in_cad_mm": d0,
            "lid_stack_higher_than_cad_mm": round(d0 - d_eq, 2),
            "sealing_force_equilibrium_n": round(f_eq, 1), "sealing_force_per_well_n": round(f_eq / 96, 2),
            "rim_pressure_mpa": round(f_eq / rim, 3), "rim_pressure_over_vapour_dp": round(f_eq / rim / m["vapour_dp_mpa_98c"], 2),
            "plate_top_rim_temperature_c_block98_lid105": round(t_rim, 1),
            "heat_through_plate_w": round(q, 2),
            "thermal_resistances_k_per_w": {"lid_to_SL1_contact": round(r_contact, 3), "SL1_roof": round(r_roof, 4),
                                            "liner_on_rims": round(r_liner, 2), "pp_well_walls_rim_to_block": round(r_wall, 1)},
            "condensation_risk": "rim above the block temperature: no condensation on the liner" if t_rim > t_blk + 2 else "CHECK",
            "basis": "ESTIMATES: silicone E and k, contact resistance, PP wall path; measure seal force and in-well/rim temperature (T-TC-05)"}
    return {
        "sealing_lid_rev_f": seal,
        "lid_travel_mm": G.TRAVEL, "lift_stroke_mm": P["lift"], "pad_gap_when_raised_mm": round(P["lift"] - P["pad_comp"], 2),
        "moving_mass_y_kg": round(moving_y, 3), "lid_group_mass_kg": round(lid_mass, 3),
        "y_drive": "GT2 6 mm belt, 20T pulley on a NEMA17 pancake stepper (fixed at the rear of the strip)",
        "y_drive_stall_force_n": round(f_belt, 1), "y_drive_force_needed_n": round(f_need, 1),
        "y_drive_margin": round(f_belt / f_need, 2),
        "y_crush_force_limit_n": round(f_belt, 1),
        "clamp": "Tr8x2 self-locking screw (NEMA17 with integrated screw) drives a presser plate; presser pushes the lid arm "
                 "through 2 die springs; presser hard stop after a 5 mm spring stroke caps the force",
        "seal_spring_force_n": round(f_spring, 1), "sealing_force_on_plate_n": round(f_seal, 1),
        "sealing_force_kg": round(f_seal / 9.81, 1),
        "sealing_force_per_well_n": round(f_seal / 96, 2), "rim_contact_pressure_mpa": round(p_rim, 3),
        "rim_pressure_over_vapour_dp": round(p_rim / m["vapour_dp_mpa_98c"], 2),
        "clamp_screw_thrust_available_n": round(f_screw_avail), "clamp_screw_thrust_needed_n": round(f_screw_need),
        "clamp_margin": round(f_screw_avail / f_screw_need, 2),
        "max_force_on_trapped_object_n": round(f_seal, 1),
        "rail_roll_moment_nm": round(m_roll, 1),
        "rail_roll_moment_margin_vs_catalogue_estimate": round(m["mgn15h_static_roll_moment_nm_catalogue"] / m_roll, 2),
        "arm_tip_deflection_mm": round(d_tip, 3), "lid_tilt_across_pad_mm": round(tilt_across_pad, 3),
        "pad_compression_mm": P["pad_comp"],
        "seal_force_basis": "ESTIMATE: commercial lids 100-300 N; 96 rims x ~7.7 mm2 at >=3x the ~0.1 MPa vapour/air "
                            "overpressure at 98 C needs >=220 N; design 250 N spring-defined, adjustable by spring choice",
    }


# ------------------------------------------------------------------ flat blanks for bent sheet parts
def flat(w, h, t, holes=()):
    f = G.box(-w / 2, w / 2, -h / 2, h / 2, 0, t)
    for x, y, d in holes:
        f = f.cut(G.cyl_z(x, y, d, 0, t))
    return f


BEND_K = 0.33          # integration rev F (M3/M8): K factor for 5052 air bending, inner radius = t (confirm with the fabricator)


def flange_flat(h_outer, t, r=None):
    """integration rev F (M8): flat length of a 90 deg flange measured beyond the OUTER edge of the adjoining face:
    (h_outer - (r+t)) + BA - (r+t), BA = pi/2 (r + K t)"""
    r = t if r is None else r
    return h_outer - 2 * (r + t) + math.pi / 2 * (r + BEND_K * t)


def flat_blanks() -> dict:
    t = P["hood_t"]
    xi, yi, xo, yo = G.hood_dims()
    fl = flange_flat(G.HOOD_TOP - G.SKIRT_BOT, t)    # integration rev F: real development (was HOOD_TOP - SKIRT_BOT - t/2)
    top = G.box(-xo, xo, -yo, yo, 0, t)
    hood = top.union(G.box(-xo + t, xo - t, yo, yo + fl, 0, t)).union(G.box(-xo + t, xo - t, -yo - fl, -yo, 0, t)) \
        .union(G.box(xo, xo + fl, -yo + t, yo - t, 0, t)).union(G.box(-xo - fl, -xo, -yo + t, yo - t, 0, t))
    for sx in (-1, 1):
        for sy in (-1, 1):
            hood = hood.cut(G.cyl_z(sx * 52.0, sy * 32.0, 3.4, 0, t))
    if getattr(PR, "REV_J", False):     # rev J (CR-21): beam hole + 2 x M2 on the front and back skirts, at their developed positions
        b, r = G.BEAM_HEAD, t
        ba = math.pi / 2 * (r + BEND_K * t)
        for z, dx_list, d in ((G.SKIRT_BOT + b["lens_up"], (0.0,), b["hole_d"]), (G.SKIRT_BOT + b["h"] / 2, (-b["holes_dx"], b["holes_dx"]), b["m2_d"])):
            u = (G.HOOD_TOP - z) - 2 * (r + t) + ba
            for sy in (-1, 1):
                for dx in dx_list:
                    hood = hood.cut(G.cyl_z(dx, sy * (yo + u), d, 0, t))
    # integration rev F (M8): TC-23/TC-24 flats were wrong (two TC-23 motor holes fell outside the blank).  Development of
    # the L bracket: foot outer f + web outer h - 2(r+t) + BA; a web feature at height z above the foot underside sits at
    # u = f - (r+t) + BA + (z - (r+t)) from the foot's free edge.
    def l_flat(width, web_h, foot, holes, t=3.0):
        r = t
        ba = math.pi / 2 * (r + BEND_K * t)
        L = web_h + foot - 2 * (r + t) + ba
        hs = [(y, foot - (r + t) + ba + (z - (r + t)) - L / 2, d) for y, z, d in holes]
        return flat(width, L, t, hs)
    top = P["top_t"]
    br_m = l_flat(G.YMOT_Y[1] - G.YMOT_Y[0], P["pulley_z"] + 22.0 - top, -227.0 - G.TC23_FOOT_X0,
                  [(0.0, P["pulley_z"] - top, 22.5)] + [(sy * 15.5, P["pulley_z"] + sz * 15.5 - top, 3.4) for sy in (-1, 1) for sz in (-1, 1)])
    br_i = l_flat(18.0, P["pulley_z"] + 10.0 - top, -239.5 - G.CHAIN_X[0], [(0.0, P["pulley_z"] - top, 5.2)])
    # integration rev F (M8): developed width with bend allowance, and the 4 TCB1 standoff holes (were missing)
    tt = P["tray_t"]
    tray_w = P["sink_w"] + 2 * flange_flat(P["tray_flange_h"] + tt, tt)
    yc = (P["tray_y0"] + P["sink_y1"] + 1.0) / 2
    tray = flat(tray_w, P["sink_y1"] + 1.0 - P["tray_y0"], tt,
                [(dx, y - yc, 4.5) for dx, y in P["rod_xy"]] + [(x - G.X0, y - yc, 3.4) for x, y in (
                    [(x, y) for x in [G.TCB1["x"][0] + 10.0 + 25.0 * k for k in range(5)] for y in [G.TCB1["y"][0] + 9.0 + 25.0 * k for k in range(3)]]
                    if getattr(PR, "REV_J", False) else G.TCB1["holes"])])
    return {"TC-11_hood_flat": hood, "TC-23_ymotor_bracket_flat": br_m, "TC-24_idler_bracket_flat": br_i,
            "TC-05_electronics_tray_flat": tray}


# ------------------------------------------------------------------ main
def main() -> dict:
    OUT.mkdir(parents=True, exist_ok=True)
    reg = build_parts()
    rep: dict = {"module": "TC-1 on-deck thermal cycler, sliding self-clamping heated lid (LH-1)",
                 "status": "fabrication candidate for a FIRST PROTOTYPE; thermal, sealing and stiffness numbers are "
                           "model estimates; bought-part dimensions unmeasured (gate G0)",
                 "frame": "global deck frame of params.py; parts exported in the lid-closed position",
                 "interface_for_software": {
                     "plate_position_nest": "C1", "plate_centre_xy": [G.X0, G.Y0],
                     "plate_seat_z": PR.LABWARE_SEAT_Z, "plate_top_z_full_skirted": G.PLATE_TOP,
                     "block_top_z": G.BLOCK_TOP, "well_bottom_z_approx": round(G.BLOCK_TOP - P["well_depth"] + 0.3, 2),
                     "lid_park_nest": "D1 (do not place labware on D1)",
                     "gripper_finger_corridor_y": [G.Y0 - PR.GRIP_OPEN / 2 - PR.GRIP_FINGER_T, G.Y0 + PR.GRIP_OPEN / 2 + PR.GRIP_FINGER_T],
                     "note": "plate top z assumes a 16.0 mm full-skirted plate; set per plate type after measurement"},
                 "parameters_mm": {k: v for k, v in P.items()}, "parts": [], "bought_parts": [], "checks": {}}

    # exports: STEP per custom part (one instance), DXF for laser, STL for prints
    valid = {}
    for n, r in reg.items():
        for w in r["w"]:
            valid[n] = valid.get(n, True) and bool(w.val().isValid())
        if r["kind"] != "custom":
            continue
        w0 = r["w"][0]
        cq.exporters.export(w0, str(OUT / f"{n}.step"))
        if r["process"] == "print_mjf_pa12":
            cq.exporters.export(w0, str(OUT / f"{n}.stl"), tolerance=0.05, angularTolerance=0.2)
        if r["process"] == "sheet_laser":
            cq.exporters.export(w0.faces("<Z"), str(OUT / f"{n}.dxf"))
            if n == "TC-22_bridge":                      # integration rev E (DFM): pin holes cut D7.8, ream D8 F7
                import ezdxf
                doc = ezdxf.readfile(str(OUT / f"{n}.dxf"))
                doc.layers.add("REAM", color=3)
                for e in doc.modelspace().query("CIRCLE"):
                    if abs(2 * e.dxf.radius - (P["pin_d"] + 0.05)) < 0.02:
                        e.dxf.radius, e.dxf.layer = 3.9, "REAM"
                doc.modelspace().add_text("REAM: cut D7.8, ream D8 F7 (guide pins D8 h6)", height=2.5).set_placement((0, -40))
                doc.saveas(str(OUT / f"{n}.dxf"))
        rep["parts"].append({"name": n, "process": r["process"], "material": r["material"], "thickness": r["t"],
                             "qty": r["qty"], "envelope_mm": env(w0), "mass_kg": round(r["mass"], 3),
                             "valid_solid": valid[n]})
    # integration rev F (M16): SL-1 sealing lid is a made part (frame + roof, CNC Al 6061-T6, hard anodised); qty 4
    sl = G.sealing_lid()
    cq.exporters.export(sl, str(OUT / "SL-1_sealing_lid.step"))
    rep["parts"].append({"name": "SL-1_sealing_lid", "process": "cnc", "material": "aluminium 6061-T6, hard anodised", "thickness": None,
                         "qty": 4, "envelope_mm": env(sl), "mass_kg": round(sl.val().Volume() * 2.70e-6, 3),
                         "valid_solid": bool(sl.val().isValid()),
                         "note": "frame on the plate deck, 2.5 mm roof, 1.5 mm liner recess; modelled in checks as REF_sealing_lid"})
    for n, fw in flat_blanks().items():
        cq.exporters.export(fw.faces("<Z"), str(OUT / f"{n}.dxf"))
        valid[n] = bool(fw.val().isValid())

    # assemblies
    for state, fname in (("closed", "tc1_assembly_lid_closed.step"), ("open", "tc1_assembly_lid_open.step")):
        pl = placed(reg, state)
        asm = cq.Assembly(name=f"TC1_{state}")
        for n, ws in pl.items():
            col = cq.Color(0.75, 0.75, 0.8) if reg[n]["kind"] == "custom" else cq.Color(0.3, 0.3, 0.35)
            for i, w in enumerate(ws):
                asm.add(w, name=f"{n}_{i}", color=col)
        asm.save(str(OUT / fname))

    # checks
    c = rep["checks"]
    states = ["closed", "raised", "slide_0.25", "slide_0.5", "slide_0.75", "open"]
    c["max_z_by_state"] = {}
    for s in states:
        z, who = max_z(placed(reg, s))
        c["max_z_by_state"][s] = {"z_max": z, "part": who}
    c["max_labware_h"] = PR.MAX_LABWARE_H
    c["all_states_below_max_labware_h"] = all(v["z_max"] < PR.MAX_LABWARE_H for v in c["max_z_by_state"].values())
    c["margin_to_max_labware_h_mm"] = round(PR.MAX_LABWARE_H - max(v["z_max"] for v in c["max_z_by_state"].values()), 2)
    c["gantry_carriage_plate_bottom_z"] = PR.X_BEAM_MID_Z + PR.CARRIAGE_PLATE_Z[0]
    c["plate_access_lid_parked"] = corridor_check(reg, placed(reg, "open"))
    c["plate_access_lid_parked"]["pass"] = c["plate_access_lid_parked"]["intersection_mm3"] == 0.0
    c["below_deck"] = below_deck_check(placed(reg, "closed"))
    c["interference"] = {s: interference(reg, placed(reg, s)) for s in ("closed", "raised", "slide_0.5", "open")}
    c["no_interference_all_states"] = all(v["total_mm3"] == 0.0 for v in c["interference"].values())
    c["pad_compression_on_rims_mm"] = round(G.PLATE_TOP - G.PAD_BOT, 2)
    c["mechanics"] = mechanics(reg)
    c["plate_top_z_above_deck_mm"] = G.PLATE_TOP
    c["all_solids_valid"] = all(valid.values())
    c["strip_outside_nests_x"] = [G.STRIP_X0, G.STRIP_X1]
    custom_mass = sum(r["mass"] * r["qty"] for r in reg.values() if r["kind"] == "custom")
    bought_mass = sum(r["mass"] * r["qty"] for r in reg.values() if r["kind"] == "bought")
    c["mass_kg"] = {"custom_parts": round(custom_mass, 2), "bought_parts_modelled": round(bought_mass, 2),
                    "wiring_fasteners_allowance": 0.25, "total_module": round(custom_mass + bought_mass + 0.25, 2),
                    "pcr_block_from_cad": round(reg["B_pcr_block"]["mass"], 3)}

    # thermal + electrical budget
    T.TH["supply_v"] = PR.SUPPLY_V
    th = T.TH
    c_block = reg["B_pcr_block"]["mass"] * th["c_al"]
    c_extra = th["plate_mass_kg"] * th["c_pp"] + 96 * th["liquid_ul_per_well"] * 1e-6 * 4180 \
        + th["tec_n"] * th["tec_ceramic_block_side_kg_each"] * 880
    c_sink = reg["TC-02_heat_sink"]["mass"] * th["c_al"]
    c_lid = reg["TC-10_lid_plate"]["mass"] * th["c_al"] + 0.02 * 1000 + 0.3 * reg["TC-11_hood"]["mass"] * th["c_ss"]
    sink_geom = {"fin_len": P["fin_y1"] - G.sink_y0(), "base_w": P["sink_w"], "fin_h": P["fin_h"],
                 "fin_t": P["fin_t"], "pitch": P["fin_pitch"]}
    tb = T.budget(c_block, c_extra, c_sink, sink_geom, c_lid)
    tb["parameters"] = {k: v for k, v in th.items()}
    tb["heat_capacities_j_per_k"] = {"block_generic_solid": round(c_block, 1), "plate_liquid_tec_ceramics": round(c_extra, 1),
                                     "heat_sink": round(c_sink, 1), "lid": round(c_lid, 1)}
    tb["targets"] = {"heat_67_98_c_per_s": 1.5, "cool_98_67_c_per_s": 1.0}
    rs_av = tb["heat_sink"]["r_sink_to_air_k_w"]
    tb["heat_sink_verdict"] = {
        "available_r_k_w": rs_av,
        "required_r_k_w_generic_block": tb["cases"]["generic_solid_block"]["required_sink_r_k_w_for_target"],
        "required_r_k_w_thinwall_block": tb["cases"]["thinwall_pcr_block"]["required_sink_r_k_w_for_target"],
        "volume_used_mm": [P["sink_w"], round(P["sink_y1"] - G.sink_y0(), 1), P["fin_h"] + P["sink_base_t"]],
        "note": "required R keeps the AVERAGE sink temperature <= 45 C during 12-cycle PCR; peaks shown per case. "
                "Fan operating point and h are estimates (+/-40 %); measure sink temperature rise at 150 W (T-TC-04)."}
    rep["thermal_budget"] = tb
    g = tb["cases"]["generic_solid_block"]
    lid = tb["lid"]
    peak_a = g["tec_peak_electrical_w"] / PR.SUPPLY_V + lid["heater_current_a"] + th["i_fans"] + th["i_steppers_supply"] + th["i_logic"]
    t_pcr = g["pcr_12_cycle_total_min"] * 60
    avg_w = g["tec_energy_pcr_wh"] * 3600 / t_pcr + 28.0 + th["i_fans"] * 24 + 0.3 * th["i_steppers_supply"] * 24 + th["i_logic"] * 24
    rep["electrical_budget"] = {
        "supply_v": PR.SUPPLY_V,
        "tec_string": f"{th['tec_n']} modules in series on one H-bridge, PWM 20-25 kHz with LC filter, current limit {th['tec_i_limit']} A",
        "tec_peak_a": round(g["tec_peak_electrical_w"] / PR.SUPPLY_V, 1), "lid_heater_a": lid["heater_current_a"],
        "fans_a": th["i_fans"], "steppers_a": th["i_steppers_supply"], "logic_a": th["i_logic"],
        "peak_a": round(peak_a, 1), "peak_w": round(peak_a * PR.SUPPLY_V),
        "average_w_during_pcr": round(avg_w), "average_a_during_pcr": round(avg_w / PR.SUPPLY_V, 1),
        "hold_4c_w_lid_105": round(g["steady_4c_lid_105"]["tec_electrical_w"] + lid["hold_w_closed_on_block_at_4c"] + 12 + 7),
        "hold_4c_w_lid_idle_30": round(g["steady_4c_lid_idle_30"]["tec_electrical_w"] + 5 + 12 + 7),
        "exceeds_12a_shared_budget": peak_a > th["machine_shared_budget_a"],
        "recommendation": "peak exceeds 12 A: give TC-1 its own 24 V supply (>= 600 W, e.g. Mean Well LRS-600-24 or "
                          "equivalent, 25 A), common 0 V with the machine; machine supply untouched",
        "safety_chain": "block (110 C), sink (75 C) and lid (130 C) KSD301 NC bimetal cut-outs in series with the coil of a "
                        "24 V 40 A automotive relay feeding the TEC H-bridge and lid MOSFET; independent of software",
    }
    rep["bought_parts"] = bought_list()
    rep["open_risks"] = open_risks(rep)
    rep["params_change_requests"] = params_requests(c)
    rep["test_plan"] = test_plan()
    write_protocol(OUT / "tc1_protocol.yaml", rep)
    (OUT / "report.json").write_text(json.dumps(rep, indent=1, default=str))
    return rep


def bought_list() -> list[dict]:
    return [
        {"item": "96-well 0.2 mL aluminium PCR block", "spec": f"<= {P['block_l']} x {P['block_w']} x {P['block_h']} mm, flat underside "
         "(<0.03 mm), wells for skirted plates. ORDER: generic solid dry-bath block ~0.45 kg (the ramp figure for it is "
         "thermal_budget generic_solid_block). integration rev F (M17): a thin-wall block (~0.17 kg) is needed for the faster "
         "figure; no purchasable source is identified - it would be a PCR-instrument spare (find one) or a made thin-wall block", "qty": 1,
         "note": "MEASURE footprint/height/mass/well depth; drill+tap 6 x M3 x 7 underside; 2 x D1.6 x 30 thermistor bores"},
        {"item": "Peltier module, thermal-cycling rated", "spec": "50 x 50 x ~4 mm, 127 couples, Imax 14-15 A, Vmax 15-16 V, "
         "dTmax >= 66 K, rated for thermal cycling (e.g. Laird PowerCycling PCX class, Kryotherm/Ferrotec cycling grades, "
         "or Chinese 'PCR grade' with >= 50k cycle test report), lapped faces, edge-sealed, solder >= 200 C", "qty": 2,
         "note": "ordinary TEC1-127xx fatigue at solder joints under cycling: do not use for production; "
                 "if no 50x50 cycling module is available, re-layout for 3 x 40x40 (params change)"},
        {"item": "Graphite thermal sheet", "spec": "0.2 mm, through-plane >= 5 W/mK, 112 x 76 mm", "qty": 1, "note": "block side (no pump-out)"},
        {"item": "Thermal grease", "spec": "non-silicone or silicone, >= 3 W/mK", "qty": 1, "note": "sink side"},
        {"item": "Silicone sponge sheet", "spec": "closed-cell, 4.5 mm, -40..200 C, cut to TC insulation frame", "qty": 1, "note": "B_insulation_sheet"},
        {"item": "Clamp screws + disc springs + PEEK top-hat bushings", "spec": "6 x M3x30 A2, 6 x PEEK top-hat 3.2/6.8, "
         "12 x DIN 2093 8 x 3.2 x 0.4 disc springs (series pairs)", "qty": 6,
         "note": "TEC clamp ~1 MPa (5 kN total, ~830 N/screw): torque 0.5 N m (ESTIMATE), even cross pattern"},
        {"item": "Fan", "spec": "80 x 80 x 25 mm, 24 V, high speed (>= 35 CFM, >= 50 Pa), PWM + tach", "qty": 2, "note": "intake from below"},
        {"item": "Fan finger guards", "spec": "80 mm wire guard", "qty": 2, "note": ""},
        {"item": "Linear rail + block", "spec": "MGN15 rail 174 mm + MGN15H block, preload Z1", "qty": 1,
         "note": "check catalogue static roll moment >= 2 x 20 N m"},
        {"item": "Stepper, pancake", "spec": "NEMA17 x 23 mm, >= 0.15 N m, 20T GT2 pulley", "qty": 1, "note": "lid Y drive"},
        {"item": "Stepper with integrated lead screw", "spec": "NEMA17 x 34 mm, >= 0.30 N m, Tr8x2 screw 60 mm + brass nut", "qty": 1,
         "note": "clamp; integration rev D: 34 mm body (was 40) so the strip parts stay below z 127"},
        {"item": "GT2 belt + idler", "spec": "6 mm GT2, 2 x ~200 mm + loop; 20T toothed idler 5 mm bore", "qty": 1, "note": ""},
        {"item": "Guide pins + flanged bushings", "spec": "2 x D8 h6 x 75 mm hardened; 2 x iglidur/PTFE-bronze flanged 8/12", "qty": 2, "note": ""},
        {"item": "Die springs", "spec": "ISO 10243 around D8 pin, rate ~25 N/mm, free 25 mm, >= 7 mm usable stroke", "qty": 2, "note": "sealing force"},
        {"item": "Return springs", "spec": "compression, D8 pin, ~5 N at 9.5 mm length", "qty": 2, "note": ""},
        {"item": "Mini drag chain", "spec": "inner >= 6 x 8 mm, outer <= 9.5 x 12 mm, R18, 0.25 m", "qty": 1,
         "note": "lid wiring, 12 conductors (CR-11); integration rev D: was inner 5 x 6"},
        # integration rev E: the heated lid no longer carries a pad; each plate gets a gripper-placed SEALING LID
        # integration rev F (M16): no purchasable lid matching this outline/height/grip band was identified -> SL-1 is MADE
        # (custom part, CNC); only the liner is bought
        {"item": "Silicone liner sheet for SL-1", "spec": "1.5 mm silicone, ~50 Shore A, PTFE-faced one side, 200 C rated, cut to "
         "123 x 81 mm (die- or knife-cut)", "qty": 20, "note": "single-use liners, 1 per plate per run; held in the SL-1 roof recess"},
        {"item": "Lid heater", "spec": "silicone rubber or polyimide, 24 V, 80 W, 90 x 54 mm, PSA", "qty": 1, "note": "7.2 ohm"},
        {"item": "Bimetal cut-outs KSD301 NC", "spec": "110 C (block), 75 C (sink), 130 C (lid), M3 tab", "qty": 3, "note": "in relay-coil chain"},
        # integration rev E: the automotive relay became the THERM DC contactor (electronics/lh1/electrical_bom.csv, counted there)
        {"item": "Spacer, stainless, M4 clearance", "spec": "OD 8 x 7 mm, bore 4.3 (replaces custom TC-03)", "qty": 4, "note": "sink to top plate"},
        {"item": "Spacer, PEEK, M3 clearance", "spec": "OD 8 x 5.8 mm (replaces custom TC-12), 105 C lid zone", "qty": 4, "note": "lid plate to hood"},
        {"item": "Thermistors", "spec": "10 k NTC 1 %, B25/85 3435-3988, epoxy/glass bead, -55..150 C (e.g. EPCOS B57861S)", "qty": 5,
         "note": "block centre, block corner A1, sink base, lid plate, ambient"},
        {"item": "ADC module", "spec": "ADS1115 16-bit I2C", "qty": 2, "note": "ratiometric NTC dividers"},
        {"item": "Microcontroller", "spec": "Raspberry Pi Pico (RP2040) or equivalent, USB CDC serial", "qty": 1, "note": "PID + protocol"},
        {"item": "H-bridge for Peltiers", "spec": "24 V, >= 20 A continuous, PWM >= 20 kHz (e.g. Pololu G2 24v21 or Cytron MD30C)", "qty": 1,
         "note": "output filtering is on the TCB1 controller board (integration rev D: separate inductor/capacitor line removed)"},
        {"item": "MOSFET module", "spec": "logic-level, 24 V 10 A, PWM", "qty": 2, "note": "lid heater, fans"},
        {"item": "Stepper drivers", "spec": "TMC2209 modules (Vref trimmer sets hardware current limit)", "qty": 2, "note": ""},
        {"item": "Sensors", "spec": "2 x optical slot sensors (lid open/closed), 2 x microswitch (clamp at stop, clamp SEALED on TC-25), "
         "1 x IR break-beam pair 3 mm (plate-top plane under the hood)", "qty": 1, "note": ""},
        {"item": "24 V supply (separate)", "spec": ">= 600 W, 25 A, e.g. Mean Well LRS-600-24", "qty": 1, "note": "see electrical_budget"},
        # integration: + 2 x M4x10 for the strip holes (deck tapped at TC1_STRIP_TAPPED_HOLES)
        {"item": "Fasteners/spacers", "spec": "4 x M4x12 (deck), 2 x M4x10 (strip, into the deck's TC-1 strip holes), 4 x M4x16 (spacers), "
         "4 x M4 threaded rod 100 + tubes, M3 set", "qty": 1, "note": ""},
    ]


def open_risks(rep) -> list[dict]:
    g = rep["thermal_budget"]["cases"]["generic_solid_block"]
    t = rep["thermal_budget"]["cases"]["thinwall_pcr_block"]
    m = rep["checks"]["mechanics"]
    return [
        {"rank": 1, "risk": f"Ramp rate and uniformity: predicted 67->98 heating {g['ramp_67_to_98_c_per_s']} C/s (generic 0.45 kg block) / "
         f"{t['ramp_67_to_98_c_per_s']} C/s (thin-wall block) vs 1.5 target; 2 x 50x50 TECs cover ~60 % of the block "
         "underside, edge wells will lag; +/-0.5 C at 72 C is unproven", "mitigation": "thin-wall block; measure with a 96-probe "
         "or 8-probe plate (T-TC-02); edge guard heater ring if needed"},
        {"rank": 2, "risk": "Peltier lifetime under thermal cycling and availability of a 50x50 cycling-rated module",
         "mitigation": "buy rated modules with cycle data; 13 A current limit; ramp-rate limit in firmware; spare set"},
        {"rank": 3, "risk": f"Film-free sealing: {m['sealing_force_on_plate_n']} N spring force, single-sided lid tilts "
         f"~{m['lid_tilt_across_pad_mm']} mm across the pad (vs {m['pad_compression_mm']} mm compression): far-edge wells may evaporate",
         "mitigation": "evaporation test 20 uL x 96 at 98 C 30 min (T-TC-05); stiffen arm (ribs) or add +X hook if > 5 % loss"},
        {"rank": 4, "risk": f"Cantilevered single-rail lid: roll moment {m['rail_roll_moment_nm']} N m on one MGN15H block; "
         "top-plate strip beside the nests is held by only 2 x M4 at x = -244 (uplift under clamp load)",   # integration: holes now in deck + TC-01
         "mitigation": "verify catalogue moment; 2 x M4 strip screws into the deck (added at integration); measure deflection"},
        {"rank": 5, "risk": "Finger pinch: 6 mm lid stroke at 250 N; Y slide force limited only by stepper stall (~24 N)",
         "mitigation": "hood skirts enclose the pinch zone (skirt 4 mm below plate top when clamped); IR break-beam under the "
         "hood must be clear before clamping; presser hard stop caps force; firmware stall detection; label"},
        {"rank": 6, "risk": "Condensation at 4 C hold (1 h): water on block sides runs into the TEC/foam layer and onto the top plate",
         "mitigation": "RTV-seal TEC perimeter and foam to block; closed-cell foam strip in the 2 mm block/top-plate gap; "
         "lid kept closed with pad during hold; sink stays above dew point; drain path to a wick under the duct"},
        {"rank": 7, "risk": "Generic block 76 mm deep leaves 1 mm to the deck window front edge (y=16)", "mitigation": "measure block; ask for window +3 mm"},
        {"rank": 8, "risk": "Semi-skirted plates seat on the block not on the top plate: plate top differs (software must use per-plate height)",
         "mitigation": "plate-type table in firmware/host; measure"},
        {"rank": 9, "risk": "Heat-sink R and fan operating point estimated (+/-40 %); exhaust into the under-deck space may recirculate",
         "mitigation": "rear grille behind D1 (params change); measure sink rise at 150 W"},
        {"rank": 10, "risk": "Lid at 105 C beside nest C2 (1 mm gap): radiant heating of labware in C2", "mitigation": "measure C2 plate temperature"},
    ]


def params_requests(c) -> list[dict]:
    return [
        {"param": "deck margin strip", "request": f"reserve deck top x {c['strip_outside_nests_x']}, y 2..219, z 0..140 beside "
         "nests C1/D1 for the TC-1 lid guide, and add 2 x M4 tapped holes at (x=-244, y=30) and (x=-244, y=190)",
         "why": "only room for a lid guide that keeps the plate open from above; the gap to C2 is 2 mm"},
        {"param": "merged window C1+D1", "request": "confirm 118 x 188 window; +3 mm in -Y at the front preferred", "why": "block clearance 1 mm"},
        {"param": "under-deck extension", "request": f"TC-1 uses {c['below_deck']['x_extension_used_beyond_window_each_side_mm']} mm "
         "beyond the window in +/-X under the deck (allowed 40)", "why": "160 mm heat sink"},
        {"param": "rear exhaust", "request": "grille (>= 120 x 50 mm) in the machine rear panel behind D1 under the deck", "why": "~30 CFM, up to +15 K"},
        {"param": "new constants", "request": "TC1_PLATE_TOP_Z = 21.0, TC1_PARK_NEST = 'D1', TC1_NEST = 'C1'", "why": "pipetting/gripper software"},
        {"param": "SUPPLY", "request": "separate 24 V >= 600 W supply for TC-1, common 0 V", "why": "peak current > 12 A"},
        {"param": "nest C2", "request": "labware in C2 must not overhang its -X edge; lid skirt at x = -66.0", "why": "1 mm clearance"},
    ]


def test_plan() -> list[str]:
    return ["T-TC-01 block/TEC flatness and clamp torque vs contact (pressure film)",
            "T-TC-02 uniformity at 72 and 98 C with 8+ probes in water-filled wells",
            "T-TC-03 ramp 67<->98 and 72->4 measured in-well", "T-TC-04 heat-sink rise at 150 W steady",
            "T-TC-05 evaporation loss 20 uL x 96 at 98 C 30 min with the pad only",
            "T-TC-06 lid force with load cell; trapped-object test (break-beam, stall)",
            "T-TC-07 4 C hold 1 h at 60 % RH: condensate path", "T-TC-08 cut-out trip tests with software disabled"]


if __name__ == "__main__":
    r = main()
    c = r["checks"]
    print(json.dumps({k: c[k] for k in ("max_z_by_state", "all_states_below_max_labware_h", "no_interference_all_states",
                                         "all_solids_valid", "mass_kg")}, indent=1))
    print(json.dumps(c["plate_access_lid_parked"], indent=1))
    print(json.dumps({k: v for k, v in c["below_deck"].items() if k != "parts"}, indent=1))
    print(json.dumps({s: v["interferences_mm3"] for s, v in c["interference"].items()}, indent=1))
    print(json.dumps(c["mechanics"], indent=1))
    print(json.dumps(r["electrical_budget"], indent=1))
    print(json.dumps({k: {a: b for a, b in v.items() if not a.startswith("steady")} for k, v in r["thermal_budget"]["cases"].items()}, indent=1))
