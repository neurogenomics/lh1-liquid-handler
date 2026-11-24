"""Parametric CAD for the RETAIN prototype parts (CadQuery).

Run:  .venv/bin/python cad/build.py
Out:  cad/out/*.step  (solids), cad/out/*.dxf (flat parts), cad/out/*.stl (printable),
      cad/out/geometry_report.json (volumes, envelopes, checks)

Maturity of everything here: FABRICATION CANDIDATE FOR A PROTOTYPE at best.
Dimensions for purchased labware interfaces (tube pocket, SBS footprint, tip
geometry) are nominal and must be confirmed against physical parts before
ordering more than one-off prints (see prototypes/*/BUILD.md, gate G0).
"""
from __future__ import annotations

import json
import math
from pathlib import Path

import cadquery as cq

OUT = Path(__file__).resolve().parent / "out"
SBS_L, SBS_W = 127.76, 85.48          # ANSI/SLAS 1-2004 footprint
PITCH = 9.0

P = {   # ---------------- parameters (mm) ----------------
    "strip_len": 79.0, "strip_w": 17.5,
    "rs8_depth": 32.0, "rs8_id_top": 7.8, "rs8_id_bot": 7.0,
    "rc4_depth": 40.0, "rc4_id_top": 13.0, "rc4_id_bot": 12.4, "rc4_pitch": 18.0,
    "ret_t": 4.0, "rs8_funnel_h": 0.5, "rc4_funnel_h": 0.3,
    "spigot_od": 4.0, "spigot_id": 1.2, "spigot_len": 5.0,
    # mesh seat: pocket depth = mesh thickness + ring thickness - compression, so the ring is squeezed
    "mesh_t": 0.06, "ring_t": 0.30, "mesh_seat_h": 0.30, "mesh_seat_d": 8.2, "rc4_mesh_seat_d": 14.0,
    "screw_clear": 2.2,                 # M2 through-bolts with nuts under the retainer (no threads in polymer)
    "loc_pin_d": 2.0, "loc_pin_xy": (37.5, 6.0),   # two locating pins: body-retainer-dock (spigots do not locate)
    "port_d": 4.6, "tube_od": 3.0, "tube_id": 1.0, "tube_to_pinch": 12.0,
    "dock_l": 96.0, "dock_w": 34.0, "dock_h": 22.0, "pinch_slot_h": 6.0, "pinch_slot_z": 5.0,
    "tube_pocket_d": 11.1, "tube_pocket_depth": 21.0, "tube_cone_deg": 20.0, "tc_h": 26.0,
    "dowel_d": 4.0, "deck_l": 560.0, "deck_w": 420.0, "deck_t": 6.0,
}


def _tapered(r_bot: float, r_top: float, h: float) -> cq.Workplane:
    return cq.Workplane("XY").circle(r_bot).workplane(offset=h).circle(r_top).loft(combine=True)


def rs8_centres() -> list[float]:
    return [(i - 3.5) * PITCH for i in range(8)]


def rc4_centres() -> list[float]:
    # columns sit on well positions B, D, F, H so their spigots use the same manifold ports
    return [rs8_centres()[i] for i in (1, 3, 5, 7)]


def _pins() -> list[tuple[float, float]]:
    x, y = P["loc_pin_xy"]
    return [(-x, y), (x, -y)]


def rs8_body() -> cq.Workplane:
    b = cq.Workplane("XY").box(P["strip_len"], P["strip_w"], P["rs8_depth"], centered=(True, True, False))
    for x in rs8_centres():
        b = b.cut(_tapered(P["rs8_id_bot"] / 2, P["rs8_id_top"] / 2, P["rs8_depth"]).translate((x, 0, 0)))
    pts = [(x, y) for x in (-27.0, 0.0, 27.0) for y in (-6.6, 6.6)]
    b = b.faces(">Z").workplane().pushPoints(pts).hole(P["screw_clear"])
    return b.faces("<Z").workplane().pushPoints([(x, -y) for x, y in _pins()]).hole(P["loc_pin_d"], 4.0)


def rc4_body() -> cq.Workplane:
    b = cq.Workplane("XY").box(P["strip_len"], P["strip_w"], P["rc4_depth"], centered=(True, True, False))
    for x in rc4_centres():
        b = b.cut(_tapered(P["rc4_id_bot"] / 2, P["rc4_id_top"] / 2, P["rc4_depth"]).translate((x, 0, 0)))
    xs = [(rc4_centres()[i] + rc4_centres()[i + 1]) / 2 for i in range(3)] + [-37.0]
    b = b.faces(">Z").workplane().pushPoints([(x, 0) for x in xs]).hole(P["screw_clear"])
    return b.faces("<Z").workplane().pushPoints([(x, -y) for x, y in _pins()]).hole(P["loc_pin_d"], 4.0)


def retainer(centres: list[float], bore: float, seat_d: float, funnel_h: float,
             screw_pts: list[tuple[float, float]], blind: list[float] | None = None) -> cq.Workplane:
    """Lower part: mesh seat (mesh disc + compressed ring), shallow funnel and drain spigot per well.
    Occupies z = -ret_t..0; spigots hang below and take a push-fit silicone drain tube."""
    t = P["ret_t"]
    r = cq.Workplane("XY").workplane(offset=-t).box(P["strip_len"], P["strip_w"], t, centered=(True, True, False))
    for x in centres + (blind or []):
        sp = cq.Workplane("XY").workplane(offset=-t - P["spigot_len"]).circle(P["spigot_od"] / 2) \
            .extrude(P["spigot_len"]).translate((x, 0, 0))
        r = r.union(sp)
    for x in centres:
        seat = cq.Workplane("XY").workplane(offset=-P["mesh_seat_h"]).circle(seat_d / 2).extrude(P["mesh_seat_h"])
        funnel = cq.Workplane("XY").workplane(offset=-P["mesh_seat_h"] - funnel_h) \
            .circle(P["spigot_id"] / 2).workplane(offset=funnel_h).circle(bore / 2).loft(combine=True)
        drain = cq.Workplane("XY").workplane(offset=-t - P["spigot_len"]).circle(P["spigot_id"] / 2) \
            .extrude(t + P["spigot_len"] - P["mesh_seat_h"] - funnel_h + 0.01)
        for tool in (seat, funnel, drain):
            r = r.cut(tool.translate((x, 0, 0)))
    holes = cq.Workplane("XY").workplane(offset=-t).pushPoints(screw_pts + _pins()).circle(P["screw_clear"] / 2).extrude(t)
    return r.cut(holes)


def rs8_retainer() -> cq.Workplane:
    pts = [(x, y) for x in (-27.0, 0.0, 27.0) for y in (-6.6, 6.6)]
    return retainer(rs8_centres(), P["rs8_id_bot"], P["mesh_seat_d"], P["rs8_funnel_h"], pts)


def rc4_retainer() -> cq.Workplane:
    xs = [(rc4_centres()[i] + rc4_centres()[i + 1]) / 2 for i in range(3)] + [-37.0]
    blind = [rs8_centres()[i] for i in (0, 2, 4, 6)]       # solid stubs fill the unused dock ports
    return retainer(rc4_centres(), P["rc4_id_bot"], P["rc4_mesh_seat_d"], P["rc4_funnel_h"], [(x, 0) for x in xs], blind)


def dock() -> cq.Workplane:
    """Drain dock WM-2 (replaces the plenum manifold WM-1 after review).

    No plenum and no internal sump.  Each spigot carries its own silicone drain tube, which runs
    down through the dock, across a pinch slot, and out to an EXTERNAL waste trap (or a collection
    rack).  A spring-loaded pinch bar closes all eight tubes: the hold is a positive mechanical
    closure, normally closed, independent of pressure, temperature and power."""
    L, W, H = P["dock_l"], P["dock_w"], P["dock_h"]
    d = cq.Workplane("XY").box(L, W, H, centered=(True, True, False))
    for x in rs8_centres():
        d = d.cut(cq.Workplane("XY").circle(P["tube_od"] / 2 + 0.3).extrude(H).translate((x, 0, 0)))            # tube passage
        d = d.cut(cq.Workplane("XY").workplane(offset=H - P["spigot_len"] - 0.5).circle(P["port_d"] / 2)
                  .extrude(P["spigot_len"] + 0.5).translate((x, 0, 0)))                                          # spigot + tube collar
    # pinch slot: open on the +Y face, reaching 3 mm past the tube line (the far wall is the anvil)
    slot = cq.Workplane("XY").workplane(offset=P["pinch_slot_z"]).center(0, (W / 2 - 3.0) / 2 + 0.0) \
        .rect(8 * PITCH + 6.0, W / 2 + 3.0).extrude(P["pinch_slot_h"])
    d = d.cut(slot)
    d = d.faces(">Z").workplane().pushPoints(_pins()).hole(P["loc_pin_d"], 4.0)
    return d.faces("<Z").workplane().pushPoints([(-43, -8), (43, -8)]).hole(3.3, 8.0)


def pinch_bar() -> cq.Workplane:
    """Square bar with a rounded nose that squeezes the eight drain tubes against the dock's anvil wall."""
    b = cq.Workplane("XY").box(8 * PITCH + 4.0, 5.0, P["pinch_slot_h"] - 0.6, centered=(True, True, False))
    return b.edges("|X and <Y").fillet(1.2)


def carrier_frame() -> cq.Workplane:
    """SBS-footprint frame CF-A carrying up to 6 strips (RS8 or RC4) on 18 mm pitch."""
    h = 10.0
    f = cq.Workplane("XY").box(SBS_L, SBS_W, h, centered=(True, True, False))
    f = f.edges("|Z").fillet(3.0)
    win = cq.Workplane("XY").box(6 * 18.0 + 0.6, P["strip_len"] - 6.0, h, centered=(True, True, False))
    f = f.cut(win)
    ledge = cq.Workplane("XY").workplane(offset=h - 4.0).box(6 * 18.0 + 0.6, P["strip_len"] + 0.4, 4.0,
                                                           centered=(True, True, False))
    f = f.cut(ledge)
    # kinematic datum holes on the underside: one round, one slot (drawn round; see drawing note)
    return f.faces("<Z").workplane().pushPoints([(-58.0, 0.0), (58.0, 0.0)]).hole(P["dowel_d"], 5.0)


def tube_carrier() -> cq.Workplane:
    """TC24-A: 24 x 1.5 mL tubes on 18 mm pitch in an SBS footprint, hinge slots clock the caps."""
    h = P["tc_h"]
    b = cq.Workplane("XY").box(SBS_L, SBS_W, h, centered=(True, True, False)).edges("|Z").fillet(3.0)
    pts = [((c - 2.5) * 18.0, (r - 1.5) * 18.0) for c in range(6) for r in range(4)]
    b = b.faces(">Z").workplane().pushPoints(pts).hole(P["tube_pocket_d"], P["tube_pocket_depth"])
    for (x, y) in pts:
        cone_h = 4.0
        cone = cq.Workplane("XY").workplane(offset=h - P["tube_pocket_depth"] - cone_h) \
            .circle(P["tube_pocket_d"] / 2 - cone_h * math.tan(math.radians(P["tube_cone_deg"]))) \
            .workplane(offset=cone_h).circle(P["tube_pocket_d"] / 2).loft(combine=True)
        slot = cq.Workplane("XY").workplane(offset=h - 3.0).center(x, y + P["tube_pocket_d"] / 2 + 1.5) \
            .rect(5.0, 4.0).extrude(3.0)
        b = b.cut(cone.translate((x, y, 0))).cut(slot)
    # gripper flats: 2 mm deep recess on the long sides, 10..18 mm above base
    for s in (-1, 1):
        rec = cq.Workplane("XY").workplane(offset=10.0).center(0, s * (SBS_W / 2 - 1.0)) \
            .rect(60.0, 2.0).extrude(8.0)
        b = b.cut(rec)
    return b.faces("<Z").workplane().pushPoints([(-58.0, 0.0), (58.0, 0.0)]).hole(P["dowel_d"], 5.0)


def deck_plate() -> tuple[cq.Workplane, list[dict]]:
    """Flat deck: 3 x 3 SBS nests, each located by two dowel pins and held by four M4."""
    d = cq.Workplane("XY").box(P["deck_l"], P["deck_w"], P["deck_t"], centered=(True, True, False))
    nests, dowels, screws = [], [], []
    for i in range(3):
        for j in range(3):
            cx, cy = (i - 1) * 170.0, (j - 1) * 125.0
            nests.append({"slot": f"{'ABC'[j]}{i + 1}", "x": cx, "y": cy})
            dowels += [(cx - 58.0, cy), (cx + 58.0, cy)]
            screws += [(cx + sx * 70.0, cy + sy * 50.0) for sx in (-1, 1) for sy in (-1, 1)]
    d = d.faces(">Z").workplane().pushPoints(dowels).hole(P["dowel_d"])
    d = d.faces(">Z").workplane().pushPoints(screws).hole(3.3)          # M4 tap drill
    frame = [(sx * (P["deck_l"] / 2 - 10.0), sy * (P["deck_w"] / 2 - 10.0)) for sx in (-1, 1) for sy in (-1, 0, 1)]
    d = d.faces(">Z").workplane().pushPoints(frame).hole(5.5)           # M5 clearance to frame
    return d, nests


# ------------------------------------------------------------------ checks
def frustum_ul(r1: float, r2: float, h: float) -> float:
    return math.pi * h / 3.0 * (r1 * r1 + r1 * r2 + r2 * r2)


def vol(w: cq.Workplane) -> float:
    return w.val().Volume()


def bbox(w: cq.Workplane) -> list[float]:
    b = w.val().BoundingBox()
    return [round(b.xlen, 2), round(b.ylen, 2), round(b.zlen, 2)]


def below_mesh_ul(bore: float, funnel_h: float) -> dict:
    funnel = frustum_ul(P["spigot_id"] / 2, bore / 2, funnel_h)
    drain = math.pi * (P["spigot_id"] / 2) ** 2 * (P["ret_t"] - P["mesh_seat_h"] - funnel_h + P["spigot_len"])
    tube = math.pi * (P["tube_id"] / 2) ** 2 * P["tube_to_pinch"]
    return {"funnel": round(funnel, 1), "drain_bore": round(drain, 1), "tube_to_pinch": round(tube, 1),
            "total": round(funnel + drain + tube, 1)}


def main() -> dict:
    OUT.mkdir(parents=True, exist_ok=True)
    for stale in ("WM-1_manifold.step", "WM-1_manifold.stl", "WM-1_gasket.step", "WM-1_gasket.dxf"):
        (OUT / stale).unlink(missing_ok=True)
    parts = {
        "RS8-A_body": rs8_body(), "RS8-A_retainer": rs8_retainer(),
        "RC4-A_body": rc4_body(), "RC4-A_retainer": rc4_retainer(),
        "WM-2_drain_dock": dock(), "WM-2_pinch_bar": pinch_bar(),
        "CF-A_carrier_frame": carrier_frame(), "TC24-A_tube_carrier": tube_carrier(),
    }
    deck, nests = deck_plate()
    parts["DK-A_deck_plate"] = deck
    rep: dict = {"status": "fabrication candidate (prototype); nominal interface dimensions unverified",
                 "revision": "B (after independent review, 12 Nov 2026): pinch-closed drain tubes, external waste, compressed mesh seat, locating pins",
                 "parameters_mm": {k: v for k, v in P.items()}, "parts": {}, "checks": {}}
    for name, w in parts.items():
        cq.exporters.export(w, str(OUT / f"{name}.step"))
        if name != "DK-A_deck_plate":
            cq.exporters.export(w, str(OUT / f"{name}.stl"), tolerance=0.02, angularTolerance=0.2)
        rep["parts"][name] = {"volume_cm3": round(vol(w) / 1000.0, 2), "envelope_mm": bbox(w),
                              "valid_solid": bool(w.val().isValid())}
    cq.exporters.export(parts["DK-A_deck_plate"].faces("<Z"), str(OUT / "DK-A_deck_plate.dxf"))

    for tag, body, ret in (("RS8-A", "RS8-A_body", "RS8-A_retainer"), ("RC4-A", "RC4-A_body", "RC4-A_retainer")):
        asm = cq.Assembly(name=tag)
        asm.add(parts[body], name="body", color=cq.Color(0.8, 0.85, 0.9, 0.6))
        asm.add(parts[ret], name="retainer", color=cq.Color(0.3, 0.3, 0.35))
        asm.save(str(OUT / f"{tag}_assembly.step"))
        inter = parts[body].intersect(parts[ret])
        rep["checks"][f"{tag}_body_retainer_interference_mm3"] = round(vol(inter), 4) if inter.vals() else 0.0
    z_strip = P["dock_h"] + P["ret_t"]
    for tag, body, ret in (("RS8-A", "RS8-A_body", "RS8-A_retainer"), ("RC4-A", "RC4-A_body", "RC4-A_retainer")):
        placed = parts[ret].translate((0, 0, z_strip))
        inter = parts["WM-2_drain_dock"].intersect(placed)
        rep["checks"][f"{tag}_retainer_dock_interference_mm3"] = round(vol(inter), 4) if inter.vals() else 0.0
    p1 = cq.Assembly(name="P1_wash_rig")
    p1.add(parts["WM-2_drain_dock"], name="dock", color=cq.Color(0.9, 0.9, 0.9, 0.5))
    p1.add(parts["WM-2_pinch_bar"], name="pinch_bar", loc=cq.Location((0, 8.0, P["pinch_slot_z"] + 0.3)), color=cq.Color(0.8, 0.2, 0.2))
    p1.add(parts["RS8-A_retainer"], name="retainer", loc=cq.Location((0, 0, z_strip)))
    p1.add(parts["RS8-A_body"], name="body", loc=cq.Location((0, 0, z_strip)), color=cq.Color(0.8, 0.85, 0.9, 0.6))
    p1.save(str(OUT / "P1_wash_rig_assembly.step"))

    c = rep["checks"]
    c["spigot_port_radial_clearance_mm"] = round((P["port_d"] - P["spigot_od"]) / 2, 3)
    c["strip_located_by_pins_not_spigots"] = (P["port_d"] - P["spigot_od"]) / 2 >= 0.25
    c["mesh_stack_compression_mm"] = round(P["mesh_t"] + P["ring_t"] - P["mesh_seat_h"], 3)
    c["mesh_ring_is_compressed"] = 0.03 <= P["mesh_t"] + P["ring_t"] - P["mesh_seat_h"] <= 0.12
    c["mesh_disc_overlap_on_seat_mm_rs8"] = round((P["mesh_seat_d"] - P["rs8_id_bot"]) / 2, 2)
    rs8_brim = frustum_ul(P["rs8_id_bot"] / 2, P["rs8_id_top"] / 2, P["rs8_depth"])
    rc4_brim = frustum_ul(P["rc4_id_bot"] / 2, P["rc4_id_top"] / 2, P["rc4_depth"])
    c["rs8_well_brim_ul_analytic"] = round(rs8_brim, 1)
    c["rs8_meets_catalogue_capacity_1100ul"] = rs8_brim >= 1100.0
    c["rs8_headspace_at_1000ul_mm"] = round(P["rs8_depth"] - 1000.0 / (math.pi * ((P["rs8_id_bot"] + P["rs8_id_top"]) / 4) ** 2), 1)
    c["rc4_column_brim_ul"] = round(rc4_brim, 0)
    c["rc4_holds_350ul_bed_plus_2x1ml"] = rc4_brim >= 350 + 2000
    c["rs8_min_wall_between_wells_mm"] = round(PITCH - P["rs8_id_top"], 2)
    c["rs8_mesh_seat_land_between_wells_mm"] = round(PITCH - P["mesh_seat_d"], 2)
    c["rc4_side_wall_mm"] = round((P["strip_w"] - P["rc4_id_top"]) / 2, 2)
    c["below_mesh_ul_rs8"] = below_mesh_ul(P["rs8_id_bot"], P["rs8_funnel_h"])
    c["below_mesh_ul_rc4"] = below_mesh_ul(P["rc4_id_bot"], P["rc4_funnel_h"])
    c["drain_bore_length_over_diameter"] = round((P["ret_t"] + P["spigot_len"]) / P["spigot_id"], 1)
    c["pinch_bar_fits_slot"] = (P["pinch_slot_h"] - 0.6) < P["pinch_slot_h"]
    c["carrier_fits_6_strips"] = 6 * 18.0 + 0.6 <= SBS_L - 2 * 6.0
    c["strip_fits_sbs_width"] = P["strip_len"] + 0.4 <= SBS_W - 2 * 2.5
    c["tube_carrier_wall_between_pockets_mm"] = round(18.0 - P["tube_pocket_d"], 2)
    c["known_first_print_risks"] = [
        "0.8 mm land between mesh seats and 1.2 mm wall between wells are at the limit for powder-bed printing",
        "flatness of the 79 mm clamping faces (bow 0.1-0.3 mm typical) may exceed the 0.06 mm ring compression: machined polypropylene is the intended liquid-path material",
        "1.2 mm x 9 mm drain bores must be cleared of powder or drilled",
        "RC4 side wall is 2.25 mm: slow heating (models/physics.py)",
    ]
    rep["deck_nests"] = nests
    c["all_solids_valid"] = all(p["valid_solid"] for p in rep["parts"].values())
    (OUT / "geometry_report.json").write_text(json.dumps(rep, indent=1))
    return rep


if __name__ == "__main__":
    r = main()
    print(json.dumps({"parts": r["parts"], "checks": r["checks"]}, indent=1))
