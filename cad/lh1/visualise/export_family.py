"""Size-family pictures, step 1: STL meshes and scene lists for the starter (S) and the large size (L) of cad/lh1/machine_family.

Run from the project root:   .venv/bin/python cad/lh1/visualise/export_family.py
Out: cad/lh1/visualise/out_family/stl/*.stl, cad/lh1/visualise/out_family/scenes.json
Then: /Applications/Blender.app/Contents/MacOS/Blender -b -P cad/lh1/visualise/render_rev_l.py -- --dir out_family
Scenes: 1 S front three-quarter (door, roof and right panel hidden); 2 S top view with every place, shelf slot and bay labelled;
3 S dock with the tools parked and carriage A lowering into bay 4; 4 S beside L at the same scale.
"""
import importlib.util
import json
import sys
from pathlib import Path

import cadquery as cq

HERE = Path(__file__).resolve().parent
LH1 = HERE.parent
OUT = HERE / "out_family"
STL = OUT / "stl"
sys.path.insert(0, str(LH1 / "machine_family"))
spec = importlib.util.spec_from_file_location("export_rev_l_for_family", str(HERE / "export_rev_l.py"))
ER = importlib.util.module_from_spec(spec)
spec.loader.exec_module(ER)                      # colours and head part grouping of the rev L pictures (same look)
import geometry as G  # noqa: E402
import scene as SC  # noqa: E402

COL = dict(ER.COL, services=("#9C4F2B", "accent"))
OBJ = {}


def put(name, shape, grp, tol=0.1, dx=0.0):
    f = STL / f"{name}.stl"
    cq.exporters.export(cq.Workplane().add(shape), str(f), tolerance=tol, angularTolerance=0.2)
    c, cls = COL[grp]
    OBJ[name] = {"stl": f"stl/{name}.stl", "colour": c, "cls": cls, "group": grp}
    if dx:
        OBJ[name]["translate"] = [dx, 0.0, 0.0]
    return name


def grp(it):
    g = it["group"]
    if g == "parked_tool":
        return "parked" if it.get("leaf") not in ("pipette", "nozzles", "syringe") else "pipette"
    if g == "station" and it["cls"] == "labware":
        return "labware"
    if g == "reserved" or "envelope" in it["name"] or "service_loops" in it["name"]:
        return None if g == "reserved" else "services"
    return g if g in COL else "structure"


def machine(name, prefix, dx=0.0, tools=None):
    S = G.Size(name)
    items, zs, pl, plan = SC.scene(S, tools=tools or SC.DOCK_TOOLS)
    yf = (S.YF0 + S.YF1) / 2 - 40.0
    gant = SC.gantry_items(S, yf)
    names = {}
    for it in items + gant:
        g = grp(it)
        if g is None:
            continue
        nm = prefix + "__" + "".join(ch if ch.isalnum() or ch in "-_" else "_" for ch in it["name"])[:90]
        names[it["name"]] = put(nm, it["solid"], g, dx=dx)
    return S, items, pl, plan, names, yf


def main():
    STL.mkdir(parents=True, exist_ok=True)
    head = G.FamilyHead()
    S, items, pl, plan, names, yf = machine("S", "s")
    hp = head.parts(xc=40.0, yf=yf, sA=0.0, sB=150.0, sG=0.0, toolA="M300", toolB="S10")
    hn = [put("sh__" + k.replace(":", "__"), v.translate(cq.Vector(*off)), ER.group(k), tol=0.05) for k, v, off in hp]
    hide = ("ENC-F-DR", "ENC-F-TOP", "ENC-F-RB", "ENC-F-SR", "ENC-F-HS", "ENC-F-HB", "ENC-F-BS_butt_strap_side_right", "ENC-F-BS_butt_strap_roof")
    scenes = {}
    nst = len([k for k in S.layout.values() if k != "EMPTY"])
    scenes["1_S_front_three_quarter"] = {
        "objs": [n for k, n in names.items() if not k.startswith(hide)] + hn, "cam": [0.9, -1.5, 0.85], "lens": 45,
        "caption": f"Starter S (ADR-011): 1,344 x 897 x 1,190 mm (1,456 with the extraction fan on the left side panel), 4 rows, {nst} stations "
                   "(tip parking TPK-S at B0); exchange head "
                   "on the gantry with the X cable chain on the beam, Y chain in its tray on the right, eight-bay dock behind row G, shelf with "
                   "towers and hand-off H1, control compartment under the base behind the drawer; door, roof and right panel hidden"}
    labels = []
    for n, p in pl.items():
        x, y = p["c"]
        labels.append({"text": f"{n} {p['key'] if p['key'] != 'EMPTY' else 'empty'}", "pos": [x, y + 30.0, 220.0], "size": 17.0})
    for sl, (x, y) in S.SLOTS.items():
        c = plan["slots"].get(sl)
        what = S.shelf_plan.get(sl, "free")
        t = c["name"] if c and c["kind"] == "handoff" else (sl + " " + (c["kind"].replace("tower_", "").replace("spent_wafers", "spent") if c else
                                                                         ("p1000 rack" if what.startswith("p1000") else "blocked" if what.startswith("blocked") else "free")))
        labels.append({"text": t, "pos": [x, y + 28.0, 220.0], "size": 13.0})
    for i, x in enumerate(S.DOCK_X):
        labels.append({"text": f"bay {i + 1}" + (f" {SC.DOCK_TOOLS[i]}" if SC.DOCK_TOOLS[i] else ""), "pos": [x, S.dock_y + 75.0, 400.0], "size": 12.0})
    deckset = [names[it["name"]] for it in items if it["name"] in names and it["group"] in ("deck", "station", "shelf", "dock", "parked_tool")]
    scenes["2_S_top_view_labelled"] = {"objs": deckset, "cam": [0.0, -0.001, 1.0], "lens": 60, "ground": False, "labels": labels,
                                       "caption": f"Starter S, top view: {nst} stations in rows F, A, B, G (column 0 pipette-only; TPK-S tip parking in the "
                                                  "left half of B0 beside the tip waste), shelf slots (I inner, O outer; "
                                                  "H1 hand-off; IG p1000 rack, carriage B only), dock bays 1-8 with camera markers behind row G"}
    hp7 = head.parts(xc=S.DOCK_X[3] - G.P.L_AXIS_X["carriage_A"], yf=S.dock_y - G.AY, sA=G.P.L_DOCK_S - 40.0, sB=0.0, sG=0.0, toolA=None, toolB="M50",
                     tipsA=False, tipsB=False)
    hn7 = [put("sh7__" + k.replace(":", "__"), v.translate(cq.Vector(*off)), ER.group(k), tol=0.05) for k, v, off in hp7]
    dockset = [names[it["name"]] for it in items if it["name"] in names and (it["group"] in ("dock", "parked_tool") or str(it.get("place", ""))[:1] == "G")]
    scenes["3_S_dock"] = {"objs": dockset + hn7, "cam": [-0.45, -1.2, 1.0], "lens": 50,
                          "caption": "Starter S dock: eight bays at 78 mm pitch behind row G, tools parked on their cradle lugs (bays 1-6), the dock lift bar "
                                     "with a pad under each bay's latch roller (actuator at the left end), the camera "
                                     "marker (tag + two holes) in front of each drip tray; carriage A lowering onto bay 4, carriage B holding the 5-50 uL "
                                     "tool clear above the neighbours; FFC loops beside the carriage plate"}
    # tip parking on the starter: carriage B's 30-300 eight-channel tool lowered at the deepest TPK-S spot, VC-2 and the funnel beside it
    sp = G.TPKS
    hp5 = head.parts(xc=sp["cx_abs"] + sp["spots_local"][-1] - G.P.L_AXIS_X["carriage_B"], yf=S.ry["B"] - G.AY, sA=0.0,
                     sB=G.M.s_for_tip_end("M300", sp["zl1"] - sp["pocket_depth"] - G.P.TIP_BELOW_NOZZLE["p300"] + 0.5), sG=0.0, toolA="M50", toolB="M300", tipsA=False, tipsB=True)
    hn5 = [put("sh5__" + k.replace(":", "__"), v.translate(cq.Vector(*off)), ER.group(k), tol=0.05) for k, v, off in hp5
           if k.startswith("carriage_B:tool") or k.startswith("carriage_B:tip")]
    tpset = [names[it["name"]] for it in items if it["name"] in names and (str(it.get("place", "")) in ("B0", "B1", "A1", "G0", "B2", "A0")
                                                                          or it["name"].startswith(("DK-F-B", "DK-F-A", "DK-F-G")))]
    scenes["5_S_tip_parking"] = {"objs": tpset + hn5, "cam": [-0.9, -1.3, 0.9], "lens": 60,
                                 "caption": "Starter S tip parking TPK-S, random access: three groups (spot, entry, spot) = 6 eight-channel p300 sets "
                                            "(48 tips, shown at full load) in 1 mm detent pockets over a walled drip tray; carriage B's 30-300 tool at "
                                            "the deepest spot, 5 mm from the VC-2 camera of the tip waste B1; designed, checked in the model, not built"}
    L, itemsL, plL, planL, namesL, yfL = machine("L", "l", dx=1700.0)
    hideL = hide
    s_objs = [n for k, n in names.items() if not k.startswith(hide)]
    l_objs = [n for k, n in namesL.items() if not k.startswith(hideL)]
    scenes["4_S_beside_L_same_scale"] = {"objs": s_objs + l_objs, "cam": [0.55, -1.5, 0.7], "lens": 40,
                                         "caption": "Same scale: starter S (left, 894 mm deep, 4 rows) and large L (right, 1,114 mm deep, 6 rows): same width, "
                                                    "height, gantry, head, dock, drawer and control compartment; L = S + rows C and D inserted in front of row G"}
    (OUT / "scenes.json").write_text(json.dumps({"objects": OBJ, "scenes": scenes}, indent=1))
    print(f"wrote {len(OBJ)} meshes, {len(scenes)} scenes")


if __name__ == "__main__":
    main()
