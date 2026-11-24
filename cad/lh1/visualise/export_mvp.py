"""MVP pictures, step 1: STL meshes and scene list for the MVP build variant (cad/lh1/machine_family/mvp.py) and the full starter beside it.

Run from the project root:   .venv/bin/python cad/lh1/visualise/export_mvp.py
Then:  /Applications/Blender.app/Contents/MacOS/Blender -b -P cad/lh1/visualise/render_rev_l.py -- --dir out_family/mvp
Out:   cad/lh1/visualise/out_family/mvp/{stl/*.stl, scenes.json, <scene>.png, captions.json}
Scenes: 1 MVP three-quarter (door, roof and right panel hidden); 2 MVP deck from above, every place labelled; 3 carriage A with the
eight-channel tool and the MCL-1 clamp; 4 MVP beside the full starter S at the same scale.  Designed and checked in the model only.
"""
import importlib.util
import json
import sys
from pathlib import Path

import cadquery as cq

HERE = Path(__file__).resolve().parent
LH1 = HERE.parent
OUT = HERE / "out_family" / "mvp"
STL = OUT / "stl"
sys.path.insert(0, str(LH1 / "machine_family"))
spec = importlib.util.spec_from_file_location("export_family_for_mvp", str(HERE / "export_family.py"))
EF = importlib.util.module_from_spec(spec)
spec.loader.exec_module(EF)
ER = EF.ER
import geometry as G  # noqa: E402
import scene as SC  # noqa: E402
import mvp as MV  # noqa: E402

COL = EF.COL
OBJ = {}
HIDE = ("ENC-F-DR", "ENC-F-TOP", "ENC-F-RB", "ENC-F-SR", "ENC-F-HS", "ENC-F-HB", "ENC-F-BS_butt_strap_side_right", "ENC-F-BS_butt_strap_roof")


def put(name, shape, grp, tol=0.1, dx=0.0):
    f = STL / f"{name}.stl"
    cq.exporters.export(cq.Workplane().add(shape), str(f), tolerance=tol, angularTolerance=0.2)
    c, cls = COL[grp]
    OBJ[name] = {"stl": f"stl/{name}.stl", "colour": c, "cls": cls, "group": grp}
    if dx:
        OBJ[name]["translate"] = [dx, 0.0, 0.0]
    return name


def clean(s):
    return "".join(ch if ch.isalnum() or ch in "-_" else "_" for ch in s)[:90]


def add_items(items, prefix, dx=0.0):
    names = {}
    for it in items:
        g = EF.grp(it)
        if g is None:
            continue
        names[it["name"]] = put(prefix + "__" + clean(it["name"]), it["solid"], g, dx=dx)
    return names


def main():
    STL.mkdir(parents=True, exist_ok=True)
    items, pl, pl_full, S, Sm = MV.scene_mvp()
    head = MV.MVPHead()
    ax = G.P.L_AXIS_X[MV.AXIS]
    xc, yf = pl["B2"]["c"][0] - ax, pl["B2"]["c"][1] - G.AY
    gant = SC.gantry_items(S, yf)
    names = add_items(items + gant, "m")
    hp = head.parts(xc, yf, sA=60.0)
    hn = [put("mh__" + clean(k), v.translate(cq.Vector(*off)), ER.group(k), tol=0.05) for k, v, off in hp]
    scenes = {}
    rep = json.loads((LH1 / "machine_family" / "out" / "MVP" / "report.json").read_text())
    W, D, H = rep["envelope_mm"]["W_D_H_mm"]
    scenes["1_MVP_three_quarter"] = {
        "objs": [n for k, n in names.items() if not k.startswith(HIDE)] + hn, "cam": [0.9, -1.5, 0.85], "lens": 45,
        "caption": f"LH-1 MVP (stage 0 of the starter): {W} x {D} x {H} mm, about {rep['mass_kg']['total_kg_ESTIMATE']} kg (estimate); the starter's own "
                   "frame, gantry, deck strips, drawer, enclosure and control compartment; ONE carriage with the eight-channel 30-300 uL tool held by "
                   "the manual clamp MCL-1 over plate 1 (B2); passive deck: p300 tips G3, reservoir F2, plates B2 and B4, tip waste B1, every "
                   "other place blanked; no dock, no shelf; door, roof and right panel hidden.  Designed and checked in the model, not built"}
    short = {"F2": "F2 reservoir", "B1": "B1 tip waste", "B2": "B2 plate 1", "B4": "B4 plate 2", "G3": "G3 p300 tips", "F4": "F4 blank (IN-B)"}
    labels = [{"text": short.get(n, f"{n} blank"), "pos": [p["c"][0], p["c"][1] + 30.0, 140.0], "size": 17.0 if n in short else 12.0}
              for n, p in pl.items()]
    deckset = [names[it["name"]] for it in items if it["name"] in names and it["group"] in ("deck", "station")]
    scenes["2_MVP_deck_top_labelled"] = {"objs": deckset, "cam": [0.0, -0.001, 1.0], "lens": 60, "ground": False, "labels": labels,
                                         "caption": "MVP deck from above: passive places only on the starter's grid (rows F, A, B, G); p300 tip frame G3 "
                                                    "(refill wafer loaded by hand), 12-column reservoir F2, two 96-well plates B2 and B4 on plain "
                                                    "inserts, tip waste B1 with its tip-check gate (VC-2 camera not fitted); every other window closed "
                                                    "by a plain insert, the shaker window F4 by the MVP-only blank IN-B"}
    hp3 = head.parts(0.0, 0.0, sA=0.0)
    hn3 = [put("mh3__" + clean(k), v.translate(cq.Vector(*off)), ER.group(k), tol=0.03) for k, v, off in hp3]
    scenes["3_MVP_carriage_and_tool"] = {"objs": hn3, "cam": [-0.8, -1.4, 0.55], "lens": 60,
                                         "caption": "MVP carriage A with the eight-channel 30-300 uL tool (tips on): Z axis, LGA35 plunger pusher with "
                                                    "its RETRACTED switch, cradle force switch, the full design's tool plate on three vee seats, held by "
                                                    "the manual fork clamp MCL-1 in the latch channel (finger tab in front of the bracket, one M5 set "
                                                    "screw at 0.30 N m: preload 550-916 N, CALCULATED); changing the tool is a manual job"}
    full_items, _, _, _ = SC.scene(S, tools=SC.DOCK_TOOLS)
    fy = (S.YF0 + S.YF1) / 2 - 40.0
    fnames = add_items(full_items + SC.gantry_items(S, fy), "s", dx=1650.0)
    fh = G.FamilyHead()
    fhp = fh.parts(xc=40.0, yf=fy, sA=0.0, sB=150.0, sG=0.0, toolA="M300", toolB="S10")
    fhn = [put("sh__" + clean(k), v.translate(cq.Vector(*off)), ER.group(k), tol=0.05, dx=1650.0) for k, v, off in fhp]
    scenes["4_MVP_beside_full_starter_same_scale"] = {
        "objs": [n for k, n in names.items() if not k.startswith(HIDE)] + hn + [n for k, n in fnames.items() if not k.startswith(HIDE)] + fhn,
        "cam": [0.55, -1.5, 0.7], "lens": 40,
        "caption": "Same scale: the MVP (left) and the full starter S (right).  Same frame, gantry, deck strips, drawer, enclosure and control "
                   "compartment; the starter adds carriage B, the gripper, the latch, the eight-bay dock with tools, the shelf with towers, the "
                   "deck modules, TC-1, tip parking, cameras and the extraction fan"}
    (OUT / "scenes.json").write_text(json.dumps({"objects": OBJ, "scenes": scenes}, indent=1))
    print(f"wrote {len(OBJ)} meshes, {len(scenes)} scenes")


if __name__ == "__main__":
    main()
