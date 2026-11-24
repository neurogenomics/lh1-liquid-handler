"""Revision L pictures, step 1: STL meshes and scene lists for the exchange head (cad/lh1/head_l).

Run from the project root:   .venv/bin/python cad/lh1/visualise/export_rev_l.py
Out: cad/lh1/visualise/out_rev_l/stl/*.stl, cad/lh1/visualise/out_rev_l/scenes.json
Scenes 1-4: the exchange head (head_l).  Scenes 5-7 (pass 2): the whole K2 machine (cad/lh1/machine_l): front three-quarter, top view of
the deck with every place labelled, the dock with the pipettes parked.
Then: /Applications/Blender.app/Contents/MacOS/Blender -b -P cad/lh1/visualise/render_rev_l.py
"""
import importlib.util
import json
import sys
from pathlib import Path

import cadquery as cq

HERE = Path(__file__).resolve().parent
LH1 = HERE.parent
OUT = HERE / "out_rev_l"
STL = OUT / "stl"
sys.path.insert(0, str(LH1))
import params as P  # noqa: E402

spec = importlib.util.spec_from_file_location("head_l_build", str(LH1 / "head_l" / "build.py"))
HL = importlib.util.module_from_spec(spec)
spec.loader.exec_module(HL)
ZB = HL.ZB

COL = {"structure": ("#2F6DB5", "accent"), "tool": ("#E07A1F", "accent"), "pipette": ("#D9DCE1", "labware"),
       "gripper": ("#7B4FB0", "accent"), "coupling": ("#C0305A", "accent"), "bought": ("#555A62", "bought"),
       "dock": ("#1E9C8A", "accent"), "old_head": ("#9AA3AD", "accent"),
       "frame": ("#8C939C", "bought"), "gantry": ("#2F6DB5", "accent"), "deck": ("#C9CED6", "labware"), "station": ("#3E8E5A", "accent"),
       "labware": ("#EDEFF2", "labware"), "shelf": ("#B07D2B", "accent"), "enclosure": ("#AFC4D6", "labware"), "drawer": ("#6B5B95", "accent"),
       "parked": ("#E07A1F", "accent")}
COUPLING = ("pins", "latch", "spring", "roller", "balls", "stud", "magnets", "latch_switch", "hall_board", "pushrod_switch", "eject_pin",
            "push_rod")
BOUGHT = ("motor", "rail", "block0", "block1", "screw", "pusher", "camera", "ring_light", "nut")


def group(key):
    leaf = key.split(":")[-1]
    if key.startswith("gripper"):
        return "bought" if leaf in BOUGHT else "gripper"
    if leaf in COUPLING:
        return "coupling"
    if ":tool:" in key:
        return "pipette" if leaf in ("pipette", "nozzles", "body") else "tool"
    if leaf in BOUGHT or leaf.startswith("xblock"):
        return "bought"
    return "structure"


OBJ = {}


def put(name, shape, grp, dx=0.0, dy=0.0, dz=0.0):
    s = shape.translate(cq.Vector(dx, dy, dz))
    f = STL / f"{name}.stl"
    cq.exporters.export(cq.Workplane().add(s), str(f), tolerance=0.05, angularTolerance=0.15)
    c, cls = COL[grp]
    OBJ[name] = {"stl": f"stl/{name}.stl", "colour": c, "cls": cls, "group": grp}
    return name


def head_scene(prefix, dx=0.0, **kw):
    names = []
    for k, v in HL.head_parts(**kw).items():
        names.append(put(f"{prefix}__{k.replace(':', '__')}", v, group(k), dx=dx, dz=ZB))
    return names


def main():
    STL.mkdir(parents=True, exist_ok=True)
    scenes = {}
    # (1) exchange head, two tools fitted (8ch 30-300 on carriage A, single 0.5-10 on B), all axes up
    scenes["1_exchange_head"] = {"objs": head_scene("h1"), "cam": [-1.0, -1.6, 0.75], "lens": 60,
                                 "caption": "Rev L exchange head: two identical tool carriages (orange tools: 8-ch 30-300 uL, single "
                                            "0.5-10 uL), gripper axis (purple), coupling parts (crimson), structure (blue), bought parts "
                                            "(dark)"}
    # (2) coupling close-up: carriage A bracket + coupling, tool plate dropped 14 mm to show balls, pin pairs and latch
    xa = P.L_AXIS_X["carriage_A"]
    mv = HL.z_axis_moving(xa, 0.0)
    tm = HL.tool_module(xa, 0.0, "M300")
    objs = []
    for k in ("bracket", "pins", "latch", "spring", "roller", "latch_switch", "hall_board", "pushrod_switch", "pusher_bracket", "pusher"):
        objs.append(put(f"h2__{k}", mv[k], group(f"carriage_A:{k}"), dz=ZB))
    for k in ("tool_plate", "balls", "stud", "magnets", "eject_pin", "push_rod"):
        objs.append(put(f"h2__tool__{k}", tm[k], group(f"carriage_A:tool:{k}"), dz=ZB - 22.0))
    scenes["2_coupling_closeup"] = {"objs": objs, "cam": [-0.9, -1.3, -0.3], "lens": 70, "ground": False,
                                    "caption": "Coupling close-up (tool plate shown 22 mm below its seated position): three D10 balls "
                                               "under three pairs of D6 ground dowels, pull stud, spring-closed latch slide with release "
                                               "roller, Hall code magnets"}
    # (3) two-bay rig dock: one tool parked in bay 0, one on a carriage above bay 1
    dock = HL.bay_parts(0.0, 0.0, 0.0, 2)
    objs = [put(f"d3__{k}", v, "dock" if "switch" not in k else "bought") for k, v in dock.items()]
    top0 = HL.tpl_top(0.0)
    dy = 55.0 - (-104.5)                       # tool plate mid-depth onto the bay mid-depth
    for i, (key, lift) in enumerate((("S10", 0.0), ("M300", 120.0))):
        cx = HL.BAY["pitch"] * (i + 0.5)
        dz = -(top0 - HL.TPL["t"]) + lift
        for k, v in HL.tool_module(0.0, 0.0, key).items():
            objs.append(put(f"d3__tool{i}__{k}", v, group(f"c:tool:{k}"), dx=cx, dy=dy, dz=dz))
        if lift:
            for k, v in HL.z_axis_moving(0.0, 0.0).items():
                objs.append(put(f"d3__car__{k}", v, group(f"c:{k}"), dx=cx, dy=dy, dz=dz))
    scenes["3_rig_two_bay_dock"] = {"objs": objs, "cam": [-1.1, -1.5, 0.8], "lens": 55,
                                    "caption": "Bench-rig two-bay dock: single-channel tool parked in bay 1, eight-channel tool on a "
                                               "carriage above bay 2 (release posts, bay switches, drip trays)"}
    # (4) new head beside the present four-tool head (same scale): present head meshes from the rev K export
    meta = json.loads((HERE / "out" / "parts.json").read_text())
    old = []
    for r in meta["parts"]:
        v = r["variant"]
        if r["module"] != "head" or r["set"] != "main" or ("s" in v and v["s"] != 0) or "tc" in v or "door" in v:
            continue
        src = HERE / "out" / r["stl"]
        if src.exists():
            nm = f"old__{r['uid']}"
            (STL / f"{nm}.stl").write_bytes(src.read_bytes())
            c, cls = COL["old_head"] if r["kind"] == "custom" else COL["bought"]
            OBJ[nm] = {"stl": f"stl/{nm}.stl", "colour": c, "cls": cls, "group": "old_head", "translate": [-420.0, 0.0, 0.0]}
            old.append(nm)
    new = head_scene("h4", dx=0.0)
    for n in new:
        OBJ[n]["translate"] = [0.0, 59.0 + 25.0, 0.0]          # carriage plate back faces on the same line (old CP back at y 59..65)
    scenes["4_head_comparison"] = {"objs": old + new, "cam": [-0.35, -1.6, 0.45], "lens": 50,
                                   "caption": "Same scale: present four-tool head (left, grey; beam mid z 400) and the rev L exchange head "
                                              "(right; beam mid z 430)"}
    scenes.update(machine_scenes())
    (OUT / "scenes.json").write_text(json.dumps({"objects": OBJ, "scenes": scenes}, indent=1))
    print(f"wrote {len(OBJ)} meshes, {len(scenes)} scenes")


def machine_scenes():
    spec_m = importlib.util.spec_from_file_location("machine_l_build", str(LH1 / "machine_l" / "build.py"))
    ML = importlib.util.module_from_spec(spec_m)
    spec_m.loader.exec_module(ML)
    plan = ML.tip_supply_plan_l()
    static, zs = ML.scene_static(plan, tools=ML.DOCK_TOOLS)
    yf = 120.0
    gant = ML.gantry_items(yf)
    head = ML.Head()

    def grp(it):
        g = it["group"]
        if g == "parked_tool":
            return "parked" if it.get("leaf") not in ("pipette", "nozzles", "syringe") else "pipette"
        if g == "station" and it["cls"] == "labware":
            return "labware"
        if g in ("reserved",):
            return None
        return g if g in COL else "structure"
    names = {}
    for it in static + gant:
        g = grp(it)
        if g is None:
            continue
        names[it["name"]] = put("m__" + it["name"].replace(":", "_").replace("/", "_"), it["solid"], g)
    hp = head.parts(xc=40.0, yf=yf, sA=0.0, sB=150.0, sG=0.0, toolA="M300", toolB="S10")
    hn = [put("mh__" + k.replace(":", "__"), v.translate(cq.Vector(*off)), group(k)) for k, v, off in hp]
    hide5 = ("ENC-L-DR", "ENC-L-TOP", "ENC-L-RB", "ENC-L-SR", "ENC-L-HS")
    sc = {}
    sc["5_machine_front_three_quarter"] = {
        "objs": [n for k, n in names.items() if not k.startswith(hide5)] + hn, "cam": [0.9, -1.5, 0.85], "lens": 45,
        "caption": "Rev L machine (K2 frame, 24 places): exchange head on the raised gantry (beam mid z 545), deck stations, side shelf with "
                   "tip towers and the two edge hand-offs, eight-bay dock behind row G, drawer with the tip crate and waste canisters; door, "
                   "roof and right panel hidden"}
    deckset = [names[it["name"]] for it in static if it["name"] in names and it["group"] in ("deck", "station", "shelf", "dock", "parked_tool")]
    labels = []
    for n, pl in ML.PLACES.items():
        x, y = pl["c"]
        labels.append({"text": n, "pos": [x, y + 30.0, 200.0], "size": 22.0})
    for sl, (x, y) in ML.SLOTS.items():
        c = plan["slots"].get(sl)
        t = c["name"] if c and c["kind"] == "handoff" else (sl + " " + c["kind"].replace("tower_", "").replace("spent_wafers", "spent") if c else sl + " free")
        labels.append({"text": t, "pos": [x, y + 25.0, 200.0], "size": 14.0})
    for i, x in enumerate(ML.DOCK_X):
        labels.append({"text": f"bay {i + 1}" + (f" {ML.DOCK_TOOLS[i]}" if ML.DOCK_TOOLS[i] else ""), "pos": [x, ML.DOCK_Y + 75.0, 400.0], "size": 13.0})
    sc["6_deck_top_view_labelled"] = {"objs": deckset, "cam": [0.0, -0.001, 1.0], "lens": 60, "ground": False, "labels": labels,
                                      "caption": "Top view of the K2 deck: 24 standard places (row F front .. row G back, columns 1-4), side shelf "
                                                 "(I = inner, O = outer column; H1/H2 edge hand-offs), dock bays 1-8 behind row G"}
    hp7 = head.parts(xc=ML.DOCK_X[3] - P.L_AXIS_X["carriage_A"], yf=ML.DOCK_Y - ML.AY, sA=P.L_DOCK_S - 40.0, sB=0.0, sG=0.0, toolA=None, toolB="M50",
                     tipsA=False, tipsB=False)
    hn7 = [put("mh7__" + k.replace(":", "__"), v.translate(cq.Vector(*off)), group(k)) for k, v, off in hp7]
    dockset = [names[it["name"]] for it in static if it["name"] in names and (it["group"] in ("dock", "parked_tool") or it.get("place", "")[:1] == "G")]
    sc["7_dock_pipettes_parked"] = {"objs": dockset + hn7, "cam": [-0.45, -1.2, 1.0], "lens": 50,
                                    "caption": "Dock seen from the front over row G: eight bays (78 mm pitch), tools parked on their cradle lugs (bays 1-6: 8-ch 0.5-10, "
                                               "5-50, 30-300 uL, single 0.5-10 and 100-1,000 uL, syringe tool; 7-8 free); carriage A lowering onto "
                                               "bay 4 with carriage B holding the 5-50 uL tool clear above the neighbours"}
    return sc


if __name__ == "__main__":
    main()
