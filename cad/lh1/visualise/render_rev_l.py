"""Revision L pictures, step 2: Blender renders of the scenes written by export_rev_l.py (same look as render.py).

Run from the project root (headless):
  /Applications/Blender.app/Contents/MacOS/Blender -b -P cad/lh1/visualise/render_rev_l.py -- [--only 1_exchange_head] [--samples 64] [--dir out_family]
Out: cad/lh1/visualise/out_rev_l/<scene>.png (2400 x 1600), out_rev_l/captions.json
"""
import json
import math
import sys
import time
from pathlib import Path

import bpy
from mathutils import Vector, Matrix

HERE = Path(__file__).resolve().parent
argv = sys.argv[sys.argv.index("--") + 1:] if "--" in sys.argv else []


def arg(name, default=None):
    return argv[argv.index(name) + 1] if name in argv else default


OUT = HERE / (arg("--dir", "out_rev_l") or "out_rev_l")     # --dir out_family: the size-family scenes (export_family.py)


ONLY = set((arg("--only", "") or "").split(",")) - {""}
SAMPLES = int(arg("--samples", 64))
WIDTH, HEIGHT = 2400, 1600
T0 = time.time()


def log(*a):
    print(f"[render_rev_l {time.time() - T0:6.1f}s]", *a, flush=True)


DATA = json.loads((OUT / "scenes.json").read_text())
bpy.ops.wm.read_factory_settings(use_empty=True)
scene = bpy.context.scene
scene.render.engine = "CYCLES"
try:
    prefs = bpy.context.preferences.addons["cycles"].preferences
    prefs.compute_device_type = "METAL"
    prefs.get_devices()
    for d in prefs.devices:
        d.use = True
    scene.cycles.device = "GPU"
except Exception as e:  # noqa: BLE001
    log("GPU setup failed, CPU:", e)
scene.cycles.samples = SAMPLES
scene.cycles.use_denoising = True
scene.render.resolution_x, scene.render.resolution_y = WIDTH, HEIGHT
scene.render.film_transparent = False
scene.render.image_settings.file_format = "PNG"
scene.view_settings.view_transform = "Standard"
scene.unit_settings.scale_length = 0.001
world = bpy.data.worlds.new("w")
scene.world = world
world.use_nodes = True
bg = world.node_tree.nodes.get("Background")
bg.inputs["Color"].default_value = (0.90, 0.91, 0.93, 1)
bg.inputs["Strength"].default_value = 0.9
scene.render.use_freestyle = True
scene.render.line_thickness_mode = "ABSOLUTE"
scene.render.line_thickness = 1.2
vl = scene.view_layers[0]
vl.use_freestyle = True
fs = vl.freestyle_settings
fs.crease_angle = math.radians(140)
ls = fs.linesets[0] if len(fs.linesets) else fs.linesets.new("lines")
ls.select_by_visibility = True
ls.select_silhouette, ls.select_border, ls.select_crease = True, True, True
if ls.linestyle is None:
    ls.linestyle = bpy.data.linestyles.new("ls")
ls.linestyle.color = (0.10, 0.11, 0.13)
ls.linestyle.alpha = 0.7


def lin(h):
    h = h.lstrip("#")
    f = lambda c: c / 12.92 if c <= 0.04045 else ((c + 0.055) / 1.055) ** 2.4  # noqa: E731
    return tuple(f(int(h[i:i + 2], 16) / 255) for i in (0, 2, 4)) + (1.0,)


MATS = {}


def material(colour, cls):
    if (colour, cls) in MATS:
        return MATS[(colour, cls)]
    m = bpy.data.materials.new(f"{cls}_{colour}")
    m.use_nodes = True
    b = m.node_tree.nodes.get("Principled BSDF")
    b.inputs["Base Color"].default_value = lin(colour)
    metal, rough = {"accent": (0.15, 0.42), "bought": (0.45, 0.5), "labware": (0.0, 0.55)}.get(cls, (0.2, 0.5))
    b.inputs["Metallic"].default_value = metal
    b.inputs["Roughness"].default_value = rough
    MATS[(colour, cls)] = m
    return m


OBJ = {}
for name, r in DATA["objects"].items():
    bpy.ops.wm.stl_import(filepath=str(OUT / r["stl"]))
    o = bpy.context.selected_objects[0]
    o.name = name
    o.data.materials.clear()
    o.data.materials.append(material(r["colour"], r["cls"]))
    if r.get("translate"):
        o.location = Vector(r["translate"])
    o.hide_render = True
    OBJ[name] = o
log("imported", len(OBJ))
bpy.ops.mesh.primitive_plane_add(size=20000)
GROUND = bpy.context.active_object
GROUND.is_shadow_catcher = True


def sun(name, strength, angle_deg):
    d = bpy.data.lights.new(name, "SUN")
    d.energy = strength
    d.angle = math.radians(angle_deg)
    o = bpy.data.objects.new(name, d)
    scene.collection.objects.link(o)
    return o


KEY, FILL, RIM = sun("key", 3.0, 12), sun("fill", 1.2, 30), sun("rim", 1.5, 20)
CAM = bpy.data.objects.new("cam", bpy.data.cameras.new("cam"))
scene.collection.objects.link(CAM)
scene.camera = CAM


def aim(obj, direction):
    obj.rotation_euler = Vector(direction).to_track_quat("Z", "Y").to_euler()


def set_lights(cd):
    cd = Vector(cd).normalized()
    az = math.atan2(cd.y, cd.x)

    def at(daz, el):
        a = az + math.radians(daz)
        return Vector((math.cos(a) * math.cos(math.radians(el)), math.sin(a) * math.cos(math.radians(el)), math.sin(math.radians(el))))
    aim(KEY, at(-40, 55))
    aim(FILL, at(60, 25))
    aim(RIM, at(170, 40))


def frame(objs, cam_dir, lens=50.0, margin=1.08):
    pts = []
    for o in objs:
        pts += [o.matrix_world @ Vector(c) for c in o.bound_box]
    lo = Vector((min(p.x for p in pts), min(p.y for p in pts), min(p.z for p in pts)))
    hi = Vector((max(p.x for p in pts), max(p.y for p in pts), max(p.z for p in pts)))
    C = (lo + hi) / 2
    fwd = -Vector(cam_dir).normalized()
    upv = Vector((0, 0, 1))
    right = fwd.cross(upv).normalized()
    upc = right.cross(fwd).normalized()
    rel = [p - C for p in pts]
    xs, ys, zs = [q.dot(right) for q in rel], [q.dot(upc) for q in rel], [q.dot(fwd) for q in rel]
    CAM.rotation_euler = Matrix((right, upc, -fwd)).transposed().to_quaternion().to_euler()
    cd = CAM.data
    cd.lens = lens
    cd.sensor_fit = "HORIZONTAL"
    tx = (cd.sensor_width / 2) / lens
    ty = tx / (WIDTH / HEIGHT)
    ox = oy = D = 0.0
    for _ in range(4):
        D = max(max(abs(x - ox) / (tx / margin) - z for x, z in zip(xs, zs)), max(abs(y - oy) / (ty / margin) - z for y, z in zip(ys, zs)))
        us = [(x - ox) / (z + D) / tx for x, z in zip(xs, zs)]
        vs = [(y - oy) / (z + D) / ty for y, z in zip(ys, zs)]
        ox += (max(us) + min(us)) / 2 * tx * D
        oy += (max(vs) + min(vs)) / 2 * ty * D
    CAM.location = C + right * ox + upc * oy - fwd * D
    cd.clip_start, cd.clip_end = 5.0, D * 10 + 5000
    GROUND.location = (C.x, C.y, lo.z - 0.5)


caps = {}
for sname, sc in DATA["scenes"].items():
    if ONLY and sname not in ONLY:
        continue
    for o in OBJ.values():
        o.hide_render = True
    objs = [OBJ[n] for n in sc["objs"] if n in OBJ]
    for o in objs:
        o.hide_render = False
    GROUND.hide_render = not sc.get("ground", True)
    set_lights(sc["cam"])
    frame(objs, sc["cam"], lens=sc.get("lens", 50))
    texts = []
    for lb in sc.get("labels", []):                  # pass 2: place labels (flat text facing up, dark)
        bpy.ops.object.text_add(location=Vector(lb["pos"]))
        t = bpy.context.active_object
        t.data.body = lb["text"]
        t.data.size = lb.get("size", 20.0)
        t.data.align_x, t.data.align_y = "CENTER", "CENTER"
        t.data.extrude = 0.4
        t.data.materials.append(material("#111318", "bought"))
        texts.append(t)
    scene.render.filepath = str(OUT / f"{sname}.png")
    log("render", sname, len(objs))
    bpy.ops.render.render(write_still=True)
    for t in texts:
        bpy.data.objects.remove(t, do_unlink=True)
    caps[sname] = sc["caption"]
_old = json.loads((OUT / "captions.json").read_text()) if (OUT / "captions.json").exists() else {}
_old.update({k: sc["caption"] for k, sc in DATA["scenes"].items() if (OUT / f"{k}.png").exists()})
_old.update(caps)
(OUT / "captions.json").write_text(json.dumps(_old, indent=1))
log("done")
