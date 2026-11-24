"""LH-1 visualisation, step 2: Blender renders of the meshes written by export_meshes.py.

Run from the project root (headless):
  /Applications/Blender.app/Contents/MacOS/Blender -b -P cad/lh1/visualise/render.py -- [--only a,b] [--samples 64] [--width 2400] [--no-lines]

In:   cad/lh1/visualise/out/parts.json + out/stl/*.stl
Out:  docs/lh1/renders/<view>.png  (2400 px wide, light neutral background), docs/lh1/renders/build_sequence_strip.png,
      cad/lh1/visualise/out/render_views.json (view list with captions, merged into docs/lh1/renders/index.json by make_index.py)

Look: Cycles, materials by kind (anodised extrusions, module accent on custom parts, dark bought parts, off-white labware),
three sun lights (key/fill/rim, soft shadows) + uniform light-grey world, shadow-catcher ground plane, Freestyle outlines.
Cameras are framed automatically from the bounding boxes of the visible (and exploded) objects.
"""
import json
import math
import sys
import time
from pathlib import Path

import bpy
import numpy as np
from mathutils import Vector, Matrix

HERE = Path(__file__).resolve().parent
ROOT = HERE.parent.parent.parent
OUT = HERE / "out"
RENDERS = ROOT / "docs" / "lh1" / "renders"

argv = sys.argv[sys.argv.index("--") + 1:] if "--" in sys.argv else []


def arg(name, default=None, flag=False):
    if name in argv:
        return True if flag else argv[argv.index(name) + 1]
    return default


ONLY = set(arg("--only", "").split(",")) - {""}
SAMPLES = int(arg("--samples", 64))
WIDTH = int(arg("--width", 2400))
HEIGHT = int(round(WIDTH * 2 / 3))
LINES = not arg("--no-lines", flag=True)
BG = (0.953, 0.957, 0.965)          # #F3F4F6 (display values)
T0 = time.time()


def log(*a):
    print(f"[render {time.time() - T0:6.1f}s]", *a, flush=True)


META = json.loads((OUT / "parts.json").read_text())
PARTS = {r["uid"]: r for r in META["parts"]}

# ============================================================================================ scene setup
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
scene.cycles.max_bounces = 6
scene.render.resolution_x, scene.render.resolution_y = WIDTH, HEIGHT
scene.render.resolution_percentage = 100
scene.render.film_transparent = True
scene.render.image_settings.file_format = "PNG"
scene.render.image_settings.color_mode = "RGBA"
scene.view_settings.view_transform = "Standard"
scene.view_settings.look = "None"
scene.view_settings.exposure = 0.0
scene.unit_settings.scale_length = 0.001

world = bpy.data.worlds.new("w")
scene.world = world
world.use_nodes = True
bg = world.node_tree.nodes.get("Background")
bg.inputs["Color"].default_value = (0.8, 0.82, 0.85, 1)
bg.inputs["Strength"].default_value = 0.45

if LINES:
    scene.render.use_freestyle = True
    scene.render.line_thickness_mode = "ABSOLUTE"
    scene.render.line_thickness = max(1.0, WIDTH / 2000)
    vl = scene.view_layers[0]
    vl.use_freestyle = True
    fs = vl.freestyle_settings
    fs.crease_angle = math.radians(140)
    ls = fs.linesets[0] if len(fs.linesets) else fs.linesets.new("lines")
    ls.select_by_visibility = True
    ls.select_silhouette, ls.select_border, ls.select_crease = True, True, True
    ls.select_by_collection = True
    if ls.linestyle is None:
        ls.linestyle = bpy.data.linestyles.new("ls")
    ls.linestyle.color = (0.10, 0.11, 0.13)
    ls.linestyle.alpha = 0.75
    ls.linestyle.thickness = max(1.0, WIDTH / 2000)


def srgb_to_lin(c):
    return c / 12.92 if c <= 0.04045 else ((c + 0.055) / 1.055) ** 2.4


def hex_lin(h):
    h = h.lstrip("#")
    return tuple(srgb_to_lin(int(h[i:i + 2], 16) / 255) for i in (0, 2, 4)) + (1.0,)


MATS = {}


def material(colour, cls):
    key = (colour, cls)
    if key in MATS:
        return MATS[key]
    m = bpy.data.materials.new(f"{cls}_{colour}")
    m.use_nodes = True
    b = m.node_tree.nodes.get("Principled BSDF")
    b.inputs["Base Color"].default_value = hex_lin(colour)
    metal, rough = {"extrusion": (0.35, 0.4), "accent": (0.15, 0.42), "deck_plate": (0.35, 0.45), "bought": (0.45, 0.5),
                    "bought_dark": (0.2, 0.55), "belt": (0.0, 0.7), "labware": (0.0, 0.55), "tip": (0.0, 0.5),
                    "panel_acm": (0.1, 0.6)}.get(cls, (0.2, 0.5))
    b.inputs["Metallic"].default_value = metal
    b.inputs["Roughness"].default_value = rough
    if cls == "panel_pc":                       # clear polycarbonate: transmissive, nearly colourless
        b.inputs["Metallic"].default_value = 0.0
        b.inputs["Roughness"].default_value = 0.03
        b.inputs["Transmission Weight"].default_value = 1.0
        b.inputs["IOR"].default_value = 1.58
    MATS[key] = m
    return m


# ============================================================================================ import meshes
lines_col = bpy.data.collections.new("lines")
scene.collection.children.link(lines_col)
OBJ = {}
log(f"importing {len(PARTS)} meshes")
for uid, r in PARTS.items():
    if r["kind"] == "reserved":
        continue
    p = OUT / r["stl"]
    if not p.exists():
        continue
    bpy.ops.wm.stl_import(filepath=str(p))
    o = bpy.context.selected_objects[0]
    o.name = uid
    for c in list(o.users_collection):
        c.objects.unlink(o)
    lines_col.objects.link(o)
    o.data.materials.clear()
    o.data.materials.append(material(r["colour"], r["cls"]))
    # smooth curved surfaces, keep flats flat
    for poly in o.data.polygons:
        poly.use_smooth = False
    try:
        o.data.set_sharp_from_angle(angle=math.radians(35))
        for poly in o.data.polygons:
            poly.use_smooth = True
    except Exception:  # noqa: BLE001
        pass
    if r["cls"] in ("panel_pc", "panel_acm"):
        o.visible_shadow = False                # panels do not shade the interior (keeps the machine readable through them)
    o.hide_render = True
    OBJ[uid] = o
if LINES:
    vl.freestyle_settings.linesets[0].collection = lines_col
log("imported", len(OBJ))

# ground (shadow catcher) and lights
bpy.ops.mesh.primitive_plane_add(size=20000)
GROUND = bpy.context.active_object
GROUND.name = "ground"
GROUND.is_shadow_catcher = True
for c in list(GROUND.users_collection):
    c.objects.unlink(GROUND)
scene.collection.objects.link(GROUND)


def sun(name, strength, angle_deg, shadow=True):
    d = bpy.data.lights.new(name, "SUN")
    d.energy = strength
    d.angle = math.radians(angle_deg)
    try:
        d.cycles.cast_shadow = shadow
    except Exception:  # noqa: BLE001
        d.use_shadow = shadow
    o = bpy.data.objects.new(name, d)
    scene.collection.objects.link(o)
    return o


KEY, FILL, RIM = sun("key", 3.2, 12), sun("fill", 1.1, 30, shadow=False), sun("rim", 1.6, 20, shadow=False)
cam_data = bpy.data.cameras.new("cam")
CAM = bpy.data.objects.new("cam", cam_data)
scene.collection.objects.link(CAM)
scene.camera = CAM


def aim(obj, direction):
    """point obj's -Z along -direction (direction = from target toward obj)"""
    obj.rotation_euler = Vector(direction).to_track_quat("Z", "Y").to_euler()


def set_lights(cam_dir):
    cd = Vector(cam_dir).normalized()
    az = math.atan2(cd.y, cd.x)
    if abs(cd.z) > 0.95:          # top view: light from front-left
        az = math.atan2(-1, -0.6)

    def dir_at(daz, el):
        a = az + math.radians(daz)
        return Vector((math.cos(a) * math.cos(math.radians(el)), math.sin(a) * math.cos(math.radians(el)), math.sin(math.radians(el))))

    aim(KEY, dir_at(-40, 55))
    aim(FILL, dir_at(60, 25))
    aim(RIM, dir_at(170, 40))


# ============================================================================================ framing
def world_corners(objs):
    pts = []
    for o in objs:
        mw = o.matrix_world
        pts += [mw @ Vector(c) for c in o.bound_box]
    return pts


def frame_camera(objs, cam_dir, ortho=False, lens=50.0, margin=1.08, up=None):
    pts = world_corners(objs)
    lo = Vector((min(p.x for p in pts), min(p.y for p in pts), min(p.z for p in pts)))
    hi = Vector((max(p.x for p in pts), max(p.y for p in pts), max(p.z for p in pts)))
    C = (lo + hi) / 2
    fwd = -Vector(cam_dir).normalized()                 # camera looks along fwd
    upv = Vector(up) if up else Vector((0, 0, 1))
    if abs(fwd.dot(upv)) > 0.99:
        upv = Vector((0, 1, 0))
    right = fwd.cross(upv).normalized()
    upc = right.cross(fwd).normalized()
    aspect = WIDTH / HEIGHT
    rel = [p - C for p in pts]
    xs = [r.dot(right) for r in rel]
    ys = [r.dot(upc) for r in rel]
    zs = [r.dot(fwd) for r in rel]
    rot = Matrix((right, upc, -fwd)).transposed().to_quaternion().to_euler()
    CAM.rotation_euler = rot
    cd = CAM.data
    cd.shift_x = cd.shift_y = 0.0
    if ortho:
        cd.type = "ORTHO"
        w = (max(xs) - min(xs)) * margin
        h = (max(ys) - min(ys)) * margin
        cd.ortho_scale = max(w, h * aspect)
        cx, cy = (max(xs) + min(xs)) / 2, (max(ys) + min(ys)) / 2
        depth = max(abs(z) for z in zs) + 2000
        CAM.location = C + right * cx + upc * cy - fwd * depth
        cd.clip_start, cd.clip_end = 1.0, depth * 3
        return
    cd.type = "PERSP"
    cd.lens = lens
    cd.sensor_fit = "HORIZONTAL"
    tx = (cd.sensor_width / 2) / lens
    ty = tx / aspect
    # distance so every corner fits, then centre with lens shift, iterate
    ox = oy = 0.0
    D = 0.0
    for _ in range(4):
        D = max(max((abs(x - ox) / (tx / margin)) - z for x, z in zip(xs, zs)),
                max((abs(y - oy) / (ty / margin)) - z for y, z in zip(ys, zs)))
        us = [(x - ox) / (z + D) / tx for x, z in zip(xs, zs)]
        vs = [(y - oy) / (z + D) / ty for y, z in zip(ys, zs)]
        ox += (max(us) + min(us)) / 2 * tx * D
        oy += (max(vs) + min(vs)) / 2 * ty * D
    CAM.location = C + right * ox + upc * oy - fwd * D
    cd.clip_start, cd.clip_end = 5.0, D * 10 + 5000


# ============================================================================================ selections
def sel(pred):
    return [u for u, r in PARTS.items() if u in OBJ and pred(r)]


def main_set(r, tc="open", head_axis_lowered=None, door="closed"):
    if r["set"] != "main":
        return False
    v = r["variant"]
    if "door" in v and v["door"] != door:
        return False
    if "tc" in v and v["tc"] != tc:
        return False
    if "s" in v:
        lowered = head_axis_lowered is not None and v["axis"] == head_axis_lowered
        return v["s"] != 0 if lowered else v["s"] == 0
    return True


MACHINE = lambda r: main_set(r) and not r.get("enclosure")  # noqa: E731     machine without the enclosure
MACHINE_ENC = lambda r: main_set(r)  # noqa: E731                               with the enclosure, door closed
MACHINE_DOOROPEN = lambda r: main_set(r, door="open")  # noqa: E731            with the enclosure, door open 100 deg


def mod(*mods, tc="open", lowered=None):
    return lambda r: main_set(r, tc, lowered) and r["module"] in mods and not r.get("enclosure")


def deck_sub(sub):
    return lambda r: main_set(r) and r["module"] == "deck_modules" and r["deck_sub"] == sub


def centre(uids):
    pts = []
    for u in uids:
        b = PARTS[u]["bbox"]
        pts.append(((b[0] + b[1]) / 2, (b[2] + b[3]) / 2, (b[4] + b[5]) / 2))
    a = np.array(pts)
    return (a.min(0) + a.max(0)) / 2


def explode_scale(uids, k, ref=None):
    """offset each part's centre away from the group centre by factor k=(kx,ky,kz)"""
    C = centre(uids) if ref is None else np.array(ref)
    out = {}
    for u in uids:
        b = PARTS[u]["bbox"]
        c = np.array(((b[0] + b[1]) / 2, (b[2] + b[3]) / 2, (b[4] + b[5]) / 2))
        out[u] = tuple((c - C) * np.array(k))
    return out


def explode_rank_z(uids, gap, base=0.0, key=None):
    """stack parts vertically in order of their centre z: the n-th distinct level is lifted by n*gap"""
    zc = {u: round((PARTS[u]["bbox"][4] + PARTS[u]["bbox"][5]) / 2 / 2.0) for u in uids} if key is None else {u: key(u) for u in uids}
    levels = sorted(set(zc.values()))
    mid = len(levels) / 2
    return {u: (0.0, 0.0, base + (levels.index(zc[u]) - mid) * gap) for u in uids}


# ============================================================================================ view list
FRONT_LEFT, FRONT_RIGHT = (-1.0, -1.25, 0.85), (1.0, -1.25, 0.85)
REAR_LEFT, REAR_RIGHT = (-1.0, 1.2, 0.8), (1.0, 1.2, 0.8)
FRONT, TOP, RIGHT = (0, -1, 0), (0, 0, 1), (1, 0, 0)
SIMPL = "Bought parts shown simplified."
VIEWS = []


def view(name, pred, cam, caption, ortho=False, lens=50, explode=None, margin=1.06, up=None, ground=True, frame_pred=None, lines=True,
         role_colours=False):
    VIEWS.append(dict(name=name, pred=pred, cam=cam, caption=caption, ortho=ortho, lens=lens, explode=explode, margin=margin, up=up,
                      ground=ground, frame_pred=frame_pred, lines=lines, role_colours=role_colours))


NOPANEL = (" Enclosure hidden (see the machine_enclosed views).")
ENCL = ("2020 cage with grey composite (ACM) top, sides and back, clear polycarbonate front panels and hinged door")
view("machine_enclosed_34_front_left", MACHINE_ENC, FRONT_LEFT,
     f"LH-1 in its enclosure, door closed, from the front left: {ENCL}; the polycarbonate is rendered clear so the deck and head show "
     "through it. " + SIMPL, lines=False)
view("machine_enclosed_34_front_right", MACHINE_ENC, FRONT_RIGHT,
     "LH-1 in its enclosure, door closed, from the front right: note the fixed clear panel on the right of the mullion (e-stop and reset) "
     "and the removable service panel low on the right side. " + SIMPL, lines=False)
view("machine_enclosed_door_open_34_front_left", MACHINE_DOOROPEN, FRONT_LEFT,
     "LH-1 in its enclosure with the door swung open 100 degrees on its left hinges (the open position the assembly script checks), "
     "from the front left: the whole deck is open to the operator. " + SIMPL, lines=False)
view("machine_enclosed_door_open_34_front_right", MACHINE_DOOROPEN, FRONT_RIGHT,
     "LH-1 with the door open, from the front right: the door opening runs from the left post to the mullion; the fixed clear panel "
     "with the e-stop and reset stays on the right. " + SIMPL, lines=False)
view("machine_34_front_left", MACHINE, FRONT_LEFT,
     "LH-1 complete, three-quarter view from the front left: the 2040 frame, deck with ten nests, four-tool head on the X beam, and the "
     "TC-1 thermocycler at the back left with its lid parked." + NOPANEL + " " + SIMPL)
view("machine_34_front_right", MACHINE, FRONT_RIGHT,
     "LH-1 complete from the front right: note the two power supplies under the deck at the right and the Y motors, belts and cable-chain "
     "tray outboard of the side beams." + NOPANEL + " " + SIMPL)
view("machine_front", MACHINE, FRONT,
     "LH-1 front elevation (orthographic): the head hangs from the X beam with its four tool plates side by side, all Z axes up; the deck "
     "sits 136 mm above the base frame." + NOPANEL + " " + SIMPL, ortho=True)
view("machine_top", MACHINE, TOP,
     "LH-1 plan view (orthographic, front of machine at the bottom): the 3 x 4 nest grid under the gantry, the head at mid travel and the "
     "TC-1 lid parked over D1 at the back left." + NOPANEL + " " + SIMPL, ortho=True, up=(0, 1, 0), ground=False)
view("machine_right", MACHINE, RIGHT,
     "LH-1 right side elevation (orthographic, front of machine at the left): the X beam and head sit high above the deck; note the "
     "deck-to-head clearance used for labware and tips." + NOPANEL + " " + SIMPL, ortho=True)
view("module_frame_gantry", mod("frame_gantry"), FRONT_LEFT,
     "Frame and gantry module alone: 2040 base, uprights, side beams carrying the Y rails, deck supports, the 80 x 40 X beam with its two "
     "rails, Y carriages, motors and belts (blue = custom brackets). " + SIMPL)
view("module_deck", mod("deck_modules"), FRONT_LEFT,
     "Deck with all deck modules in the default layout: tip waste at A1, tip racks, magnet module at B2, thermal block at C3, reservoir, "
     "and the merged C1+D1 thermocycler bay left open. " + SIMPL)
view("module_head_front", mod("head"), FRONT_LEFT,
     "Head from the front left: carriage plate with four Z axes; tools left to right are the 0.5-10 uL single pipette, 8-channel 50 uL, "
     "gripper, and 8-channel 300 uL (orange = custom parts). " + SIMPL, lens=60)
view("module_head_rear", mod("head"), REAR_RIGHT,
     "Head from the rear right: the X-rail blocks, belt clamp and 2040 stiffeners on the back of the carriage plate, and the Z motors on "
     "top of each axis. " + SIMPL, lens=60)
view("module_head_z_lowered", mod("head", lowered=META["lower_axis"]), FRONT_LEFT,
     f"Head with the 8-channel 50 uL pipette's Z axis lowered {META['lower_s']:.0f} mm while the other three stay up: each tool moves "
     "on its own rail and lead screw. " + SIMPL, lens=60)
view("module_thermocycler_closed", mod("thermocycler", tc="closed"), FRONT_LEFT,
     "Thermocycler TC-1 with the lid closed over the plate in C1: the heated lid in its stainless hood presses the sealing lid onto the "
     "plate; carriage on the MGN15 rail, heat sink and fan duct below the deck. " + SIMPL, lens=60)
view("module_thermocycler_open", mod("thermocycler", tc="open"), FRONT_LEFT,
     "Thermocycler TC-1 with the lid lifted and parked over D1, leaving the PCR plate in C1 open to the pipettes and gripper. " + SIMPL,
     lens=60)
view("module_magnet", deck_sub("magnet_block"), FRONT_LEFT,
     "Passive magnet block MB-1 in B2 (the default since revision F): insert plate with machined corner locators, the printed block "
     "carrying its magnets, and a PCR plate seated on it; no motor or lift. " + SIMPL, lens=70)
view("module_thermal_block", deck_sub("thermal"), FRONT_LEFT,
     "Thermal block (C3): 24-tube cold block in an insulating frame, Peltiers and heat sink with fan shroud hanging below the deck. "
     + SIMPL, lens=70)
view("module_tip_waste", deck_sub("tip_waste"), FRONT_LEFT,
     "Tip waste (A1): insert plate with the machined TW-4D strip comb, the TW-5 eject hopper beside it, the two tip-check posts with "
     "their through-beam sensors, and the funnel that drops tips into a bin under the deck. " + SIMPL, lens=70)
TOOLTXT = {"pipette_small": "0.5-10 uL single-channel pipette", "pipette_8ch_50": "8-channel 50 uL pipette",
           "pipette_8ch_300": "8-channel 300 uL pipette"}
for t, m_ in META.get("strip", {}).items():
    view(f"strip_comb_{t}", (lambda st: (lambda r: r["set"] == st))(m_["set"]), (-1.0, -0.6, 0.35),
         f"Tip strip-off at A1, section on the nozzle line (x = {m_['nozzle_x']:g}): the {TOOLTXT[t]} lowered to nozzle z {m_['nozzle_z']:g} "
         f"and slid into the TW-4D comb, so the {m_['tip']} tip collar sits under the comb (underside z {m_['comb_underside_z']:g}); lifting "
         "then strips the tips into the funnel. " + SIMPL, lens=70, margin=1.35,
         frame_pred=(lambda st: (lambda r: r["set"] == st and (r["name"].endswith("strip_comb") or r["name"].endswith("_funnel")
                                                                or (r["name"].startswith("tip_") and r["bbox"][4] < 60))))(m_["set"]))


def lift_tc():
    off = {}
    for u in sel(lambda r: main_set(r, "closed")):
        r = PARTS[u]
        if r["module"] == "thermocycler" or r["part"] == "DK-2_tc_bay_subplate":
            off[u] = (0, 0, 170)
    return off


view("tc_bay_lift_out", lambda r: main_set(r, "closed") and not r.get("enclosure") and r["nest"] not in ("C2", "D2") and r["module"] != "head"
     and r["build_step"] not in ("02_uprights_y_rails", "06_gantry_beam"),
     FRONT_LEFT,
     "Removing the thermocycler: with the C2 and D2 inserts taken out, TC-1 lifts straight up 170 mm together with the DK-2 sub-plate it "
     "is mounted on, leaving the cut-out in the deck plate (head, gantry and uprights hidden for clarity). " + SIMPL, explode=lift_tc, lens=45, margin=1.3,
     frame_pred=lambda r: main_set(r, "closed") and r["module"] in ("deck_modules", "thermocycler") and r["nest"] not in ("C2", "D2"))
view("head_rear_controller_boards", mod("head"), (-0.6, 1.0, 1.1),
     "Head from the rear left, close up: the MBB-1 printed bracket on the back of the carriage plate carrying the two stacked MMB CAN "
     "controller boards, next to the board tray and cable-chain anchor. " + SIMPL, lens=60, margin=1.1,
     frame_pred=lambda r: main_set(r) and r["module"] == "head" and r["name"] in ("MBB-1", "mmb_a", "mmb_b", "EB-1", "board", "CCA-1", "CP-1"))
BAY = ("CBP_", "VBP_", "CBC_", "PSUP_", "PSU_")


def bay_lift():
    return {u: (0, 0, 140) for u in sel(lambda r: main_set(r) and r["name"].startswith("CBC_"))}


view("control_bay", lambda r: main_set(r) and not r.get("enclosure") and r["module"] == "frame_gantry"
     and (r["name"].startswith(BAY) or r["build_step"] == "01_base_frame"), (1.0, -1.1, 0.75),
     "Control bay under the right of the deck: the CBP plate on the base members, the vertical board panel VBP at the right, the two "
     "24 V supplies on their plates with their PSD-1 exhaust ducts, and the removable CBC cover (carried on VBP and two CBC-L angles) "
     "lifted 140 mm off to show the inside (deck hidden). " + SIMPL,
     explode=bay_lift, lens=55, margin=1.15, frame_pred=lambda r: main_set(r) and r["name"].startswith(BAY))


# ---- exploded
def ex_machine():
    us = sel(MACHINE)
    off = {}  # enclosure not shown
    for u in us:
        m = PARTS[u]["module"]
        off[u] = {"frame_gantry": (0, 0, 0), "deck_modules": (0, -650, 0), "head": (0, -320, 480), "thermocycler": (0, 0, 300)}[m]
        if m == "frame_gantry" and PARTS[u]["build_step"] == "06_gantry_beam":
            off[u] = (0, 0, 220)
    return off


view("exploded_machine_by_module", MACHINE, FRONT_LEFT,
     "Exploded by module: the deck with its modules slides out to the front, the thermocycler lifts out of its bay, the gantry beam rises "
     "off the Y rails and the head comes off the beam. " + SIMPL, explode=ex_machine, lens=45)


def ex_frame():
    us = sel(mod("frame_gantry"))
    lift = {"01_base_frame": 0, "09_electronics_bay": 120, "03_deck_supports": 260, "02_uprights_y_rails": 420, "06_gantry_beam": 700}
    off = {}
    for u in us:
        r = PARTS[u]
        z = lift[r["build_step"]]
        if r["build_step"] == "02_uprights_y_rails" and r["name"].startswith("EXT_upright"):
            z = 180
        off[u] = (0, 0, z)
    return off


view("exploded_frame_gantry", mod("frame_gantry"), FRONT_LEFT,
     "Frame and gantry exploded in build order from the bottom: base frame, power-supply plates, deck supports, uprights, side beams "
     "with Y rails and motors, and the X beam with its carriages on top. " + SIMPL, explode=ex_frame, lens=45)


def ex_deck():
    us = sel(mod("deck_modules"))
    off = {}
    for u in us:
        r = PARTS[u]
        n = r["name"]
        if r["part"] in ("DK-1_deck_plate", "DK-2_tc_bay_subplate"):
            z = 0
        elif "insert" in n:
            z = 70
        elif "_loc" in n:
            z = 100
        elif r["kind"] == "labware":
            z = 170
        else:
            zc = (r["bbox"][4] + r["bbox"][5]) / 2
            z = -60 if zc < 0 else 120
            if r["deck_sub"] == "magnet" and zc < 0:
                z = -60 - 40 * ["housing", "carrier", "magnets"].index(n.split("_", 1)[1]) if n.split("_", 1)[1] in ("housing", "carrier", "magnets") else -60
        off[u] = (0, 0, z)
    return off


view("exploded_deck", mod("deck_modules"), FRONT_LEFT,
     "Deck exploded: the 6 mm deck plate with its windows, the insert plate for each nest lifted above it, printed corner locators above "
     "those, labware on top, and the module bodies dropped below the deck. " + SIMPL, explode=ex_deck, lens=45, margin=1.12)


HEAD_LAYER = [   # (name prefix, (dy, dz)): layers pulled forward (-Y) from the carriage plate, motors lifted
    ("tip_", (-215, -45)), ("XBC-1", (0, 0)), ("xblk", (60, 0)), ("stiff", (60, 0)), ("EB-1", (0, 70)), ("board", (0, 85)),
    ("CCA-1", (30, 50)), ("zrail", (-35, 0)), ("ZMB", (-35, 90)), ("coupler", (-35, 105)), ("motor", (-35, 140)), ("screw", (-35, 0)),
    ("collars", (-35, -20)), ("ZBH", (-35, -20)), ("ZSW", (-35, 30)), ("switch", (-35, 30)), ("zblk", (-70, 0)), ("ZC", (-105, 0)),
    ("nut", (-105, 25)), ("TP", (-150, 0)), ("CP-1", (0, 0)), ("XSP", (40, 0)), ("MBB-1", (60, 60)), ("mmb", (60, 90)),
    ("XFL-1", (0, 0))]


def ex_head():
    us = sel(mod("head"))
    off = {}
    for u in us:
        n = PARTS[u]["name"]
        d = next((v for pre, v in HEAD_LAYER if n.startswith(pre)), None)
        if d is None:
            d = (-215, 0) if "@" in n else (0, 0)       # tool parts (pipette cradles, gripper) furthest forward
        off[u] = (0, d[0], d[1])
    return off


view("exploded_head", mod("head"), FRONT_LEFT,
     "Head exploded in layers from the back: X-rail blocks and stiffeners behind the carriage plate, then the Z rails and lead screws, Z "
     "blocks, Z carriages, tool plates and the tools pulled forward, with the Z motors lifted. " + SIMPL, explode=ex_head, lens=50, margin=1.1)


def ex_tc():
    us = sel(mod("thermocycler", tc="closed"))
    return explode_scale(us, (0.4, 0.4, 2.4))


view("exploded_thermocycler", mod("thermocycler", tc="closed"), FRONT_LEFT,
     "Thermocycler exploded vertically: fan and duct at the bottom, heat sink, Peltiers and insulation, block and plate, then the lid "
     "stack (pad, heater, lid plate, hood) and its carriage. " + SIMPL, explode=ex_tc, lens=55)


def ex_sub(sub, gap):
    def f():
        us = sel(deck_sub(sub))
        return explode_rank_z(us, gap)
    return f


view("exploded_magnet", deck_sub("magnet_block"), FRONT_LEFT,
     "Passive magnet block exploded vertically: plate, locators, insert plate, magnets and the MB-1 block. " + SIMPL,
     explode=ex_sub("magnet_block", 28), lens=60)
view("exploded_thermal_block", deck_sub("thermal"), FRONT_LEFT,
     "Thermal block exploded vertically, parts in their stacking order. " + SIMPL, explode=ex_sub("thermal", 30), lens=60)
view("exploded_tip_waste", deck_sub("tip_waste"), FRONT_LEFT,
     "Tip waste exploded vertically: strip comb, eject hopper and tip-check posts, insert plate, funnel and bin. " + SIMPL, explode=ex_sub("tip_waste", 40), lens=60)

# ---- sections
view("section_thermocycler", lambda r: r["set"] == "section_tc", (-0.3, -1.0, 0.25),
     f"Section through TC-1 at y = {META['section_tc_y']:.1f} mm (lid closed): from the top, hood and heated lid pressing the SL-1 "
     "sealing lid onto the PCR plate, the block, the graphite and grease interface layers each side of the two Peltiers, the heat sink "
     "fins and the fan duct below the deck. " + SIMPL, lens=70)
for tg, txt in (("zsec_top", "at the top of its stroke"), ("zsec_bottom", f"at the bottom of its stroke ({META['travel_z']:.0f} mm down)")):
    view(f"section_z_axis_{tg[5:]}", (lambda t: (lambda r: r["set"] == t))(tg), (-0.35, -1.0, 0.25),
         f"Section through the 8-channel 300 uL Z axis on the lead-screw plane (y = {META['section_z_y']:.1f} mm), front half removed, axis "
         f"{txt}: the two MGN12H blocks on the 312 mm rail, the T8 screw with its F5-10M thrust bearing at the fixed end and the nut on the "
         "Z carriage, and the flat-cable rolling loop between plate and carriage. " + SIMPL, lens=60, margin=1.08)



# ---- machine details (head poses from export_meshes.py)
def in_set(tag, names=None, prefixes=None, zmax=None):
    def f(r):
        if r["set"] != tag:
            return False
        if zmax is not None and r["bbox"][4] > zmax:
            return False
        if names is None and prefixes is None:
            return True
        return (names and r["name"] in names) or (prefixes and r["name"].startswith(prefixes))
    return f


view("mb1_gripper_approach", in_set("mb1_gripper"), (-0.9, -1.1, 0.55),
     "The gripper approaching a PCR plate on the passive magnet block MB-1 in B2: fingers open, 20 mm above the plate top, the other "
     "three tools up. " + SIMPL, lens=55, margin=1.25,
     frame_pred=lambda r: r["set"] == "mb1_gripper" and (r["nest"] == "B2" or r["name"].startswith(("GR", "pad", "g_")) and r["name"].endswith("@2")))
view("tipcheck_8ch_traverse", in_set("tipcheck"), (-1.0, -0.9, 0.35),
     f"Tip check at A1: the 8-channel 300 uL pipette traverses in Y with its tips 8 mm below the through-beam stretched between the two "
     "TCK-1 posts, so each of the eight tips breaks the beam in turn. " + SIMPL, lens=60, margin=1.3,
     frame_pred=lambda r: r["set"] == "tipcheck" and (r["nest"] == "A1" and "tipcheck" in r["name"] or r["name"].startswith("tip_") and "@3_" in r["name"]
                                                       or r["name"].startswith(("HP2B@3", "HP3B@3"))))
for t, m_ in META.get("eject", {}).items():
    ax = {"pipette_small": 0, "pipette_8ch_50": 1, "pipette_8ch_300": 3}[t]
    view(f"eject_finger_{t}", in_set(m_["set"]), (-1.0, -0.8, 0.35),
         f"Tip ejection, method B, {TOOLTXT[t]}: the Z axis is driven {m_['eject_travel']:g} mm above its normal top so the EJ-1 finger on "
         f"the carriage plate presses the pipette's own ejector; the tips are shown part-way ({m_['tip_fall_shown_mm']:g} mm) through their "
         "fall into the TW-5 hopper over A1. " + SIMPL, lens=55, margin=1.15,
         frame_pred=(lambda st, a: (lambda r: r["set"] == st and (r["nest"] == "A1" and r["name"].endswith(("eject_hopper", "insert", "strip_comb"))
                                                                   or r["name"] in (f"EJ{a}", f"ejb{a}", f"pipette@{a}")
                                                                   or r["name"].startswith("tip_") and f"@{a}_" in r["name"])))(m_["set"], ax))
    own = (lambda st, a: (lambda r: r["set"] == st and (r["name"] == "CP-1" or ("@" not in r["name"] and r["name"].endswith(str(a))
                                                                               and not r["name"].startswith(("xblk", "XSP")))
                                                         or r["name"].endswith(f"@{a}"))))(m_["set"], ax)
    view(f"eject_finger_closeup_{t}", own, (-1.0, -0.35, 0.25),
         f"Close-up of the method-B eject finger EJ-1 over the ejector button of the {TOOLTXT[t]} (button position nominal, measure on the "
         "pipette), with the axis at its eject height (neighbouring axes hidden). " + SIMPL, lens=60, margin=2.6,
         frame_pred=(lambda st, a: (lambda r: r["set"] == st and r["name"] in (f"EJ{a}", f"ejb{a}")))(m_["set"], ax))
view("z_up_and_home_switches", lambda r: main_set(r) and r["module"] == "head" and (r["name"] == "CP-1" or r["name"].endswith("@3") or (
     "@" not in r["name"] and r["name"].endswith("3") and not r["name"].startswith(("xblk", "XSP")))), (1.0, -0.7, 0.35),
     "The 8-channel 300 uL axis from the right: the ZUC-1 cam bars on the Z carriage, with the Z-up roller-lever switch on the +X bar "
     "(made from the normal top to the eject top) and the KW10 home switch on the -X bar (other axes hidden). " + SIMPL, lens=60,
     margin=2.2,
     frame_pred=lambda r: main_set(r) and r["name"] in ("ZUC3", "zup3", "switch3", "ZSW3"))

# ---- later-stage modules (deck_modules2): not part of the base machine
STG = {"SM-1": "cooled orbital shaker SM-1", "WM-1": "capsule wash module WM-1"}
for mod_, m_ in META.get("stage", {}).items():
    tag = m_["set"]
    view(f"later_stage_{mod_.replace('-', '')}", in_set(tag), FRONT_LEFT,
         f"LATER-STAGE MODULE, not part of the base machine: the {STG[mod_]} as a deck insert for nest {m_['nest']}, with its labware. "
         + SIMPL, lens=60)
    view(f"later_stage_{mod_.replace('-', '')}_exploded", in_set(tag), FRONT_LEFT,
         f"LATER-STAGE MODULE, not part of the base machine: the {STG[mod_]} exploded vertically in stacking order. " + SIMPL,
         explode=(lambda t: (lambda: explode_rank_z(sel(in_set(t)), 22)))(tag), lens=55, margin=1.1)
    view(f"later_stage_{mod_.replace('-', '')}_on_deck", (lambda t, n: (lambda r: r["set"] == t or (main_set(r) and r["module"] == "deck_modules"
                                                                                            and r["nest"] != n)))(tag, m_["nest"]), FRONT_LEFT,
         f"LATER-STAGE MODULE fitted: the deck with the {STG[mod_]} in nest {m_['nest']} in place of its normal insert (not the base "
         "layout). " + SIMPL, lens=50)

# ---- bench rig BR-1
def rig(pose_):
    return lambda r: r["set"] in ("br1_static", f"br1_axis_{pose_}")


RIGC = "The bench rig is the first thing to build: one LH-1 Z axis on a belt module over test stations. "
view("br1_34_front_left", rig("rack"), FRONT_LEFT,
     RIGC + "Three-quarter view from the front left with the 8-channel 300 uL pipette over the tip-rack station; stations left to right: tip "
     "rack on load cell LC-1, strip comb and eject hopper, plate nest, balance. " + SIMPL, lens=45)
view("br1_34_front_right", rig("rack"), FRONT_RIGHT,
     RIGC + "Three-quarter view from the front right: BR-3 base plate, two 2040 posts carrying the linear module, and the BR-2 plate "
     "carrying the Z axis. " + SIMPL, lens=45)


def br1_explode():
    off = {}
    for u in sel(rig("rack")):
        st = PARTS[u]["build_step"]
        off[u] = {"br1_1_base": (0, 0, 0), "br1_2_linear_module": (0, 120, 220), "br1_3_stations": (0, -160, 60),
                  "br1_4_z_axis": (0, -60, 330), "br1_5_pipette_cradle": (0, -260, 330)}.get(st, (0, 0, 0))
    return off


view("br1_exploded_machine_vs_rig_parts", rig("rack"), FRONT_LEFT,
     "Bench rig exploded by sub-assembly and coloured by role: orange parts are LH-1 machine parts reused on the rig, purple parts are "
     "rig-only (base, posts, linear module, BR-2 plate, load cells, balance), grey are labware and tips. " + SIMPL,
     explode=br1_explode, lens=45, margin=1.08, role_colours=True)
STN = [("tip_rack", "rack", "the tip-rack station: plain insert with corner locators and a tip rack on the LC-1 platform load cell, the "
        "8-channel nozzles above", ("rack_", "tip_rack", "LC-1")),
       ("comb", "comb", "the strip station: the TW-4D comb and TW-5 eject hopper on a TW-1 insert, hung from the LC-2 load cell over the "
        "tip bin, nozzles engaged under the comb", ("comb", "eject_hopper", "LC-2", "tip_bin")),
       ("eject", "eject", "method-B ejection on the rig: the axis 12 mm above its normal top, the EJ-1 finger on the BR-2 plate over the "
        "pipette's ejector, the tips over the eject hopper", ("eject_hopper", "comb", "EJ3", "ejb3")),
       ("plate_balance", "plate", "the plate nest with a PCR plate and the 6-place balance with its evaporation-trap vial (bought parts as "
        "envelopes)", ("plate_", "pcr_plate", "BAL_")),
       ("pipette_cradle", "rack", "the pipette cradle on the tool plate: HP-1 back plate, V-blocks and straps, the HP-7 datum collar and "
        "HP-8 stop with the disc-spring stack and D2F force switch, the plunger actuator and HP-6 plunger switch",
        ("HP", "hp_act", "hp_spr", "f_sw", "pl_sw", "TP3"))]
for nm, pose_, txt, pre in STN:
    view(f"br1_station_{nm}", rig(pose_), (-1.0, -1.0, 0.55), f"Bench rig, close up: {txt}. " + SIMPL, lens=55, margin=1.3,
         frame_pred=(lambda p_, pr: (lambda r: rig(p_)(r) and r["name"].startswith(pr)))(pose_, pre))
BR1_STEPS = [("br1_1_base", "BR-3 base plate and the two posts"), ("br1_2_linear_module", "belt linear module"),
             ("br1_3_stations", "stations: tip rack on LC-1, comb and hopper on LC-2, plate nest, balance"),
             ("br1_4_z_axis", "BR-2 mount plate with the LH-1 Z axis"), ("br1_5_pipette_cradle", "pipette cradle, pipette and tips")]
for k, (st, label) in enumerate(BR1_STEPS):
    done = {s_ for s_, _ in BR1_STEPS[:k + 1]}
    view(f"br1_build_{k + 1:02d}", (lambda d: (lambda r: rig("rack")(r) and r["build_step"] in d))(done), FRONT_LEFT,
         f"Bench rig build step {k + 1} of {len(BR1_STEPS)}: add the {label}.", lens=45, margin=1.06, frame_pred=rig("rack"))

# ---- build sequence (cumulative), fixed camera framed on the whole machine
STEPS = [("01_base_frame", "base frame and feet"), ("02_uprights_y_rails", "uprights, side beams, Y rails and Y drives"),
         ("03_deck_supports", "deck support beams and post"), ("04_deck_plate", "deck plate"), ("05_deck_modules", "deck inserts and modules"),
         ("06_gantry_beam", "X beam, Y carriages and X drive"), ("07_head", "head"), ("08_thermocycler", "thermocycler TC-1"),
         ("09_electronics_bay", "control bay (plate, board panel, cover, power supplies)"), ("10_panels", "enclosure cage, panels and door")]
for k, (st, label) in enumerate(STEPS):
    done = {s for s, _ in STEPS[:k + 1]}
    view(f"build_{k + 1:02d}_{st[3:]}", (lambda d: (lambda r: MACHINE_ENC(r) and r["build_step"] in d))(done), FRONT_LEFT,
         f"Build step {k + 1} of {len(STEPS)}: add the {label}.", lens=50, margin=1.06, frame_pred=MACHINE_ENC, lines=st != "10_panels")


# ============================================================================================ render loop
def composite(path):
    img = bpy.data.images.load(str(path))
    w, h = img.size
    a = np.empty(w * h * 4, dtype=np.float32)
    img.pixels.foreach_get(a)
    a = a.reshape(h, w, 4)
    al = a[..., 3:4]
    rgb = a[..., :3] * al + np.array(BG, dtype=np.float32) * (1 - al)
    outp = np.concatenate([rgb, np.ones_like(al)], axis=2)
    img.pixels.foreach_set(outp.ravel())
    img.filepath_raw = str(path)
    img.file_format = "PNG"
    img.save()
    bpy.data.images.remove(img)


RENDERS.mkdir(parents=True, exist_ok=True)
done_views = []
for v in VIEWS:
    if ONLY and v["name"] not in ONLY and not any(v["name"].startswith(o.rstrip("*")) for o in ONLY if o.endswith("*")):
        continue
    t = time.time()
    vis = set(sel(v["pred"]))
    for u, o in OBJ.items():
        o.hide_render = u not in vis
        o.location = (0, 0, 0)
    off = v["explode"]() if v["explode"] else {}
    for u, d in off.items():
        if u in OBJ:
            OBJ[u].location = d
    bpy.context.view_layer.update()
    fobjs = [OBJ[u] for u in (sel(v["frame_pred"]) if v.get("frame_pred") else vis)]
    scene.render.use_freestyle = LINES and v["lines"]
    swapped = []
    if v.get("role_colours"):
        for u in vis:
            role = PARTS[u].get("rig_role")
            c = {"machine": "#E07A1F", "rig_only": "#7B4FB0", "labware": "#C9CDD3"}.get(role)
            if c:
                o = OBJ[u]
                swapped.append((o, o.data.materials[0]))
                o.data.materials[0] = material(c, "accent" if role != "labware" else "labware")
    zmin = min(p.z for p in world_corners(fobjs))
    GROUND.location = (0, 0, zmin - 0.5)
    GROUND.hide_render = not v["ground"] or v["cam"] == TOP and False
    frame_camera(fobjs, v["cam"], ortho=v["ortho"], lens=v["lens"], margin=v["margin"], up=v["up"])
    set_lights(v["cam"])
    path = RENDERS / f"{v['name']}.png"
    scene.render.filepath = str(path)
    bpy.ops.render.render(write_still=True)
    composite(path)
    for o, m in swapped:
        o.data.materials[0] = m
    done_views.append({k: v[k] for k in ("name", "caption")})
    log(f"{v['name']}: {len(vis)} parts, {time.time() - t:.0f}s")

# build-sequence contact strips from the step images
def contact_strip(paths, out_name, ncol, caption):
    if not all(p.exists() for p in paths):
        return
    tiles = []
    for p in paths:
        img = bpy.data.images.load(str(p))
        w, h = img.size
        a = np.empty(w * h * 4, dtype=np.float32)
        img.pixels.foreach_get(a)
        tiles.append(a.reshape(h, w, 4)[::3, ::3])
        bpy.data.images.remove(img)
    th, tw = tiles[0].shape[:2]
    pad = 8
    nr = (len(tiles) + ncol - 1) // ncol
    grid = np.ones((nr * th + (nr + 1) * pad, ncol * tw + (ncol + 1) * pad, 4), dtype=np.float32)
    grid[..., :3] = 0.85
    for k, tile in enumerate(tiles):
        r, c = nr - 1 - k // ncol, k % ncol          # Blender images are bottom-up: first row at the top
        y0, x0 = pad + r * (th + pad), pad + c * (tw + pad)
        grid[y0:y0 + th, x0:x0 + tw] = tile
    H, W = grid.shape[:2]
    out = bpy.data.images.new(out_name, W, H, alpha=False)
    out.pixels.foreach_set(grid.ravel())
    out.filepath_raw = str(RENDERS / f"{out_name}.png")
    out.file_format = "PNG"
    out.save()
    done_views.append({"name": out_name, "caption": caption})


contact_strip([RENDERS / f"build_{k + 1:02d}_{st[3:]}.png" for k, (st, _) in enumerate(STEPS)], "build_sequence_strip", 5,
              f"LH-1 build sequence in {len(STEPS)} steps, left to right and top to bottom, from the base frame to the enclosure panels and "
              "door. " + SIMPL)
contact_strip([RENDERS / f"br1_build_{k + 1:02d}.png" for k in range(len(BR1_STEPS))], "br1_build_sequence_strip", len(BR1_STEPS),
              f"Bench rig BR-1 build sequence in {len(BR1_STEPS)} steps: base and posts, linear module, stations, Z axis on BR-2, pipette "
              "cradle. " + SIMPL)

prev = {}
vf = OUT / "render_views.json"
if vf.exists():
    prev = {d["name"]: d for d in json.loads(vf.read_text())}
for d in done_views:
    prev[d["name"]] = d
order = [v["name"] for v in VIEWS] + ["build_sequence_strip", "br1_build_sequence_strip"]
vf.write_text(json.dumps([prev[n] for n in order if n in prev], indent=1))
log("done", len(done_views), "views")
