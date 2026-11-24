#!/usr/bin/env python3
"""Draw the LH-1 documentation figures from the CAD and electrical reports (nothing is drawn by hand).

Run from the repository root:   python tools/make_drawings.py
Reads:  cad/lh1/machine_family/out/{MVP,S,M,L}/report.json, out/S/family_S.json, out/MVP/roadmap.json,
        cad/lh1/head_l/out/report.json (exchange steps), electronics/lh1/system.yaml (system blocks)
Writes: docs/img/*.svg and a PNG copy of each (PNG needs cairosvg; if this interpreter lacks it, another python3 on the
        PATH that has it is used; otherwise the PNGs are skipped and the script says so).
Every figure sits on its own white card with dark ink, so it reads the same on a white or a dark page.
Status shown on every figure: designed and simulated only; nothing is built or measured.
"""
from __future__ import annotations

import json
import shutil
import subprocess
import sys
from pathlib import Path
from xml.sax.saxutils import escape

ROOT = Path(__file__).resolve().parents[1]
FAM = ROOT / "cad" / "lh1" / "machine_family" / "out"
IMG = ROOT / "docs" / "img"
FONT = "Helvetica, Arial, 'DejaVu Sans', sans-serif"
INK, MUTED, LINE = "#1b1f24", "#4b5563", "#9ca3af"
FILL = {"tips": "#dbeafe", "liquid": "#dcfce7", "plate": "#fef9c3", "thermal": "#fee2e2", "waste": "#e5e7eb", "module": "#ede9fe",
        "blank": "#f9fafb", "dock": "#ffedd5", "other": "#f3f4f6"}
STATUS = "Designed and simulated only. Nothing is built or measured."

SHORT = {  # station code -> short label (the CAD report's own 'kind' is shown in the docs tables)
    "TC1": ("Thermocycler TC-1", "thermal"), "TC1LID": ("TC-1 lid park", "thermal"), "RES1": ("Reservoir, 4 troughs", "liquid"),
    "RES2": ("Reservoir, 12 columns", "liquid"), "STACK": ("Plate stack x4", "plate"), "STACK2": ("Plate stack 2", "plate"),
    "SHAKER": ("Heater-shaker", "module"), "ATR3": ("Tube rack, ambient", "liquid"), "CTB1": ("Cold tube block", "thermal"),
    "CTB2": ("Cold large-tube block", "thermal"), "TPKS": ("Tip parking", "tips"), "TIPWASTE": ("Tip waste + gate", "waste"),
    "STRIP": ("Cold strip carrier", "thermal"), "LW": ("Liquid waste", "waste"), "MB1": ("Magnet block", "module"),
    "HAZ": ("Lidded trough (hazardous liquid)", "liquid"), "P10": ("Tips 10 uL", "tips"), "P200": ("Tips 200 uL", "tips"),
    "P300": ("Tips 300 uL", "tips"), "CPB1": ("Cold plate", "thermal"), "P1000R": ("Tips 1000 uL (rack)", "tips"),
    "PLATE": ("Plate", "plate"), "EMPTY": ("blank insert", "blank"), "MAG2": ("Magnet 2", "module"), "BAL": ("Balance", "module"),
    "WN1": ("Weighing nest", "module"), "LIDPARK2": ("Lid park 2", "plate"), "ATR1": ("Tube rack 1", "liquid"), "ATR2": ("Tube rack 2", "liquid"),
}


def load(p):
    p = Path(p)
    return json.loads(p.read_text()) if p.exists() else None


class Svg:
    def __init__(self, w, h, title):
        self.w, self.h, self.items = w, h, []
        self.rect(0, 0, w, h, fill="#ffffff", stroke="none")
        self.text(24, 34, title, size=20, weight="bold")
        self.text(24, h - 14, STATUS + "  Source files named in docs/.", size=11, fill=MUTED)

    def rect(self, x, y, w, h, fill="none", stroke=INK, sw=1.2, rx=0, dash=None):
        d = f' stroke-dasharray="{dash}"' if dash else ""
        self.items.append(f'<rect x="{x:.1f}" y="{y:.1f}" width="{w:.1f}" height="{h:.1f}" rx="{rx}" fill="{fill}" stroke="{stroke}" stroke-width="{sw}"{d}/>')

    def line(self, x1, y1, x2, y2, stroke=INK, sw=1.2, arrow=False, dash=None):
        a = ' marker-end="url(#arr)"' if arrow else ""
        d = f' stroke-dasharray="{dash}"' if dash else ""
        self.items.append(f'<line x1="{x1:.1f}" y1="{y1:.1f}" x2="{x2:.1f}" y2="{y2:.1f}" stroke="{stroke}" stroke-width="{sw}"{a}{d}/>')

    def text(self, x, y, s, size=12, anchor="start", weight="normal", fill=INK):
        for i, ln in enumerate(str(s).split("\n")):
            self.items.append(f'<text x="{x:.1f}" y="{y + i * size * 1.2:.1f}" font-family="{FONT}" font-size="{size}" '
                              f'font-weight="{weight}" text-anchor="{anchor}" fill="{fill}">{escape(ln)}</text>')

    def svg(self):
        head = (f'<svg xmlns="http://www.w3.org/2000/svg" width="{self.w}" height="{self.h}" viewBox="0 0 {self.w} {self.h}">'
                '<defs><marker id="arr" viewBox="0 0 10 10" refX="9" refY="5" markerWidth="7" markerHeight="7" orient="auto-start-reverse">'
                f'<path d="M0,0 L10,5 L0,10 z" fill="{INK}"/></marker></defs>')
        return head + "\n".join(self.items) + "</svg>\n"


def wrap(s, n):
    out, cur = [], ""
    for w in str(s).split():
        if len(cur) + len(w) + 1 > n and cur:
            out.append(cur)
            cur = w
        else:
            cur = (cur + " " + w).strip()
    if cur:
        out.append(cur)
    return "\n".join(out)


def save(name, s: Svg, written):
    IMG.mkdir(parents=True, exist_ok=True)
    (IMG / f"{name}.svg").write_text(s.svg())
    written.append(name)


def missing(name, title, what, written):
    s = Svg(900, 220, title)
    s.rect(24, 60, 852, 110, fill="#fff7ed", stroke="#c2410c", rx=8)
    s.text(450, 110, "MVP files not yet generated", size=20, anchor="middle", weight="bold", fill="#9a3412")
    s.text(450, 140, f"Needs {what}. Re-run the export after the MVP build.", size=13, anchor="middle")
    save(name, s, written)


# ------------------------------------------------------------------------------------------------ deck maps (top view)
def deck_map(size, written):
    rep = load(FAM / size / "report.json")
    if rep is None:
        return missing(f"deck_map_{size}", f"LH-1 {size}: deck map", f"cad/lh1/machine_family/out/{size}/report.json", written)
    places = rep.get("deck_positions") or rep.get("deck_places") or {}
    env = rep["envelope_mm"]
    fam = load(FAM / size / f"family_{size}.json") or {}
    bays = (fam.get("dock") or {}).get("bays") or []
    x0, x1 = env["x"]
    y0, y1 = env["y"]
    k = 0.75
    W = int((x1 - x0) * k + 330)
    H = int((y1 - y0) * k + 170)
    title = {"MVP": "LH-1 MVP: deck map, top view", "S": "LH-1 starter (S): deck map, top view"}.get(size, f"LH-1 {size}: deck map")
    s = Svg(W, H, title)
    ox, oy = 40, 70

    def X(x):
        return ox + (x - x0) * k

    def Y(y):          # +y is towards the back: draw the back at the top
        return oy + (y1 - y) * k

    s.rect(X(x0), Y(y1), (x1 - x0) * k, (y1 - y0) * k, fill="#fafafa", stroke=INK, sw=1.6)
    wf = (env.get("with_fan") or [None])[0]
    fan = " (width includes the extraction fan on the left side panel)" if wf and round(x1 - x0) == wf else ""
    s.text(X(x0) + 6, Y(y1) + 16, f"machine outline {round(x1 - x0)} x {round(y1 - y0)} mm{fan}", size=11, fill=MUTED)
    s.text(X((x0 + x1) / 2), Y(y0) + 22, "FRONT (door)", size=12, anchor="middle", weight="bold")
    s.text(X((x0 + x1) / 2), Y(y1) - 6, "BACK", size=12, anchor="middle", weight="bold")
    used = set()
    for pid, v in places.items():
        if not isinstance(v, dict) or "centre_xy" not in v:
            continue
        st = v.get("station") or v.get("type") or "?"
        cx, cy = v["centre_xy"]
        lab, cat = SHORT.get(st.split()[0], (st, "other"))
        if st.startswith("EMPTY"):
            lab, cat = "blank insert", "blank"
        w_, h_ = (130, 108) if pid != "B0" else (92, 94)
        s.rect(X(cx - w_ / 2), Y(cy + h_ / 2), w_ * k, h_ * k, fill=FILL[cat], stroke=INK, sw=1.0, rx=3)
        s.text(X(cx), Y(cy) - 4, pid, size=14, anchor="middle", weight="bold")
        s.text(X(cx), Y(cy) + 12, wrap(lab, 16), size=10, anchor="middle", fill=INK if cat != "blank" else MUTED)
        used.add(cat)
    for b in bays:
        bx, by = b["x"], b["y"]
        s.rect(X(bx - 30), Y(by + 25), 60 * k, 50 * k, fill=FILL["dock"], stroke=INK, sw=0.9, rx=2)
        s.text(X(bx), Y(by) + 4, f"bay {b['bay']}", size=9, anchor="middle")
    if bays:
        used.add("dock")
        s.text(X(bays[0]["x"]) - 28 * k, Y(bays[0]["y"] + 25) - 6, "tool dock (8 bays)", size=11, weight="bold")
    # legend
    lx, ly = W - 270, 70
    s.text(lx, ly, "Key", size=13, weight="bold")
    names = {"tips": "tips", "liquid": "liquids and tubes", "plate": "plates and lids", "thermal": "thermal (heated or cooled)",
             "waste": "waste", "module": "module (magnet, shaker, weighing)", "blank": "blank (plain insert, free place)",
             "dock": "tool dock bay", "other": "other"}
    i = 0
    for cat in FILL:
        if cat in used:
            s.rect(lx, ly + 12 + i * 22, 18, 14, fill=FILL[cat], stroke=INK, sw=0.8)
            s.text(lx + 26, ly + 24 + i * 22, names[cat], size=11)
            i += 1
    s.text(lx, ly + 40 + i * 22, wrap("Places on a 132 x 110 mm grid. Rows F (front), A, B, G (back); columns 0 to 4. "
                                      "Every place takes the 130 x 108 x 5 mm insert of the deck-module standard.", 34), size=10, fill=MUTED)
    s.text(lx, ly + 130 + i * 22, f"Source: cad/lh1/machine_family/out/{size}/report.json", size=10, fill=MUTED)
    save(f"deck_map_{size}", s, written)


# ------------------------------------------------------------------------------------------------ outline drawings
def outlines(written):
    sizes = [z for z in ("MVP", "S", "M", "L") if (FAM / z / "report.json").exists()]
    reps = {z: load(FAM / z / "report.json") for z in sizes}
    k = 0.14
    W, H = 1300, 700
    s = Svg(W, H, "LH-1 outline dimensions: MVP, S, M and L (to scale)")
    x = 40
    for z in sizes:
        e = reps[z]["envelope_mm"]
        w_, d_, h_ = e.get("W_D_H_mm") or e.get("without_fan")
        wf = (e.get("with_fan") or [w_])[0]
        # plan view (top) and front view (bottom)
        top = 110
        s.text(x, top - 30, {"MVP": "MVP (stage 0)", "S": "S, starter", "M": "M, intermediate", "L": "L, large"}[z], size=15, weight="bold")
        s.text(x, top - 12, "plan", size=11, fill=MUTED)
        s.rect(x, top, w_ * k, d_ * k, fill="#f3f4f6")
        s.line(x, top + d_ * k + 10, x + w_ * k, top + d_ * k + 10, arrow=True)
        s.line(x + w_ * k, top + d_ * k + 10, x, top + d_ * k + 10, arrow=True)
        s.text(x + w_ * k / 2, top + d_ * k + 26, f"W {w_}", size=11, anchor="middle")
        s.line(x + w_ * k + 10, top, x + w_ * k + 10, top + d_ * k, arrow=True)
        s.line(x + w_ * k + 10, top + d_ * k, x + w_ * k + 10, top, arrow=True)
        s.text(x + w_ * k + 16, top + d_ * k / 2 + 4, f"D {d_}", size=11)
        ft = 360
        s.text(x, ft - 12, "front", size=11, fill=MUTED)
        s.rect(x, ft, w_ * k, h_ * k, fill="#f3f4f6")
        s.line(x + w_ * k + 10, ft, x + w_ * k + 10, ft + h_ * k, arrow=True)
        s.line(x + w_ * k + 10, ft + h_ * k, x + w_ * k + 10, ft, arrow=True)
        s.text(x + w_ * k + 16, ft + h_ * k / 2 + 4, f"H {h_}", size=11)
        mass = reps[z]["mass_kg"]
        mk = mass.get("total_kg_ESTIMATE") or mass.get("total_kg (ESTIMATE)") or next((v for kk, v in mass.items() if "total" in kk and isinstance(v, (int, float))), None)
        note = f"{w_} x {d_} x {h_} mm" + (f"\n{wf} mm wide with the extraction fan" if wf != w_ else "\nno extraction fan fitted")
        if mk:
            note += f"\nabout {round(mk)} kg (estimate)"
        s.text(x, ft + h_ * k + 24, note, size=11)
        x += max(w_ * k + 120, 300)
    s.text(40, H - 40, "Units mm. W x D x H without the extraction fan unless stated. Sources: cad/lh1/machine_family/out/<size>/report.json (envelope_mm, mass_kg).",
           size=10, fill=MUTED)
    save("outline_sizes", s, written)


# ------------------------------------------------------------------------------------------------ system block diagram
def system_blocks(written):
    try:
        import yaml
        sysd = yaml.safe_load((ROOT / "electronics" / "lh1" / "system.yaml").read_text())
    except Exception:
        sysd = {}
    from collections import Counter
    kinds = Counter((v.get("kind") or v.get("type") or "?") for v in (sysd.get("devices") or {}).values())
    rails = list((sysd.get("rails") or {}).keys())
    W, H = 1180, 700
    s = Svg(W, H, "LH-1 system blocks: power, control, motion, tools, modules, sensing, safety")

    def box(x, y, w, h, head, body, fill):
        s.rect(x, y, w, h, fill=fill, stroke=INK, rx=8)
        s.text(x + 12, y + 22, head, size=14, weight="bold")
        s.text(x + 12, y + 42, body, size=11)

    box(40, 70, 340, 150, "Power", "mains inlet IEC C14, earth stud\nfour bought supplies (24 V x3, 5 V)\nrails: " +
        wrap(", ".join(rails), 40) + "\nfuse per rail and branch", "#fef9c3")
    box(420, 70, 340, 150, "Control", "host computer (runs the planner)\nmain board: Octopus Pro (Klipper)\nio board: second Octopus Pro\n"
        "two head boards (MMB, over CAN)\nUSB hubs for cameras and modules", "#dbeafe")
    box(800, 70, 340, 150, "Safety", "e-stop into safety relay SR1 + contactors\ncoded door sensor into SR2 + contactors\n"
        "e-stop and door remove MOTION power\nover-temperature cut-outs, host watchdog\nno safety assessment done", "#fee2e2")
    box(40, 300, 340, 170, "Motion", f"{kinds.get('motor', 0)} motors in system.yaml:\nX, two Y (closed-loop NEMA 17)\n"
        "Z per tool carriage (A, B), gripper Z\ntwo plunger pushers (LGA35)\ndock lift bar, thermocycler lid", "#dcfce7")
    box(420, 300, 340, 170, "Tools and head", "two tool carriages, each takes any tool\nkinematic coupling: 3 balls, 3 vees,\n"
        "retention knob; toggle latch\neight-bay passive dock with lift bar\npipette tools: bought hand pipettes\nin machined cradles; syringe tool", "#ede9fe")
    box(800, 300, 340, 170, "Deck modules", "thermocycler TC-1 (Peltier, heated lid)\nthermal modules (Peltier, bought controller)\n"
        "magnet block, heater-shaker (bought)\nliquid waste to canisters\nmodule standard: 4 x M4, 2 dowels,\nkeyed connector per module place", "#ffedd5")
    box(420, 520, 720, 120, "Sensing", f"{kinds.get('switch', 0)} switches, {kinds.get('sensor', 0)} sensors, {kinds.get('camera', 0)} cameras in system.yaml: "
        "home and limit switches,\ntool lock / bay / force switches, through-beam tip gate, bin-full beam, waste level switches,\n"
        "head camera, tip-check camera, overview camera. Checks can only hold a run; they never steer a move.", "#f3f4f6")
    box(40, 520, 340, 120, "Firmware", "Klipper configuration: printer.cfg\n+ one size file (MVP, S, M or L)\n"
        "guard macros refuse unsafe commands;\nchecked by electronics/lh1/*.py", "#e5e7eb")
    s.line(380, 145, 420, 145, arrow=True)
    s.line(760, 145, 800, 145, arrow=True)
    s.line(210, 220, 210, 300, arrow=True)
    s.line(590, 220, 590, 300, arrow=True)
    s.line(970, 220, 970, 300, arrow=True)
    s.line(590, 470, 590, 520, arrow=True)
    s.line(210, 470, 210, 520, arrow=True)
    s.line(800, 200, 380, 330, dash="5 4", arrow=True)
    s.text(600, 268, "MOTION power switched by e-stop and door", size=10, anchor="middle", fill=MUTED)
    s.text(40, H - 36, "Sources: electronics/lh1/system.yaml (supplies, rails, controllers, devices, safety_relays), control_bay.yaml, wiring.csv.",
           size=10, fill=MUTED)
    save("system_blocks", s, written)


# ------------------------------------------------------------------------------------------------ roadmap
def roadmap(written):
    rm = load(FAM / "MVP" / "roadmap.json")
    if rm is None:
        return missing("roadmap", "LH-1 roadmap: MVP to the large machine", "cad/lh1/machine_family/out/MVP/roadmap.json", written)
    st = rm["stages"]
    n = len(st)
    W = 1180
    rowh = 92
    H = 100 + n * rowh + 50
    s = Svg(W, H, "LH-1 roadmap: from the MVP to the large machine")
    for i, g in enumerate(st):
        y = 70 + i * rowh
        blocked = bool(g.get("blocked_by"))
        s.rect(40, y, 300, rowh - 22, fill="#dbeafe" if i == 0 else "#f3f4f6", stroke=INK, rx=8)
        s.text(54, y + 22, f"Stage {g['id']}", size=14, weight="bold")
        s.text(54, y + 40, wrap(g["name"], 40), size=11)
        s.text(360, y + 18, "demonstrates: " + wrap(g["demonstrates"], 110).split("\n")[0], size=11)
        s.text(360, y + 36, "gate to next: " + wrap(g["test_before_next_stage"], 110).split("\n")[0] + " ...", size=11, fill=MUTED)
        if blocked:
            s.rect(360, y + 46, 780, 20, fill="#fff7ed", stroke="#c2410c", rx=4)
            s.text(368, y + 60, "OPEN PROBLEM: " + wrap(g["blocked_by"][0], 120).split("\n")[0], size=10, fill="#9a3412")
        if i < n - 1:
            s.line(190, y + rowh - 22, 190, y + rowh, arrow=True)
    s.text(40, H - 34, "Source: cad/lh1/machine_family/out/MVP/roadmap.json. Full text of every stage: docs/roadmap.md.", size=10, fill=MUTED)
    save("roadmap", s, written)


# ------------------------------------------------------------------------------------------------ tool exchange sequence
def exchange(written):
    hr = load(ROOT / "cad" / "lh1" / "head_l" / "out" / "report.json")
    steps = list(((hr or {}).get("exchange") or {}).get("steps_s", {}).items())
    total = ((hr or {}).get("exchange") or {}).get("total_s")
    lanes = ["Gantry X-Y", "Carriage Z", "Dock lift bar", "Latch", "Tool"]
    who = []
    for txt, sec in steps:
        t = txt.lower()
        acts = set()
        if "xy" in t or t.startswith("x ") or "x back" in t:
            acts.add(0)
        if "z " in t or t.startswith("z") or "z and" in t:
            acts.add(1)
        if "bar" in t:
            acts.add(2)
        if "latch" in t or "toggle" in t or "jaw" in t or "verify" in t:
            acts.add(3)
        if "tool" in t or "lugs" in t or "balls" in t or "pins" in t:
            acts.add(4)
        who.append((txt, sec, acts or {0}))
    W = 1180
    rowh = 34
    H = 150 + len(who) * rowh + 80
    s = Svg(W, H, "Tool exchange sequence: put one tool down, pick another up (as designed)")
    lx = 560
    lw = 120
    for j, ln in enumerate(lanes):
        x = lx + j * lw
        s.text(x + lw / 2, 78, ln, size=12, anchor="middle", weight="bold")
        s.line(x + lw / 2, 88, x + lw / 2, 100 + len(who) * rowh, stroke=LINE, dash="4 4")
    s.text(40, 78, "Step (seconds, calculated)", size=12, weight="bold")
    t_acc = 0.0
    for i, (txt, sec, acts) in enumerate(who):
        y = 108 + i * rowh
        t_acc += sec
        s.text(40, y + 4, wrap(f"{i + 1}. {txt}", 78).split("\n")[0], size=10.5)
        s.text(520, y + 4, f"{sec:g}", size=10.5, anchor="end", fill=MUTED)
        for j in sorted(acts):
            x = lx + j * lw + lw / 2
            s.rect(x - 40, y - 10, 80, 20, fill="#dbeafe" if j != 3 else "#fee2e2", stroke=INK, sw=0.8, rx=4)
            s.text(x, y + 4, ["move", "move", "move", "open/close", "seat/leave"][j], size=10, anchor="middle")
    yb = 108 + len(who) * rowh
    s.text(40, yb + 10, f"Total about {total:g} s per exchange (head report, calculated; nothing measured)." if total else "", size=12, weight="bold")
    s.text(40, yb + 32, wrap("Open problems: the latch must hold its keyhole on the knob head while bar and Z move in alternate 1 mm "
                             "steps, and the one-sided wedge pushes the tool sideways while latching (docs/open-problems.md, W1 and W2).", 150),
           size=11, fill="#9a3412")
    s.text(40, H - 34, "Source: cad/lh1/head_l/out/report.json (exchange.steps_s); firmware macros _LH_DROP and _LH_PICKUP in firmware/lh1/printer.cfg.",
           size=10, fill=MUTED)
    save("tool_exchange_sequence", s, written)


def to_png(names):
    try:
        import cairosvg  # noqa: F401
        for n in names:
            cairosvg.svg2png(url=str(IMG / f"{n}.svg"), write_to=str(IMG / f"{n}.png"), output_width=None, scale=1.5)
        return "cairosvg"
    except ImportError:
        pass
    for cand in ("python3", "/opt/homebrew/bin/python3", "/usr/local/bin/python3", "/usr/bin/python3"):
        w = shutil.which(cand) or (cand if Path(cand).exists() else None)
        if w and Path(w).resolve() != Path(sys.executable).resolve():
            if subprocess.run([w, "-c", "import cairosvg"], capture_output=True).returncode == 0:
                r = subprocess.run([w, __file__, "--png-only"] + names, capture_output=True, text=True)
                if r.returncode == 0:
                    return f"cairosvg via {Path(w).name}"
    if shutil.which("rsvg-convert"):
        for n in names:
            subprocess.run(["rsvg-convert", "-z", "1.5", "-o", str(IMG / f"{n}.png"), str(IMG / f"{n}.svg")], check=True)
        return "rsvg-convert"
    return None


def main(argv):
    if "--png-only" in argv:
        import cairosvg
        for n in argv[argv.index("--png-only") + 1:]:
            cairosvg.svg2png(url=str(IMG / f"{n}.svg"), write_to=str(IMG / f"{n}.png"), scale=1.5)
        return 0
    written = []
    deck_map("MVP", written)
    deck_map("S", written)
    outlines(written)
    system_blocks(written)
    roadmap(written)
    exchange(written)
    how = to_png(written)
    print(json.dumps({"svg": written, "png": how or "SKIPPED: install cairosvg (pip install cairosvg) to write the PNG copies"}))
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv))
