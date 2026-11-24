"""SVG block diagram of a harness: devices as boxes grouped by location in up to three columns, one line per cable
labelled with its cable_id and coloured by signal class; cables inside one column are drawn as loops on its outer side.

Inputs: wiring rows, devices {name: {group, kind}}, and a style mapping (see README.md.txt):
  title, subtitle ({cables} and {devices} are filled in), width, columns {L/C/R: x},
  groups [{id, column, title}] (drawing order), classes [{key, colour, legend, prefixes, first_word_contains, dash,
  width}] (first match wins; a class with neither prefixes nor first_word_contains matches everything),
  dashed_note (notes text that makes a line dashed '3,3'), kind_fills {kind: colour}, inset {x, title, lines}.
"""
from __future__ import annotations

from html import escape

DEFAULTS = {"width": 2000, "box_w": 230, "box_h": 20, "gap": 6, "group_gap": 46, "top": 120, "line_width": 1.3,
            "dashed_note": "RESERVED", "reserved_dash": "3,3", "kind_fills": {}, "subtitle": "{cables} cables, {devices} devices."}


class DrawError(ValueError):
    pass


def classify(signal: str, classes: list[dict]) -> dict:
    for c in classes:
        pre, fw = c.get("prefixes"), c.get("first_word_contains")
        if not pre and not fw:
            return c
        if (pre and signal.startswith(tuple(pre))) or (fw and any(x in signal.split(" ")[0] for x in fw)):
            return c
    raise DrawError(f"no signal class matches {signal!r} (add a catch-all class)")


def draw_svg(rows: list[dict], devices: dict, style: dict) -> tuple[str, dict]:
    s = dict(DEFAULTS)
    s.update(style)
    BW, BH, GAP, GROUP_GAP, TOP = s["box_w"], s["box_h"], s["gap"], s["group_gap"], s["top"]
    COLX = s["columns"]
    groups = s["groups"]
    layout = {g["id"]: g["column"] for g in groups}
    for d, v in devices.items():
        if v["group"] not in layout:
            raise DrawError(f"device {d}: group {v['group']!r} has no column in the style")
    for r in rows:
        for d in (r["from_device"], r["to_device"]):
            if d not in devices:
                raise DrawError(f"{r['cable_id']}: device {d} is not in the devices list")
    pos, groups_box = {}, {}
    col_y = {c: TOP for c in COLX}
    for g in groups:
        col = g["column"]
        members = [d for d, v in devices.items() if v["group"] == g["id"]]
        y0 = col_y[col] + 22
        for i, d in enumerate(members):
            pos[d] = (COLX[col], y0 + i * (BH + GAP))
        y1 = y0 + len(members) * (BH + GAP)
        groups_box[g["id"]] = (COLX[col] - 14, col_y[col], BW + 28, y1 - col_y[col] + 6)
        col_y[col] = y1 + GROUP_GAP
    H = max(col_y.values()) + 330
    W = s["width"]
    sub = s["subtitle"].format(cables=len(rows), devices=len(devices))
    out = [f'<svg xmlns="http://www.w3.org/2000/svg" width="{W}" height="{H}" viewBox="0 0 {W} {H}" font-family="Helvetica, Arial, sans-serif">',
           f'<rect width="{W}" height="{H}" fill="#ffffff"/>',
           f'<text x="20" y="34" font-size="22" font-weight="bold" fill="#111">{escape(s.get("title", "Wiring harness"))}</text>',
           f'<text x="20" y="58" font-size="13" fill="#333">{escape(sub)}</text>']
    titles = {g["id"]: g.get("title", g["id"]) for g in groups}
    for grp, (x, y, w, h) in groups_box.items():
        out.append(f'<rect x="{x}" y="{y}" width="{w}" height="{h}" rx="8" fill="#f5f7fa" stroke="#9aa5b1"/>')
        out.append(f'<text x="{x + 8}" y="{y + 15}" font-size="12" font-weight="bold" fill="#1f2d3d">{escape(titles[grp])}</text>')
    # cables
    side = layout
    lines, labels = [], []
    used_intra = {}
    for r in rows:
        a, b = r["from_device"], r["to_device"]
        c = classify(r["signal"], s["classes"])
        col = c["colour"]
        if c.get("dash"):
            dash = f' stroke-dasharray="{c["dash"]}"'
        elif s["dashed_note"] and s["dashed_note"] in r.get("notes", ""):
            dash = f' stroke-dasharray="{s["reserved_dash"]}"'
        else:
            dash = ""
        wdt = c.get("width", s["line_width"])
        ga, gb = devices[a]["group"], devices[b]["group"]
        (xa, ya), (xb, yb) = pos[a], pos[b]
        ya += BH / 2; yb += BH / 2
        if side[ga] == side[gb]:
            k = used_intra.get(side[ga], 0) % 16
            used_intra[side[ga]] = used_intra.get(side[ga], 0) + 1
            if side[ga] == "L":
                x0 = xa; bulge = x0 - 20 - 7 * k
            else:
                x0 = xa + BW; bulge = x0 + 20 + 9 * k
            d = f"M{x0},{ya} C{bulge},{ya} {bulge},{yb} {x0},{yb}"
            lx, ly = bulge + (-4 if side[ga] == "L" else 4), (ya + yb) / 2
            anchor = "end" if side[ga] == "L" else "start"
        else:
            if COLX[side[ga]] > COLX[side[gb]]:
                (xa, ya, ga), (xb, yb, gb) = (xb, yb, gb), (xa, ya, ga)
            x0, x1 = xa + BW, xb
            d = f"M{x0},{ya} C{(x0 + x1) / 2},{ya} {(x0 + x1) / 2},{yb} {x1},{yb}"
            lx, ly = (x0 + x1) / 2, (ya + yb) / 2
            anchor = "middle"
        lines.append(f'<path d="{d}" fill="none" stroke="{col}" stroke-width="{wdt}"{dash} opacity="0.85"><title>{escape(r["cable_id"])}: '
                     f'{escape(a)} {escape(r.get("from_connector", ""))} -> {escape(b)} {escape(r.get("to_connector", ""))}; {escape(r["signal"])}; '
                     f'{r["conductors"]} x {r["gauge_mm2"]} mm2; {r.get("route", "")}; {r["length_mm_estimate"]} mm</title></path>')
        labels.append(f'<text x="{lx:.1f}" y="{ly + 3:.1f}" font-size="8.5" text-anchor="{anchor}" fill="{col}" '
                      f'stroke="#ffffff" stroke-width="2.5" paint-order="stroke">{escape(r["cable_id"])}</text>')
    out += lines
    fills = s["kind_fills"]
    for d, (x, y) in pos.items():
        kind = devices[d]["kind"]
        fill = fills.get(kind, "#ffffff")
        out.append(f'<rect x="{x}" y="{y}" width="{BW}" height="{BH}" rx="3" fill="{fill}" stroke="#4a5568"/>')
        out.append(f'<text x="{x + 5}" y="{y + 14}" font-size="10.5" fill="#111"><tspan font-weight="bold">{escape(d)}</tspan> '
                   f'<tspan fill="#555">{escape(kind)}</tspan></text>')
    out += labels
    # legend
    ly = max(col_y.values()) + 10
    out.append(f'<text x="20" y="{ly}" font-size="13" font-weight="bold" fill="#111">Rails and signal classes</text>')
    for i, c in enumerate(s["classes"]):
        x, y = 20 + (i % 3) * 300, ly + 18 + (i // 3) * 18
        dash = f' stroke-dasharray="{c["dash"]}"' if c.get("dash") else ""
        out.append(f'<line x1="{x}" y1="{y}" x2="{x + 34}" y2="{y}" stroke="{c["colour"]}" stroke-width="2.5"{dash}/>')
        out.append(f'<text x="{x + 42}" y="{y + 4}" font-size="11" fill="#222">{escape(c.get("legend", c["key"]))}</text>')
    inset = s.get("inset")
    if inset:
        sx, sy = inset.get("x", 1000), ly - 12
        steps = inset.get("lines") or []
        out.append(f'<rect x="{sx}" y="{sy}" width="{inset.get("width", 880)}" height="{inset.get("height", 190)}" rx="8" fill="#fff7f7" stroke="#c00000"/>')
        out.append(f'<text x="{sx + 10}" y="{sy + 20}" font-size="13" font-weight="bold" fill="#c00000">{escape(inset.get("title", ""))}</text>')
        for i, t in enumerate(steps):
            out.append(f'<text x="{sx + 10}" y="{sy + 42 + i * 18}" font-size="11" fill="#222">{escape(t)}</text>')
    out.append("</svg>")
    return "\n".join(out), {"devices": len(pos), "cables": len(rows), "width": W, "height": H}
