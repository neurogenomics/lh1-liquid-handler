"""LH-1 deck rule checker for every size of the family (S, M, L) and for revision L.

The K2 rules of deck_k (R1 behind-height, R2 beside-height, R3 tip frames in the back row) plus the ten family rules N1-N10 found by
running the machine_l interference engine on the size-family layouts (cad/lh1/size_family/out/size_family_study.txt, section 2.2).
This is the checker used by cad/lh1/machine_family/build.py for every size; the station catalogue (height above the seat, pipette work,
under-deck parts, gripper use) is the one the study used, with the heights after the family recesses.

Status: rule check (CALCULATED from the model's clearances); a layout that passes still needs the CAD run (machine_family/build.py),
which is the real test.  Nothing built or measured.
"""
from __future__ import annotations

# station key -> (kind, height above the seat mm, pipette work, needs the gripper, has parts under the deck)
#   work: "deep" (eight- and single-channel into wells), "deep_single_only" (single channels into tubes), "shallow" (tip pick-up /
#   eject / free dispense at the top), "centre" (dispense at the centre only), None (gripper only or nothing)
STATIONS = {
    "TC1": ("thermocycler TC-1 (double place with its lid park behind)", 21.0, "deep", True, True),
    "TC1LID": ("TC-1 lid park", 40.0, None, True, False),
    "TIPWASTE": ("tip waste funnel + gate + VC-2 -> vertical chute -> crate in the drawer", 0.0, "shallow", False, True),
    "SHAKER": ("heater-shaker on SK-1, sunk 40 mm on a tray, set 8 mm outward (edge place)", 90.0, None, True, True),
    "CTB1": ("cold tube block CTB-1 (48 x 1.5/2 mL) on a thermal module (module zone with CTB-2)", 82.0, "deep_single_only", False, True),
    "CTB2": ("cold tube block CTB-2 (6 x 5, 4 x 15, 3 x 50 mL), recessed 60", 100.0, "deep_single_only", False, True),
    "ATR1": ("ambient tube rack ATR-1 (6 x 50 + 6 x 15 mL), recessed 60", 100.0, "deep_single_only", False, True),
    "ATR2": ("ambient tube rack ATR-2 (24 x 1.5/2 + 10 x 5 mL), recessed 20", 50.0, "deep_single_only", False, True),
    "ATR3": ("low ambient tube rack ATR-3 (24 x 1.5/2 mL), on a plain insert, not recessed (nothing under the deck)", 48.0, "deep_single_only", False, False),
    "RES1": ("reservoir place: 4-trough 73 mL (water, buffers; ethanol trough under a slotted cover), plain insert", 44.0, "deep", False, False),
    "RES2": ("reservoir place: 12-column reservoir (buffer A, bead clean-up, binding buffer; ethanol columns under the slotted cover ECV-12), plain insert", 44.0, "deep", False, False),
    "HAZ": ("hazardous-liquid trough, lidded, ambient (a thermal module fits under it if 37 C is wanted)", 44.0, "deep", False, False),
    "MB1": ("magnet MB-1", 16.0, "deep", True, True),
    "MAG2": ("magnet 2: low-elution, post-PCR", 16.0, "deep", True, False),
    "CPB1": ("cold plate CPB-1 on a thermal module", 30.0, "deep", True, True),
    "STRIP": ("cold strip carrier on a thermal module", 23.5, "deep", True, True),
    "LW": ("liquid waste LW-1 (two streams) -> closed canisters in the drawer", 45.0, "centre", False, True),
    "STACK": ("plate stack x4 (gripper only)", 51.0, None, True, False),
    "STACK2": ("plate stack x4, second (gripper only)", 51.0, None, True, False),
    "LIDPARK2": ("lid park 2: shaker, sealing and 3 rescue lids", 47.5, None, True, False),
    "TPK": ("tip parking TPK-1 (pre-PCR tips; enables the waste-tip-per-well rules)", 81.6, "shallow", False, False),
    "TPKS": ("tip parking TPK-S: random-access p300 comb for the M300, 3 groups (spot, entry, spot) = 6 eight-channel sets (48 tips), on a "
             "half insert in the left part of B0 beside the tip waste (pre-PCR tips; enables the waste-tip-per-well rules)", 97.6, "shallow", False, False),
    "WN1": ("weighing nest WN-1", 44.0, "deep", True, True),
    "BAL": ("balance (under-deck bay, draught shield)", 60.0, "shallow", False, True),
    "P10": ("tip frame p10 (gripper-refilled)", 50.0, "shallow", True, False),
    "P200": ("tip frame p200, recessed 13 so it is 51 mm above the seat", 51.0, "shallow", True, True),
    "P300": ("tip frame p300, recessed 15 so it is 57 mm above the seat", 57.0, "shallow", True, True),
    "P1000R": ("p1000 tip rack (person-loaded), recessed 75", 72.0, "shallow", False, True),
    "EMPTY": ("empty plain place (standard bolt pattern, plain insert, cable route)", 0.0, None, False, False),
    # MVP build variant (cad/lh1/machine_family/mvp.py): a 96-well SBS plate placed by hand on a plain insert; used by no size layout
    "PLATE96": ("96-well plate (SBS, 14.4 mm) on a plain insert, placed by hand (MVP)", 14.4, "deep", False, False),
}
FRAMES = ("P10", "P200", "P300")
SINGLE_DEEP = ("deep", "deep_single_only")

RULES = {
    "R1": "behind (+Y) a place worked deep: height <= 51 mm; behind shallow/centre work <= 72 mm (rev K stack cap from the head's rear parts)",
    "R2": "beside (X) a place worked deep by an eight-channel tool: height <= 64 mm (K19); TC-1's own lid strip excepted (CAD-checked)",
    "R3": "tip frames (p10, p200, p300) in the back row G only (bare-nozzle single-tip pick-up needs nothing behind)",
    "N1": "beside a tip frame: height <= 57 mm (the eight-channel cradle cheeks pass over the next place in X)",
    "N2": "in front (-Y) of a place worked deep by single channels: height <= 100 mm (the cradle straps reach about 25 mm in front)",
    "N3": "tip waste in column 1 only, with column 0 of that row empty (VC-2 camera and bracket stand 34-38 mm outside the place on -X up to z 102)",
    "N4": "TC-1 at column 1 with its lid park directly behind; column 2 of both its rows without under-deck parts (body 165 mm wide); "
          "column 0 of both rows unusable (the 140 mm lid guide strip stands there)",
    "N5": "shaker at the right edge (column 4), set 8 mm outward with the shelf cut to x 350; its column-3 neighbour without under-deck parts",
    "N6": "column 0 is pipette-only: nothing there that needs the gripper",
    "N7": "tip waste in rows F, A or B (over the crate in the drawer; those rows never move)",
    "N8": "the cold tube blocks CTB-1 and CTB-2 side by side in one row (one module zone, one connector)",
    "N9": "hazardous liquid not beside or in front of/behind a buffer reservoir (deck_k Y8: splash and carry-over)",
    "N10": "TC-1 in row F or A only (a row inserted in front of row G never separates it from its lid park)",
    "N11": "the narrow tip parking TPK-S only at column 0 of the tip-waste row (the half of the place VC-2 leaves free; its comb ends 10.7 mm "
           "from the VC-2 bracket, CAD-checked); it is the one station N3 admits there",
}


def _h(k):
    return 0.0 if k in (None, "EMPTY", "OFF") else STATIONS[k][1]


def _work(k):
    return None if k in (None, "EMPTY", "OFF") else STATIONS[k][2]


def check(rows, layout, final=True):
    """(the TC-1 lid strip marker is "TCSTRIP": it used to be "STRIP", the same name as the cold strip carrier station)
    every rule on a layout {place: station key}; place = row letter + column digit (column 0 = the pipette-only column).
    Returns a list of violation strings (empty = pass).  final=False treats unlisted places as unknown (for a search)."""
    bad = []
    tc_rows = []
    for n, k in layout.items():
        if k == "TC1":
            i = rows.index(n[0])
            tc_rows = [n[0]] + ([rows[i + 1]] if i + 1 < len(rows) else [])

    def get(r, c):
        if r is None or c < 0 or c > 4:
            return "OFF"
        if c == 0 and r in tc_rows:
            return "TCSTRIP"
        n = f"{r}{c}"
        if n in layout:
            return layout[n]
        return "EMPTY" if final else None

    def h(k):
        return {"TCSTRIP": 140.0, "OFF": 0.0}.get(k, _h(k)) if k is not None else None

    for n, k in layout.items():
        if k in (None, "EMPTY"):
            continue
        r, c = n[0], int(n[1])
        i = rows.index(r)
        rb = rows[i + 1] if i + 1 < len(rows) else None
        rf = rows[i - 1] if i > 0 else None
        w = _work(k)
        kb, kf, kl, kr = get(rb, c), get(rf, c), get(r, c - 1), get(r, c + 1)
        if w and rb and kb is not None:
            lim = 51.0 if w in SINGLE_DEEP else 72.0
            if h(kb) > lim:
                bad.append(f"R1 {n} {k}: behind {rb}{c} {kb} {h(kb):g} > {lim:g}")
        if w == "deep":
            for kk, cc in ((kl, c - 1), (kr, c + 1)):
                if kk not in (None, "OFF", "TCSTRIP") and h(kk) > 64.0:
                    bad.append(f"R2 {n} {k}: beside {r}{cc} {kk} {h(kk):g} > 64")
        if k in FRAMES:
            if r != rows[-1]:
                bad.append(f"R3 {n}: tip frames in the back row")
            for kk, cc in ((kl, c - 1), (kr, c + 1)):
                if kk not in (None, "OFF") and h(kk) > 57.0:
                    bad.append(f"N1 {n} {k}: beside {r}{cc} {kk} {h(kk):g} > 57")
        if w in SINGLE_DEEP and kf not in (None, "OFF") and h(kf) > 100.0:
            bad.append(f"N2 {n} {k}: in front {rf}{c} {kf} {h(kf):g} > 100")
        if k == "TIPWASTE":
            if c != 1:
                bad.append(f"N3 {n}: tip waste in column 1 only")
            elif get(r, 0) not in ("EMPTY", None, "TPKS"):
                bad.append(f"N3 {n}: column 0 of row {r} must be clear for VC-2 ({get(r, 0)})")
            if r not in ("F", "A", "B"):
                bad.append(f"N7 {n}: tip waste over the crate (rows F-B)")
        if k == "TC1":
            if c != 1 or r not in ("F", "A"):
                bad.append(f"N4/N10 {n}: TC-1 at F1 or A1")
            if rb and get(rb, c) not in ("TC1LID", None):
                bad.append(f"N4 {n}: lid park directly behind")
            for rr in tc_rows:
                k2 = get(rr, 2)
                if k2 not in (None, "OFF") and k2 in STATIONS and STATIONS[k2][4]:
                    bad.append(f"N4 {rr}2 {k2}: under-deck parts where the TC-1 body is")
                if f"{rr}0" in layout and layout[f"{rr}0"] not in ("EMPTY",):
                    bad.append(f"N4 {rr}0: the TC-1 lid strip stands there")
        if k == "SHAKER":
            if c != 4:
                bad.append(f"N5 {n}: shaker at the right edge")
            if kl not in (None, "OFF") and kl in STATIONS and STATIONS[kl][4]:
                bad.append(f"N5 {n}: the column-3 neighbour {kl} has under-deck parts")
        if k == "TPKS" and (c != 0 or get(r, 1) != "TIPWASTE"):
            bad.append(f"N11 {n}: TPK-S only at column 0 beside the tip waste")
        if c == 0 and STATIONS[k][3]:
            bad.append(f"N6 {n} {k}: column 0 is pipette-only")
        if k == "CTB1":
            if get(r, c - 1) not in ("CTB2", None) and get(r, c + 1) not in ("CTB2", None):
                bad.append(f"N8 {n}: CTB-1 beside CTB-2")
        if k == "HAZ":
            for rr, cc in ((r, c - 1), (r, c + 1), (rf, c), (rb, c)):
                if rr and get(rr, cc) in ("RES1", "RES2"):
                    bad.append(f"N9 {n}: hazardous liquid beside a reservoir at {rr}{cc}")
    return bad


def report(rows, layout):
    v = check(rows, layout)
    n_places = len([k for k in layout.values() if k not in (None, "EMPTY")])
    return {"rules": RULES, "places_checked": n_places, "violations": v, "pass": not v,
            "note": "rule check only (sufficient clearances from the CAD runs); the CAD interference run is the real test"}


# revision L as designed, translated to station keys (heights as machine_l pass 2 set them) for information
REV_L_ROWS = ["F", "A", "B", "C", "D", "G"]
REV_L_LAYOUT = {"F1": "ATR1", "F2": "BAL", "F3": "CTB1", "F4": "CTB2", "A1": "TIPWASTE", "A2": "ATR2", "A3": "HAZ", "A4": "LW",
                "B1": "WN1", "B2": "MB1", "B3": "CPB1", "B4": "MAG2", "C1": "TC1", "C2": "STACK", "C3": "STRIP", "C4": "STACK",
                "D1": "TC1LID", "D2": "TPK", "D3": "LIDPARK2", "D4": "SHAKER", "G1": "P10", "G2": "P200", "G3": "P300", "G4": "P1000R"}
