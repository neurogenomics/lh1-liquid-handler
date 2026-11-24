#!/usr/bin/env python3
"""Fail if any forbidden term appears anywhere in this repository.

Run from the repository root:   python tools/check_no_private_content.py [ROOT]

What it scans
  * every file path (folder and file names);
  * text files of any kind (Python, JSON, YAML, CSV, SVG, Markdown, Klipper .cfg, DXF, HTML, ...), line by line;
  * STEP files: the header and every quoted string (part, product and assembly names);
  * PNG files: their text chunks (tEXt, zTXt, iTXt); JPEG files: their comment and APPn (EXIF, XMP) segments;
  * STL files: the header (binary) or the solid names (ASCII).
  Other binary files (.bin, .zip, images other than PNG) are listed as "not scanned" and fail the check, so that
  nothing slips in unexamined; add their extension to SKIP_OK below only for generated mesh data.

How terms are held
  The forbidden list itself is private.  tools/forbidden_hashes.json holds only salted SHA-256 hashes of the terms,
  so the checker can find them without the repository naming them.  Text is lower-cased and split into words on
  anything that is not a-z or 0-9; a term matches as a whole word ("token"), anywhere inside a word ("contains")
  or as consecutive words ("ngram").  Two patterns are kept in the clear because they are not private: words of the
  form sk<two or more digits> (sample identifiers) and absolute home-directory paths.

Exit status 0 = clean; 1 = at least one hit (each printed as path:line: word).
"""
from __future__ import annotations

import hashlib
import json
import re
import struct
import sys
import zlib
from pathlib import Path

HERE = Path(__file__).resolve().parent
SKIP_DIRS = {".git", "__pycache__", ".venv", "venv", ".pytest_cache", "node_modules", ".mypy_cache", ".ruff_cache"}
SKIP_OK = {".bin"}                    # generated mesh data for the viewer; checked by name only
WORD = re.compile(r"[a-z0-9]+")
SAMPLE_ID = re.compile(r"^sk\d{2,}$")
HOME_PATH = re.compile(r"(/Users/[A-Za-z]|/home/[a-z]|[A-Za-z]:\\Users\\)")
TEXT_EXT = None                       # every file not handled below is tried as text


def load_terms(path: Path):
    d = json.loads(path.read_text())
    salt = d["salt"].encode()
    tok = set(d["token"])
    con = {}
    for L, hs in d["contains"].items():
        con[int(L)] = set(hs)
    ngr = [tuple(x) for x in d["ngram"]]
    return salt, tok, con, ngr


class Scanner:
    def __init__(self, hashes_path: Path):
        self.salt, self.tok, self.con, self.ngr = load_terms(hashes_path)
        self.first = {}
        for g in self.ngr:
            self.first.setdefault(g[0], []).append(g)
        self.cache = {}

    def h(self, s: str) -> str:
        return hashlib.sha256(self.salt + s.encode()).hexdigest()[:20]

    def word(self, w: str):
        """(hit, word_hash) for one lower-case word"""
        c = self.cache.get(w)
        if c is not None:
            return c
        wh = self.h(w)
        hit = wh in self.tok or bool(SAMPLE_ID.match(w))
        if not hit and not w.isdigit():
            for L, hs in self.con.items():
                if len(w) >= L and any(self.h(w[i:i + L]) in hs for i in range(len(w) - L + 1)):
                    hit = True
                    break
        c = (hit, wh)
        if len(self.cache) < 2_000_000:
            self.cache[w] = c
        return c

    def line(self, text: str):
        hits = []
        if HOME_PATH.search(text):
            hits.append(HOME_PATH.search(text).group(0) + "...")
        words = WORD.findall(text.lower())
        if not words:
            return hits
        hs = []
        for w in words:
            hit, wh = self.word(w)
            hs.append(wh)
            if hit:
                hits.append(w)
        for i, wh in enumerate(hs):
            for g in self.first.get(wh, ()):
                if tuple(hs[i:i + len(g)]) == g:
                    hits.append(" ".join(words[i:i + len(g)]))
        return hits


def png_text(b: bytes) -> str:
    out, i = [], 8
    while i + 8 <= len(b):
        n, typ = struct.unpack(">I4s", b[i:i + 8])
        data = b[i + 8:i + 8 + n]
        if typ == b"tEXt":
            out.append(data.decode("latin-1"))
        elif typ == b"zTXt":
            k, _, rest = data.partition(b"\0")
            try:
                out.append(k.decode("latin-1") + " " + zlib.decompress(rest[1:]).decode("latin-1"))
            except Exception:
                out.append(k.decode("latin-1"))
        elif typ == b"iTXt":
            out.append(data.replace(b"\0", b" ").decode("utf-8", "replace"))
        if typ == b"IEND":
            break
        i += 12 + n
    return "\n".join(out)


def jpeg_text(b: bytes) -> str:
    """comment (COM) and application (APPn: EXIF, XMP, ICC) segments, as text"""
    out, i = [], 2
    while i + 4 <= len(b) and b[i] == 0xFF:
        m = b[i + 1]
        if m in (0xD9, 0xDA):                       # end of image / start of scan: no metadata after this
            break
        n = struct.unpack(">H", b[i + 2:i + 4])[0]
        if m == 0xFE or 0xE0 <= m <= 0xEF:
            out.append(b[i + 4:i + 2 + n].replace(b"\0", b" ").decode("latin-1"))
        i += 2 + n
    return "\n".join(out)


def step_text(b: bytes) -> str:
    t = b.decode("latin-1")
    head, _, data = t.partition("DATA;")
    lines = head.splitlines()
    # one line per quoted string, prefixed with its line number in the file
    base, last, ln = head.count("\n") + 1, 0, 0
    for m in re.finditer(r"'((?:[^']|'')*)'", data):
        s = m.group(1)
        if s and re.search(r"[A-Za-z]", s):
            ln += data.count("\n", last, m.start())
            last = m.start()
            lines.append(f"\x00{base + ln}\x00{s}")
    return "\n".join(lines)


def file_lines(p: Path):
    """yield (line_no, text) for the scannable content of one file; None for an unscanned binary"""
    ext = p.suffix.lower()
    b = p.read_bytes()
    if ext == ".png":
        return list(enumerate(png_text(b).splitlines(), 1))
    if ext in (".jpg", ".jpeg"):
        return list(enumerate(jpeg_text(b).splitlines(), 1))
    if ext in (".step", ".stp"):
        out = []
        for i, ln in enumerate(step_text(b).splitlines(), 1):
            if ln.startswith("\x00"):
                _, n, s = ln.split("\x00", 2)
                out.append((int(n), s))
            else:
                out.append((i, ln))
        return out
    if ext == ".stl":
        if b[:5].lower() == b"solid" and b"facet" in b[:2000]:
            return [(i, ln) for i, ln in enumerate(b.decode("latin-1").splitlines(), 1) if "solid" in ln]
        return [(1, b[:80].decode("latin-1"))]
    if ext in SKIP_OK:
        return []
    if b"\0" in b[:8192]:
        return None
    return list(enumerate(b.decode("utf-8", "replace").splitlines(), 1))


def main(argv):
    root = Path(argv[1]).resolve() if len(argv) > 1 else HERE.parent
    sc = Scanner(HERE / "forbidden_hashes.json")
    hits, unscanned, n = [], [], 0
    for p in sorted(root.rglob("*")):
        rel = p.relative_to(root)
        if any(part in SKIP_DIRS for part in rel.parts):
            continue
        for w in sc.line(str(rel)):
            hits.append(f"{rel}:0 (file name): {w}")
        if not p.is_file():
            continue
        if p.resolve() == (HERE / "forbidden_hashes.json").resolve():
            continue
        n += 1
        fl = file_lines(p)
        if fl is None:
            unscanned.append(str(rel))
            continue
        for ln, text in fl:
            for w in sc.line(text):
                hits.append(f"{rel}:{ln}: {w}")
    for h in hits:
        print(h)
    for u in unscanned:
        print(f"{u}:0: binary file not scanned (remove it or add a reader)")
    print(f"check_no_private_content: {n} files scanned, {len(hits)} hits, {len(unscanned)} unscanned binaries")
    return 1 if hits or unscanned else 0


if __name__ == "__main__":
    sys.exit(main(sys.argv))
