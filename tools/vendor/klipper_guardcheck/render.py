"""Render Klipper gcode_macro templates against a mocked ``printer`` and check what they do.

Rendering follows klippy/extras/gcode_macro.py: ``jinja2.Environment('{%', '%}', '{', '}')``; the context is the macro's
own variables, ``printer`` (the mocked status), ``params`` (upper-case keys, string values), ``rawparams`` and the
``action_*`` functions.  ``action_raise_error`` raises :class:`Refused`.  Called macros are NOT expanded: each macro is
rendered on its own, and the output is the text it would send to the G-code queue.

Case-file schema: see README.md.txt.
"""
from __future__ import annotations

import copy
import re
from collections import namedtuple
from pathlib import Path

import jinja2
import yaml

from .config import KlipperConfig

Coord = namedtuple("Coord", ("x", "y", "z", "e"), defaults=(0.0, 0.0, 0.0, 0.0))
# status fields Klipper reports as a Coord (Status_Reference: toolhead, gcode_move, motion_report)
COORD_FIELDS = {("toolhead", "position"), ("gcode_move", "position"), ("gcode_move", "gcode_position"),
                ("gcode_move", "homing_origin"), ("motion_report", "live_position")}
_EXPR = re.compile(r"\$\{(.+?)\}")
_EXPR_ENV = jinja2.Environment(undefined=jinja2.StrictUndefined)


class Refused(Exception):
    """action_raise_error was called."""


class CaseFileError(ValueError):
    pass


class Macro:
    def __init__(self, cfg: KlipperConfig, name: str):
        sec = cfg.macro_section(name)
        if not cfg.has_section(sec):
            raise CaseFileError(f"no [{sec}] in {cfg.path or 'the config'}")
        self.name = name
        self.src = cfg.get(sec, "gcode")
        self.variables = cfg.variables(sec)
        self.tpl = jinja2.Environment("{%", "%}", "{", "}").from_string(self.src)

    def render(self, printer: dict, params: dict) -> str:
        def raise_error(msg):
            raise Refused(msg)
        ctx = dict(self.variables)
        ctx.update(printer=printer, params={k.upper(): str(v) for k, v in params.items()},
                   rawparams=" ".join(f"{k}={v}" for k, v in params.items()),
                   action_raise_error=raise_error, action_respond_info=lambda m: "", action_emergency_stop=lambda m="": "",
                   action_call_remote_method=lambda *a, **k: "")
        return self.tpl.render(ctx)


# ------------------------------------------------------------------ printer status
def _deep_merge(dst: dict, src: dict) -> None:
    for k, v in src.items():
        if isinstance(v, dict) and isinstance(dst.get(k), dict):
            _deep_merge(dst[k], v)
        else:
            dst[k] = copy.deepcopy(v)


def _unset(status: dict, path: str) -> None:
    keys = path.split("/")
    d = status
    for k in keys[:-1]:
        d = d[k]
    d.pop(keys[-1], None)


def _coordify(status: dict) -> dict:
    """Return a copy with Coord fields turned into Coord objects (attribute and index access, as in Klipper)."""
    out = dict(status)
    for obj, field in COORD_FIELDS:
        if isinstance(out.get(obj), dict) and field in out[obj]:
            v = out[obj][field]
            o = dict(out[obj])
            o[field] = Coord(**v) if isinstance(v, dict) else Coord(*v)
            out[obj] = o
    return out


def build_status(cfg: KlipperConfig, spec: dict) -> dict:
    """Base printer status from a ``printers.<name>`` entry: literal ``status`` plus ``macro_variables`` copied from
    the config (gcode_macro NAME -> its variable_* values, then the given overrides)."""
    status = copy.deepcopy(spec.get("status") or {})
    for name, overrides in (spec.get("macro_variables") or {}).items():
        sec = cfg.macro_section(name)
        if not cfg.has_section(sec):
            raise CaseFileError(f"macro_variables: no [{sec}] in {cfg.path or 'the config'}")
        v = cfg.variables(sec)
        v.update(overrides or {})
        status[sec] = v
    return status


# ------------------------------------------------------------------ ${...} expressions
def _eval(expr: str, ctx: dict):
    return _EXPR_ENV.compile_expression(expr.strip(), undefined_to_none=False)(**ctx)


def substitute(value, ctx: dict):
    """Replace ${expr} in strings (recursively in lists/dicts).  A string that is exactly one ${expr} keeps the
    expression's type; otherwise each ${expr} is replaced by str(value)."""
    if isinstance(value, str):
        m = _EXPR.fullmatch(value)
        if m:
            return _eval(m.group(1), ctx)
        return _EXPR.sub(lambda mm: str(_eval(mm.group(1), ctx)), value) if "${" in value else value
    if isinstance(value, list):
        return [substitute(v, ctx) for v in value]
    if isinstance(value, dict):
        return {substitute(k, ctx) if isinstance(k, str) else k: substitute(v, ctx) for k, v in value.items()}
    return value


# ------------------------------------------------------------------ case file
CASE_KEYS = {"macro", "printer", "params", "state", "set", "unset", "expect", "emits", "emits_regex", "message", "sets",
             "note"}


def _expand(items, defaults: dict, loopvars: dict, where: str):
    for i, item in enumerate(items or []):
        if not isinstance(item, dict):
            raise CaseFileError(f"{where}[{i}]: not a mapping")
        if "cases" in item or "foreach" in item:
            unknown = set(item) - {"cases", "foreach"} - CASE_KEYS
            if unknown:
                raise CaseFileError(f"{where}[{i}]: unknown keys {sorted(unknown)}")
            d = dict(defaults)
            d.update({k: v for k, v in item.items() if k not in ("cases", "foreach")})
            fe = item.get("foreach")
            if fe:
                if not isinstance(fe, dict) or len(fe) != 1:
                    raise CaseFileError(f"{where}[{i}]: foreach takes one variable: {{name: [values]}}")
                (var, values), = fe.items()
                for v in values:
                    yield from _expand(item.get("cases"), d, {**loopvars, var: v}, f"{where}[{i}]")
            else:
                yield from _expand(item.get("cases"), d, loopvars, f"{where}[{i}]")
        else:
            unknown = set(item) - CASE_KEYS
            if unknown:
                raise CaseFileError(f"{where}[{i}]: unknown keys {sorted(unknown)}")
            c = dict(defaults)
            c.update(item)
            yield c, dict(loopvars), f"{where}[{i}]"


class CaseFile:
    """A parsed case file.  ``configs`` maps config names to paths; ``config_overrides`` (name -> path) replaces them."""

    def __init__(self, data: dict, base_dir: Path, config_overrides: dict | None = None):
        if not isinstance(data, dict):
            raise CaseFileError("case file must be a mapping")
        unknown = set(data) - {"configs", "printers", "cases"}
        if unknown:
            raise CaseFileError(f"unknown top-level keys {sorted(unknown)}")
        self.base_dir = base_dir
        paths = {k: (base_dir / v) for k, v in (data.get("configs") or {}).items()}
        for k, v in (config_overrides or {}).items():
            paths[k] = Path(v)
        if not paths:
            raise CaseFileError("no configs")
        self.configs = {k: KlipperConfig.load(p) for k, p in paths.items()}
        self.printers = {}
        for name, spec in (data.get("printers") or {}).items():
            spec = spec or {}
            cname = spec.get("config") or (next(iter(self.configs)) if len(self.configs) == 1 else None)
            if cname not in self.configs:
                raise CaseFileError(f"printers.{name}: config {cname!r} is not one of {sorted(self.configs)}")
            cfg = self.configs[cname]
            status = build_status(cfg, spec)
            lets = {}
            for lk, expr in (spec.get("let") or {}).items():
                lets[lk] = _eval(expr, {"printer": status, **lets})
            self.printers[name] = {"config": cfg, "status": status, "let": lets}
        if not self.printers:
            raise CaseFileError("no printers")
        self.cases = list(_expand(data.get("cases"), {}, {}, "cases"))

    @classmethod
    def load(cls, path, config_overrides: dict | None = None) -> "CaseFile":
        path = Path(path)
        return cls(yaml.safe_load(path.read_text()), path.parent, config_overrides)


def run_case(cf: CaseFile, case: dict, loopvars: dict, where: str) -> dict:
    pname = case.get("printer") or (next(iter(cf.printers)) if len(cf.printers) == 1 else None)
    if pname not in cf.printers:
        raise CaseFileError(f"{where}: printer {pname!r} is not one of {sorted(cf.printers)}")
    P = cf.printers[pname]
    ctx = {"printer": P["status"], **P["let"], **loopvars}
    c = substitute({k: v for k, v in case.items() if k != "note"}, ctx)
    if "macro" not in c:
        raise CaseFileError(f"{where}: no macro")
    want = c.get("expect")
    if want not in ("refuse", "accept"):
        raise CaseFileError(f"{where}: expect must be 'refuse' or 'accept', got {want!r}")
    params = c.get("params") or {}
    pr = copy.deepcopy(P["status"])
    if c.get("set"):
        _deep_merge(pr, c["set"])
    for path in c.get("unset") or []:
        _unset(pr, path)
    m = Macro(P["config"], c["macro"])
    error = None
    try:
        out = m.render(_coordify(pr), params)
        refused, msg = False, ""
    except Refused as e:
        out, refused, msg = "", True, str(e)
    except jinja2.TemplateError as e:
        # Klipper would abort the command too, but by accident: a guard must refuse on purpose, so this always fails
        out, refused, msg, error = "", False, f"{type(e).__name__}: {e}", True
    problems = []
    if error:
        problems.append(f"template error ({msg}); want {want}")
    elif refused != (want == "refuse"):
        problems.append(f"want {want}, got {'refuse' if refused else 'accept'}")
    for s in c.get("emits") or []:
        if s not in out:
            problems.append(f"output lacks {s!r}")
    for rx in c.get("emits_regex") or []:
        if not re.search(rx, out, re.M):
            problems.append(f"output does not match /{rx}/")
    if c.get("message") is not None and str(c["message"]) not in msg:
        problems.append(f"refusal message lacks {c['message']!r}")
    for sv in c.get("sets") or []:
        rx = (rf"(?m)^\s*SET_GCODE_VARIABLE(?:_BASE)?\b(?=.*\bMACRO={re.escape(str(sv['macro']))}(?:\s|$))"
              rf"(?=.*\bVARIABLE={re.escape(str(sv['variable']))}(?:\s|$))(?=.*\bVALUE={re.escape(str(sv['value']))}(?:\s|$))")
        if not re.search(rx, out):
            problems.append(f"does not set {sv['macro']}.{sv['variable']} = {sv['value']}")
    rec = {"macro": c["macro"], "params": params, "state": c.get("state") or "good", "want": want,
           "got": "error" if error else ("refuse" if refused else "accept"), "msg": msg[:160],
           "out": " | ".join(l.strip() for l in out.splitlines() if l.strip())[:240], "ok": not problems}
    if problems:
        rec["problems"] = problems
        rec["where"] = where
    return rec


def run_cases(cf: CaseFile) -> dict:
    cases = [run_case(cf, c, lv, w) for c, lv, w in cf.cases]
    failed = [c for c in cases if not c["ok"]]
    return {"jinja2": jinja2.__version__, "n": len(cases), "failed": len(failed), "cases": cases}


def run_case_file(path, config_overrides: dict | None = None) -> dict:
    return run_cases(CaseFile.load(path, config_overrides))
