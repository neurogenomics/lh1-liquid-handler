"""Audit the built-in G-code commands a Klipper config leaves reachable, against the user's classification table.

Each row of the table classifies one or more built-in commands, per config column (e.g. ``machine``, ``rig``):

* ``refused``  - the config defines ``[gcode_macro CMD]`` that calls ``action_raise_error`` and never calls the
  renamed built-in (its ``rename_existing`` name) at the start of a line;
* ``wrapped``  - the config defines ``[gcode_macro CMD]`` with ``rename_existing`` (a guard that checks, then calls the
  base).  ``G28`` may instead be wrapped by a ``[homing_override]`` section (Klipper's documented mechanism);
* ``left``     - deliberately left as Klipper provides it (the row's ``why`` should say why);
* ``n/a``      - not registered with this config.

Findings (each has ``code``, ``config``, ``msg``):

* COMMAND_NOT_OVERRIDDEN  - a refused/wrapped command has no macro, or the macro does not refuse / has no
  rename_existing (``reason`` says which);
* COMMAND_AUDIT_STALE     - the config has a section listed in ``unaudited_sections`` (it registers commands the table
  does not cover);
* COMMAND_UNCLASSIFIED    - a command the table's ``registered_always`` / ``registered_by_section`` says is registered
  with this config has no row, no value for this config, or the value ``n/a``;
* COMMAND_AUDIT_BAD_ROW   - a row without ``cmd`` or with a value other than the four above;
* CONFIG_INCLUDE_NOT_FOLLOWED - the config has an ``[include]`` section (this tool does not follow includes, so the
  audit would be incomplete).

Which commands Klipper registers is NOT built in: it comes from the table (``registered_always``,
``registered_by_section``), so the reachability check is only as complete as that list.
"""
from __future__ import annotations

import re
from pathlib import Path

import yaml

from .config import KlipperConfig

VALUES = ("refused", "wrapped", "left", "n/a")


class AuditTableError(ValueError):
    pass


def _dig(data, key: str):
    for k in key.split("/") if key else []:
        data = data.get(k) if isinstance(data, dict) else None
    return data


def load_table(path) -> dict:
    """Load an audit table file.  ``commands`` may be inline or read from another YAML file with
    ``commands_from: {file: other.yaml, key: dotted/path}`` (relative to this file)."""
    path = Path(path)
    t = yaml.safe_load(path.read_text()) or {}
    if not isinstance(t, dict):
        raise AuditTableError(f"{path}: must be a mapping")
    t = dict(t)
    if "commands_from" in t:
        cf = t["commands_from"]
        t["commands"] = _dig(yaml.safe_load((path.parent / cf["file"]).read_text()), cf.get("key", ""))
    t["_base_dir"] = path.parent
    return t


def config_paths(table: dict) -> dict:
    base = table.get("_base_dir", Path("."))
    return {k: Path(base) / v for k, v in (table.get("configs") or {}).items()}


def _check_override(cfg: KlipperConfig, c: str, want: str, wrapped_by: dict) -> tuple[bool, str]:
    sec = f"gcode_macro {c}"
    alt = wrapped_by.get(c)
    if alt and cfg.has_section(alt):
        return True, f"[{alt}]"
    if not cfg.has_section(sec):
        return False, f"no [{sec}]" + (f" and no [{alt}]" if alt else "")
    if want == "refused":
        g = cfg.get(sec, "gcode", fallback="")
        base = cfg.rename_existing(sec)
        if "action_raise_error" not in g:
            return False, f"[{sec}] never calls action_raise_error"
        if base and re.search(rf"(?m)^\s*{re.escape(base)}\b", g):
            return False, f"[{sec}] calls its renamed built-in {base}"
        return True, f"[{sec}] refuses"
    if not cfg.has_option(sec, "rename_existing"):
        return False, f"[{sec}] has no rename_existing (it replaces the built-in instead of wrapping it)"
    return True, f"[{sec}] wraps {cfg.rename_existing(sec)}"


def audit(table: dict, configs: dict) -> dict:
    """``configs``: column name -> KlipperConfig / RawConfigParser / path / None (None = that config is absent and its
    column is skipped).  Returns {"report": {column: {cmd: value or "MISSING"}}, "findings": [...], "passed": bool}."""
    name = table.get("name") or "the audit table"
    rows = table.get("commands") or []
    wrapped_by = {"G28": "homing_override"}
    wrapped_by.update(table.get("wrapped_by_section") or {})
    cfgs = {col: KlipperConfig.wrap(c) for col, c in configs.items()}
    findings = []
    report = {col: {} for col in cfgs}

    def add(code, col, msg, **kw):
        findings.append({"code": code, "config": col, "msg": msg, **kw})

    # --- the table itself
    for i, row in enumerate(rows):
        if not isinstance(row, dict) or not row.get("cmd"):
            add("COMMAND_AUDIT_BAD_ROW", None, f"{name} row {i}: no cmd")
            continue
        for col in cfgs:
            v = row.get(col)
            if v is not None and v not in VALUES:
                add("COMMAND_AUDIT_BAD_ROW", col, f"{name} row {i} {row['cmd']}: {col} value {v!r} not one of {VALUES}")
    rows = [r for r in rows if isinstance(r, dict) and r.get("cmd")]
    # --- refused / wrapped: the override exists and does what the table says
    for row in rows:
        for col, cfg in cfgs.items():
            want = row.get(col)
            if cfg is None or want not in ("refused", "wrapped"):
                continue
            for c in row["cmd"]:
                ok, reason = _check_override(cfg, c, want, wrapped_by)
                report[col][c] = want if ok else "MISSING"
                if not ok:
                    add("COMMAND_NOT_OVERRIDDEN", col,
                        f"{col}: {c} is '{want}' in {name} but its override is missing or does not refuse",
                        command=c, reason=reason)
    # --- sections that register commands nobody has audited
    unaudited = set(table.get("unaudited_sections") or [])
    for col, cfg in cfgs.items():
        if cfg is None:
            continue
        for sec in cfg.sections():
            if cfg.section_type(sec) in unaudited:
                add("COMMAND_AUDIT_STALE", col, f"{col}: [{sec}] registers commands not covered by {name}", section=sec)
    # --- includes are not followed
    for col, cfg in cfgs.items():
        if cfg is not None:
            for sec in cfg.includes():
                add("CONFIG_INCLUDE_NOT_FOLLOWED", col, f"{col}: [{sec}] is not followed; the audit does not cover that file",
                    section=sec)
    # --- reachable but unclassified
    cls = {}
    for row in rows:
        for c in row["cmd"]:
            cls.setdefault(c, row)
    always = list(table.get("registered_always") or [])
    by_sec = table.get("registered_by_section") or {}
    for col, cfg in cfgs.items():
        if cfg is None:
            continue
        reach = {c: "always" for c in always}
        for st in cfg.section_types():
            for c in by_sec.get(st) or []:
                reach.setdefault(c, f"[{st}]")
        for c, why in reach.items():
            row = cls.get(c)
            v = row.get(col) if row else None
            if row is None:
                add("COMMAND_UNCLASSIFIED", col, f"{col}: {c} is registered ({why}) but has no row in {name}", command=c)
            elif v is None:
                add("COMMAND_UNCLASSIFIED", col, f"{col}: {c} is registered ({why}) but its row has no '{col}' value", command=c)
            elif v == "n/a":
                add("COMMAND_UNCLASSIFIED", col, f"{col}: {c} is registered ({why}) but {name} says n/a", command=c)
    return {"report": report, "findings": findings, "passed": not findings}


def audit_file(table_path, config_overrides: dict | None = None) -> dict:
    table = load_table(table_path)
    paths = config_paths(table)
    for k, v in (config_overrides or {}).items():
        paths[k] = Path(v)
    if not paths:
        raise AuditTableError("no configs: give 'configs' in the table or --config NAME=PATH")
    configs = {k: (KlipperConfig.load(p) if p.exists() else None) for k, p in paths.items()}
    res = audit(table, configs)
    if not table.get("commands"):
        res["findings"].insert(0, {"code": "COMMAND_AUDIT_MISSING", "config": None, "msg": f"{table_path}: no command rows"})
    missing = [k for k, c in configs.items() if c is None]
    for k in missing:
        res["findings"].append({"code": "CONFIG_MISSING", "config": k, "msg": f"{k}: {paths[k]} not found"})
    res["passed"] = not res["findings"]
    return res
