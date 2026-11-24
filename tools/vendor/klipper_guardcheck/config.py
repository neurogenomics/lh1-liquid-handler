"""Read a Klipper config file the way Klipper's configfile does (INI with ':' or '=' delimiters, '#'/';' comments).

Differences from Klipper, on purpose:
* duplicate sections or options are an error (strict), where Klipper lets the later one win;
* ``[include ...]`` sections are NOT followed: they are kept as ordinary sections and reported by ``includes()``
  so a caller can refuse to pass an incomplete config;
* ``SAVE_CONFIG`` blocks are read as ordinary text (they come after a ``#*#`` comment marker and are ignored).
"""
from __future__ import annotations

import ast
import configparser
from pathlib import Path


def _parser() -> configparser.RawConfigParser:
    return configparser.RawConfigParser(strict=True, delimiters=(":", "="), comment_prefixes=("#", ";"),
                                        inline_comment_prefixes=("#", ";"))


class KlipperConfig:
    """A parsed Klipper config.  ``cp`` is the underlying ``configparser.RawConfigParser``."""

    def __init__(self, cp: configparser.RawConfigParser, path: Path | None = None):
        self.cp = cp
        self.path = path

    @classmethod
    def from_text(cls, text: str, path: Path | None = None) -> "KlipperConfig":
        cp = _parser()
        cp.read_string(text, source=str(path) if path else "<string>")
        return cls(cp, path)

    @classmethod
    def load(cls, path) -> "KlipperConfig":
        path = Path(path)
        return cls.from_text(path.read_text(), path)

    @classmethod
    def wrap(cls, cfg) -> "KlipperConfig | None":
        """Accept a KlipperConfig, a RawConfigParser (as an existing script may already hold), a path, or None."""
        if cfg is None or isinstance(cfg, KlipperConfig):
            return cfg
        if isinstance(cfg, configparser.RawConfigParser):
            return cls(cfg)
        return cls.load(cfg)

    # ---- sections
    def sections(self) -> list[str]:
        return self.cp.sections()

    def has_section(self, sec: str) -> bool:
        return self.cp.has_section(sec)

    def has_option(self, sec: str, opt: str) -> bool:
        return self.cp.has_option(sec, opt)

    def get(self, sec: str, opt: str, fallback=None):
        return self.cp.get(sec, opt, fallback=fallback)

    @staticmethod
    def section_type(sec: str) -> str:
        return sec.split()[0]

    def section_types(self) -> list[str]:
        out = []
        for s in self.sections():
            t = self.section_type(s)
            if t not in out:
                out.append(t)
        return out

    def includes(self) -> list[str]:
        return [s for s in self.sections() if self.section_type(s) == "include"]

    # ---- macros
    @staticmethod
    def macro_section(name: str) -> str:
        """'G0' -> 'gcode_macro G0'; a name that already has a section type ('delayed_gcode X') is used as is."""
        return name if " " in name.strip() else f"gcode_macro {name}"

    def macros(self) -> list[str]:
        return [s.split(None, 1)[1] for s in self.sections() if self.section_type(s) == "gcode_macro" and " " in s]

    def body(self, sec: str) -> str:
        """The gcode template of a section ('' if it has none)."""
        return self.cp.get(sec, "gcode") if self.cp.has_section(sec) and self.cp.has_option(sec, "gcode") else ""

    def rename_existing(self, sec: str) -> str | None:
        return self.cp.get(sec, "rename_existing", fallback=None) if self.cp.has_section(sec) else None

    def variables(self, sec: str) -> dict:
        """variable_* options of a section, evaluated with ast.literal_eval as Klipper's gcode_macro does."""
        return {k[len("variable_"):]: ast.literal_eval(v) for k, v in self.cp.items(sec) if k.startswith("variable_")}
