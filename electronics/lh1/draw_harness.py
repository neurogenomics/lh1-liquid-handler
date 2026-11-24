"""Draw harness.svg, the LH-1 electrical block diagram, from wiring.csv and system.yaml (so it cannot drift).

Run: .venv/bin/python electronics/lh1/draw_harness.py  -> electronics/lh1/harness.svg
Thin caller of harnessgen.draw_svg (packages/harnessgen) with the LH-1 style in harness_style.yaml: devices are boxes
grouped by location (frame and deck on the left, control bay in the centre, gantry and head on the right); every
wiring.csv row is one line labelled with its cable_id, coloured by rail/signal class; safety-chain cables are drawn
dashed and heavier, and an inset shows the safety chain.  Cables between devices of the same group are drawn as loops
on the group's outer side.
"""
from pathlib import Path

import yaml
from harnessgen import draw_svg, read_rows

HERE = Path(__file__).resolve().parent
SYS = yaml.safe_load((HERE / "system.yaml").read_text())
ROWS = read_rows(HERE / "wiring.csv")


def main():
    size = SYS["family"]["build_size"]
    devs = {k: v for k, v in SYS["devices"].items() if size in v.get("sizes", ["S", "M", "L"])}
    svg, info = draw_svg(ROWS, devs, yaml.safe_load((HERE / "harness_style.yaml").read_text()))
    (HERE / "harness.svg").write_text(svg)
    print(f"harness.svg: {info['devices']} devices, {info['cables']} cables, {info['width']} x {info['height']}")


if __name__ == "__main__":
    main()
