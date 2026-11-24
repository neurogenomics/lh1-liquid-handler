# Stage 6: cameras, tip supply towers and tip parking

Designed and simulated only. Nothing is built.

**Demonstrates:** camera re-measurement of places and bays; long runs from towers; tip re-use by parking

**Test before the next stage:** CT-0c comb entry with real tips (500 cycles); place re-measurement within 0.3 mm of CAD

## Open before it can proceed
- comb tolerance passes only at 1.04 and only with plates cut to a scanned nozzle row; no plan uses parking (verification 24 Nov 2026)

## What it adds

**cameras**: head camera + ring light (head table), VC-2 camera at B1 (1 part)

**tip_parking_TPK-S_B0**: lines 5, custom_qty 5, bought_qty 0
- IN-H_half_insert_92x94
- TPK-SL_lower_plate_p300_8mm
- TPK-SP_posts_x4
- TPK-ST_drip_tray
- TPK-SU_upper_plate_p300_6mm

**tip towers / spent stack / p1000 rack IG**: on the stage 2 shelf (tip_supply in out/S/report.json)

## Electrical additions

**cameras (VC-1, VC-2, VC-3), deck light, chain spares, tip supply towers and the tip parking comb (no electrics)**

- devices added: VC1, VC2, VC3, DECK_LIGHT, HEAD_SPARE
- cables added (6):
  - W-C-06: HUB1 to VC3, USB deck camera VC-3, USB2_AB, 2850 mm
  - W-C-30: HUB1 to VC1, USB head camera VC-1 (ring light on the camera's USB power), USB2_HF, 4850 mm
  - W-C-31: HUB1 to HEAD_SPARE, USB spare to the head (pressure-sensing channel reserve), USB2_HF, 4400 mm
  - W-C-32: XT to HEAD_SPARE, SIGNAL spare 8-core to the head, MC8_025_HF, 4650 mm
  - W-I-23: IO_CTRL to DECK_LIGHT, 24V LOGIC deck LED strip (switched), MC2_05, 2400 mm, fuse Octopus heater fuse
  - W-C-22: HUB1 to VC2, USB tip-check camera VC-2, USB2_AB, 1500 mm
- firmware: sections added output_pin deck_light; commands restored LH_TIP_PARK, LH_TIP_UNPARK, LH_TIP_PARK_SET, LH_TIP_PARK_STATUS; no longer marked absent: cameras, deck_light, tip_parking

Source: [upgrade_steps.json](../../electronics/lh1/out/mvp/upgrade_steps.json).

**Carried over:** all

## Files
- `cad/lh1/machine_family/build.py`
- `cad/lh1/machine_family/geometry.py`
- `cad/lh1/machine_family/scene.py`
- `cad/lh1/machine_family/checks.py`
- `cad/lh1/machine_family/deck_rules.py`
- `cad/lh1/machine_family/mechanisms.py`
- `cad/lh1/machine_l/build.py (engine, read-only)`
- `cad/lh1/head_l/build.py (head and tool parts)`
- `cad/lh1/params.py`
- `cad/lh1/deck_k/build.py (VC-2)`
