# Stage 5: exchangeable tools and the dock

Designed and simulated only. Nothing is built.

**Demonstrates:** the machine changes its own tools

**Test before the next stage:** CT-0a/b latch release and re-latch; CT-1 coupling repeat <= 0.10 mm at the nozzle over 500 exchanges; 20 exchanges per carriage per bay

## Open before it can proceed
- W1 keyhole alignment during the exchange (verification 24 Nov 2026): jaw stays on the head only within -0.22/+0.17 mm of bar height
- W2 one-sided 45 deg wedge preload pushes the tool sideways about as hard as the preload; no calculation covers it
- major: re-latch margin 1.5 only at nominal (1.39-1.49 in its band, 1.04 in CT-0b); constant-force springs at about 4 GPa
- the redesign must be rig-tested before any machine purchase (ADR-014)

## What it adds

**dock**: lines 14, custom_qty 45, bought_qty 10
- DMK-F_dock_bay_marker
- DOCK_base
- DOCK_drip_tray
- DOCK_lift_actuator
- DOCK_lift_bar
- DOCK_lift_flags
- DOCK_lift_guides
- DOCK_lift_home_switch
- DOCK_lift_open_switch
- DOCK_lift_pad
- DOCK_pins
- DOCK_saddle
- DOCK_switch

**head_and_tool**: lines 12, qty 61
- TPL-L_tool_plate x 4
- CBL-M10/M50/S1000_cradle_bodies x 3
- SYR-L_syringe_body_clamp x 1
- CBL-straps x 4
- CBL-C_manifold_cheeks (fixed + loose) x 4
- LTG-1 latch set (jaw, link/crank/arm, anchor, cartridge, flag, switch bracket) x 2
- HCP-F_hall_clamp_plate x 2
- tooling_ball_D10_threaded_shank x 12
- retention_knob_45deg_D16_M10 x 4
- magnet_N52_D4x2 (tool-plate ID magnets, fitted as on every tool plate) x 16
- latch bought set (disc springs, needle bearings, D4 pins, constant-force springs, roller, LOCK/EMPTY switches, ladder resistors, splices) x 1
- Hall switch 6.5 mm leaded (tool ID) x 8

## Electrical additions

**exchangeable tools and the dock (LTG-1 latches replace the MCL-1 clamps; latch ladders, Hall codes, dock lift, bay switches)**

- devices added: LATCH_SW_A, LATCH_SW_B, HALL_A, HALL_B, DOCK_LIFT_MOTOR, DOCK_LIFT_HOME, DOCK_BAR_OPEN, BAY_SW_1, BAY_SW_2, BAY_SW_3, BAY_SW_4, BAY_SW_5, BAY_SW_6, BAY_SW_7, BAY_SW_8
- cables added (15):
  - W-H-15: MMB_A to LATCH_SW_A, SIGNAL LATCH_SW_A (LOCKED / CLOSED-EMPTY / NOT CLOSED), FFC_20_1MM, 1000 mm, fuse signal
  - W-H-16: MMB_A to HALL_A, SIGNAL HALL_A tool code + cable loop, FFC_20_1MM, 1000 mm, fuse signal
  - W-H-25: MMB_B to LATCH_SW_B, SIGNAL LATCH_SW_B (LOCKED / CLOSED-EMPTY / NOT CLOSED), FFC_20_05MM, 900 mm, fuse signal
  - W-H-26: MMB_B to HALL_B, SIGNAL HALL_B tool code + cable loop, FFC_20_05MM, 900 mm, fuse signal
  - W-I-01: IO_CTRL to BAY_SW_1, ENDSTOP dock bay 1 occupied, MC2_025, 2000 mm, fuse signal
  - W-I-02: IO_CTRL to BAY_SW_2, ENDSTOP dock bay 2 occupied, MC2_025, 1900 mm, fuse signal
  - W-I-03: IO_CTRL to BAY_SW_3, ENDSTOP dock bay 3 occupied, MC2_025, 1850 mm, fuse signal
  - W-I-04: IO_CTRL to BAY_SW_4, ENDSTOP dock bay 4 occupied, MC2_025, 1750 mm, fuse signal
  - W-I-05: IO_CTRL to BAY_SW_5, ENDSTOP dock bay 5 occupied, MC2_025, 1700 mm, fuse signal
  - W-I-06: IO_CTRL to BAY_SW_6, ENDSTOP dock bay 6 occupied, MC2_025, 1750 mm, fuse signal
  - W-I-07: IO_CTRL to BAY_SW_7, ENDSTOP dock bay 7 occupied, MC2_025, 1850 mm, fuse signal
  - W-I-08: IO_CTRL to BAY_SW_8, ENDSTOP dock bay 8 occupied, MC2_025, 1950 mm, fuse signal
  - W-I-40: MAIN_CTRL to DOCK_LIFT_MOTOR, MOTOR DOCK_LIFT_MOTOR, MC4_05_SH, 1650 mm, fuse driver current limit
  - W-I-41: XT to DOCK_LIFT_HOME, SENSOR dock lift bar DOWN (supply + output), MC3_025, 2150 mm, fuse F7
  - W-I-43: DOCK_BAR_OPEN to EXPB1, SIGNAL dock lift bar at the OPEN height, MC2_025, 2450 mm, fuse signal
- firmware: sections added manual_stepper dock_lift, tmc5160 manual_stepper dock_lift, gcode_button bay\d, gcode_button dock_bar_open, gcode_button hall_[ab]\d, gcode_button latch_[ab].*, gcode_button cable_[ab]; commands restored LH_TOOL_PARK, LH_TOOL_PICK, LH_TOOL_EXCHANGE, LH_DOCK_LEAVE, LH_DOCK_STATUS, _LH_DOCK_ENTER; no longer marked absent: dock

Source: [upgrade_steps.json](../../electronics/lh1/out/mvp/upgrade_steps.json).

**Carried over:** all; tool plates, vee seats and retention knobs are already the full design's

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
- `cad/lh1/head_l/latch.py`
- `cad/lh1/head_l/build.py (bay_parts, latch_parts)`
- `cad/lh1/bench_rig/build.py`
