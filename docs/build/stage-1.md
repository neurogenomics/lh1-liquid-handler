# Stage 1: second carriage B with a fixed single-channel 0.5-10 uL pipette

Designed and simulated only. Nothing is built.

**Demonstrates:** two carriages without collision (firmware keep-out), single-channel tube work, 8-channel and single-channel in one run

**Test before the next stage:** both carriages run the T0 programme interleaved for 4 h with no collision guard trip; S10 gravimetric at 1, 5, 10 uL

## What it adds

**head_and_tool**: lines 26, qty 35
- ZC-L_z_carriage x 1
- CBR-L_coupling_bracket (with the latch channel, unchanged) x 1
- PB-F_pusher_bracket (U standoff for the LGA35) x 1
- PRX-L_pusher_rod_extension x 1
- EJA-F_eject_arm_post_pin (line x -26) x 1
- FGP-F_ffc_guide_plate x 1
- FFA-F_ffc_moving_arm x 1
- MCL-K_clamp_fork (MVP clamp, replaces the LTG-1 latch until stage 5) x 1
- MCL-S_set_screw_M5x12_dog_point_12.9 x 1
- TPL-L_tool_plate x 1
- CBL-S10_cradle_body x 1
- CBL-straps x 2
- MGN15_rail_479_Z x 1
- MGN15H_block (4 X + 2 per Z axis) x 2
- NEMA17_48 Z motor x 1
- T8x2_screw_473_and_nut x 1
- Nanotec_LGA351S12-A-UIAP-038 pusher (LGA35) x 1
- tooling_ball_D10_threaded_shank x 3
- vee_seat_hardened_90deg_16x14x8 x 3
- retention_knob_45deg_D16_M10 x 1
- magnet_N52_D4x2 (tool-plate ID magnets, fitted as on every tool plate) x 4
- sealed switch: cradle FORCE switch on the push-rod x 1
- D2F switch: pusher CONTACT switch (pad on the slider) x 1
- sealed switch: pusher RETRACTED switch x 1
- FFC 20-way 0.5 mm (carriage B) x 1
- single-channel 0.5-10 uL pipette x 1

**deck**: p10 tip frame at G1 (starter place), small-tube rack ATR-3 at A2 (passive, plain insert already fitted)

**sub_assemblies**:
- carriage B Z axis + pusher + MCL-1 clamp
- S10 tool

## Electrical additions

**second carriage B with a fixed single-channel 0.5-10 uL pipette (MCL-1 clamp)**

- devices added: MMB_B, Z_MOTOR_B, Z_HOME_B, Z_UP_B, PUSH_MOTOR_B, PUSH_HOME_B, PUSH_CONTACT_B, FORCE_SW_B
- cables added (8):
  - W-C-10: CAN_SPLICE to MMB_B, 24V MOTION_H + CAN Klipper, XT30_LEAD, 150 mm, fuse F22
  - W-H-02: MMB_B to Z_MOTOR_B, MOTOR Z_MOTOR_B, MOTOR_LEAD, 300 mm, fuse driver current limit
  - W-H-06: MMB_B to Z_HOME_B, ENDSTOP Z_HOME_B, MC2_025, 350 mm, fuse signal
  - W-H-21: MMB_B to PUSH_MOTOR_B, MOTOR PUSH_MOTOR_B (phases doubled), FFC_20_05MM, 850 mm, fuse driver current limit
  - W-H-22: MMB_B to PUSH_HOME_B, ENDSTOP PUSH_HOME_B, FFC_20_05MM, 850 mm, fuse signal
  - W-H-23: MMB_B to PUSH_CONTACT_B, ENDSTOP PUSH_CONTACT_B (pusher pad on the slider), FFC_20_05MM, 900 mm, fuse signal
  - W-H-24: MMB_B to FORCE_SW_B, ENDSTOP FORCE_SW_B (push-rod cradle force switch), FFC_20_05MM, 900 mm, fuse signal
  - W-H-27: MMB_B to HALL_B, 0V carriage B switch and Hall return (shared), FFC_20_05MM, 900 mm, fuse signal
- firmware: sections added mcu head_b, manual_stepper z_b, tmc5160 manual_stepper z_b, manual_stepper pusher_b, tmc5160 manual_stepper pusher_b, gcode_button force_b, gcode_button contact_b; no longer marked absent: carriage_b

Source: [upgrade_steps.json](../../electronics/lh1/out/mvp/upgrade_steps.json).

**Carried over:** all stage 0 parts

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
- `cad/lh1/head_l/build.py (carriage B is the same axis at x 0)`
