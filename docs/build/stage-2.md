# Stage 2: plate gripper and lid handling

Designed and simulated only. Nothing is built.

**Demonstrates:** moving plates between places and lids on and off, so a run can pass plates without the person

**Test before the next stage:** 500 plate moves and 200 lid lifts with no drop; grip force at 0.35 A measured (the audit has NO DATA for it)

## Open before it can proceed
- gripper jaw force at the firmware current: NO DATA in the mechanism audit (needs the motor's rated current or a rig measurement)

## What it adds

**head_and_tool**: lines 10, qty 12
- ZC-L_z_carriage x 1
- FGP-F_ffc_guide_plate x 1
- FFA-F_ffc_moving_arm x 1
- GR-1 gripper set (hanger, body, fingers) x 1
- MGN15_rail_479_Z x 1
- MGN15H_block (4 X + 2 per Z axis) x 2
- NEMA17_48 Z motor x 1
- T8x2_screw_473_and_nut x 1
- sealed switches: gripper open + gripped x 2
- FFC 12-way 0.5 mm (gripper) x 1

**shelf**: lines 3, custom_qty 8, bought_qty 0, note the shelf plate is needed for the lid park OG; tip towers on it come in stage 6
- SPT-F_shelf_pocket_tray_IG
- STG-F_tower_guides
- TSS-F_tip_supply_shelf

**deck**: plate stack F3 (plain insert already fitted)

## Electrical additions

**plate gripper and lid handling (gripper Z on head_a EZ3, jaws on head_b EZ3)**

- devices added: Z_MOTOR_GRIP, Z_HOME_GRIP, Z_UP_GRIP, GRIP_MOTOR, GRIP_OPEN_SW, GRIPPED_SW
- cables added (5):
  - W-H-03: MMB_A to Z_MOTOR_GRIP, MOTOR Z_MOTOR_GRIP, MOTOR_LEAD, 450 mm, fuse driver current limit
  - W-H-07: MMB_A to Z_HOME_GRIP, ENDSTOP Z_HOME_GRIP, MC2_025, 400 mm, fuse signal
  - W-H-09: MMB_B to GRIP_MOTOR, MOTOR GRIP_MOTOR, FFC_12_05MM, 950 mm, fuse driver current limit
  - W-H-31: MMB_B to GRIP_OPEN_SW, ENDSTOP GRIP_OPEN_SW, FFC_12_05MM, 1000 mm, fuse signal
  - W-H-32: MMB_B to GRIPPED_SW, ENDSTOP GRIPPED_SW, FFC_12_05MM, 1050 mm, fuse signal
- firmware: sections added tmc5160 stepper_z, manual_stepper gripper, tmc5160 manual_stepper gripper, gcode_button gripped; commands restored LH_GRIPPER, LH_GRIPPER_LIFT, LH_GRIPPER_RELEASE, LH_GRIPPER_ASSERT_HOLDING; no longer marked absent: gripper

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
- `cad/lh1/head/build.py (gripper force)`
