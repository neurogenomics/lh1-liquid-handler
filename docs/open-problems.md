# Open problems

Problems found by the latest independent design check (24 November 2026) and by the design's own checks. Only the hardware, firmware and electrical findings are listed. Each is a problem statement: none is solved.

Severity: W1 and W2 are blockers for the automatic tool exchange (stage 5 of the roadmap). The MVP does not depend on them.

## W1

**The latch may not keep its keyhole aligned on the knob head during a tool exchange.**

At full open the latch arm is about 10 degrees short of vertical, so the roller can rise only about 0.27 mm more before the toggle passes top dead centre. The firmware moves the dock lift bar and Z in alternate 1 mm steps, so after each half-step the roller is pushed beyond full open. The bar-to-carriage height must stay within about -0.22/+0.17 mm through the 13 mm follow, and the build allows more than that. A tool could be left half-captured. Needed: a hard open stop well short of vertical, a compliant bar pad, coordinated bar and Z moves, a switch that proves the jaw is aligned, and a stroke check at every half-step with the tolerances.

Files: `cad/lh1/head_l/latch.py`, `cad/lh1/head_l/build.py`, `firmware/lh1/printer.cfg (_LH_DROP, _LH_PICKUP, _LH_DOCK)`, `cad/lh1/head_l/out/report.json (latch_stroke_check, assembly_steps_and_gauges)`

## W2

**The latch pushes the tool sideways about as hard as it pulls it up.**

The preload is applied through a single-sided 45 degree wedge, so the stud also takes the jaw force sideways (about 600-925 N, the size of the vertical preload). A three-vee coupling with 45 degree flanks resists a sideways load of roughly half its preload, so the tool position would be set by friction, not by the kinematic contacts, and the 0.10 mm repeatability at the nozzle has no basis. No calculation includes this load. Needed: an axial, symmetric pull (two opposed jaws, a ball-lock or collet, or a vertical drawbar), the lateral term added to the coupling calculation, and a rig test that logs tool shift during latching.

Files: `cad/lh1/head_l/latch.py (preload_from_stack)`, `cad/lh1/head_l/build.py (coupling_calcs, jaw geometry)`

## W3

**The re-latch margin meets the 1.5 rule only at the nominal setting.**

Inside the latch's own setting band, or with a slightly high knob, the margin falls to about 1.39-1.49; with reduced spring force it falls further. The constant-force springs are also highly stressed. Needed: more margin or a tighter, checkable setting.

Files: `cad/lh1/head_l/latch.py`, `cad/lh1/head_l/out/report.json`

## W4

**The force audit does not stop the build when a mechanism fails it.**

The design rule says a margin below 1.5 fails the build, but no check reads the audit's pass flag. The starter's report has pass = false (thermocycler lid clamp 1.27). Needed: make the audit a gate in the checks.

Files: `cad/lh1/machine_family/mechanisms.py`, `cad/lh1/machine_family/out/S/report.json (mechanism_audit)`

## W8

**The tip-parking comb passes its tolerance check only narrowly.**

Worst case 0.383 mm against 0.40 mm of clearance (ratio 1.04), and only if the plates are cut to the measured nozzle row of the pipette used. Needed: more clearance or a design that does not depend on a scanned nozzle row.

Files: `cad/lh1/machine_family/geometry.py (TPKS)`, `cad/lh1/machine_family/out/S/report.json (tip_parking)`

## TC-1

**The thermocycler lid clamp fails the force audit.**

Margin 1.27 against the 1.5 rule. Needed: a stronger clamp drive or less clamp force, then a rig measurement.

Files: `cad/lh1/thermocycler/`, `cad/lh1/machine_family/out/S/report.json (mechanism_audit)`

## W10

**The firmware's stale-read lint can be bypassed.**

It finds switch reads only directly in a macro's body, so a wrapper macro that calls a reader is missed. Needed: follow calls when linting.

Files: `electronics/lh1/fw_render_check.py`, `firmware/lh1/printer.cfg`

## W11

**Force-ended tip pick-up is slow.**

It steps 0.2 mm at a time from the host with a settle per step, about 9 s for a 6 mm window. Needed: an MCU-side stop on the force switch.

Files: `firmware/lh1/printer.cfg (tip pick-up macros)`

## W12

**The heater-shaker interlock is a host-declared state with no expiry.**

The firmware accepts the shaker's idle state from the host at any time and never checks it against the device. Needed: a device-confirmed state or a timeout.

Files: `firmware/lh1/printer.cfg (LH_SHAKER)`, `electronics/lh1/system.yaml (heater_shaker_interface)`

## W13

**The latch resistor ladder can be fooled by leakage.**

A broken wire or a short reads outside every band (safe), but a leakage path of about 5 kOhm (wet splices) could move one valid state into another. Needed: protected wiring or a coding that leakage cannot shift.

Files: `electronics/lh1/system.yaml`, `electronics/lh1/wiring.csv`

## W14

**The dock lift homes across two controller boards.**

The stepper is on the main board and its switch on the io board, so homing relies on Klipper's multi-MCU synchronisation. It stops safely on a timeout, but the names in the configuration need checking on the Klipper version installed.

Files: `firmware/lh1/printer.cfg (dock lift)`

## W15

**The dock lift lead screw is only marginally self-locking.**

Lead angle 5.2 degrees against a friction angle of about 5.9 degrees, so a raised bar may creep down under vibration with power off. Needed: a brake, a finer screw or a park position that does not rely on friction.

Files: `cad/lh1/head_l/`, `electronics/lh1/system.yaml (dock_lift)`

## W16

**The latch calculation file contradicts itself in places.**

The text names a four-disc cartridge while the code uses three; the wear list names plain bushings while the pins use needle bearings. Needed: one consistent description and a load check of the needle bearings.

Files: `cad/lh1/head_l/latch.py`

## W17

**Conductor protection ignores grouping.**

The fuses match their conductors on the file's own table, but no grouping factor is applied where several loaded circuits share a duct, although the docstring says it is. Needed: apply the factor or correct the text.

Files: `electronics/lh1/check.py`, `electronics/lh1/cable_types.yaml`

## W18

**The thermocycler guard checks only the centre of an eight-channel row.**

An eight-channel row spans +/-31.5 mm, so channels can be over the thermocycler zone while the row centre is outside. Needed: test the row's footprint.

Files: `firmware/lh1/printer.cfg (_LH_TC_GUARD)`

## W19

**Dropped-tool detection acts only when the running macro ends.**

It is a button script, not an interrupt. Needed: a detection that stops motion at once.

Files: `firmware/lh1/printer.cfg (DROPPED_TOOL)`

## W45

**The 20-way flat cable to carriage B has no spare way and runs near its rating.**

All 20 ways are used and the current is about 99 % of a peak rating. Needed: a larger cable or fewer conductors.

Files: `electronics/lh1/system.yaml (FFC_RATING_BASIS)`, `electronics/lh1/wiring.csv`

## W53

**The X travel end margin is small.**

10.5 mm against a design rule of 10 mm (ratio 1.05).

Files: `cad/lh1/machine_family/out/S/report.json (travel)`

## FW

**Klipper has never loaded this configuration.**

printer.cfg is parsed for syntax, duplicates and wiring only. A Klipper commit after v0.13.0 is needed, and nothing has run on a board.

Files: `firmware/lh1/printer.cfg`, `electronics/lh1/system.yaml (klipper_target)`

## GR

**Gripper jaw force has no data.**

The mechanism audit cannot rate the gripper until the motor's current or a rig measurement is known.

Files: `cad/lh1/machine_family/mechanisms.py`

## SH

**No heater-shaker model is chosen.**

Its speed, temperature range and interface are not fixed.

Files: `electronics/lh1/system.yaml (heater_shaker_interface)`

## Warnings from the electrical check

From `electronics/lh1/out/check_report.json` at export time:

- `FFC_RATING_BASIS`: the head FFC check compares rms per conductor with a continuous (thermal) rating; if the cable or ZIF bought states only a PEAK rating, HL_A 0.99 A peak = 99 %, HL_B 0.495 A peak = 99 %, HL_G 0.495 A peak = 99 % of it: buy parts with a published continuous rating >= HL_A 0.875 A, HL_B 0.437 A, HL_G 0.437 A (system.yaml plunger_motor purchase condition)
- `SUPPLY_FEEDER_SHORT_RUN`: W-P-01 4.0 mm2 (600 mm, Iz 23.0 A) is protected only by the supply's current limit (about 30.0 A): short feeder before the first fuse (competent person: IEC 60204-1 7.2.8)
- `SUPPLY_FEEDER_SHORT_RUN`: W-P-02 4.0 mm2 (700 mm, Iz 23.0 A) is protected only by the supply's current limit (about 30.0 A): short feeder before the first fuse (competent person: IEC 60204-1 7.2.8)
- `SUPPLY_FEEDER_SHORT_RUN`: W-P-03 4.0 mm2 (700 mm, Iz 23.0 A) is protected only by the supply's current limit (about 30.0 A): short feeder before the first fuse (competent person: IEC 60204-1 7.2.8)
- `SUPPLY_FEEDER_SHORT_RUN`: W-P-50 4.0 mm2 (450 mm, Iz 23.0 A) is protected only by the supply's current limit (about 30.0 A): short feeder before the first fuse (competent person: IEC 60204-1 7.2.8)
- `TC_STRIP_FIXED_OBSTACLE`: the TC-1 lid guide strip (CAD keepouts, top z 140) is a fixed obstacle above the lid guard's z 129: the firmware does not guard fixed obstacles (planner keep-out)
- `TIP_GATE_NOT_ARMED`: tip gate beam position now from the CAD (family keepouts tip_check_gate_beam: y -60, z 105, faces); LH_TIP_CHECK stays disarmed (NOT OBSERVED) until the commissioning dry run measures the beam with real tips
- `ADR009_LOWER_FIGURE`: spare fitted TEC channels 0 < 2: lower figure stated in system.yaml family.io_budget (TEC_WARM, two channels, reserved: space, fuse, USB port, supply headroom)
- `COMPARTMENT_ONE_FAN`: one compartment fan failed: 24.0 CFM for 139 W = 10.2 K rise (> 10 K); each fan's rotation-detect output is wired (io PF0 / PF1): a stopped fan is reported and LH_THERM_ON refused (class figures, UNVERIFIED)
- `NOT_LOADED_BY_KLIPPER`: printer.cfg is parsed for syntax, duplicates and wiring only; Klipper has not loaded it
- `NOT_SAFETY_ASSESSED`: no electrical safety assessment has been done (system.yaml safety_assessment)
