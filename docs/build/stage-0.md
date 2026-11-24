# Stage 0: MVP: one carriage, one fixed eight-channel pipette, passive deck

Designed and simulated only. Nothing is built.

**Demonstrates:** XYZ motion, homing and the e-stop/door chain; force-ended pick-up of eight p300 tips; aspirate/dispense with the pusher; reservoir-to-plate and plate-to-plate transfers; a serial dilution across a plate; eject at the tip waste; repeated cycles

**Test before the next stage:** T0 (mvp_notes.txt): gravimetric transfer of 8 x 100 uL and 8 x 30 uL to a plate weighed on a lab balance, 10 repeats: per-channel mean within 2 % and CV within 1.5 % at 100 uL (ISO 8655 class targets, ASSUMED for this machine); a 1:2 serial dilution of dye over 11 columns read by absorbance, slope within 5 % of the ideal; 200 pick-up/eject cycles with no missed or retained tip; 20 re-clamps of the tool with nozzle position repeat <= 0.10 mm

## What it adds

**machine_parts**: lines 114, make_qty 128, buy_qty 38

**head_and_tool**: lines 29, qty 42
- CP-L_carriage_plate x 1
- ZC-L_z_carriage x 1
- CBR-L_coupling_bracket (with the latch channel, unchanged) x 1
- PB-F_pusher_bracket (U standoff for the LGA35) x 1
- PRX-L_pusher_rod_extension x 1
- EJA-F_eject_arm_post_pin (line x -26) x 1
- FGP-F_ffc_guide_plate x 1
- FFA-F_ffc_moving_arm x 1
- XCB-F_x_chain_end_bracket x 1
- MCL-K_clamp_fork (MVP clamp, replaces the LTG-1 latch until stage 5) x 1
- MCL-S_set_screw_M5x12_dog_point_12.9 x 1
- TPL-L_tool_plate x 1
- CBL-M300_cradle_body x 1
- CBL-straps x 1
- CBL-C_manifold_cheeks (fixed + loose) x 2
- MGN15_rail_479_Z x 1
- MGN15H_block (4 X + 2 per Z axis) x 6
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
- FFC 20-way 1.0 mm (carriage A) x 1
- eight-channel 30-300 uL pipette (bought, plunger driven by the pusher) x 1

**sub_assemblies**:
- frame, base, feet
- gantry, beam, X and Y drives, chains
- deck strips F/A/B/G
- carriage A + MCL-1 clamp + M300 tool
- passive places: tip frame G3, reservoir F2, plates B2/B4, tip waste B1
- drawer with tip crate
- enclosure with door sensor, e-stop
- control compartment

**Carried over:** everything (starter parts unchanged; MCL-1 kept on carriage A until stage 5)

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
- `cad/lh1/machine_family/mvp.py`
- `cad/lh1/machine_family/out/MVP/*`
- `cad/lh1/visualise/export_mvp.py`
