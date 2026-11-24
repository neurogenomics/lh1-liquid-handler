# Stage 3: deck modules one at a time under the module standard (pitch 132 x 110, insert 130 x 108 x 5, 4 x M4, 2 x D4)

Designed and simulated only. Nothing is built.

**Demonstrates:** each module's own function on the deck: magnetic bead separation, shaking, held temperature, liquid waste to canisters

**Test before the next stage:** per module: its own acceptance test (setpoint held, bead capture, shake without spill) and the deck rule and interference checks re-run with that module in the layout

## Open before it can proceed
- no shaker model chosen (verification 24 Nov 2026)

## What it adds

**modules (each a separate step, any order)**: 
- magnet MB-1 at B4: lines 2, custom_qty 2, bought_qty 0; parts: IN-P_plain_insert_plate, MB-1_magnet_block
- heater-shaker at F4 on its tray: lines 4, custom_qty 4, bought_qty 0; parts: SKT-F_shaker_tray, shaker_adapter, shaker_body, shaker_mounts
- cold places: CTB-1 + CTB-2 at A3/A4, cold strip at B2, CPB-1 at G4: lines 6, custom_qty 6, bought_qty 4; parts: CON-F_fan_connector (keyed, 2-way, panel mount), CON-F_place_connector (keyed, panel mount, 6-pin), IN-P_plain_insert_plate, thermal_module_rev_K, thermal_module_rev_K (CTB-2 recessed into it)
- liquid waste LW-1 at B3 + canisters: lines 6, custom_qty 1, bought_qty 7; parts: CON-F_place_connector (keyed, panel mount, 6-pin), LW-1_drain_FEP_1, LW-1_drain_FEP_2, LW-1_twin_waste_funnel, LWB-1_waste_canister_2L, LWB-2_waste_canister_2L
- hazardous-liquid trough G0, ATR-3 tube rack A2 (passive): lines 2, custom_qty 2, bought_qty 1; parts: CON-F_place_connector (keyed, panel mount, 6-pin), IN-P_plain_insert_plate

**extraction**: inline fan on the left side panel (EXF-F) when ethanol is first used

## Electrical additions

**deck modules, common part: third supply PSU_AUX, FT2, XM, TB_RELAY, EXP breakouts, USB hubs, second compartment fan, mains strip**

- devices added: PSU_AUX, FT2, XM, TB_RELAY, EXPB1, EXPB2, HUB1, HUB2, CAB_FAN2, KM6, MAINS_STRIP
- cables added (31):
  - W-E-18: PE_STUD to PSU_AUX, PE, PE_GY_15, 750 mm
  - W-E-19: PE_STUD to MAINS_STRIP, PE switched strip, PE_GY_15, 500 mm
  - W-P-50: PSU_AUX to FT2, 24V PSU_AUX out, SC_40, 450 mm, fuse PSU_AUX current limit (about 30 A)
  - W-P-51: PSU_AUX to XT, 0V common link PSU_AUX-PSU_MAIN, SC_15, 950 mm, fuse PSU_AUX limit (reference link)
  - W-P-14: FT2 to KM1, 24V THERM_AUX, SC_40, 1050 mm, fuse F4
  - W-P-15: KM1 to KM2, 24V THERM_AUX, SC_40, 300 mm, fuse F4
  - W-P-16: KM2 to XT, 24V THERM_AUX (switched), SC_40, 550 mm, fuse F4
  - W-P-23: PSU_5V to HUB1, 5V hub 1, SC_05, 650 mm, fuse PSU_5V current limit
  - W-P-25: XT to CAB_FAN2, 24V LOGIC (F7) compartment intake fan 2, DEVICE_LEAD, 1000 mm, fuse F7
  - W-S-26: XT to KM1, 24V SAFE to aux chain, SC_05, 600 mm, fuse F6
  - W-S-27: KM1 to KM2, 24V SAFE aux chain, SC_05, 300 mm, fuse F6
  - W-S-28: KM2 to XT, 24V SAFE_E (e-stop OK), SC_05, 550 mm, fuse F6
  - W-C-03: HOST to HUB1, USB hub 1 uplink (bay, thermal, cameras), USB2_AB, 350 mm
  - W-S-47: TB_RELAY to MAIN_CTRL, SIGNAL TB_RELAY mirror 2 (status), MC2_025, 750 mm, fuse signal
  - W-P-27: XT to TB_RELAY, 24V THERM_AUX, SC_40, 800 mm, fuse F4
  - W-K-16: TB_RELAY to FT2, 24V THERM_AUX (switched) to the controller fuses, SC_40, 400 mm, fuse F4
  - W-S-38: TB_RELAY to XT, 0V cold-module contactor coil return, SC_05, 800 mm, fuse F6
  - W-N-22: IO_CTRL to EXPB1, SIGNAL EXP1 breakout, PLUG_IN, 0 mm
  - W-N-23: IO_CTRL to EXPB2, SIGNAL EXP2 breakout, PLUG_IN, 0 mm
  - W-P-54: KM2 to XM, 24V MODULE (switched) to the place fuses, SC_15, 1100 mm, fuse F9
  - W-P-52: FT2 to KM1, 24V MODULE, SC_15, 1050 mm, fuse F9
  - W-P-53: KM1 to KM2, 24V MODULE, SC_15, 300 mm, fuse F9
  - W-P-55: XT to XM, 0V MODULE and SAFE return, SC_15, 900 mm, fuse F9 (return)
  - W-C-21: HOST to HUB2, USB hub 2 uplink (head, deck places), USB2_AB, 400 mm
  - W-P-24b: PSU_5V to HUB2, 5V hub 2, SC_05, 650 mm, fuse PSU_5V current limit
  - W-I-42: XT to EXPB1, SIGNAL dock lift bar DOWN (endstop), SC_05, 450 mm, fuse signal
  - W-I-31: CAB_FAN2 to XDB5, SIGNAL compartment fan 2 rotation detect, DEVICE_LEAD, 900 mm, fuse signal
  - W-M-05: PSU_5V to KM6, MAINS 230 V L/N to the strip contactor, MAINS_2X10, 950 mm, fuse inlet T10A
  - W-M-06: KM6 to MAINS_STRIP, MAINS 230 V L/N switched strip, MAINS_2X10, 500 mm, fuse inlet T10A
  - W-S-50: XT to KM6, 24V SAFE_E strip contactor coil, SC_05, 850 mm, fuse F6
  - W-S-51: KM6 to XT, 0V coil return KM6, SC_05, 850 mm, fuse F6
- firmware: sections added gcode_button safe_.*, gcode_button tb_mirror, gcode_button cab_fan_ok_2; no longer marked absent: modules

**module B2 (cold strips) on TEC_COLD ch1 (replaces the plate insert at B2)**

- devices added: TEC_COLD, TM_B2, MODCONN_B2
- cables added (8):
  - W-C-05: HUB1 to TEC_COLD, USB cold-block controller (MeCom), USB2_AB, 1100 mm
  - W-K-10: FT2 to TEC_COLD, 24V THERM_AUX (switched) TEC_COLD, SC_15, 1350 mm, fuse F33
  - W-K-13: TEC_COLD to PSU_AUX, 0V THERM_AUX return TEC_COLD, SC_15, 1650 mm, fuse F33 (return)
  - W-K-01: TEC_COLD to TM_B2, TEC THERM_AUX TM_B2 Peltier + NTCs, MC6_075, 950 mm, fuse TEC_COLD 3.5 A current limit
  - W-S-35: XT to TM_B2, 24V SAFE_E cold-module coil chain, MC2_05, 1150 mm, fuse F6
  - W-N-02: XM to MODCONN_B2, 24V MODULE place B2 + SAFE lines, MC6_05, 1550 mm, fuse F42
  - W-N-12: HUB2 to MODCONN_B2, USB deck place B2, USB2_AB, 1250 mm
  - W-N-30: MODCONN_B2 to TM_B2, 24V MODULE TM_B2 fan + SAFE loop-back, PLUG_IN, 0 mm

**modules A3+A4 (tube blocks) on TEC_TUBE**

- devices added: TEC_TUBE, TM_A3, TM_A4, MODCONN_A3, CTB_FANCON
- cables added (12):
  - W-K-11: FT2 to TEC_TUBE, 24V THERM_AUX (switched) TEC_TUBE, SC_15, 1500 mm, fuse F34
  - W-K-14: TEC_TUBE to PSU_AUX, 0V THERM_AUX return TEC_TUBE, SC_15, 1750 mm, fuse F34 (return)
  - W-K-03: TEC_TUBE to TM_A3, TEC THERM_AUX TM_A3 Peltier + NTCs, MC6_075, 1200 mm, fuse TEC_TUBE 3.5 A current limit
  - W-K-04: TEC_TUBE to TM_A4, TEC THERM_AUX TM_A4 Peltier + NTCs, MC6_075, 1350 mm, fuse TEC_TUBE 3.5 A current limit
  - W-S-60: TM_B2 to TM_A3, 24V SAFE_E cold-module coil chain, MC2_05, 950 mm, fuse F6
  - W-S-61: TM_A3 to TM_A4, 24V SAFE_E cold-module coil chain, MC2_05, 1350 mm, fuse F6
  - W-N-01: XM to MODCONN_A3, 24V MODULE place A3 + SAFE lines, MC6_05, 1600 mm, fuse F41
  - W-N-11: HUB2 to MODCONN_A3, USB deck place A3, USB2_AB, 1350 mm
  - W-N-31: MODCONN_A3 to TM_A3, 24V MODULE TM_A3 fan + SAFE loop-back, PLUG_IN, 0 mm
  - W-N-35: XM to CTB_FANCON, 24V MODULE zone A3+A4 second fan (F41), MC2_05, 1650 mm, fuse F41
  - W-N-36: CTB_FANCON to TM_A4, 24V MODULE TM_A4 heat-sink fan, PLUG_IN, 0 mm
  - W-C-23: HUB1 to TEC_TUBE, USB tube-block controller (MeCom), USB2_AB, 1250 mm

**module G4 (cold plate) on TEC_COLD ch2, through the connector plane**

- devices added: TM_G4, MODCONN_G4
- cables added (6):
  - W-K-02: TEC_COLD to TM_G4, TEC THERM_AUX TM_G4 Peltier + NTCs, MC6_075, 1350 mm, fuse TEC_COLD 3.5 A current limit
  - W-S-62: TM_A4 to TM_G4, 24V SAFE_E cold-module coil chain, MC2_05, 1650 mm, fuse F6
  - W-S-37: TM_G4 to TB_RELAY, 24V SAFE_E cold-module coil chain end, MC2_05, 1800 mm, fuse F6
  - W-N-03: XM to MODCONN_G4, 24V MODULE place G4 + SAFE lines, MC6_05, 1950 mm, fuse F43
  - W-N-13: HUB2 to MODCONN_G4, USB deck place G4, USB2_AB, 1700 mm
  - W-N-32: MODCONN_G4 to TM_G4, 24V MODULE TM_G4 fan + SAFE loop-back, PLUG_IN, 0 mm

**reserved place G0 (loop-back plug) and the heater-shaker at F4 on the switched mains strip**

- devices added: MODCONN_G0, SHAKER
- cables added (4):
  - W-N-04: XM to MODCONN_G0, 24V MODULE place G0 + SAFE lines, MC6_05, 1900 mm, fuse F44
  - W-N-14: HUB2 to MODCONN_G0, USB deck place G0, USB2_AB, 1650 mm
  - W-N-42: HUB2 to SHAKER, USB heater-shaker (host device, own supply), USB2_AB, 1650 mm
  - W-N-45: MAINS_STRIP to SHAKER, MAINS heater-shaker supply (moulded cord), DEVICE_LEAD, 1250 mm
- firmware: commands restored LH_SHAKER; no longer marked absent: shaker

**liquid waste LW-1 at B3 with its two level switches, and the ethanol extraction fan with its airflow switch (assigned to stage 3 by the electrical owner: roadmap.json lists the liquid waste as a module and no stage for the fan)**

- devices added: XDB4, LVL_1, LVL_2, EXT_FAN, AIRFLOW_SW
- cables added (8):
  - W-I-10: IO_CTRL to AIRFLOW_SW, SIGNAL extraction airflow proven, MC2_025, 2050 mm, fuse signal
  - W-I-15: LVL_1 to XT, 24V LOGIC (F7) waste level LVL_1 + OUT, MC3_025, 1350 mm, fuse F7
  - W-I-16: XT to XDB4, SIGNAL waste level LVL_1, SC_05, 450 mm, fuse signal
  - W-I-17: LVL_2 to XT, 24V LOGIC (F7) waste level LVL_2 + OUT, MC3_025, 1250 mm, fuse F7
  - W-I-18: XT to XDB4, SIGNAL waste level LVL_2, SC_05, 450 mm, fuse signal
  - W-I-21: IO_CTRL to XDB4, SIGNAL breakout (inputs), PLUG_IN, 0 mm
  - W-I-25: XT to EXT_FAN, 24V LOGIC (F8) extraction fan, always on, MC2_075, 2300 mm, fuse F8
  - W-P-61: FT to XT, 24V LOGIC (F8 extraction), SC_05, 500 mm, fuse F8
- firmware: sections added gcode_button waste1_ok, gcode_button waste2_ok, gcode_button airflow; commands restored LH_ETHANOL; no longer marked absent: liquid_waste, extraction

Source: [upgrade_steps.json](../../electronics/lh1/out/mvp/upgrade_steps.json).

**Carried over:** all; the plain insert at each module place comes out and is kept as a spare

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
- `cad/lh1/deck_k/build.py (shaker, LW-1, VC-2, tip frames)`
- `cad/lh1/deck_modules/build.py`
- `cad/lh1/deck_modules2/build.py`
