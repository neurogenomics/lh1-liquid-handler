# Stage 4: thermocycler TC-1 at F1 with its lid park at A1

Designed and simulated only. Nothing is built.

**Demonstrates:** on-deck thermal cycling with the lid moved by the machine

**Test before the next stage:** T-TC-05 lid clamp force on the TC-1 rig; block uniformity; the all-tools-up lid interlock proved

## Open before it can proceed
- TC-1 lid clamp FAILS the 1.5 rule in the audit (owner TC-1, rev H)

## What it adds

**TC-1**: lines 3, custom_qty 3, bought_qty 0
- IN-P_plain_insert_plate
- TC-1_lid_guide_strip
- TC-1_thermocycler

## Electrical additions

**thermocycler TC-1 at F1 (lid park A1) and its THERM path (PSU_TC, F5, KM5, TC_RELAY, lid heater, lid drivers, watchdog)**

- devices added: PSU_TC, KM5, TC_RELAY, LID_SSR, K_TU, WDOG, XDA2, XDA5, TEC_TC, TC_DRV_SLIDE, TC_DRV_CLAMP, TC_TEC, TC_LID_HEATER, TC_FAN1, TC_FAN2, TC_M_SLIDE, TC_M_CLAMP, TC_S_OPEN, TC_S_CLOSED, TC_SW_CLAMP, TC_SW_PARKED, TC_SW_SEALED, TC_BEAM_TX, TC_BEAM_RX, TC_NTC_BLOCK, TC_NTC_A1, TC_NTC_SINK, TC_NTC_LID, TC_TS_BLOCK, TC_TS_SINK, TC_TS_LID
- cables added (52):
  - W-E-03: PE_STUD to PSU_TC, PE, PE_GY_15, 1450 mm
  - W-P-03: PSU_TC to FT, 24V PSU_TC out, SC_40, 700 mm, fuse PSU_TC current limit (about 30 A)
  - W-P-04: PSU_TC to XT, 0V common link PSU_TC-PSU_MAIN, SC_15, 600 mm, fuse PSU_TC limit (reference link, no load current)
  - W-S-41: TC_RELAY to KM5, 24V thermostat chain end to KM5 coil, SC_05, 750 mm, fuse F6
  - W-S-43: KM5 to XT, 0V coil return KM5, SC_05, 400 mm, fuse F6
  - W-T-18: MAIN_CTRL to TC_SW_PARKED, ENDSTOP TC lid parked (interlock), MC2_025, 1100 mm, fuse signal
  - W-Z-08: XT to TC_BEAM_RX, SAFETY lid permissive chain: beam receiver supply, MC2_025, 1400 mm, fuse F7
  - W-Z-09: TC_BEAM_RX to K_TU, SAFETY gate: beam receiver output drives K_TU, SC_05, 2050 mm, fuse F7
  - W-Z-10: K_TU to XT, 0V gate relay coil return, SC_05, 750 mm, fuse F7
  - W-Z-11: K_TU to MAIN_CTRL, SIGNAL lid gate readback (K_TU NC), MC2_025, 800 mm, fuse signal
  - W-Z-12: FT to K_TU, 24V MOTION lid drivers (to the gate), SC_05, 950 mm, fuse F23
  - W-Z-13: K_TU to TC_DRV_SLIDE, 24V MOTION lid drivers (gated), SC_05, 1550 mm, fuse F23
  - W-Z-14: TC_DRV_SLIDE to TC_DRV_CLAMP, 24V MOTION lid drivers (gated), SC_05, 400 mm, fuse F23
  - W-Z-15: XT to TC_DRV_SLIDE, 0V lid drivers, SC_05, 1300 mm, fuse F23 (return)
  - W-Z-16: MAIN_CTRL to TC_BEAM_TX, 24V LOGIC beam emitter (switched by FAN0), MC2_025, 1250 mm, fuse Octopus fan fuse
  - W-P-34: FT to KM5, 24V THERM, SC_60, 500 mm, fuse F5
  - W-P-41: KM5 to TC_RELAY, 24V THERM (second channel), SC_60, 750 mm, fuse F5
  - W-P-35: TC_RELAY to FT, 24V THERM (switched) to the TEC fuse, SC_60, 850 mm, fuse F5
  - W-P-44: FT to TEC_TC, 24V THERM (switched) to the TC-1 block controller, SC_25, 950 mm, fuse F32
  - W-P-36: TEC_TC to PSU_TC, 0V THERM return, SC_25, 1900 mm, fuse F32 (return)
  - W-P-42: TC_RELAY to FT, 24V THERM (switched) to the lid-heater fuse, SC_60, 850 mm, fuse F5
  - W-P-43: FT to LID_SSR, 24V THERM lid heater (fused), SC_15, 950 mm, fuse F31
  - W-T-02: LID_SSR to TC_LID_HEATER, HEATER THERM lid, SC_HF_075, 2150 mm, fuse F31
  - W-T-20: TC_LID_HEATER to PSU_TC, HEATER THERM lid return, SC_HF_075, 1600 mm, fuse F31 (return)
  - W-T-21: MAIN_CTRL to LID_SSR, SIGNAL lid heater SSR control, MC2_025, 800 mm, fuse Octopus heater fuse
  - W-S-29: SR1 to TC_TS_BLOCK, 24V SAFE THERM coil chain (from SR1 33-34), SC_05, 1250 mm, fuse F6
  - W-S-30: TC_TS_BLOCK to TC_TS_SINK, 24V SAFE_E coil chain, SC_05, 300 mm, fuse F36
  - W-S-31: TC_TS_SINK to TC_TS_LID, 24V SAFE_E coil chain, SC_HF_025, 1000 mm, fuse F36
  - W-S-32: TC_TS_LID to WDOG, 24V SAFE_E coil chain to the host-loss watchdog, SC_HF_025, 2150 mm, fuse F36
  - W-S-33: WDOG to TC_RELAY, 24V SAFE_E coil chain end to the THERM contactors, SC_05, 400 mm, fuse F36
  - W-S-34: TC_RELAY to MAIN_CTRL, SIGNAL THERM enable: TC_RELAY coil return through HE1, SC_05, 700 mm, fuse F6
  - W-S-44: XT to WDOG, 24V LOGIC watchdog supply, SC_05, 750 mm, fuse F7
  - W-S-45: MAIN_CTRL to WDOG, SIGNAL watchdog heartbeat (2 Hz from the Octopus), MC2_025, 800 mm, fuse Octopus fan fuse
  - W-S-46: TC_RELAY to MAIN_CTRL, SIGNAL TC_RELAY mirror 2 (weld check), MC2_025, 700 mm, fuse signal
  - W-C-04: HUB1 to TEC_TC, USB TC-1 block controller (MeCom), USB2_AB, 1200 mm
  - W-T-01: TEC_TC to TC_TEC, TEC THERM TC-1 string, SC_25_SIL, 450 mm, fuse TEC_TC 13 A current limit + F5
  - W-T-03: XT to TC_FAN1, 24V LOGIC fan TC-1, DEVICE_LEAD, 1300 mm, fuse F7
  - W-T-04: XT to TC_FAN2, 24V LOGIC fan TC-1, DEVICE_LEAD, 1400 mm, fuse F7
  - W-C-16: MAIN_CTRL to XDA2, STEP/DIR TC lid slide, PLUG_IN, 0 mm
  - W-C-17: MAIN_CTRL to XDA5, STEP/DIR TC lid clamp, PLUG_IN, 0 mm
  - W-T-22: XDA2 to TC_DRV_SLIDE, STEP/DIR TC lid slide, MC4_025, 1000 mm, fuse signal
  - W-T-23: XDA5 to TC_DRV_CLAMP, STEP/DIR TC lid clamp, MC4_025, 850 mm, fuse signal
  - W-T-05: TC_DRV_SLIDE to TC_M_SLIDE, MOTOR TC lid slide, MOTOR_LEAD, 500 mm, fuse DM542 current setting
  - W-T-06: TC_DRV_CLAMP to TC_M_CLAMP, MOTOR TC clamp, SC_HF_025, 950 mm, fuse DM542 current setting
  - W-T-07: TC_S_OPEN to XT, SENSOR lid open (supply + output), DEVICE_LEAD, 1900 mm, fuse F7
  - W-T-08: TC_S_CLOSED to XT, SENSOR lid closed (supply + output), DEVICE_LEAD, 2050 mm, fuse F7
  - W-T-09: TC_SW_CLAMP to XDA4, ENDSTOP TC clamp UP, SC_HF_025, 1450 mm, fuse signal
  - W-T-17: TC_SW_SEALED to MAIN_CTRL, ENDSTOP TC clamp SEALED, SC_HF_025, 1450 mm, fuse signal
  - W-T-12: TEC_TC to TC_NTC_BLOCK, SENSOR NTC TC_NTC_BLOCK, MC2_025, 450 mm, fuse signal
  - W-T-13: TEC_TC to TC_NTC_SINK, SENSOR NTC TC_NTC_SINK, MC2_025, 350 mm, fuse signal
  - W-T-14: TEC_TC to TC_NTC_A1, SENSOR NTC TC_NTC_A1, MC2_025, 550 mm, fuse signal
  - W-T-16: TC_NTC_LID to MAIN_CTRL, SENSOR NTC TC_NTC_LID, SC_HF_025, 1600 mm, fuse signal
- firmware: sections added gcode_button lid_parked, thermistor lid_ntc_10k, heater_generic lid_heater, verify_heater lid_heater, output_pin therm_enable, output_pin wd_heartbeat, output_pin beam_emitter, gcode_button lid_gate, gcode_button therm_mirror, gcode_button lid_closed, gcode_button clamp_sealed, manual_stepper tc_lid_slide, manual_stepper tc_lid_clamp; commands restored LH_HEARTBEAT, LH_THERM_ON, LH_THERM_OFF, LH_THERM_CLEAR, LH_THERM_STATUS, LH_LID_HEATER, LH_LID_HOME, LH_LID_CLOSE, LH_LID_OPEN, LH_LID_CLEAR, LH_LID_STATUS; no longer marked absent: thermocycler

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
- `cad/lh1/thermocycler/build.py`
- `cad/lh1/thermocycler/geometry.py`
