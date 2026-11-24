"""Bench rig BR-1 (cad/lh1/bench_rig): wiring table and rig-only electrical parts.

Run: .venv/bin/python electronics/lh1/build_bench_rig.py
  -> electronics/lh1/bench_rig_wiring.csv   (same columns as wiring.csv)
  -> electronics/lh1/bench_rig_bom.csv      (electrical parts the rig needs that are NOT also LH-1 machine parts)
Controller: one BTT MMB CAN V2.0 (the head board type) on USB to a Raspberry Pi 4 (no CAN adapter needed on the rig);
pins as firmware/lh1/bench_rig.cfg, checked by check.py.  24 V SELV from a bought desktop supply with an IEC C14 inlet:
no mains wiring at all on the rig.  Lengths: estimated for the 760 x 250 mm base plate and 430 mm posts (BR-3), +20 %.
"""
import csv
from pathlib import Path

HERE = Path(__file__).resolve().parent
COLUMNS = ["cable_id", "from_device", "from_connector", "from_pins", "to_device", "to_connector", "to_pins", "signal", "conductors",
           "gauge_mm2", "shielded", "max_current_a", "length_mm_estimate", "route", "connector_part", "notes", "cable_type", "protected_by", "fw_pins"]
R = []


PREMADE = ("JST-XH", "JST-PH", "Amass XT30(2+2)")


def C(cid, a, ac, ap, b, bc, bp, sig, n, g, i, L, conn, notes="", ctype="", prot="", fw="", route="fixed", sh="no"):
    # ADR-003 / review section 4: every small-contact plug is a bought pre-made lead (no crimping on the rig)
    conn = " | ".join(("pre-made " + e.strip()) if e.strip().startswith(PREMADE) else e.strip() for e in conn.split("|"))
    R.append(dict(zip(COLUMNS, [cid, a, ac, ap, b, bc, bp, sig, n, g, sh, i, L, route, conn, notes, ctype, prot, fw])))


XH = "JST-XH"
C("BR-P-01", "BR_PSU", "DC plug", "+,-", "BR_FUSE", "in-line holder", "1,2", "24V rig supply", 2, 1.0, 3.5, 600, "barrel/screw adapter | Wago 221 lever splice",
  "desktop supply GST160A24 class (IEC C14 inlet, 24 V 6.67 A, double insulated): no mains wiring", "MC2_10", "supply limit")
C("BR-P-02", "BR_FUSE", "out", "+", "BR_ESTOP", "NC1", "11", "24V rig (to e-stop)", 1, 1.0, 3.5, 400, "Wago 221 lever splice | ferrule to screw terminal",
  "in-line 5 A fuse", "SC_10", "F_BR 5 A")
C("BR-P-03", "BR_ESTOP", "NC1-NC2 in series", "12-21,22", "BR_MMB", "XT30 (power+CAN)", "V+,GND", "24V rig switched", 2, 1.0, 3.5, 500,
  "ferrule to screw terminal | Amass XT30(2+2) 4-pin", "bought XT30(2+2) lead with ferrules at the e-stop end; both e-stop contacts in series in the + line (removes motor AND MMB logic: Klipper restart after every stop); CAN pins unused (USB)", "MC2_10", "F_BR 5 A")
C("BR-P-04", "BR_PI_PSU", "USB-C", "VBUS,GND", "BR_PI", "USB-C power", "VBUS,GND", "5V host", 2, 0.5, 3.0, 1500, "USB-C | USB-C", "official 5.1 V 3 A plug-in supply", "USBC_PWR", "supply limit")
C("BR-C-01", "BR_PI", "USB-A", "USB", "BR_MMB", "USB-C", "USB", "USB Klipper (rig)", 4, 0.2, 0.1, 1000, "USB-A 4-pin | USB-C", "rig uses USB, not CAN", "USB2_AC")
C("BR-C-02", "BR_BALANCE", "RS232/USB", "TX,RX,GND", "BR_PI", "USB-A", "USB", "USB/RS232 balance", 4, 0.2, 0.1, 2000, "balance port | USB-A 4-pin",
  "read by the bench host script (MT-SICS 'S' stable weight, 9600 8N1 typical), not by Klipper", "USB2_AB")
# motors (MMB EZ drivers)
C("BR-M-01", "BR_MMB", "M1 (EZ1)", "1A,1B,2A,2B", "BR_ZMOTOR", "motor", "A+,A-,B+,B-", "MOTOR Z", 4, 0.2, 0.9, 500, f"{XH} 4-pin | JST-PH 6-pin", "", "MOTOR_LEAD",
  "driver limit", "PD4,PD3,PD5,PB5")
C("BR-M-02", "BR_MMB", "M2 (EZ2)", "1A,1B,2A,2B", "BR_PLUNGER", "motor via FFC", "A+,A-,B+,B-", "MOTOR plunger (LGA351S12-A-UIAP-038 pusher at 0.7 A, phases doubled)", 8, 0.0105, 0.35, 600,
  "ZIF adapter 0.5 mm 20-way 20-pin | ZIF adapter 0.5 mm 20-way 20-pin", "the machine's carriage-B cable: 20-way 0.5 mm FFC, each phase on two ways (1-8), 0.35 A per conductor at 0.7 A: the rig proves the current and the flex life", "FFC_20_05MM", "driver limit", "PC9,PC8,PD2,PB4", "head_loop")
C("BR-M-03", "BR_MMB", "M3 (EZ3)", "1A,1B,2A,2B", "BR_XMOTOR", "motor", "A+,A-,B+,B-", "MOTOR X module", 4, 0.5, 1.5, 1200, f"{XH} 4-pin | JST-PH 6-pin",
  "4 x 0.5 mm2 screened extension to the belt module motor", "MC4_05_SH", "driver limit", "PC15,PC11,PC10,PB3", sh="yes")
# switches: all NC to GND with pull-ups (fail-safe: a broken wire reads 'triggered')
C("BR-S-01", "BR_MMB", "endstop PA15", "PA15,GND", "BR_ZHOME", "C,NC", "C,NC", "ENDSTOP Z home", 2, 0.25, 0.01, 500, f"{XH} 3-pin | solder + heat-shrink",
  "KW10 roller lever on the ZUC-1 cam bar, NC: actuated (open) from the normal top up to the eject top; broken wire reads triggered (fail-safe)", "MC2_025", "signal", "PA15")
C("BR-S-02", "BR_MMB", "endstop PD8", "PD8,GND", "BR_ZUP", "C,NO", "C,NO", "SIGNAL Z top (guard input)", 2, 0.25, 0.01, 500, f"{XH} 3-pin | solder + heat-shrink", "D2F roller lever on the ZUC-1 cam bar, wired NO: CLOSED from the normal top up to the eject top; broken wire reads 'not up' and blocks X moves (fail-safe)", "MC2_025", "signal", "PD8")
C("BR-S-03", "BR_MMB", "endstop PA10", "PA10,GND", "BR_PLUNGER_HOME", "C,NC", "C,NC", "ENDSTOP pusher retracted", 2, 0.0105, 0.01, 600,
  "ZIF adapter 0.5 mm 20-way 20-pin | ZIF adapter 0.5 mm 20-way 20-pin", "FFC ways 9-10, sealed retracted switch with factory lead, NC (fail-safe)", "FFC_20_05MM", "signal", "PA10", "head_loop")
C("BR-S-04", "BR_MMB", "sensor PC2", "PC2,GND", "BR_FORCE_SW", "C,NC", "C,NC", "ENDSTOP cradle force switch", 2, 0.0105, 0.01, 600,
  "ZIF adapter 0.5 mm 20-way 20-pin | ZIF adapter 0.5 mm 20-way 20-pin", "FFC ways 11-12; sealed changeover switch wired NC (fail-safe) although the rig report lists NO", "FFC_20_05MM", "signal", "PC2", "head_loop")
C("BR-S-05", "BR_MMB", "endstop PD9", "PD9,GND", "BR_XHOME", "C,NC", "C,NC", "ENDSTOP X module home", 2, 0.25, 0.01, 1200, f"{XH} 3-pin | solder + heat-shrink", "module limit switch NC", "MC2_025", "signal", "PD9")
# load cells: bridge -> HX711 (5 m max, 4-wire shielded) -> MMB header (VCC 3.3 V, GND, DOUT, SCK)
for i, (lc, hx, dout, sclk, where) in enumerate((("LC_RACK", "HX_RACK", "PC7", "PA9", "under the tip-rack nest"),
                                                  ("LC_COMB", "HX_COMB", "PB12", "PB10", "under the comb nest (LC-2 single-point platform cell, rev G3; was an S-type hanger)"),
                                                  ("LC_YOKE", "HX_YOKE", "PB1", "PC5", "HP-5 plunger yoke (moves with Z: through the FFC is NOT possible, use a 4-core high-flex lead in a service loop)"),
                                                  ("LC_EJECT", "HX_EJECT", "PC6", "PA8", "LC-4 single-point bar under the EJ-1 eject finger (BR-6/BR-7 on BR-2, rev G4; method B ejection force)"))):
    C(f"BR-L-{2 * i + 1:02d}", lc, "bridge lead", "E+,E-,S+,S-", hx, "E+,E-,A+,A-", "E+,E-,A+,A-", f"SENSOR load cell {lc}", 4, 0.14, 0.005,
      1500 if lc == "LC_YOKE" else 800, "moulded device lead | solder + heat-shrink", where, "MC4_025", "signal", sh="yes")
    C(f"BR-L-{2 * i + 2:02d}", hx, "VCC,GND,DT,SCK", "VCC,GND,DT,SCK", "BR_MMB", "2x7 header", f"3V3,GND,{dout},{sclk}", f"SIGNAL HX711 {hx}", 4, 0.14, 0.01, 400,
      "JST-XH 4-pin | board plug-in", "HX711 on the BR-8 electronics plate next to the MMB (rev G4), powered at 3.3 V so DT is a 3.3 V signal; RATE pin high (80 SPS). Klipper [load_cell] reads it on the MMB, not on the Pi GPIO as the bench report wiring text says", "MC4_025", "signal", f"{dout},{sclk}")

# ---- revision L: coupling (latch + four Hall code switches), two-bay dock switches, syringe prototype (spare MMB pins) ----
C("BR-R-01", "BR_MMB", "M4 (EZ4)", "1A,1B,2A,2B", "BR_SYRINGE", "motor", "A+,A-,B+,B-", "MOTOR direct-drive syringe pusher (LGA351S12-A-UIAP-038, as the machine)", 4, 0.2, 0.7, 600, f"{XH} 4-pin | JST-PH 6-pin",
  "", "MOTOR_LEAD", "driver limit", "PC13,PC12,PC14,PD6")
C("BR-R-02", "BR_MMB", "I2C3 header PC0", "PC0,GND", "BR_SYR_HOME", "C,NC", "C,NC", "ENDSTOP syringe retracted", 2, 0.25, 0.01, 600, f"{XH} 3-pin | Wago 221 lever splice",
  "NC; PC0 header access UNVERIFIED", "MC2_025", "signal", "PC0")
C("BR-R-04", "BR_MMB", "in_l_3..5, PA1, 5V, GND", "PB2,PB0,PC4,PA1,5V,GND", "BR_HALL", "4 x Hall", "O0,O1,O2,O3,V+,GND", "SIGNAL tool code (4 Hall switches)", 6, 0.14, 0.05, 800,
  "JST-XH 6-pin | Wago 221 lever splice", "four bought 6.5 mm cylindrical LEADED Hall switches in the rig coupling bracket (as the machine, CR-L7): open collector, magnet = low; MMB 5 V pin UNVERIFIED", "MC6_025_SH", "signal", "PB2,PB0,PC4,PA1", sh="yes")
C("BR-R-05", "BR_MMB", "servo PA0", "PA0,GND", "BR_BAY1", "C,NC", "C,NC", "ENDSTOP rig bay 1 occupied", 2, 0.25, 0.01, 900, f"{XH} 3-pin | Wago 221 lever splice", "NC", "MC2_025", "signal", "PA0")
C("BR-R-06", "BR_MMB", "RGB PC3", "PC3,GND", "BR_BAY2", "C,NC", "C,NC", "ENDSTOP rig bay 2 occupied", 2, 0.25, 0.01, 1000, f"{XH} 3-pin | Wago 221 lever splice", "NC", "MC2_025", "signal", "PC3")

# ---- ADR-014 (23 Nov 2026): second MMB 'rig2' on USB: the rig dock lift (the machine's DLB-1 drive), its DOWN slot sensor and
# OPEN-height switch, the three-state latch ladder (ADC PC2) and LC-5, the 500 N button cell under the lift pad (CT-0a / CT-0b)
C("BR-P-05", "BR_ESTOP", "NC1-NC2 in series (after)", "22", "BR_MMB2", "XT30 (power+CAN)", "V+,GND", "24V rig switched (second MMB)", 2, 1.0, 2.0, 500,
  "Wago 221 lever splice | Amass XT30(2+2) 4-pin", "spliced to BR-P-03 after the e-stop contacts: the e-stop removes both boards and every motor", "MC2_10", "F_BR 5 A")
C("BR-C-03", "BR_PI", "USB-A 2", "USB", "BR_MMB2", "USB-C", "USB", "USB Klipper rig2", 4, 0.2, 0.1, 1000, "USB-A 4-pin | USB-C", "second MMB as Klipper MCU 'rig2'", "USB2_AC")
C("BR-M-05", "BR_MMB2", "M1 (EZ1)", "1A,1B,2A,2B", "BR_LIFT", "motor", "A+,A-,B+,B-", "MOTOR rig dock lift (NEMA17 48 + Tr8x2, 1.3 A)", 4, 0.2, 1.3, 600, f"{XH} 4-pin | JST-PH 6-pin",
  "the machine's DLB-1 drive (CAD rig part DLB-L)", "MOTOR_LEAD", "driver limit", "rig2:PD4,rig2:PD3,rig2:PD5,rig2:PB5")
C("BR-S-06", "BR_MMB2", "endstop PA15 (V, GND, S)", "5V,GND,PA15", "BR_LIFT_HOME", "pre-wired lead (V, G, S)", "V,G,S", "ENDSTOP rig lift bar DOWN (slot sensor)", 3, 0.25, 0.02, 700,
  f"{XH} 3-pin | Wago 221 lever splice", "optical slot sensor (NPN, conducts with the flag in = home; broken wire = not home: '^!'); supply from the endstop header (5 V pin UNVERIFIED: the sensor takes 5-24 V)", "MC3_025", "signal", "rig2:PA15")
C("BR-S-07", "BR_MMB2", "endstop PA10", "PA10,GND", "BR_BAR_OPEN", "C,NO", "C,NO", "SIGNAL rig lift bar at the OPEN height", 2, 0.25, 0.01, 700, f"{XH} 3-pin | Wago 221 lever splice",
  "sealed switch made by the bar flag from rise + 2 mm, NO ('^!': broken wire = not at height)", "MC2_025", "signal", "rig2:PA10")
C("BR-R-03", "BR_MMB2", "sensor PC2 (ADC)", "PC2,GND", "BR_LATCH", "ladder (2 switches + 4 resistors, lever splices)", "SIG,RET", "SIGNAL LTG-1 latch (LOCKED / CLOSED-EMPTY / NOT CLOSED)", 2, 0.25, 0.01, 800, f"{XH} 3-pin | Wago 221 lever splice",
  "the machine's latch sensor: LOCK and EMPTY sealed switches + 1.0 k series, 22 k, 4.7 k, 1.5 k ladder in the lead (rig Z moves: service loop, not FFC)", "MC2_025", "signal", "rig2:PC2")
C("BR-L-09", "LC_LIFT", "bridge lead", "E+,E-,S+,S-", "HX_LIFT", "E+,E-,A+,A-", "E+,E-,A+,A-", "SENSOR load cell LC_LIFT", 4, 0.14, 0.005, 800, "moulded device lead | solder + heat-shrink",
  "LC-5: 500 N button cell under one rig dock lift pad (CAD rig part load_cell_button_500N; CT-0a release, CT-0b re-latch)", "MC4_025", "signal", sh="yes")
C("BR-L-10", "HX_LIFT", "VCC,GND,DT,SCK", "VCC,GND,DT,SCK", "BR_MMB2", "2x7 header", "3V3,GND,PC7,PA9", "SIGNAL HX711 HX_LIFT", 4, 0.14, 0.01, 400, "JST-XH 4-pin | board plug-in",
  "HX711 next to rig2, powered at 3.3 V, RATE high (80 SPS)", "MC4_025", "signal", "rig2:PC7,rig2:PA9")

RIG_ONLY = [
    ("belt linear module 500 mm", "belt-driven linear module, 500 mm stroke, NEMA17 motor fitted, MGN/HGR guided, carriage >= 80 x 80, with an NC limit switch at the home end", 1),
    ("24 V desktop supply", "enclosed 24 V 6.67 A (160 W) desktop supply with IEC C14 inlet, double insulated, IEC 62368-1 (Mean Well GST160A24 class) + mains lead", 1),
    ("in-line fuse 5 A", "in-line blade fuse holder for 1.0 mm2 + ATM 5 A fuses x 3", 1),
    ("rig e-stop", "22 mm e-stop, 2 NC positive-opening, in a yellow surface box; both contacts wired in series in the 24 V + line (DC-1 >= 6 A at 24 V)", 1),
    ("host computer", "Raspberry Pi 4 (4 GB) + official 5.1 V 3 A USB-C supply + 32 GB card (shared type with LH-1, separate unit)", 1),
    ("load cell, platform", "single-point 50 kg (500 N) platform load cell, IP65, 4-wire, for the tip-rack nest (LC-1)", 1),
    ("load cell, platform", "single-point 50 kg (500 N) platform load cell, IP65, 4-wire, under the comb nest (LC-2, rev G3: replaces the S-type hanger; the off-centre comb load needs a moment-compensated cell)", 1),
    ("load cell, button", "button 20 kg (200 N) for the pusher line (LC-3, ADR-014 CT-0d: the CAD rig lists the same 200 N cell (head report rig parts, bench_rig LC-3); also the HP-5 plunger yoke sweeps), with a 4-core high-flex lead", 1),
    ("CT-0d spring", "die spring about 12.5 N/mm, 12 mm working travel (about 150 N), in series with LC-3 in the pusher line: the LGA35 pushes at a set speed against a rising force", 1),
    ("load cell, bar 200 N", "single-point bar 20 kg (200 N), 4-wire, under the EJ-1 eject finger (LC-4, rev G4 force path BR-6 -> LC-4 -> BR-7 -> BR-2)", 1),
    ("HX711 amplifier", "HX711 24-bit board with RATE pin brought out (80 SPS), 2.6-5.5 V, shielded; one per load cell (LC-1 to LC-5); prefer a version with screw terminals for the load-cell side (generic listings only: NOT verified) so the moulded load-cell leads need no soldering", 5),
    ("second rig board (ADR-014)", "BigTreeTech MMB CAN V2.0 (the head board type) on USB as Klipper MCU 'rig2' + XT30(2+2) lead: rig dock lift, latch ladder, LC-5 (the 500 N cell and the lift motor are CAD rig parts)", 1),
    ("rig dock lift sensors (ADR-014)", "optical slot sensor EE-SX67x class with pre-wired lead (bar DOWN) + sealed subminiature switch with factory lead (bar OPEN height)", 1),
    ("rig latch sensor (ADR-014)", "second sealed subminiature switch (EMPTY) for the rig LTG-1 latch + ladder resistors 1.0 k, 22 k, 4.7 k, 2.0 k (1 % metal film) + heat-shrink", 1),
    ("rig coupling sensors (family)", "4 x 6.5 mm leaded Hall switch (as the machine) + latch, retracted, force and 2 bay switches (sealed subminiature, factory lead) + 1 syringe retracted switch", 1),
    ("rig pushers", "2 x Nanotec LGA351S12-A-UIAP-038 (the machine's pusher, ADR-014: plunger + direct-drive syringe) + maker's leads, run at 0.7 A; order with the machine's two (5 in stock, 21 Nov 2026)", 2),
    ("rig flat cable", "high-flex FFC 20-way 0.5 mm (the machine's carriage-B cable) 450 mm + 2 x ZIF-to-terminal adapters 0.5 mm 20-way", 1),
    ("head driver for the rig", "BigTreeTech EZ5160 Pro x 5 (Z, plunger, X module, syringe drive, and the rig dock lift on rig2) - the same driver as the machine so the rig tests it (ADR-003); read the sense-resistor marking", 5),
    ("pre-made leads (rig)", "pre-crimped JST-XH 3-pin and 4-pin leads with flying ends (switches, HX711), JST-PH6 motor lead, XT30(2+2) lead: no crimping on the rig", 1),
    ("balance", "6-place (0.001 mg) analytical micro-balance with RS232/USB (builder's own if available) + cable to the Pi", 1),
    ("evaporation trap", "ISO 8655-6 evaporation trap or lidded vial with a 2 mm hole", 2),
    ("cable", "4 x 0.14 mm2 screened (load cells, HX711 leads) 6 m; 2 x 0.25 mm2 3 m; 2 x 1.0 mm2 1.5 m; 4 x 0.5 mm2 screened 1.5 m", 1),
    ("USB cables", "USB A-C 1 m (Pi -> MMB), balance cable 2 m", 2),
]


def main():
    with open(HERE / "bench_rig_wiring.csv", "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=COLUMNS)
        w.writeheader()
        w.writerows(R)
    with open(HERE / "bench_rig_bom.csv", "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["item", "specification", "quantity", "note"])
        for it, sp, q in RIG_ONLY:
            w.writerow([it, sp, q, "rig only (not an LH-1 machine part)"])
    print(f"bench_rig_wiring.csv: {len(R)} cables; bench_rig_bom.csv: {len(RIG_ONLY)} rig-only lines")


if __name__ == "__main__":
    main()
