# Bench rig BR-1

A one-axis rig to measure the risky functions before a machine is bought. It is a concept: **not built**. Files: `cad/lh1/bench_rig/` (CAD, `out/BR1_parts_list.csv`, `out/report.json`), `electronics/lh1/bench_rig_bom.csv` and `bench_rig_wiring.csv`, `firmware/lh1/bench_rig.cfg`.

Status in the report: bench rig concept for first measurements; NOT built; envelopes for bought rig items. Revision: mechanical G5 (13 Nov 2026): G4 + 48 mm NEMA17 Z motor; wiring statements agree with bench_rig.cfg / wiring table.

## What it measures

- plunger force vs travel per pipette (LC-3 in the HP-5 yoke + actuator steps)
- gravimetric dispense 0.5, 1, 2, 5, 10, 50, 300 uL (ISO 8655-6 method; 6-place balance below 10 uL)
- one-tip and eight-tip pick-up force (LC-1 under the rack) and strip force (LC-2 under the comb)
- thousands of unattended pick-up/strip cycles: comb mode with a bin

## Tests for the exchange head (run on the rig)

| Test | Method and pass criterion |
|---|---|
| FIRST - CT-0a latch release force | the rig carriage with the LTG-1 latch, a tool latched on a seated two-bay dock; the dock lift pushes the roller through the 500 N button cell under the pad: log force vs bar stroke; pass: peak <= lift thrust / 1.5 and within 20 % of the calculated profile (head report coupling.latch.table) |
| FIRST - CT-0b re-latch force and preload | bar lowered slowly with the stud seated: the springs must close the toggle 100 of 100 times with 0.67 x spring force (a third of the springs' force taken off by a hanging weight); preload measured with PLF-L and the S-cell (separation at the tool plate edge) for each of 6 knobs: pass >= 520 N |
| FIRST - CT-1 repeatability, 1,000 exchanges | two indicators (X, Y) on magnetic bases touch the ground D6 pin held on the nozzle (NRP-L); exchange tool in/out of the two-bay dock with the dock lift; pass <= 0.10 mm 3 sigma at the nozzle |
| FIRST - CT-0c comb entry with real tips | one TPK-S group (S E S) cut for the rig M300's measured nozzle row: 500 park/re-pick cycles of real p300 tips in random order: no strike, no collar left out of its pocket, no tip left on a strip |
| FIRST - CT-0d pusher force at 0.7 A | LGA35 at 0.7 A, 24 V, 2 / 5 / 8 mm/s against the 200 N button cell with the die spring in series in the pusher line: pass >= 120 N at 5 mm/s (80 N x 1.5) without lost steps; also log the homing stall current (<= 0.57 A) |
| CT-2 added deflection at 120 / 150 N | LJF-L pushes the nozzle pin sideways through the 500 N S-cell; pass <= 0.05 mm |
| CT-3 no lift-off at 2 x the working force | PLF-L pulls the tool plate down through the S-cell to 2 x eject/strip force; separation < 0.005 mm |
| CT-4 power-off pull and half-locks | motor power off, pull at 2 x; shim under one ball, wrong bay, debris, a constant-force spring removed: the latch must never read LOCKED with a bad seat it can detect (latch.SENSE: stud 0.12 mm or more off), and the tool never drops; record which shims read CLOSED-EMPTY / NOT CLOSED |
| CT-SYR syringe return spring | 10 mL syringe with the 1.5 x 20 x 75 return spring: plunger seal friction (pull-out force at 1 mm/s, wet), the plunger follows the pusher up 100 of 100 refills at 5 mm/s; pass: spring force out / measured friction + valve >= 1.5 |
| CT-5 3,000 exchanges | automatic cycling between the two bays with the dock lift; no undetected failure, <= 1 detected per 1,000 |
| CT-6 pipetting after exchange | gravimetric, S10 and M300, against the same pipette bolted directly |
| CT-7 force switch through the push-rod | load cell under the nozzle: trip within 15 % of the cradle's set force |
| CT-8 tool code | Hall switches against 1,000 insertions: no misread |
| CT-9 real forces | eight-tip pick-up, comb strip, ejector, 1,000 uL plunger |


## Procedures

| Procedure | What to do |
|---|---|
| plunger | step the plunger at 0.1 mm, log LC-3 force and step count to first and second stop, per pipette |
| gravimetric | dispense 0.5, 1, 2, 5, 10, 50, 300 uL into the trap vial on the balance, 20-100 replicates each, ISO 8655-6 corrections (Z factor, evaporation blank); dial at the build's fixed setting and at the volume, to test M11 |
| pickup_strip | pick one and eight tips over LC-1 with the force switch ending the stroke; strip on TW-4D over LC-2; log peak forces |
| hard_stop | per cradle (BR-B11): stop screw fully down; jog with a 0.5 mm feeler on the rack until pinched -> s_pinch; contact s_c = s_pinch + 0.5; jog to s_c - set gap (3.0 rig / 3.5 machine), raise the stop screw to the ledge, lock the nut; verify >= 2 mm gap with the axis on the stop; firmware limit = stop - 1 |


## Open items
- bought-part hole patterns are NOMINAL (module carriage and ends, load-cell ends, boards on BR-8): measure before machining
- load-cell deflections at rated load (0.3 mm) and safe overloads (150 %) are ASSUMED: set the 0.2 mm stop gaps after measuring
- the 24 V desktop supply stands on the bench (no fixing needed)
