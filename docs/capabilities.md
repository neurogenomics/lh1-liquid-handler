# Capabilities as designed

Every figure below is designed or calculated, not measured. Source: `cad/lh1/machine_family/out/<size>/report.json` unless stated.

## Sizes

| Size | W x D x H | With extraction fan | Mass (estimate) | Places with a station |
|---|---|---|---|---|
| MVP | 1,344 x 894 x 1,190 mm | no fan | about 114 kg | 5 |
| S | 1,344 x 894 x 1,190 mm | 1,456 x 894 x 1,190 mm | about 150 kg | 19 |
| M | 1,344 x 1,004 x 1,190 mm | 1,456 x 1,004 x 1,190 mm | about 160 kg | 22 |
| L | 1,344 x 1,114 x 1,190 mm | 1,456 x 1,114 x 1,190 mm | about 171 kg | 26 |


## Tools (the starter's set)

| Tool | What | Channels | Range | Tip | Dock bay |
|---|---|---|---|---|---|
| M50 | eight-channel 5-50 uL | 8 | 5-50 uL | p200 | 2 |
| M300 | eight-channel 30-300 uL | 8 | 30-300 uL | p300 | 3 |
| S10 | single 0.5-10 uL | 1 | 0.5-10 uL | p10 | 4 |
| S100 | single 10-100 uL | 1 | 10-100 uL | p200 | 7 (reserved) |
| S1000 | single 100-1,000 uL | 1 | 100-1000 uL | p1000 | 5 |
| M10 | eight-channel 0.5-10 uL | 8 | 0.5-10 uL | p10 | 1 |
| SYR | syringe dispense tool (10 mL, direct drive) | 1 |  | None | 6 |


## Starter deck

| Place | Station | What |
|---|---|---|
| F1 | TC1 | thermocycler TC-1 (double place with its lid park behind) |
| F2 | RES2 | reservoir place: 12-column reservoir (buffer A, bead clean-up, binding buffer; ethanol columns under the slotted cover ECV-12), plain insert |
| F3 | STACK | plate stack x4 (gripper only) |
| F4 | SHAKER | heater-shaker on SK-1, sunk 40 mm on a tray, set 8 mm outward (edge place) |
| A1 | TC1LID | TC-1 lid park |
| A2 | ATR3 | low ambient tube rack ATR-3 (24 x 1.5/2 mL), on a plain insert, not recessed (nothing under the deck) |
| A3 | CTB1 | cold tube block CTB-1 (48 x 1.5/2 mL) on a thermal module (module zone with CTB-2) |
| A4 | CTB2 | cold tube block CTB-2 (6 x 5, 4 x 15, 3 x 50 mL), recessed 60 |
| B0 | TPKS | tip parking TPK-S: random-access p300 comb for the M300, 3 groups (spot, entry, spot) = 6 eight-channel sets (48 tips), on a half insert in the left part of B0 beside the tip waste (pre-PCR tips; enables the waste-tip-per-well rules) |
| B1 | TIPWASTE | tip waste funnel + gate + VC-2 -> vertical chute -> crate in the drawer |
| B2 | STRIP | cold strip carrier on a thermal module |
| B3 | LW | liquid waste LW-1 (two streams) -> closed canisters in the drawer |
| B4 | MB1 | magnet MB-1 |
| G0 | HAZ | hazardous-liquid trough, lidded, ambient (a thermal module fits under it if 37 C is wanted) |
| G1 | P10 | tip frame p10 (gripper-refilled) |
| G2 | P200 | tip frame p200, recessed 13 so it is 51 mm above the seat |
| G3 | P300 | tip frame p300, recessed 15 so it is 57 mm above the seat |
| G4 | CPB1 | cold plate CPB-1 on a thermal module |
| IG | P1000R | p1000 rack (person-loaded) on the shelf, sunk 15 mm |


## Motion

- X travel 803.5 mm, Y travel 591.0 mm (starter).
- Deflection at the nozzle under a 120 N tip pick-up load: 0.358 mm against a limit of 0.5 mm (calculated).
- Tool exchange about 92.3 s (head report, calculated).

## Tips

- Tips held on the starter: 3,648 in 5 shelf towers and deck frames.

## Headroom

The design rule is to provide at least 1.5 times the largest known need. From the starter report:

| Capacity | Largest known need | Provided | Ratio | Meets rule |
|---|---|---|---|---|
| p300 tips held (frames + towers / rack) | 2160 | 2208 | 1.53 | yes |
| p200 tips held (frames + towers / rack) | 396 | 480 | 1.82 | yes |
| p10 tips held (frames + towers / rack) | 863 | 864 | 1.5 | yes |
| p1000 tips held (frames + towers / rack) | 21 | 96 | 6.86 | yes |
| tip waste crate (tips) | 2293 | 12413 | 5.41 | yes |
| pipette dock bays | 6 | 8 | 1.33 | yes |
| edge hand-off positions (open) | 2 | 0 | None | no |
| shelf tower slots | 5 | 5 | 1.0 | no |
| Z clearance over the tallest deck item | 33.0 | 50.0 | 1.52 | yes |
| Z clearance over the shelf towers | 33.0 | 36.0 | 1.09 | no |
| Z clearance over the dock (no tips) | 8.0 | 12.0 | 1.5 | yes |
| X travel end margin | 10.0 | 10.5 | 1.05 | no |
| Y travel end margin | 10.0 | 15.0 | 1.5 | yes |
| lateral load at which M1 horizontal reaches 0.4 mm | 120.0 | 187 | 1.56 | yes |
| pusher thrust (LGA35 at 0.7 A, 5 mm/s; sourcing curve INFERENCE) | 80.0 | 128.0 | 1.6 | yes |
| extraction air (fan running) | 10.6 | 30.0 | 2.83 | yes |
| 24 V and TEC supplies | None | None | None | no |
| tip parking (p300 tips parked at once) | 32 | 48 | 1.5 | yes |
| control-compartment airflow | 23.9 | 48.0 | 2.01 | yes |


## Installation

- Bench area needed 1,706 x 994 mm; bench at least 894 mm deep and rated for 232 kg.
