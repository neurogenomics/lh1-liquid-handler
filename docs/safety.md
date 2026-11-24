# Safety

**No safety assessment has been done.** The design aims toward a stop category and performance level, but nothing has been verified. Do not power a machine built from these files until a competent person has assessed it.

## What needs a competent person

- Mains wiring: the inlet, supplies, mains terminal blocks, cord grips and the mains duct; insulation tests.
- Protective earth: the earth stud and every bond (frame, plates, compartment, deck, panels); continuity and the test regime.
- The safety relays and contactors: settings, feedback loops, proof tests of the e-stop and the door, and the stop category and performance level claim.
- The heater power path (thermocycler and thermal modules): thermostats, contactor coil chain, watchdog.
- The guard: enclosure, coded door sensor, and whether the arrangement suits the room and the people using it.

## Hazards the design addresses (on paper)

- Moving gantry and head: full enclosure; the door and the e-stop each remove motion power through safety relays and contactors; tools are held by the latch (or the MVP clamp) with power off.
- Heat: over-temperature cut-outs on every heater rail; host-loss watchdog; heated lid at about 105 C.
- Hazardous or volatile liquids: lidded troughs, a separate waste stream, an extraction fan with an airflow switch, and spark sources kept in the separate control compartment. A flammable solvent needs the extraction running.
- Pinch points at the dock lift bar and the thermocycler lid.

## Standards an assessor should consider

IEC/EN 61010-1, -2-081 and -2-010; EN ISO 13849-1 and -2; EN ISO 13850; EN ISO 14119; EN 60204-1 (as listed in `electronics/lh1/system.yaml`).

## Record in the design files

```text
{
 "status": "NOT ASSESSED. No electrical safety assessment has been done. The machine must not be used (beyond supervised, low-voltage bench bring-up of individual boards from a current-limited bench supply) until a competent person has assessed it.",
 "standards_to_consider": "IEC/EN 61010-1 (laboratory equipment) with 61010-2-081 (automatic lab equipment) and 61010-2-010 (heating); EN ISO 13849-1/-2 for the e-stop and door functions (target PL d, Cat 3); EN ISO 13850 (e-stop); EN ISO 14119 (interlock, defeat resistance); EN 60204-1 where machinery rules apply; EN 61326-1 (EMC); UK PSSR / Supply of Machinery and Electrical Equipment (Safety) Regulations for placing in service.",
 "must_cover": [
  "mains: inlet, fuse rating and breaking capacity, switch rating, terminal wiring to both supplies, PE conductor sizes, strain relief, creepage of the inlet installation, earth continuity (< 0.1 ohm inlet PE pin to every exposed conductive part incl. door, panels, deck, gantry), insulation resistance and earth-leakage tests (two supplies' Y-capacitors summed), PAT/in-service test regime",
  "SELV: both supplies' certification and their separation; decide SELV (0 V floating, as drawn) or PELV (0 V bonded to PE at one point: cable W-PE-0V reserved); touch current",
  "e-stop and door: PL verification (SISTEMA or equivalent) for SR1+KM1/KM2 and SR2+KM3/KM4, contactor DC-1/capacitive making ratings, feedback loops, reset behaviour (manual for e-stop; automatic for the door must be justified), stop category (0), defeat resistance of the door sensor, response time vs the 590 mm/s-class gantry speed and finger access, whether the single TC-1 relay plus TCB1 SAFE input is adequate for the thermal energy removal",
  "overcurrent: every fuse protects its cable (check.py), selectivity, short-circuit current of the supplies vs DC breaking capacity of fuses and contactors",
  "thermal: touch temperatures (TC-1 block 98 C, lid 105 C with 130 C cut-out, sinks), thermostat placement and ratings, fire enclosure for the heater and TEC circuits (61010-1 clause 9), cabinet temperature rise with the PSUs at load",
  "mechanical hazards on loss of power (self-locking Z and gripper verified by drop test), pinch points of the TC-1 lid (force-limited, break-beam), guards and labels",
  "EMC and functional: CAN/USB immunity with the steppers running, the Klipper shutdown on every door opening"
 ]
}
```
