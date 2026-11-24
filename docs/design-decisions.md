# Design decisions

The hardware decisions behind LH-1, rewritten in general terms. Dates are when each was decided in 2026. None of them has been tested on a built machine.

## 1. Build it from bought parts wherever function allows (13 November)

Every custom board, custom part, unique fastener and hand-made cable joint must justify itself against a bought, plug-together alternative. An assembly step is acceptable only if a careful amateur with hand tools, a multimeter and written instructions can do it right first time, or if a mistake shows itself before it can do harm.

- Two custom circuit boards were dropped: bought drivers rated for 48 V, keyed pre-made leads and lever connectors replace a head interface board; bought Peltier controllers, relays and the motion controller's own heater channel replace a thermal controller board.
- Pre-made leads replace hand-crimped small contacts; fabrication processes cut to four (machining, sheet, sintered print, cut stock); go/no-go gauges or catalogue parts replace feeler settings where possible.
- What stays custom: frame brackets, deck, nests, tool plate, Z carriages, pipette cradles, and the thermocycler's block, lid and lid slide. No bought part holds a hand pipette on a gantry, and the bought thermocycler found costs more than the machine.
- What must not be simplified: the certified safety relays and contactors with their feedback loops, the coded door sensor, the thermostat chain, the stiffness of the tip pick-up force path, self-locking Z screws, fail-safe switch wiring.

## 2. Cameras check the deck and the liquid; they never steer a move (14 November)

The steppers know where the gantry is. What the machine does not know is whether the deck holds what the host believes, where each place really is after a home build, whether liquid entered the tips (hand pipettes have no pressure sensing) and whether the gripper moved a plate. So: a head camera looking down at markers (a tag for identity, two machined holes for position) on every place; a side camera at the tip gate against a backlight; an overview camera that records runs. A position more than 1.0 mm off nominal is a mechanical fault, not a correction. An unreadable picture holds the run. Each check is trusted only after a bench test with a pass criterion.

## 3. Weigh labware where it sits (14 November)

A plain nest on a bought single-point load cell can weigh a plate just before and after each dispense; 1 uL of water is 1 mg. A hobby-class amplifier should resolve a few milligrams, so a missed dispense of roughly 5-10 uL and above should be visible, single small additions not. A 0.1 mg balance is a deck station on the large size. Weight, like the cameras, can only hold a run. Weighing is left out of the starter and arrives with the large size's extra row.

## 4. Build in spare capacity (14 November)

The machine is sized for a family of work, not one task. Every capacity derived from the reference workload is provided at 1.5 times the largest need with at least two spare places; ranges cover 0.5-1,000 uL, 0.2 mL wells to 50 mL tubes and 4-99 C somewhere on the deck; every place is interchangeable; spare driver sockets, I/O and at least 30 % margin on each rail. A capability is added now if it is a normal part of bench work, cheap and needs no specialist; otherwise space, holes, cable route and connector are reserved for it.

## 5. Exchangeable tools and a modular deck

Two carriages each take any tool from a dock, so the machine can carry more pipette ranges than it has carriages. The tools are passive (no electronics on the tool). Every deck place follows one standard (130 x 108 x 5 mm insert, four M4, two 4 mm dowels, keyed connector at module places), so modules can be swapped and new ones designed by anyone.

## 6. One design in three sizes; the starter first (20 November)

The machine grows in depth only, one row of deck places at a time, from S to M to L. Width, height, gantry, head, dock, door, drawer, control compartment and electronics are fixed at the final size from the start, and the controllers and firmware are sized for L. An upgrade replaces the side rails, side beams, belts and chain tray, extends the base, roof and panels, and adds one deck strip with its stations. Sizes now: S 1,456 x 894 x 1,190 mm, M 1,456 x 1,004 x 1,190 mm, L 1,456 x 1,114 x 1,190 mm (with extraction fan).

## 7. Tool latch redesigned, coupling from bought parts, force audit on every mechanism (23 November)

An independent check found the earlier latch could not be opened by the machine: the force needed was far beyond the Z axis. So: holding and release are decoupled (an over-centre toggle on the carriage holds; a lift bar in the dock releases); the coupling is assembled from bought tooling balls, hardened vee seats and a retention knob screwed to shoulders, with nothing bonded, pressed or reamed; and every mechanism an axis drives has its force and travel computed against the thrust read from the electrical design, with a 1.5 margin rule. The dock lift brings an exchange to about 92.3 s (calculated). A later check found two blockers in this latch (see [open problems](open-problems.md), W1 and W2), and the audit is not yet a gate (W4).

## 8. The MVP first

The first build is the starter's own frame and enclosure with one carriage, one eight-channel pipette held by a manual clamp and a passive deck. It does not depend on the latch or dock, whose problems are unsolved. Each later stage adds hardware without throwing any away ([roadmap](roadmap.md)).
