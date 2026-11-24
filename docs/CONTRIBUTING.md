# Contributing to LH-1

Thank you for helping. LH-1 is a design: nothing is built or measured yet. Contributions that make it easier to build, safer
or better checked are the most useful.

## How to work

1. **Open an issue first.** Say what you want to change and why. Link the files involved.
2. **One branch per issue**, named after it (for example `12-latch-open-stop`).
3. **Make the change in the generator, not the output.** CAD outputs (STEP, DXF, CSV, JSON) are written by the scripts in
   `cad/`; the wiring and bills of materials by `electronics/lh1/build_harness.py`; the firmware size files by
   `electronics/lh1/gen_size_files.py`. Change the script and regenerate.
4. **Run the checks** before you push:
   ```sh
   python tools/check_no_private_content.py
   python electronics/lh1/check.py
   python electronics/lh1/fw_render_check.py
   pytest -q tests
   ```
5. **Open a pull request** that says `Closes #<issue>`, what changed, what you ran and what it printed.

## Rules for content

- Describe the machine only. Do not add content about a specific assay, protocol, sample, reagent brand or laboratory.
  Generic techniques that describe a capability are fine (thermal cycling, magnetic-bead clean-up, serial dilution).
- Do not claim anything is built, working, measured or verified unless it has been, and say how.
- Label every figure: designed, calculated, assumed or measured.
- Do not add files over 40 MB. Regenerate large outputs instead.
- Safety-related changes (mains, safety relays, guards, heaters) need a review by a competent person before they are merged.

## Licence

No licence has been chosen yet: all rights reserved until one is added. By contributing you agree that your contribution
may be released under the licence the maintainers choose.
