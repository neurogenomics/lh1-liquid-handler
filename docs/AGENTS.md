# Instructions for AI agents working in this repository

- Start from an issue. Create or reference one before changing anything.
- Work on a branch per issue; never commit to `main` directly; open a pull request that says `Closes #<issue>`.
- Change generators, not their outputs. Regenerate the outputs with the scripts and commit both.
- Run, and report the output of: `python tools/check_no_private_content.py`, `python electronics/lh1/check.py`,
  `python electronics/lh1/fw_render_check.py`, `pytest -q tests`. Do not open a pull request while one fails without saying so.
- Never add assay-specific, protocol-specific, sample, reagent-brand, laboratory or personal content. The machine only.
- Never claim anything is built, working, measured or verified. Everything here is designed and simulated.
- Keep every number traceable to a file in the repository. Do not type figures from memory.
- Do not add files over 40 MB, secrets, absolute local paths or generated caches.
