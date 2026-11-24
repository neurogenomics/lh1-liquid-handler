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

## Engineering standards

Follow Nathan's engineering standards (in `~/.codex/AGENTS.md` and `~/.claude/CLAUDE.md`):

- Start from a GitHub issue in a milestone; create both if missing.
- Branch `<issue#>-short-slug`; never commit straight to the default branch (it is protected).
- Open a small PR whose body says `Closes #N`; use the PR template.
- Tests and CI pass before merge; update README/docs and this file when commands or structure change.
- No secrets, `.env`, data, build outputs or files over 50 MB in git. Never force-push or rewrite history.

### Commands
- `pytest`
