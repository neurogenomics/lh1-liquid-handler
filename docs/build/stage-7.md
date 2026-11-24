# Stage 7: depth upgrades S -> M -> L (and weighing, which only L's row D carries)

Designed and simulated only. Nothing is built.

**Demonstrates:** larger batches (layouts for 48 and 96 samples)

**Test before the next stage:** re-commissioning per kit (kits.json steps)

## Open before it can proceed
- M and L still carry the old LIFO comb at C0 in the study layouts (ADR-014 consequences)

## What it adds

**S_to_M**: re_used_lines 145, replaced_lines 5, extended_lines 1, added_lines 33, added_custom_qty 39, added_bought_qty 5, removed_lines 3

**M_to_L**: re_used_lines 167, replaced_lines 5, extended_lines 1, added_lines 47, added_custom_qty 49, added_bought_qty 8, removed_lines 1

**note**: counts from out/kits.json (kit_S_to_M.csv, kit_M_to_L.csv); weighing WN-1 and the balance are at D4/D3 of L only, so the model puts weighing here rather than in stage 6

## Electrical additions

**depth upgrades S -> M -> L (kits out/kits_electrical.csv); weighing (balance D3, weighing nest D4) arrives only with L**

- firmware: no longer marked absent: weighing

Source: [upgrade_steps.json](../../electronics/lh1/out/mvp/upgrade_steps.json).

**Carried over:** all except the replaced lines

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
- `cad/lh1/machine_family/out/kits.json`
- `cad/lh1/size_family/build.py`
