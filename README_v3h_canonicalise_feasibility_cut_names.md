# v3h_canonicalise_feasibility_cut_names

## Role
Production naming cleanup for the active feasibility-cut callback helpers.

## Why this exists
The current `callback_core.py` still uses troubleshooting names such as `_v3a_feas_*`, `_v3b_feas_*`, `V3B_FEAS_MIN_CHAIN_SIZE`, and `[V3D-FEAS-SUMMARY]`. Those labels were useful while diagnosing the callback, but they are not meaningful in a working version of the code.

## What this overlay does
It renames the feasibility-cut helper functions and constants to production-style names, for example:

```text
_v3b_feas_find_front_pruned_candidate -> _find_front_pruned_infeasible_chain
_v3a_feas_test_chain -> _test_fragment_chain_feasibility
V3B_FEAS_MIN_CHAIN_SIZE -> FEASIBILITY_CUT_MIN_CHAIN_SIZE
V3C_MANUAL_FEAS_ACTIVE -> ACTIVE_FEASIBILITY_CUT_ENABLED
V3D_FEAS_SUMMARY -> FEASIBILITY_CUT_SUMMARY
_v3d_record_feas_cut -> _record_feasibility_cut_summary
```

It also renames runtime tags:

```text
[V3D-FEAS-SUMMARY] -> [FEAS-CUT-SUMMARY]
[V3C-FEAS-ACTIVE] -> [FEAS-CUT-ACTIVE]
[V3B-FEAS-DRY] -> [FEAS-CUT-DRY-RUN]
[V3A-FEAS] -> [FEAS-CUT-DIAGNOSTIC]
```

## What it does not do
It does not change the active feasibility-cut logic. It only renames helpers, constants, comments and output tags.

## Apply
Unzip at repository root, then run:

```powershell
python .\APPLY_v3h_canonicalise_feasibility_cut_names.py
```

## Validate

```powershell
python -m py_compile evrp_fragments/callback_core.py
python -m experiments.run_solver --instance instances/c103C16.txt --max-base-path-len 18 --k-max 1 --force-exact-k --use-callback
```

Expected summary tag after this patch:

```text
[FEAS-CUT-SUMMARY]
```

instead of:

```text
[V3D-FEAS-SUMMARY]
```
