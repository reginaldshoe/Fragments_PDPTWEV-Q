# evrp_fragments_consolidated_v1

Role: **draft output v1 / mechanically extracted package candidate**.

## Purpose

This overlay consolidates the benchmark-passing parity path into a fixed extracted package. It is designed to reduce runtime dependency on `ev_fragmentsv3.py` by mechanically generating:

```text
evrp_fragments_consolidated_v1/legacy_core.py
```

from the local behavioural source.

The generated package does **not** AST-load `ev_fragmentsv3.py` at solver runtime. The legacy file is only used once by the build step.

## Step 1 — unzip at repo root

Unzip this archive at the repository root. Do not unzip into a nested folder.

## Step 2 — build the extracted core

```powershell
python -m experiments.build_consolidated_v1 --legacy ev_fragmentsv3.py
```

This writes:

```text
evrp_fragments_consolidated_v1/legacy_core.py
MANIFEST_consolidated_v1_generated_core.json
```

## Step 3 — run the reference regression gate

```powershell
python -m experiments.regression_consolidated_v1_c101C6_2
```

Expected benchmark gate:

```text
source_sha256: e1db5b59971d852dc008ec6de1b9034ffb76b806092767222a95995d4f3950df
base_paths: 126
restricted_raw: 450
restricted_dedup: 210
restricted_meta: 210
restricted_undominated: 28
extended_raw: 112
extended_dedup: 112
extended_meta: 112
extended_undominated: 112
objective: 376.07267704045523
theta: 65.73276248806948
selected_arc_ids: [4, 12, 27, 67, 91, 103]
status: 2
```

## Step 4 — run another instance manually

```powershell
python -m experiments.run_solver_consolidated_v1 --instance instances/c101C6_2.txt --max-base-path-len 18 --k-max 1 --force-exact-k --use-callback
```

## Notes

This is not the final clean modular design. It is the first consolidation point: fixed extracted code, no runtime AST loading, preserved legacy behaviour.

Do not rename variables, change callback structure, introduce dataclasses, or alter the model until this package has passed all required benchmark checks.
