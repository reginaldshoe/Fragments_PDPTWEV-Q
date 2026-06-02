# evrp_fragments_consolidated_v1e_remove_legacy_runtime

Role: **draft output v1e / remove legacy runtime dependency**.

## Purpose

This stage follows `consolidated_v1d cleanup globals regression gate passed`.

It keeps `legacy_core.py` in the repository as provenance/reference, but removes it from the active runtime path in the new pipeline:

```text
evrp_fragments_consolidated_v1/pipeline_v1e.py
```

Runtime now uses only:

```text
fragment_core.py
master_core.py
callback_core.py
dependency_bridge.py
```

No algorithm bodies, callback signatures, DP logic, route extraction or model formulation are changed.

## Apply

Unzip this overlay at the repository root.

## Regression gate

```powershell
python -m experiments.regression_consolidated_v1e_remove_legacy_runtime_c101C6_2
```

Expected target remains:

```text
objective: 376.07267704045523
theta: 65.73276248806948
selected_arc_ids: [4, 12, 27, 67, 91, 103]
```

## Manual benchmark rerun

```powershell
python -m experiments.run_solver_consolidated_v1e --instance instances/YOUR_INSTANCE.txt --max-base-path-len 18 --k-max 1 --force-exact-k --use-callback
```

## Scope discipline

Do not delete `legacy_core.py` yet. This stage only removes it from the new pipeline's runtime imports. Keep it for provenance until final archive/cleanup.
