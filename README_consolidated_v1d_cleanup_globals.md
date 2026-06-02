# evrp_fragments_consolidated_v1d_cleanup_globals

Role: **draft output v1d / cleanup globals**.

## Purpose

This stage starts cleanup after the v1c callback split passed all benchmark checks.

It does **not** change algorithm bodies, callback structure, DP logic, route extraction, or the model formulation.

Instead, it centralises cross-module legacy global bindings into:

```text
evrp_fragments_consolidated_v1/dependency_bridge.py
```

The runner now uses:

```text
evrp_fragments_consolidated_v1/pipeline_v1d.py
```

rather than scattering helper injection and `__globals__` synchronisation throughout the pipeline.

## Apply

Unzip this overlay at the repository root.

## Regression gate

```powershell
python -m experiments.regression_consolidated_v1d_cleanup_globals_c101C6_2
```

Expected target remains:

```text
objective: 376.07267704045523
theta: 65.73276248806948
selected_arc_ids: [4, 12, 27, 67, 91, 103]
```

## Manual benchmark rerun

```powershell
python -m experiments.run_solver_consolidated_v1d --instance instances/YOUR_INSTANCE.txt --max-base-path-len 18 --k-max 1 --force-exact-k --use-callback
```

## Scope discipline

Do not remove `legacy_core.py` yet. Do not rename variables or rewrite callback/DP internals. This stage only centralises dependency bridging.
