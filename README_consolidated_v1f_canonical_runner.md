# evrp_fragments_consolidated_v1f_canonical_runner

Role: **draft output v1f / canonical runner overlay**.

## Purpose

This overlay follows the passing `consolidated_v1e remove legacy runtime` stage.

It creates stable canonical entry points that call the v1e runtime path without changing solver behaviour.

## Files added or replaced

```text
evrp_fragments_consolidated_v1/pipeline.py
experiments/run_solver_consolidated.py
experiments/regression_consolidated_c101C6_2.py
README_consolidated_v1f_canonical_runner.md
MANIFEST_consolidated_v1f_canonical_runner.json
```

## Runtime path

The canonical pipeline aliases:

```text
evrp_fragments_consolidated_v1/pipeline_v1e.py
```

So the active solver path remains:

```text
fragment_core.py
master_core.py
callback_core.py
dependency_bridge.py
pipeline_v1e.py
```

`legacy_core.py` remains available as provenance/reference but is not imported by the canonical pipeline.

## Regression gate

```powershell
python -m experiments.regression_consolidated_c101C6_2
```

Expected target remains:

```text
objective: 376.07267704045523
theta: 65.73276248806948
selected_arc_ids: [4, 12, 27, 67, 91, 103]
```

## Canonical manual run

```powershell
python -m experiments.run_solver_consolidated --instance instances/c101C6_2.txt --max-base-path-len 18 --k-max 1 --force-exact-k --use-callback
```

## Scope discipline

This stage only creates canonical aliases. It does not delete versioned pipelines, remove provenance files, add batch handling, or alter algorithm code.
