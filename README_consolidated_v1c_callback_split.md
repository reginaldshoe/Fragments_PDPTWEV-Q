# evrp_fragments_consolidated_v1c_callback_split

Role: **draft output v1c / mechanical modular split**.

## Purpose

This is the third modular split after `consolidated_v1b` master split passed.

It extracts the callback, DP and route-tool layer from:

```text
evrp_fragments_consolidated_v1/legacy_core.py
```

into:

```text
evrp_fragments_consolidated_v1/callback_core.py
```

The callback signature is preserved exactly. No callback factory, no DP rewrite and no formulation change are introduced.

## Apply

Unzip this overlay at the repository root.

## Step 1 — build the callback layer

```powershell
python -m experiments.build_consolidated_v1c_callback
```

This writes:

```text
evrp_fragments_consolidated_v1/callback_core.py
MANIFEST_consolidated_v1c_callback_core.json
```

## Step 2 — run the regression gate

```powershell
python -m experiments.regression_consolidated_v1c_callback_c101C6_2
```

Expected target remains:

```text
objective: 376.07267704045523
theta: 65.73276248806948
selected_arc_ids: [4, 12, 27, 67, 91, 103]
```

## Step 3 — manually rerun other benchmark instances

```powershell
python -m experiments.run_solver_consolidated_v1c --instance instances/YOUR_INSTANCE.txt --max-base-path-len 18 --k-max 1 --force-exact-k --use-callback
```

## Scope discipline

This stage only splits callback, DP and route tools mechanically. Do not convert the callback to a factory and do not alter the DP logic.
