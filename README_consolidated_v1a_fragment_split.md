# evrp_fragments_consolidated_v1a_fragment_split

Role: **draft output v1a / mechanical modular split**.

## Purpose

This is the first modular split after `evrp_fragments_consolidated_v1` passed all benchmark checks.

It extracts the data and fragment-construction layer from:

```text
evrp_fragments_consolidated_v1/legacy_core.py
```

into:

```text
evrp_fragments_consolidated_v1/fragment_core.py
```

The master model, callback, DP and route tools remain in `legacy_core.py` for this stage.

## Apply

Unzip this overlay at the repository root.

## Step 1 — build the fragment layer

```powershell
python -m experiments.build_consolidated_v1a_fragments
```

This writes:

```text
evrp_fragments_consolidated_v1/fragment_core.py
MANIFEST_consolidated_v1a_fragment_core.json
```

## Step 2 — run the regression gate

```powershell
python -m experiments.regression_consolidated_v1a_fragments_c101C6_2
```

Expected target remains:

```text
objective: 376.07267704045523
theta: 65.73276248806948
selected_arc_ids: [4, 12, 27, 67, 91, 103]
```

## Step 3 — manually rerun your other benchmark instances

Use:

```powershell
python -m experiments.run_solver_consolidated_v1a --instance instances/YOUR_INSTANCE.txt --max-base-path-len 18 --k-max 1 --force-exact-k --use-callback
```

## Scope discipline

This stage only splits the fragment layer. Do not modify callback, DP, master model, route extraction or the model formulation in this stage.
