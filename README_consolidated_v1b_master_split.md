# evrp_fragments_consolidated_v1b_master_split

Role: **draft output v1b / mechanical modular split**.

## Purpose

This is the second modular split after `consolidated_v1a` fragment split passed.

It extracts the master/network/model layer from:

```text
evrp_fragments_consolidated_v1/legacy_core.py
```

into:

```text
evrp_fragments_consolidated_v1/master_core.py
```

The callback, DP and route tools remain in `legacy_core.py` for this stage.

## Apply

Unzip this overlay at the repository root.

## Step 1 — build the master layer

```powershell
python -m experiments.build_consolidated_v1b_master
```

This writes:

```text
evrp_fragments_consolidated_v1/master_core.py
MANIFEST_consolidated_v1b_master_core.json
```

## Step 2 — run the regression gate

```powershell
python -m experiments.regression_consolidated_v1b_master_c101C6_2
```

Expected target remains:

```text
objective: 376.07267704045523
theta: 65.73276248806948
selected_arc_ids: [4, 12, 27, 67, 91, 103]
```

## Step 3 — manually rerun other benchmark instances

```powershell
python -m experiments.run_solver_consolidated_v1b --instance instances/YOUR_INSTANCE.txt --max-base-path-len 18 --k-max 1 --force-exact-k --use-callback
```

## Scope discipline

This stage only splits master/network/model construction. Do not modify callback, DP, route extraction or the model formulation in this stage.
