# evrp_fragments_consolidated_v1b_master_split_patch2

Role: **draft output v1b patch 2 / master split fix**.

## What this fixes

The first v1b master split failed because `master_core.raw_depot_arcs` still references fragment-layer helpers such as `compute_T_E_L`. In the monolithic legacy file those helpers were visible in the same module globals. After the split, they live in `fragment_core.py`.

This patch updates `pipeline_v1b.py` to inject the required fragment helper names into `master_core` globals before building the model.

## Apply

Unzip this patch at the repository root, overwriting:

```text
evrp_fragments_consolidated_v1/pipeline_v1b.py
```

You do not need to rebuild `master_core.py` unless you have changed `legacy_core.py`.

Then rerun:

```powershell
python -m experiments.regression_consolidated_v1b_master_c101C6_2
```
