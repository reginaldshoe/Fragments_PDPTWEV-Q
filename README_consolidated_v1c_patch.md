# evrp_fragments_consolidated_v1c patch

Role: **draft output v1c / builder patch**.

## What this fixes

The previous patch still allowed a Unicode escape parsing error in `experiments/build_consolidated_v1.py`. This version removes backslash-heavy text from the module docstring and also keeps the generated `legacy_core.py` docstring path-neutral.

## Apply

Unzip this patch at the repository root, overwriting:

```text
experiments/build_consolidated_v1.py
```

Then rebuild the generated core:

```powershell
python -m experiments.build_consolidated_v1 --legacy ev_fragmentsv3.py
```

Then rerun:

```powershell
python -m experiments.regression_consolidated_v1_c101C6_2
```
