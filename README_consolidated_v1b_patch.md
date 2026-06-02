# evrp_fragments_consolidated_v1b patch

Role: **draft output v1b / builder patch**.

## What this fixes

The generated `legacy_core.py` failed to import on Windows because the generated module docstring included a raw Windows path, for example a path beginning with `C:\Users\...`.

Inside a Python string literal, `\U` is interpreted as the start of a Unicode escape. The v1b builder writes the source path in POSIX form inside the generated docstring, while preserving the real path in `SOURCE_PATH` using `repr`.

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
