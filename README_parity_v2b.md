# EVRP fragments parity v2b

Role: **draft output v2 / derived parity harness patch**.

## What changed from v2

This patch fixes the first-run issue where `--legacy ev_fragmentsv3` was treated as an exact filename. The loader now accepts either:

```powershell
--legacy ev_fragmentsv3
```

or:

```powershell
--legacy ev_fragmentsv3.py
```

## Commands

```powershell
python -m experiments.compare_legacy_parity_v2_functions --legacy ev_fragmentsv3
```

or:

```powershell
python -m experiments.compare_legacy_parity_v2_functions --legacy ev_fragmentsv3.py
```

Then:

```powershell
python -m experiments.run_solver_parity_v2 --source ev_fragmentsv3 --instance instances/c101C6_2.txt --max-base-path-len 18 --k-max 1 --force-exact-k
```

And with callback:

```powershell
python -m experiments.run_solver_parity_v2 --source ev_fragmentsv3 --instance instances/c101C6_2.txt --max-base-path-len 18 --k-max 1 --force-exact-k --use-callback
```
