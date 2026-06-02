# EVRP fragments parity v2c

Role: **draft output v3 / derived parity harness patch**.

## What changed

This patch fixes the `NameError: name 'data' is not defined` raised inside legacy `cust_locs` during `attach_metadata`.

Cause: the harness copied the AST-loaded namespace into a new dictionary, but loaded functions retained their original `__globals__`. In the monolithic legacy script, `data` existed as a module-level global. The patch synchronises harness variables back into each loaded function's global namespace before invoking the legacy pipeline.

## Commands

```powershell
python -m experiments.compare_legacy_parity_v2_functions --legacy ev_fragmentsv3.py
```

```powershell
python -m experiments.run_solver_parity_v2 --source ev_fragmentsv3.py --instance instances/c101C6_2.txt --max-base-path-len 18 --k-max 1 --force-exact-k
```

```powershell
python -m experiments.run_solver_parity_v2 --source ev_fragmentsv3.py --instance instances/c101C6_2.txt --max-base-path-len 18 --k-max 1 --force-exact-k --use-callback
```
