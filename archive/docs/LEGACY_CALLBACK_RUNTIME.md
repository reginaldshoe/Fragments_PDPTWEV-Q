# Legacy callback runtime diagnostic

Role: **derived output / parity diagnostic**.

This patch adds a diagnostic runner that uses the current integrated fragment/master pipeline but loads the callback, DP and helper functions directly from the local `ev_fragmentsv3.py` source using Python AST extraction. It avoids manual copy/paste.

## Files added

```text
evrp_fragments/callback/legacy_runtime.py
experiments/run_solver_legacy_callback.py
```

## Command

```powershell
python -m experiments.run_solver_legacy_callback --source ev_fragmentsv3.py --instance instances/c101C6_2.txt --max-base-path-len 18 --k-max 1 --force-exact-k --diagnose
```

## Interpretation

- If this matches the legacy objective, the issue is the refactored callback/DP implementation.
- If this still diverges or hangs, compare master functions next, because the callback is now being loaded from legacy source.

This runner does not replace the normal solver. It is only for diagnosis.
