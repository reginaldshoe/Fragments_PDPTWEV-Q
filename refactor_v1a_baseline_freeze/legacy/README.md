# legacy

This directory is the landing zone for the exact behavioural baseline copy:

```text
legacy/ev_fragmentsv3_legacy.py
```

The copy is created by `experiments/run_baseline.py` from the root-level `ev_fragmentsv3.py`.

This avoids editing or partially reconstructing the source file during the refactor. The copied file should be treated as **source-derived, frozen baseline material** once created.
