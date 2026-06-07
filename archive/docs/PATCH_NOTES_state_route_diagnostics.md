# State-route callback diagnostic patch

Role: **derived output / targeted diagnostic and callback patch**.

This patch replaces:

```text
evrp_fragments/callback/route_tools.py
evrp_fragments/callback/lazy_cuts.py
experiments/run_solver.py
```

## Suspected issue addressed

The previous callback reconstructed selected routes by matching physical locations (`Start` -> `End`). In the master model, however, flow conservation is enforced on state nodes (`u` -> `v`), where state includes both location and onboard set. Multiple arcs can share the same physical location while representing different onboard states. A location-only reconstruction can therefore validate the wrong skeleton route in the callback.

## Diagnostic output added

Run with `--diagnose` to print:

- chosen arc ids
- selected arc distance
- theta
- selected arc distance + theta
- DP total distance
- per-route SID sequence
- per-route skeleton
- per-route DP status and path

This is intended to show where the gap to the 376.07 benchmark is entering the calculation.
