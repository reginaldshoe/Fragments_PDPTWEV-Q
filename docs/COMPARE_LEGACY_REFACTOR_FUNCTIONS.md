# Compare legacy and integrated functions

Role: **derived output / offline diagnostic**.

This diagnostic compares function source code from `ev_fragmentsv3.py` against the current integrated package. It does not run Gurobi and does not require the callback to complete.

## Command

```powershell
python -m experiments.compare_legacy_refactor_functions --legacy ev_fragmentsv3.py
```

It writes:

```text
artefacts/legacy_refactor_function_diff.json
```

## What it compares

- `callback`
- `dp_leg_frontier_charge_to_full`
- `dp_route_min_dist`
- `extract_routes_from_solution`
- `stitch_sid_sequence`
- `route_distance_from_sids`
- `build_master_model`
- `raw_depot_arcs`
- `build_network`

For the integrated package, legacy `callback` is compared against `make_energy_callback` because the refactor uses a callback factory rather than a top-level callback.

## What to inspect first

Look for:

- functions found in legacy but not found in integrated;
- signature differences;
- large line-count differences;
- any `hash_match: true` values. If all are false, the implementation is not identical and we should not assume behavioural parity.

The full JSON contains unified diffs for each function.
