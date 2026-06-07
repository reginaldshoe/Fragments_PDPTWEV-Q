# refactor_v1d_callback_dp

Role: **derived output / callback and energy-DP refactor overlay**.

This is step 4 in the staged refactor. It introduces explicit callback, energy dynamic programming, route extraction, route stitching and validation utilities.

## Source / reference / prior output roles

- Source: root-level `ev_fragmentsv3.py` remains the behavioural baseline.
- Prior output: `refactor_v1a_baseline_freeze` froze the current behaviour.
- Prior output: `refactor_v1b_fragment_pipeline` isolated the fragment pipeline.
- Prior output: `refactor_v1c_master_model` isolated depot arcs, network construction and master-model construction.
- Derived output: this `refactor_v1d_callback_dp` overlay.

## Environment

- Python version: 3.12
- Key libraries: `gurobipy` is only required when using the lazy-callback module inside a Gurobi optimisation run.
- Target shape: package overlay plus validation script.

## What this version does

1. Adds `callback/energy_dp.py` for station-insertion DP over route skeletons.
2. Adds `callback/route_tools.py` for selected-arc route extraction, stitching and distance calculation.
3. Adds `callback/lazy_cuts.py` for a Gurobi callback factory that checks selected routes and adds lazy constraints.
4. Adds `callback/diagnostics.py` for state-transition and DP validation summaries.
5. Adds `experiments/validate_callback_tools.py` for a non-Gurobi smoke check of route extraction and DP function imports.

## What this version deliberately does not do

- It does not rewrite fragment generation.
- It does not rewrite the master-model build from v1c.
- It does not run the full solver.
- It does not introduce queueing.

## How to place it

Place this folder beside the prior refactor packages:

```text
Fragments_PDPTWEV-Q/
  ev_fragmentsv3.py
  refactor_v1c_master_model/
  refactor_v1d_callback_dp/
```

## Basic validation

```bash
python -m compileall refactor_v1d_callback_dp
python -m refactor_v1d_callback_dp.experiments.validate_callback_tools
```

The second command only checks internal callback helpers. It does not need Gurobi.

## Integration note

The callback factory expects the same objects returned by the v1c master model: model, x variables, arcs, node_id, depot_u, arc_by_id, theta and big_m. It should be wired into the optimisation call only after v1c model-building parity is established.
