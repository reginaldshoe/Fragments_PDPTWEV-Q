# refactor_v1c_master_model

Role: **derived output / master-model refactor**.

This is step 3 in the staged refactor. It extends the v1b fragment-pipeline work by adding explicit master-network and master-model modules.

## Source / reference / prior output roles

- Source: root-level `ev_fragmentsv3.py` remains the behavioural baseline.
- Prior output: `refactor_v1a_baseline_freeze` is the baseline-freeze package.
- Prior output: `refactor_v1b_fragment_pipeline` isolated the fragment pipeline.
- Derived output: this `refactor_v1c_master_model` package.

## Environment

- Python version: 3.12
- Key libraries: `gurobipy` is required only when building the model; fragment and network summaries can be compiled without executing Gurobi.
- Target shape: script package plus functions.

## What this version does

1. Keeps the v1b fragment-pipeline structure.
2. Adds `master/depot_arcs.py` for depot-start arcs.
3. Adds `master/network.py` for fragment-state network construction.
4. Adds `master/model.py` for Gurobi master-model construction.
5. Adds `experiments/compare_master_model.py` to build fragments, build the network, optionally build the model, and write a summary artefact.

## What this version deliberately does not do

- It does not move the callback or energy DP subproblem.
- It does not add lazy cuts.
- It does not run the full K sweep.
- It does not replace the current working solver.

## Main command

From your project root:

```bash
python -m refactor_v1c_master_model.experiments.compare_master_model --source ev_fragmentsv3.py --instance instances/c101C6_2.txt --max-base-path-len 18 --k-max 1 --skip-gurobi
```

Then, if Gurobi is available:

```bash
python -m refactor_v1c_master_model.experiments.compare_master_model --source ev_fragmentsv3.py --instance instances/c101C6_2.txt --max-base-path-len 18 --k-max 1
```

## Important design note

`read_instance`, `step`, and `enumerate_base_paths` are still loaded from the legacy source by prefix adapter. This is deliberate: v1c is the master-model step, not the base-path rewrite step.

The depot-arc implementation is explicit in this package. It creates depot-to-pickup arcs from `D0` with empty starting load and pickup onboard at the end, using either a direct leg or one charging station if needed. This should be checked against your v1a baseline before moving to callback/DP refactoring.
