# EVRP fragments parity v2

Role: **draft output v1 / derived parity harness**.

## Objective

This zip restarts the refactor around mechanical parity with the root-level `ev_fragmentsv3.py`. It does **not** patch the integrated solver and does **not** redesign the callback, DP, master model, depot arcs, network construction or route extraction.

## How to use

Unzip this archive at the repository root, so the repo contains:

```text
evrp_fragments_parity_v2/
experiments/run_solver_parity_v2.py
experiments/compare_legacy_parity_v2_functions.py
README_parity_v2.md
```

## Commands

First, check that the harness can load the relevant legacy functions:

```powershell
python -m experiments.compare_legacy_parity_v2_functions --legacy ev_fragmentsv3.py
```

Then run the construction/solve harness without the callback:

```powershell
python -m experiments.run_solver_parity_v2 --source ev_fragmentsv3.py --instance instances/c101C6_2.txt --max-base-path-len 18 --k-max 1 --force-exact-k
```

Then run with the callback:

```powershell
python -m experiments.run_solver_parity_v2 --source ev_fragmentsv3.py --instance instances/c101C6_2.txt --max-base-path-len 18 --k-max 1 --force-exact-k --use-callback
```

## Design note

The harness parses `ev_fragmentsv3.py` using `ast` and executes import statements, selected configuration assignments, and selected function definitions in a controlled namespace. It avoids executing the monolithic script's top-level preprocessing and optimisation block.

This is a parity harness, not a cleaned package. If it reproduces the legacy result, the next step is to move from dynamic extraction to fixed mechanically extracted modules, one function group at a time.

## Known limitation

If a legacy function depends on a top-level variable created only by the monolithic execution path, the harness may still expose that dependency. That is useful evidence: it identifies exactly which hidden global must be preserved or mechanically relocated before cleanup.
