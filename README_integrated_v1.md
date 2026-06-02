# evrp_fragments integrated v1

Role: **derived output / canonical integrated package**.

This package combines the relevant staged refactor outputs into a single canonical `evrp_fragments` package.

## Source and prior-output roles

- Source: root-level `ev_fragmentsv3.py` remains the behavioural baseline.
- Prior output: `refactor_v1a_baseline_freeze` provides baseline-freeze evidence.
- Prior output: `refactor_v1b_fragment_pipeline` provides the fragment-pipeline split.
- Prior output: `refactor_v1c_master_model` provides master-network and master-model modules.
- Prior output: `refactor_v1d_callback_dp` provides callback, route tools and energy-DP modules.
- Prior output: `refactor_v1e_cleanup` provides cleanup and assurance rules.
- Derived output: this integrated `evrp_fragments` package.

## Important implementation note

The package is integrated into one namespace, but `read_instance` and `enumerate_base_paths` still delegate to the root-level legacy `ev_fragmentsv3.py` through `legacy_adapter.py`. This preserves behaviour while avoiding a risky reconstruction of those functions from partial search snippets. The next targeted cleanup should be to move `read_instance`, `step`, and `enumerate_base_paths` fully into `evrp_fragments/data/io.py` and `evrp_fragments/fragments/base_paths.py` after parity tests pass.

## Basic commands

```bash
python -m compileall evrp_fragments
python -m experiments.validate_integrated_helpers
```

To build the fragment/network/model summary without Gurobi:

```bash
python -m experiments.compare_integrated --source ev_fragmentsv3.py --instance instances/c101C6_2.txt --max-base-path-len 18 --k-max 1 --skip-gurobi
```

To run the solver after local validation:

```bash
python -m experiments.run_solver --source ev_fragmentsv3.py --instance instances/c101C6_2.txt --max-base-path-len 18 --k-max 1 --force-exact-k
```

Add `--use-callback` only after the no-callback model build path is validated.
