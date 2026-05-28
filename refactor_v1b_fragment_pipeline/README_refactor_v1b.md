# refactor_v1b_fragment_pipeline

Role: **derived output / fragment-pipeline refactor**.

This package is the second implementation step. It isolates the fragment-building pipeline while preserving the current solver as the behavioural baseline.

## Source / reference / prior output roles

- Source: root-level `ev_fragmentsv3.py` remains the behavioural baseline.
- Prior output: `refactor_v1a_baseline_freeze` is the frozen baseline mechanism.
- Draft output / superseded reference: existing first-pass refactor modules in the linked folder.
- Derived output: this `refactor_v1b_fragment_pipeline` package.

## Environment

- Python version: 3.12
- Key libraries: no pandas/numpy dependency is introduced in this step; `gurobipy` is not required for the fragment-only pipeline unless your legacy source imports it during prefix loading.
- Target shape: script package plus typed fragment-pipeline functions.

## What this version does

1. Introduces typed records for fragment-pipeline artefacts.
2. Moves fragment-specific logic into `evrp_fragments_v1b/fragments/` modules.
3. Adds `build_fragment_sets(...)`, which owns the restricted-fragment and extended-fragment pipeline.
4. Adds a legacy-prefix adapter so `read_instance`, `step`, and `enumerate_base_paths` can be loaded from the current root `ev_fragmentsv3.py` without executing the full solver block.
5. Adds a comparison script to generate fragment-pipeline summaries for checking against the v1a baseline.

## What this version deliberately does not do

- It does not refactor the master model.
- It does not refactor the callback or DP subproblem.
- It does not delete the legacy source.
- It does not change the Gurobi solve loop.
- It does not claim parity until you run the comparison locally against your working source.

## Main command

From the project root:

```bash
python -m refactor_v1b_fragment_pipeline.experiments.compare_fragment_pipeline   --source ev_fragmentsv3.py   --instance instances/c101C6_2.txt   --max-base-path-len 18
```

This writes:

```text
refactor_v1b_fragment_pipeline/artefacts/fragment_pipeline_summary.json
```

## Minimum validation before moving to v1c

```bash
python -m compileall refactor_v1b_fragment_pipeline
python -m refactor_v1b_fragment_pipeline.experiments.compare_fragment_pipeline --source ev_fragmentsv3.py --instance instances/c101C6_2.txt --max-base-path-len 18
```

Acceptance criteria:

- The package compiles.
- The comparison script runs without traceback.
- The summary JSON is created.
- Fragment counts look consistent with your v1a baseline output.

## Material assumption

`ev_fragmentsv3.py` has a clear script-level execution block beginning at the `instance = ...` assignment. The legacy-prefix adapter executes only the function-definition prefix before that line.
