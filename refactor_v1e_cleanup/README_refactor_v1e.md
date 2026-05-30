# refactor_v1e_cleanup

Role: **derived output / cleanup and assurance layer**.

This is step 5 in the staged refactor. It does not change solver behaviour. It provides the cleanup, audit and assurance layer for the staged refactor packages created so far.

## Source / reference / prior output roles

- Source: root-level `ev_fragmentsv3.py` remains the behavioural baseline.
- Prior output: `refactor_v1a_baseline_freeze` froze the current behaviour.
- Prior output: `refactor_v1b_fragment_pipeline` isolated the fragment pipeline.
- Prior output: `refactor_v1c_master_model` isolated depot arcs, network construction and master-model construction.
- Prior output: `refactor_v1d_callback_dp` isolated route extraction, energy DP, callback cuts and diagnostics.
- Derived output: this `refactor_v1e_cleanup` package.

## Environment

- Python version: 3.12
- Key library versions: this cleanup layer only uses the Python standard library. `gurobipy==13.0.1` remains a solver dependency for the model/callback stages in your local environment.
- Target shape: script package plus audit functions.

## What this version does

1. Adds audit tooling to scan refactor packages for duplicate function names.
2. Adds package compile checks for `refactor_v1a` through `refactor_v1d` where those folders exist locally.
3. Adds a cleanup manifest documenting which modules are source, prior output, derived output, and finalised output candidates.
4. Adds a finalisation checklist for deciding when duplicated legacy helpers can be removed.
5. Adds a small smoke-test suite for the audit layer.

## What this version deliberately does not do

- It does not rewrite solver logic.
- It does not delete any existing file.
- It does not overwrite v1a, v1b, v1c or v1d.
- It does not wire v1d into v1c automatically.
- It does not claim numerical parity. Parity must be checked against your local artefacts.

## Main command

From the project root:

```bash
python -m refactor_v1e_cleanup.experiments.run_cleanup_audit
```

This writes:

```text
refactor_v1e_cleanup/artefacts/cleanup_audit.json
```

Optional explicit paths:

```bash
python -m refactor_v1e_cleanup.experiments.run_cleanup_audit --paths refactor_v1a_baseline_freeze refactor_v1b_fragment_pipeline refactor_v1c_master_model refactor_v1d_callback_dp
```

## Minimum validation

```bash
python -m compileall refactor_v1e_cleanup
python -m refactor_v1e_cleanup.experiments.run_cleanup_audit
```

If `pytest` is available:

```bash
python -m pytest refactor_v1e_cleanup/tests
```

## Acceptance criteria before treating the refactor as cleaned up

- v1a baseline copy and run artefacts exist.
- v1b fragment-pipeline counts are reconciled against v1a.
- v1c network and model build counts are reconciled against v1a.
- v1d route extraction and DP helper tests pass.
- Duplicate function names are either intentional transition adapters or are listed for removal.
- No package relies on a top-level import path for a nested package.
- Any removal of duplicated helpers happens only after parity is demonstrated.
