# refactor_v1a_baseline_freeze

Role: **derived output / baseline freeze**.

This package is the first implementation step in the gradual refactor. It does **not** refactor the solver logic. Its purpose is to preserve and execute the current working `ev_fragmentsv3.py` behaviour as the baseline for later parity checks.

## Source material

- Source: `ev_fragmentsv3.py` in the root of your existing project.
- Prior output / superseded draft output: existing first-pass refactor modules in the linked folder.
- Derived output: this `refactor_v1a_baseline_freeze` package.

## Environment

Target environment:

- Python 3.12
- gurobipy 13.0.1, if that is the installed version in your current environment
- pandas 3.0.1 and numpy 2.4.2 are not required by this baseline runner unless used by your existing source
- Target shape: script package with a command-line runner

The exact dependency pins should be reconciled against your actual environment before thesis benchmarking.

## What this version does

1. Copies the current root `ev_fragmentsv3.py` into `legacy/ev_fragmentsv3_legacy.py`.
2. Runs the copied legacy script without modifying its algorithm.
3. Captures stdout, stderr, return code, and run metadata under `artefacts/`.
4. Provides a stable baseline command for later parity testing.

## What this version deliberately does not do

- It does not change fragment logic.
- It does not introduce dataclasses.
- It does not move functions into the final package structure.
- It does not delete duplicated functions.
- It does not attempt to fix any solver, callback, DP, or state-boundary issue.

## Usage from your project root

Place the `refactor_v1a_baseline_freeze` folder beside your existing root-level `ev_fragmentsv3.py`, then run:

```bash
python -m refactor_v1a_baseline_freeze.experiments.run_baseline
```

Optional arguments:

```bash
python -m refactor_v1a_baseline_freeze.experiments.run_baseline   --source ev_fragmentsv3.py   --legacy-copy refactor_v1a_baseline_freeze/legacy/ev_fragmentsv3_legacy.py   --artefact-dir refactor_v1a_baseline_freeze/artefacts
```

## Validation gates for this version

Run:

```bash
python -m compileall refactor_v1a_baseline_freeze
python -m refactor_v1a_baseline_freeze.experiments.run_baseline --copy-only
```

Then, if your Gurobi environment and instances folder are available:

```bash
python -m refactor_v1a_baseline_freeze.experiments.run_baseline
```

## Material assumptions

- The root-level `ev_fragmentsv3.py` is the behavioural baseline.
- The current script expects to be run from the project root, because it uses `Path.cwd() / "instances"`.
- Later refactor versions should compare against the captured artefacts from this baseline freeze.
