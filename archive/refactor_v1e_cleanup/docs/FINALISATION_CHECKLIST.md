# Finalisation checklist

Role: **derived output / assurance checklist**.

Use this checklist before deleting or merging duplicated functions.

## 1. Baseline integrity

- `refactor_v1a_baseline_freeze/legacy/ev_fragmentsv3_legacy.py` exists.
- `refactor_v1a_baseline_freeze/artefacts/baseline_metadata.json` exists.
- `source_sha256 == legacy_sha256` in the v1a metadata.

## 2. Fragment-pipeline parity

- v1b fragment counts have been generated.
- Restricted and extended fragment counts are reconciled against the v1a baseline console output.
- Any difference is recorded as either expected or requiring correction.

## 3. Master-model parity

- v1c network-only summary has been generated with `--skip-gurobi`.
- v1c Gurobi model build summary has been generated where Gurobi is available.
- Depot arc count, network arc count and network node count are stable across repeat runs.

## 4. Callback/DP helper validation

- v1d callback helper validation passes.
- Route extraction and SID stitching behave deterministically.
- State-transition diagnostics report location and onboard-state matches for selected adjacent arcs.

## 5. Cleanup rule

A duplicated helper can be removed only when:

1. its replacement exists;
2. imports point to the replacement;
3. the relevant parity check passes;
4. the old helper is not imported by any active runner;
5. the deletion is captured in a versioned commit or zip.
