# EVRP fragments consolidated baseline

Role: **finalised working baseline / canonical consolidated runner documentation**.

## Current canonical status

The current active solver entry points are:

```powershell
python -m experiments.run_solver_consolidated --instance instances/c101C6_2.txt --max-base-path-len 18 --k-max 1 --force-exact-k --use-callback
```

and:

```powershell
python -m experiments.regression_consolidated_c101C6_2
```

The canonical runner uses:

```text
evrp_fragments_consolidated_v1/pipeline.py
```

which aliases the passed v1e runtime path.

## Active runtime modules

The active runtime path is:

```text
evrp_fragments_consolidated_v1/fragment_core.py
evrp_fragments_consolidated_v1/master_core.py
evrp_fragments_consolidated_v1/callback_core.py
evrp_fragments_consolidated_v1/dependency_bridge.py
evrp_fragments_consolidated_v1/pipeline_v1e.py
evrp_fragments_consolidated_v1/pipeline.py
```

`legacy_core.py` remains in the repository as provenance/reference material, but it is not imported by the canonical pipeline.

## Reference regression target

The canonical `c101C6_2` regression target is:

```text
instance: instances/c101C6_2.txt
max_base_path_len: 18
k_max: 1
force_exact_k: true
use_callback: true
objective: 376.07267704045523
theta: 65.73276248806948
selected_arc_ids: [4, 12, 27, 67, 91, 103]
status: 2
```

The expected construction counts are:

```text
base_paths: 126
restricted_raw: 450
restricted_dedup: 210
restricted_meta: 210
restricted_undominated: 28
extended_raw: 112
extended_dedup: 112
extended_meta: 112
extended_undominated: 112
```

## Refactor history

The successful refactor path was:

```text
v1   fixed generated core
v1a  fragment layer split
v1b  master/network/model split
v1c  callback/DP/route-tool split
v1d  centralised global compatibility bridge
v1e  removed legacy_core from active runtime path
v1f  canonical runner overlay
v1g  documentation and archive markers
```

## Material roles

| Material | Role | Current treatment |
|---|---|---|
| ev_fragmentsv3.py | source / behavioural baseline | Keep frozen for provenance. |
| evrp_fragments_parity_v2c | reference harness / prior output | Keep as audit evidence. |
| evrp_fragments_consolidated_v1/legacy_core.py | generated reference / provenance | Keep; not active runtime. |
| fragment_core.py | active runtime / fragment layer | Canonical. |
| master_core.py | active runtime / master layer | Canonical. |
| callback_core.py | active runtime / callback-DP-route layer | Canonical. |
| dependency_bridge.py | active runtime / compatibility bridge | Canonical for now. |
| pipeline_v1e.py | active runtime implementation | Canonical implementation target. |
| pipeline.py | active runtime / public alias | Canonical import target. |
| older versioned pipelines/runners | prior output / audit trail | Keep until an explicit archive/delete step. |

## Scope discipline

Do not delete prior-output files until the repository is committed or archived.

Do not modify the following without a new regression gate:

```text
callback signature
DP label logic
route extraction
master formulation
fragment dictionary structure
benchmark settings
```

Future cleanup should proceed one small dependency class at a time.
