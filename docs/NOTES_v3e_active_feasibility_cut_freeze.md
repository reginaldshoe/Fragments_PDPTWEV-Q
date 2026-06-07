# v3e active feasibility cut freeze note

## Role
This note records the accepted stopping point for the active guarded front-pruned feasibility-cut workstream.

## Current accepted implementation state
The accepted implementation state is the active guarded feasibility-cut version with summary-only output:

```text
v3d_active_feasibility_cut_summary_only
```

The active feasibility branch now uses a guarded front-pruned infeasible chain, `S_feas`, when the candidate is valid and smaller than the full selected route. The branch falls back to the full selected route if the guarded candidate is unavailable or fails the guard checks.

The active feasibility cut form is:

```python
model.cbLazy(gp.quicksum(x_vars[aid] for aid in S_feas) <= len(S_feas) - 1)
```

## Validation basis accepted for this stage
For the current non-queueing feasibility-cut strengthening stage, `instances/c103C16.txt` is treated as the primary and sufficient validation instance.

The rationale is:

- `c101C6_2` is too small or benign to reliably exercise the infeasible-route branch.
- `c103C16_2` and larger replicated instances are likely to interact with queueing effects that are not yet implemented in the subproblem.
- `c103C16` does exercise the infeasible-route branch and has completed with active guarded feasibility cuts enabled.

The accepted v3d validation evidence from `c103C16` was:

```text
[V3D-FEAS-SUMMARY] total=164 active=164 fallback=0
[V3D-FEAS-SUMMARY] cut_size_distribution={2: 55, 3: 46, 4: 13, 5: 35, 6: 15}
[V3D-FEAS-SUMMARY] full_size_distribution={7: 30, 8: 122, 9: 12}
[V3D-FEAS-SUMMARY] reason_distribution={'guarded_candidate_active': 164}
```

This means all recorded feasibility cuts in that run used the guarded candidate rather than the full-route fallback.

## Scope limitation
This freeze is limited to the non-queueing feasibility-cut strengthening stage.

It does not validate:

- queueing-aware subproblem behaviour;
- larger replicated instances that are likely to require queueing logic;
- global minimal infeasible subsets;
- theta or optimality-cut changes.

## Implementation discipline
From this point, the active feasibility-cut logic should be treated as frozen unless a regression failure or new queueing requirement requires reopening it.

Future work should avoid modifying:

- `_v3b_feas_find_front_pruned_candidate(...)`;
- the active `S_feas` lazy-cut branch;
- v3d summary recording;
- theta or optimality-cut logic.

## Recommended next stage
The next substantive modelling stage is queueing-aware feasibility/subproblem development. Until that work begins, this feasibility-cut strengthening stream should be considered complete for the current implementation stage.
