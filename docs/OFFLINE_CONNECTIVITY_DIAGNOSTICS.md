# Offline connectivity diagnostics

Role: **derived output / diagnostic script**.

This script diagnoses the master candidate network without requiring the callback solver loop to complete.

## Commands

Graph-only diagnosis:

```powershell
python -m experiments.diagnose_connectivity --source ev_fragmentsv3.py --instance instances/c101C6_2.txt --max-base-path-len 18
```

Diagnose the known bad incumbent:

```powershell
python -m experiments.diagnose_connectivity --source ev_fragmentsv3.py --instance instances/c101C6_2.txt --max-base-path-len 18 --chosen 4 27 67 83 97 107 117
```

## What to look for

- `non_depot_cyclic_scc_count`: if positive, the candidate graph contains non-depot cycles that can support disconnected subtours.
- `chosen_solution_diagnostic.disconnected_component_count`: for `k_max=1`, this should be zero in a valid solution.
- `nodes_not_reachable_from_depot` and `nodes_that_cannot_reach_depot`: these indicate structural network reachability issues before optimisation.

## Why this helps

The previous callback patches showed that the callback may be forced to separate many disconnected incumbents. This script checks whether the network itself contains cyclic state components that can explain that behaviour, without relying on Gurobi completing the run.
