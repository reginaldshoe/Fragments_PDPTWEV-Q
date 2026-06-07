# Master connectivity-flow patch

Role: **derived output / targeted master-model patch**.

## Diagnosis supported by offline output

The offline diagnostic showed:

- the candidate network has 9 state nodes and 118 arcs;
- nodes 7 and 8 are not reachable from the depot;
- the known incumbent contains 3 selected components under `k_max=1`;
- 2 of those components are disconnected from the depot.

The issue is therefore not that the candidate graph contains isolated non-depot cyclic SCCs. The issue is that the master formulation allows selected pickup-covering cycles to be disconnected from the depot component.

## Fix

This patch adds a single-commodity connectivity flow to the master model.

Each selected arc can carry up to `number_of_pickups` units of artificial flow:

```python
f[a] <= number_of_pickups * x[a]
```

The depot supplies one unit for each pickup. Commodity is consumed on arcs that cover pickups, using the same pickup coverage convention as the master model.

This makes disconnected pickup-covering cycles infeasible in the base model rather than relying on the callback to cut them one by one.

## File replaced

```text
evrp_fragments/master/model.py
```

## Suggested test sequence

First run without callback:

```powershell
python -m experiments.run_solver --source ev_fragmentsv3.py --instance instances/c101C6_2.txt --max-base-path-len 18 --k-max 1 --force-exact-k --diagnose
```

Check that `disconnected_component_count` is 0.

Then run with callback:

```powershell
python -m experiments.run_solver --source ev_fragmentsv3.py --instance instances/c101C6_2.txt --max-base-path-len 18 --k-max 1 --force-exact-k --use-callback --diagnose
```
