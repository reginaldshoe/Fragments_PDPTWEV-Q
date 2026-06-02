# Subtour / disconnected component cut patch

Role: **derived output / targeted callback patch**.

## Issue diagnosed

The diagnostic output showed three selected components with `k_max=1`:

```text
[D0, ..., D0]
[C7, ..., C7]
[C12, ..., C12]
```

Only the first component is connected to the depot. The other two are disconnected cycles. This explains why the objective is too low relative to the benchmark: the master model is satisfying pickup coverage using cycles that do not pay the cost of connecting back to the depot route.

## Fix

The callback now adds a lazy no-good cut for any selected component that does not start at `depot_u` and end at `depot_u`:

```python
sum(x[a] for a in component) <= len(component) - 1
```

This forces Gurobi to eliminate disconnected components.

## Files replaced

```text
evrp_fragments/callback/lazy_cuts.py
experiments/run_solver.py
```

`run_solver.py` also adds explicit diagnostic fields:

- component_count
- disconnected_component_count
- starts_at_depot
- ends_at_depot
- connected_to_depot
