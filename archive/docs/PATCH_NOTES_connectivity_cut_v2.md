# Connectivity cut patch v2

Role: **derived output / targeted callback performance and correctness patch**.

## Why v1 was slow

The first disconnected-component patch used a no-good cut:

```python
sum(x[a] for a in selected_component) <= len(selected_component) - 1
```

That removes only the exact disconnected cycle currently found. The solver can then find many nearby disconnected cycles, which can make the callback loop slow.

## v2 fix

This patch adds a stronger component cut. For a selected component whose state-node set does not contain the depot, the callback requires at least one selected arc entering that component from outside:

```python
sum(x[a] for a in entering_arcs(component)) >= 1
```

Because flow conservation is already present, this is a much stronger connectivity cut than removing one exact cycle.

The callback also returns immediately after adding connectivity cuts, so it does not waste time running the energy DP on an incumbent that is already structurally invalid.

## Files replaced

```text
evrp_fragments/callback/lazy_cuts.py
experiments/run_solver.py
```
