# Base-path dominance pruning

Role: **derived documentation / frozen method note**.

## Purpose

The base-path generator expands a depth-layer working set in `enumerate_base_paths`. On the larger `c103C16_2` instance, the unpruned working set grew to millions of states before the fragment build could complete. The active pruning method reduces the next depth layer locally before assigning `NewWork` back to `Work`.

## Active method

The active hook is local to `enumerate_base_paths` and is applied immediately before:

```python
Work = NewWork
```

The hook calls:

```python
NewWork, _bp_dom_stats = _prune_base_path_working_set(data, NewWork)
```

and prints:

```text
[BASE-PATH-DOMINANCE] source=local_newwork ...
```

This is the frozen base-path reduction path.

## State shape

The base-path state tuple used by `step(...)` and `enumerate_base_paths(...)` is:

```python
(path, phase, onboard, E, t_depart, seenP, seenD, seenS, deliv_count, distance)
```

The local dominance helper is tuple-aware for this state shape.

## Safety posture

Step-level dominance is disabled by default. The earlier step-level wrapper was useful diagnostically because it proved the dominance hook could affect enumeration, but it over-pruned the hard instance and is therefore not part of the frozen method.

The active method is layer-local. It waits until a complete `NewWork` layer has been generated, then prunes that layer before it becomes the next `Work` layer.

## Validation status

The current status is empirical rather than formally proven exact for all instances.

Observed validation outcomes in the working session:

- `c103C16_2` base-path enumeration previously timed out before the fragment build completed. With local `NewWork` dominance, the full fragment build completed and produced a tractable model build path.
- `c103C16` solved to the benchmark objective under the local dominance method.
- `c103C16_2` remains a queueing-required instance and is not expected to solve to the benchmark objective until queueing is implemented. It is used as a build/callback stress instance, not as an optimality validation case.

## Reporting language

Use cautious language:

> Local base-path working-set dominance made the larger queueing variant tractable at the fragment-build stage and preserved the benchmark objective on `c103C16`. The reduction is empirically validated for the tested benchmark path but is not yet a formal proof of exactness for all instances.

Avoid saying:

> The method preserves the exact full base-path enumeration on `c103C16_2`.

The unpruned `c103C16_2` enumeration did not complete, so that exact comparison is unavailable.
