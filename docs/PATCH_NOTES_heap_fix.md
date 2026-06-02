# Callback DP heap fix

Role: **derived output / targeted patch**.

This patch replaces only:

```text
evrp_fragments/callback/energy_dp.py
```

It fixes the callback error:

```text
TypeError: '<' not supported between instances of 'EnergyLabel' and 'EnergyLabel'
```

## Cause

The DP priority queue used tuples ending with an `EnergyLabel` object. When two queue items tied on earlier tuple fields, Python tried to compare the `EnergyLabel` objects to break the tie. Dataclasses are not orderable by default, so `heapq` raised a `TypeError`.

## Fix

The heap tuple now includes a strictly increasing integer tie-breaker using `itertools.count()`:

```python
(distance, time, station_visits, tie_breaker, label)
```

No other file is changed.
