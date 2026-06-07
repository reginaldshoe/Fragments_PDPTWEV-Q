"""Energy DP for station insertion.

Patch v1: fixes heap ordering when two queue entries have the same priority.
The heap now includes a monotonically increasing integer tie-breaker so Python
never tries to compare EnergyLabel objects.
"""
from __future__ import annotations
from dataclasses import dataclass
from heapq import heappop, heappush
from itertools import count
from typing import Any

@dataclass(frozen=True)
class EnergyLabel:
    node_sid: str
    time: float
    energy: float
    distance: float
    path: tuple[str, ...]


def _is_station(data: dict[str, Any], sid: str) -> bool:
    node = data['nodes'][data['sid_to_i'][sid]]
    return node[1] == 'S' or node[2] == 'f'


def _ready(data: dict[str, Any], sid: str) -> float:
    return data['nodes'][data['sid_to_i'][sid]][6]


def _due(data: dict[str, Any], sid: str) -> float:
    return data['nodes'][data['sid_to_i'][sid]][7]


def _service(data: dict[str, Any], sid: str) -> float:
    if _is_station(data, sid):
        return data['CapE'] * data.get('rech', 0.0)
    return data['nodes'][data['sid_to_i'][sid]][8]


def _travel(data: dict[str, Any], a_sid: str, b_sid: str) -> tuple[float, float, float]:
    a = data['sid_to_i'][a_sid]
    b = data['sid_to_i'][b_sid]
    return data['traveltime'](a, b), data['energy'](a, b), data['dist'](a, b)


def _dominates(a: EnergyLabel, b: EnergyLabel) -> bool:
    return (
        a.time <= b.time + 1e-9
        and a.energy >= b.energy - 1e-9
        and a.distance <= b.distance + 1e-9
        and (
            a.time < b.time - 1e-9
            or a.energy > b.energy + 1e-9
            or a.distance < b.distance - 1e-9
        )
    )


def _add(labels: list[EnergyLabel], candidate: EnergyLabel, max_labels: int) -> list[EnergyLabel]:
    if any(_dominates(existing, candidate) for existing in labels):
        return labels
    out = [existing for existing in labels if not _dominates(candidate, existing)]
    out.append(candidate)
    out.sort(key=lambda label: (label.distance, label.time, -label.energy, label.path))
    return out[:max_labels]


def dp_leg_frontier_charge_to_full(
    data: dict[str, Any],
    u_sid: str,
    v_sid: str,
    t0: float,
    e0: float,
    max_station_visits: int = 6,
    max_labels_per_node: int = 500,
) -> list[EnergyLabel]:
    """Return nondominated arrival labels for one mandatory skeleton leg.

    The vehicle may insert charging stations before reaching ``v_sid``. Whenever
    a station is visited, this DP assumes charging to full before leaving it.
    """
    stations = [data['nodes'][idx][0] for idx in data.get('S', [])]
    start = EnergyLabel(u_sid, t0, e0, 0.0, (u_sid,))

    # Critical fix: include a strictly increasing tie-breaker. Without this,
    # heapq eventually compares EnergyLabel objects when distance/time/visits
    # ties occur, causing TypeError: '<' not supported between EnergyLabel.
    tie_breaker = count()
    queue: list[tuple[float, float, int, int, EnergyLabel]] = []
    heappush(queue, (0.0, t0, 0, next(tie_breaker), start))

    labels_by_state: dict[tuple[str, int], list[EnergyLabel]] = {(u_sid, 0): [start]}
    arrivals: list[EnergyLabel] = []

    while queue:
        _, _, station_visits, _, label = heappop(queue)
        for next_sid in stations + [v_sid]:
            if next_sid == label.node_sid:
                continue

            next_station_visits = station_visits + (1 if next_sid != v_sid else 0)
            if next_station_visits > max_station_visits:
                continue

            travel_time, energy_use, distance = _travel(data, label.node_sid, next_sid)
            if energy_use > label.energy + 1e-9:
                continue

            depart_time = label.time + _service(data, label.node_sid)
            arrival_time = max(_ready(data, next_sid), depart_time + travel_time)
            if arrival_time > _due(data, next_sid) + 1e-9:
                continue

            remaining_energy = label.energy - energy_use
            if _is_station(data, next_sid):
                remaining_energy = data['CapE']

            candidate = EnergyLabel(
                node_sid=next_sid,
                time=arrival_time,
                energy=remaining_energy,
                distance=label.distance + distance,
                path=label.path + (next_sid,),
            )

            if next_sid == v_sid:
                arrivals = _add(arrivals, candidate, max_labels_per_node)
                continue

            state = (next_sid, next_station_visits)
            old_labels = labels_by_state.get(state, [])
            new_labels = _add(old_labels, candidate, max_labels_per_node)
            if new_labels != old_labels:
                labels_by_state[state] = new_labels
                heappush(
                    queue,
                    (
                        candidate.distance,
                        candidate.time,
                        next_station_visits,
                        next(tie_breaker),
                        candidate,
                    ),
                )

    return arrivals


def dp_route_min_dist(
    data: dict[str, Any],
    skeleton_sids: list[str] | tuple[str, ...],
    t0: float = 0.0,
    e0: float | None = None,
    max_station_visits_per_leg: int = 6,
    max_labels_per_node: int = 500,
) -> tuple[bool, float | None, tuple[str, ...] | None, int | None]:
    """Run station-insertion DP across a mandatory skeleton route.

    Returns ``(ok, best_distance, full_path, fail_index)``.
    """
    if len(skeleton_sids) < 2:
        return True, 0.0, tuple(skeleton_sids), None
    if e0 is None:
        e0 = data['CapE']

    labels = [EnergyLabel(skeleton_sids[0], t0, e0, 0.0, (skeleton_sids[0],))]
    for leg_index, (u_sid, v_sid) in enumerate(zip(skeleton_sids, skeleton_sids[1:])):
        next_labels: list[EnergyLabel] = []
        for label in labels:
            arrivals = dp_leg_frontier_charge_to_full(
                data,
                u_sid,
                v_sid,
                label.time,
                label.energy,
                max_station_visits=max_station_visits_per_leg,
                max_labels_per_node=max_labels_per_node,
            )
            for arrival in arrivals:
                stitched_path = label.path + arrival.path[1:]
                merged = EnergyLabel(
                    node_sid=v_sid,
                    time=arrival.time,
                    energy=arrival.energy,
                    distance=label.distance + arrival.distance,
                    path=stitched_path,
                )
                next_labels = _add(next_labels, merged, max_labels_per_node)

        if not next_labels:
            return False, None, None, leg_index
        labels = next_labels

    best = min(labels, key=lambda label: (label.distance, label.time, -label.energy, label.path))
    return True, best.distance, best.path, None
