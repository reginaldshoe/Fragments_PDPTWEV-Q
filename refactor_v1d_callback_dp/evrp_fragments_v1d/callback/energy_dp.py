"""Energy dynamic programming for station insertion.

This module checks whether a mandatory skeleton route can be realised by
inserting charging stations. It is intentionally separated from diagnostics and
from the Gurobi callback wrapper.
"""
from __future__ import annotations
from dataclasses import dataclass
from heapq import heappop, heappush
from typing import Any

EPS = 1e-9
DEFAULT_MAX_STATION_VISITS_PER_LEG = 6
DEFAULT_MAX_LABELS_PER_NODE = 500

@dataclass(frozen=True)
class EnergyLabel:
    node_sid: str
    time: float
    energy: float
    distance: float
    path: tuple[str, ...]


def _is_station(data: dict[str, Any], sid: str) -> bool:
    idx = data['sid_to_i'].get(sid)
    if idx is None:
        return False
    node = data['nodes'][idx]
    return node[1] == 'S' or node[2] == 'f'


def _service_time(data: dict[str, Any], sid: str) -> float:
    idx = data['sid_to_i'][sid]
    if _is_station(data, sid):
        return data['CapE'] * data.get('rech', 0.0)
    return data['nodes'][idx][8]


def _ready(data: dict[str, Any], sid: str) -> float:
    return data['nodes'][data['sid_to_i'][sid]][6]


def _due(data: dict[str, Any], sid: str) -> float:
    return data['nodes'][data['sid_to_i'][sid]][7]


def _travel(data: dict[str, Any], a_sid: str, b_sid: str) -> tuple[float, float]:
    a = data['sid_to_i'][a_sid]
    b = data['sid_to_i'][b_sid]
    return data['traveltime'](a, b), data['energy'](a, b)


def _dist(data: dict[str, Any], a_sid: str, b_sid: str) -> float:
    return data['dist'](data['sid_to_i'][a_sid], data['sid_to_i'][b_sid])


def _dominates(a: EnergyLabel, b: EnergyLabel) -> bool:
    return (
        a.time <= b.time + EPS
        and a.energy >= b.energy - EPS
        and a.distance <= b.distance + EPS
        and (a.time < b.time - EPS or a.energy > b.energy + EPS or a.distance < b.distance - EPS)
    )


def _add_nondominated(labels: list[EnergyLabel], candidate: EnergyLabel, max_labels: int) -> list[EnergyLabel]:
    if any(_dominates(existing, candidate) for existing in labels):
        return labels
    labels = [existing for existing in labels if not _dominates(candidate, existing)]
    labels.append(candidate)
    labels.sort(key=lambda x: (x.distance, x.time, -x.energy))
    return labels[:max_labels]


def dp_leg_frontier_charge_to_full(
    data: dict[str, Any],
    u_sid: str,
    v_sid: str,
    t0: float,
    e0: float,
    max_station_visits: int = DEFAULT_MAX_STATION_VISITS_PER_LEG,
    max_labels_per_node: int = DEFAULT_MAX_LABELS_PER_NODE,
) -> list[EnergyLabel]:
    """Return nondominated arrival labels for one mandatory leg.

    The vehicle may insert charging stations before reaching `v_sid`. Whenever a
    station is visited, this DP assumes charging to full before leaving it.
    """
    cap_e = data['CapE']
    station_sids = [data['nodes'][idx][0] for idx in data.get('S', [])]
    candidates = station_sids + [v_sid]
    start = EnergyLabel(u_sid, t0, e0, 0.0, (u_sid,))
    queue: list[tuple[float, float, int, EnergyLabel]] = []
    heappush(queue, (0.0, t0, 0, start))
    labels_by_state: dict[tuple[str, int], list[EnergyLabel]] = {(u_sid, 0): [start]}
    arrivals: list[EnergyLabel] = []

    while queue:
        _, _, station_visits, label = heappop(queue)
        for next_sid in candidates:
            if next_sid == label.node_sid:
                continue
            next_station_visits = station_visits + (1 if next_sid != v_sid else 0)
            if next_station_visits > max_station_visits:
                continue
            travel_time, energy_use = _travel(data, label.node_sid, next_sid)
            if energy_use > label.energy + EPS:
                continue
            depart_time = label.time + _service_time(data, label.node_sid)
            arrival_time = max(_ready(data, next_sid), depart_time + travel_time)
            if arrival_time > _due(data, next_sid) + EPS:
                continue
            remaining_energy = label.energy - energy_use
            if _is_station(data, next_sid):
                remaining_energy = cap_e
            candidate = EnergyLabel(
                node_sid=next_sid,
                time=arrival_time,
                energy=remaining_energy,
                distance=label.distance + _dist(data, label.node_sid, next_sid),
                path=label.path + (next_sid,),
            )
            if next_sid == v_sid:
                arrivals = _add_nondominated(arrivals, candidate, max_labels_per_node)
                continue
            state = (next_sid, next_station_visits)
            current = labels_by_state.get(state, [])
            updated = _add_nondominated(current, candidate, max_labels_per_node)
            if len(updated) != len(current) or candidate in updated:
                labels_by_state[state] = updated
                heappush(queue, (candidate.distance, candidate.time, next_station_visits, candidate))
    return arrivals


def dp_route_min_dist(
    data: dict[str, Any],
    skeleton_sids: list[str] | tuple[str, ...],
    t0: float = 0.0,
    e0: float | None = None,
    max_station_visits_per_leg: int = DEFAULT_MAX_STATION_VISITS_PER_LEG,
    max_labels_per_node: int = DEFAULT_MAX_LABELS_PER_NODE,
) -> tuple[bool, float | None, tuple[str, ...] | None, int | None]:
    """Run station-insertion DP across a mandatory skeleton route.

    Returns `(ok, best_distance, full_path, fail_index)`.
    `fail_index` is the zero-based skeleton leg index that failed.
    """
    if len(skeleton_sids) < 2:
        return True, 0.0, tuple(skeleton_sids), None
    if e0 is None:
        e0 = data['CapE']

    labels = [EnergyLabel(skeleton_sids[0], t0, e0, 0.0, (skeleton_sids[0],))]
    for i, (u_sid, v_sid) in enumerate(zip(skeleton_sids, skeleton_sids[1:])):
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
                stitched = label.path + arrival.path[1:]
                merged = EnergyLabel(v_sid, arrival.time, arrival.energy, label.distance + arrival.distance, stitched)
                next_labels = _add_nondominated(next_labels, merged, max_labels_per_node)
        if not next_labels:
            return False, None, None, i
        labels = next_labels
    best = min(labels, key=lambda x: (x.distance, x.time, -x.energy))
    return True, best.distance, best.path, None
