"""Route extraction and route utilities.

Patch v1: route extraction can now follow master-network state nodes (`u` -> `v`)
instead of only matching location SIDs (`Start` -> `End`). This matters because
multiple arcs can share the same physical start location while having different
onboard states.
"""
from __future__ import annotations
from typing import Any


def extract_routes_from_solution(
    chosen_arc_ids: list[int] | set[int] | tuple[int, ...],
    arc_by_id: dict[int, dict[str, Any]],
    depot_sid: str = 'D0',
    depot_u: int | None = None,
) -> list[list[int]]:
    """Convert selected arc ids into ordered routes.

    If `depot_u` is supplied, routes are reconstructed using master-network node
    ids (`u` and `v`). This is the preferred path for callback validation.

    If `depot_u` is omitted, the function falls back to physical location SIDs
    (`Start` and `End`) for backwards-compatible diagnostics.
    """
    if depot_u is not None:
        return _extract_routes_by_state_node(chosen_arc_ids, arc_by_id, depot_u)
    return _extract_routes_by_location(chosen_arc_ids, arc_by_id, depot_sid)


def _extract_routes_by_state_node(
    chosen_arc_ids: list[int] | set[int] | tuple[int, ...],
    arc_by_id: dict[int, dict[str, Any]],
    depot_u: int,
) -> list[list[int]]:
    remaining = set(chosen_arc_ids)
    out_by_u: dict[int, list[int]] = {}
    for aid in remaining:
        out_by_u.setdefault(arc_by_id[aid]['u'], []).append(aid)
    for aids in out_by_u.values():
        aids.sort()

    routes: list[list[int]] = []
    while True:
        starts = [aid for aid in out_by_u.get(depot_u, []) if aid in remaining]
        if not starts:
            break
        current = starts[0]
        route: list[int] = []
        seen: set[int] = set()
        while current in remaining and current not in seen:
            seen.add(current)
            route.append(current)
            remaining.remove(current)
            next_node = arc_by_id[current]['v']
            if next_node == depot_u:
                break
            options = [aid for aid in out_by_u.get(next_node, []) if aid in remaining]
            if not options:
                break
            current = options[0]
        routes.append(route)

    # Preserve disconnected selections for diagnostics rather than hiding them.
    while remaining:
        current = min(remaining)
        route = []
        seen = set()
        while current in remaining and current not in seen:
            seen.add(current)
            route.append(current)
            remaining.remove(current)
            next_node = arc_by_id[current]['v']
            options = [aid for aid in out_by_u.get(next_node, []) if aid in remaining]
            if not options:
                break
            current = options[0]
        routes.append(route)
    return routes


def _extract_routes_by_location(
    chosen_arc_ids: list[int] | set[int] | tuple[int, ...],
    arc_by_id: dict[int, dict[str, Any]],
    depot_sid: str = 'D0',
) -> list[list[int]]:
    remaining = set(chosen_arc_ids)
    start_map: dict[str, list[int]] = {}
    for aid in remaining:
        start_map.setdefault(arc_by_id[aid]['Start'], []).append(aid)
    for aids in start_map.values():
        aids.sort()

    routes: list[list[int]] = []
    while True:
        starts = [aid for aid in start_map.get(depot_sid, []) if aid in remaining]
        if not starts:
            break
        current = starts[0]
        route: list[int] = []
        seen: set[int] = set()
        while current in remaining and current not in seen:
            seen.add(current)
            route.append(current)
            remaining.remove(current)
            end = arc_by_id[current]['End']
            if end == depot_sid:
                break
            options = [aid for aid in start_map.get(end, []) if aid in remaining]
            if not options:
                break
            current = options[0]
        routes.append(route)

    while remaining:
        current = min(remaining)
        route = []
        while current in remaining:
            route.append(current)
            remaining.remove(current)
            end = arc_by_id[current]['End']
            options = [aid for aid, arc in arc_by_id.items() if aid in remaining and arc['Start'] == end]
            if not options:
                break
            current = min(options)
        routes.append(route)
    return routes


def stitch_sid_sequence(route: list[int], arc_by_id: dict[int, dict[str, Any]]) -> list[str]:
    sid_seq: list[str] = []
    for pos, aid in enumerate(route):
        seq = list(arc_by_id[aid]['seq'])
        if pos == 0:
            sid_seq = seq
        elif sid_seq and seq and sid_seq[-1] == seq[0]:
            sid_seq.extend(seq[1:])
        else:
            sid_seq.extend(seq)
    return sid_seq


def route_distance_from_sids(data: dict[str, Any], sid_seq: list[str] | tuple[str, ...]) -> float:
    return sum(data['dist'](data['sid_to_i'][u], data['sid_to_i'][v]) for u, v in zip(sid_seq, sid_seq[1:]))


def strip_stations_from_sids(data: dict[str, Any], sid_seq: list[str] | tuple[str, ...]) -> list[str]:
    out: list[str] = []
    for sid in sid_seq:
        idx = data['sid_to_i'].get(sid)
        if idx is None:
            out.append(sid)
            continue
        node = data['nodes'][idx]
        if not (node[1] == 'S' or node[2] == 'f'):
            out.append(sid)
    return out
