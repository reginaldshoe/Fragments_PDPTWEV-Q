"""Route extraction and route utility functions for selected master arcs."""
from __future__ import annotations
from typing import Any


def extract_routes_from_solution(
    chosen_arc_ids: list[int] | set[int] | tuple[int, ...],
    arc_by_id: dict[int, dict[str, Any]],
    depot_sid: str = 'D0',
) -> list[list[int]]:
    """Convert selected arc ids into ordered routes starting at the depot.

    This function follows selected arcs by matching each arc's `Start` to the
    previous arc's `End`. It is intentionally deterministic: if several arcs are
    available from the same start state, the lowest arc id is selected first.
    """
    remaining = set(chosen_arc_ids)
    start_map: dict[str, list[int]] = {}
    for aid in remaining:
        arc = arc_by_id[aid]
        start_map.setdefault(arc['Start'], []).append(aid)
    for aids in start_map.values():
        aids.sort()

    routes: list[list[int]] = []
    while True:
        depot_options = [aid for aid in start_map.get(depot_sid, []) if aid in remaining]
        if not depot_options:
            break
        current = depot_options[0]
        route: list[int] = []
        while current in remaining:
            route.append(current)
            remaining.remove(current)
            end_sid = arc_by_id[current]['End']
            if end_sid == depot_sid:
                break
            next_options = [aid for aid in start_map.get(end_sid, []) if aid in remaining]
            if not next_options:
                break
            current = next_options[0]
        routes.append(route)

    # Preserve any disconnected cycles/fragments for diagnostics rather than hiding them.
    while remaining:
        current = min(remaining)
        route = []
        seen = set()
        while current in remaining and current not in seen:
            seen.add(current)
            route.append(current)
            remaining.remove(current)
            end_sid = arc_by_id[current]['End']
            next_options = [aid for aid, arc in arc_by_id.items() if aid in remaining and arc['Start'] == end_sid]
            if not next_options:
                break
            current = min(next_options)
        routes.append(route)
    return routes


def stitch_sid_sequence(route: list[int], arc_by_id: dict[int, dict[str, Any]]) -> list[str]:
    """Stitch selected fragment sequences into a single SID sequence."""
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
    """Calculate route distance using the instance distance function."""
    total = 0.0
    sid_to_i = data['sid_to_i']
    for u_sid, v_sid in zip(sid_seq, sid_seq[1:]):
        total += data['dist'](sid_to_i[u_sid], sid_to_i[v_sid])
    return total


def strip_stations_from_sids(data: dict[str, Any], sid_seq: list[str] | tuple[str, ...]) -> list[str]:
    """Return a route skeleton without charging stations."""
    out = []
    for sid in sid_seq:
        idx = data['sid_to_i'].get(sid)
        if idx is None:
            out.append(sid)
            continue
        node = data['nodes'][idx]
        if not (node[1] == 'S' or node[2] == 'f'):
            out.append(sid)
    return out
