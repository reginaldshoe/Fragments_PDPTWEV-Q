"""Callback and route diagnostics."""
from __future__ import annotations
from typing import Any
from .energy_dp import dp_route_min_dist
from .route_tools import stitch_sid_sequence, strip_stations_from_sids


def state_transition_report(routes: list[list[int]], arc_by_id: dict[int, dict[str, Any]]) -> list[dict[str, Any]]:
    """Report state-boundary consistency across adjacent selected arcs."""
    rows: list[dict[str, Any]] = []
    for route_index, route in enumerate(routes):
        for pos in range(len(route) - 1):
            a = arc_by_id[route[pos]]
            b = arc_by_id[route[pos + 1]]
            rows.append({
                'route_index': route_index,
                'position': pos,
                'from_arc': route[pos],
                'to_arc': route[pos + 1],
                'end_location': a['End'],
                'next_start_location': b['Start'],
                'end_onboard': a['end_onboard'],
                'next_start_onboard': b['start_onboard'],
                'location_match': a['End'] == b['Start'],
                'onboard_match': a['end_onboard'] == b['start_onboard'],
            })
    return rows


def validate_routes_with_dp(data: dict[str, Any], routes: list[list[int]], arc_by_id: dict[int, dict[str, Any]]) -> list[dict[str, Any]]:
    """Run energy DP validation for each selected route."""
    out = []
    for route_index, route in enumerate(routes):
        sid_seq = stitch_sid_sequence(route, arc_by_id)
        skeleton = strip_stations_from_sids(data, sid_seq)
        ok, best_dist, full_path, fail_index = dp_route_min_dist(data, skeleton)
        out.append({
            'route_index': route_index,
            'arc_ids': route,
            'sid_sequence': sid_seq,
            'skeleton': skeleton,
            'dp_ok': ok,
            'dp_distance': best_dist,
            'dp_path': full_path,
            'fail_index': fail_index,
        })
    return out
