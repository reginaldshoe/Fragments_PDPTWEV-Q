"""Route diagnostics."""
from __future__ import annotations
from typing import Any
from .route_tools import stitch_sid_sequence, strip_stations_from_sids
from .energy_dp import dp_route_min_dist

def state_transition_report(routes: list[list[int]], arc_by_id: dict[int, dict[str, Any]]) -> list[dict[str, Any]]:
    rows = []
    for ridx, route in enumerate(routes):
        for pos in range(len(route)-1):
            a = arc_by_id[route[pos]]; b = arc_by_id[route[pos+1]]
            rows.append({'route_index': ridx, 'position': pos, 'from_arc': route[pos], 'to_arc': route[pos+1], 'location_match': a['End'] == b['Start'], 'onboard_match': a['end_onboard'] == b['start_onboard'], 'end_onboard': a['end_onboard'], 'next_start_onboard': b['start_onboard']})
    return rows

def validate_routes_with_dp(data: dict[str, Any], routes: list[list[int]], arc_by_id: dict[int, dict[str, Any]]) -> list[dict[str, Any]]:
    out = []
    for ridx, route in enumerate(routes):
        sid_seq = stitch_sid_sequence(route, arc_by_id); skeleton = strip_stations_from_sids(data, sid_seq); ok, dist, path, fail = dp_route_min_dist(data, skeleton)
        out.append({'route_index': ridx, 'arc_ids': route, 'sid_sequence': sid_seq, 'skeleton': skeleton, 'dp_ok': ok, 'dp_distance': dist, 'dp_path': path, 'fail_index': fail})
    return out
