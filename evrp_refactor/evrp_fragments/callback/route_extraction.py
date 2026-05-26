"""Utilities for reconstructing selected routes from master arc decisions."""
from __future__ import annotations


def extract_routes_from_solution(chosen_arc_ids, arc_by_id, depot_sid='D0'):
    # index chosen arcs by Start node
    start_map = {}
    for aid in chosen_arc_ids:
        a = arc_by_id[aid]
        start_map.setdefault(a['Start'], []).append(aid)

    routes = []

    # depot-starting arcs define routes
    for aid0 in start_map.get(depot_sid, []):
        route = [aid0]
        cur_aid = aid0
        cur_end = arc_by_id[cur_aid]['End']

        while cur_end != depot_sid:
            nexts = start_map.get(cur_end, [])
            if len(nexts) != 1:
                raise RuntimeError(
                    f"Ambiguous or missing continuation at {cur_end}: {nexts}"
                )
            cur_aid = nexts[0]
            route.append(cur_aid)
            cur_end = arc_by_id[cur_aid]['End']

        routes.append(route)

    return routes

def stitch_sid_sequence(route, arc_by_id):
    sid_seq = []
    for k, aid in enumerate(route):
        s = arc_by_id[aid]['seq']
        if k == 0:
            sid_seq = list(s)
        else:
            if sid_seq[-1] == s[0]:
                sid_seq.extend(s[1:])
            else:
                sid_seq.extend(s)
    return sid_seq
