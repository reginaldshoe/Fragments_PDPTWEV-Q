"""Post-solver solution inspection helpers.

These helpers are kept outside callback_core.py so the callback module stays
focused on the Gurobi callback, route-DP feasibility retesting, and lazy-cut
construction. They are not part of the active callback path.
"""

from __future__ import annotations

from .fragment_core import step

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

def route_distance_from_sids(data, sid_seq):
    dist = data['dist']         # helper from read_instance
    sid_to_i = data['sid_to_i'] # SID -> node index
    total = 0.0
    for u_sid, v_sid in zip(sid_seq, sid_seq[1:]):
        ui = sid_to_i[u_sid]
        vi = sid_to_i[v_sid]
        total += dist(ui, vi)
    return total

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

def simulate_route(data, sid_seq):
    st = (
        (0,),                 # path placeholder
        0,                    # time
        frozenset(),          # onboard
        data['CapE'],         # energy
        0.0,                  # cost
        frozenset(),          # visited pickups
        frozenset(),          # visited deliveries
        frozenset(),          # visited stations
        0,                    # last node
        0.0                   # slack
    )

    for sid in sid_seq[1:]:
        j = data['sid_to_i'][sid]
        st2, reason = step(data, st, j)
        if st2 is None:
            return False, sid, reason
        st = st2

    return True, None, None

# V4A2_CALLBACK_CORE_ENABLING_CLEANUP_APPLIED
