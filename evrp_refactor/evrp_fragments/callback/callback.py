"""Gurobi callback orchestration for current energy checks and lazy cuts."""
from __future__ import annotations
import gurobipy as gp
from ..config import DEBUG, DEBUG_CALLBACK, DP_MAX_STATION_VISITS_PER_LEG, DP_MAX_LABELS_PER_NODE
from ..node_utils import is_station
from .energy_dp import dp_route_min_dist


def callback(model, where, x_vars, arcs, node_id, depot_u, data, theta, M):
    if where != gp.GRB.Callback.MIPSOL:
        return

    # Selected arcs
    xsol = model.cbGetSolution(x_vars)
    choose = [a_id for a_id, val in xsol.items() if val > 0.5]
    if not choose:
        return

    arc_by_id = {a['id']: a for a in arcs}

    # Build successor/predecessor on state nodes
    out_map = {}
    in_map = {}
    for a_id in choose:
        a = arcs[a_id]
        out_map.setdefault(a['u'], []).append(a_id)
        in_map.setdefault(a['v'], []).append(a_id)

    # find cycles that don't touch depot. track visited nodes from depot and see if any chosen arc lies outside.
    seen_nodes = set([depot_u])
    stack = [depot_u]
    while stack:
        n = stack.pop()
        for a_id in out_map.get(n, []):
            v = arcs[a_id]['v']
            if v not in seen_nodes:
                seen_nodes.add(v)
                stack.append(v)

    # any chosen arc with tail not reachable from depot implies a disconnected cycle or subtour
    bad_cycle_arcs = [a_id for a_id in choose if arcs[a_id]['u'] not in seen_nodes]
    if bad_cycle_arcs:
        # cut here b/c all arcs in that subtour can be selected together ie. |S| - 1
        model.cbLazy(gp.quicksum(x_vars[a_id] for a_id in bad_cycle_arcs) <= len(bad_cycle_arcs) - 1)
        return

    # extract routes starting from depot
    routes = []
    for a_id in out_map.get(depot_u, []):
        route = [a_id]
        cur = arcs[a_id]['v']
        # follow until depot (or dead end)
        while cur != depot_u:
            nxts = out_map.get(cur, [])
            if not nxts:
                break
            # integer solution should have 1 outgoing; if multiple, pick one (shouldn't happen)
            a_id2 = nxts[0]
            route.append(a_id2)
            cur = arcs[a_id2]['v']
            # safety against infinite loops
            if len(route) > len(choose) + 5:
                break
        routes.append(route)

    sid_to_i = data['sid_to_i']
    dist_fn = data['dist']

    # We'll compute DP-based feasibility/cost per route and aggregate delta_total across all routes
    delta_total = 0.0

    # Helper: build skeleton and edge->arc mapping from a route arc list
    def skeleton_and_cover(route_arc_ids):
        # Build the skeleton nodes in order (stations removed), and map each skeleton edge to the arc index that covers it
        cover_arc_index = []  # cover_arc_index[j] = index in route_arc_ids of arc covering edge j: skeleton[j]->skeleton[j+1]

        # Build per-arc skeleton subseqs and then align
        arc_skel = []
        for idx, a_idid in enumerate(route_arc_ids):
            seq = arcs[a_id]['seq']
            sk = [sid for sid in seq if not is_station(data, sid)]
            # sk should have at least start/end
            arc_skel.append((idx, sk))

        # Merge in order, tracking edge coverage by arc index
        # Start with first arc's skeleton
        if not arc_skel or not arc_skel[0][1]:
            return [], []

        skeleton = list(arc_skel[0][1])

        # edges in this first arc are covered by arc index 0
        for _ in range(len(skeleton) - 1):
            cover_arc_index.append(arc_skel[0][0])

        # Append subsequent arcs, avoiding duplicate join node
        for (aidx, sk) in arc_skel[1:]:
            if not sk:
                continue
            if skeleton and sk and skeleton[-1] == sk[0]:
                # append sk[1:]
                for sid in sk[1:]:
                    skeleton.append(sid)
                for _ in range(len(sk) - 1):
                    cover_arc_index.append(aidx)
            else:
                # discontinuity; still append
                for sid in sk:
                    skeleton.append(sid)
                for _ in range(len(sk) - 1):
                    cover_arc_index.append(aidx)

        return skeleton, cover_arc_index

    # --- process each route ---
    for route in routes:
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
        skeleton = [sid for sid in sid_seq if not is_station(data, sid)]
        if len(skeleton) < 2:
            continue

        # Run multi-leg DP: station insertion allowed, objective min distance
        ok, dp_dist, dp_full_path, fail_i = dp_route_min_dist(
            data, skeleton, t0=0.0, E0=data['CapE'],
            max_station_visits_per_leg=DP_MAX_STATION_VISITS_PER_LEG,
            max_labels_per_node=DP_MAX_LABELS_PER_NODE
        )

        if DEBUG_CALLBACK:
            print("DP path:", dp_full_path)
            print("DP distance:", dp_dist)
            print("Stations used:", [s for s in dp_full_path if is_station(data, s)])
            print("Num stations:", sum(is_station(data, s) for s in dp_full_path))

        if not ok:
            # For now: cut the whole route (safe, not minimal).
            # Once everything is stable, we can tighten to minimal prefix.
            model.cbLazy(gp.quicksum(x_vars[aid] for aid in route) <= len(route) - 1)
            return


        # Base cost consistent with master objective: sum of Df of chosen arcs on this route
        base_route_cost = 0.0
        for aid in route:
            base_route_cost += arcs[aid]['Df']

        delta = dp_dist - base_route_cost


        if DEBUG:
            print("CALLBACK: delta_total =", delta_total, "choose_arcs =", choose)

        if delta > 1e-6:
            delta_total += delta

    # --- Optimality cut for whole incumbent ---
    if delta_total > 1e-6:
        # Condition on selecting all chosen arcs
        S = choose
        model.cbLazy(theta >= delta_total - M*(len(S) - gp.quicksum(x_vars[a] for a in S)))
