"""Dynamic programme for charging-station insertion and route energy validation."""
from __future__ import annotations
import heapq
from ..config import DP_MAX_STATION_VISITS_PER_LEG, DP_MAX_LABELS_PER_NODE
from ..node_utils import is_station


def dp_leg_frontier_charge_to_full(data, u_sid, v_sid, t0, E0, max_station_visits=3):
    """
    Single skeleton leg u -> v, stations can be inserted.
    Returns a list of NONDOMINATED arrival labels at v.

    Each label is:
      (dist_leg, t_start_at_v, E_arr_at_v, path_tuple)

    Stations: always open (no TW).
    Destination v: TW enforced.
    Charging: if station visited, charge to full with time (CapE - E_arr)/rech.
    """
    sid_to_i = data['sid_to_i']
    dist_fn  = data['dist']
    tt_fn    = data['traveltime']
    en_fn    = data['energy']
    CapE     = data['CapE']
    rech     = data['rech']
    horizon  = data['horizon']

    v_i = sid_to_i[v_sid]
    v_ready = data['nodes'][v_i][6]
    v_due   = data['nodes'][v_i][7]

    station_sids = [data['nodes'][i][0] for i in data['S']]

    # PQ state: (dist_so_far, dep_time, -dep_energy, cur_sid, path_tuple, stations_used, visited_stations_frozenset)
    pq = []
    heapq.heappush(pq, (0.0, t0, -E0, u_sid, (u_sid,), 0, frozenset()))

    # nondominated labels per intermediate node (including stations): (dist, time, energy)
    best_at_node = {u_sid: [(0.0, t0, E0)]}

    # nondominated arrivals at v: (dist, t_start, E_arr, path)
    arrivals = []

    def dominates3(a, b, eps=1e-9):
        # a,b = (dist, time, energy)
        da, ta, ea = a
        db, tb, eb = b
        return (da <= db + eps) and (ta <= tb + eps) and (ea >= eb - eps)

    def keep_nondominated_list(L, newlab3):
        # keep nondominated in list of (d,t,e)
        for lab in L:
            if dominates3(lab, newlab3):
                return L, False
        out = [lab for lab in L if not dominates3(newlab3, lab)]
        out.append(newlab3)
        return out, True

    def dominates_arr(a, b, eps=1e-9):
        # a,b = (dist, t_start, E_arr, path)
        da, ta, ea, _ = a
        db, tb, eb, _ = b
        return (da <= db + eps) and (ta <= tb + eps) and (ea >= eb - eps)

    def keep_arrivals(arrivals, cand):
        for lab in arrivals:
            if dominates_arr(lab, cand):
                return arrivals
        out = [lab for lab in arrivals if not dominates_arr(cand, lab)]
        out.append(cand)
        # optional: sort for neatness
        out.sort(key=lambda x: (x[0], x[1], -x[2]))
        return out

    while pq:
        d_sofar, t_dep, negE, cur_sid, path, k, visitedS = heapq.heappop(pq)
        E_dep = -negE

        # Explore either go to v, or to a station (if budget remains)
        next_nodes = [v_sid]
        if k < max_station_visits:
            next_nodes += station_sids

        for nxt_sid in next_nodes:
            if nxt_sid == cur_sid:
                continue
            if nxt_sid in visitedS:
                continue

            ui = sid_to_i[cur_sid]
            ni = sid_to_i[nxt_sid]

            e_need = en_fn(ui, ni)
            if e_need > E_dep + 1e-9:
                continue

            t_arr = t_dep + tt_fn(ui, ni)
            E_arr = E_dep - e_need
            d_new = d_sofar + dist_fn(ui, ni)

            if nxt_sid == v_sid:
                t_start = max(t_arr, v_ready)
                if t_start > v_due + 1e-9:
                    continue
                cand = (d_new, t_start, E_arr, path + (nxt_sid,))
                arrivals = keep_arrivals(arrivals, cand)
                continue

            # station
            if t_arr > horizon + 1e-9:
                continue

            charge_time = (CapE - E_arr) / rech
            t_dep2 = t_arr + charge_time
            E_dep2 = CapE

            lab3 = (d_new, t_dep2, E_dep2)
            lst = best_at_node.get(nxt_sid, [])
            lst2, kept = keep_nondominated_list(lst, lab3)
            if not kept:
                continue
            best_at_node[nxt_sid] = lst2

            new_visitedS = visitedS | frozenset([nxt_sid])
            heapq.heappush(pq, (d_new, t_dep2, -E_dep2, nxt_sid, path + (nxt_sid,), k+1, new_visitedS))

    return arrivals  # may be empty

def dp_route_min_dist(data, skeleton_sids, t0=0.0, E0=None,
                  max_station_visits_per_leg=DP_MAX_STATION_VISITS_PER_LEG,
                  max_labels_per_node=DP_MAX_LABELS_PER_NODE):
    """
    Multi-leg DP over mandatory skeleton nodes (no stations in skeleton_sids).
    Returns:
      ok, best_dist, best_full_path_tuple, fail_index
    fail_index = i means failure on leg skeleton[i] -> skeleton[i+1].
    """
    if E0 is None:
        E0 = data['CapE']

    for sid in skeleton_sids:
        if is_station(data, sid):
            raise ValueError(f"skeleton contains station {sid}; filter stations out first")

    sid_to_i = data['sid_to_i']

    # label at mandatory node: (dist_so_far, dep_time, dep_energy, full_path_tuple)
    def dominates_label(a, b, eps=1e-9):
        da, ta, ea, _ = a
        db, tb, eb, _ = b
        return (da <= db + eps) and (ta <= tb + eps) and (ea >= eb - eps)

    def insert_label(L, newlab):
        for lab in L:
            if dominates_label(lab, newlab):
                return L
        out = [lab for lab in L if not dominates_label(newlab, lab)]
        out.append(newlab)
        out.sort(key=lambda x: (x[0], x[1], -x[2]))
        return out[:max_labels_per_node]

    start_sid = skeleton_sids[0]
    labels = {start_sid: [(0.0, t0, E0, (start_sid,))]}

    for i in range(len(skeleton_sids) - 1):
        u = skeleton_sids[i]
        v = skeleton_sids[i+1]

        next_labels_for_v = []

        for (d_sofar, t_dep, E_dep, path_sofar) in labels.get(u, []):
            arrivals = dp_leg_frontier_charge_to_full(
                data, u, v, t_dep, E_dep,
                max_station_visits=max_station_visits_per_leg
            )

            # each arrival is (d_leg, t_start_v, E_arr_v, path_leg)
            for (d_leg, t_start_v, E_arr_v, path_leg) in arrivals:
                # service at v
                v_i = sid_to_i[v]
                serv_v = data['nodes'][v_i][8]
                t_dep_v = t_start_v + serv_v
                E_dep_v = E_arr_v

                # stitch paths (avoid duplicating u)
                if path_sofar[-1] == path_leg[0]:
                    stitched = path_sofar + path_leg[1:]
                else:
                    stitched = path_sofar + path_leg

                newlab = (d_sofar + d_leg, t_dep_v, E_dep_v, stitched)
                next_labels_for_v.append(newlab)

        if not next_labels_for_v:
            return False, None, None, i  # failed on leg i

        L_v = []
        for lab in next_labels_for_v:
            L_v = insert_label(L_v, lab)
        labels[v] = L_v

    final_sid = skeleton_sids[-1]
    best = min(labels[final_sid], key=lambda x: (x[0], x[1], -x[2]))
    best_dist, best_t, best_E, best_path = best
    return True, best_dist, best_path, None
