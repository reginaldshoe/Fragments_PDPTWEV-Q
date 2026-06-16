"""Mechanically extracted callback and DP layer from monolith"""

from __future__ import annotations
from .fragment_core import is_station,step

# --- v5g queueing callback import ---
try:
    from . import queueing_core as qcore
except ImportError:  # pragma: no cover - fallback for direct script-style imports
    import evrp_fragments.queueing_core as qcore
# --- end v5g queueing callback import ---
import gurobipy as gp
from time import perf_counter
import heapq
from collections import Counter

CALLBACK_CORE_GENERATED = True

DP_MAX_STATION_VISITS_PER_LEG = 6
DP_MAX_LABELS_PER_NODE = 500

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

    return arrivals

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


FEASIBILITY_CUT_DIAGNOSTICS_ENABLED = False
FEASIBILITY_CUT_MAX_FRONT_PRUNE_TESTS = 50

def _fragment_arc_sid_sequence(arcs, aid):
    """Return the SID sequence stored on a selected fragment arc."""
    a = arcs[aid]
    if 'seq' in a:
        return list(a['seq'])
    if 'Seq' in a:
        return list(a['Seq'])
    raise KeyError(f"Arc {aid!r} has no 'seq' or 'Seq' field; keys={list(a.keys())}")

def _stitch_fragment_sid_sequence(route_arc_ids, arcs):
    """Stitch selected fragment-arc SID sequences into a route SID sequence."""
    sid_seq = []
    for k, aid in enumerate(route_arc_ids):
        s = _fragment_arc_sid_sequence(arcs, aid)
        if k == 0:
            sid_seq = list(s)
        elif sid_seq and s and sid_seq[-1] == s[0]:
            sid_seq.extend(s[1:])
        else:
            sid_seq.extend(s)
    return sid_seq

def _build_route_skeleton_and_cover(route_arc_ids, arcs, data):
    """Build the customer/depot skeleton and map each skeleton edge to its covering arc."""
    arc_skeletons = []
    for pos, aid in enumerate(route_arc_ids):
        seq = _fragment_arc_sid_sequence(arcs, aid)
        sk = [sid for sid in seq if not is_station(data, sid)]
        if sk:
            arc_skeletons.append((pos, sk))

    if not arc_skeletons:
        return [], []

    skeleton = list(arc_skeletons[0][1])
    cover_arc_index = [arc_skeletons[0][0]] * max(0, len(skeleton) - 1)

    for pos, sk in arc_skeletons[1:]:
        if not sk:
            continue
        if skeleton and skeleton[-1] == sk[0]:
            append_part = sk[1:]
        else:
            append_part = sk
        skeleton.extend(append_part)
        cover_arc_index.extend([pos] * max(0, len(append_part)))

    expected = max(0, len(skeleton) - 1)
    if len(cover_arc_index) > expected:
        cover_arc_index = cover_arc_index[:expected]
    elif len(cover_arc_index) < expected and arc_skeletons:
        cover_arc_index.extend([arc_skeletons[-1][0]] * (expected - len(cover_arc_index)))

    return skeleton, cover_arc_index

def _test_fragment_chain_feasibility(data, arcs, chain_arc_ids):
    """Retest a contiguous selected-arc chain with the route DP."""
    sid_seq = _stitch_fragment_sid_sequence(chain_arc_ids, arcs)
    skeleton = [sid for sid in sid_seq if not is_station(data, sid)]
    if len(skeleton) < 2:
        return True, None, skeleton
    ok, _dp_dist, _dp_full_path, fail_i = dp_route_min_dist(
        data,
        skeleton,
        t0=0.0,
        E0=data['CapE'],
        max_station_visits_per_leg=DP_MAX_STATION_VISITS_PER_LEG,
        max_labels_per_node=DP_MAX_LABELS_PER_NODE,
    )
    return ok, fail_i, skeleton



FEASIBILITY_CUT_VERBOSE_CANDIDATE_TESTS = False
FEASIBILITY_CUT_MIN_CHAIN_SIZE = 1

FEASIBILITY_CUT_PRUNING_SUMMARY = {
    "total": 0,
    "front_prune_success": 0,
    "front_prune_failure": 0,
    "route_size": Counter(),
    "prefix_size": Counter(),
    "candidate_size": Counter(),
    "rear_removed": Counter(),
    "front_removed": Counter(),
    "reason": Counter(),
}
SOLVER_BOUND_SUMMARY = {
    "snapshots": 0,
    "best_obj": None,
    "best_bound": None,
    "gap": None,
    "node_count": None,
    "runtime": None,
    "last_where": None,
}


def _record_pruning_summary(route_size, prefix_size, candidate_size, reason, success):
    s = FEASIBILITY_CUT_PRUNING_SUMMARY
    s["total"] += 1
    if success:
        s["front_prune_success"] += 1
    else:
        s["front_prune_failure"] += 1
    s["route_size"][int(route_size)] += 1
    if prefix_size is not None:
        s["prefix_size"][int(prefix_size)] += 1
        s["rear_removed"][max(0, int(route_size) - int(prefix_size))] += 1
    if candidate_size is not None:
        s["candidate_size"][int(candidate_size)] += 1
        if prefix_size is not None:
            s["front_removed"][max(0, int(prefix_size) - int(candidate_size))] += 1
    s["reason"][str(reason)] += 1


def _print_pruning_summary():
    s = FEASIBILITY_CUT_PRUNING_SUMMARY
    if s["total"] == 0:
        print("[FEAS-CUT-PRUNING-SUMMARY] total=0")
        return
    print("[FEAS-CUT-PRUNING-SUMMARY] total=" + str(s["total"]) + " front_success=" + str(s["front_prune_success"]) + " front_failure=" + str(s["front_prune_failure"]))
    print("[FEAS-CUT-PRUNING-SUMMARY] route_size_distribution=" + str(dict(sorted(s["route_size"].items()))))
    print("[FEAS-CUT-PRUNING-SUMMARY] prefix_size_distribution=" + str(dict(sorted(s["prefix_size"].items()))))
    print("[FEAS-CUT-PRUNING-SUMMARY] candidate_size_distribution=" + str(dict(sorted(s["candidate_size"].items()))))
    print("[FEAS-CUT-PRUNING-SUMMARY] rear_removed_distribution=" + str(dict(sorted(s["rear_removed"].items()))))
    print("[FEAS-CUT-PRUNING-SUMMARY] front_removed_distribution=" + str(dict(sorted(s["front_removed"].items()))))
    print("[FEAS-CUT-PRUNING-SUMMARY] reason_distribution=" + str(dict(s["reason"].most_common())))


def _safe_cb_get(model, what):
    try:
        return model.cbGet(what)
    except Exception:
        return None


def _record_solver_bound_snapshot(model, where):
    s = SOLVER_BOUND_SUMMARY
    try:
        if where == gp.GRB.Callback.MIP:
            best = _safe_cb_get(model, gp.GRB.Callback.MIP_OBJBST)
            bound = _safe_cb_get(model, gp.GRB.Callback.MIP_OBJBND)
            node_count = _safe_cb_get(model, gp.GRB.Callback.MIP_NODCNT)
            runtime = _safe_cb_get(model, gp.GRB.Callback.RUNTIME)
        elif where == gp.GRB.Callback.MIPSOL:
            best = _safe_cb_get(model, gp.GRB.Callback.MIPSOL_OBJBST)
            bound = _safe_cb_get(model, gp.GRB.Callback.MIPSOL_OBJBND)
            node_count = None
            runtime = _safe_cb_get(model, gp.GRB.Callback.RUNTIME)
        else:
            return
    except Exception:
        return
    s["snapshots"] += 1
    s["last_where"] = int(where)
    if best is not None:
        s["best_obj"] = float(best)
    if bound is not None:
        s["best_bound"] = float(bound)
    if node_count is not None:
        s["node_count"] = float(node_count)
    if runtime is not None:
        s["runtime"] = float(runtime)
    best_obj = s.get("best_obj")
    best_bound = s.get("best_bound")
    if best_obj is not None and best_bound is not None and abs(best_obj) > 1e-12:
        s["gap"] = abs(best_obj - best_bound) / abs(best_obj)


def _print_solver_bound_summary():
    s = SOLVER_BOUND_SUMMARY
    print("[SOLVER-BOUNDS-SUMMARY] snapshots=" + str(s["snapshots"]) + " best_obj=" + str(s["best_obj"]) + " best_bound=" + str(s["best_bound"]) + " gap=" + str(s["gap"]) + " node_count=" + str(s["node_count"]) + " runtime=" + str(s["runtime"]))




# Diagnostic-only subtour/cycle lazy-cut instrumentation.
# Active callback behaviour is unchanged: subtour cuts are still added in the same branch.
SUBTOUR_CUT_SUMMARY = {
    "mipsol_callbacks": 0,
    "subtour_cuts": 0,
    "returned_before_route_dp": 0,
    "reached_route_dp": 0,
    "routes_checked": 0,
    "bad_cycle_size": Counter(),
    "chosen_arc_count": Counter(),
    "route_count": Counter(),
    "reason": Counter(),
}

def _record_subtour_mipsol(choose_count):
    s = SUBTOUR_CUT_SUMMARY
    s["mipsol_callbacks"] += 1
    s["chosen_arc_count"][int(choose_count)] += 1

def _record_route_dp_stage(routes):
    s = SUBTOUR_CUT_SUMMARY
    s["reached_route_dp"] += 1
    s["route_count"][int(len(routes))] += 1
    s["routes_checked"] += int(len(routes))




def _find_front_pruned_infeasible_chain(route, arcs, data, fail_i):
    """Return a guarded infeasible chain from the failure prefix.

    The route DP reports the first failed skeleton edge. This helper maps that
    failed edge back to the selected fragment arc that covers it, forms the
    selected-route prefix ending at that covering arc, then front-prunes that
    prefix. Each candidate suffix is retested with the same route-DP feasibility
    check before it can be used as a lazy feasibility cut.

    In this callback, "guarded" means a smaller candidate is not trusted merely
    because it is shorter. It must be structurally mappable and must retest as
    infeasible. If those checks fail, the callback falls back to the full-route
    feasibility cut.

    """
    out = {
        "ok": False,
        "reason": None,
        "route_size": len(route),
        "prefix_size": None,
        "candidate_size": None,
        "candidate_arcs": None,
        "failed_edge": None,
        "covering_arc_pos": None,
        "covering_arc_id": None,
        "tests_run": 0,
        "would_shrink_vs_full": False,
        "would_shrink_vs_prefix": False,
    }

    try:
        skeleton, cover_arc_index = _build_route_skeleton_and_cover(route, arcs, data)
    except Exception as exc:
        out["reason"] = f"skeleton_cover_error:{type(exc).__name__}:{exc}"
        return out

    if fail_i is None or fail_i < 0 or fail_i + 1 >= len(skeleton):
        out["reason"] = "fail_i_unavailable_or_out_of_range"
        return out

    out["failed_edge"] = f"{skeleton[fail_i]}->{skeleton[fail_i + 1]}"

    if fail_i >= len(cover_arc_index):
        out["reason"] = "cover_arc_index_missing_fail_i"
        return out

    covering_arc_pos = cover_arc_index[fail_i]
    if covering_arc_pos < 0 or covering_arc_pos >= len(route):
        out["reason"] = "covering_arc_pos_out_of_range"
        return out

    out["covering_arc_pos"] = covering_arc_pos
    out["covering_arc_id"] = route[covering_arc_pos]

    prefix = list(route[:covering_arc_pos + 1])
    out["prefix_size"] = len(prefix)
    if len(prefix) < FEASIBILITY_CUT_MIN_CHAIN_SIZE:
        out["reason"] = "prefix_too_short_for_guarded_chain_cut"
        return out

    best_bad = None
    max_tests = min(len(prefix), FEASIBILITY_CUT_MAX_FRONT_PRUNE_TESTS)
    for start_pos in range(0, max_tests):
        candidate = list(prefix[start_pos:])
        if len(candidate) < FEASIBILITY_CUT_MIN_CHAIN_SIZE:
            continue
        try:
            cand_ok, cand_fail_i, cand_skeleton = _test_fragment_chain_feasibility(data, arcs, candidate)
        except Exception as exc:
            if FEASIBILITY_CUT_VERBOSE_CANDIDATE_TESTS:
                print(
                    f"[FEAS-CUT-DRY-RUN] candidate_test_error start_pos={start_pos} "
                    f"candidate_size={len(candidate)} error={type(exc).__name__}:{exc}"
                )
            continue
        out["tests_run"] += 1
        if FEASIBILITY_CUT_VERBOSE_CANDIDATE_TESTS:
            print(
                f"[FEAS-CUT-DRY-RUN] candidate_test start_pos={start_pos} "
                f"candidate_size={len(candidate)} candidate_ok={cand_ok} "
                f"candidate_fail_i={cand_fail_i} candidate_skeleton_len={len(cand_skeleton)} "
                f"candidate_arc_ids={candidate}"
            )
        if not cand_ok:
            best_bad = candidate

    if best_bad is None:
        out["reason"] = "no_front_pruned_candidate_retested_infeasible"
        _record_pruning_summary(len(route), len(prefix), None, out["reason"], False)
        return out

    out["ok"] = True
    out["reason"] = "candidate_retested_infeasible"
    out["candidate_size"] = len(best_bad)
    out["candidate_arcs"] = list(best_bad)
    out["would_shrink_vs_full"] = len(best_bad) < len(route)
    out["would_shrink_vs_prefix"] = len(best_bad) < len(prefix)
    _record_pruning_summary(len(route), len(prefix), len(best_bad), out["reason"], True)
    return out

# Active feasibility cut remains unchanged: full-route cut is still applied below.

ACTIVE_FEASIBILITY_CUT_ENABLED = True
FEASIBILITY_CUT_PRINT_PER_CUT = False

import atexit
FEASIBILITY_CUT_SUMMARY_ONLY = True
FEASIBILITY_CUT_SUMMARY = {
    "total": 0,
    "fallback": 0,
    "active": 0,
    "cut_size": Counter(),
    "full_size": Counter(),
    "reason": Counter(),
    "pattern": Counter(),
}

def _record_feasibility_cut_summary(full_size, cut_size, fallback, reason, cut_arcs):
    FEASIBILITY_CUT_SUMMARY["total"] += 1
    if fallback:
        FEASIBILITY_CUT_SUMMARY["fallback"] += 1
    else:
        FEASIBILITY_CUT_SUMMARY["active"] += 1
    FEASIBILITY_CUT_SUMMARY["cut_size"][int(cut_size)] += 1
    FEASIBILITY_CUT_SUMMARY["full_size"][int(full_size)] += 1
    FEASIBILITY_CUT_SUMMARY["reason"][str(reason)] += 1
    FEASIBILITY_CUT_SUMMARY["pattern"][tuple(cut_arcs)] += 1

def _print_feasibility_cut_summary():
    s = FEASIBILITY_CUT_SUMMARY
    if s["total"] == 0:
        print("[FEAS-CUT-SUMMARY] total=0")
        return
    print("[FEAS-CUT-SUMMARY] total=" + str(s["total"]) + " active=" + str(s["active"]) + " fallback=" + str(s["fallback"]))
    print("[FEAS-CUT-SUMMARY] cut_size_distribution=" + str(dict(sorted(s["cut_size"].items()))))
    print("[FEAS-CUT-SUMMARY] full_size_distribution=" + str(dict(sorted(s["full_size"].items()))))
    print("[FEAS-CUT-SUMMARY] reason_distribution=" + str(dict(s["reason"].most_common())))
    print("[FEAS-CUT-SUMMARY] top_cut_patterns=" + str([(list(k), v) for k, v in s["pattern"].most_common(20)]))

atexit.register(_print_feasibility_cut_summary)
atexit.register(_print_pruning_summary)
atexit.register(_print_solver_bound_summary)

# Defines component-summary diagnostics before atexit registration. This fixes
# definition in callback_core.py.
import os as os

if 'SUBTOUR_COMPONENT_CUTS_ENABLED' not in globals():
    # Component-wise disconnected-subtour cuts are now the default. Use
    # EVRP_SUBTOUR_COMPONENT_CUTS=0/false/no/off/aggregate to force the
    # legacy aggregate cut over the full disconnected arc set.
    SUBTOUR_COMPONENT_CUTS_ENABLED = str(__import__("os").getenv("EVRP_SUBTOUR_COMPONENT_CUTS", "1")).strip().lower() not in {"0", "false", "no", "off", "aggregate"}

if 'SUBTOUR_COMPONENT_SUMMARY' not in globals():
    SUBTOUR_COMPONENT_SUMMARY = {
        "analysed": 0,
        "component_cut_mode_used": 0,
        "aggregate_cut_mode_used": 0,
        "component_count": Counter(),
        "component_arc_size": Counter(),
        "component_node_size": Counter(),
        "largest_component_arc_size": Counter(),
        "smallest_component_arc_size": Counter(),
    }

# Active lazy-cut selection remains unchanged in this overlay.

# Active lazy-cut selection remains the guarded failure-prefix/front-pruned feasibility cut.

# Adds bounded-run diagnostics: front/rear pruning decomposition and solver bound snapshots.
# Active feasibility-cut logic remains unchanged.


# Self-contained subtour and component diagnostics for capped solves.
# Active behaviour is unchanged by default: aggregate bad_cycle_arcs cut remains active.
# Optional strengthening: set EVRP_SUBTOUR_COMPONENT_CUTS=1 to cut each disconnected component.
import os as os

if 'SUBTOUR_CUT_SUMMARY' not in globals():
    SUBTOUR_CUT_SUMMARY = {
        "mipsol_callbacks": 0,
        "subtour_cuts": 0,
        "returned_before_route_dp": 0,
        "reached_route_dp": 0,
        "routes_checked": 0,
        "chosen_arc_count": Counter(),
        "bad_cycle_size": Counter(),
        "route_count": Counter(),
        "reason": Counter(),
    }

if '_record_subtour_lazy_cut' not in globals():
    def _record_subtour_lazy_cut(cut_size, bad_tail_count=None):
        SUBTOUR_CUT_SUMMARY["subtour_cuts"] += 1
        SUBTOUR_CUT_SUMMARY["returned_before_route_dp"] += 1
        SUBTOUR_CUT_SUMMARY["bad_cycle_size"][int(cut_size)] += 1
        SUBTOUR_CUT_SUMMARY["reason"]["disconnected_cycle_or_subtour"] += 1

if '_print_subtour_cut_summary' not in globals():
    def _print_subtour_cut_summary():
        s = SUBTOUR_CUT_SUMMARY
        print(
            "[SUBTOUR-CUT-SUMMARY] "
            + "mipsol_callbacks=" + str(s["mipsol_callbacks"])
            + " subtour_cuts=" + str(s["subtour_cuts"])
            + " returned_before_route_dp=" + str(s["returned_before_route_dp"])
            + " reached_route_dp=" + str(s["reached_route_dp"])
            + " routes_checked=" + str(s["routes_checked"])
        )
        print("[SUBTOUR-CUT-SUMMARY] bad_cycle_size_distribution=" + str(dict(sorted(s["bad_cycle_size"].items()))))
        print("[SUBTOUR-CUT-SUMMARY] chosen_arc_count_distribution=" + str(dict(sorted(s["chosen_arc_count"].items()))))
        print("[SUBTOUR-CUT-SUMMARY] route_count_distribution=" + str(dict(sorted(s["route_count"].items()))))
        print("[SUBTOUR-CUT-SUMMARY] reason_distribution=" + str(dict(s["reason"].most_common())))

# Component-wise disconnected-subtour cuts are now the default. Use
# EVRP_SUBTOUR_COMPONENT_CUTS=0/false/no/off/aggregate to force the
# legacy aggregate cut over the full disconnected arc set.
SUBTOUR_COMPONENT_CUTS_ENABLED = str(__import__("os").getenv("EVRP_SUBTOUR_COMPONENT_CUTS", "1")).strip().lower() not in {"0", "false", "no", "off", "aggregate"}
SUBTOUR_COMPONENT_SUMMARY = {
    "analysed": 0,
    "component_cut_mode_used": 0,
    "aggregate_cut_mode_used": 0,
    "component_count": Counter(),
    "component_arc_size": Counter(),
    "component_node_size": Counter(),
    "largest_component_arc_size": Counter(),
    "smallest_component_arc_size": Counter(),
}

def _selected_disconnected_components(bad_cycle_arcs, arcs):
    adj = {}
    arc_lookup = {}
    for aid in bad_cycle_arcs:
        a = arcs[aid]
        u = a['u']
        v = a['v']
        adj.setdefault(u, set()).add(v)
        adj.setdefault(v, set()).add(u)
        arc_lookup.setdefault(u, set()).add(aid)
        arc_lookup.setdefault(v, set()).add(aid)
    seen = set()
    comps = []
    for n in list(adj):
        if n in seen:
            continue
        stack = [n]
        seen.add(n)
        nodes = set()
        comp_arcs = set()
        while stack:
            x = stack.pop()
            nodes.add(x)
            comp_arcs.update(arc_lookup.get(x, set()))
            for y in adj.get(x, set()):
                if y not in seen:
                    seen.add(y)
                    stack.append(y)
        comps.append({"nodes": nodes, "arcs": sorted(comp_arcs)})
    comps.sort(key=lambda c: (-len(c["arcs"]), -len(c["nodes"]), c["arcs"][0] if c["arcs"] else -1))
    return comps

def _record_subtour_component_summary(components, mode):
    SUBTOUR_COMPONENT_SUMMARY["analysed"] += 1
    if mode == "component":
        SUBTOUR_COMPONENT_SUMMARY["component_cut_mode_used"] += 1
    else:
        SUBTOUR_COMPONENT_SUMMARY["aggregate_cut_mode_used"] += 1
    sizes = []
    SUBTOUR_COMPONENT_SUMMARY["component_count"][int(len(components))] += 1
    for comp in components:
        arc_size = len(comp.get("arcs", []))
        node_size = len(comp.get("nodes", []))
        sizes.append(arc_size)
        SUBTOUR_COMPONENT_SUMMARY["component_arc_size"][int(arc_size)] += 1
        SUBTOUR_COMPONENT_SUMMARY["component_node_size"][int(node_size)] += 1
    if sizes:
        SUBTOUR_COMPONENT_SUMMARY["largest_component_arc_size"][int(max(sizes))] += 1
        SUBTOUR_COMPONENT_SUMMARY["smallest_component_arc_size"][int(min(sizes))] += 1

def _print_subtour_component_summary():
    s = SUBTOUR_COMPONENT_SUMMARY
    print(
        "[SUBTOUR-COMPONENT-SUMMARY] "
        + "analysed=" + str(s["analysed"])
        + " component_cut_mode_used=" + str(s["component_cut_mode_used"])
        + " aggregate_cut_mode_used=" + str(s["aggregate_cut_mode_used"])
        + " component_cuts_enabled=" + str(SUBTOUR_COMPONENT_CUTS_ENABLED)
    )
    print("[SUBTOUR-COMPONENT-SUMMARY] component_count_distribution=" + str(dict(sorted(s["component_count"].items()))))
    print("[SUBTOUR-COMPONENT-SUMMARY] component_arc_size_distribution=" + str(dict(sorted(s["component_arc_size"].items()))))
    print("[SUBTOUR-COMPONENT-SUMMARY] component_node_size_distribution=" + str(dict(sorted(s["component_node_size"].items()))))
    print("[SUBTOUR-COMPONENT-SUMMARY] largest_component_arc_size_distribution=" + str(dict(sorted(s["largest_component_arc_size"].items()))))
    print("[SUBTOUR-COMPONENT-SUMMARY] smallest_component_arc_size_distribution=" + str(dict(sorted(s["smallest_component_arc_size"].items()))))

# --- v5j queueing callback cut/reporting refinement helpers ---
QUEUEING_CALLBACK_ENABLED = True
QUEUEING_CALLBACK_PRINT_PER_INCUMBENT = False
QUEUEING_CALLBACK_PRINT_PER_CUT = True
QUEUEING_CALLBACK_PRINT_DUPLICATE_CUTS = False
QUEUEING_CALLBACK_SUMMARY_ENABLED = True

_QUEUEING_CALLBACK_STATS = {
    "evaluations": 0,
    "feasible_no_delay": 0,
    "feasible_with_delay": 0,
    "infeasible": 0,
    "cuts": 0,
    "fallback_cuts": 0,
    "duplicate_cut_candidates": 0,
    "duplicate_cut_prints_suppressed": 0,
    "unique_cut_patterns": 0,
    "total_delay_positive": 0,
    "total_service_delay_positive": 0,
    "resource_wait_positive": 0,
    "resource_wait_event_positive": 0,
    "max_total_delay": 0.0,
    "max_total_service_delay": 0.0,
    "max_resource_wait_time": 0.0,
    "max_resource_wait_event_count": 0,
    "sum_resource_wait_time": 0.0,
    "cut_size_distribution": {},
    "reason_distribution": {},
    "delayed_service_count_distribution": {},
    "resource_wait_event_count_distribution": {},
    "resolved_overlap_count_distribution": {},
    "cut_pattern_counts": {},
}

_QUEUEING_CALLBACK_SEEN_CUT_KEYS = set()


def _queueing_bump_distribution(name, key):
    dist = _QUEUEING_CALLBACK_STATS.setdefault(name, {})
    dist[key] = dist.get(key, 0) + 1


def _queueing_cut_key(arcs):
    return tuple(sorted(int(aid) for aid in arcs))


def _record_queueing_callback_evaluation(q_eval):
    _QUEUEING_CALLBACK_STATS["evaluations"] += 1
    _queueing_bump_distribution("reason_distribution", getattr(q_eval, "reason", "unknown"))

    if getattr(q_eval, "ok", False):
        if getattr(q_eval, "total_service_delay", 0.0) > 1e-6:
            _QUEUEING_CALLBACK_STATS["feasible_with_delay"] += 1
        else:
            _QUEUEING_CALLBACK_STATS["feasible_no_delay"] += 1
    else:
        _QUEUEING_CALLBACK_STATS["infeasible"] += 1

    total_delay = float(getattr(q_eval, "total_delay", 0.0) or 0.0)
    total_service_delay = float(getattr(q_eval, "total_service_delay", 0.0) or 0.0)
    resource_wait_time = float(getattr(q_eval, "resource_wait_time", 0.0) or 0.0)
    resource_wait_event_count = int(getattr(q_eval, "resource_wait_event_count", 0) or 0)
    resolved_overlap_count = int(getattr(q_eval, "resolved_overlap_count", resource_wait_event_count) or 0)

    if total_delay > 1e-6:
        _QUEUEING_CALLBACK_STATS["total_delay_positive"] += 1
    if total_service_delay > 1e-6:
        _QUEUEING_CALLBACK_STATS["total_service_delay_positive"] += 1
    if resource_wait_time > 1e-6:
        _QUEUEING_CALLBACK_STATS["resource_wait_positive"] += 1
    if resource_wait_event_count > 0:
        _QUEUEING_CALLBACK_STATS["resource_wait_event_positive"] += 1

    _QUEUEING_CALLBACK_STATS["max_total_delay"] = max(_QUEUEING_CALLBACK_STATS["max_total_delay"], total_delay)
    _QUEUEING_CALLBACK_STATS["max_total_service_delay"] = max(_QUEUEING_CALLBACK_STATS["max_total_service_delay"], total_service_delay)
    _QUEUEING_CALLBACK_STATS["max_resource_wait_time"] = max(_QUEUEING_CALLBACK_STATS["max_resource_wait_time"], resource_wait_time)
    _QUEUEING_CALLBACK_STATS["max_resource_wait_event_count"] = max(_QUEUEING_CALLBACK_STATS["max_resource_wait_event_count"], resource_wait_event_count)
    _QUEUEING_CALLBACK_STATS["sum_resource_wait_time"] += resource_wait_time

    _queueing_bump_distribution("delayed_service_count_distribution", int(getattr(q_eval, "delayed_service_count", 0) or 0))
    _queueing_bump_distribution("resource_wait_event_count_distribution", resource_wait_event_count)
    _queueing_bump_distribution("resolved_overlap_count_distribution", resolved_overlap_count)


def _record_queueing_callback_cut(cut_arcs, fallback):
    cut_key = _queueing_cut_key(cut_arcs)
    pattern_counts = _QUEUEING_CALLBACK_STATS.setdefault("cut_pattern_counts", {})
    pattern_counts[cut_key] = pattern_counts.get(cut_key, 0) + 1

    duplicate = cut_key in _QUEUEING_CALLBACK_SEEN_CUT_KEYS
    if duplicate:
        _QUEUEING_CALLBACK_STATS["duplicate_cut_candidates"] += 1
    else:
        _QUEUEING_CALLBACK_SEEN_CUT_KEYS.add(cut_key)
        _QUEUEING_CALLBACK_STATS["unique_cut_patterns"] = len(_QUEUEING_CALLBACK_SEEN_CUT_KEYS)

    _QUEUEING_CALLBACK_STATS["cuts"] += 1
    if fallback:
        _QUEUEING_CALLBACK_STATS["fallback_cuts"] += 1
    _queueing_bump_distribution("cut_size_distribution", len(cut_key))
    return duplicate, pattern_counts[cut_key]


def _format_top_queueing_cut_patterns(limit=10):
    pattern_counts = _QUEUEING_CALLBACK_STATS.get("cut_pattern_counts", {})
    ranked = sorted(pattern_counts.items(), key=lambda kv: (-kv[1], len(kv[0]), kv[0]))
    return [(list(pattern), count) for pattern, count in ranked[:limit]]


def _print_queueing_callback_summary():
    if not QUEUEING_CALLBACK_SUMMARY_ENABLED:
        return
    stats = _QUEUEING_CALLBACK_STATS
    print(
        "[QUEUEING-CALLBACK-SUMMARY]",
        "evaluations=", stats.get("evaluations", 0),
        "feasible_no_delay=", stats.get("feasible_no_delay", 0),
        "feasible_with_delay=", stats.get("feasible_with_delay", 0),
        "infeasible=", stats.get("infeasible", 0),
        "cuts=", stats.get("cuts", 0),
        "fallback_cuts=", stats.get("fallback_cuts", 0),
        "unique_cut_patterns=", stats.get("unique_cut_patterns", 0),
        "duplicate_cut_candidates=", stats.get("duplicate_cut_candidates", 0),
        "duplicate_cut_prints_suppressed=", stats.get("duplicate_cut_prints_suppressed", 0),
        "positive_delay=", stats.get("total_delay_positive", 0),
        "positive_service_delay=", stats.get("total_service_delay_positive", 0),
        "positive_resource_wait=", stats.get("resource_wait_positive", 0),
        "positive_resource_wait_events=", stats.get("resource_wait_event_positive", 0),
        "max_total_delay=", stats.get("max_total_delay", 0.0),
        "max_total_service_delay=", stats.get("max_total_service_delay", 0.0),
        "max_resource_wait_time=", stats.get("max_resource_wait_time", 0.0),
        "max_resource_wait_event_count=", stats.get("max_resource_wait_event_count", 0),
        "sum_resource_wait_time=", stats.get("sum_resource_wait_time", 0.0),
    )
    print("[QUEUEING-CALLBACK-SUMMARY] reason_distribution=", stats.get("reason_distribution", {}))
    print("[QUEUEING-CALLBACK-SUMMARY] delayed_service_count_distribution=", stats.get("delayed_service_count_distribution", {}))
    print("[QUEUEING-CALLBACK-SUMMARY] resource_wait_event_count_distribution=", stats.get("resource_wait_event_count_distribution", {}))
    print("[QUEUEING-CALLBACK-SUMMARY] resolved_overlap_count_distribution=", stats.get("resolved_overlap_count_distribution", {}))
    print("[QUEUEING-CALLBACK-SUMMARY] cut_size_distribution=", stats.get("cut_size_distribution", {}))
    print("[QUEUEING-CALLBACK-SUMMARY] top_cut_patterns=", _format_top_queueing_cut_patterns())


def _register_queueing_callback_summary():
    global _QUEUEING_CALLBACK_SUMMARY_REGISTERED
    if globals().get("_QUEUEING_CALLBACK_SUMMARY_REGISTERED", False):
        return
    atexit.register(_print_queueing_callback_summary)
    _QUEUEING_CALLBACK_SUMMARY_REGISTERED = True


_register_queueing_callback_summary()
# --- end v5j queueing callback cut/reporting refinement helpers ---
def callback(model, where, x_vars, arcs, node_id, depot_u, data, theta, M):
    if where == gp.GRB.Callback.MIP:
        _record_solver_bound_snapshot(model, where)
        return
    if where == gp.GRB.Callback.MIPSOL:
        _record_solver_bound_snapshot(model, where)
    if where != gp.GRB.Callback.MIPSOL:
        return

    # Selected arcs
    xsol = model.cbGetSolution(x_vars)
    choose = [a_id for a_id, val in xsol.items() if val > 0.5]
    _record_subtour_mipsol(len(choose))
    if not choose:
        return

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
        components = _selected_disconnected_components(bad_cycle_arcs, arcs)
        if SUBTOUR_COMPONENT_CUTS_ENABLED and components:
            _record_subtour_component_summary(components, "component")
            for comp in components:
                comp_arcs = list(comp.get("arcs", []))
                if comp_arcs:
                    _record_subtour_lazy_cut(len(comp_arcs), len(comp.get("nodes", [])))
                    model.cbLazy(gp.quicksum(x_vars[a_id] for a_id in comp_arcs) <= len(comp_arcs) - 1)
        else:
            _record_subtour_component_summary(components, "aggregate")
            _record_subtour_lazy_cut(len(bad_cycle_arcs), len({arcs[a_id]['u'] for a_id in bad_cycle_arcs}))
            model.cbLazy(gp.quicksum(x_vars[a_id] for a_id in bad_cycle_arcs) <= len(bad_cycle_arcs) - 1)
        return

    # v5g queueing callback diagnostic and infeasibility hook.
    # v5j refinement: fingerprint cut patterns and suppress duplicate cut prints.
    # This runs after disconnected subtour/component cuts and before the energy-DP.
    if QUEUEING_CALLBACK_ENABLED:
        q_eval = qcore.evaluate_queueing_for_selected_arcs(
            data=data,
            arcs=arcs,
            depot_u=depot_u,
            selected_arc_ids=choose,
        )
        _record_queueing_callback_evaluation(q_eval)

        if QUEUEING_CALLBACK_PRINT_PER_INCUMBENT and (
            (not q_eval.ok) or q_eval.total_service_delay > 1e-6
        ):
            print(
                "[QUEUEING-CALLBACK]",
                "ok=", q_eval.ok,
                "reason=", q_eval.reason,
                "routes=", q_eval.route_count,
                "activities=", q_eval.activity_count,
                "services=", q_eval.service_count,
                "total_delay=", q_eval.total_delay,
                "service_delay=", q_eval.total_service_delay,
                "resource_wait=", getattr(q_eval, "resource_wait_time", 0.0),
                "resource_wait_events=", getattr(q_eval, "resource_wait_event_count", 0),
                "resolved_overlaps=", getattr(q_eval, "resolved_overlap_count", 0),
                "delayed_services=", q_eval.delayed_service_count,
                "conflicts=", q_eval.conflict_count,
                "infeasible_arcs=", list(q_eval.infeasible_arc_ids),
                "resolved_delay_arcs=", list(q_eval.resolved_delay_arc_ids),
            )

        if not q_eval.ok:
            S_queue = [aid for aid in q_eval.infeasible_arc_ids if aid in x_vars]
            fallback = False
            if not S_queue:
                S_queue = list(choose)
                fallback = True
            if S_queue:
                duplicate, occurrence_count = _record_queueing_callback_cut(S_queue, fallback)
                should_print_cut = QUEUEING_CALLBACK_PRINT_PER_CUT and (
                    QUEUEING_CALLBACK_PRINT_DUPLICATE_CUTS or not duplicate
                )
                if duplicate and QUEUEING_CALLBACK_PRINT_PER_CUT and not QUEUEING_CALLBACK_PRINT_DUPLICATE_CUTS:
                    _QUEUEING_CALLBACK_STATS["duplicate_cut_prints_suppressed"] += 1
                if should_print_cut:
                    print(
                        "[QUEUEING-CUT-ACTIVE]",
                        "cut_size=", len(S_queue),
                        "fallback=", fallback,
                        "duplicate=", duplicate,
                        "occurrence=", occurrence_count,
                        "reason=", q_eval.reason,
                        "cut_arcs=", S_queue,
                    )
                model.cbLazy(gp.quicksum(x_vars[aid] for aid in S_queue) <= len(S_queue) - 1)
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

    _record_route_dp_stage(routes)
    # We'll compute DP-based feasibility/cost per route and aggregate delta_total across all routes
    delta_total = 0.0

    # --- process each route ---
    for route in routes:
        sid_seq = []
        for k, aid in enumerate(route):
            s = arcs[aid]['seq']
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

        if not ok:
            # Active guarded front-pruned feasibility cut.
            candidate_info = _find_front_pruned_infeasible_chain(route, arcs, data, fail_i)
            if ACTIVE_FEASIBILITY_CUT_ENABLED and candidate_info.get('ok') and candidate_info.get('candidate_arcs') and candidate_info.get('candidate_size', len(route)) < len(route):
                S_feas = list(candidate_info['candidate_arcs'])
                fallback = False
                reason = 'guarded_candidate_active'
            else:
                S_feas = list(route)
                fallback = True
                reason = candidate_info.get('reason', 'candidate_not_available_or_not_smaller')
            if FEASIBILITY_CUT_PRINT_PER_CUT:
                print('[FEAS-CUT-ACTIVE]', 'cut_size=', len(S_feas), 'full_size=', len(route), 'fallback=', fallback, 'reason=', reason, 'cut_arcs=', S_feas)
            _record_feasibility_cut_summary(len(route), len(S_feas), fallback, reason, S_feas)
            model.cbLazy(gp.quicksum(x_vars[aid] for aid in S_feas) <= len(S_feas) - 1)
            return

        # Base cost consistent with master objective: sum of Df of chosen arcs on this route
        base_route_cost = 0.0
        for aid in route:
            base_route_cost += arcs[aid]['Df']

        delta = dp_dist - base_route_cost

        if delta > 1e-6:
            delta_total += delta

    # --- Optimality cut for whole incumbent ---
    if delta_total > 1e-6:
        # Condition on selecting all chosen arcs
        S = choose
        model.cbLazy(theta >= delta_total - M*(len(S) - gp.quicksum(x_vars[a] for a in S)))

# Register final branch-summary printers after consolidated definitions are in
# scope. Instrumentation only; does not alter subtour or feasibility logic.
def _register_callback_branch_summaries():
    global _CALLBACK_BRANCH_SUMMARIES_REGISTERED
    if globals().get("_CALLBACK_BRANCH_SUMMARIES_REGISTERED", False):
        return
    registered = []
    for name in ("_print_subtour_cut_summary", "_print_subtour_component_summary"):
        func = globals().get(name)
        if callable(func):
            atexit.register(func)
            registered.append(name)
    _CALLBACK_BRANCH_SUMMARIES_REGISTERED = True
    globals()["_CALLBACK_BRANCH_SUMMARY_REGISTERED_NAMES"] = tuple(registered)

_register_callback_branch_summaries()
