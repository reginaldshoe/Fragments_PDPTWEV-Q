"""
dp.py
-----
Dynamic-programming sub-problem for charging-station insertion.

Role in the algorithm
---------------------
After the MILP master problem produces an integer solution, the callback
must verify that each route in that solution is energy-feasible when stations
are allowed to be inserted between skeleton nodes.  If a route is infeasible,
a feasibility cut is added.  If it is feasible but the direct-distance cost
understates the true cost (because stations add detour distance), the
difference (delta) is accumulated and an optimality cut is added to tighten
theta.

The DP operates on a *skeleton* — the mandatory sequence of non-station nodes
(depot, pickups, deliveries) that define the route structure.  For each
consecutive pair of skeleton nodes (a leg), it finds all non-dominated ways
to travel from one to the other, optionally stopping at charging stations
along the way.  Labels are propagated across legs to find the minimum-distance
realisation of the full skeleton.

Two functions
-------------
solve_leg(inst, from_id, to_id, t0, E0, max_station_visits)
    Single-leg DP: finds all non-dominated arrival labels at to_id, where
    intermediate charging stations may be inserted.  Returns a list of
    LegArrival named tuples.

solve_route(inst, skeleton, t0, E0, max_station_visits_per_leg,
            max_labels_per_node, stats)
    Multi-leg DP over a full skeleton.  Calls solve_leg for each consecutive
    pair, propagates labels across the full route, and returns a DPResult.
    If *stats* is provided it is updated in place.

Statistics
----------
DPStats accumulates information across all callback invocations and is the
primary instrumentation for the research report.  Pass the same DPStats
object to every solve_route() call throughout a solve run.
"""

from __future__ import annotations

import heapq
from dataclasses import dataclass, field
from typing import Dict, List, Optional, Tuple

from instance import Instance

EPS = 1e-9


# ---------------------------------------------------------------------------
# Label types
# ---------------------------------------------------------------------------

@dataclass(frozen=True)
class LegArrival:
    """
    A non-dominated arrival at the destination of a single leg.

    distance  : total distance travelled on this leg (including any detour
                via charging stations)
    time      : service-start time at the destination node (after waiting for
                ready_time if necessary)
    energy    : remaining battery on arrival at the destination (before service)
    path      : sequence of node_ids traversed on this leg
    """
    distance: float
    time:     float
    energy:   float
    path:     Tuple[str, ...]


@dataclass(frozen=True)
class RouteLabel:
    """
    A non-dominated label at a mandatory skeleton node during the multi-leg DP.

    distance      : cumulative distance from skeleton start to this node
    departure_time: time at which the vehicle departs this node (after service)
    departure_energy: remaining battery when departing this node
    path          : full node_id sequence from skeleton start to this node
    """
    distance:         float
    departure_time:   float
    departure_energy: float
    path:             Tuple[str, ...]


# ---------------------------------------------------------------------------
# DPResult
# ---------------------------------------------------------------------------

@dataclass
class DPResult:
    """
    Result of a single solve_route() call.

    feasible      : True if the skeleton can be realised with energy feasibility
    total_distance: minimum total distance of the realised route (None if infeasible)
    realised_path : full node_id sequence including inserted stations (None if infeasible)
    fail_leg      : index i of the first leg skeleton[i]->skeleton[i+1] that
                    could not be completed (None if feasible)
    direct_distance: sum of direct leg distances ignoring stations — the
                    MILP's optimistic cost estimate for this route
    station_detour : total_distance - direct_distance  (0 if infeasible)
    """
    feasible:        bool
    total_distance:  Optional[float]
    realised_path:   Optional[Tuple[str, ...]]
    fail_leg:        Optional[int]
    direct_distance: float
    station_detour:  float = 0.0


# ---------------------------------------------------------------------------
# DPStats  (the research instrumentation)
# ---------------------------------------------------------------------------

@dataclass
class DPStats:
    """
    Cumulative statistics across all DP sub-problem calls during a solve run.

    Intended usage: create one DPStats object per K-iteration and pass it to
    every solve_route() call.  Inspect after solve for reporting.

    Call-level counters
    -------------------
    total_calls        : total number of solve_route() calls
    feasible_calls     : calls that returned a feasible route
    infeasible_calls   : calls that returned an infeasible route

    Cut counters (updated by the callback, not by solve_route itself)
    ----------------------------------------------------------------
    feasibility_cuts_added : number of feasibility cuts added to the model
    optimality_cuts_added  : number of optimality cuts added to the model

    Work counters (updated by solve_route / solve_leg)
    --------------------------------------------------
    total_legs_solved      : total number of single-leg DPs executed
    total_labels_generated : total labels created across all leg DPs
    total_labels_kept      : total non-dominated labels kept
    total_station_visits   : total charging-station visits across all paths

    Cost tracking
    -------------
    total_direct_distance  : sum of direct skeleton distances across all calls
    total_realised_distance: sum of DP-realised distances across feasible calls
    total_station_detour   : sum of (realised - direct) across feasible calls

    Per-call history (for detailed reporting)
    ------------------------------------------
    call_log : list of DPCallRecord, one per solve_route() call
    """

    # call-level
    total_calls:       int = 0
    feasible_calls:    int = 0
    infeasible_calls:  int = 0

    # cuts (set externally by the callback)
    feasibility_cuts_added: int = 0
    optimality_cuts_added:  int = 0

    # work
    total_legs_solved:       int = 0
    total_labels_generated:  int = 0
    total_labels_kept:       int = 0
    total_station_visits:    int = 0

    # cost
    total_direct_distance:   float = 0.0
    total_realised_distance: float = 0.0
    total_station_detour:    float = 0.0

    # per-call log
    call_log: List[DPCallRecord] = field(default_factory=list)

    # ------------------------------------------------------------------
    # Update helpers (called internally by solve_route)
    # ------------------------------------------------------------------

    def _record_call(self, result: DPResult, record: DPCallRecord) -> None:
        self.total_calls += 1
        if result.feasible:
            self.feasible_calls += 1
            self.total_realised_distance += result.total_distance
        else:
            self.infeasible_calls += 1
        self.total_direct_distance  += result.direct_distance
        self.total_station_detour   += result.station_detour
        self.call_log.append(record)

    # ------------------------------------------------------------------
    # Summary for reporting
    # ------------------------------------------------------------------

    def summary(self) -> Dict:
        """Return a flat dict of all statistics, suitable for printing or logging."""
        return {
            'total_dp_calls':           self.total_calls,
            'feasible_calls':           self.feasible_calls,
            'infeasible_calls':         self.infeasible_calls,
            'feasibility_cuts_added':   self.feasibility_cuts_added,
            'optimality_cuts_added':    self.optimality_cuts_added,
            'total_legs_solved':        self.total_legs_solved,
            'total_labels_generated':   self.total_labels_generated,
            'total_labels_kept':        self.total_labels_kept,
            'avg_labels_per_leg': (
                self.total_labels_kept / self.total_legs_solved
                if self.total_legs_solved > 0 else 0.0
            ),
            'total_station_visits':     self.total_station_visits,
            'total_direct_distance':    round(self.total_direct_distance,   4),
            'total_realised_distance':  round(self.total_realised_distance, 4),
            'total_station_detour':     round(self.total_station_detour,    4),
        }

    def print_summary(self, label: str = '') -> None:
        header = f'DP Statistics{f" — {label}" if label else ""}'
        print(f'\n{header}')
        print('-' * len(header))
        for k, v in self.summary().items():
            print(f'  {k:<35} {v}')


@dataclass
class DPCallRecord:
    """Per-call record stored inside DPStats.call_log."""
    skeleton:          Tuple[str, ...]
    feasible:          bool
    direct_distance:   float
    realised_distance: Optional[float]
    station_detour:    float
    legs_solved:       int
    labels_generated:  int
    labels_kept:       int
    station_visits:    int
    fail_leg:          Optional[int]


# ---------------------------------------------------------------------------
# Internal dominance helpers
# ---------------------------------------------------------------------------

def _dominates_label(
    a: Tuple[float, float, float],
    b: Tuple[float, float, float],
) -> bool:
    """
    True if label a dominates label b.

    Labels are (distance, time, energy) triples.
    a dominates b if a is at least as good on all three and strictly better
    on at least one:
        - smaller distance  (less travel)
        - smaller time      (arrives/departs sooner)
        - larger  energy    (more battery remaining)
    """
    da, ta, ea = a
    db, tb, eb = b
    better_or_equal = (da <= db + EPS) and (ta <= tb + EPS) and (ea >= eb - EPS)
    strictly_better = (da < db - EPS) or  (ta < tb - EPS)  or  (ea > eb + EPS)
    return better_or_equal and strictly_better


def _insert_into_nondominated(
    existing: List[Tuple[float, float, float]],
    candidate: Tuple[float, float, float],
) -> Tuple[List[Tuple[float, float, float]], bool]:
    """
    Add *candidate* to *existing* if it is not dominated, removing any
    existing labels that the candidate now dominates.

    Returns (updated_list, was_inserted).
    """
    for lab in existing:
        if _dominates_label(lab, candidate):
            return existing, False
    kept = [lab for lab in existing if not _dominates_label(candidate, lab)]
    kept.append(candidate)
    return kept, True


# ---------------------------------------------------------------------------
# Single-leg DP
# ---------------------------------------------------------------------------

def solve_leg(
    inst:               Instance,
    from_id:            str,
    to_id:              str,
    t0:                 float,
    E0:                 float,
    max_station_visits: int = 3,
) -> Tuple[List[LegArrival], int, int, int]:
    """
    Find all non-dominated ways to travel from *from_id* to *to_id*, starting
    at time *t0* with battery level *E0*, optionally inserting charging stations.

    Stations are treated as always open (no time window) up to the planning
    horizon.  The destination enforces its own time window.  Charging is
    always to full battery.

    Parameters
    ----------
    inst               : problem instance
    from_id            : starting node_id
    to_id              : destination node_id
    t0                 : departure time from from_id
    E0                 : battery level at departure from from_id
    max_station_visits : maximum number of charging stops allowed on this leg

    Returns
    -------
    arrivals          : list of non-dominated LegArrival objects at to_id
                        (empty if the leg is infeasible)
    labels_generated  : total labels pushed onto the priority queue
    labels_kept       : total labels retained after dominance filtering
    station_visits    : total station-visit events across all explored paths
    """
    from_idx = inst.idx(from_id)
    to_idx   = inst.idx(to_id)
    to_node  = inst.node(to_id)

    station_ids = inst.station_ids

    # Priority queue entries:
    # (distance_so_far, departure_time, -departure_energy,
    #  current_id, path_tuple, station_count, visited_stations_frozenset)
    pq: List = []
    heapq.heappush(pq, (0.0, t0, -E0, from_id, (from_id,), 0, frozenset()))

    # Non-dominated (distance, departure_time, departure_energy) labels per
    # intermediate node.  Keyed by node_id.
    best_at_node: Dict[str, List[Tuple[float, float, float]]] = {
        from_id: [(0.0, t0, E0)]
    }

    arrivals:         List[LegArrival]               = []
    labels_generated: int = 0
    labels_kept:      int = 0
    station_visits:   int = 0

    def _keep_arrival(cand: LegArrival) -> bool:
        """Add cand to arrivals if non-dominated; return True if kept."""
        nonlocal arrivals
        cand_triple = (cand.distance, cand.time, cand.energy)
        for arr in arrivals:
            if _dominates_label((arr.distance, arr.time, arr.energy), cand_triple):
                return False
        arrivals = [
            a for a in arrivals
            if not _dominates_label(cand_triple, (a.distance, a.time, a.energy))
        ]
        arrivals.append(cand)
        arrivals.sort(key=lambda a: (a.distance, a.time, -a.energy))
        return True

    while pq:
        d_so_far, t_dep, neg_E, cur_id, path, n_stations, visited_stations = (
            heapq.heappop(pq)
        )
        E_dep = -neg_E

        # Candidates: always try reaching the destination; try stations if budget allows
        candidates = [to_id] + (station_ids if n_stations < max_station_visits else [])

        for nxt_id in candidates:
            if nxt_id == cur_id or nxt_id in visited_stations:
                continue

            cur_idx = inst.idx(cur_id)
            nxt_idx = inst.idx(nxt_id)

            e_needed = inst.energy_consumed(cur_idx, nxt_idx)
            if e_needed > E_dep + EPS:
                continue

            t_arr  = t_dep + inst.travel_time(cur_idx, nxt_idx)
            E_arr  = E_dep - e_needed
            d_new  = d_so_far + inst.distance(cur_idx, nxt_idx)
            labels_generated += 1

            # --- reached destination ---
            if nxt_id == to_id:
                t_start = max(t_arr, to_node.ready_time)
                if t_start > to_node.due_time + EPS:
                    continue
                arrival = LegArrival(
                    distance = d_new,
                    time     = t_start,
                    energy   = E_arr,
                    path     = path + (nxt_id,),
                )
                if _keep_arrival(arrival):
                    labels_kept += 1
                continue

            # --- reached a charging station ---
            if t_arr > inst.horizon + EPS:
                continue

            charge_time = inst.charge_time_from(E_arr)
            t_dep2      = t_arr + charge_time
            E_dep2      = inst.battery_capacity

            lab_triple = (d_new, t_dep2, E_dep2)
            existing   = best_at_node.get(nxt_id, [])
            updated, kept = _insert_into_nondominated(existing, lab_triple)
            if not kept:
                continue

            best_at_node[nxt_id] = updated
            labels_kept    += 1
            station_visits += 1

            new_visited = visited_stations | frozenset([nxt_id])
            heapq.heappush(pq, (
                d_new, t_dep2, -E_dep2,
                nxt_id, path + (nxt_id,),
                n_stations + 1, new_visited,
            ))

    return arrivals, labels_generated, labels_kept, station_visits


# ---------------------------------------------------------------------------
# Multi-leg route DP
# ---------------------------------------------------------------------------

def solve_route(
    inst:                      Instance,
    skeleton:                  Tuple[str, ...],
    t0:                        float = 0.0,
    E0:                        Optional[float] = None,
    max_station_visits_per_leg: int = 3,
    max_labels_per_node:        int = 50,
    stats:                     Optional[DPStats] = None,
) -> DPResult:
    """
    Multi-leg DP over *skeleton* (a sequence of mandatory non-station node_ids).

    For each consecutive pair of skeleton nodes, calls solve_leg() to find
    non-dominated arrival labels at the next node.  Labels are propagated
    across legs, subject to a cap of *max_labels_per_node* to bound work.

    The best (minimum distance, then time, then energy) label at the final
    skeleton node is returned as the optimal realisation.

    Parameters
    ----------
    inst                       : problem instance
    skeleton                   : tuple of node_ids, must contain no stations
    t0                         : departure time from skeleton[0]
    E0                         : battery level at skeleton[0] (defaults to full)
    max_station_visits_per_leg : passed to solve_leg for each leg
    max_labels_per_node        : cap on non-dominated labels kept per node
    stats                      : DPStats object to update in place (optional)

    Returns
    -------
    DPResult
    """
    if E0 is None:
        E0 = inst.battery_capacity

    # Validate skeleton: no stations, no repeated customer nodes
    for sid in skeleton:
        if inst.node(sid).is_station:
            raise ValueError(
                f"Skeleton must not contain stations; found '{sid}'. "
                "Filter stations from the sequence before calling solve_route()."
            )
    customer_visits = [s for s in skeleton if inst.node(s).is_customer]
    if len(customer_visits) != len(set(customer_visits)):
        seen, dups = set(), []
        for s in customer_visits:
            if s in seen and s not in dups:
                dups.append(s)
            seen.add(s)
        raise ValueError(
            f"Skeleton contains repeated customer nodes {dups} in {list(skeleton)}. "
            "This indicates a bug in arc stitching or route extraction."
        )

    # Compute direct distance (MILP's optimistic cost)
    direct_distance = sum(
        inst.distance(inst.idx(u), inst.idx(v))
        for u, v in zip(skeleton, skeleton[1:])
    )

    # Per-call accumulators for stats
    call_legs_solved      = 0
    call_labels_generated = 0
    call_labels_kept      = 0
    call_station_visits   = 0

    # Labels at each mandatory node: list of RouteLabel
    # Seed at the first skeleton node
    start_id = skeleton[0]
    labels: Dict[str, List[RouteLabel]] = {
        start_id: [RouteLabel(
            distance         = 0.0,
            departure_time   = t0,
            departure_energy = E0,
            path             = (start_id,),
        )]
    }

    def _insert_route_label(
        existing: List[RouteLabel],
        candidate: RouteLabel,
    ) -> List[RouteLabel]:
        """Insert candidate into existing, maintaining non-domination and cap."""
        c_triple = (candidate.distance, candidate.departure_time, candidate.departure_energy)
        for lab in existing:
            l_triple = (lab.distance, lab.departure_time, lab.departure_energy)
            if _dominates_label(l_triple, c_triple):
                return existing
        kept = [
            lab for lab in existing
            if not _dominates_label(
                c_triple,
                (lab.distance, lab.departure_time, lab.departure_energy),
            )
        ]
        kept.append(candidate)
        kept.sort(key=lambda l: (l.distance, l.departure_time, -l.departure_energy))
        return kept[:max_labels_per_node]

    # --- Leg-by-leg propagation ---
    for leg_idx in range(len(skeleton) - 1):
        u_id = skeleton[leg_idx]
        v_id = skeleton[leg_idx + 1]
        v_node = inst.node(v_id)

        candidate_labels_at_v: List[RouteLabel] = []

        for route_lab in labels.get(u_id, []):
            leg_arrivals, gen, kept, sv = solve_leg(
                inst,
                from_id            = u_id,
                to_id              = v_id,
                t0                 = route_lab.departure_time,
                E0                 = route_lab.departure_energy,
                max_station_visits = max_station_visits_per_leg,
            )
            call_legs_solved      += 1
            call_labels_generated += gen
            call_labels_kept      += kept
            call_station_visits   += sv

            for arr in leg_arrivals:
                # Stitch the leg path onto the route path (avoid duplicating junction node)
                if route_lab.path and arr.path and route_lab.path[-1] == arr.path[0]:
                    stitched = route_lab.path + arr.path[1:]
                else:
                    stitched = route_lab.path + arr.path

                dep_time   = arr.time + v_node.service_time
                dep_energy = arr.energy   # no energy used during service

                candidate_labels_at_v.append(RouteLabel(
                    distance         = route_lab.distance + arr.distance,
                    departure_time   = dep_time,
                    departure_energy = dep_energy,
                    path             = stitched,
                ))

        if not candidate_labels_at_v:
            # This leg is infeasible
            result = DPResult(
                feasible        = False,
                total_distance  = None,
                realised_path   = None,
                fail_leg        = leg_idx,
                direct_distance = direct_distance,
                station_detour  = 0.0,
            )
            if stats is not None:
                record = DPCallRecord(
                    skeleton          = skeleton,
                    feasible          = False,
                    direct_distance   = direct_distance,
                    realised_distance = None,
                    station_detour    = 0.0,
                    legs_solved       = call_legs_solved,
                    labels_generated  = call_labels_generated,
                    labels_kept       = call_labels_kept,
                    station_visits    = call_station_visits,
                    fail_leg          = leg_idx,
                )
                stats.total_legs_solved      += call_legs_solved
                stats.total_labels_generated += call_labels_generated
                stats.total_labels_kept      += call_labels_kept
                stats.total_station_visits   += call_station_visits
                stats._record_call(result, record)
            return result

        # Reduce to non-dominated labels at v
        L_v: List[RouteLabel] = []
        for lab in candidate_labels_at_v:
            L_v = _insert_route_label(L_v, lab)
        labels[v_id] = L_v

    # --- Extract best label at the final skeleton node ---
    final_id    = skeleton[-1]
    final_labels = labels.get(final_id, [])

    best = min(
        final_labels,
        key=lambda l: (l.distance, l.departure_time, -l.departure_energy),
    )

    station_detour = best.distance - direct_distance
    result = DPResult(
        feasible        = True,
        total_distance  = best.distance,
        realised_path   = best.path,
        fail_leg        = None,
        direct_distance = direct_distance,
        station_detour  = max(0.0, station_detour),
    )

    if stats is not None:
        record = DPCallRecord(
            skeleton          = skeleton,
            feasible          = True,
            direct_distance   = direct_distance,
            realised_distance = best.distance,
            station_detour    = result.station_detour,
            legs_solved       = call_legs_solved,
            labels_generated  = call_labels_generated,
            labels_kept       = call_labels_kept,
            station_visits    = call_station_visits,
            fail_leg          = None,
        )
        stats.total_legs_solved      += call_legs_solved
        stats.total_labels_generated += call_labels_generated
        stats.total_labels_kept      += call_labels_kept
        stats.total_station_visits   += call_station_visits
        stats._record_call(result, record)

    return result
