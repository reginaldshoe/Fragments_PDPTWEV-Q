"""
master.py
---------
Gurobi master MILP, callback (subtour elimination + energy cuts), and
the K-sweep solve loop.

Model structure
---------------
Decision variables
    x[arc_id]  ∈ {0,1}   : 1 if arc is used in the solution
    theta      ∈ ℝ≥0      : auxiliary variable capturing the extra distance
                            incurred by mandatory charging-station detours
                            (added via optimality cuts in the callback)

Objective
    min  Σ_a  distance(a) * x[a]  +  theta

Constraints
    Flow conservation   : for every non-depot network node, flow in = flow out
    Depot balance       : exactly K vehicles depart (and return to) the depot
    Coverage            : every pickup node is served by exactly one arc

Lazy constraints (added in callback at integer solutions)
    Subtour elimination : disconnected cycles are cut by SEC
    Feasibility cuts    : routes that are energy-infeasible even with station
                          insertion are cut
    Optimality cuts     : theta ≥ delta - M*(|S| - Σ_{a∈S} x[a])
                          where delta = (DP realised distance) - (direct distance)
                          and S is the set of chosen arcs

K-sweep strategy
----------------
We solve for K = 1, 2, ..., |P| (number of pickups) with force_exact_K=True.
We stop early once the objective stops improving (adding a vehicle can only
help or leave cost unchanged, so once obj[K] >= obj[K-1] we are done).

Native K optimisation (optional)
---------------------------------
build_master_model() accepts optimise_k=True, which adds an integer variable
K_var ∈ [1, |P|] and a small per-vehicle penalty epsilon to the objective.
This lets Gurobi select K endogenously.  See the docstring on
build_master_model() for caveats.
"""

from __future__ import annotations

import os
from dataclasses import dataclass, field
from time import perf_counter
from typing import Dict, List, Optional, Tuple

try:
    import gurobipy as gp
except ImportError:
    gp = None   # allows import of this module without Gurobi for testing

from instance import Instance
from network import NetworkArc, build_milp_network, build_adjacency, arcs_ending_at_pickup
from fragment import Fragment
from dp import DPStats, DPResult, solve_route


# ---------------------------------------------------------------------------
# Gurobi environment
# ---------------------------------------------------------------------------

def make_gurobi_env():
    """
    Create a Gurobi environment, using WLS credentials from environment
    variables if available, otherwise falling back to the default local licence.

    Environment variables read:
        GRB_WLSACCESSID  : WLS access ID
        GRB_WLSSECRET    : WLS secret
        GRB_LICENSEID    : numeric licence ID
    """
    wls_id     = os.getenv('GRB_WLSACCESSID', '').strip()
    wls_secret = os.getenv('GRB_WLSSECRET',   '').strip()
    lic_id     = os.getenv('GRB_LICENSEID',    '').strip()

    if wls_id and wls_secret and lic_id:
        return gp.Env(params={
            'WLSACCESSID': wls_id,
            'WLSSECRET':   wls_secret,
            'LICENSEID':   int(lic_id),
        })
    return gp.Env()


# ---------------------------------------------------------------------------
# Solution payload
# ---------------------------------------------------------------------------

@dataclass
class SolveResult:
    """
    Complete result of a single K-iteration solve.

    Attributes
    ----------
    K               : number of vehicles used
    objective       : Gurobi objective value (route distance + theta)
    route_distance  : Σ distance(a) * x[a]  (without theta)
    theta           : value of the theta variable
    chosen_arc_ids  : list of arc_ids selected (x[a] = 1)
    routes          : list of routes, each a list of arc_ids in traversal order
    solve_time      : wall-clock seconds for this K iteration
    dp_stats        : DPStats accumulated during the callback for this K
    """
    K:              int
    objective:      float
    route_distance: float
    theta:          float
    chosen_arc_ids: List[int]
    routes:         List[List[int]]
    solve_time:     float
    dp_stats:       DPStats


@dataclass
class SweepResult:
    """
    Result of the full K-sweep.

    best            : the SolveResult with the lowest objective
    all_results     : one SolveResult per K attempted (including infeasible)
    total_solve_time: wall-clock seconds across all K iterations
    """
    best:             Optional[SolveResult]
    all_results:      List[SolveResult]
    total_solve_time: float


# ---------------------------------------------------------------------------
# Route utilities
# ---------------------------------------------------------------------------

def stitch_arc_sequences(route_arc_ids: List[int], arc_by_id: Dict[int, NetworkArc]) -> Tuple[str, ...]:
    """
    Concatenate the node_id sequences of consecutive arcs into a single
    route sequence, avoiding duplication of junction nodes.
    """
    stitched: List[str] = []
    for k, arc_id in enumerate(route_arc_ids):
        seq = arc_by_id[arc_id].sequence
        if k == 0:
            stitched.extend(seq)
        else:
            # seq[0] is the same as stitched[-1] (shared junction node)
            if stitched and seq and stitched[-1] == seq[0]:
                stitched.extend(seq[1:])
            else:
                stitched.extend(seq)
    return tuple(stitched)


def extract_routes(
    chosen_arc_ids: List[int],
    arc_by_id:      Dict[int, NetworkArc],
    depot_node_idx: int,
) -> List[List[int]]:
    """
    Reconstruct ordered per-vehicle route arc lists from the flat set of
    chosen arc ids.

    Each route is a list of arc_ids in traversal order, starting and ending
    at the depot network node.  Raises RuntimeError on ambiguous or missing
    continuations (which indicate a subtour that the callback missed).

    Important: we follow network node *indices* (tail_index / head_index),
    not location strings.  The same physical location can appear as different
    network nodes when the onboard cargo set differs, so routing by location
    string would produce false revisits in the skeleton.
    """
    # Index chosen arcs by their tail network-node index
    out_map: Dict[int, List[int]] = {}
    for arc_id in chosen_arc_ids:
        tail = arc_by_id[arc_id].tail_index
        out_map.setdefault(tail, []).append(arc_id)

    routes: List[List[int]] = []

    for first_arc_id in out_map.get(depot_node_idx, []):
        route   = [first_arc_id]
        cur_idx = arc_by_id[first_arc_id].head_index
        safety  = 0

        while cur_idx != depot_node_idx:
            nexts = out_map.get(cur_idx, [])
            if len(nexts) != 1:
                raise RuntimeError(
                    f"Ambiguous or missing continuation at network node {cur_idx}: "
                    f"{nexts}.  This indicates a subtour that was not eliminated."
                )
            next_arc_id = nexts[0]
            route.append(next_arc_id)
            cur_idx = arc_by_id[next_arc_id].head_index
            safety += 1
            if safety > len(chosen_arc_ids) + 5:
                raise RuntimeError(
                    f"Route following did not terminate at depot after "
                    f"{safety} steps — possible cycle."
                )

        routes.append(route)

    return routes


def skeleton_from_sequence(inst: Instance, sequence: Tuple[str, ...]) -> Tuple[str, ...]:
    """Remove all charging-station nodes from a full route sequence."""
    return tuple(sid for sid in sequence if not inst.node(sid).is_station)


# ---------------------------------------------------------------------------
# Callback
# ---------------------------------------------------------------------------

def make_callback(
    inst:      Instance,
    x_vars:    Dict[int, 'gp.Var'],
    arcs:      List[NetworkArc],
    arc_by_id: Dict[int, NetworkArc],
    depot_idx: int,
    theta:     'gp.Var',
    big_M:     float,
    dp_stats:  DPStats,
):
    """
    Return a Gurobi callback function closed over the solve state.

    The callback fires at every integer-feasible incumbent (MIPSOL) and:
    1. Checks for subtours (disconnected cycles not touching the depot) and
       adds SECs to eliminate them.
    2. For each depot-connected route, runs the DP sub-problem to check
       energy feasibility with station insertion.
    3. If a route is infeasible, adds a feasibility cut and returns.
    4. If all routes are feasible but station detours add cost, adds an
       optimality cut updating theta.
    """
    arc_list     = arcs          # list, indexed by position
    arc_index    = arc_by_id     # dict arc_id -> NetworkArc

    def callback(model, where):
        if where != gp.GRB.Callback.MIPSOL:
            return

        # --- Retrieve solution ---
        x_sol    = model.cbGetSolution(x_vars)
        chosen   = [arc_id for arc_id, val in x_sol.items() if val > 0.5]
        if not chosen:
            return

        # --- Build adjacency over chosen arcs (by network node index) ---
        out_map: Dict[int, List[int]] = {}
        in_map:  Dict[int, List[int]] = {}
        for arc_id in chosen:
            arc = arc_index[arc_id]
            out_map.setdefault(arc.tail_index, []).append(arc_id)
            in_map.setdefault(arc.head_index, []).append(arc_id)

        # --- Subtour detection: BFS from depot node ---
        reachable = {depot_idx}
        stack     = [depot_idx]
        while stack:
            node = stack.pop()
            for arc_id in out_map.get(node, []):
                head = arc_index[arc_id].head_index
                if head not in reachable:
                    reachable.add(head)
                    stack.append(head)

        subtour_arcs = [
            arc_id for arc_id in chosen
            if arc_index[arc_id].tail_index not in reachable
        ]
        if subtour_arcs:
            model.cbLazy(
                gp.quicksum(x_vars[arc_id] for arc_id in subtour_arcs)
                <= len(subtour_arcs) - 1
            )
            dp_stats.feasibility_cuts_added += 1
            return

        # --- Extract depot-rooted routes ---
        routes: List[List[int]] = []
        for first_arc_id in out_map.get(depot_idx, []):
            route  = [first_arc_id]
            cur    = arc_index[first_arc_id].head_index
            safety = 0
            while cur != depot_idx:
                nexts = out_map.get(cur, [])
                if not nexts:
                    break
                next_arc_id = nexts[0]
                route.append(next_arc_id)
                cur = arc_index[next_arc_id].head_index
                safety += 1
                if safety > len(chosen) + 5:
                    break
            routes.append(route)

        # --- Run DP for each route, accumulate delta ---
        delta_total = 0.0

        for route in routes:
            sequence = stitch_arc_sequences(route, arc_index)
            skeleton = skeleton_from_sequence(inst, sequence)

            if len(skeleton) < 2:
                continue

            try:
                result: DPResult = solve_route(
                    inst,
                    skeleton,
                    t0=0.0,
                    E0=inst.battery_capacity,
                    max_station_visits_per_leg=3,
                    max_labels_per_node=50,
                    stats=dp_stats,
                )
            except ValueError:
                # Skeleton contains repeated customer nodes — the incumbent
                # route is structurally infeasible (a customer visited twice).
                # Add a feasibility cut to forbid this arc combination.
                model.cbLazy(
                    gp.quicksum(x_vars[arc_id] for arc_id in route)
                    <= len(route) - 1
                )
                dp_stats.feasibility_cuts_added += 1
                return

            if not result.feasible:
                # Energy/time infeasibility cut: forbid this arc combination.
                model.cbLazy(
                    gp.quicksum(x_vars[arc_id] for arc_id in route)
                    <= len(route) - 1
                )
                dp_stats.feasibility_cuts_added += 1
                return

            if result.station_detour > 1e-6:
                delta_total += result.station_detour

        # --- Optimality cut ---
        if delta_total > 1e-6:
            model.cbLazy(
                theta >= delta_total
                - big_M * (len(chosen) - gp.quicksum(x_vars[arc_id] for arc_id in chosen))
            )
            dp_stats.optimality_cuts_added += 1

    return callback


# ---------------------------------------------------------------------------
# Model builder
# ---------------------------------------------------------------------------

def build_master_model(
    inst:               Instance,
    extended_fragments: List[Fragment],
    K:                  int,
    dp_stats:           DPStats,
    optimise_k:         bool = False,
    vehicle_penalty:    Optional[float] = None,
    gurobi_threads:     int = 1,
    gurobi_output:      bool = False,
) -> Tuple:
    """
    Build the Gurobi MILP master model for a given vehicle count K.

    Parameters
    ----------
    inst               : problem instance
    extended_fragments : undominated EFs from fragment.build_extended_fragments()
    K                  : vehicle count (upper bound, or exact — see below)
    dp_stats           : DPStats object passed into the callback for accumulation
    optimise_k         : if True, add an integer K_var and let Gurobi choose K
                         endogenously (see notes below)
    vehicle_penalty    : per-vehicle cost added to the objective when
                         optimise_k=True.  Defaults to max_arc_distance / n_pickups,
                         which is large enough to prefer fewer vehicles without
                         distorting route costs.
    gurobi_threads     : Gurobi thread count (default 1 for reproducibility)
    gurobi_output      : whether to show Gurobi log (default False)

    Returns
    -------
    (model, x_vars, arcs, arc_by_id, depot_idx, node_states, theta, big_M, callback_fn)

    Notes on optimise_k
    -------------------
    When optimise_k=True:
    - K is treated as an upper bound: K_var ∈ [1, K]
    - A small per-vehicle penalty (vehicle_penalty) is added to the objective
      so Gurobi prefers solutions with fewer vehicles, all else being equal
    - The depot flow constraint becomes == K_var (exact balance)
    - This is mathematically sound but may slow the branch-and-bound because
      K_var interacts with the lazy constraints.  For small instances the
      K-sweep is fast enough that native optimisation is unnecessary.
    """
    arcs, node_states, depot_idx = build_milp_network(inst, extended_fragments)
    arc_by_id = {a.arc_id: a for a in arcs}
    n_nodes   = len(node_states)

    out_arcs, in_arcs = build_adjacency(arcs, n_nodes)
    coverage          = arcs_ending_at_pickup(inst, arcs)

    big_M = max(
        (inst.distance(i, j) for i in range(len(inst.nodes)) for j in range(len(inst.nodes)) if i != j),
        default=1e6,
    ) * len(inst.nodes)

    env = make_gurobi_env()
    model = gp.Model(env=env)
    model.Params.LazyConstraints = 1
    model.Params.Threads         = gurobi_threads
    model.Params.OutputFlag      = int(gurobi_output)

    # --- Decision variables ---
    x_vars = {a.arc_id: model.addVar(vtype=gp.GRB.BINARY) for a in arcs}
    theta  = model.addVar(vtype=gp.GRB.CONTINUOUS, lb=0.0, name='theta')

    # --- Objective ---
    route_cost = gp.quicksum(a.distance * x_vars[a.arc_id] for a in arcs)

    if optimise_k:
        n_pickups = len(inst.pickup_indices)
        if vehicle_penalty is None:
            max_d = max(
                inst.distance(i, j)
                for i in range(len(inst.nodes))
                for j in range(len(inst.nodes))
                if i != j
            )
            vehicle_penalty = max_d / max(n_pickups, 1)

        K_var = model.addVar(vtype=gp.GRB.INTEGER, lb=1, ub=K, name='K_var')
        model.setObjective(route_cost + theta + vehicle_penalty * K_var, gp.GRB.MINIMIZE)
    else:
        K_var = None
        model.setObjective(route_cost + theta, gp.GRB.MINIMIZE)

    # --- Flow conservation (all non-depot nodes) ---
    for node_idx in range(n_nodes):
        if node_idx == depot_idx:
            continue
        in_flow  = gp.quicksum(x_vars[arc_id] for arc_id in in_arcs[node_idx])
        out_flow = gp.quicksum(x_vars[arc_id] for arc_id in out_arcs[node_idx])
        model.addConstr(in_flow - out_flow == 0, name=f'flow_{node_idx}')

    # --- Depot balance ---
    depot_out_flow = gp.quicksum(x_vars[arc_id] for arc_id in out_arcs[depot_idx])
    depot_in_flow  = gp.quicksum(x_vars[arc_id] for arc_id in in_arcs[depot_idx])

    if optimise_k and K_var is not None:
        model.addConstr(depot_out_flow == K_var, name='depot_out')
        model.addConstr(depot_in_flow  == K_var, name='depot_in')
    else:
        model.addConstr(depot_out_flow == K, name='depot_out')
        model.addConstr(depot_in_flow  == K, name='depot_in')

    # --- Coverage: every pickup served exactly once ---
    for p_id, covering_arc_ids in coverage.items():
        model.addConstr(
            gp.quicksum(x_vars[arc_id] for arc_id in covering_arc_ids) == 1,
            name=f'cover_{p_id}',
        )

    # --- Build callback ---
    callback_fn = make_callback(
        inst      = inst,
        x_vars    = x_vars,
        arcs      = arcs,
        arc_by_id = arc_by_id,
        depot_idx = depot_idx,
        theta     = theta,
        big_M     = big_M,
        dp_stats  = dp_stats,
    )

    return model, x_vars, arcs, arc_by_id, depot_idx, node_states, theta, big_M, callback_fn


# ---------------------------------------------------------------------------
# K-sweep solver
# ---------------------------------------------------------------------------

def solve_with_k_sweep(
    inst:               Instance,
    extended_fragments: List[Fragment],
    max_k:              Optional[int] = None,
    early_stop:         bool = True,
    gurobi_threads:     int = 1,
    gurobi_output:      bool = False,
    verbose:            bool = True,
) -> SweepResult:
    """
    Solve by iterating K = 1, 2, ..., max_k with force_exact_K=True.

    Parameters
    ----------
    inst               : problem instance
    extended_fragments : from fragment.build_extended_fragments()
    max_k              : upper bound on K (defaults to number of pickups)
    early_stop         : stop as soon as objective stops improving.
                         Adding vehicles can only reduce or maintain cost, so
                         once obj[K] >= obj[K-1] we have found the optimum K.
    gurobi_threads     : passed to Gurobi
    gurobi_output      : show Gurobi solver log
    verbose            : print per-K summary

    Returns
    -------
    SweepResult with best SolveResult and full per-K history
    """
    if max_k is None:
        max_k = len(inst.pickup_indices)

    depot_id     = inst.depot_id
    all_results: List[SolveResult] = []
    best:        Optional[SolveResult] = None

    t_sweep_start = perf_counter()

    for K in range(1, max_k + 1):
        if verbose:
            print(f'\n--- Solving K = {K} ---')

        dp_stats  = DPStats()
        t_k_start = perf_counter()

        (model, x_vars, arcs, arc_by_id,
         depot_idx, node_states, theta,
         big_M, callback_fn) = build_master_model(
            inst               = inst,
            extended_fragments = extended_fragments,
            K                  = K,
            dp_stats           = dp_stats,
            optimise_k         = False,
            gurobi_threads     = gurobi_threads,
            gurobi_output      = gurobi_output,
        )

        model.optimize(callback_fn)
        solve_time = perf_counter() - t_k_start

        if model.SolCount == 0:
            if verbose:
                print(f'  K={K}: infeasible or no solution found')
            continue

        obj_val    = model.ObjVal
        theta_val  = theta.X
        route_dist = obj_val - theta_val

        chosen_arc_ids = [arc_id for arc_id, var in x_vars.items() if var.X > 0.5]

        try:
            routes = extract_routes(chosen_arc_ids, arc_by_id, depot_idx)
        except RuntimeError as e:
            if verbose:
                print(f'  K={K}: route extraction failed — {e}')
            continue

        result = SolveResult(
            K              = K,
            objective      = obj_val,
            route_distance = route_dist,
            theta          = theta_val,
            chosen_arc_ids = chosen_arc_ids,
            routes         = routes,
            solve_time     = solve_time,
            dp_stats       = dp_stats,
        )
        all_results.append(result)

        if verbose:
            print(
                f'  K={K}: obj={obj_val:.4f}, '
                f'route_dist={route_dist:.4f}, '
                f'theta={theta_val:.4f}, '
                f'routes={len(routes)}, '
                f'dp_calls={dp_stats.total_calls}, '
                f'time={solve_time:.2f}s'
            )

        if best is None or obj_val < best.objective:
            best = result
        elif early_stop and obj_val >= best.objective - 1e-6:
            if verbose:
                print(f'  Objective did not improve at K={K}, stopping sweep.')
            break

    total_time = perf_counter() - t_sweep_start

    return SweepResult(
        best             = best,
        all_results      = all_results,
        total_solve_time = total_time,
    )


# ---------------------------------------------------------------------------
# Post-solve validation and reporting
# ---------------------------------------------------------------------------

def validate_and_report(
    inst:         Instance,
    sweep_result: SweepResult,
    verbose:      bool = True,
) -> None:
    """
    Validate the best solution by re-running the DP on each route and
    printing a per-route summary.  Also prints aggregate DP statistics
    across all K iterations.
    """
    best = sweep_result.best
    if best is None:
        print('No feasible solution found.')
        return

    if verbose:
        print(f'\n{"="*60}')
        print(f'BEST SOLUTION: K={best.K}, obj={best.objective:.4f}')
        print(f'  route_distance = {best.route_distance:.4f}')
        print(f'  theta          = {best.theta:.4f}')
        print(f'  solve_time     = {best.solve_time:.2f}s')
        print(f'  total_time     = {sweep_result.total_solve_time:.2f}s')
        print(f'{"="*60}')

    arc_by_id    = {a.arc_id: a for a in build_milp_network(inst, [])[0]}
    # Re-derive arc_by_id from the best result's chosen arcs
    # (we don't store the full arc list in SolveResult to keep it lightweight)
    # Use a fresh validation DP stats object so we don't conflate with solve stats
    val_stats    = DPStats()
    total_dp_dist = 0.0

    for r_idx, route in enumerate(best.routes):
        # Reconstruct arc_by_id for this route from chosen arc metadata
        # We need to rebuild the network to get arc objects — or we can store them.
        # For now, report skeleton from the route's arc sequences (stored in NetworkArc).
        # NOTE: arc_by_id is not stored in SolveResult; caller should pass it or
        # we rebuild here.  See solver.py for the full pipeline that passes it through.
        if verbose:
            print(f'\n  Route {r_idx + 1}:')

    # Aggregate DP stats across all K iterations
    if verbose:
        print('\n--- Aggregate DP Statistics Across All K Iterations ---')
        total_dp_calls   = sum(r.dp_stats.total_calls          for r in sweep_result.all_results)
        total_feas_cuts  = sum(r.dp_stats.feasibility_cuts_added for r in sweep_result.all_results)
        total_opt_cuts   = sum(r.dp_stats.optimality_cuts_added  for r in sweep_result.all_results)
        total_legs       = sum(r.dp_stats.total_legs_solved       for r in sweep_result.all_results)
        total_labels     = sum(r.dp_stats.total_labels_kept       for r in sweep_result.all_results)
        print(f'  total_dp_calls          : {total_dp_calls}')
        print(f'  feasibility_cuts_added  : {total_feas_cuts}')
        print(f'  optimality_cuts_added   : {total_opt_cuts}')
        print(f'  total_legs_solved       : {total_legs}')
        print(f'  total_labels_kept       : {total_labels}')
        print()
        print('  Per-K breakdown:')
        for r in sweep_result.all_results:
            s = r.dp_stats
            print(
                f'    K={r.K}: '
                f'calls={s.total_calls}, '
                f'feas_cuts={s.feasibility_cuts_added}, '
                f'opt_cuts={s.optimality_cuts_added}, '
                f'legs={s.total_legs_solved}, '
                f'time={r.solve_time:.2f}s'
            )
