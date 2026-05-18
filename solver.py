"""
solver.py
---------
Top-level entry point for the PDPTWEV-Q fragment-based solver.

This module owns the full pipeline:
    1. Parse instance file
    2. Enumerate and filter fragments  (preprocessing)
    3. Build and solve the MILP master problem  (K-sweep)
    4. Validate the best solution with a final DP pass
    5. Report timing and statistics

Usage
-----
Run directly:
    python solver.py instances/c101C6.txt

Or import and call solve():
    from solver import solve, SolverConfig
    result = solve(instance_path, config=SolverConfig(max_k=3, verbose=True))

Configuration
-------------
All tunable parameters live in SolverConfig.  Defaults replicate the
behaviour of the original ev_fragmentsv3.py script.
"""

from __future__ import annotations

import sys
from dataclasses import dataclass, field
from pathlib import Path
from time import perf_counter
from typing import Optional

from instance import Instance, read_instance
from fragment import (
    build_restricted_fragments,
    build_extended_fragments,
    fragment_stats,
)
from network import build_milp_network
from dp import DPStats, solve_route
from master import (
    SweepResult,
    SolveResult,
    solve_with_k_sweep,
    stitch_arc_sequences,
    skeleton_from_sequence,
    extract_routes,
)


# ---------------------------------------------------------------------------
# Configuration
# ---------------------------------------------------------------------------

@dataclass
class SolverConfig:
    """
    All tunable solver parameters in one place.

    Preprocessing
    -------------
    max_base_path_length : BFS depth limit for base-path enumeration.
                           Increase for larger instances (at exponential cost).

    MILP / K-sweep
    --------------
    max_k           : upper bound on vehicle count.  Defaults to |P| (number
                      of pickups), which is the theoretical maximum.
    early_stop      : stop the K-sweep once the objective stops improving.
                      Safe because adding vehicles cannot increase the optimum.
    optimise_k      : use native Gurobi K-optimisation instead of the sweep.
                      Experimental — validate on small instances first.
    vehicle_penalty : per-vehicle cost when optimise_k=True.  None = auto.
    gurobi_threads  : Gurobi thread count (1 for reproducibility).
    gurobi_output   : show Gurobi solver log.

    DP sub-problem
    --------------
    max_station_visits_per_leg : charging-station insertions allowed per leg.
    max_labels_per_node        : non-dominated label cap per mandatory node.

    Output
    ------
    verbose         : print per-K summaries and final report.
    debug           : print fragment and network diagnostics.
    """

    # preprocessing
    max_base_path_length: int = 18

    # MILP / K-sweep
    max_k:           Optional[int] = None   # None → |P|
    early_stop:      bool  = True
    optimise_k:      bool  = False
    vehicle_penalty: Optional[float] = None
    gurobi_threads:  int   = 1
    gurobi_output:   bool  = False

    # DP
    max_station_visits_per_leg: int = 3
    max_labels_per_node:        int = 50

    # output
    verbose: bool = True
    debug:   bool = False


# ---------------------------------------------------------------------------
# Result container
# ---------------------------------------------------------------------------

@dataclass
class SolverResult:
    """
    Complete output of a solve() call.

    instance_path  : path to the instance file
    instance       : parsed Instance object
    sweep          : SweepResult from the K-sweep (all K iterations + best)
    preprocess_time: wall-clock seconds for fragment enumeration and filtering
    total_time     : wall-clock seconds for preprocessing + solving
    validated      : True if the best solution passed the final DP validation
    """
    instance_path:   Path
    instance:        Instance
    sweep:           SweepResult
    preprocess_time: float
    total_time:      float
    validated:       bool = False


# ---------------------------------------------------------------------------
# Validation
# ---------------------------------------------------------------------------

def validate_solution(
    inst:         Instance,
    best:         SolveResult,
    config:       SolverConfig,
) -> bool:
    """
    Re-run the DP on every route in the best solution and confirm feasibility.

    This is a post-solve sanity check, not part of the optimisation.  It runs
    the DP with the same parameters as the callback and reports per-route
    distances.

    Returns True if every route is DP-feasible, False otherwise.
    """
    # Rebuild arc_by_id from the network (lightweight — no Gurobi involved)
    from fragment import build_restricted_fragments, build_extended_fragments
    _, ef = _rebuild_fragments(inst, config)
    arcs, _, _ = build_milp_network(inst, ef)
    arc_by_id  = {a.arc_id: a for a in arcs}

    val_stats    = DPStats()
    total_dp_dist = 0.0
    all_ok        = True

    print(f'\n--- DP validation of best solution (K={best.K}) ---')

    for r_idx, route in enumerate(best.routes):
        sequence = stitch_arc_sequences(route, arc_by_id)
        skeleton = skeleton_from_sequence(inst, sequence)

        result = solve_route(
            inst,
            skeleton,
            t0  = 0.0,
            E0  = inst.battery_capacity,
            max_station_visits_per_leg = config.max_station_visits_per_leg,
            max_labels_per_node        = config.max_labels_per_node,
            stats = val_stats,
        )

        print(f'\n  Route {r_idx + 1}:')
        print(f'    skeleton      : {list(skeleton)}')

        if result.feasible:
            print(f'    realised path : {list(result.realised_path)}')
            print(f'    DP distance   : {result.total_distance:.4f}')
            print(f'    station detour: {result.station_detour:.4f}')
            total_dp_dist += result.total_distance
        else:
            print(f'    DP INFEASIBLE at leg {result.fail_leg}')
            all_ok = False

    print(f'\n  Total DP distance : {total_dp_dist:.4f}')
    print(f'  Theta             : {best.theta:.4f}')
    print(f'  Objective         : {best.objective:.4f}')
    print(f'  Validation        : {"PASS" if all_ok else "FAIL"}')

    return all_ok


def _rebuild_fragments(inst: Instance, config: SolverConfig):
    """
    Re-enumerate fragments from scratch (used during post-solve validation).

    In a future refactor this could be cached and passed through, but for now
    re-enumeration is fast enough for small instances and keeps validate_solution
    self-contained.
    """
    rf, _ = build_restricted_fragments(inst, maxlen=config.max_base_path_length)
    ef    = build_extended_fragments(inst, rf)
    return rf, ef


# ---------------------------------------------------------------------------
# Reporting
# ---------------------------------------------------------------------------

def print_fragment_diagnostics(
    inst:  Instance,
    rf:    list,
    ef:    list,
    pruning_stats: dict,
) -> None:
    """Print fragment counts and pruning statistics."""
    print('\n--- Fragment diagnostics ---')
    print(f'  Pruning stats (base path enumeration):')
    for reason, count in sorted(pruning_stats.items(), key=lambda x: -x[1]):
        print(f'    {reason:<30} {count}')
    print(f'\n  Restricted fragments (after dominance filter): {len(rf)}')
    print(f'  Extended fragments   (after dominance filter): {len(ef)}')


def print_final_report(result: SolverResult) -> None:
    """Print the final timing and solution summary."""
    sweep = result.sweep
    best  = sweep.best

    print(f'\n{"="*60}')
    print(f'SOLVER REPORT: {result.instance_path.name}')
    print(f'{"="*60}')
    print(f'  Instance:')
    print(f'    pickups    : {len(result.instance.pickup_indices)}')
    print(f'    deliveries : {len(result.instance.delivery_indices)}')
    print(f'    stations   : {len(result.instance.station_indices)}')

    print(f'\n  Timing:')
    print(f'    preprocessing : {result.preprocess_time:.3f}s')
    print(f'    solving       : {sweep.total_solve_time:.3f}s')
    print(f'    total         : {result.total_time:.3f}s')

    if best is None:
        print('\n  No feasible solution found.')
        return

    print(f'\n  Best solution:')
    print(f'    K              : {best.K}')
    print(f'    objective      : {best.objective:.4f}')
    print(f'    route_distance : {best.route_distance:.4f}')
    print(f'    theta          : {best.theta:.4f}')
    print(f'    validated      : {result.validated}')

    print(f'\n  K-sweep summary:')
    for r in sweep.all_results:
        marker = ' <-- best' if r.K == best.K else ''
        s = r.dp_stats
        print(
            f'    K={r.K}: obj={r.objective:.4f}, '
            f'dp_calls={s.total_calls}, '
            f'feas_cuts={s.feasibility_cuts_added}, '
            f'opt_cuts={s.optimality_cuts_added}, '
            f'time={r.solve_time:.2f}s'
            f'{marker}'
        )

    print(f'\n  Aggregate DP statistics (all K):')
    total_calls = sum(r.dp_stats.total_calls            for r in sweep.all_results)
    total_fc    = sum(r.dp_stats.feasibility_cuts_added for r in sweep.all_results)
    total_oc    = sum(r.dp_stats.optimality_cuts_added  for r in sweep.all_results)
    total_legs  = sum(r.dp_stats.total_legs_solved      for r in sweep.all_results)
    total_lbls  = sum(r.dp_stats.total_labels_generated for r in sweep.all_results)
    total_sv    = sum(r.dp_stats.total_station_visits   for r in sweep.all_results)
    print(f'    dp_calls         : {total_calls}')
    print(f'    feasibility_cuts : {total_fc}')
    print(f'    optimality_cuts  : {total_oc}')
    print(f'    legs_solved      : {total_legs}')
    print(f'    labels_generated : {total_lbls}')
    print(f'    station_visits   : {total_sv}')
    print(f'{"="*60}\n')


# ---------------------------------------------------------------------------
# Main pipeline
# ---------------------------------------------------------------------------

def solve(
    instance_path: Path,
    config:        Optional[SolverConfig] = None,
) -> SolverResult:
    """
    Run the full EPDP fragment solver on the given instance file.

    Parameters
    ----------
    instance_path : path to the instance .txt file
    config        : SolverConfig (uses defaults if None)

    Returns
    -------
    SolverResult containing the best solution, all K-iteration results,
    timing, and validation status.
    """
    if config is None:
        config = SolverConfig()

    t_total_start = perf_counter()

    # ------------------------------------------------------------------
    # 1. Parse instance
    # ------------------------------------------------------------------
    if config.verbose:
        print(f'Reading instance: {instance_path.name}')

    inst = read_instance(instance_path)

    if config.verbose:
        print(
            f'  {len(inst.pickup_indices)} requests, '
            f'{len(inst.station_indices)} stations, '
            f'battery={inst.battery_capacity}, '
            f'capacity={inst.load_capacity}'
        )

    # ------------------------------------------------------------------
    # 2. Fragment preprocessing
    # ------------------------------------------------------------------
    if config.verbose:
        print('\nPreprocessing fragments...')

    t_pre_start = perf_counter()

    rf, pruning_stats = build_restricted_fragments(
        inst, maxlen=config.max_base_path_length
    )
    ef = build_extended_fragments(inst, rf)

    preprocess_time = perf_counter() - t_pre_start

    if config.verbose:
        print(f'  Restricted fragments (undominated): {len(rf)}')
        print(f'  Extended fragments   (undominated): {len(ef)}')
        print(f'  Preprocessing time: {preprocess_time:.3f}s')

    if config.debug:
        print_fragment_diagnostics(inst, rf, ef, pruning_stats)

    if not ef:
        print('ERROR: No extended fragments generated. Instance may be infeasible.')
        sweep = SweepResult(best=None, all_results=[], total_solve_time=0.0)
        return SolverResult(
            instance_path   = instance_path,
            instance        = inst,
            sweep           = sweep,
            preprocess_time = preprocess_time,
            total_time      = perf_counter() - t_total_start,
            validated       = False,
        )

    # ------------------------------------------------------------------
    # 3. Solve (K-sweep or native K-optimisation)
    # ------------------------------------------------------------------
    if config.verbose:
        mode = 'native K optimisation' if config.optimise_k else 'K-sweep'
        print(f'\nSolving ({mode})...')

    max_k = config.max_k if config.max_k is not None else len(inst.pickup_indices)

    sweep = solve_with_k_sweep(
        inst               = inst,
        extended_fragments = ef,
        max_k              = max_k,
        early_stop         = config.early_stop,
        gurobi_threads     = config.gurobi_threads,
        gurobi_output      = config.gurobi_output,
        verbose            = config.verbose,
    )

    total_time = perf_counter() - t_total_start

    # ------------------------------------------------------------------
    # 4. Validate best solution
    # ------------------------------------------------------------------
    validated = False
    if sweep.best is not None and config.verbose:
        validated = validate_solution(inst, sweep.best, config)

    # ------------------------------------------------------------------
    # 5. Report
    # ------------------------------------------------------------------
    result = SolverResult(
        instance_path   = instance_path,
        instance        = inst,
        sweep           = sweep,
        preprocess_time = preprocess_time,
        total_time      = total_time,
        validated       = validated,
    )

    if config.verbose:
        print_final_report(result)

    return result


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------

def main() -> None:
    if len(sys.argv) < 2:
        print('Usage: python solver.py <instance_file> [options]')
        print()
        print('Options:')
        print('  --max-k N        maximum number of vehicles (default: |P|)')
        print('  --no-early-stop  disable early stopping of K-sweep')
        print('  --optimise-k     use native Gurobi K-optimisation')
        print('  --debug          enable fragment and network diagnostics')
        print('  --gurobi-log     show Gurobi solver output')
        print()
        print('Example:')
        print('  python solver.py instances/c101C6.txt --max-k 3 --debug')
        sys.exit(1)

    instance_path = Path(sys.argv[1])
    if not instance_path.exists():
        print(f'ERROR: Instance file not found: {instance_path}')
        sys.exit(1)

    # --- Parse optional flags ---
    args = sys.argv[2:]
    config = SolverConfig()

    i = 0
    while i < len(args):
        arg = args[i]
        if arg == '--max-k' and i + 1 < len(args):
            config.max_k = int(args[i + 1])
            i += 2
        elif arg == '--no-early-stop':
            config.early_stop = False
            i += 1
        elif arg == '--optimise-k':
            config.optimise_k = True
            i += 1
        elif arg == '--debug':
            config.debug = True
            i += 1
        elif arg == '--gurobi-log':
            config.gurobi_output = True
            i += 1
        else:
            print(f'Unknown argument: {arg}')
            i += 1

    solve(instance_path, config=config)


if __name__ == '__main__':
    main()
