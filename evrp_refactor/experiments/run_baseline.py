"""Baseline experiment runner for the refactored EV fragment model.

This runner keeps the original `ev_fragmentsv3.py` algorithmic sequence intact:
read an instance, enumerate restricted fragments, extend them, build the master
model, solve over exact K, and validate the best solution with the energy DP.
"""
from __future__ import annotations

from time import perf_counter
import gurobipy as gp

from evrp_fragments.config import INSTANCE_DIR, DEBUG
from evrp_fragments.data import read_instance
from evrp_fragments.fragments.base_paths import enumerate_base_paths
from evrp_fragments.fragments.restricted import enumerate_fragments
from evrp_fragments.fragments.extended import extend_all_fragments
from evrp_fragments.fragments.metadata import dedup_exact, dedup_by_signature, attach_metadata
from evrp_fragments.fragments.dominance import dominance_filter
from evrp_fragments.fragments.diagnostics import stats_frags, stats_ext
from evrp_fragments.master.model import build_master_model
from evrp_fragments.callback.callback import callback
from evrp_fragments.callback.route_extraction import extract_routes_from_solution, stitch_sid_sequence
from evrp_fragments.callback.energy_dp import dp_route_min_dist
from evrp_fragments.node_utils import is_station


def build_fragment_pool(data, max_base_path_len=18):
    """Build restricted and extended fragments using the current baseline logic."""
    base_paths, pruned_paths = enumerate_base_paths(data, max_base_path_len)
    restricted_raw = enumerate_fragments(data, base_paths)

    restricted_dedup = dedup_exact(restricted_raw)
    restricted_meta = attach_metadata(data, restricted_dedup, exclude_last_ef=False)
    restricted_undominated = dominance_filter(restricted_meta)
    restricted_undominated = dedup_by_signature(restricted_undominated)

    extended_raw = extend_all_fragments(data, restricted_undominated)
    extended_dedup = dedup_exact(extended_raw)
    extended_meta = attach_metadata(data, extended_dedup, exclude_last_ef=True)
    extended_undominated = dominance_filter(extended_meta)
    extended_undominated = dedup_by_signature(extended_undominated)

    return {
        "base_paths": base_paths,
        "pruned_paths": pruned_paths,
        "restricted_raw": restricted_raw,
        "restricted_undominated": restricted_undominated,
        "extended_raw": extended_raw,
        "extended_undominated": extended_undominated,
    }


def solve_exact_k_sweep(data, extended_fragments):
    """Solve the current exact-K sweep and return the best payload found."""
    best_obj = float("inf")
    best_payload = None
    k_upper = len(data["P"])

    for k_value in range(1, k_upper + 1):
        print(f"\n--- solving with exact K = {k_value} ---")
        model, x_vars, arcs, node_id, depot_u, arc_by_id, theta, big_m = build_master_model(
            data, extended_fragments, K_max=k_value, force_exact_K=True
        )
        model.optimize(lambda m, where: callback(m, where, x_vars, arcs, node_id, depot_u, data, theta, big_m))

        if model.SolCount > 0 and model.ObjVal < best_obj:
            chosen = [aid for aid, var in x_vars.items() if var.X > 0.5]
            best_obj = model.ObjVal
            best_payload = {
                "K": k_value,
                "obj": model.ObjVal,
                "theta": theta.X,
                "chosen": chosen,
                "arc_by_id": arc_by_id,
            }
        elif DEBUG and model.Status == gp.GRB.INFEASIBLE:
            model.computeIIS()
            model.write("master_iis.ilp")
            print("IIS written to master_iis.ilp")

    return best_payload


def validate_best_solution(data, best_payload):
    """Run the existing post-solve DP validation and state-transition diagnostics."""
    if best_payload is None:
        print("No feasible solution found.")
        return

    print("\nBEST SOLUTION:")
    print("K =", best_payload["K"])
    print("Obj =", best_payload["obj"])
    print("Theta =", best_payload["theta"])

    chosen = best_payload["chosen"]
    arc_by_id = best_payload["arc_by_id"]
    routes = extract_routes_from_solution(chosen, arc_by_id)

    total_dp = 0.0
    print("\n--- DP validation of BEST solution ---")
    for route_index, route in enumerate(routes):
        sid_sequence = stitch_sid_sequence(route, arc_by_id)
        skeleton = [sid for sid in sid_sequence if not is_station(data, sid)]
        ok, dp_distance, dp_path, fail_index = dp_route_min_dist(data, skeleton)
        print(f"Route {route_index}: DP feasible={ok}, distance={dp_distance}, fail_index={fail_index}")
        if ok:
            total_dp += dp_distance

    print("\nTOTAL DP distance =", total_dp)
    print("Theta =", best_payload["theta"])
    print("Objective =", best_payload["obj"])

    print("\n=== STATE TRANSITIONS (ACTUAL CHOSEN ROUTE) ===")
    for route_index, route in enumerate(routes):
        print(f"Route {route_index}: arc IDs = {route}")
        for pos in range(len(route) - 1):
            current_arc = arc_by_id[route[pos]]
            next_arc = arc_by_id[route[pos + 1]]
            print(f"{current_arc['seq']} -> {next_arc['seq']}")
            print(" end_on :", current_arc["end_onboard"])
            print(" start_on:", next_arc["start_onboard"])
            print(" MATCH? :", current_arc["end_onboard"] == next_arc["start_onboard"])


def main(instance="c101C6_2.txt", max_base_path_len=18):
    data = read_instance(INSTANCE_DIR / instance)

    preprocess_start = perf_counter()
    print("INSTANCE:", instance, "horizon=", data["horizon"], "stations=", len(data["S"]), "pickups=", len(data["P"]))

    pools = build_fragment_pool(data, max_base_path_len=max_base_path_len)
    extended_fragments = pools["extended_undominated"]
    print("Num arcs:", len(extended_fragments))

    preprocess_end = perf_counter()

    if DEBUG:
        print(pools["pruned_paths"])
        print("RF stats pre dominance filter")
        stats_frags(pools["restricted_raw"])
        print("RF stats post dominance filter")
        stats_frags(pools["restricted_undominated"])
        print("EF stats pre dominance filtering")
        stats_ext(pools["extended_raw"])
        print("EF stats post dominance filtering")
        stats_ext(pools["extended_undominated"])

    solve_start = perf_counter()
    best_payload = solve_exact_k_sweep(data, extended_fragments)
    solve_end = perf_counter()

    print("Preprocessing time:", preprocess_end - preprocess_start)
    print("Solve time:", solve_end - solve_start)
    print("Total time:", (preprocess_end - preprocess_start) + (solve_end - solve_start))

    validate_best_solution(data, best_payload)


if __name__ == "__main__":
    main()
