"""Integrated solver runner with diagnostic output."""
from __future__ import annotations
import argparse
from evrp_fragments.data import read_instance
from evrp_fragments.fragments import build_fragment_sets
from evrp_fragments.master.model import build_master_model
from evrp_fragments.callback.lazy_cuts import make_energy_callback
from evrp_fragments.callback.route_tools import extract_routes_from_solution, stitch_sid_sequence, strip_stations_from_sids, route_distance_from_sids
from evrp_fragments.callback.energy_dp import dp_route_min_dist


def _solution_diagnostics(data, master) -> dict:
    chosen = [aid for aid, var in master.x.items() if var.X > 0.5]
    routes = extract_routes_from_solution(chosen, master.arc_by_id, depot_u=master.depot_u)
    route_rows = []
    selected_arc_distance = sum(float(master.arc_by_id[aid].get('Df', 0.0)) for aid in chosen)
    dp_total = 0.0
    dp_all_ok = True
    disconnected_components = 0

    for route in routes:
        component_nodes = set()
        for aid in route:
            component_nodes.add(master.arc_by_id[aid]['u'])
            component_nodes.add(master.arc_by_id[aid]['v'])
        contains_depot = master.depot_u in component_nodes
        starts_at_depot = bool(route) and master.arc_by_id[route[0]]['u'] == master.depot_u
        ends_at_depot = bool(route) and master.arc_by_id[route[-1]]['v'] == master.depot_u
        connected_to_depot = contains_depot and starts_at_depot and ends_at_depot
        if not connected_to_depot:
            disconnected_components += 1

        sid_seq = stitch_sid_sequence(route, master.arc_by_id)
        skeleton = strip_stations_from_sids(data, sid_seq)
        ok, dp_dist, dp_path, fail_index = dp_route_min_dist(data, skeleton)
        if not ok:
            dp_all_ok = False
        if dp_dist is not None:
            dp_total += dp_dist

        route_rows.append({
            'arc_ids': route,
            'contains_depot_node': contains_depot,
            'starts_at_depot': starts_at_depot,
            'ends_at_depot': ends_at_depot,
            'connected_to_depot': connected_to_depot,
            'start_u': master.arc_by_id[route[0]]['u'] if route else None,
            'end_v': master.arc_by_id[route[-1]]['v'] if route else None,
            'depot_u': master.depot_u,
            'component_nodes': sorted(component_nodes),
            'sid_sequence': sid_seq,
            'skeleton': skeleton,
            'selected_arc_distance': sum(float(master.arc_by_id[aid].get('Df', 0.0)) for aid in route),
            'stitched_sid_distance': route_distance_from_sids(data, sid_seq),
            'dp_ok': ok,
            'dp_distance': dp_dist,
            'dp_path': dp_path,
            'fail_index': fail_index,
        })

    return {
        'chosen_arc_ids': chosen,
        'component_count': len(routes),
        'disconnected_component_count': disconnected_components,
        'selected_arc_distance': selected_arc_distance,
        'theta': master.theta.X if master.model.SolCount else None,
        'selected_plus_theta': selected_arc_distance + (master.theta.X if master.model.SolCount else 0.0),
        'dp_total_distance': dp_total,
        'dp_all_ok': dp_all_ok,
        'routes': route_rows,
    }


def main(argv=None):
    ap = argparse.ArgumentParser()
    ap.add_argument('--source', default='ev_fragmentsv3.py')
    ap.add_argument('--instance', default='instances/c101C6_2.txt')
    ap.add_argument('--max-base-path-len', type=int, default=18)
    ap.add_argument('--k-max', type=int, default=1)
    ap.add_argument('--force-exact-k', action='store_true')
    ap.add_argument('--use-callback', action='store_true')
    ap.add_argument('--diagnose', action='store_true')
    args = ap.parse_args(argv)

    data = read_instance(args.instance, args.source)
    fragments = build_fragment_sets(data, args.max_base_path_len, args.source)
    master = build_master_model(data, fragments.extended_undominated, args.k_max, args.force_exact_k)

    if args.use_callback:
        callback = make_energy_callback(
            master.x,
            master.arc_by_id,
            data,
            theta=master.theta,
            big_m=master.big_m,
            depot_u=master.depot_u,
        )
        master.model.optimize(callback)
    else:
        master.model.optimize()

    result = {
        'status': master.model.Status,
        'objective': master.model.ObjVal if master.model.SolCount else None,
        'sol_count': master.model.SolCount,
    }
    print(result)

    if args.diagnose and master.model.SolCount:
        print(_solution_diagnostics(data, master))
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
