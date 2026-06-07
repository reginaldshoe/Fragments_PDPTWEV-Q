"""Run integrated fragments/master with the legacy ev_fragmentsv3 callback stack.

This runner is diagnostic. It tests whether the callback/DP implementation is the
source of divergence without copy/pasting legacy code into the package.
"""
from __future__ import annotations

import argparse

from evrp_fragments.data import read_instance
from evrp_fragments.fragments import build_fragment_sets
from evrp_fragments.master.model import build_master_model
from evrp_fragments.callback.legacy_runtime import make_legacy_callback
from evrp_fragments.callback.route_tools import extract_routes_from_solution, stitch_sid_sequence, strip_stations_from_sids, route_distance_from_sids
from evrp_fragments.callback.energy_dp import dp_route_min_dist


def _diagnose(data, master):
    chosen = [aid for aid, var in master.x.items() if var.X > 0.5]
    routes = extract_routes_from_solution(chosen, master.arc_by_id, depot_u=master.depot_u)
    rows = []
    for route in routes:
        sid_seq = stitch_sid_sequence(route, master.arc_by_id)
        skeleton = strip_stations_from_sids(data, sid_seq)
        ok, dp_dist, dp_path, fail = dp_route_min_dist(data, skeleton)
        rows.append({
            'arc_ids': route,
            'sid_sequence': sid_seq,
            'skeleton': skeleton,
            'selected_arc_distance': sum(float(master.arc_by_id[aid].get('Df', 0.0)) for aid in route),
            'stitched_sid_distance': route_distance_from_sids(data, sid_seq),
            'dp_ok_integrated_dp': ok,
            'dp_distance_integrated_dp': dp_dist,
            'dp_path_integrated_dp': dp_path,
            'fail_index_integrated_dp': fail,
        })
    return {
        'chosen_arc_ids': chosen,
        'selected_arc_distance': sum(float(master.arc_by_id[aid].get('Df', 0.0)) for aid in chosen),
        'theta': master.theta.X if master.model.SolCount else None,
        'routes_using_integrated_route_tools': rows,
    }


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description='Run integrated model with legacy callback loaded from ev_fragmentsv3.py.')
    parser.add_argument('--source', default='ev_fragmentsv3.py')
    parser.add_argument('--instance', default='instances/c101C6_2.txt')
    parser.add_argument('--max-base-path-len', type=int, default=18)
    parser.add_argument('--k-max', type=int, default=1)
    parser.add_argument('--force-exact-k', action='store_true')
    parser.add_argument('--diagnose', action='store_true')
    args = parser.parse_args(argv)

    data = read_instance(args.instance, args.source)
    fragments = build_fragment_sets(data, args.max_base_path_len, args.source)
    master = build_master_model(data, fragments.extended_undominated, args.k_max, args.force_exact_k)

    callback = make_legacy_callback(
        source_path=args.source,
        x_vars=master.x,
        arcs=master.arcs,
        node_id=master.node_id,
        depot_u=master.depot_u,
        data=data,
        theta=master.theta,
        big_m=master.big_m,
    )
    master.model.optimize(callback)

    result = {
        'status': master.model.Status,
        'objective': master.model.ObjVal if master.model.SolCount else None,
        'sol_count': master.model.SolCount,
    }
    print(result)
    if args.diagnose and master.model.SolCount:
        print(_diagnose(data, master))
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
