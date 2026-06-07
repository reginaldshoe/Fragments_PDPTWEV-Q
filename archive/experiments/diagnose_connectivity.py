"""Offline connectivity diagnostics for the integrated EVRP fragment model.

This script does not require the optimisation/callback loop to run to completion.
It builds the fragment set and master candidate network, then diagnoses graph
connectivity and optionally inspects a supplied set of selected arc ids.

Example:
    python -m experiments.diagnose_connectivity --source ev_fragmentsv3.py --instance instances/c101C6_2.txt --max-base-path-len 18 --chosen 4 27 67 83 97 107 117
"""
from __future__ import annotations

import argparse
import json
from collections import defaultdict, deque
from pathlib import Path
from typing import Any

from evrp_fragments.data import read_instance
from evrp_fragments.fragments import build_fragment_sets
from evrp_fragments.master.depot_arcs import raw_depot_arcs
from evrp_fragments.master.network import build_network
from evrp_fragments.callback.route_tools import extract_routes_from_solution, stitch_sid_sequence, strip_stations_from_sids, route_distance_from_sids
from evrp_fragments.callback.energy_dp import dp_route_min_dist


def _json_default(value: Any) -> Any:
    if isinstance(value, (set, frozenset)):
        return sorted(value)
    if isinstance(value, tuple):
        return list(value)
    return str(value)


def _adjacency(arcs: list[dict[str, Any]]) -> tuple[dict[int, list[int]], dict[int, list[int]]]:
    out_by_node: dict[int, list[int]] = defaultdict(list)
    in_by_node: dict[int, list[int]] = defaultdict(list)
    for arc in arcs:
        out_by_node[arc['u']].append(arc['id'])
        in_by_node[arc['v']].append(arc['id'])
    return out_by_node, in_by_node


def _reachable_from(start: int, out_by_node: dict[int, list[int]], arc_by_id: dict[int, dict[str, Any]]) -> set[int]:
    seen = {start}
    q = deque([start])
    while q:
        node = q.popleft()
        for aid in out_by_node.get(node, []):
            nxt = arc_by_id[aid]['v']
            if nxt not in seen:
                seen.add(nxt)
                q.append(nxt)
    return seen


def _can_reach_depot(depot_u: int, in_by_node: dict[int, list[int]], arc_by_id: dict[int, dict[str, Any]]) -> set[int]:
    seen = {depot_u}
    q = deque([depot_u])
    while q:
        node = q.popleft()
        for aid in in_by_node.get(node, []):
            prev = arc_by_id[aid]['u']
            if prev not in seen:
                seen.add(prev)
                q.append(prev)
    return seen


def _tarjan_scc(nodes: list[int], out_by_node: dict[int, list[int]], arc_by_id: dict[int, dict[str, Any]]) -> list[list[int]]:
    index = 0
    stack: list[int] = []
    on_stack: set[int] = set()
    indices: dict[int, int] = {}
    lowlink: dict[int, int] = {}
    components: list[list[int]] = []

    def strongconnect(v: int) -> None:
        nonlocal index
        indices[v] = index
        lowlink[v] = index
        index += 1
        stack.append(v)
        on_stack.add(v)

        for aid in out_by_node.get(v, []):
            w = arc_by_id[aid]['v']
            if w not in indices:
                strongconnect(w)
                lowlink[v] = min(lowlink[v], lowlink[w])
            elif w in on_stack:
                lowlink[v] = min(lowlink[v], indices[w])

        if lowlink[v] == indices[v]:
            comp: list[int] = []
            while True:
                w = stack.pop()
                on_stack.remove(w)
                comp.append(w)
                if w == v:
                    break
            components.append(comp)

    for node in nodes:
        if node not in indices:
            strongconnect(node)
    return components


def _pickups_covered_by_arcs(arcs: list[dict[str, Any]], data: dict[str, Any]) -> set[str]:
    pickup_sids = {data['nodes'][i][0] for i in data['P']}
    covered: set[str] = set()
    for arc in arcs:
        seq = arc['seq']
        # Match the model's coverage convention: skip first node to avoid
        # boundary double-counting.
        for sid in seq[1:]:
            if sid in pickup_sids:
                covered.add(sid)
    return covered


def _component_summary(component_nodes: list[int], arcs: list[dict[str, Any]], data: dict[str, Any], depot_u: int) -> dict[str, Any]:
    node_set = set(component_nodes)
    internal_arcs = [arc for arc in arcs if arc['u'] in node_set and arc['v'] in node_set]
    entering_arcs = [arc for arc in arcs if arc['u'] not in node_set and arc['v'] in node_set]
    leaving_arcs = [arc for arc in arcs if arc['u'] in node_set and arc['v'] not in node_set]
    return {
        'nodes': sorted(node_set),
        'contains_depot': depot_u in node_set,
        'internal_arc_count': len(internal_arcs),
        'entering_arc_count': len(entering_arcs),
        'leaving_arc_count': len(leaving_arcs),
        'covered_pickups_internal': sorted(_pickups_covered_by_arcs(internal_arcs, data)),
        'sample_internal_arc_ids': [arc['id'] for arc in internal_arcs[:20]],
        'sample_entering_arc_ids': [arc['id'] for arc in entering_arcs[:20]],
        'sample_leaving_arc_ids': [arc['id'] for arc in leaving_arcs[:20]],
    }


def _diagnose_chosen(chosen: list[int], arc_by_id: dict[int, dict[str, Any]], data: dict[str, Any], depot_u: int) -> dict[str, Any]:
    routes = extract_routes_from_solution(chosen, arc_by_id, depot_u=depot_u)
    rows = []
    for route in routes:
        component_nodes = set()
        for aid in route:
            component_nodes.add(arc_by_id[aid]['u'])
            component_nodes.add(arc_by_id[aid]['v'])
        sid_seq = stitch_sid_sequence(route, arc_by_id)
        skeleton = strip_stations_from_sids(data, sid_seq)
        ok, dp_dist, dp_path, fail_index = dp_route_min_dist(data, skeleton)
        rows.append({
            'arc_ids': route,
            'component_nodes': sorted(component_nodes),
            'contains_depot_node': depot_u in component_nodes,
            'starts_at_depot': bool(route) and arc_by_id[route[0]]['u'] == depot_u,
            'ends_at_depot': bool(route) and arc_by_id[route[-1]]['v'] == depot_u,
            'sid_sequence': sid_seq,
            'skeleton': skeleton,
            'selected_arc_distance': sum(float(arc_by_id[aid].get('Df', 0.0)) for aid in route),
            'stitched_sid_distance': route_distance_from_sids(data, sid_seq),
            'dp_ok': ok,
            'dp_distance': dp_dist,
            'dp_path': dp_path,
            'fail_index': fail_index,
        })
    return {
        'chosen_arc_ids': chosen,
        'component_count': len(routes),
        'disconnected_component_count': sum(1 for row in rows if not (row['contains_depot_node'] and row['starts_at_depot'] and row['ends_at_depot'])),
        'components': rows,
    }


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description='Offline connectivity diagnostics for evrp_fragments.')
    parser.add_argument('--source', default='ev_fragmentsv3.py')
    parser.add_argument('--instance', default='instances/c101C6_2.txt')
    parser.add_argument('--max-base-path-len', type=int, default=18)
    parser.add_argument('--chosen', nargs='*', type=int, default=None, help='Optional selected arc ids to diagnose without solving.')
    parser.add_argument('--artefact', default='artefacts/connectivity_diagnostics.json')
    return parser


def main(argv: list[str] | None = None) -> int:
    args = build_parser().parse_args(argv)
    data = read_instance(args.instance, args.source)
    fragment_result = build_fragment_sets(data, args.max_base_path_len, args.source)
    candidate_arcs = raw_depot_arcs(data) + list(fragment_result.extended_undominated)
    network = build_network(candidate_arcs)
    arcs = network.arcs
    arc_by_id = {arc['id']: arc for arc in arcs}

    depot_sid = data['nodes'][0][0]
    depot_u = network.node_id.get((depot_sid, frozenset()))
    if depot_u is None:
        raise RuntimeError('Depot state node was not found in the candidate network.')

    out_by_node, in_by_node = _adjacency(arcs)
    all_nodes = sorted(network.node_id.values())
    reachable_from_depot = _reachable_from(depot_u, out_by_node, arc_by_id)
    can_reach_depot = _can_reach_depot(depot_u, in_by_node, arc_by_id)
    sccs = _tarjan_scc(all_nodes, out_by_node, arc_by_id)
    non_depot_cyclic_sccs = [
        comp for comp in sccs
        if depot_u not in comp and (
            len(comp) > 1 or any(arc_by_id[aid]['u'] == arc_by_id[aid]['v'] for node in comp for aid in out_by_node.get(node, []))
        )
    ]

    payload: dict[str, Any] = {
        'source': str(Path(args.source)),
        'instance': str(Path(args.instance)),
        'fragment_counts': fragment_result.summary(),
        'candidate_arc_count': len(arcs),
        'node_count': len(all_nodes),
        'depot_u': depot_u,
        'nodes_reachable_from_depot_count': len(reachable_from_depot),
        'nodes_that_can_reach_depot_count': len(can_reach_depot),
        'nodes_not_reachable_from_depot': sorted(set(all_nodes) - reachable_from_depot),
        'nodes_that_cannot_reach_depot': sorted(set(all_nodes) - can_reach_depot),
        'scc_count': len(sccs),
        'non_depot_cyclic_scc_count': len(non_depot_cyclic_sccs),
        'non_depot_cyclic_sccs': [
            _component_summary(comp, arcs, data, depot_u)
            for comp in non_depot_cyclic_sccs
        ],
    }

    if args.chosen is not None and len(args.chosen) > 0:
        payload['chosen_solution_diagnostic'] = _diagnose_chosen(args.chosen, arc_by_id, data, depot_u)

    out_path = Path(args.artefact)
    out_path.parent.mkdir(parents=True, exist_ok=True)
    out_path.write_text(json.dumps(payload, indent=2, default=_json_default), encoding='utf-8')
    print(json.dumps(payload, indent=2, default=_json_default))
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
