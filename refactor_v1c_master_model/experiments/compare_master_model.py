"""Build a v1c fragment/network/master-model summary."""
from __future__ import annotations
import argparse
import json
from pathlib import Path
from typing import Any
from ..evrp_fragments_v1c.fragments.base_paths import read_instance
from ..evrp_fragments_v1c.fragments.pipeline import build_fragment_sets
from ..evrp_fragments_v1c.master.depot_arcs import raw_depot_arcs
from ..evrp_fragments_v1c.master.network import build_network


def json_default(value: Any) -> Any:
    if isinstance(value, frozenset | set): return sorted(value)
    if isinstance(value, tuple): return list(value)
    return str(value)

def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description='Build v1c master-model summary.')
    parser.add_argument('--source', default='ev_fragmentsv3.py')
    parser.add_argument('--instance', default='instances/c101C6_2.txt')
    parser.add_argument('--max-base-path-len', type=int, default=18)
    parser.add_argument('--k-max', type=int, default=1)
    parser.add_argument('--force-exact-k', action='store_true')
    parser.add_argument('--skip-gurobi', action='store_true')
    parser.add_argument('--artefact', default='refactor_v1c_master_model/artefacts/master_model_summary.json')
    return parser

def main(argv: list[str] | None = None) -> int:
    args = build_parser().parse_args(argv)
    data = read_instance(args.instance, source_path=args.source)
    fragment_result = build_fragment_sets(data, max_base_path_len=args.max_base_path_len, source_path=args.source)
    depot_arcs = raw_depot_arcs(data)
    all_arcs = depot_arcs + list(fragment_result.extended_undominated)
    network = build_network(all_arcs)

    payload: dict[str, Any] = {
        'source': str(Path(args.source)),
        'instance': str(Path(args.instance)),
        'max_base_path_len': args.max_base_path_len,
        'k_max': args.k_max,
        'fragment_counts': fragment_result.summary(),
        'depot_arc_count': len(depot_arcs),
        'master_candidate_arc_count': len(all_arcs),
        'network_node_count': len(network.node_id),
        'network_arc_count': len(network.arcs),
        'gurobi_model_built': False,
    }

    if not args.skip_gurobi:
        from ..evrp_fragments_v1c.master.model import build_master_model
        master = build_master_model(data, fragment_result.extended_undominated, k_max=args.k_max, force_exact_k=args.force_exact_k)
        payload.update({
            'gurobi_model_built': True,
            'model_var_count': master.model.NumVars,
            'model_constr_count': master.model.NumConstrs,
            'big_m': master.big_m,
        })

    artefact_path = Path(args.artefact)
    artefact_path.parent.mkdir(parents=True, exist_ok=True)
    artefact_path.write_text(json.dumps(payload, indent=2, default=json_default), encoding='utf-8')
    print(json.dumps(payload, indent=2, default=json_default))
    return 0

if __name__ == '__main__':
    raise SystemExit(main())
