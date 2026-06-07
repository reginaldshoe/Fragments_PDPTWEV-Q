"""Build integrated fragment/network/model summaries."""
from __future__ import annotations
import argparse, json
from pathlib import Path
from typing import Any
from evrp_fragments.data import read_instance
from evrp_fragments.fragments import build_fragment_sets
from evrp_fragments.master.depot_arcs import raw_depot_arcs
from evrp_fragments.master.network import build_network

def default(o: Any):
    if isinstance(o, (set, frozenset)): return sorted(o)
    if isinstance(o, tuple): return list(o)
    return str(o)

def main(argv=None):
    ap = argparse.ArgumentParser(); ap.add_argument('--source', default='ev_fragmentsv3.py'); ap.add_argument('--instance', default='instances/c101C6_2.txt'); ap.add_argument('--max-base-path-len', type=int, default=18); ap.add_argument('--k-max', type=int, default=1); ap.add_argument('--force-exact-k', action='store_true'); ap.add_argument('--skip-gurobi', action='store_true'); ap.add_argument('--artefact', default='artefacts/integrated_summary.json')
    args = ap.parse_args(argv); data = read_instance(args.instance, args.source); fr = build_fragment_sets(data, args.max_base_path_len, args.source); depot = raw_depot_arcs(data); network = build_network(depot + fr.extended_undominated)
    payload = {'fragment_counts': fr.summary(), 'depot_arc_count': len(depot), 'network_node_count': len(network.node_id), 'network_arc_count': len(network.arcs), 'gurobi_model_built': False}
    if not args.skip_gurobi:
        from evrp_fragments.master.model import build_master_model
        master = build_master_model(data, fr.extended_undominated, args.k_max, args.force_exact_k); payload.update({'gurobi_model_built': True, 'model_var_count': master.model.NumVars, 'model_constr_count': master.model.NumConstrs, 'big_m': master.big_m})
    Path(args.artefact).parent.mkdir(parents=True, exist_ok=True); Path(args.artefact).write_text(json.dumps(payload, indent=2, default=default), encoding='utf-8'); print(json.dumps(payload, indent=2, default=default)); return 0
if __name__ == '__main__': raise SystemExit(main())
