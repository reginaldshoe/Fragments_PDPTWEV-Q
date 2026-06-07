"""State-space network construction."""
from __future__ import annotations
from dataclasses import dataclass
from typing import Any

@dataclass(frozen=True)
class NetworkBuildResult:
    node_id: dict[tuple[str, frozenset[str]], int]
    arcs: list[dict[str, Any]]

def build_network(fragment_arcs: list[dict[str, Any]]) -> NetworkBuildResult:
    node_id = {}; arcs = []
    def get_node(loc_sid: str, onboard_fs: frozenset[str]) -> int:
        state = (loc_sid, onboard_fs)
        if state not in node_id: node_id[state] = len(node_id)
        return node_id[state]
    for idx, frag in enumerate(fragment_arcs):
        start_onboard = frozenset(frag['start_onboard']); end_onboard = frozenset(frag['end_onboard'])
        arcs.append({'id': idx, 'u': get_node(frag['Start'], start_onboard), 'v': get_node(frag['End'], end_onboard), 'seq': tuple(frag['seq']), 'Start': frag['Start'], 'End': frag['End'], 'start_onboard': start_onboard, 'end_onboard': end_onboard, 'Tf': float(frag['Tf']), 'Ef': float(frag['Ef']), 'Lf': float(frag['Lf']), 'Df': float(frag['Df']), 'Emin': float(frag.get('Emin', frag.get('min_start_energy', 0.0))), 'LocsC': frag.get('LocsC', frozenset())})
    return NetworkBuildResult(node_id, arcs)
