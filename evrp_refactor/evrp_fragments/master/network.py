"""State-space network construction from master arc candidates."""
from __future__ import annotations


def build_network(ef_list):

    node_id = {}
    arcs = []

    # node for each unique location/onboard combination
    def get_node(loc_sid, onboard_fs):
        state = (loc_sid, onboard_fs)
        # assign unique combo with index
        if state not in node_id:
            node_id[state] = len(node_id)
        return node_id[state]

    # iterate through each fragment, create nodes based on start and end, then generate arc data
    for idx, f in enumerate(ef_list):
        u = get_node(f['Start'], f['start_onboard'])
        v = get_node(f['End'], f['end_onboard'])
        arcs.append({
            'seq': f['seq'],
            'Start': f['Start'],
            'End': f['End'],
            'start_onboard': f['start_onboard'],
            'end_onboard': f['end_onboard'],
            'id': idx,
            'u': u,
            'v': v,
            'Tf': float(f['Tf']),
            'Ef': float(f['Ef']),
            'Lf': float(f['Lf']),
            'Df': float(f['Df']),
            'Emin': float(f.get('Emin', f.get('min_start_energy', 0.0))),
            'LocsC': f.get('LocsC', frozenset()),
        })

    return node_id, arcs
