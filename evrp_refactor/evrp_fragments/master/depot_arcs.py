"""Depot connection arcs for the fragment master network."""
from __future__ import annotations
from ..resources import compute_T_E_L, compute_distance


def raw_depot_arcs(data):

    nodes = data['nodes']
    depot_sid = nodes[0][0]
    out = []

    for p in data['P']:
        p_sid = nodes[p][0]

        # Build a fragment dict
        seq = (depot_sid, p_sid)
        Tf, Ef, Lf = compute_T_E_L(data, seq)
        Df = compute_distance(data, seq)

        # depot parameters (many are empty)
        out.append({
            'seq': seq,
            'Start': depot_sid,
            'End': p_sid,
            'start_onboard': frozenset(),
            'end_onboard': frozenset({p_sid}),
            'contains_charge': False,
            'Tf': Tf,
            'Ef': Ef,
            'Lf': Lf,
            'Df': Df,
            'Emin': 0.0,
            'LocsC': frozenset(),
            'ext_to': p_sid,
            'ext_station': None,
            'ext_delivery': data['p2d'].get(p_sid),
        })
    return out
