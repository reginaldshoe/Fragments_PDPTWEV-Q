"""Depot arc construction."""
from __future__ import annotations
from typing import Any
from ..resources import best_station_between, compute_Emin, compute_T_E_L, compute_distance, energy_ok_fullbatt

def raw_depot_arcs(data: dict[str, Any]) -> list[dict[str, Any]]:
    nodes = data['nodes']; out = []
    for p_idx in data['P']:
        pickup_sid = nodes[p_idx][0]; station = None
        if energy_ok_fullbatt(data, 'D0', pickup_sid): seq = ('D0', pickup_sid)
        else:
            station = best_station_between(data, 'D0', pickup_sid)
            if station is None: continue
            seq = ('D0', station, pickup_sid)
        tf, ef, lf = compute_T_E_L(data, seq)
        start_onboard = frozenset(); end_onboard = frozenset({pickup_sid}); locs_c = frozenset({pickup_sid})
        out.append({'seq': seq, 'Start': 'D0', 'End': pickup_sid, 'start_onboard': start_onboard, 'end_onboard': end_onboard, 'contains_charge': station is not None, 'min_start_energy': compute_Emin(data, seq), 'Tf': tf, 'Ef': ef, 'Lf': lf, 'Df': compute_distance(data, seq), 'Emin': compute_Emin(data, seq), 'LocsC': locs_c, 'dom_key': ('D0', pickup_sid, start_onboard, end_onboard, locs_c), 'ext_to': pickup_sid, 'ext_station': station, 'ext_delivery': data['p2d'].get(pickup_sid)})
    return out
