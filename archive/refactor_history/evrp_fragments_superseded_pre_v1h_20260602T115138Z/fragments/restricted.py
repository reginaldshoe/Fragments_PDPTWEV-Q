"""Restricted-fragment generation from base paths."""
from __future__ import annotations
from typing import Any
from ..resources import compute_Emin

def trim_base_path(data: dict[str, Any], base_path: list[int] | tuple[int, ...]) -> list[dict[str, Any]]:
    nodes = data['nodes']; d2p = data['d2p']; pickups = set(data['P']); deliveries = set(data['D'])
    d_switch = next((pos for pos, idx in enumerate(base_path) if nodes[idx][2] == 'cd'), None)
    if d_switch is None: return []
    pickup_seq = [nodes[i][0] for i in base_path[:d_switch] if i in pickups]
    delivery_seq = [nodes[i][0] for i in base_path[d_switch:] if i in deliveries]
    if not pickup_seq or not delivery_seq or len(pickup_seq) != len(delivery_seq): return []
    pos_by_sid = {nodes[idx][0]: pos for pos, idx in enumerate(base_path)}
    out = []
    for a in range(len(pickup_seq)):
        start_onboard = frozenset(pickup_seq[:a + 1]); start_pos = pos_by_sid[pickup_seq[a]]
        for b in range(len(delivery_seq)):
            kept_delivery = delivery_seq[:len(delivery_seq) - b]; removed_delivery = delivery_seq[len(delivery_seq) - b:]
            if not kept_delivery: continue
            end_pos = pos_by_sid[kept_delivery[-1]]
            if end_pos < start_pos: continue
            subseq = base_path[start_pos:end_pos + 1]; seq_sids = tuple(nodes[i][0] for i in subseq)
            end_onboard = {d2p[d] for d in removed_delivery if d in d2p}
            out.append({'seq': seq_sids, 'start_onboard': start_onboard, 'end_onboard': frozenset(end_onboard), 'contains_charge': any(nodes[i][1] == 'S' or nodes[i][2] == 'f' for i in subseq), 'min_start_energy': compute_Emin(data, seq_sids)})
    return out

def enumerate_fragments(data: dict[str, Any], base_paths: list[Any]) -> list[dict[str, Any]]:
    out = []
    for base_path in base_paths: out.extend(trim_base_path(data, base_path))
    return out
