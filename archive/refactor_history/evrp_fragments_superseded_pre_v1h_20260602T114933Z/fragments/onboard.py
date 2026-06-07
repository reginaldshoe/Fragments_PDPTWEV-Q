"""Onboard-state evolution for fragment boundaries."""
from __future__ import annotations
from typing import Any

def compute_onboard_states(seq: tuple[str, ...] | list[str], start_onboard: frozenset[str], data: dict[str, Any]) -> tuple[frozenset[str], frozenset[str]]:
    nodes = data['nodes']; d2p = data['d2p']; pickup_nodes = {nodes[i][0] for i in data['P']}
    load = set(start_onboard); start_on = set(load)
    for sid in seq:
        if sid in pickup_nodes: load.add(sid)
        elif sid in d2p and d2p[sid] in load: load.remove(d2p[sid])
    return frozenset(start_on), frozenset(load)
