"""Smoke tests for v1c network construction."""
from __future__ import annotations
from ..evrp_fragments_v1c.master.network import build_network


def test_build_network_creates_state_nodes_and_arcs() -> None:
    frags = [
        {'seq': ('D0', 'C1'), 'Start': 'D0', 'End': 'C1', 'start_onboard': frozenset(), 'end_onboard': frozenset({'C1'}), 'Tf': 1, 'Ef': 1, 'Lf': 10, 'Df': 1, 'Emin': 1, 'LocsC': frozenset({'C1'})},
        {'seq': ('C1', 'D1'), 'Start': 'C1', 'End': 'D1', 'start_onboard': frozenset({'C1'}), 'end_onboard': frozenset(), 'Tf': 1, 'Ef': 2, 'Lf': 11, 'Df': 1, 'Emin': 1, 'LocsC': frozenset({'C1', 'D1'})},
    ]
    network = build_network(frags)
    assert len(network.arcs) == 2
    assert ('D0', frozenset()) in network.node_id
    assert ('C1', frozenset({'C1'})) in network.node_id
