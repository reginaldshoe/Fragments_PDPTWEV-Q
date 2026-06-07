"""Smoke tests for v1d callback helpers."""
from __future__ import annotations
from ..evrp_fragments_v1d.callback.route_tools import extract_routes_from_solution, stitch_sid_sequence
from ..evrp_fragments_v1d.callback.diagnostics import state_transition_report


def test_route_extraction_and_stitching() -> None:
    arc_by_id = {
        0: {'Start': 'D0', 'End': 'C1', 'seq': ('D0', 'C1'), 'start_onboard': frozenset(), 'end_onboard': frozenset({'C1'})},
        1: {'Start': 'C1', 'End': 'D0', 'seq': ('C1', 'D0'), 'start_onboard': frozenset({'C1'}), 'end_onboard': frozenset()},
    }
    routes = extract_routes_from_solution([0, 1], arc_by_id)
    assert routes == [[0, 1]]
    assert stitch_sid_sequence(routes[0], arc_by_id) == ['D0', 'C1', 'D0']


def test_state_transition_report_flags_matches() -> None:
    arc_by_id = {
        0: {'Start': 'D0', 'End': 'C1', 'seq': ('D0', 'C1'), 'start_onboard': frozenset(), 'end_onboard': frozenset({'C1'})},
        1: {'Start': 'C1', 'End': 'D0', 'seq': ('C1', 'D0'), 'start_onboard': frozenset({'C1'}), 'end_onboard': frozenset()},
    }
    rows = state_transition_report([[0, 1]], arc_by_id)
    assert rows[0]['location_match'] is True
    assert rows[0]['onboard_match'] is True
