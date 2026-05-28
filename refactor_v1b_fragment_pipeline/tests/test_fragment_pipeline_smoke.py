"""Smoke tests for v1b fragment-pipeline helpers."""

from __future__ import annotations

from evrp_fragments_v1b.fragments.metadata import dedup_exact
from evrp_fragments_v1b.fragments.pipeline import summarise_fragments


def test_dedup_exact_keeps_unique_boundary_state() -> None:
    frags = [
        {"seq": ("C1", "C2"), "start_onboard": frozenset({"C1"}), "end_onboard": frozenset()},
        {"seq": ("C1", "C2"), "start_onboard": frozenset({"C1"}), "end_onboard": frozenset()},
        {"seq": ("C1", "C2"), "start_onboard": frozenset({"C1", "C3"}), "end_onboard": frozenset({"C3"})},
    ]
    assert len(dedup_exact(frags)) == 2


def test_summarise_fragments_handles_empty_list() -> None:
    summary = summarise_fragments([])
    assert summary.count == 0
    assert summary.min_len is None
    assert summary.max_len is None
    assert summary.avg_len is None
