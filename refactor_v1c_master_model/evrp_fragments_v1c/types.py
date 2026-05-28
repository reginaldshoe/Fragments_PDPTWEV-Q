"""Typed records for v1c."""
from __future__ import annotations
from dataclasses import dataclass
from typing import Any, Mapping

@dataclass(frozen=True)
class FragmentSummary:
    count: int
    min_len: int | None
    max_len: int | None
    avg_len: float | None
    with_charging: int
    without_charging: int

@dataclass(frozen=True)
class FragmentBuildResult:
    base_paths: list[Any]
    pruned: Any
    restricted_raw: list[dict[str, Any]]
    restricted_dedup: list[dict[str, Any]]
    restricted_meta: list[dict[str, Any]]
    restricted_undominated: list[dict[str, Any]]
    extended_raw: list[dict[str, Any]]
    extended_dedup: list[dict[str, Any]]
    extended_meta: list[dict[str, Any]]
    extended_undominated: list[dict[str, Any]]
    def summary(self) -> dict[str, int]:
        return {
            'base_paths': len(self.base_paths),
            'restricted_raw': len(self.restricted_raw),
            'restricted_dedup': len(self.restricted_dedup),
            'restricted_meta': len(self.restricted_meta),
            'restricted_undominated': len(self.restricted_undominated),
            'extended_raw': len(self.extended_raw),
            'extended_dedup': len(self.extended_dedup),
            'extended_meta': len(self.extended_meta),
            'extended_undominated': len(self.extended_undominated),
        }

@dataclass(frozen=True)
class NetworkBuildResult:
    node_id: dict[tuple[str, frozenset[str]], int]
    arcs: list[dict[str, Any]]

@dataclass(frozen=True)
class MasterBuildResult:
    model: Any
    x: dict[int, Any]
    arcs: list[dict[str, Any]]
    node_id: dict[tuple[str, frozenset[str]], int]
    depot_u: int
    arc_by_id: dict[int, dict[str, Any]]
    theta: Any
    big_m: float
