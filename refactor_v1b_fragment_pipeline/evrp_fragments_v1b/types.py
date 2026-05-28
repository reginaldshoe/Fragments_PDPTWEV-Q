"""Typed records for the v1b fragment-pipeline refactor.

Target environment:
- Python 3.12
- Target shape: dataclasses and typed containers
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Any, Mapping, Sequence


@dataclass(frozen=True)
class FragmentRecord:
    """A restricted or extended fragment before full metadata attachment."""

    seq: tuple[str, ...]
    start_onboard: frozenset[str]
    end_onboard: frozenset[str]
    contains_charge: bool
    min_start_energy: float
    ext_to: str | None = None
    ext_station: str | None = None
    ext_delivery: str | None = None

    @classmethod
    def from_mapping(cls, item: Mapping[str, Any]) -> "FragmentRecord":
        return cls(
            seq=tuple(item["seq"]),
            start_onboard=frozenset(item["start_onboard"]),
            end_onboard=frozenset(item["end_onboard"]),
            contains_charge=bool(item["contains_charge"]),
            min_start_energy=float(item["min_start_energy"]),
            ext_to=item.get("ext_to"),
            ext_station=item.get("ext_station"),
            ext_delivery=item.get("ext_delivery"),
        )

    def to_dict(self) -> dict[str, Any]:
        out: dict[str, Any] = {
            "seq": self.seq,
            "start_onboard": self.start_onboard,
            "end_onboard": self.end_onboard,
            "contains_charge": self.contains_charge,
            "min_start_energy": self.min_start_energy,
        }
        if self.ext_to is not None:
            out["ext_to"] = self.ext_to
        if self.ext_station is not None:
            out["ext_station"] = self.ext_station
        if self.ext_delivery is not None:
            out["ext_delivery"] = self.ext_delivery
        return out


@dataclass(frozen=True)
class FragmentSummary:
    """Small auditable summary of a fragment collection."""

    count: int
    min_len: int | None
    max_len: int | None
    avg_len: float | None
    with_charging: int
    without_charging: int


@dataclass(frozen=True)
class FragmentBuildResult:
    """Outputs from the v1b fragment-building pipeline."""

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
            "base_paths": len(self.base_paths),
            "restricted_raw": len(self.restricted_raw),
            "restricted_dedup": len(self.restricted_dedup),
            "restricted_meta": len(self.restricted_meta),
            "restricted_undominated": len(self.restricted_undominated),
            "extended_raw": len(self.extended_raw),
            "extended_dedup": len(self.extended_dedup),
            "extended_meta": len(self.extended_meta),
            "extended_undominated": len(self.extended_undominated),
        }
