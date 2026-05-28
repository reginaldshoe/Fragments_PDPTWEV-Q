"""Fragment metadata attachment and duplicate removal."""

from __future__ import annotations

from typing import Any

from ..node_utils import is_customer
from ..resources import compute_T_E_L, compute_distance


def cust_locs(data: dict[str, Any], seq: tuple[str, ...] | list[str], exclude_last: bool) -> frozenset[str]:
    if not seq:
        return frozenset()
    out = [sid for sid in seq if is_customer(data, sid)]
    end_sid = seq[-1]
    if exclude_last and is_customer(data, end_sid):
        out = [sid for sid in out if sid != end_sid]
    return frozenset(out)


def dedup_exact(frags: list[dict[str, Any]]) -> list[dict[str, Any]]:
    seen: set[tuple[Any, ...]] = set()
    out: list[dict[str, Any]] = []
    for frag in frags:
        sig = (tuple(frag["seq"]), frozenset(frag["start_onboard"]), frozenset(frag["end_onboard"]))
        if sig in seen:
            continue
        seen.add(sig)
        out.append(frag)
    return out


def dedup_by_signature(frags: list[dict[str, Any]]) -> list[dict[str, Any]]:
    seen: set[tuple[Any, ...]] = set()
    out: list[dict[str, Any]] = []
    for frag in frags:
        sig = (
            tuple(frag["seq"]),
            frozenset(frag["start_onboard"]),
            frozenset(frag["end_onboard"]),
            frag["Ef"],
            frag["Lf"],
            frag["Tf"],
            frag["Emin"],
        )
        if sig in seen:
            continue
        seen.add(sig)
        out.append(frag)
    return out


def attach_metadata(
    data: dict[str, Any],
    frags: list[dict[str, Any]],
    exclude_last_ef: bool = False,
) -> list[dict[str, Any]]:
    out: list[dict[str, Any]] = []
    for frag in frags:
        seq = tuple(frag["seq"])
        tf, ef, lf = compute_T_E_L(data, seq)
        item = dict(frag)
        item["seq"] = seq
        item["Tf"] = tf
        item["Ef"] = ef
        item["Lf"] = lf
        item["Df"] = compute_distance(data, seq)
        item["Emin"] = item["min_start_energy"]
        item["Start"] = seq[0]
        item["End"] = seq[-1]
        item["LocsC"] = cust_locs(data, seq, exclude_last=exclude_last_ef)
        item["dom_key"] = (
            item["Start"],
            item["End"],
            item["start_onboard"],
            item["end_onboard"],
            item["LocsC"],
        )
        out.append(item)
    return out
