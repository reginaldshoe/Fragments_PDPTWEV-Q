"""Restricted-fragment generation from base paths."""

from __future__ import annotations

from typing import Any

from ..resources import compute_Emin


def trim_base_path(data: dict[str, Any], base_path: list[int] | tuple[int, ...]) -> list[dict[str, Any]]:
    nodes = data["nodes"]
    d2p = data["d2p"]
    pickups = set(data["P"])
    deliveries = set(data["D"])

    d_switch: int | None = None
    for pos, idx in enumerate(base_path):
        if nodes[idx][2] == "cd":
            d_switch = pos
            break
    if d_switch is None:
        return []

    pickup_part = base_path[:d_switch]
    delivery_part = base_path[d_switch:]
    pickup_seq = [nodes[i][0] for i in pickup_part if i in pickups]
    delivery_seq = [nodes[i][0] for i in delivery_part if i in deliveries]

    if not pickup_seq or not delivery_seq or len(pickup_seq) != len(delivery_seq):
        return []

    pos_by_sid = {nodes[idx][0]: pos for pos, idx in enumerate(base_path)}
    out: list[dict[str, Any]] = []
    p_len = len(pickup_seq)
    d_len = len(delivery_seq)

    for a in range(p_len):
        start_onboard = frozenset(pickup_seq[: a + 1])
        start_sid = pickup_seq[a]
        start_pos = pos_by_sid[start_sid]

        for b in range(d_len):
            kept_delivery = delivery_seq[: d_len - b]
            removed_delivery = delivery_seq[d_len - b :]
            if not kept_delivery:
                continue
            end_sid = kept_delivery[-1]
            end_pos = pos_by_sid[end_sid]
            if end_pos < start_pos:
                continue

            subseq = base_path[start_pos : end_pos + 1]
            seq_sids = tuple(nodes[i][0] for i in subseq)

            end_onboard: set[str] = set()
            for d_sid in removed_delivery:
                p_sid = d2p.get(d_sid)
                if p_sid is not None:
                    end_onboard.add(p_sid)

            out.append(
                {
                    "seq": seq_sids,
                    "start_onboard": start_onboard,
                    "end_onboard": frozenset(end_onboard),
                    "contains_charge": any(nodes[i][1] == "S" or nodes[i][2] == "f" for i in subseq),
                    "min_start_energy": compute_Emin(data, seq_sids),
                }
            )
    return out


def enumerate_fragments(data: dict[str, Any], base_paths: list[Any]) -> list[dict[str, Any]]:
    out: list[dict[str, Any]] = []
    for base_path in base_paths:
        out.extend(trim_base_path(data, base_path))
    return out
