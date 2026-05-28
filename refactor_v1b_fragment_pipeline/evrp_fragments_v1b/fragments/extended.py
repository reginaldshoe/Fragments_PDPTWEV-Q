"""Extended-fragment generation from restricted fragments."""

from __future__ import annotations

from typing import Any

from ..resources import (
    best_station_between,
    compute_Emin,
    earliest_delivery_possible,
    energy_ok_fullbatt,
)
from .onboard import compute_onboard_states


def extend_all_fragments(data: dict[str, Any], frags: list[dict[str, Any]]) -> list[dict[str, Any]]:
    nodes = data["nodes"]
    sid_to_i = data["sid_to_i"]
    p2d = data["p2d"]
    cap_l = data["CapL"]
    pickup_sids = [nodes[i][0] for i in data["P"]]

    out: list[dict[str, Any]] = []
    for frag in frags:
        seq = tuple(frag["seq"])
        start_on, end_on = compute_onboard_states(seq, frag["start_onboard"], data)
        visited = set(seq)
        end_sid = seq[-1]

        if len(end_on) == 0:
            if energy_ok_fullbatt(data, end_sid, "D0"):
                seq2 = seq + ("D0",)
                station = None
            else:
                station = best_station_between(data, end_sid, "D0")
                if station is None:
                    continue
                seq2 = seq + (station, "D0")
            out.append(
                {
                    **frag,
                    "seq": seq2,
                    "start_onboard": frag["start_onboard"],
                    "end_onboard": frozenset(),
                    "contains_charge": frag["contains_charge"] or station is not None,
                    "min_start_energy": compute_Emin(data, seq2),
                    "ext_to": "D0",
                    "ext_station": station,
                    "ext_delivery": None,
                }
            )

        for pickup_sid in pickup_sids:
            if pickup_sid in visited:
                continue
            if pickup_sid in start_on or pickup_sid in end_on:
                continue
            delivery_sid = p2d.get(pickup_sid)
            if delivery_sid in visited:
                continue

            new_end_on = end_on | {pickup_sid}
            load = sum(nodes[sid_to_i[n]][5] for n in new_end_on)
            if load > cap_l + 1e-9:
                continue
            if not earliest_delivery_possible(data, pickup_sid):
                continue

            station = None
            if not energy_ok_fullbatt(data, end_sid, pickup_sid):
                station = best_station_between(data, end_sid, pickup_sid)
                if station is None:
                    continue

            if station is None:
                seq2 = seq + (pickup_sid,)
            else:
                seq2 = seq + (station, pickup_sid)

            out.append(
                {
                    "seq": seq2,
                    "start_onboard": frag["start_onboard"],
                    "end_onboard": frozenset(new_end_on),
                    "contains_charge": frag["contains_charge"] or station is not None,
                    "min_start_energy": compute_Emin(data, seq2),
                    "ext_to": pickup_sid,
                    "ext_station": station,
                    "ext_delivery": delivery_sid,
                }
            )
    return out
