"""Resource calculations used by the fragment pipeline."""

from __future__ import annotations

from typing import Any

from .node_utils import is_station


def energy_ok_fullbatt(data: dict[str, Any], a_sid: str, b_sid: str) -> bool:
    a = data["sid_to_i"][a_sid]
    b = data["sid_to_i"][b_sid]
    return data["energy"](a, b) <= data["CapE"] + 1e-9


def compute_Emin(data: dict[str, Any], seq_sids: tuple[str, ...] | list[str]) -> float:
    sid_to_i = data["sid_to_i"]
    total = 0.0
    for u_sid, v_sid in zip(seq_sids, seq_sids[1:]):
        ui = sid_to_i[u_sid]
        vi = sid_to_i[v_sid]
        total += data["energy"](ui, vi)
        if is_station(data, v_sid):
            break
    return total


def compute_distance(data: dict[str, Any], seq_sids: tuple[str, ...] | list[str]) -> float:
    sid_to_i = data["sid_to_i"]
    total = 0.0
    for u_sid, v_sid in zip(seq_sids, seq_sids[1:]):
        ui = sid_to_i[u_sid]
        vi = sid_to_i[v_sid]
        total += data["dist"](ui, vi)
    return total


def earliest_delivery_possible(data: dict[str, Any], p_sid: str) -> bool:
    nodes = data["nodes"]
    sid_to_i = data["sid_to_i"]
    d_sid = data["p2d"].get(p_sid)
    if d_sid is None:
        return False
    p_i = sid_to_i[p_sid]
    d_i = sid_to_i[d_sid]
    ready_p, due_p, serv_p = nodes[p_i][6], nodes[p_i][7], nodes[p_i][8]
    due_d = nodes[d_i][7]
    t0 = ready_p
    t_arr = t0 + serv_p + data["traveltime"](p_i, d_i)
    return t0 <= due_p + 1e-9 and t_arr <= due_d + 1e-9


def best_station_between(data: dict[str, Any], a_sid: str, b_sid: str) -> str | None:
    sid_to_i = data["sid_to_i"]
    nodes = data["nodes"]
    cap_e = data["CapE"]
    a = sid_to_i[a_sid]
    b = sid_to_i[b_sid]
    best: str | None = None
    best_score: float | None = None
    for s in data["S"]:
        s_sid = nodes[s][0]
        e1 = data["energy"](a, s)
        e2 = data["energy"](s, b)
        if e1 <= cap_e + 1e-9 and e2 <= cap_e + 1e-9:
            score = data["traveltime"](a, s) + data["traveltime"](s, b)
            if best_score is None or score < best_score:
                best_score = score
                best = s_sid
    return best


def compute_T_E_L(data: dict[str, Any], seq: tuple[str, ...] | list[str]) -> tuple[float, float, float]:
    nodes = data["nodes"]
    sid_to_i = data["sid_to_i"]
    cap_e = data["CapE"]
    recharge_rate = data["rech"]
    full_charge_time = cap_e * recharge_rate

    def ready(sid: str) -> float:
        return nodes[sid_to_i[sid]][6]

    def due(sid: str) -> float:
        return nodes[sid_to_i[sid]][7]

    def service(sid: str) -> float:
        return nodes[sid_to_i[sid]][8]

    def process_time(sid: str) -> float:
        if is_station(data, sid):
            return full_charge_time
        return service(sid)

    tf = 0.0
    for u, v in zip(seq, seq[1:]):
        ui = sid_to_i[u]
        vi = sid_to_i[v]
        tf += process_time(u) + data["traveltime"](ui, vi)

    t = ready(seq[0])
    for u, v in zip(seq, seq[1:]):
        ui = sid_to_i[u]
        vi = sid_to_i[v]
        t = t + process_time(u) + data["traveltime"](ui, vi)
        t = max(ready(v), t)
    ef = t

    t = due(seq[-1])
    for u, v in zip(reversed(seq[:-1]), reversed(seq[1:])):
        ui = sid_to_i[u]
        vi = sid_to_i[v]
        t = t - data["traveltime"](ui, vi) - process_time(u)
        t = min(due(u), t)
    lf = t

    return tf, ef, lf
