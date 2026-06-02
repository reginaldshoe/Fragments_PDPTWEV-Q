"""Resource calculations used by fragments, master model and diagnostics."""
from __future__ import annotations
from typing import Any
from .node_utils import is_station

def energy_ok_fullbatt(data: dict[str, Any], a_sid: str, b_sid: str) -> bool:
    return data['energy'](data['sid_to_i'][a_sid], data['sid_to_i'][b_sid]) <= data['CapE'] + 1e-9

def compute_Emin(data: dict[str, Any], seq_sids: tuple[str, ...] | list[str]) -> float:
    total = 0.0
    for u_sid, v_sid in zip(seq_sids, seq_sids[1:]):
        total += data['energy'](data['sid_to_i'][u_sid], data['sid_to_i'][v_sid])
        if is_station(data, v_sid):
            break
    return total

def compute_distance(data: dict[str, Any], seq_sids: tuple[str, ...] | list[str]) -> float:
    return sum(data['dist'](data['sid_to_i'][u], data['sid_to_i'][v]) for u, v in zip(seq_sids, seq_sids[1:]))

def max_dist(data: dict[str, Any]) -> float:
    out = 0.0
    for i in range(len(data['nodes'])):
        for j in range(len(data['nodes'])):
            if i != j:
                out = max(out, data['dist'](i, j))
    return out

def earliest_delivery_possible(data: dict[str, Any], p_sid: str) -> bool:
    d_sid = data['p2d'].get(p_sid)
    if d_sid is None:
        return False
    nodes = data['nodes']; sid_to_i = data['sid_to_i']
    p_i = sid_to_i[p_sid]; d_i = sid_to_i[d_sid]
    ready_p, due_p, serv_p = nodes[p_i][6], nodes[p_i][7], nodes[p_i][8]
    due_d = nodes[d_i][7]
    return ready_p <= due_p + 1e-9 and ready_p + serv_p + data['traveltime'](p_i, d_i) <= due_d + 1e-9

def best_station_between(data: dict[str, Any], a_sid: str, b_sid: str) -> str | None:
    sid_to_i = data['sid_to_i']; nodes = data['nodes']; cap_e = data['CapE']
    a = sid_to_i[a_sid]; b = sid_to_i[b_sid]
    best = None; best_score = None
    for s in data['S']:
        s_sid = nodes[s][0]
        if data['energy'](a, s) <= cap_e + 1e-9 and data['energy'](s, b) <= cap_e + 1e-9:
            score = data['traveltime'](a, s) + data['traveltime'](s, b)
            if best_score is None or score < best_score:
                best_score = score; best = s_sid
    return best

def compute_T_E_L(data: dict[str, Any], seq: tuple[str, ...] | list[str]) -> tuple[float, float, float]:
    nodes = data['nodes']; sid_to_i = data['sid_to_i']
    full_charge_time = data['CapE'] * data['rech']
    def ready(sid: str) -> float: return nodes[sid_to_i[sid]][6]
    def due(sid: str) -> float: return nodes[sid_to_i[sid]][7]
    def service(sid: str) -> float: return nodes[sid_to_i[sid]][8]
    def process_time(sid: str) -> float: return full_charge_time if is_station(data, sid) else service(sid)
    tf = sum(process_time(u) + data['traveltime'](sid_to_i[u], sid_to_i[v]) for u, v in zip(seq, seq[1:]))
    t = ready(seq[0])
    for u, v in zip(seq, seq[1:]):
        t = max(ready(v), t + process_time(u) + data['traveltime'](sid_to_i[u], sid_to_i[v]))
    ef = t
    t = due(seq[-1])
    for u, v in zip(reversed(seq[:-1]), reversed(seq[1:])):
        t = min(due(u), t - data['traveltime'](sid_to_i[u], sid_to_i[v]) - process_time(u))
    return tf, ef, t
