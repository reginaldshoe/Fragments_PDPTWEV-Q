"""Resource calculations for distance, time, load and energy."""
from __future__ import annotations
from .node_utils import is_station, is_customer, is_pickup, is_delivery


def energy_ok_fullbatt(data, a_sid, b_sid):
    a = data['sid_to_i'][a_sid]
    b = data['sid_to_i'][b_sid]
    return data['energy'](a, b) <= data['CapE'] + 1e-9

def compute_Emin(data, seq_sids):
    sid_to_i = data['sid_to_i']
    total = 0.0
    for u_sid, v_sid in zip(seq_sids, seq_sids[1:]):
        ui = sid_to_i[u_sid]
        vi = sid_to_i[v_sid]
        total += data['energy'](ui, vi)
        if is_station(data, v_sid):
            break
    return total

def max_dist(data):
    nodes = range(len(data['nodes']))
    max_dist = 0.0
    for i in nodes:
        for j in nodes:
            if i == j:
                continue
            d = data['dist'](i, j)
            if d > max_dist:
                max_dist = d
    return max_dist

def earliest_delivery_possible(data, p_sid):
    nodes = data['nodes']
    sid_to_i = data['sid_to_i']
    p2d = data['p2d']

    d_sid = p2d.get(p_sid)
    p_i = sid_to_i[p_sid]
    d_i = sid_to_i[d_sid]

    # time windows
    ready_p, due_p, serv_p = nodes[p_i][6], nodes[p_i][7], nodes[p_i][8]
    ready_d, due_d = nodes[d_i][6], nodes[d_i][7]

    # earliest service start at pickup
    t0 = ready_p
    # earliest arrival at delivery (direct)
    t_arr = t0 + serv_p + data['traveltime'](p_i, d_i)
    if t_arr <= due_d + 1e-9:
        # also need that pickup itself is feasible
        return t0 <= due_p + 1e-9

    # if direct timing fails, station won't help timing
    return False

def best_station_between(data, a_sid, b_sid):

    sid_to_i = data['sid_to_i']
    nodes = data['nodes']
    CapE = data['CapE']

    a = sid_to_i[a_sid]
    b = sid_to_i[b_sid]

    best = None
    best_score = None

    for s in data['S']:
        s_sid = nodes[s][0]
        e1 = data['energy'](a, s)
        e2 = data['energy'](s, b)
        if e1 <= CapE + 1e-9 and e2 <= CapE + 1e-9:
            # choose the station that minimizes travel time detour
            score = data['traveltime'](a, s) + data['traveltime'](s, b)
            if best_score is None or score < best_score:
                best_score = score
                best = s_sid

    return best

def compute_T_E_L(data, seq):

    nodes = data['nodes']
    sid_to_i = data['sid_to_i']
    CapE = data['CapE']
    rech = data['rech']
    full_charge_time = CapE * rech

    # per-node ready/due/service and station marker
    def ready(sid): return nodes[sid_to_i[sid]][6]
    def due(sid):   return nodes[sid_to_i[sid]][7]
    def serv(sid):  return nodes[sid_to_i[sid]][8]

    # time spent at sid before leaving it
    def node_process_time(sid):
        # pessimistic view of station charging (make sure dominance based on like-for-like comparison)
        if is_station(data, sid):
            return full_charge_time
        else:
            return serv(sid)

    # Tf - duration from begin service at start frag to begin service at end frag
    Tf = 0.0
    for u, v in zip(seq, seq[1:]):
        ui = sid_to_i[u]
        vi = sid_to_i[v]
        Tf += node_process_time(u) + data['traveltime'](ui, vi)

    # Ef - earliest time service may start at end of frag
    t = ready(seq[0])  # start at earliest feasible at start
    for u, v in zip(seq, seq[1:]):
        ui = sid_to_i[u]
        vi = sid_to_i[v]
        t = t + node_process_time(u) + data['traveltime'](ui, vi)
        t = max(ready(v), t)
    Ef = t

    # Lf - latest-start time service may start at start of frag
    t = due(seq[-1])  # latest start at end
    # walk backwards: enforce arriving at next node by its current latest start t
    for u, v in zip(reversed(seq[:-1]), reversed(seq[1:])):
        ui = sid_to_i[u]
        vi = sid_to_i[v]
        # to start service at v by time t, we must start u by:
        t = t - data['traveltime'](ui, vi) - node_process_time(u)
        t = min(due(u), t)
    Lf = t

    return Tf, Ef, Lf

def compute_distance(data, seq_sids):
    sid_to_i = data['sid_to_i']
    dist_fn = data['dist']
    total = 0.0
    for u_sid, v_sid in zip(seq_sids, seq_sids[1:]):
        ui = sid_to_i[u_sid]
        vi = sid_to_i[v_sid]
        total += dist_fn(ui, vi)
    return total

def route_distance_from_sids(data, sid_seq):
    dist = data['dist']         # helper from read_instance
    sid_to_i = data['sid_to_i'] # SID -> node index
    total = 0.0
    for u_sid, v_sid in zip(sid_seq, sid_seq[1:]):
        ui = sid_to_i[u_sid]
        vi = sid_to_i[v_sid]
        total += dist(ui, vi)
    return total

def simulate_route(data, sid_seq):
    st = (
        (0,),                 # path placeholder
        0,                    # time
        frozenset(),          # onboard
        data['CapE'],         # energy
        0.0,                  # cost
        frozenset(),          # visited pickups
        frozenset(),          # visited deliveries
        frozenset(),          # visited stations
        0,                    # last node
        0.0                   # slack
    )

    for sid in sid_seq[1:]:
        j = data['sid_to_i'][sid]
        st2, reason = step(data, st, j)
        if st2 is None:
            return False, sid, reason
        st = st2

    return True, None, None
