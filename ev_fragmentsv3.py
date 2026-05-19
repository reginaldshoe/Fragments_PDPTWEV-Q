# Focusing on trying to develop fragment algo first.

# Implementation structure, tried to follow principles from Rist and Forbes
# 1. enumerate full 'base' paths (defined where onboard load is empty at the start and the end of path)
# 1a. using a helper algorithm to update the next step
# 2. trim paths to truncated restricted fragments (defined where path has exactly one switch from pickup to delivery)
# 3. extend truncated restricted fragments to the next pickup
# 4. Domination filtering for both restricted frags and extended frags
# 5. Master Problem
# 6a. Sub-problem, feasibility
# 6b. subproblem, optimality (ie. if this fragment join fails due to energy feasibility, can we insert a station
# somewhere that would make it feasible?)

import math
from pathlib import Path
import os
import gurobipy as gp
from time import perf_counter
import heapq

path = Path.cwd() / "instances"

DEBUG = False
DEBUG_CALLBACK = False
DEBUG_DIAGNOSTICS = True

FORCE_CHAIN = False
RUN_OPTIONA = True
DIAG_SKELETON = False

DP_MAX_STATION_VISITS_PER_LEG = 6
DP_MAX_LABELS_PER_NODE = 500


# Used Copilot to develop read_instance block, needed some editing
def read_instance(path):
    #ID, type, x, y, demand, ready, due, service time, partner ID
    with open(path, 'r') as f:
        f.readline()
        rows = []
        first_param = None
        for line in f:
            s = line.strip()
            if not s:
                continue
            if ':' in s:
                first_param = s
                break
            parts = s.split()
            if len(parts) < 9:
                continue
            sid, typ, x, y, dem, rt, dt, st, partner = parts[:9]
            rows.append((sid, typ, float(x), float(y), float(dem), float(rt), float(dt), float(st), partner))

        if first_param is None:
            raise ValueError('Missing parameter section.')

        def val_after_colon(s):
            return float(s.split(':', 1)[1].strip())

        CapE = val_after_colon(first_param)
        CapL = val_after_colon(next(f).strip())
        cons = val_after_colon(next(f).strip())
        rech = val_after_colon(next(f).strip())
        speed = val_after_colon(next(f).strip())

        rest = [ln.strip() for ln in f if ln.strip()]

    belongs = {}
    if rest and rest[0].lower() == 'belongs':
        for ln in rest[1:]:
            p = ln.split()
            if not p:
                continue
            if p[0].lower().startswith('numowner') or p[0].lower() == 'end':
                break
            if p[0].startswith('C') and len(p) >= 2:
                try:
                    belongs[p[0]] = int(p[1])
                except:
                    pass

    # Build flat arrays
    # Indexing: 0 = depot, then customers (C*), then stations (S*)
    depot = None
    customers = []
    stations = []
    for sid, typ, x, y, dem, rt, dt, st, partner in rows:
        kind = sid[0]
        if kind == 'D':
            depot = (sid, kind, typ, x, y, dem, rt, dt, st, partner)
        elif kind == 'C':
            customers.append((sid, kind, typ, x, y, dem, rt, dt, st, partner))
        elif kind == 'S':
            stations.append((sid, kind, typ, x, y, dem, rt, dt, st, partner))

    if depot is None:
        raise ValueError('Depot D0 not found')

    nodes = [depot] + customers + stations

    sid_to_i = {nodes[i][0]: i for i in range(len(nodes))}

    P = set(i for i in range(len(nodes)) if nodes[i][1] == 'C' and nodes[i][2] == 'cp')
    D = set(i for i in range(len(nodes)) if nodes[i][1] == 'C' and nodes[i][2] == 'cd')
    S = set(i for i in range(len(nodes)) if nodes[i][1] == 'S' or nodes[i][2] == 'f')

    # partner maps on StringIDs
    p2d = {}
    d2p = {}
    for sid, kind, typ, *_ , partner in nodes:
        if kind != 'C':
            continue
        if partner != '0':
            if typ == 'cp':
                p2d[sid] = partner
            elif typ == 'cd':
                d2p[sid] = partner

    # allow d2p even if only given via cp
    for p_sid, d_sid in p2d.items():
        d2p.setdefault(d_sid, p_sid)

    xs = [nodes[i][3] for i in range(len(nodes))]
    ys = [nodes[i][4] for i in range(len(nodes))]

    def dist(i, j):
        return math.hypot(xs[i] - xs[j], ys[i] - ys[j])

    def traveltime(i, j):
        return speed * dist(i, j)

    def energy(i, j):
        return cons * dist(i, j)

    horizon = nodes[0][7]  # depot due time

    data = {
        'nodes': nodes,
        'sid_to_i': sid_to_i,
        'P': P,
        'D': D,
        'S': S,
        'p2d': p2d,
        'd2p': d2p,
        'CapE': CapE,
        'CapL': CapL,
        'cons': cons,
        'rech': rech,
        'speed': speed,
        'horizon': horizon,
        'dist': dist,
        'traveltime': traveltime,
        'energy': energy,
    }

    return data

# helper functions
def is_station(data, sid):
    nodes = data['nodes']
    i = data['sid_to_i'].get(sid)
    if i is None:
        return False
    return (nodes[i][1] == 'S') or (nodes[i][2] == 'f')

def is_customer(data, sid):
    nodes = data['nodes']
    i = data['sid_to_i'].get(sid)
    if i is None:
        return False
    return nodes[i][1] == 'C'

def is_pickup(data, sid):
    i = data['sid_to_i'].get(sid)
    if i is None:
        return False
    return data['nodes'][i][2] == 'cp'

def is_delivery(data, sid):
    i = data['sid_to_i'].get(sid)
    if i is None:
        return False
    return data['nodes'][i][2] == 'cd'

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

def strip_stations(seq, data):
    return [sid for sid in seq if not is_station(data, sid)]


# Dominance + metadata helpers

# customer locations (excludes stations)
def cust_locs(seq, exclude_last):
    if not seq:
        return frozenset()
    out = []
    end_sid = seq[-1]
    for sid in seq:
        if is_customer(data, sid):
            out.append(sid)
    if exclude_last and is_customer(data, end_sid):
        out = [x for x in out if x != end_sid]
    return frozenset(out)

# compute time windows, Tf, Ef, Lf

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


# eliminate exact duplicates
def dedup_exact(frags):
    seen = set()
    out = []
    for f in frags:
        sig = (f['seq'], f['start_onboard'], f['end_onboard'])
        if sig in seen:
            continue
        seen.add(sig)
        out.append(f)
    return out

# eliminate duplicates based on metadata
def dedup_by_signature(frags):
    seen = set()
    out = []
    for f in frags:
        sig = (f['seq'], f['start_onboard'], f['end_onboard'], f['Ef'], f['Lf'], f['Tf'], f['Emin'])
        if sig in seen:
            continue
        seen.add(sig)
        out.append(f)
    return out

# attach Tf/Ef/Lf/Emin and dominance keys for frags (flag for RF or EF)
def attach_metadata(data, frags, exclude_last_ef = False):
    out = []
    for f in frags:
        seq = f['seq']
        Tf, Ef, Lf = compute_T_E_L(data, seq)
        g = dict(f)
        g['Tf'] = Tf
        g['Ef'] = Ef
        g['Lf'] = Lf
        g['Df'] = compute_distance(data, seq)
        g['Emin'] = g['min_start_energy']  # already computed in trimming
        g['Start'] = seq[0]
        g['End'] = seq[-1]
        g['LocsC'] = cust_locs(seq, exclude_last=exclude_last_ef)
        # Dominance key without stations
        g['dom_key'] = (g['Start'], g['End'], g['start_onboard'], g['end_onboard'], g['LocsC'])
        out.append(g)
    return out

# dominance within same dominance key. Smaller Ef/Tf/Emin and larger Lf is better.
def dominates(a, b):

    if a['dom_key'] != b['dom_key']:
        return False
    better_or_equal = (a['Ef'] <= b['Ef'] + 1e-9 and
                       a['Lf'] >= b['Lf'] - 1e-9 and
                       a['Tf'] <= b['Tf'] + 1e-9 and
                       a['Emin'] <= b['Emin'] + 1e-9)
    strictly_better = (a['Ef'] < b['Ef'] - 1e-9 or
                       a['Lf'] > b['Lf'] + 1e-9 or
                       a['Tf'] < b['Tf'] - 1e-9 or
                       a['Emin'] < b['Emin'] - 1e-9)
    return better_or_equal and strictly_better

# filtration function
def filter_by_key(items):
    keep = []
    for x in items:
        # if any in kept dominates x -> drop x
        if any(dominates(k, x) for k in keep):
            continue
        # else remove those dominated by x
        keep = [k for k in keep if not dominates(x, k)]
        keep.append(x)
    return keep

# Group by fragments by dom_key then apply filter in each group.
def dominance_filter(items):
    buckets = {}
    for f in items:
        buckets.setdefault(f['dom_key'], []).append(f)
    out = []
    for key, group in buckets.items():
        out.extend(filter_by_key(group))
    return out


def print_dp_path_structure(tag, data, dp_path, dp_dist):
    stations = [sid for sid in dp_path if is_station(data, sid)]
    print(f"[{tag}] DP distance = {dp_dist:.6f}")
    print(f"[{tag}] DP path = {list(dp_path)}")
    print(f"[{tag}] Stations used ({len(stations)}): {stations}")


# /end helpers
# step update (analogous to Algo 1: Appendix B of Rist/Forbes), extend existing path to node j
# Original paper only tracked time, load and distance
# had to include a bunch of extra state information including energy and load capacity
# returns (new_state, None) if feasible, else (None, reason).

def step(data, state, j):

    nodes = data['nodes']
    traveltime = data['traveltime']
    energy = data['energy']
    CapE = data['CapE']
    CapL = data['CapL']
    rech = data['rech']
    sid_to_i = data['sid_to_i']
    d2p = data['d2p']
    dist = data['dist']


    # current state
    # path, pickup/delivery 0/1, load onboard, energy level, time depart last node, prev visited P/D/S, delivery count, total distance
    path, phase, onboard, E, t_depart, seenP, seenD, seenS, deliv_count,total_dist = state

    # last step of path
    i = path[-1]
    sid_j, kind_j, typ_j, xj, yj, dem_j, ready_j, due_j, serv_j, partner_j = nodes[j]

    dist2 = total_dist

    # avoid revisiting same station
    # if is_station(data, sid_j) and sid_j in seenS:
    #     return None, 'revisit_station'

    # time feasibility
    t_arr = t_depart + traveltime(i, j)
    t_start = max(ready_j, t_arr)
    if t_start > due_j + 1e-9:
        return None, 'timewindow'

    # energy feasibility
    E_arr = E - energy(i, j)
    if E_arr < 0:
        return None, 'energy'

    # If node is a charging station, charge to full capacity and return
    if is_station(data, sid_j):
        charge_time = (CapE - E_arr) * rech
        t_depart2 = t_start + charge_time
        E2 = CapE
        seenS2 = set(seenS)
        seenS2.add(sid_j)
        dist2 += dist(i,j)
        return (path + (j,), phase, onboard, E2, t_depart2, seenP, seenD, frozenset(seenS2), deliv_count, dist2), None

    # if node is a customer, update params
    onboard2 = set(onboard)
    seenP2 = set(seenP)
    seenD2 = set(seenD)
    phase2 = phase
    deliv2 = deliv_count

    # add load if pickup
    if is_pickup(data, sid_j):
        onboard2.add(sid_j)
        seenP2.add(sid_j)
    elif is_delivery(data, sid_j):
        # must correspond to an onboard pickup
        p_sid = d2p.get(sid_j)
        if p_sid not in onboard2:
            return None, 'delivery_not_onboard'
        onboard2.remove(p_sid)
        seenD2.add(sid_j)
        deliv2 += 1
        if phase2 == 0:
            phase2 = 1  # switch to DELIVERY

    # if pickup, check current demand and make sure += new demand <CapL
    load = 0.0
    for p_sid in onboard2:
        pi = sid_to_i[p_sid]
        load += nodes[pi][5] #demand
    if load > CapL + 1e-9:
        return None, 'capacity'

    # update time and distance
    t_depart2 = t_start + serv_j
    dist2 += dist(i,j)

    return (path + (j,), phase2, frozenset(onboard2), E_arr, t_depart2,
            frozenset(seenP2), frozenset(seenD2), seenS, deliv2,dist2), None


# base paths as per step 1 (copilot helped me turn this into a nice function that reports the pruned paths)

def enumerate_base_paths(data, maxlen):

    nodes = data['nodes']
    CapE = data['CapE']
    energy = data['energy']
    traveltime = data['traveltime']
    sid_to_i = data['sid_to_i']
    p2d = data['p2d']
    dist = data['dist']

    P = list(data['P'])
    S = list(data['S'])


    pruned = {}
    def prune(r):
        pruned[r] = pruned.get(r, 0) + 1

    # seed states from each pickup, starting at depot with full charge
    depot = 0

    # start a set for 'working' paths with all seeds
    Work = set()
    for p in P:
        sid_p, kind_p, typ_p, xp, yp, dem_p, ready_p, due_p, serv_p, partner_p = nodes[p]
        # travel from depot
        E_arr = CapE - energy(depot, p)
        if E_arr < -1e-9:
            prune('seed_energy')
            continue
        t_arr = traveltime(depot, p)
        t_start = max(ready_p, t_arr)
        if t_start > due_p + 1e-9:
            prune('seed_timewindow')
            continue
        t_depart = t_start + serv_p
        distance = dist(depot, p)

        onboard = frozenset([sid_p])
        state = ((p,), 0, onboard, E_arr, t_depart, frozenset([sid_p]), frozenset(), frozenset(), 0,distance)
        Work.add(state)

    # set of base paths
    Base = set()

    for depth in range(maxlen):
        NewWork = set()

        # check each working path to see if it can be completed, otherwise attempt extension
        for st in Work:
            path, phase, onboard, E, t_depart, seenP, seenD, seenS, deliv_count,distance = st

            # base path complete (i.e. empty + currently in delivery phase + at least one delivery)
            if phase == 1 and len(onboard) == 0 and deliv_count > 0:
                Base.add(path)
                continue

            # otherwise extend path
            # try a delivery of currently onboard pickups
            for p_sid in onboard:
                d_sid = p2d.get(p_sid)
                j = sid_to_i.get(d_sid)
                newst, reason = step(data, st, j)
                if newst is None:
                    prune(reason)
                else:
                    NewWork.add(newst)

            # if still pickup phase, allow more pickups
            if phase == 0:
                for q in P:
                    sid_q = nodes[q][0]
                    if sid_q in seenP:
                        continue
                    newst, reason = step(data, st, q)
                    if newst is None:
                        prune(reason)
                    else:
                        NewWork.add(newst)

            # move to charging station (avoid station to station)
            # TODO: can probably nuance this, only attempt station if not enough energy to do anything else
            prev_station = (path[-1] in S)
            if prev_station:
                prune('prev_station')
            if not prev_station:
                for s in S:
                    newst, reason = step(data, st, s)
                    if newst is None:
                        prune(reason)
                    else:
                        NewWork.add(newst)

        Work = NewWork
        if DEBUG:
            print('depth', depth, 'work', len(Work), 'base', len(Base))
        if not Work:
            break

    return list(Base), pruned


# trim from a base path to a restricted fragment (i.e. 1 pick-up -> delivery switch)

def trim_base_path(data, base_path):

    nodes = data['nodes']
    d2p = data['d2p']

    P = list(data['P'])
    D = list(data['D'])

    # find first delivery index within a base path (phase switch)
    d_switch = None
    for d, idx in enumerate(base_path):
        if nodes[idx][2] == 'cd':
            d_switch = d
            break
    # no delivery = no fragment
    if d_switch is None:
        return []

    # split base path into two parts, pickup and delivery
    pickup_part = base_path[:d_switch]
    delivery_part = base_path[d_switch:]

    # convert back to sid
    Pseq = [nodes[i][0] for i in pickup_part if i in P]
    Dseq = [nodes[i][0] for i in delivery_part if i in D]

    # validate that both pickup/delivery occurs
    if not Pseq or not Dseq or len(Pseq) != len(Dseq):
        return []

    # position maps for slicing
    pos = {}
    for t, idx in enumerate(base_path):
        sid = nodes[idx][0]
        pos[sid] = t

    frags = []
    p_len = len(Pseq)
    d_len = len(Dseq)

    # loop all possible starting point in base_path
    for a in range(p_len):
        start_onboard = frozenset(Pseq[:a+1])
        start_sid = Pseq[a]
        s0 = pos[start_sid]

        # loop all possible end points, track what is still on board
        for b in range(d_len):
            kept_delivery = Dseq[:d_len - b]
            removed_delivery = Dseq[d_len - b:]
            # at least one delivery
            if not kept_delivery:
                continue
            end_sid = kept_delivery[-1]
            s1 = pos[end_sid]
            if s1 < s0:
                continue

            # fragment sid
            subseq = base_path[s0:s1 + 1]
            seq_sids = tuple(nodes[i][0] for i in subseq)

            # onboard at end of fragment
            end_onboard = set()
            for d_sid in removed_delivery:
                p_sid = d2p.get(d_sid)
                if p_sid:
                    end_onboard.add(p_sid)

            # calculate energy required to reach first charging station
            # prefix fragment in network would need to know this for feasibility
            energy_req = compute_Emin(data, seq_sids)

            frags.append({
                'seq': seq_sids,
                'start_onboard': start_onboard,
                'end_onboard': frozenset(end_onboard),
                'contains_charge': any(nodes[i][1] == 'S' or nodes[i][2] == 'f' for i in subseq),
                'min_start_energy': energy_req,
            })
    if DEBUG:
        print("trim_base_path returning", len(frags), "frags")
    return frags

# generate all the restricted fragments for a set of base paths
def enumerate_fragments(data, base_paths):
    frags = []
    for bp in base_paths:
        frags.extend(trim_base_path(data, bp))
    return frags

def compute_onboard_states(seq, start_onboard, data):
    nodes = data['nodes']
    p2d = data['p2d']
    d2p = data['d2p']

    # identify pickups
    pickup_nodes = {nodes[i][0] for i in data['P']}

    load = set(start_onboard)
    start_on = load.copy()

    for sid in seq:
        if sid in pickup_nodes:
            load.add(sid)
        elif sid in d2p:
            # SAFE REMOVE (important)
            if d2p[sid] in load:
                load.remove(d2p[sid])
            else:
                # this can happen if fragment starts mid-route
                # → skip instead of crashing
                pass

    end_on = load.copy()

    return frozenset(start_on), frozenset(end_on)

# extend fragments to new pickup i, and attach/update metadata
def extend_all_fragments(data, frags):

    nodes = data['nodes']
    sid_to_i = data['sid_to_i']
    p2d = data['p2d']
    CapL = data['CapL']
    P = data['P']

    pickups = [nodes[i][0] for i in P]  # list of pickup sids
    out = []

    for f in frags:
        seq = f['seq']
        start_on, end_on = compute_onboard_states(seq, f['start_onboard'],data)

        # visited customers + stations from sequence
        visited = set(seq)

        # end sid of this fragment
        end_sid = seq[-1]

        # depot extension if end_onboard empty
        if len(end_on) == 0:
            if energy_ok_fullbatt(data, end_sid,'D0'):
                seq2 = seq + ('D0',)
            # allow additional station if depot unreachable
            else:
                s = best_station_between(data, end_sid, 'D0')
                if s is None:
                    continue
                seq2 = seq + (s,'D0')
            out.append({
                **f,
                'seq': seq2,
                'Start': seq2[0],
                'End': 'D0',
                'start_onboard': f['start_onboard'],
                'min_start_energy': compute_Emin(data, seq2),
                'end_onboard': frozenset(),
                'ext_to': 'D0',
                'ext_station': None,
                'ext_delivery': None,
            })

        # extend to every pickup i
        for i_sid in pickups:
            # EXCLUSIONS
            # exclude if next pickup already visited in fragment
            if i_sid in visited:
                continue
            # exclude if next pickup had been onboard at some stage during fragment
            if i_sid in start_on or i_sid in end_on:
                continue
            # exclude next pickup if its delivery already occurred inside the fragment
            d_sid = p2d.get(i_sid)
            if d_sid in visited:
                continue

            # capacity after picking i
            new_end_on = end_on | {i_sid}
            if sum(nodes[sid_to_i[n]][5] for n in new_end_on) > CapL + 1e-9:
                continue

            # the request from pickup to delivery must be time-feasible on its own
            if not earliest_delivery_possible(data, i_sid):
                continue

            # energy reachability end -> i (allow one station)
            ext_station = None
            if energy_ok_fullbatt(data, end_sid, i_sid):
                ext_station = None
            else:
                ext_station = best_station_between(data, end_sid, i_sid)
                if ext_station is None:
                    continue

            # build extended sequence
            if ext_station is None:
                seq2 = seq + (i_sid,)
            else:
                seq2 = seq + (ext_station, i_sid)

            out.append({
                'seq': seq2,
                'start_onboard': f['start_onboard'],
                'end_onboard': frozenset(new_end_on),
                'contains_charge': f['contains_charge'] or (ext_station is not None),
                'min_start_energy': compute_Emin(data, seq2),
                'ext_to': i_sid,
                'ext_station': ext_station,
                'ext_delivery': d_sid,
            })

    return out

def enforce_start_states(frags, data):
    pickup_nodes = {data['nodes'][i][0] for i in data['P']}
    out = []

    for f in frags:
        seq = f['seq']

        # always keep original (do NOT filter)
        out.append(f)

        # ensure minimal valid state exists
        if seq[0] in pickup_nodes:
            load = {seq[0]}
        else:
            load = set()

        # recompute end_on
        for sid in seq[1:]:
            if sid in pickup_nodes:
                load.add(sid)
            elif sid in data['d2p']:
                p = data['d2p'][sid]
                if p in load:
                    load.remove(p)

        new_f = dict(f)
        new_f['start_onboard'] = frozenset({seq[0]}) if seq[0] in pickup_nodes else frozenset()
        new_f['end_onboard'] = frozenset(load)

        out.append(new_f)

    return out

from itertools import combinations

def augment_states(data, frags, max_extra=1):

    nodes = data['nodes']
    P = data['P']
    pickup_nodes = {nodes[i][0] for i in P}
    d2p = data['d2p']

    out = []
    seen = set()

    for f in frags:
        seq = tuple(f['seq'])
        out.append(f)
        seen.add((seq, f['start_onboard'], f['end_onboard']))

        # pickups that occur anywhere in this fragment (cannot be "carried-in")
        pickups_in_seq = {sid for sid in seq if sid in pickup_nodes}

        # pickups whose deliveries occur in this fragment (cannot be "carried-in")
        delivered_pickups = set()
        for sid in seq:
            if sid in d2p:          # sid is a delivery node
                delivered_pickups.add(d2p[sid])  # pickup paired to that delivery

        # carry candidates: pickups not in seq and not delivered in seq
        carry_candidates = sorted(pickup_nodes - pickups_in_seq - delivered_pickups)

        # add a small number of carryovers (max_extra=1 is usually enough)
        base_start = f['start_onboard']

        for k in range(1, max_extra + 1):
            for extra in combinations(carry_candidates, k):
                start2 = frozenset(set(base_start) | set(extra))
                _, end2 = compute_onboard_states(seq, start2, data)

                key = (seq, start2, end2)
                if key in seen:
                    continue

                g = dict(f)
                g['start_onboard'] = start2
                g['end_onboard'] = end2
                out.append(g)
                seen.add(key)

    return out

def compute_min_start_onboard(seq, data):
    pickup_nodes = {data['nodes'][i][0] for i in data['P']}
    d2p = data['d2p']

    load = set()
    required = set()
    allowed = set() # pickups seen so far

    for sid in seq:
        if sid in pickup_nodes:
            # MUST be onboard when visiting pickup
            required.add(sid)
            load.add(sid)
            allowed.add(sid)
        elif sid in d2p:
            p = d2p[sid]

            if p not in load:
                required.add(p)
            else:
                load.remove(p)

    required = {p for p in required if p in allowed}
    return frozenset(required)

def extract_subarcs(data, frags):
    nodes = data['nodes']
    P = data['P']
    d2p = data['d2p']
    # pickup nodes (cp)
    pickup_nodes = {nodes[i][0] for i in P}

    new_arcs = []

    for f in frags:
        seq = list(f['seq'])

        for i in range(len(seq) - 1):
            u = seq[i]
            v = seq[i + 1]

            # u is a pickup, v is its paired delivery
            if u in pickup_nodes and v in d2p and d2p[v] == u:

                # Try direct to depot, else one station
                if energy_ok_fullbatt(data, v, 'D0'):
                    cand = (u, v, 'D0')
                else:
                    s = best_station_between(data, v, 'D0')
                    if s is None:
                        continue
                    cand = (u, v, s, 'D0')

                # enforce proper states: pickup is onboard at start, empty at end
                start_on = frozenset({u})
                end_on = frozenset()

                Tf, Ef, Lf = compute_T_E_L(data, cand)

                new_arcs.append({
                    'seq': cand,
                    'Start': cand[0],
                    'End': cand[-1],
                    'start_onboard': start_on,
                    'end_onboard': end_on,
                    'contains_charge': any(is_station(data, x) for x in cand),
                    'min_start_energy': compute_Emin(data, cand),
                    'Tf': float(Tf),
                    'Ef': float(Ef),
                    'Lf': float(Lf),
                    'Df': compute_distance(data, cand),
                    'Emin': compute_Emin(data, cand),
                    'LocsC': cust_locs(cand, exclude_last=False),
                })

    return new_arcs

def deduplicate_arcs(arcs):
    seen = set()
    out = []

    for f in arcs:
        key = (f['seq'], f['start_onboard'], f['end_onboard'])
        if key not in seen:
            seen.add(key)
            out.append(f)

    return out

# stats helper functions to test fragment output

def stats_frags(frags):
    lens = [len(f['seq']) for f in frags]
    with_ch = sum(1 for f in frags if f['contains_charge'])
    out = {
        'count': len(frags),
        'min_len': min(lens),
        'max_len': max(lens),
        'avg_len': sum(lens) / len(lens),
        'with_charging': with_ch,
        'without_charging': len(frags) - with_ch,
    }
    print(out)

def stats_ext(efrags):
    if not efrags:
        out = {'count': 0}
        print(out)
        return out
    lens = [len(f['seq']) for f in efrags]
    depot_end = sum(1 for f in efrags if f.get('ext_to') == 'D0')
    with_station = sum(1 for f in efrags if f.get('ext_station') is not None)
    out = {
        'count': len(efrags),
        'min_len': min(lens),
        'max_len': max(lens),
        'avg_len': sum(lens)/len(lens),
        'depot_extensions': depot_end,
        'extensions_with_station': with_station,
    }
    print(out)
    return out

# enumerate fragments and stats
instance = 'c101C6_2.txt'
data = read_instance(path / instance)

t0_preprocess = perf_counter() #time on

print("INSTANCE:", instance, "horizon=", data['horizon'], "stations=", len(data['S']), "pickups=", len(data['P']))

#base paths
base, pruned = enumerate_base_paths(data, 18)
frags = enumerate_fragments(data, base)

#restricted fragments
r_frags_dedup = dedup_exact(frags)
r_frags_meta = attach_metadata(data, r_frags_dedup, exclude_last_ef = False)
r_frags_undom = dominance_filter(r_frags_meta)
r_frags_undom = dedup_by_signature(r_frags_undom)
# r_frags_undom = enforce_start_states(r_frags_undom,data)
# r_frags_undom = augment_states(data, r_frags_undom)

#extended fragments
e_frags = extend_all_fragments(data, r_frags_undom)
e_frags_dedup = dedup_exact(e_frags)
e_frags_meta = attach_metadata(data, e_frags_dedup, exclude_last_ef = True)
e_frags_undom = dominance_filter(e_frags_meta)
e_frags_undom = dedup_by_signature(e_frags_undom)
e_frags_aug = e_frags_undom
# e_frags_aug = deduplicate_arcs(e_frags_undom + extract_subarcs(data,e_frags_undom))

print("Num arcs:", len(e_frags_aug))

t1_preprocess = perf_counter() #time off

# stats for fragments
if DEBUG:
    print(pruned)
    print("RF stats pre dominance filter")
    stats_frags(frags)
    print("RF stats post dominance filter")
    stats_frags(r_frags_undom)
    print("RF raw:", len(frags),"EF dedup:", len(r_frags_dedup), "RF meta:", len(r_frags_meta), "RF undominated:", len(r_frags_undom))
    print("EF stats pre dominance filtering")
    stats_ext(e_frags)
    print("EF stats post dominance filtering")
    stats_ext(e_frags_undom)
    print("EF raw:", len(e_frags), "EF dedup:", len(e_frags_dedup), "EF meta:", len(e_frags_meta), "EF undominated:",
          len(e_frags_undom))

###############
# start MILP formulation
# generally speaking, the network graph has the fragments as arcs, and the nodes are location 'states'

# all current fragments start at a pickup, we need additional depot arcs
def raw_depot_arcs(data):

    nodes = data['nodes']
    depot_sid = nodes[0][0]
    out = []

    for p in data['P']:
        p_sid = nodes[p][0]

        # Build a fragment dict
        seq = (depot_sid, p_sid)
        Tf, Ef, Lf = compute_T_E_L(data, seq)
        Df = compute_distance(data, seq)

        # depot parameters (many are empty)
        out.append({
            'seq': seq,
            'Start': depot_sid,
            'End': p_sid,
            'start_onboard': frozenset(),
            'end_onboard': frozenset({p_sid}),
            'contains_charge': False,
            'Tf': Tf,
            'Ef': Ef,
            'Lf': Lf,
            'Df': Df,
            'Emin': 0.0,
            'LocsC': frozenset(),
            'ext_to': p_sid,
            'ext_station': None,
            'ext_delivery': data['p2d'].get(p_sid),
        })
    return out

# build network nodes and arcs. Leverage fragments to determine feasible start/end states for nodes
def build_network(ef_list):

    node_id = {}
    arcs = []

    # node for each unique location/onboard combination
    def get_node(loc_sid, onboard_fs):
        state = (loc_sid, onboard_fs)
        # assign unique combo with index
        if state not in node_id:
            node_id[state] = len(node_id)
        return node_id[state]

    # iterate through each fragment, create nodes based on start and end, then generate arc data
    for idx, f in enumerate(ef_list):
        u = get_node(f['Start'], f['start_onboard'])
        v = get_node(f['End'], f['end_onboard'])
        arcs.append({
            'seq': f['seq'],
            'Start': f['Start'],
            'End': f['End'],
            'start_onboard': f['start_onboard'],
            'end_onboard': f['end_onboard'],
            'id': idx,
            'u': u,
            'v': v,
            'Tf': float(f['Tf']),
            'Ef': float(f['Ef']),
            'Lf': float(f['Lf']),
            'Df': float(f['Df']),
            'Emin': float(f.get('Emin', f.get('min_start_energy', 0.0))),
            'LocsC': f.get('LocsC', frozenset()),
        })

    return node_id, arcs

# callback helpers

# =========================
# DP subproblem (stations insertion), objective = min added distance
# Stations have no time windows (treated always open).
# where station TW covers planning horizon.
# =========================

def dp_leg_frontier_charge_to_full(data, u_sid, v_sid, t0, E0, max_station_visits=3):
    """
    Single skeleton leg u -> v, stations can be inserted.
    Returns a list of NONDOMINATED arrival labels at v.

    Each label is:
      (dist_leg, t_start_at_v, E_arr_at_v, path_tuple)

    Stations: always open (no TW).
    Destination v: TW enforced.
    Charging: if station visited, charge to full with time (CapE - E_arr)/rech.
    """
    sid_to_i = data['sid_to_i']
    dist_fn  = data['dist']
    tt_fn    = data['traveltime']
    en_fn    = data['energy']
    CapE     = data['CapE']
    rech     = data['rech']
    horizon  = data['horizon']

    v_i = sid_to_i[v_sid]
    v_ready = data['nodes'][v_i][6]
    v_due   = data['nodes'][v_i][7]

    station_sids = [data['nodes'][i][0] for i in data['S']]

    # PQ state: (dist_so_far, dep_time, -dep_energy, cur_sid, path_tuple, stations_used, visited_stations_frozenset)
    pq = []
    heapq.heappush(pq, (0.0, t0, -E0, u_sid, (u_sid,), 0, frozenset()))

    # nondominated labels per intermediate node (including stations): (dist, time, energy)
    best_at_node = {u_sid: [(0.0, t0, E0)]}

    # nondominated arrivals at v: (dist, t_start, E_arr, path)
    arrivals = []

    def dominates3(a, b, eps=1e-9):
        # a,b = (dist, time, energy)
        da, ta, ea = a
        db, tb, eb = b
        return (da <= db + eps) and (ta <= tb + eps) and (ea >= eb - eps)

    def keep_nondominated_list(L, newlab3):
        # keep nondominated in list of (d,t,e)
        for lab in L:
            if dominates3(lab, newlab3):
                return L, False
        out = [lab for lab in L if not dominates3(newlab3, lab)]
        out.append(newlab3)
        return out, True

    def dominates_arr(a, b, eps=1e-9):
        # a,b = (dist, t_start, E_arr, path)
        da, ta, ea, _ = a
        db, tb, eb, _ = b
        return (da <= db + eps) and (ta <= tb + eps) and (ea >= eb - eps)

    def keep_arrivals(arrivals, cand):
        for lab in arrivals:
            if dominates_arr(lab, cand):
                return arrivals
        out = [lab for lab in arrivals if not dominates_arr(cand, lab)]
        out.append(cand)
        # optional: sort for neatness
        out.sort(key=lambda x: (x[0], x[1], -x[2]))
        return out

    while pq:
        d_sofar, t_dep, negE, cur_sid, path, k, visitedS = heapq.heappop(pq)
        E_dep = -negE

        # Explore either go to v, or to a station (if budget remains)
        next_nodes = [v_sid]
        if k < max_station_visits:
            next_nodes += station_sids

        for nxt_sid in next_nodes:
            if nxt_sid == cur_sid:
                continue
            if nxt_sid in visitedS:
                continue

            ui = sid_to_i[cur_sid]
            ni = sid_to_i[nxt_sid]

            e_need = en_fn(ui, ni)
            if e_need > E_dep + 1e-9:
                continue

            t_arr = t_dep + tt_fn(ui, ni)
            E_arr = E_dep - e_need
            d_new = d_sofar + dist_fn(ui, ni)

            if nxt_sid == v_sid:
                t_start = max(t_arr, v_ready)
                if t_start > v_due + 1e-9:
                    continue
                cand = (d_new, t_start, E_arr, path + (nxt_sid,))
                arrivals = keep_arrivals(arrivals, cand)
                continue

            # station
            if t_arr > horizon + 1e-9:
                continue

            charge_time = (CapE - E_arr) / rech
            t_dep2 = t_arr + charge_time
            E_dep2 = CapE

            lab3 = (d_new, t_dep2, E_dep2)
            lst = best_at_node.get(nxt_sid, [])
            lst2, kept = keep_nondominated_list(lst, lab3)
            if not kept:
                continue
            best_at_node[nxt_sid] = lst2

            new_visitedS = visitedS | frozenset([nxt_sid])
            heapq.heappush(pq, (d_new, t_dep2, -E_dep2, nxt_sid, path + (nxt_sid,), k+1, new_visitedS))

    return arrivals  # may be empty


def dp_route_min_dist(data, skeleton_sids, t0=0.0, E0=None,
                  max_station_visits_per_leg=DP_MAX_STATION_VISITS_PER_LEG,
                  max_labels_per_node=DP_MAX_LABELS_PER_NODE):
    """
    Multi-leg DP over mandatory skeleton nodes (no stations in skeleton_sids).
    Returns:
      ok, best_dist, best_full_path_tuple, fail_index
    fail_index = i means failure on leg skeleton[i] -> skeleton[i+1].
    """
    if E0 is None:
        E0 = data['CapE']

    for sid in skeleton_sids:
        if is_station(data, sid):
            raise ValueError(f"skeleton contains station {sid}; filter stations out first")

    sid_to_i = data['sid_to_i']

    # label at mandatory node: (dist_so_far, dep_time, dep_energy, full_path_tuple)
    def dominates_label(a, b, eps=1e-9):
        da, ta, ea, _ = a
        db, tb, eb, _ = b
        return (da <= db + eps) and (ta <= tb + eps) and (ea >= eb - eps)

    def insert_label(L, newlab):
        for lab in L:
            if dominates_label(lab, newlab):
                return L
        out = [lab for lab in L if not dominates_label(newlab, lab)]
        out.append(newlab)
        out.sort(key=lambda x: (x[0], x[1], -x[2]))
        return out[:max_labels_per_node]

    start_sid = skeleton_sids[0]
    labels = {start_sid: [(0.0, t0, E0, (start_sid,))]}

    for i in range(len(skeleton_sids) - 1):
        u = skeleton_sids[i]
        v = skeleton_sids[i+1]

        next_labels_for_v = []

        for (d_sofar, t_dep, E_dep, path_sofar) in labels.get(u, []):
            arrivals = dp_leg_frontier_charge_to_full(
                data, u, v, t_dep, E_dep,
                max_station_visits=max_station_visits_per_leg
            )

            # each arrival is (d_leg, t_start_v, E_arr_v, path_leg)
            for (d_leg, t_start_v, E_arr_v, path_leg) in arrivals:
                # service at v
                v_i = sid_to_i[v]
                serv_v = data['nodes'][v_i][8]
                t_dep_v = t_start_v + serv_v
                E_dep_v = E_arr_v

                # stitch paths (avoid duplicating u)
                if path_sofar[-1] == path_leg[0]:
                    stitched = path_sofar + path_leg[1:]
                else:
                    stitched = path_sofar + path_leg

                newlab = (d_sofar + d_leg, t_dep_v, E_dep_v, stitched)
                next_labels_for_v.append(newlab)

        if not next_labels_for_v:
            return False, None, None, i  # failed on leg i

        L_v = []
        for lab in next_labels_for_v:
            L_v = insert_label(L_v, lab)
        labels[v] = L_v

    final_sid = skeleton_sids[-1]
    best = min(labels[final_sid], key=lambda x: (x[0], x[1], -x[2]))
    best_dist, best_t, best_E, best_path = best
    return True, best_dist, best_path, None

# build gurobi model + access licence

def make_gurobi_env():

    wls_id = os.getenv("GRB_WLSACCESSID", "").strip()
    wls_secret = os.getenv("GRB_WLSSECRET", "").strip()
    lic = os.getenv("GRB_LICENSEID", "").strip()

    if wls_id and wls_secret and lic:
        params = {
            "WLSACCESSID": wls_id,
            "WLSSECRET": wls_secret,
            "LICENSEID": int(lic),
        }
        return gp.Env(params=params)

    # return default env if there is no licence
    return gp.Env()

# is 208 even doable for instance c101C6? and for K=1 vehicles?
if DEBUG:
    def brute_force_best_k1(data):
        # operations are customer nodes C* that are pickups/deliveries
        nodes = data['nodes']
        sid_to_i = data['sid_to_i']
        CapL = data['CapL']

        # collect pickup and delivery SIDs
        pickups = [nodes[i][0] for i in data['P']]
        deliveries = [nodes[i][0] for i in data['D']]
        p2d = data['p2d']

        # demand by sid
        demand = {nodes[i][0]: nodes[i][5] for i in range(len(nodes))}

        # precedence: pickup must come before its delivery
        def ok_precedence(seq):
            pos = {sid: k for k, sid in enumerate(seq)}
            for p, d in p2d.items():
                if pos[p] > pos[d]:
                    return False
            return True

        # capacity feasibility along seq (starting empty)
        def ok_capacity(seq):
            load = 0.0
            for sid in seq:
                load += demand[sid]
                if load > CapL + 1e-9:
                    return False
                if load < -1e-9:  # shouldn't happen in 1-1 PDP
                    return False
            return abs(load) < 1e-6  # end empty
        # recursive generation of feasible sequences (topological + capacity pruning)
        best = None
        best_seq = None

        # maintain sets of available pickups/deliveries
        all_ops = pickups + deliveries
        # deliveries become available only after their pickup is visited
        def rec(prefix, visited, load):
            nonlocal best, best_seq
            if len(prefix) == len(all_ops):
                # full sequence
                seq = list(prefix)
                # evaluate via DP
                skeleton = ['D0'] + seq + ['D0']
                ok, dp_dist, dp_path, fail_i = dp_route_min_dist(data, skeleton, t0=0.0, E0=data['CapE'],
                                                    max_station_visits_per_leg=DP_MAX_STATION_VISITS_PER_LEG,
                                                    max_labels_per_node=DP_MAX_LABELS_PER_NODE)

                if ok:
                    if best is None or dp_dist < best:
                        best = dp_dist
                        best_seq = skeleton
                return

            # build candidate next ops
            candidates = []
            for sid in pickups:
                if sid not in visited:
                    # pickup always allowed if capacity allows
                    if load + demand[sid] <= CapL + 1e-9:
                        candidates.append(sid)
            for sid in deliveries:
                if sid not in visited:
                    # allowed only if its pickup already visited
                    p = data['d2p'][sid]
                    if p in visited:
                        # delivery reduces load (negative demand)
                        if load + demand[sid] >= -1e-9:
                            candidates.append(sid)

            for nxt in candidates:
                rec(prefix + (nxt,), visited | {nxt}, load + demand[nxt])

        rec(tuple(), set(), 0.0)
        return best, best_seq

    best_dist, best_skeleton = brute_force_best_k1(data)
    print("BRUTE FORCE K=1 best DP distance =", best_dist)
    print("BRUTE FORCE best skeleton =", best_skeleton)



##############################
####### model build ##########
##############################
##############################


def build_master_model(data, ef_undom, K_max, force_exact_K = False):

    # add start arcs to existing frags
    depot_arcs = raw_depot_arcs(data)
    all_frags = depot_arcs + list(ef_undom)

    # build state nodes + arc sets
    node_id, arcs = build_network(all_frags)

    # identify depot node (D0, empty onboard)
    depot_sid = data['nodes'][0][0]
    depot_node = (depot_sid, frozenset())
    if depot_node not in node_id:
        # ensure depot node exists even if no arcs created it
        node_id[depot_node] = len(node_id)
    depot_u = node_id[depot_node]

    # map which arcs enter a node
    in_arcs = {n: [] for n in node_id.values()}
    # which arcs leave a node
    out_arcs = {n: [] for n in node_id.values()}
    for a in arcs:
        out_arcs[a['u']].append(a['id'])
        in_arcs[a['v']].append(a['id'])

    # Pickup list by sid
    pickups = [data['nodes'][i][0] for i in data['P']]

    arc_visits_pickups = {}  # aid -> set of pickup SIDs visited by this arc

    for a in arcs:
        s = strip_stations(a['seq'],data)
        # exclude the first node to avoid boundary double-counting across consecutive arcs
        visited = set(s[1:])
        arc_visits_pickups[a['id']] = {sid for sid in visited if sid in pickups}

    # Map pickup sid -> arc ids that end at that pickup (across any onboard)
    end_at_pickup = {p: [] for p in pickups}
    for a in arcs:
        if a['End'] in end_at_pickup:
            end_at_pickup[a['End']].append(a['id'])

    arc_by_id = {a['id']: a for a in arcs}

    for p in pickups:
        good = 0
        for aid in end_at_pickup[p]:
            v = arc_by_id[aid]['v']
            if len(out_arcs[v]) > 0:
                good += 1

    # Create model (w/ licence)
    env = make_gurobi_env()
    m = gp.Model(env=env)

    X = {a['id']: m.addVar(vtype=gp.GRB.BINARY) for a in arcs}
    theta = m.addVar(vtype=gp.GRB.CONTINUOUS)

    for a in arcs:
        if 'Df' not in a:
            a['Df'] = compute_distance(data, a['seq'])

    # objective
    m.setObjective(gp.quicksum(a['Df'] * X[a['id']] for a in arcs) + theta, gp.GRB.MINIMIZE)

    # Flow conservation on all state nodes except depot node
    for n in node_id.values():
        if n == depot_u:
            continue
        m.addConstr(gp.quicksum(X[i] for i in in_arcs[n]) - gp.quicksum(X[i] for i in out_arcs[n]) == 0)
    #
    # Depot balance defines vehicle count y
    # Changes between <= and == for sweeping across all K
    if force_exact_K:
        m.addConstr(gp.quicksum(X[i] for i in out_arcs[depot_u]) == K_max)
        m.addConstr(gp.quicksum(X[i] for i in in_arcs[depot_u]) == K_max)
    else:
        m.addConstr(gp.quicksum(X[i] for i in out_arcs[depot_u]) <= K_max)
        m.addConstr(gp.quicksum(X[i] for i in in_arcs[depot_u]) <= K_max)

    # coverage: each pickup must be served once
    for p in pickups:
        m.addConstr(
            gp.quicksum(X[aid] for aid in arc_visits_pickups if p in arc_visits_pickups[aid]) == 1)

    M = max_dist(data)

    m.Params.LazyConstraints = 1
    m.Params.Threads = 1
    m.Params.OutputFlag = 0

    return m, X, arcs, node_id, depot_u, arc_by_id, theta, M

# is 362 even feasible for instance c104C10?

if RUN_OPTIONA:
    import random

    def random_feasible_skeletons(data, n_samples=5000, seed=1):
        random.seed(seed)

        nodes = data['nodes']
        CapL = data['CapL']
        demand = {nodes[i][0]: nodes[i][5] for i in range(len(nodes))}

        pickups = [nodes[i][0] for i in data['P']]
        deliveries = [nodes[i][0] for i in data['D']]
        p2d = data['p2d']
        d2p = data['d2p']

        all_ops = pickups + deliveries

        def is_valid(seq):
            seen = set()
            load = 0.0
            for sid in seq:
                # precedence
                if sid in deliveries and d2p[sid] not in seen:
                    return False
                load += demand[sid]
                if load < -1e-9 or load > CapL + 1e-9:
                    return False
                seen.add(sid)
            return abs(load) < 1e-6

        skels = []
        for _ in range(n_samples):
            perm = []
            available_p = pickups.copy()
            available_d = []

            load = 0.0
            while available_p or available_d:
                choices = []
                for p in available_p:
                    if load + demand[p] <= CapL + 1e-9:
                        choices.append(p)
                for d in available_d:
                    if load + demand[d] >= -1e-9:
                        choices.append(d)

                if not choices:
                    break

                sid = random.choice(choices)
                perm.append(sid)
                load += demand[sid]

                if sid in available_p:
                    available_p.remove(sid)
                    available_d.append(p2d[sid])
                else:
                    available_d.remove(sid)

            if len(perm) == len(all_ops) and is_valid(perm):
                skels.append(['D0'] + perm + ['D0'])

        return skels


    best = float('inf')
    best_skel = None

    skels = random_feasible_skeletons(data, n_samples=8000, seed=1)

    for skel in skels:
        ok, dp_dist, _, _ = dp_route_min_dist(
            data,
            skel,
            t0=0.0,
            E0=data['CapE'],
            max_station_visits_per_leg=DP_MAX_STATION_VISITS_PER_LEG,
            max_labels_per_node=DP_MAX_LABELS_PER_NODE

        )
        if ok and dp_dist < best:
            best = dp_dist
            best_skel = skel

    print_dp_path_structure("OPTION-A", data, best_skel, best)


if DEBUG_DIAGNOSTICS:
    m_tmp, x_tmp, arcs_full, node_id_tmp, depot_u_tmp, arc_by_id_full, theta_tmp, M_tmp = build_master_model(
        data, e_frags_aug, K_max=1, force_exact_K=True
    )

    def coverage_report_for_skeleton(skeleton, arcs_full, data):
        """
        For each consecutive (u, v) in the skeleton,
        report how many EF fragments can realise u->v consecutively
        in their station-stripped sequence.
        """
        print("\n=== COVERAGE REPORT ===")
        print("Skeleton:", skeleton)

        stripped_ef_seqs = [
            strip_stations(f['seq'], data)
            for f in arcs_full
        ]

        for u, v in zip(skeleton, skeleton[1:]):
            count = 0
            examples = []

            for seq in stripped_ef_seqs:
                for i in range(len(seq) - 1):
                    if seq[i] == u and seq[i + 1] == v:
                        count += 1
                        if len(examples) < 3:
                            examples.append(seq)

            print(f"\nLink {u} -> {v}:")
            print(f"  Supported by {count} EF(s)")
            if examples:
                print("  Example stripped EF seqs:")
                for ex in examples:
                    print("    ", ex)
            else:
                print("  *** NO EF SUPPORT ***")


    debug_skeleton = best_skel
    coverage_report_for_skeleton(debug_skeleton, e_frags_undom, data)


    def segmentation_coverage_report(target_skeleton, arcs_full, data, max_examples=5):
        """
        Determine whether target_skeleton can be represented as a concatenation
        of station-stripped arc sequences in arcs_full.
        """
        target = target_skeleton
        n = len(target)

        # Precompute each arc's stripped seq
        arc_stripped = {}
        for a in arcs_full:
            sid_seq = a['seq']
            stripped = strip_stations(sid_seq, data)
            arc_stripped[a['id']] = stripped

        # For each position i in target, find arcs whose stripped seq matches target[i:i+len(seq)]
        matches_from = [[] for _ in range(n)]
        for aid, seq in arc_stripped.items():
            L = len(seq)
            if L < 2:
                continue
            for i in range(n - L + 1):
                if target[i:i+L] == seq:
                    matches_from[i].append(aid)

        # DP to find any segmentation
        prev = [None] * (n + 1)   # prev[pos] = (prev_pos, arc_id)
        prev[0] = (-1, None)

        for i in range(n):
            if prev[i] is None:
                continue
            for aid in matches_from[i]:
                L = len(arc_stripped[aid])
                j = i + L
                if prev[j] is None:
                    prev[j] = (i, aid)

        feasible = prev[n] is not None
        print("\n=== SEGMENTATION COVERAGE REPORT ===")
        print("Target skeleton:", target)
        print("Feasible segmentation:", feasible)

        # Print useful diagnostics
        for i in range(n - 1):
            print(f"pos {i}: {target[i]} -> {target[i+1]}  | candidate arcs starting here: {len(matches_from[i])}")

        if not feasible:
            # show first unreachable position
            first_bad = next((i for i in range(n+1) if prev[i] is None), None)
            print("First unreachable prefix length:", first_bad)
            return False, None

        # Reconstruct one segmentation
        seg = []
        pos = n
        while pos != 0:
            i, aid = prev[pos]
            seg.append(aid)
            pos = i
        seg.reverse()

        print("\nOne segmentation (arc IDs):", seg)
        print("Segment sequences:")
        for aid in seg[:max_examples]:
            print("  ", aid, arc_stripped[aid])

        return True, seg


    debug_skeleton = best_skel
    segmentation_coverage_report(debug_skeleton, arcs_full, data)


    def near_miss_report(target_skeleton, arcs_full, data, focus_pos):
        """
        focus_pos is the index i such that we care about target[i] -> target[i+1]
        """

        if focus_pos < 0 or focus_pos >= len(target_skeleton) - 1:
            print(f"[near_miss_report] focus_pos={focus_pos} out of range for skeleton length {len(target_skeleton)}")
            return

        u = target_skeleton[focus_pos]
        v = target_skeleton[focus_pos + 1]

        print(f"\n=== NEAR MISS at pos {focus_pos}: {u} -> {v} ===")

        def strip(seq):
            return [sid for sid in seq if not is_station(data, sid)]

        # 1) arcs whose stripped seq STARTS at u
        starts_at_u = []
        for a in arcs_full:
            s = strip(a['seq'])
            if len(s) >= 2 and s[0] == u:
                starts_at_u.append((a['id'], s[:10]))

        print(f"Arcs starting at {u}: {len(starts_at_u)}")
        for aid, s in starts_at_u[:10]:
            print("  ", aid, s)

        # 2) arcs whose stripped seq contains u->v internally
        contains_uv = []
        for a in arcs_full:
            s = strip(a['seq'])
            for i in range(len(s) - 1):
                if s[i] == u and s[i + 1] == v:
                    contains_uv.append((a['id'], s[max(0, i-3): i+4], s[0]))
                    break

        print(f"Arcs containing {u}->{v} internally: {len(contains_uv)}")
        for aid, window, start in contains_uv[:10]:
            print("  ", aid, "start=", start, "window=", window)


    target = debug_skeleton
    for pos in [2,3,7,10,11]:  # the zeros
        near_miss_report(target, arcs_full, data, pos)


    def is_boundary_node(data, sid):
        return (sid == 'D0') or is_pickup(data, sid)

    def segmentation_report_pickup_boundaries(target_skeleton, arcs_full, data):
        """
        Can we tile the full operation skeleton using arcs, if we only allow
        segment boundaries at depot + pickup nodes (not deliveries)?
        """
        target = target_skeleton
        n = len(target)

        # boundary positions in the skeleton where a segment is allowed to START/END
        boundary_pos = [i for i,sid in enumerate(target) if is_boundary_node(data, sid)]
        boundary_set = set(boundary_pos)

        def strip(seq):
            return [sid for sid in seq if not is_station(data, sid)]

        # Precompute stripped seq for each arc
        arc_stripped = {}
        for a in arcs_full:
            arc_stripped[a['id']] = strip(a['seq'])

        # matches_from[i] = list of arc IDs whose stripped seq matches target[i:i+L]
        # AND i is a boundary start AND i+L-1 is a boundary end
        matches_from = [[] for _ in range(n)]
        for aid, seq in arc_stripped.items():
            L = len(seq)
            if L < 2:
                continue
            for i in range(n - L + 1):
                j = i + L - 1
                if i in boundary_set and j in boundary_set and target[i:i+L] == seq:
                    matches_from[i].append(aid)

        # DP over boundary positions
        prev = [None] * n
        prev[0] = (-1, None)  # start at position 0 (D0)

        for i in range(n):
            if prev[i] is None:
                continue
            for aid in matches_from[i]:
                L = len(arc_stripped[aid])
                j = i + L - 1
                if prev[j] is None:
                    prev[j] = (i, aid)

        feasible = prev[n-1] is not None

        print("\n=== PICKUP-BOUNDARY SEGMENTATION REPORT ===")
        print("Target skeleton:", target)
        print("Boundary positions:", boundary_pos)
        print("Feasible segmentation:", feasible)

        # Show candidate counts only at boundary starts (more meaningful)
        for i in boundary_pos[:-1]:
            print(f"start at pos {i} ({target[i]}): {len(matches_from[i])} candidate arc(s)")

        if not feasible:
            # Find the furthest reachable boundary
            reachable = [i for i in boundary_pos if prev[i] is not None]
            print("Reachable boundary positions:", reachable)
            print("Furthest reachable boundary:", max(reachable) if reachable else None)
            return False, None

        # Reconstruct segmentation
        seg = []
        pos = n - 1
        while pos != 0:
            i, aid = prev[pos]
            seg.append(aid)
            pos = i
        seg.reverse()

        print("One segmentation (arc IDs):", seg)
        print("Segment stripped sequences:")
        for aid in seg:
            print("  ", aid, arc_stripped[aid])

        return True, seg



    target = best_skel
    segmentation_report_pickup_boundaries(target, arcs_full, data)

    _ , segmentation_ids = segmentation_report_pickup_boundaries(target, arcs_full, data)

    def state_compatible_segmentation(target_skeleton, arcs_full, data):
        """
        Find a chain of arcs that:
          (i) has station-stripped sequences that concatenate to target_skeleton, with boundaries at depot+pickups
          (ii) is composable in the master state graph (arc.v == next_arc.u)
        """
        target = target_skeleton
        n = len(target)

        def strip(seq):
            return [sid for sid in seq if not is_station(data, sid)]

        def is_boundary(sid):
            return (sid == 'D0') or is_pickup(data, sid)

        boundary_pos = [i for i, sid in enumerate(target) if is_boundary(sid)]
        boundary_set = set(boundary_pos)

        # Precompute stripped seq for each arc
        arc_stripped = {a['id']: strip(a['seq']) for a in arcs_full}
        arc_by_id = {a['id']: a for a in arcs_full}

        # Build candidate arcs for each boundary start i: arcs whose stripped seq equals target[i:i+L]
        candidates_from = {i: [] for i in boundary_pos}
        for aid, seq in arc_stripped.items():
            L = len(seq)
            if L < 2:
                continue
            for i in boundary_pos:
                j = i + L - 1
                if j >= n:
                    continue
                if j in boundary_set and target[i:i+L] == seq:
                    candidates_from[i].append(aid)

        # DP over boundary positions with state compatibility
        # dp[pos][state_node] = (prev_pos, prev_state_node, arc_id)
        dp = {}
        start_pos = boundary_pos[0]  # should be 0
        initial_state = frozenset()
        dp[(start_pos, initial_state)] = [(-1, None, None)]  # None means "start of route"

        for i in boundary_pos[:-1]:
            # collect all entries at this boundary position
            current_entries = [(key, val) for key, val in dp.items() if key[0] == i]
            if not current_entries:
                continue

            for (pos, last_state), _ in current_entries:
                for aid in candidates_from[i]:
                    a = arc_by_id[aid]
                    end_pos = i + len(arc_stripped[aid]) - 1
                    end_state = a['v']
                    start_state = a['u']

                    # state compatibility:
                    # - if this is the first segment in the route, accept it
                    # - else require last_state == start_state

                    if last_state != start_state:
                        continue

                    key2 = (end_pos, end_state)

                    if key2 not in dp:
                        dp[key2] = []
                    dp[key2].append((pos, last_state, aid))

        final_pos = boundary_pos[-1]  # should be n-1
        # accept any end_state at final_pos
        finals = [(key, dp[key]) for key in dp if key[0] == final_pos]
        feasible = len(finals) > 0

        print("\n=== STATE-COMPATIBLE SEGMENTATION REPORT ===")
        print("Target skeleton:", target)
        print("Boundary positions:", boundary_pos)
        print("Feasible:", feasible)

        if not feasible:
            # show where it dies
            reachable = sorted(set(pos for (pos, _) in dp.keys()))
            print("Reachable boundary positions:", [p for p in boundary_pos if p in reachable])
            # also show candidate counts at each boundary
            for i in boundary_pos[:-1]:
                print(f"start at pos {i} ({target[i]}): {len(candidates_from[i])} candidates")
            return False, None

        # reconstruct one solution
        # pick any final
        (final_key, _) = finals[0]
        pos, state = final_key
        seg = []
        while True:
            prev_pos, prev_state, aid = dp[(pos, state)][0]
            if aid is None:
                break
            seg.append(aid)
            # move backwards: previous segment ended at prev_pos with prev_state
            # but prev_state is the "last_state" before taking aid; we need the state at prev_pos
            # We can recover it by scanning dp keys; easiest is to set it explicitly:
            # Here we use the start_state of the chosen arc as the predecessor state
            a = arc_by_id[aid]
            state = a['u'] if prev_state is not None else None
            pos = prev_pos

        seg.reverse()
        print("Arc chain (IDs):", seg)
        print("Arc stripped sequences:")
        for aid in seg:
            print("  ", aid, arc_stripped[aid], "u=", arc_by_id[aid]['u'], "v=", arc_by_id[aid]['v'])
        return True, seg



    target = best_skel
    state_compatible_segmentation(target, arcs_full, data)


# start callback (hopefully I know what I am doing here)
def callback(model, where, x_vars, arcs, node_id, depot_u, data, theta, M):
    if where != gp.GRB.Callback.MIPSOL:
        return

    # Selected arcs
    xsol = model.cbGetSolution(x_vars)
    choose = [a_id for a_id, val in xsol.items() if val > 0.5]
    if not choose:
        return

    # Build successor/predecessor on state nodes
    out_map = {}
    in_map = {}
    for a_id in choose:
        a = arcs[a_id]
        out_map.setdefault(a['u'], []).append(a_id)
        in_map.setdefault(a['v'], []).append(a_id)

    # find cycles that don't touch depot. track visited nodes from depot and see if any chosen arc lies outside.
    seen_nodes = set([depot_u])
    stack = [depot_u]
    while stack:
        n = stack.pop()
        for a_id in out_map.get(n, []):
            v = arcs[a_id]['v']
            if v not in seen_nodes:
                seen_nodes.add(v)
                stack.append(v)

    # any chosen arc with tail not reachable from depot implies a disconnected cycle or subtour
    bad_cycle_arcs = [a_id for a_id in choose if arcs[a_id]['u'] not in seen_nodes]
    if bad_cycle_arcs:
        # cut here b/c all arcs in that subtour can be selected together ie. |S| - 1
        model.cbLazy(gp.quicksum(x_vars[a_id] for a_id in bad_cycle_arcs) <= len(bad_cycle_arcs) - 1)
        return

    # extract routes starting from depot
    routes = []
    for a_id in out_map.get(depot_u, []):
        route = [a_id]
        cur = arcs[a_id]['v']
        # follow until depot (or dead end)
        while cur != depot_u:
            nxts = out_map.get(cur, [])
            if not nxts:
                break
            # integer solution should have 1 outgoing; if multiple, pick one (shouldn't happen)
            a_id2 = nxts[0]
            route.append(a_id2)
            cur = arcs[a_id2]['v']
            # safety against infinite loops
            if len(route) > len(choose) + 5:
                break
        routes.append(route)

    sid_to_i = data['sid_to_i']
    dist_fn = data['dist']

    # We'll compute DP-based feasibility/cost per route and aggregate delta_total across all routes
    delta_total = 0.0

    # Helper: build skeleton and edge->arc mapping from a route arc list
    def skeleton_and_cover(route_arc_ids):
        # Build the skeleton nodes in order (stations removed), and map each skeleton edge to the arc index that covers it
        cover_arc_index = []  # cover_arc_index[j] = index in route_arc_ids of arc covering edge j: skeleton[j]->skeleton[j+1]

        # Build per-arc skeleton subseqs and then align
        arc_skel = []
        for idx, a_idid in enumerate(route_arc_ids):
            seq = arcs[a_id]['seq']
            sk = [sid for sid in seq if not is_station(data, sid)]
            # sk should have at least start/end
            arc_skel.append((idx, sk))

        # Merge in order, tracking edge coverage by arc index
        # Start with first arc's skeleton
        if not arc_skel or not arc_skel[0][1]:
            return [], []

        skeleton = list(arc_skel[0][1])

        # edges in this first arc are covered by arc index 0
        for _ in range(len(skeleton) - 1):
            cover_arc_index.append(arc_skel[0][0])

        # Append subsequent arcs, avoiding duplicate join node
        for (aidx, sk) in arc_skel[1:]:
            if not sk:
                continue
            if skeleton and sk and skeleton[-1] == sk[0]:
                # append sk[1:]
                for sid in sk[1:]:
                    skeleton.append(sid)
                for _ in range(len(sk) - 1):
                    cover_arc_index.append(aidx)
            else:
                # discontinuity; still append
                for sid in sk:
                    skeleton.append(sid)
                for _ in range(len(sk) - 1):
                    cover_arc_index.append(aidx)

        return skeleton, cover_arc_index

    # --- process each route ---
    for route in routes:
        sid_seq = []
        for k, aid in enumerate(route):
            s = arc_by_id[aid]['seq']
            if k == 0:
                sid_seq = list(s)
            else:
                if sid_seq[-1] == s[0]:
                    sid_seq.extend(s[1:])
                else:
                    sid_seq.extend(s)
        skeleton = [sid for sid in sid_seq if not is_station(data, sid)]
        if len(skeleton) < 2:
            continue

        # Run multi-leg DP: station insertion allowed, objective min distance
        ok, dp_dist, dp_full_path, fail_i = dp_route_min_dist(
            data, skeleton, t0=0.0, E0=data['CapE'],
            max_station_visits_per_leg=DP_MAX_STATION_VISITS_PER_LEG,
            max_labels_per_node=DP_MAX_LABELS_PER_NODE
        )

        if DEBUG_CALLBACK:
            print("DP path:", dp_full_path)
            print("DP distance:", dp_dist)
            print("Stations used:", [s for s in dp_full_path if is_station(data, s)])
            print("Num stations:", sum(is_station(data, s) for s in dp_full_path))

        if not ok:
            # For now: cut the whole route (safe, not minimal).
            # Once everything is stable, we can tighten to minimal prefix.
            model.cbLazy(gp.quicksum(x_vars[aid] for aid in route) <= len(route) - 1)
            return

        # # Feasibility cut: minimal prefix up to failing edge fail_i
            # # fail_i corresponds to skeleton[fail_i] -> skeleton[fail_i+1]
            # # cover_arc_idx[j] gives arc-index covering edge j
            # # prefix must include all arcs up to arc-index cover_arc_idx[fail_i]
            # if fail_i is None or fail_i >= len(cover_arc_idx):
            #     # fallback: cut whole route
            #     S_prefix = route
            # else:
            #     max_arcpos = cover_arc_idx[fail_i]
            #     S_prefix = route[:max_arcpos + 1]
            # model.cbLazy(gp.quicksum(x_vars[aid] for aid in S_prefix) <= len(S_prefix) - 1)
            # return  # safe to return after adding a feasibility cut

        # Compute optimistic direct distance for skeleton (no stations)
        # direct_dist = 0.0
        # for u_sid, v_sid in zip(skeleton, skeleton[1:]):
        #     ui = sid_to_i[u_sid]
        #     vi = sid_to_i[v_sid]
        #     direct_dist += dist_fn(ui, vi)
        #
        # delta = dp_dist - direct_dist

        # Base cost consistent with master objective: sum of Df of chosen arcs on this route
        base_route_cost = 0.0
        for aid in route:
            base_route_cost += arcs[aid]['Df']

        delta = dp_dist - base_route_cost


        if DEBUG:
            print("CALLBACK: delta_total =", delta_total, "choose_arcs =", choose)

        if delta > 1e-6:
            delta_total += delta

    # --- Optimality cut for whole incumbent ---
    if delta_total > 1e-6:
        # Condition on selecting all chosen arcs
        S = choose
        model.cbLazy(theta >= delta_total - M*(len(S) - gp.quicksum(x_vars[a] for a in S)))


### some diagnostic functions###

def extract_routes_from_solution(chosen_arc_ids, arc_by_id, depot_sid='D0'):
    # index chosen arcs by Start node
    start_map = {}
    for aid in chosen_arc_ids:
        a = arc_by_id[aid]
        start_map.setdefault(a['Start'], []).append(aid)

    routes = []

    # depot-starting arcs define routes
    for aid0 in start_map.get(depot_sid, []):
        route = [aid0]
        cur_aid = aid0
        cur_end = arc_by_id[cur_aid]['End']

        while cur_end != depot_sid:
            nexts = start_map.get(cur_end, [])
            if len(nexts) != 1:
                raise RuntimeError(
                    f"Ambiguous or missing continuation at {cur_end}: {nexts}"
                )
            cur_aid = nexts[0]
            route.append(cur_aid)
            cur_end = arc_by_id[cur_aid]['End']

        routes.append(route)

    return routes

def route_distance_from_sids(data, sid_seq):
    dist = data['dist']         # helper from read_instance
    sid_to_i = data['sid_to_i'] # SID -> node index
    total = 0.0
    for u_sid, v_sid in zip(sid_seq, sid_seq[1:]):
        ui = sid_to_i[u_sid]
        vi = sid_to_i[v_sid]
        total += dist(ui, vi)
    return total

def stitch_sid_sequence(route, arc_by_id):
    sid_seq = []
    for k, aid in enumerate(route):
        s = arc_by_id[aid]['seq']
        if k == 0:
            sid_seq = list(s)
        else:
            if sid_seq[-1] == s[0]:
                sid_seq.extend(s[1:])
            else:
                sid_seq.extend(s)
    return sid_seq

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

# Sweep through all K values (max K = number of pickups)

best_obj = float('inf')
best_payload = None

K_upper = len(data['P'])

t0_solve = perf_counter()
for K in range(1, K_upper + 1):
    print(f"\n--- solving with exact K = {K} ---")

    m, x, arcs, node_id, depot_u, arc_by_id, theta, M = build_master_model(
        data, e_frags_aug, K_max=K, force_exact_K=True)

    def cb(model, where):
        return callback(model, where, x, arcs, node_id, depot_u, data, theta, M)

    m.optimize(cb)

    if m.SolCount == 0:
        print(f"K={K}: infeasible")
        continue

    chosen = [aid for aid, var in x.items() if var.X > 0.5]
    starts = [aid for aid in chosen if arc_by_id[aid]['Start'] == 'D0']

    print(f"K={K}: obj={m.ObjVal:.6f}, theta={theta.X:.6f}, depot_starts={len(starts)}")

    if m.ObjVal < best_obj:
        best_obj = m.ObjVal
        best_payload = {
            "K": K,
            "obj": m.ObjVal,
            "theta": theta.X,
            "chosen": chosen,
            "arc_by_id": arc_by_id
        }
t1_solve = perf_counter() #time off

print("\nBEST SOLUTION:")
print("K =", best_payload["K"])
print("Obj =", best_payload["obj"])
print("Theta =", best_payload["theta"])


chosen = best_payload["chosen"]
arc_by_id = best_payload["arc_by_id"]

routes = extract_routes_from_solution(chosen, arc_by_id)

# diagnose issues with
if DEBUG:
    if m.Status == gp.GRB.INFEASIBLE:
        m.computeIIS()
        m.write("master_iis.ilp")
        print("IIS written to master_iis.ilp")

    # Print chosen arcs if solved
    if m.SolCount > 0:
        chosen = [aid for aid, var in x.items() if var.X > 0.5]
        print("Chosen arcs:", chosen)
        for aid in chosen:
            a = arc_by_id[aid] if 'arc_by_id' in globals() else next(xx for xx in arcs if xx['id'] == aid)
            print(a['id'], a['Start'], "->", a['End'], "start_on", a['start_onboard'], "end_on", a['end_onboard'], "seq",
                  a['seq'])

pre_time = t1_preprocess - t0_preprocess
sol_time = t1_solve - t0_solve
print("Preprocessing time:", pre_time)
print("Solve time:", sol_time)
print("Total time:", pre_time + sol_time)


total_dp = 0.0

print("\n--- DP validation of BEST solution ---")

for r, route in enumerate(routes):
    sid_seq = stitch_sid_sequence(route, arc_by_id)
    skeleton = [sid for sid in sid_seq if not is_station(data, sid)]

    ok_dp, dp_dist, dp_path, fail_i = dp_route_min_dist(
        data,
        skeleton,
        t0=0.0,
        E0=data['CapE'],
        max_station_visits_per_leg=DP_MAX_STATION_VISITS_PER_LEG,
        max_labels_per_node=DP_MAX_LABELS_PER_NODE

    )

    print(f"\nRoute {r}")
    print("Skeleton:", skeleton)

    if ok_dp:
        print("DP realised path:", list(dp_path))
        print_dp_path_structure("POSTSOLVE", data, dp_path, dp_dist)
        ok_sim, fsid, rsn = simulate_route(data, list(dp_path))
        print("[POSTSOLVE] simulate_route =", ok_sim, fsid, rsn)
        total_dp += dp_dist
    else:
        print("DP INFEASIBLE at leg", fail_i)

print("\nTOTAL DP distance =", total_dp)
print("Theta =", best_payload["theta"])
print("Objective =", best_payload["obj"])

# segment finding
# target_segment = ('C11','C4','D0')
#
# matches = [f for f in e_frags_aug if f['seq'] == target_segment]
#
# print("Num matching arcs:", len(matches))
#
# for f in matches[:5]:
#     print("start_on:", f['start_onboard'], "end_on:", f['end_onboard'])
#
# found = []
#
# for f in e_frags_aug:
#     seq = list(f['seq'])
#     for i in range(len(seq) - 1):
#         if seq[i] == 'C11' and seq[i+1] == 'C4':
#             found.append(seq)
#

print("\n=== STATE TRANSITIONS (ACTUAL CHOSEN ROUTE) ===")
for ridx, route in enumerate(routes):
    print(f"Route {ridx}: arc IDs = {route}")
    for k in range(len(route)-1):
        a = arc_by_id[route[k]]
        b = arc_by_id[route[k+1]]
        print(f"{a['seq']} -> {b['seq']}")
        print("  end_on  :", a['end_onboard'])
        print("  start_on:", b['start_onboard'])
        print("  MATCH?  :", a['end_onboard'] == b['start_onboard'])
        print()

