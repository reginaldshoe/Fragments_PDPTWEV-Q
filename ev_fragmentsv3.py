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

path = Path.cwd() / "instances"

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
        sig = (f['seq'], f['Ef'], f['Lf'], f['Tf'], f['Emin'])
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
    if is_station(data, sid_j) and sid_j in seenS:
        return None, 'revisit_station'

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
    print("trim_base_path returning", len(frags), "frags")
    return frags

# generate all the restricted fragments for a set of base paths
def enumerate_fragments(data, base_paths):
    frags = []
    for bp in base_paths:
        frags.extend(trim_base_path(data, bp))
    return frags

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
        start_on = set(f['start_onboard'])
        end_on = set(f['end_onboard'])

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
instance = 'c101C6.txt'
data = read_instance(path / instance)
t0_preprocess = perf_counter()
base, pruned = enumerate_base_paths(data, 18)
print(pruned)
frags = enumerate_fragments(data, base)

print("RF stats pre dominance filter")
stats_frags(frags)

r_frags_dedup = dedup_exact(frags)
r_frags_meta = attach_metadata(data, r_frags_dedup, exclude_last_ef = False)
r_frags_undom = dominance_filter(r_frags_meta)
r_frags_undom = dedup_by_signature(r_frags_undom)

print("RF stats post dominance filter")
stats_frags(r_frags_undom)


print("RF raw:", len(frags),"EF dedup:", len(r_frags_dedup), "RF meta:", len(r_frags_meta), "RF undominated:", len(r_frags_undom))

e_frags = extend_all_fragments(data, r_frags_undom)
print("EF stats pre dominance filtering")
stats = stats_ext(e_frags)


e_frags_dedup = dedup_exact(e_frags)
e_frags_meta = attach_metadata(data, e_frags_dedup, exclude_last_ef = True)
e_frags_undom = dominance_filter(e_frags_meta)
e_frags_undom = dedup_by_signature(e_frags_undom)

print("EF stats post dominance filtering")
e_stats = stats_ext(e_frags_undom)
print("EF raw:", len(e_frags), "EF dedup:", len(e_frags_dedup), "EF meta:", len(e_frags_meta), "EF undominated:", len(e_frags_undom))

t1_preprocess = perf_counter()
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
            'Emin': float(f.get('Emin', f.get('min_start_energy', 0.0))),
            'LocsC': f.get('LocsC', frozenset()),
        })

    return node_id, arcs

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

def build_master_model(data, ef_undom, K_max):

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

    # Map pickup sid -> arc ids that end at that pickup (across any onboard)
    end_at_pickup = {p: [] for p in pickups}
    for a in arcs:
        if a['End'] in end_at_pickup:
            end_at_pickup[a['End']].append(a['id'])

    for p, arcs_p in end_at_pickup.items():
        print(p, len(arcs_p))

    arc_by_id = {a['id']: a for a in arcs}

    for p in pickups:
        good = 0
        for aid in end_at_pickup[p]:
            v = arc_by_id[aid]['v']
            if len(out_arcs[v]) > 0:
                good += 1
        print(p, "coverage arcs:", len(end_at_pickup[p]), "good (non-deadend):", good)

    # Create model (w/ licence)
    env = make_gurobi_env()
    m = gp.Model(env=env)

    X = {a['id']: m.addVar(vtype=gp.GRB.BINARY) for a in arcs}


    # objective
    m.setObjective(gp.quicksum(a['Tf'] * X[a['id']] for a in arcs), gp.GRB.MINIMIZE)

    # Flow conservation on all state nodes except depot node
    for n in node_id.values():
        if n == depot_u:
            continue
        m.addConstr(gp.quicksum(X[i] for i in in_arcs[n]) - gp.quicksum(X[i] for i in out_arcs[n]) == 0)
    #
    # # Depot balance defines vehicle count y
    m.addConstr(gp.quicksum(X[i] for i in out_arcs[depot_u]) <= K_max)
    m.addConstr(gp.quicksum(X[i] for i in in_arcs[depot_u]) <= K_max)

    # coverage: each pickup must be served once
    for p in pickups:
        m.addConstr(gp.quicksum(X[i] for i in end_at_pickup[p]) == 1)

    m.Params.LazyConstraints = 1
    m.Params.Threads = 1

    return m, X, arcs, node_id, depot_u, arc_by_id

# start callback (hopefully I know what I am doing here)
def callback(model, where, x_vars, arcs, node_id, depot_u, data):
    if where != gp.GRB.Callback.MIPSOL:
        return

    # Selected arcs
    xsol = model.cbGetSolution(x_vars)
    chosen = [a_id for a_id, val in xsol.items() if val > 0.5]
    if not chosen:
        return

    # Build successor/predecessor on state nodes
    out_map = {}
    in_map = {}
    for a_id in chosen:
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
    bad_cycle_arcs = [a_id for a_id in chosen if arcs[a_id]['u'] not in seen_nodes]
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
            if len(route) > len(chosen) + 5:
                break
        routes.append(route)

    # Feasibility simulate each route
    for route in routes:
        # Build full SID sequence by concatenating arc seqs (avoid duplicating boundary node)
        sid_seq = []
        arc_at_pos = []
        for t, aid in enumerate(route):
            s = arcs[aid]['seq']
            if t == 0:
                sid_seq.extend(list(s))
                arc_at_pos = [None] + [aid]*(len(s) - 1)
            else:
                # drop first if repeated
                if sid_seq and s and sid_seq[-1] == s[0]:
                    sid_seq.extend(list(s[1:]))
                    arc_at_pos.extend([aid]*(len(s) - 1))
                else:
                    sid_seq.extend(list(s))
                    arc_at_pos.extend([aid]*len(s))

        # Simulate on node indices using step()
        sid_to_i = data['sid_to_i']
        CapE = data['CapE']

        # initial state at depot index 0
        st = ((0,), 0, frozenset(), CapE, 0.0, frozenset(), frozenset(), frozenset(), 0, 0.0)

        fail_reason = None
        fail_at = None
        fail_pos = None
        fail_arc = None

        # skip the first node (should be depot)

        for pos, sid in enumerate(sid_seq[1:], start=1):
            j = data['sid_to_i'].get(sid)
            if j is None:
                fail_reason = "unknown_sid"
                fail_at = sid
                fail_pos = pos
                fail_arc = arc_at_pos[pos]
                break

            st2, reason = step(data, st, j)
            if st2 is None:
                fail_reason = reason
                fail_at = sid
                fail_pos = pos
                fail_arc = arc_at_pos[pos]
                break

            st = st2

        # Add a simple prefix cut: forbid selecting all arcs up to the arc where failure occurred.
        # Find the earliest arc index in the route whose seq contains fail_at.
        cut_upto = None

        if fail_reason is not None:
            cut_upto = route.index(fail_arc) if fail_arc in route else len(route) - 1
            S = route[:cut_upto + 1]
            model.cbLazy(gp.quicksum(x_vars[aid] for aid in S) <= len(S) - 1)

            print("FAIL", fail_reason, "route arcs", route, "fail_at", fail_at, "fail_arc", fail_arc)
            print("CUT size", len(S), "S", S)
            return

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

m, x, arcs, node_id, depot_u,arc_by_id = build_master_model(data, e_frags_undom,K_max=len(data['P']))

def cb(model, where):
    return callback(model, where, x, arcs, node_id, depot_u, data)

t0_solve = perf_counter()
m.optimize(cb)
t1_solve = perf_counter()

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

routes = extract_routes_from_solution(chosen, arc_by_id)
total_dist = 0.0

for r, route in enumerate(routes):
    sid_seq = stitch_sid_sequence(route, arc_by_id)
    print(f"\nRoute {r}:")
    print("  arc IDs :", route)
    print("  SID seq :", sid_seq)
    ok, fail_sid, reason = simulate_route(data, sid_seq)
    if ok:
        print(f"Route {r} is FEASIBLE")
    else:
        print(f"Route {r} FAILED at {fail_sid} reason={reason}")
    d = route_distance_from_sids(data, sid_seq)
    total_dist += d
    print(f"Route {r}: distance={d:.2f}  SID seq={sid_seq}")

print(f"TOTAL distance travelled: {total_dist:.2f}")
