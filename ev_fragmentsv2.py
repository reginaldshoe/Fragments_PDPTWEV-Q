# Focusing on trying to develop fragment algo first.

# Implementation structure, tried to follow principles from Rist and Forbes
# 1. enumerate full 'base' paths (defined where onboard load is empty at the start and the end of path)
# 1a. using a helper algorithm to update the next step
# 2. trim paths to truncated restricted fragments (defined where path has exactly one switch from pickup to delivery)
# 3. extend truncated restricted fragments to the next pickup

import math
from pathlib import Path
import os
import gurobipy as gp

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

# dominance within same dom_key. Smaller Ef/Tf/Emin and larger Lf is better.
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

    # loop all possible starting point in base_path
    for a in range(p_len):
        start_onboard = frozenset(Pseq[:a])
        start_sid = Pseq[a]
        s0 = pos[start_sid]

        # loop all possible end points, track what is still on board
        for b in range(p_len):
            kept_delivery = Dseq[:d - b]
            removed_delivery = Dseq[d - b:]
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
            energy_req = 0.0
            for u, v in zip(subseq, subseq[1:]):
                energy_req += data['energy'](u, v)
                if nodes[v][1] == 'S' or nodes[v][2] == 'f':
                    break

            frags.append({
                'seq': seq_sids,
                'start_onboard': start_onboard,
                'end_onboard': frozenset(end_onboard),
                'contains_charge': any(nodes[i][1] == 'S' or nodes[i][2] == 'f' for i in subseq),
                'min_start_energy': energy_req,
            })

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
            out.append({
                **f,
                'ext_to': 'D0',
                'ext_station': None,
                'ext_delivery': None,
                # end_onboard unchanged
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
                'min_start_energy': f['min_start_energy'],
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
instance = 'c101C12_3.txt'
data = read_instance(path / instance)
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

# start MILP formulation
