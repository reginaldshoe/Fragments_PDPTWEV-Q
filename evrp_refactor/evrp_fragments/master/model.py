"""Gurobi master model construction."""
from __future__ import annotations
import os
import gurobipy as gp
from ..node_utils import strip_stations
from ..resources import compute_distance, max_dist
from .depot_arcs import raw_depot_arcs
from .network import build_network


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
