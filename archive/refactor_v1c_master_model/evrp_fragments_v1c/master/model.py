"""Gurobi master model construction."""
from __future__ import annotations
import os
from typing import Any
import gurobipy as gp
from ..node_utils import strip_stations
from ..resources import compute_distance, max_dist
from ..types import MasterBuildResult
from .depot_arcs import raw_depot_arcs
from .network import build_network


def make_gurobi_env() -> gp.Env:
    wls_id = os.getenv('GRB_WLSACCESSID', '').strip()
    wls_secret = os.getenv('GRB_WLSSECRET', '').strip()
    lic = os.getenv('GRB_LICENSEID', '').strip()
    if wls_id and wls_secret and lic:
        return gp.Env(params={'WLSACCESSID': wls_id, 'WLSSECRET': wls_secret, 'LICENSEID': int(lic)})
    return gp.Env()


def build_master_model(data: dict[str, Any], ef_undom: list[dict[str, Any]], k_max: int, force_exact_k: bool = False) -> MasterBuildResult:
    if k_max <= 0:
        raise ValueError('k_max must be positive.')
    depot_arcs = raw_depot_arcs(data)
    all_frags = depot_arcs + list(ef_undom)
    network = build_network(all_frags)
    node_id = network.node_id
    arcs = network.arcs

    depot_sid = data['nodes'][0][0]
    depot_node = (depot_sid, frozenset())
    if depot_node not in node_id:
        node_id[depot_node] = len(node_id)
    depot_u = node_id[depot_node]

    in_arcs = {n: [] for n in node_id.values()}
    out_arcs = {n: [] for n in node_id.values()}
    for arc in arcs:
        out_arcs[arc['u']].append(arc['id'])
        in_arcs[arc['v']].append(arc['id'])

    pickups = [data['nodes'][i][0] for i in data['P']]
    arc_visits_pickups = {}
    for arc in arcs:
        stripped = strip_stations(data, arc['seq'])
        visited = set(stripped[1:])
        arc_visits_pickups[arc['id']] = {sid for sid in visited if sid in pickups}

    arc_by_id = {arc['id']: arc for arc in arcs}

    env = make_gurobi_env()
    model = gp.Model(env=env)
    x = {arc['id']: model.addVar(vtype=gp.GRB.BINARY, name=f'x_{arc["id"]}') for arc in arcs}
    theta = model.addVar(vtype=gp.GRB.CONTINUOUS, name='theta')

    for arc in arcs:
        if 'Df' not in arc:
            arc['Df'] = compute_distance(data, arc['seq'])

    model.setObjective(gp.quicksum(arc['Df'] * x[arc['id']] for arc in arcs) + theta, gp.GRB.MINIMIZE)

    for node in node_id.values():
        if node == depot_u:
            continue
        model.addConstr(gp.quicksum(x[i] for i in in_arcs[node]) - gp.quicksum(x[i] for i in out_arcs[node]) == 0, name=f'flow_{node}')

    if force_exact_k:
        model.addConstr(gp.quicksum(x[i] for i in out_arcs[depot_u]) == k_max, name='depot_out_exact_k')
        model.addConstr(gp.quicksum(x[i] for i in in_arcs[depot_u]) == k_max, name='depot_in_exact_k')
    else:
        model.addConstr(gp.quicksum(x[i] for i in out_arcs[depot_u]) <= k_max, name='depot_out_max_k')
        model.addConstr(gp.quicksum(x[i] for i in in_arcs[depot_u]) <= k_max, name='depot_in_max_k')

    for pickup_sid in pickups:
        model.addConstr(gp.quicksum(x[aid] for aid, visited in arc_visits_pickups.items() if pickup_sid in visited) == 1, name=f'cover_{pickup_sid}')

    big_m = max_dist(data)
    model.Params.LazyConstraints = 1
    model.Params.Threads = 1
    model.Params.OutputFlag = 0

    return MasterBuildResult(model=model, x=x, arcs=arcs, node_id=node_id, depot_u=depot_u, arc_by_id=arc_by_id, theta=theta, big_m=big_m)
