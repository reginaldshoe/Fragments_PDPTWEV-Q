"""Gurobi lazy callback factory for connectivity and energy-DP validation.

Patch v2: uses stronger connectivity cuts for disconnected components.

A no-good cut of the form `sum(selected component arcs) <= len(component)-1`
only removes the exact disconnected cycle currently observed. That can lead to a
large number of callback iterations. This version cuts the disconnected state
component itself: for any selected component not containing the depot, require at
least one selected arc entering that component from outside.
"""
from __future__ import annotations
from typing import Any, Callable

from .energy_dp import dp_route_min_dist
from .route_tools import extract_routes_from_solution, stitch_sid_sequence, strip_stations_from_sids


def _component_node_set(route: list[int], arc_by_id: dict[int, dict[str, Any]]) -> set[int]:
    nodes: set[int] = set()
    for aid in route:
        nodes.add(arc_by_id[aid]['u'])
        nodes.add(arc_by_id[aid]['v'])
    return nodes


def _route_connected_to_depot(route: list[int], arc_by_id: dict[int, dict[str, Any]], depot_u: int | None) -> bool:
    if depot_u is None or not route:
        return True
    nodes = _component_node_set(route, arc_by_id)
    return depot_u in nodes and arc_by_id[route[0]]['u'] == depot_u and arc_by_id[route[-1]]['v'] == depot_u


def _entering_arc_ids(
    component_nodes: set[int],
    arc_by_id: dict[int, dict[str, Any]],
) -> list[int]:
    """Return all candidate arcs entering a component from outside."""
    return [
        aid
        for aid, arc in arc_by_id.items()
        if arc['v'] in component_nodes and arc['u'] not in component_nodes
    ]


def make_energy_callback(
    x_vars: dict[int, Any],
    arc_by_id: dict[int, dict[str, Any]],
    data: dict[str, Any],
    theta: Any | None = None,
    big_m: float | None = None,
    depot_sid: str = 'D0',
    depot_u: int | None = None,
) -> Callable[[Any, int], None]:
    import gurobipy as gp

    def callback(model: Any, where: int) -> None:
        if where != gp.GRB.Callback.MIPSOL:
            return

        chosen = [aid for aid, var in x_vars.items() if model.cbGetSolution(var) > 0.5]
        routes = extract_routes_from_solution(chosen, arc_by_id, depot_sid=depot_sid, depot_u=depot_u)

        # First separate connectivity. If the current incumbent contains any
        # disconnected component, cut those components and return immediately.
        # Do not spend time running DP on an incumbent that is already invalid.
        added_connectivity_cut = False
        for route in routes:
            if _route_connected_to_depot(route, arc_by_id, depot_u):
                continue

            component_nodes = _component_node_set(route, arc_by_id)
            entering = _entering_arc_ids(component_nodes, arc_by_id)

            if entering:
                # Stronger component cut: a selected component not containing the
                # depot must have at least one selected incoming arc from outside.
                model.cbLazy(gp.quicksum(x_vars[aid] for aid in entering) >= 1)
            else:
                # Fallback for pathological closed components with no possible
                # incoming arc in the candidate graph.
                model.cbLazy(gp.quicksum(x_vars[aid] for aid in route) <= len(route) - 1)

            added_connectivity_cut = True

        if added_connectivity_cut:
            return

        # Only run energy DP after the selected solution is one or more
        # depot-connected route components.
        for route in routes:
            skeleton = strip_stations_from_sids(data, stitch_sid_sequence(route, arc_by_id))
            ok, best_dist, _, _ = dp_route_min_dist(data, skeleton)
            route_expr = gp.quicksum(x_vars[aid] for aid in route)

            if not ok:
                model.cbLazy(route_expr <= len(route) - 1)
                continue

            if theta is not None and big_m is not None and best_dist is not None:
                selected = sum(float(arc_by_id[aid].get('Df', 0.0)) for aid in route)
                added = max(0.0, best_dist - selected)
                if added > 1e-9:
                    model.cbLazy(theta >= added - big_m * (len(route) - route_expr))

    return callback
