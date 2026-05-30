"""Gurobi lazy-callback integration for energy-DP route validation."""
from __future__ import annotations
from typing import Any, Callable

from .energy_dp import dp_route_min_dist
from .route_tools import extract_routes_from_solution, stitch_sid_sequence, strip_stations_from_sids


def make_energy_callback(
    x_vars: dict[int, Any],
    arc_by_id: dict[int, dict[str, Any]],
    data: dict[str, Any],
    theta: Any | None = None,
    big_m: float | None = None,
    depot_sid: str = 'D0',
) -> Callable[[Any, int], None]:
    """Create a Gurobi callback that checks selected routes with the energy DP.

    The callback adds a no-good cut when the DP cannot realise a selected route.
    If `theta` and `big_m` are supplied, it also adds a basic lower-bound cut on
    theta for routes whose realised DP distance exceeds the selected skeleton
    distance. This is intentionally conservative and should be compared against
    the legacy callback before relying on solve results.
    """
    import gurobipy as gp

    def callback(model: Any, where: int) -> None:
        if where != gp.GRB.Callback.MIPSOL:
            return
        chosen = [aid for aid, var in x_vars.items() if model.cbGetSolution(var) > 0.5]
        if not chosen:
            return
        routes = extract_routes_from_solution(chosen, arc_by_id, depot_sid=depot_sid)
        for route in routes:
            sid_seq = stitch_sid_sequence(route, arc_by_id)
            skeleton = strip_stations_from_sids(data, sid_seq)
            ok, best_dist, _full_path, _fail_index = dp_route_min_dist(data, skeleton)
            route_expr = gp.quicksum(x_vars[aid] for aid in route)
            if not ok:
                model.cbLazy(route_expr <= len(route) - 1)
                continue
            if theta is not None and big_m is not None and best_dist is not None:
                selected_distance = sum(float(arc_by_id[aid].get('Df', 0.0)) for aid in route)
                added_distance = max(0.0, best_dist - selected_distance)
                if added_distance > 1e-9:
                    model.cbLazy(theta >= added_distance - big_m * (len(route) - route_expr))
    return callback
