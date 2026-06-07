"""Runtime loader for the legacy ev_fragmentsv3 callback stack.

This module deliberately avoids copy/pasting legacy function bodies. Instead, it
parses the local `ev_fragmentsv3.py`, extracts imports, simple global constants,
and function definitions, and executes only those definitions in an isolated
namespace. Script-level solve code is not executed.
"""
from __future__ import annotations

import ast
from pathlib import Path
from types import MappingProxyType
from typing import Any, Iterable

TARGET_FUNCTIONS = {
    'callback',
    'dp_leg_frontier_charge_to_full',
    'dp_route_min_dist',
    'extract_routes_from_solution',
    'stitch_sid_sequence',
    'route_distance_from_sids',
    'simulate_route',
    'print_dp_path_structure',
    'is_station',
    'is_customer',
    'is_pickup',
    'is_delivery',
    'strip_stations',
    'max_dist',
}

# Include all function definitions by default because the legacy callback may
# call small helpers that are not obvious from the signature. We still exclude
# script-level executable statements.
INCLUDE_ALL_FUNCTIONS = True

SAFE_GLOBAL_ASSIGNMENTS = {
    'DEBUG',
    'DEBUG_CALLBACK',
    'DEBUG_DIAGNOSTICS',
    'FORCE_CHAIN',
    'RUN_OPTIONA',
    'DIAG_SKELETON',
    'DP_MAX_STATION_VISITS_PER_LEG',
    'DP_MAX_LABELS_PER_NODE',
}


def _is_safe_assignment(node: ast.AST) -> bool:
    if not isinstance(node, ast.Assign):
        return False
    for target in node.targets:
        if not isinstance(target, ast.Name):
            return False
        if target.id not in SAFE_GLOBAL_ASSIGNMENTS:
            return False
    return True


def _filter_module_tree(source_text: str) -> ast.Module:
    tree = ast.parse(source_text)
    body: list[ast.stmt] = []
    for node in tree.body:
        if isinstance(node, (ast.Import, ast.ImportFrom)):
            body.append(node)
        elif _is_safe_assignment(node):
            body.append(node)  # constants only
        elif isinstance(node, (ast.FunctionDef, ast.AsyncFunctionDef)):
            if INCLUDE_ALL_FUNCTIONS or node.name in TARGET_FUNCTIONS:
                body.append(node)
    module = ast.Module(body=body, type_ignores=[])
    ast.fix_missing_locations(module)
    return module


def load_legacy_namespace(source_path: str | Path = 'ev_fragmentsv3.py') -> MappingProxyType[str, Any]:
    path = Path(source_path)
    if not path.exists():
        raise FileNotFoundError(f'Legacy source not found: {path}')
    source_text = path.read_text(encoding='utf-8')
    module = _filter_module_tree(source_text)
    namespace: dict[str, Any] = {'__name__': 'ev_fragmentsv3_legacy_runtime'}
    exec(compile(module, str(path), 'exec'), namespace)
    missing = [name for name in ['callback', 'dp_route_min_dist', 'dp_leg_frontier_charge_to_full'] if name not in namespace]
    if missing:
        raise RuntimeError(f'Legacy runtime is missing required functions: {missing}')
    return MappingProxyType(namespace)


def make_legacy_callback(
    source_path: str | Path,
    x_vars: dict[int, Any],
    arcs: list[dict[str, Any]],
    node_id: dict[Any, int],
    depot_u: int,
    data: dict[str, Any],
    theta: Any,
    big_m: float,
):
    """Return a Gurobi callback closure that calls the legacy callback exactly.

    The legacy callback signature is:

        callback(model, where, x_vars, arcs, node_id, depot_u, data, theta, M)
    """
    namespace = load_legacy_namespace(source_path)
    legacy_callback = namespace['callback']

    def _callback(model: Any, where: int) -> None:
        return legacy_callback(model, where, x_vars, arcs, node_id, depot_u, data, theta, big_m)

    return _callback


def get_legacy_function(source_path: str | Path, name: str):
    namespace = load_legacy_namespace(source_path)
    if name not in namespace:
        raise KeyError(f'Legacy function not found: {name}')
    return namespace[name]
