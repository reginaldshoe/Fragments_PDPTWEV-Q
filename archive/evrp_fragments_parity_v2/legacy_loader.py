"""Load legacy EVRP functions from ev_fragmentsv3.py without running top-level solve code.

Environment:
- Python 3.12
- gurobipy expected in the target repo environment

Role: draft output v3 / parity harness patch.

v2b/v2c behaviour:
- Accepts either --legacy ev_fragmentsv3 or --legacy ev_fragmentsv3.py.
- Searches the current working directory for the .py file if a suffix-less stem is supplied.
"""

from __future__ import annotations

import ast
import hashlib
import linecache
from dataclasses import dataclass
from pathlib import Path
from types import MappingProxyType
from typing import Any, Iterable

DEFAULT_FUNCTIONS: tuple[str, ...] = (
    "read_instance", "is_station", "is_customer", "is_pickup", "is_delivery",
    "energy_ok_fullbatt", "compute_Emin", "max_dist", "earliest_delivery_possible",
    "best_station_between", "strip_stations", "cust_locs", "compute_T_E_L",
    "compute_distance", "dedup_exact", "dedup_by_signature", "attach_metadata",
    "dominates", "filter_by_key", "dominance_filter", "print_dp_path_structure",
    "step", "enumerate_base_paths", "trim_base_path", "enumerate_fragments",
    "compute_onboard_states", "extend_all_fragments", "stats_frags", "stats_ext",
    "raw_depot_arcs", "build_network", "dp_leg_frontier_charge_to_full",
    "dp_route_min_dist", "make_gurobi_env", "build_master_model", "callback",
    "extract_routes_from_solution", "route_distance_from_sids", "stitch_sid_sequence",
    "simulate_route",
)

SAFE_ASSIGNMENT_NAMES: tuple[str, ...] = (
    "DEBUG", "DEBUG_CALLBACK", "DEBUG_DIAGNOSTICS", "FORCE_CHAIN", "RUN_OPTIONA",
    "DIAG_SKELETON", "DP_MAX_STATION_VISITS_PER_LEG", "DP_MAX_LABELS_PER_NODE",
)

@dataclass(frozen=True)
class LegacyLoadResult:
    source_path: Path
    source_sha256: str
    loaded_functions: tuple[str, ...]
    missing_requested_functions: tuple[str, ...]
    namespace: MappingProxyType[str, Any]
    function_hashes: MappingProxyType[str, str]

def resolve_legacy_source(source: str | Path) -> Path:
    raw = Path(source)
    candidates: list[Path] = [raw]
    if raw.suffix == "":
        candidates.append(raw.with_suffix(".py"))
    candidates.append(Path.cwd() / raw.name)
    if raw.suffix == "":
        candidates.append(Path.cwd() / f"{raw.name}.py")
    seen: set[Path] = set()
    for candidate in candidates:
        candidate = candidate.expanduser()
        if candidate in seen:
            continue
        seen.add(candidate)
        if candidate.exists() and candidate.is_file():
            return candidate.resolve()
    tried = "\n".join(f"- {c}" for c in candidates)
    raise FileNotFoundError(
        "Legacy source not found. Tried:\n"
        f"{tried}\n\nUse --legacy ev_fragmentsv3.py, or run this command from the repo root."
    )

def _sha256_text(text: str) -> str:
    return hashlib.sha256(text.encode("utf-8")).hexdigest()

def _function_hash(node: ast.FunctionDef | ast.AsyncFunctionDef) -> str:
    clone = ast.fix_missing_locations(ast.parse(ast.unparse(node)).body[0])
    payload = ast.dump(clone, include_attributes=False)
    return hashlib.sha256(payload.encode("utf-8")).hexdigest()

def _is_safe_assignment(node: ast.stmt) -> bool:
    if not isinstance(node, ast.Assign):
        return False
    names = [target.id for target in node.targets if isinstance(target, ast.Name)]
    return bool(names) and all(name in SAFE_ASSIGNMENT_NAMES for name in names)

def _compile_and_exec(nodes: Iterable[ast.stmt], filename: str, namespace: dict[str, Any]) -> None:
    module = ast.Module(body=list(nodes), type_ignores=[])
    ast.fix_missing_locations(module)
    exec(compile(module, filename=filename, mode="exec"), namespace, namespace)

def load_legacy(source: str | Path = "ev_fragmentsv3.py", functions: Iterable[str] = DEFAULT_FUNCTIONS) -> LegacyLoadResult:
    source_path = resolve_legacy_source(source)
    source_text = source_path.read_text(encoding="utf-8")
    source_hash = _sha256_text(source_text)
    linecache.cache[str(source_path)] = (len(source_text), None, source_text.splitlines(keepends=True), str(source_path))
    tree = ast.parse(source_text, filename=str(source_path))
    requested = tuple(dict.fromkeys(functions))
    requested_set = set(requested)
    import_nodes: list[ast.stmt] = []
    assignment_nodes: list[ast.stmt] = []
    function_nodes: dict[str, ast.FunctionDef | ast.AsyncFunctionDef] = {}
    for node in tree.body:
        if isinstance(node, (ast.Import, ast.ImportFrom)):
            import_nodes.append(node)
        elif _is_safe_assignment(node):
            assignment_nodes.append(node)
        elif isinstance(node, (ast.FunctionDef, ast.AsyncFunctionDef)) and node.name in requested_set:
            function_nodes[node.name] = node
    namespace: dict[str, Any] = {"__file__": str(source_path), "__name__": "ev_fragmentsv3_parity_namespace"}
    _compile_and_exec(import_nodes + assignment_nodes, str(source_path), namespace)
    ordered = [node for node in tree.body if isinstance(node, (ast.FunctionDef, ast.AsyncFunctionDef)) and node.name in function_nodes]
    _compile_and_exec(ordered, str(source_path), namespace)
    loaded = tuple(node.name for node in ordered)
    missing = tuple(name for name in requested if name not in function_nodes)
    hashes = {name: _function_hash(function_nodes[name]) for name in loaded}
    return LegacyLoadResult(source_path, source_hash, loaded, missing, MappingProxyType(namespace), MappingProxyType(hashes))
