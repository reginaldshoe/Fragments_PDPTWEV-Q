"""Parity pipeline using mechanically loaded legacy functions.

Role: draft output v3 / parity harness.

v2c patch:
- Synchronises globals across AST-loaded legacy functions before invoking them.
  This preserves the monolithic script behaviour where helpers such as cust_locs
  can see top-level names like data.
"""

from __future__ import annotations

import json
from dataclasses import asdict, dataclass
from pathlib import Path
from typing import Any

from .legacy_loader import load_legacy

@dataclass(frozen=True)
class ParityRunSummary:
    source: str
    source_sha256: str
    instance: str
    max_base_path_len: int
    k_max: int
    force_exact_k: bool
    use_callback: bool
    base_paths: int
    restricted_raw: int
    restricted_dedup: int
    restricted_meta: int
    restricted_undominated: int
    extended_raw: int
    extended_dedup: int
    extended_meta: int
    extended_undominated: int
    objective: float | None
    theta: float | None
    selected_arc_count: int | None
    selected_arc_ids: list[int] | None
    status: int | None
    missing_legacy_functions: tuple[str, ...]

def _sync_function_globals(ns: dict[str, Any]) -> None:
    """Synchronise AST-loaded function global namespaces.

    The legacy file is monolithic. Some helper functions intentionally or
    accidentally rely on names in module globals. When this harness copies the
    namespace for orchestration, those globals must be pushed back into each
    loaded function's own __globals__ mapping.
    """
    for value in list(ns.values()):
        globals_dict = getattr(value, "__globals__", None)
        if isinstance(globals_dict, dict):
            globals_dict.update(ns)

def _get_var_value(var: Any) -> float:
    for attr in ("X", "x"):
        if hasattr(var, attr):
            return float(getattr(var, attr))
    return float(var)

def build_legacy_fragment_sets(ns: dict[str, Any], instance: Path, max_base_path_len: int) -> dict[str, Any]:
    _sync_function_globals(ns)
    data = ns["read_instance"](instance)
    ns["data"] = data
    _sync_function_globals(ns)

    base_result = ns["enumerate_base_paths"](data, max_base_path_len)
    if isinstance(base_result, tuple) and len(base_result) == 2:
        base_paths, pruned = base_result
    else:
        base_paths, pruned = base_result, None

    restricted_raw = ns["enumerate_fragments"](data, base_paths)
    restricted_dedup = ns["dedup_exact"](restricted_raw)
    restricted_meta = ns["attach_metadata"](data, restricted_dedup, exclude_last_ef=False)
    restricted_undominated = ns["dominance_filter"](restricted_meta)
    restricted_undominated = ns["dedup_by_signature"](restricted_undominated)

    extended_raw = ns["extend_all_fragments"](data, restricted_undominated)
    extended_dedup = ns["dedup_exact"](extended_raw)
    extended_meta = ns["attach_metadata"](data, extended_dedup, exclude_last_ef=True)
    extended_undominated = ns["dominance_filter"](extended_meta)
    extended_undominated = ns["dedup_by_signature"](extended_undominated)

    return {
        "data": data,
        "pruned": pruned,
        "base_paths": base_paths,
        "restricted_raw": restricted_raw,
        "restricted_dedup": restricted_dedup,
        "restricted_meta": restricted_meta,
        "restricted_undominated": restricted_undominated,
        "extended_raw": extended_raw,
        "extended_dedup": extended_dedup,
        "extended_meta": extended_meta,
        "extended_undominated": extended_undominated,
    }

def run_parity_solver(source: str | Path, instance: str | Path, max_base_path_len: int, k_max: int, force_exact_k: bool, use_callback: bool) -> ParityRunSummary:
    loaded = load_legacy(source)
    ns = dict(loaded.namespace)
    _sync_function_globals(ns)
    sets = build_legacy_fragment_sets(ns, Path(instance), max_base_path_len)
    data = sets["data"]
    ef = sets["extended_undominated"]
    ns.update({"data": data, "e_frags_aug": ef})
    _sync_function_globals(ns)

    model_result = ns["build_master_model"](data, ef, k_max, force_exact_K=force_exact_k)
    if not isinstance(model_result, tuple) or len(model_result) < 8:
        raise RuntimeError("Legacy build_master_model did not return the expected 8-tuple.")

    m, x_vars, arcs, node_id, depot_u, arc_by_id, theta, big_m = model_result[:8]
    ns.update({
        "data": data,
        "e_frags_aug": ef,
        "arc_by_id": arc_by_id,
        "arcs_full": arcs,
        "node_id_tmp": node_id,
        "depot_u_tmp": depot_u,
        "theta_tmp": theta,
        "M_tmp": big_m,
        "x_tmp": x_vars,
    })
    _sync_function_globals(ns)

    if use_callback:
        def _cb(model: Any, where: Any) -> None:
            ns["callback"](model, where, x_vars, arcs, node_id, depot_u, data, theta, big_m)
        m.optimize(_cb)
    else:
        m.optimize()

    status = int(getattr(m, "Status", getattr(m, "status", -1)))
    objective = None
    if hasattr(m, "ObjVal"):
        try:
            objective = float(m.ObjVal)
        except Exception:
            objective = None
    try:
        theta_value = _get_var_value(theta)
    except Exception:
        theta_value = None
    try:
        selected_ids = [int(aid) for aid, var in x_vars.items() if _get_var_value(var) > 0.5]
        selected_count = len(selected_ids)
    except Exception:
        selected_ids = None
        selected_count = None

    return ParityRunSummary(
        source=str(loaded.source_path), source_sha256=loaded.source_sha256,
        instance=str(Path(instance)), max_base_path_len=max_base_path_len,
        k_max=k_max, force_exact_k=force_exact_k, use_callback=use_callback,
        base_paths=len(sets["base_paths"]), restricted_raw=len(sets["restricted_raw"]),
        restricted_dedup=len(sets["restricted_dedup"]), restricted_meta=len(sets["restricted_meta"]),
        restricted_undominated=len(sets["restricted_undominated"]), extended_raw=len(sets["extended_raw"]),
        extended_dedup=len(sets["extended_dedup"]), extended_meta=len(sets["extended_meta"]),
        extended_undominated=len(sets["extended_undominated"]), objective=objective,
        theta=theta_value, selected_arc_count=selected_count, selected_arc_ids=selected_ids,
        status=status, missing_legacy_functions=loaded.missing_requested_functions,
    )

def summary_to_json(summary: ParityRunSummary) -> str:
    return json.dumps(asdict(summary), indent=2, sort_keys=True)
