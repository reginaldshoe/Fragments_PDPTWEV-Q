"""Consolidated-v1e pipeline without legacy_core in the runtime path.

Environment:
- Python 3.12
- gurobipy available in the target environment

Role: draft output v1e / remove legacy runtime dependency candidate.

This stage keeps legacy_core.py in the repository as provenance/reference, but
pipeline_v1e does not import it or pass it through dependency_bridge. Runtime
uses only fragment_core.py, master_core.py, callback_core.py and
dependency_bridge.py.

No algorithm bodies, callback signatures, DP logic, route extraction or model
formulation are changed.
"""

from __future__ import annotations

import json
from dataclasses import asdict, dataclass
from pathlib import Path
from typing import Any

from . import callback_core as cbcore
from . import dependency_bridge as deps
from . import fragment_core as frag
from . import master_core as master


@dataclass(frozen=True)
class ConsolidatedV1eRunSummary:
    package: str
    fragment_core_source_sha256: str | None
    master_core_source_sha256: str | None
    callback_core_source_sha256: str | None
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


def _ensure_generated() -> None:
    if not bool(getattr(frag, "FRAGMENT_CORE_GENERATED", False)):
        raise RuntimeError("fragment_core.py has not been generated. Run build_consolidated_v1a_fragments first.")
    if not bool(getattr(master, "MASTER_CORE_GENERATED", False)):
        raise RuntimeError("master_core.py has not been generated. Run build_consolidated_v1b_master first.")
    if not bool(getattr(cbcore, "CALLBACK_CORE_GENERATED", False)):
        raise RuntimeError("callback_core.py has not been generated. Run build_consolidated_v1c_callback first.")


def _get_var_value(var: Any) -> float:
    for attr in ("X", "x"):
        if hasattr(var, attr):
            return float(getattr(var, attr))
    return float(var)


def build_fragment_sets(instance: str | Path, max_base_path_len: int) -> dict[str, Any]:
    _ensure_generated()
    deps.bind_all_static_dependencies(frag, master, cbcore)

    data = frag.read_instance(Path(instance))
    deps.bind_data_context(data, frag, master, cbcore)

    base_result = frag.enumerate_base_paths(data, max_base_path_len)
    if isinstance(base_result, tuple) and len(base_result) == 2:
        base_paths, pruned = base_result
    else:
        base_paths, pruned = base_result, None

    restricted_raw = frag.enumerate_fragments(data, base_paths)
    restricted_dedup = frag.dedup_exact(restricted_raw)
    restricted_meta = frag.attach_metadata(data, restricted_dedup, exclude_last_ef=False)
    restricted_undominated = frag.dominance_filter(restricted_meta)
    restricted_undominated = frag.dedup_by_signature(restricted_undominated)

    extended_raw = frag.extend_all_fragments(data, restricted_undominated)
    extended_dedup = frag.dedup_exact(extended_raw)
    extended_meta = frag.attach_metadata(data, extended_dedup, exclude_last_ef=True)
    extended_undominated = frag.dominance_filter(extended_meta)
    extended_undominated = frag.dedup_by_signature(extended_undominated)

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


def run_solver(instance: str | Path, max_base_path_len: int, k_max: int, force_exact_k: bool, use_callback: bool) -> ConsolidatedV1eRunSummary:
    sets = build_fragment_sets(instance, max_base_path_len)
    data = sets["data"]
    ef = sets["extended_undominated"]

    deps.bind_fragment_context(ef, master, cbcore)
    deps.bind_all_static_dependencies(frag, master, cbcore)

    model_result = master.build_master_model(data, ef, k_max, force_exact_K=force_exact_k)
    if not isinstance(model_result, tuple) or len(model_result) < 8:
        raise RuntimeError("build_master_model did not return the expected 8-tuple.")

    m, x_vars, arcs, node_id, depot_u, arc_by_id, theta, big_m = model_result[:8]

    deps.bind_callback_context(
        data=data,
        extended_fragments=ef,
        arcs=arcs,
        arc_by_id=arc_by_id,
        node_id=node_id,
        depot_u=depot_u,
        theta=theta,
        big_m=big_m,
        x_vars=x_vars,
        callback_module=cbcore,
    )
    deps.bind_all_static_dependencies(frag, master, cbcore)

    if use_callback:
        def _cb(model: Any, where: Any) -> None:
            cbcore.callback(model, where, x_vars, arcs, node_id, depot_u, data, theta, big_m)
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

    return ConsolidatedV1eRunSummary(
        package="evrp_fragments_consolidated_v1e_remove_legacy_runtime",
        fragment_core_source_sha256=getattr(frag, "FRAGMENT_CORE_SOURCE_SHA256", None),
        master_core_source_sha256=getattr(master, "MASTER_CORE_SOURCE_SHA256", None),
        callback_core_source_sha256=getattr(cbcore, "CALLBACK_CORE_SOURCE_SHA256", None),
        instance=str(Path(instance)),
        max_base_path_len=max_base_path_len,
        k_max=k_max,
        force_exact_k=force_exact_k,
        use_callback=use_callback,
        base_paths=len(sets["base_paths"]),
        restricted_raw=len(sets["restricted_raw"]),
        restricted_dedup=len(sets["restricted_dedup"]),
        restricted_meta=len(sets["restricted_meta"]),
        restricted_undominated=len(sets["restricted_undominated"]),
        extended_raw=len(sets["extended_raw"]),
        extended_dedup=len(sets["extended_dedup"]),
        extended_meta=len(sets["extended_meta"]),
        extended_undominated=len(sets["extended_undominated"]),
        objective=objective,
        theta=theta_value,
        selected_arc_count=selected_count,
        selected_arc_ids=selected_ids,
        status=status,
    )


def summary_to_json(summary: ConsolidatedV1eRunSummary) -> str:
    return json.dumps(asdict(summary), indent=2, sort_keys=True)
