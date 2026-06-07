"""Consolidated-v1b pipeline using split fragment and master layers.

Environment:
- Python 3.12
- gurobipy available in the target environment

Role: draft output v1b patch 2 / modular split candidate.

Patch 2 fixes cross-module legacy helper dependencies in master_core. The
mechanically extracted master functions still reference fragment-layer helpers
such as compute_T_E_L and compute_distance. In this stage we preserve behaviour
by injecting fragment helper names into master_core globals before model build.
"""

from __future__ import annotations

import json
from dataclasses import asdict, dataclass
from pathlib import Path
from typing import Any

from . import fragment_core as frag
from . import legacy_core as core
from . import master_core as master


FRAGMENT_HELPERS_FOR_MASTER: tuple[str, ...] = (
    "is_station",
    "is_customer",
    "is_pickup",
    "is_delivery",
    "compute_T_E_L",
    "compute_distance",
    "compute_Emin",
    "cust_locs",
)


@dataclass(frozen=True)
class ConsolidatedV1bRunSummary:
    package: str
    fragment_core_source_sha256: str | None
    master_core_source_sha256: str | None
    legacy_core_source_sha256: str | None
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
    if not bool(getattr(core, "GENERATED", False)):
        raise RuntimeError("legacy_core.py has not been generated. Run build_consolidated_v1 first.")
    if not bool(getattr(frag, "FRAGMENT_CORE_GENERATED", False)):
        raise RuntimeError("fragment_core.py has not been generated. Run build_consolidated_v1a_fragments first.")
    if not bool(getattr(master, "MASTER_CORE_GENERATED", False)):
        raise RuntimeError("master_core.py has not been generated. Run build_consolidated_v1b_master first.")


def _sync_module_globals(module: Any) -> None:
    namespace = vars(module)
    for value in list(namespace.values()):
        globals_dict = getattr(value, "__globals__", None)
        if isinstance(globals_dict, dict):
            globals_dict.update(namespace)


def _inject_fragment_helpers_into_master() -> None:
    """Preserve legacy helper visibility after splitting master_core.

    In legacy_core.py, raw_depot_arcs/build_master_model could see fragment
    helpers because everything lived in one module. After the v1b split, these
    names live in fragment_core. This function explicitly exposes only the
    helper names needed by the mechanically extracted master layer.
    """
    for name in FRAGMENT_HELPERS_FOR_MASTER:
        if hasattr(frag, name):
            setattr(master, name, getattr(frag, name))


def _sync_all_globals() -> None:
    _sync_module_globals(frag)
    _inject_fragment_helpers_into_master()
    _sync_module_globals(master)
    _sync_module_globals(core)


def _get_var_value(var: Any) -> float:
    for attr in ("X", "x"):
        if hasattr(var, attr):
            return float(getattr(var, attr))
    return float(var)


def build_fragment_sets(instance: str | Path, max_base_path_len: int) -> dict[str, Any]:
    _ensure_generated()
    _sync_all_globals()
    data = frag.read_instance(Path(instance))
    frag.data = data
    master.data = data
    core.data = data
    _sync_all_globals()

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


def run_solver(instance: str | Path, max_base_path_len: int, k_max: int, force_exact_k: bool, use_callback: bool) -> ConsolidatedV1bRunSummary:
    sets = build_fragment_sets(instance, max_base_path_len)
    data = sets["data"]
    ef = sets["extended_undominated"]

    master.data = data
    master.e_frags_aug = ef
    _sync_all_globals()

    model_result = master.build_master_model(data, ef, k_max, force_exact_K=force_exact_k)
    if not isinstance(model_result, tuple) or len(model_result) < 8:
        raise RuntimeError("build_master_model did not return the expected 8-tuple.")

    m, x_vars, arcs, node_id, depot_u, arc_by_id, theta, big_m = model_result[:8]

    core.data = data
    core.e_frags_aug = ef
    core.arc_by_id = arc_by_id
    core.arcs_full = arcs
    core.node_id_tmp = node_id
    core.depot_u_tmp = depot_u
    core.theta_tmp = theta
    core.M_tmp = big_m
    core.x_tmp = x_vars
    _sync_module_globals(core)

    if use_callback:
        def _cb(model: Any, where: Any) -> None:
            core.callback(model, where, x_vars, arcs, node_id, depot_u, data, theta, big_m)
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

    return ConsolidatedV1bRunSummary(
        package="evrp_fragments_consolidated_v1b_master_split_patch2",
        fragment_core_source_sha256=getattr(frag, "FRAGMENT_CORE_SOURCE_SHA256", None),
        master_core_source_sha256=getattr(master, "MASTER_CORE_SOURCE_SHA256", None),
        legacy_core_source_sha256=getattr(core, "SOURCE_SHA256", None),
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


def summary_to_json(summary: ConsolidatedV1bRunSummary) -> str:
    return json.dumps(asdict(summary), indent=2, sort_keys=True)
