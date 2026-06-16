"""Inspect callback-facing queueing evaluator from a solved master incumbent.

Environment:
- Python 3.12
- gurobipy available in the target environment

Role:
- Diagnostic runner only.
- Builds fragment sets and the master model.
- Solves without callback to obtain a selected incumbent.
- Calls queueing_core.evaluate_queueing_for_selected_arcs(...).
- Writes the compact callback-facing queueing result.
"""
from __future__ import annotations

import argparse
import json
from pathlib import Path
from typing import Any

from evrp_fragments import fragment_core as frag
from evrp_fragments import master_core as master
from evrp_fragments import queueing_core as qcore


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Inspect callback-facing queueing evaluator.")
    parser.add_argument("--instance", required=True)
    parser.add_argument("--max-base-path-len", type=int, required=True)
    parser.add_argument("--k-max", type=int, required=True)
    parser.add_argument("--force-exact-k", action="store_true")
    parser.add_argument("--time-limit-sec", type=float, default=None)
    parser.add_argument("--out", default="queueing_callback_evaluation.json")
    return parser.parse_args()


def _get_var_value(var: Any) -> float:
    for attr in ("X", "x"):
        if hasattr(var, attr):
            return float(getattr(var, attr))
    return float(var)


def _build_fragment_sets(instance: str, max_base_path_len: int) -> dict[str, Any]:
    data = frag.read_instance(instance)
    base_paths, base_path_metadata = frag.enumerate_base_paths(data, max_base_path_len)
    restricted_raw = frag.enumerate_fragments(data, base_paths)
    restricted_dedup = frag.dedup_exact(restricted_raw)
    restricted_meta = frag.attach_metadata(data, restricted_dedup)
    restricted_undominated = frag.dominance_filter(restricted_meta)
    extended_raw = frag.extend_all_fragments(data, restricted_undominated)
    extended_dedup = frag.dedup_exact(extended_raw)
    extended_meta = frag.attach_metadata(data, extended_dedup, exclude_last_ef=True)
    extended_undominated = frag.dominance_filter(extended_meta)
    return {
        "data": data,
        "base_paths": base_paths,
        "restricted_raw": restricted_raw,
        "restricted_undominated": restricted_undominated,
        "extended_raw": extended_raw,
        "extended_undominated": extended_undominated,
    }


def main() -> None:
    args = parse_args()
    sets = _build_fragment_sets(args.instance, args.max_base_path_len)
    data = sets["data"]

    model, x_vars, arcs, node_id, depot_u, arc_by_id, theta, big_m = master.build_master_model(
        data,
        sets["extended_undominated"],
        args.k_max,
        force_exact_K=args.force_exact_k,
    )

    if args.time_limit_sec is not None:
        if args.time_limit_sec <= 0:
            raise ValueError("--time-limit-sec must be positive when supplied")
        model.Params.TimeLimit = float(args.time_limit_sec)

    model.optimize()

    selected_arc_ids = sorted(
        aid for aid, var in x_vars.items()
        if _get_var_value(var) > 0.5
    )

    evaluation = qcore.evaluate_queueing_for_selected_arcs(
        data=data,
        arcs=arcs,
        depot_u=depot_u,
        selected_arc_ids=selected_arc_ids,
    )
    evaluation_payload = qcore.queueing_callback_evaluation_to_jsonable(evaluation)

    output = {
        "summary": {
            "instance": args.instance,
            "status": int(getattr(model, "Status", getattr(model, "status", 0))),
            "selected_arc_count": len(selected_arc_ids),
            "queueing_ok": evaluation.ok,
            "queueing_reason": evaluation.reason,
            "queueing_total_delay": evaluation.total_delay,
            "queueing_total_service_delay": evaluation.total_service_delay,
            "queueing_resource_wait_time": getattr(evaluation, "resource_wait_time", 0.0),
            "queueing_resource_wait_event_count": getattr(evaluation, "resource_wait_event_count", 0),
            "queueing_resolved_overlap_count": getattr(evaluation, "resolved_overlap_count", 0),
            "queueing_delayed_activity_count": evaluation.delayed_activity_count,
            "queueing_delayed_service_count": evaluation.delayed_service_count,
            "queueing_conflict_count": evaluation.conflict_count,
            "queueing_infeasible_arc_ids": list(evaluation.infeasible_arc_ids),
            "queueing_resolved_delay_arc_ids": list(evaluation.resolved_delay_arc_ids),
            "base_paths": len(sets["base_paths"]),
            "restricted_raw": len(sets["restricted_raw"]),
            "restricted_undominated": len(sets["restricted_undominated"]),
            "extended_raw": len(sets["extended_raw"]),
            "extended_undominated": len(sets["extended_undominated"]),
        },
        "selected_arc_ids": selected_arc_ids,
        "queueing_callback_evaluation": evaluation_payload,
    }

    Path(args.out).write_text(json.dumps(output, indent=2), encoding="utf-8")
    print(json.dumps(output["summary"], indent=2))


if __name__ == "__main__":
    main()
