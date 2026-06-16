#!/usr/bin/env python3
"""Validate selected solver arcs using evrp_fragments.solution_helpers.

v4z7 fixes the runner so it handles pipeline.run_solver returning either a dict
or a dataclass / object-style run summary such as SolverRunSummary.
"""
from __future__ import annotations

import argparse
import dataclasses
import inspect
import json
from pathlib import Path
from typing import Any


def _jsonable(value: Any):
    if isinstance(value, (str, int, float, bool)) or value is None:
        return value
    if isinstance(value, dict):
        return {str(k): _jsonable(v) for k, v in value.items()}
    if dataclasses.is_dataclass(value):
        return _jsonable(dataclasses.asdict(value))
    if isinstance(value, (list, tuple)):
        return [_jsonable(v) for v in value]
    if isinstance(value, (set, frozenset)):
        return sorted(_jsonable(v) for v in value)
    if hasattr(value, "__dict__"):
        return {str(k): _jsonable(v) for k, v in vars(value).items() if not k.startswith("_")}
    return repr(value)


def _summary_to_dict(summary: Any) -> dict[str, Any]:
    if isinstance(summary, dict):
        return dict(summary)
    if dataclasses.is_dataclass(summary):
        return dataclasses.asdict(summary)
    if hasattr(summary, "_asdict"):
        return dict(summary._asdict())
    if hasattr(summary, "__dict__"):
        return {k: v for k, v in vars(summary).items() if not k.startswith("_")}
    # Last-resort extraction for object summaries with properties but no __dict__.
    out = {}
    for name in dir(summary):
        if name.startswith("_"):
            continue
        try:
            value = getattr(summary, name)
        except Exception:
            continue
        if callable(value):
            continue
        out[name] = value
    return out


def _get_summary_value(summary: Any, key: str, default=None):
    if isinstance(summary, dict):
        return summary.get(key, default)
    if hasattr(summary, key):
        return getattr(summary, key)
    return _summary_to_dict(summary).get(key, default)


def _call_with_supported_kwargs(fn, **kwargs):
    sig = inspect.signature(fn)
    params = sig.parameters
    accepts_var_kw = any(p.kind == inspect.Parameter.VAR_KEYWORD for p in params.values())
    if accepts_var_kw:
        return fn(**kwargs)
    return fn(**{k: v for k, v in kwargs.items() if k in params})


def _time_limit_kwarg_name(fn):
    params = inspect.signature(fn).parameters
    for name in ("time_limit_sec", "solver_time_limit_sec", "time_limit", "TimeLimit"):
        if name in params:
            return name
    return None


def _iter_named_lists(obj: Any):
    if isinstance(obj, dict):
        items = obj.items()
    elif dataclasses.is_dataclass(obj):
        items = dataclasses.asdict(obj).items()
    elif hasattr(obj, "__dict__"):
        items = vars(obj).items()
    else:
        items = []
    for name, value in items:
        if isinstance(value, list):
            yield name, value


def _has_arc_fields(frag: Any) -> bool:
    return isinstance(frag, dict) and "Start" in frag and "End" in frag and "seq" in frag


def _explicit_arc_id(frag: dict[str, Any]):
    for key in ("id", "arc_id", "ArcID", "arcId", "fragment_id", "fid"):
        if key in frag:
            try:
                return int(frag[key])
            except Exception:
                return frag[key]
    return None


def _candidate_arc_maps(fragment_sets: Any):
    for name, values in _iter_named_lists(fragment_sets):
        if not values or not all(_has_arc_fields(v) for v in values[: min(10, len(values))]):
            continue

        explicit = {}
        has_explicit = False
        for idx, frag in enumerate(values):
            aid = _explicit_arc_id(frag)
            if aid is not None:
                explicit[aid] = frag
                has_explicit = True
        if has_explicit:
            yield f"{name}:explicit_id", explicit

        yield f"{name}:zero_based_index", {idx: frag for idx, frag in enumerate(values)}
        yield f"{name}:one_based_index", {idx + 1: frag for idx, frag in enumerate(values)}


def _evaluate_candidate(data, chosen_arc_ids, arc_map_name, arc_by_id, depot_sid, helpers):
    missing = [aid for aid in chosen_arc_ids if aid not in arc_by_id]
    result = {
        "arc_map_name": arc_map_name,
        "arc_count": len(arc_by_id),
        "missing_selected_arc_ids": missing,
        "routes": None,
        "route_count": None,
        "stitched_sid_sequences": None,
        "route_distances": None,
        "simulation_results": None,
        "error": None,
    }
    if missing:
        result["error"] = "selected ids missing from this arc map"
        return result

    try:
        routes = helpers.extract_routes_from_solution(chosen_arc_ids, arc_by_id, depot_sid=depot_sid)
    except Exception as exc:
        result["error"] = f"extract_routes_from_solution failed: {type(exc).__name__}: {exc}"
        return result

    stitched = []
    distances = []
    simulations = []
    for route in routes:
        try:
            sid_seq = helpers.stitch_sid_sequence(route, arc_by_id)
            stitched.append(sid_seq)
            distances.append(helpers.route_distance_from_sids(data, sid_seq))
            ok, failed_sid, reason = helpers.simulate_route(data, sid_seq)
            simulations.append({"ok": ok, "failed_sid": failed_sid, "reason": reason})
        except Exception as exc:
            simulations.append({"ok": False, "failed_sid": None, "reason": f"{type(exc).__name__}: {exc}"})

    result.update({
        "routes": routes,
        "route_count": len(routes),
        "stitched_sid_sequences": stitched,
        "route_distances": distances,
        "simulation_results": simulations,
        "error": None,
    })
    return result


def main() -> int:
    parser = argparse.ArgumentParser(description="Validate selected solver route using solution_helpers.py")
    parser.add_argument("--instance", required=True)
    parser.add_argument("--max-base-path-len", type=int, required=True)
    parser.add_argument("--k-max", type=int, default=1)
    parser.add_argument("--time-limit-sec", type=float, default=300.0)
    parser.add_argument("--use-callback", action="store_true")
    parser.add_argument("--depot-sid", default="D0")
    parser.add_argument("--output", default="selected_route_validation.json")
    args = parser.parse_args()

    from evrp_fragments.pipeline import build_fragment_sets, run_solver
    from evrp_fragments.fragment_core import read_instance
    from evrp_fragments import solution_helpers as helpers

    data = read_instance(args.instance)
    solver_kwargs = {
        "instance": args.instance,
        "max_base_path_len": args.max_base_path_len,
        "k_max": args.k_max,
        "force_exact_k": False,
        "use_callback": args.use_callback,
    }
    tl_name = _time_limit_kwarg_name(run_solver)
    if tl_name is not None:
        solver_kwargs[tl_name] = args.time_limit_sec

    summary_obj = _call_with_supported_kwargs(run_solver, **solver_kwargs)
    summary = _summary_to_dict(summary_obj)
    chosen_arc_ids = _get_summary_value(summary_obj, "selected_arc_ids", None) or summary.get("selected_arc_ids") or []

    payload = {
        "instance": args.instance,
        "run_solver_return_type": type(summary_obj).__name__,
        "run_solver_supported_parameters": list(inspect.signature(run_solver).parameters),
        "time_limit_parameter_used": tl_name,
        "summary": summary,
        "selected_arc_ids": chosen_arc_ids,
        "selected_arc_count": len(chosen_arc_ids),
        "candidate_results": [],
        "best_result": None,
    }

    if not chosen_arc_ids:
        payload["best_result"] = {"error": "solver summary contains no selected_arc_ids"}
        Path(args.output).write_text(json.dumps(_jsonable(payload), indent=2), encoding="utf-8")
        print(json.dumps(_jsonable(payload), indent=2))
        print(f"[ROUTE-VALIDATION-WRITTEN] path={args.output}", flush=True)
        return 1

    fragment_sets = build_fragment_sets(args.instance, args.max_base_path_len)
    results = []
    for name, arc_by_id in _candidate_arc_maps(fragment_sets):
        results.append(_evaluate_candidate(data, chosen_arc_ids, name, arc_by_id, args.depot_sid, helpers))

    payload["candidate_results"] = results
    successful = [
        r for r in results
        if r.get("error") is None and r.get("simulation_results") and all(x.get("ok") for x in r["simulation_results"])
    ]
    plausible = [r for r in results if r.get("error") is None]
    if successful:
        payload["best_result"] = successful[0]
    elif plausible:
        payload["best_result"] = plausible[0]
    else:
        payload["best_result"] = {"error": "no candidate arc map produced extractable routes"}

    Path(args.output).write_text(json.dumps(_jsonable(payload), indent=2), encoding="utf-8")
    print(json.dumps(_jsonable({"summary": summary, "best_result": payload["best_result"]}), indent=2))
    print(f"[ROUTE-VALIDATION-WRITTEN] path={args.output}", flush=True)
    return 0 if successful else 1


if __name__ == "__main__":
    raise SystemExit(main())
