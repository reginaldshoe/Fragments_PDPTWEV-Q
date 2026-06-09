#!/usr/bin/env python3
"""Expose master arc_by_id mapping and validate selected routes.

v4z9 hardens the v4z8 introspection runner. The previous runner treated dataclass
classes as dataclass instances in one recursive branch, which caused:

    TypeError: asdict() should be called on dataclass instances

This version only calls dataclasses.asdict(...) for dataclass instances, not dataclass
classes, and skips class/module objects during recursive value inspection unless they
are explicitly handled module roots.
"""
from __future__ import annotations

import argparse
import dataclasses
import inspect
import json
import types
from pathlib import Path
from typing import Any

ARC_VALUE_REQUIRED_KEYS = {"Start", "End", "seq"}
ARC_MAP_NAMES = {"arc_by_id", "arcs_by_id", "arc_map", "id_to_arc", "arc_id_to_fragment"}


def _is_dataclass_instance(value: Any) -> bool:
    return dataclasses.is_dataclass(value) and not isinstance(value, type)


def _jsonable(value: Any):
    if isinstance(value, (str, int, float, bool)) or value is None:
        return value
    if isinstance(value, dict):
        return {str(k): _jsonable(v) for k, v in value.items()}
    if _is_dataclass_instance(value):
        return _jsonable(dataclasses.asdict(value))
    if isinstance(value, (list, tuple)):
        return [_jsonable(v) for v in value]
    if isinstance(value, (set, frozenset)):
        return sorted(_jsonable(v) for v in value)
    if hasattr(value, "__dict__") and not isinstance(value, (types.ModuleType, type)):
        return {str(k): _jsonable(v) for k, v in vars(value).items() if not k.startswith("_")}
    return repr(value)


def _summary_to_dict(summary: Any) -> dict[str, Any]:
    if isinstance(summary, dict):
        return dict(summary)
    if _is_dataclass_instance(summary):
        return dataclasses.asdict(summary)
    if hasattr(summary, "_asdict"):
        return dict(summary._asdict())
    if hasattr(summary, "__dict__"):
        return {k: v for k, v in vars(summary).items() if not k.startswith("_")}
    out = {}
    for name in dir(summary):
        if name.startswith("_"):
            continue
        try:
            value = getattr(summary, name)
        except Exception:
            continue
        if not callable(value):
            out[name] = value
    return out


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


def _is_arc_record(value: Any) -> bool:
    return isinstance(value, dict) and ARC_VALUE_REQUIRED_KEYS.issubset(value.keys())


def _is_arc_by_id(value: Any) -> bool:
    if not isinstance(value, dict) or not value:
        return False
    sample = list(value.items())[:20]
    intish_keys = 0
    arcish_values = 0
    for k, v in sample:
        try:
            int(k)
            intish_keys += 1
        except Exception:
            pass
        if _is_arc_record(v):
            arcish_values += 1
    return intish_keys > 0 and arcish_values > 0


def _iter_object_items(obj: Any):
    if isinstance(obj, dict):
        yield from obj.items()
    elif _is_dataclass_instance(obj):
        yield from dataclasses.asdict(obj).items()
    elif isinstance(obj, types.ModuleType):
        # Only inspect public module variables shallowly; avoid descending into imported classes.
        yield from ((k, v) for k, v in vars(obj).items() if not k.startswith("_"))
    elif hasattr(obj, "__dict__") and not isinstance(obj, type):
        yield from ((k, v) for k, v in vars(obj).items() if not k.startswith("_"))


def _should_descend(value: Any) -> bool:
    if isinstance(value, type):
        return False
    if isinstance(value, (str, bytes, int, float, bool)) or value is None:
        return False
    if inspect.isfunction(value) or inspect.ismethod(value) or inspect.isbuiltin(value):
        return False
    return isinstance(value, (dict, list, tuple)) or _is_dataclass_instance(value) or (hasattr(value, "__dict__") and not isinstance(value, type))


def _find_arc_maps(obj: Any, *, label: str, max_depth: int = 4):
    found = []
    seen = set()

    def visit(x: Any, path: str, depth: int):
        if depth < 0:
            return
        oid = id(x)
        if oid in seen:
            return
        seen.add(oid)

        if _is_arc_by_id(x):
            found.append((path, x))
            return

        if isinstance(x, dict):
            iterable = list(x.items())[:300]
        elif isinstance(x, (list, tuple)):
            iterable = [(f"[{i}]", v) for i, v in enumerate(x[:150])]
        else:
            iterable = list(_iter_object_items(x) or [])[:300]

        for k, v in iterable:
            key = str(k)
            child_path = f"{path}{key}" if key.startswith("[") else f"{path}.{key}"
            if key in ARC_MAP_NAMES and _is_arc_by_id(v):
                found.append((child_path, v))
            elif _should_descend(v):
                visit(v, child_path, depth - 1)

    visit(obj, label, max_depth)
    return found


def _try_master_build_functions(master_core, *, data, fragment_sets, instance, max_base_path_len, k_max, use_callback):
    outputs = []
    for name in dir(master_core):
        if name.startswith("_"):
            continue
        obj = getattr(master_core, name)
        if not callable(obj):
            continue
        lname = name.lower()
        if not any(token in lname for token in ("build", "create", "make", "construct")):
            continue
        if any(token in lname for token in ("solve", "callback", "route_dp")):
            continue
        try:
            sig = inspect.signature(obj)
        except Exception:
            continue
        kwargs = {
            "data": data,
            "fragment_sets": fragment_sets,
            "sets": fragment_sets,
            "instance": instance,
            "max_base_path_len": max_base_path_len,
            "k_max": k_max,
            "use_callback": use_callback,
            "force_exact_k": False,
        }
        required = [
            p for p in sig.parameters.values()
            if p.default is inspect._empty
            and p.kind in (inspect.Parameter.POSITIONAL_OR_KEYWORD, inspect.Parameter.KEYWORD_ONLY)
        ]
        if any(p.name not in kwargs for p in required):
            continue
        try:
            result = _call_with_supported_kwargs(obj, **kwargs)
        except Exception as exc:
            outputs.append({"function": name, "error": f"{type(exc).__name__}: {exc}"})
            continue
        outputs.append({"function": name, "result": result})
    return outputs


def _fallback_fragment_list_maps(fragment_sets: Any):
    candidates = []
    items = fragment_sets.items() if isinstance(fragment_sets, dict) else vars(fragment_sets).items() if hasattr(fragment_sets, "__dict__") else []
    for name, value in items:
        if not isinstance(value, list) or not value:
            continue
        if not all(_is_arc_record(v) for v in value[: min(10, len(value))]):
            continue
        candidates.append((f"fragment_sets.{name}:zero_based_index", {i: v for i, v in enumerate(value)}))
        candidates.append((f"fragment_sets.{name}:one_based_index", {i + 1: v for i, v in enumerate(value)}))
    return candidates


def _validate_arc_map(name, arc_by_id, *, selected_arc_ids, data, helpers, depot_sid):
    missing = [aid for aid in selected_arc_ids if aid not in arc_by_id]
    out = {
        "arc_map_name": name,
        "arc_count": len(arc_by_id),
        "missing_selected_arc_ids": missing,
        "selected_arc_records": {str(aid): arc_by_id.get(aid) for aid in selected_arc_ids if aid in arc_by_id},
        "routes": None,
        "stitched_sid_sequences": None,
        "route_distances": None,
        "simulation_results": None,
        "error": None,
    }
    if missing:
        out["error"] = "selected ids missing from arc map"
        return out
    try:
        routes = helpers.extract_routes_from_solution(selected_arc_ids, arc_by_id, depot_sid=depot_sid)
    except Exception as exc:
        out["error"] = f"extract_routes_from_solution failed: {type(exc).__name__}: {exc}"
        return out
    if not routes:
        out["routes"] = []
        out["error"] = "selected ids present but no depot-starting route was extracted"
        return out
    stitched = []
    distances = []
    sims = []
    for route in routes:
        try:
            sid_seq = helpers.stitch_sid_sequence(route, arc_by_id)
            stitched.append(sid_seq)
            distances.append(helpers.route_distance_from_sids(data, sid_seq))
            ok, failed_sid, reason = helpers.simulate_route(data, sid_seq)
            sims.append({"ok": ok, "failed_sid": failed_sid, "reason": reason})
        except Exception as exc:
            sims.append({"ok": False, "failed_sid": None, "reason": f"{type(exc).__name__}: {exc}"})
    out["routes"] = routes
    out["stitched_sid_sequences"] = stitched
    out["route_distances"] = distances
    out["simulation_results"] = sims
    return out


def main() -> int:
    parser = argparse.ArgumentParser(description="Expose master arc_by_id and validate selected route")
    parser.add_argument("--instance", required=True)
    parser.add_argument("--max-base-path-len", type=int, required=True)
    parser.add_argument("--k-max", type=int, default=1)
    parser.add_argument("--use-callback", action="store_true")
    parser.add_argument("--time-limit-sec", type=float, default=300.0)
    parser.add_argument("--depot-sid", default="D0")
    parser.add_argument("--output", default="master_arc_by_id_route_validation.json")
    args = parser.parse_args()

    from evrp_fragments import solution_helpers as helpers
    from evrp_fragments.fragment_core import read_instance
    from evrp_fragments.pipeline_v1e import build_fragment_sets, run_solver
    import evrp_fragments.master_core as master_core
    import evrp_fragments.pipeline_v1e as pipeline_v1e

    data = read_instance(args.instance)
    fragment_sets = build_fragment_sets(args.instance, args.max_base_path_len)

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
    selected_arc_ids = summary.get("selected_arc_ids") or getattr(summary_obj, "selected_arc_ids", None) or []

    discovered = []
    scan_errors = []
    for label, obj in [
        ("fragment_sets", fragment_sets),
        ("summary", summary_obj),
        ("pipeline_v1e_module", pipeline_v1e),
        ("master_core_module", master_core),
    ]:
        try:
            discovered.extend(_find_arc_maps(obj, label=label))
        except Exception as exc:
            scan_errors.append({"label": label, "error": f"{type(exc).__name__}: {exc}"})

    build_outputs = _try_master_build_functions(
        master_core,
        data=data,
        fragment_sets=fragment_sets,
        instance=args.instance,
        max_base_path_len=args.max_base_path_len,
        k_max=args.k_max,
        use_callback=args.use_callback,
    )
    for item in build_outputs:
        if "result" in item:
            try:
                discovered.extend(_find_arc_maps(item["result"], label=f"master_core.{item['function']}()"))
            except Exception as exc:
                scan_errors.append({"label": f"master_core.{item['function']}()", "error": f"{type(exc).__name__}: {exc}"})

    if not discovered:
        discovered.extend(_fallback_fragment_list_maps(fragment_sets))

    seen_ids = set()
    unique = []
    for path, arc_map in discovered:
        oid = id(arc_map)
        if oid in seen_ids:
            continue
        seen_ids.add(oid)
        unique.append((path, arc_map))

    validation_results = [
        _validate_arc_map(path, arc_map, selected_arc_ids=selected_arc_ids, data=data, helpers=helpers, depot_sid=args.depot_sid)
        for path, arc_map in unique
    ]
    successful = [
        r for r in validation_results
        if r.get("error") is None and r.get("simulation_results") and all(x.get("ok") for x in r["simulation_results"])
    ]
    extractable = [r for r in validation_results if r.get("error") is None]

    payload = {
        "instance": args.instance,
        "run_solver_return_type": type(summary_obj).__name__,
        "run_solver_supported_parameters": list(inspect.signature(run_solver).parameters),
        "time_limit_parameter_used": tl_name,
        "summary": summary,
        "selected_arc_ids": selected_arc_ids,
        "selected_arc_count": len(selected_arc_ids),
        "scan_errors": scan_errors,
        "master_build_probe_results": [
            {"function": x.get("function"), "error": x.get("error"), "result_type": type(x.get("result")).__name__ if "result" in x else None}
            for x in build_outputs
        ],
        "arc_map_candidates": [
            {"name": name, "arc_count": len(arc_map), "sample_keys": list(arc_map.keys())[:10]}
            for name, arc_map in unique
        ],
        "validation_results": validation_results,
        "best_result": successful[0] if successful else extractable[0] if extractable else {"error": "no usable arc_by_id mapping found"},
    }

    Path(args.output).write_text(json.dumps(_jsonable(payload), indent=2), encoding="utf-8")
    print(json.dumps(_jsonable({
        "selected_arc_ids": selected_arc_ids,
        "scan_errors": scan_errors,
        "arc_map_candidates": payload["arc_map_candidates"],
        "best_result": payload["best_result"],
    }), indent=2))
    print(f"[MASTER-ARC-BY-ID-VALIDATION-WRITTEN] path={args.output}", flush=True)
    return 0 if successful else 1


if __name__ == "__main__":
    raise SystemExit(main())
