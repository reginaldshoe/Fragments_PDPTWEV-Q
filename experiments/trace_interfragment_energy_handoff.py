#!/usr/bin/env python3
"""Trace inter-fragment energy handoffs for a selected master route.

This diagnostic runner narrows the v4za SID-step failure into fragment-boundary
conditions. It does not modify solver, callback, master, fragment, or helper code.

It:
1. runs the existing solver;
2. reads the actual pipeline_v1e.cbcore.arc_by_id mapping populated during solve;
3. reconstructs selected route arc chains using solution_helpers;
4. replays each fragment sequence through fragment_core.step(...);
5. reports residual energy at each fragment end;
6. compares residual energy against the next fragment's Emin;
7. writes a JSON handoff report highlighting energy deficits.
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
    if dataclasses.is_dataclass(value) and not isinstance(value, type):
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
    if dataclasses.is_dataclass(summary) and not isinstance(summary, type):
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
    if any(p.kind == inspect.Parameter.VAR_KEYWORD for p in params.values()):
        return fn(**kwargs)
    return fn(**{k: v for k, v in kwargs.items() if k in params})


def _time_limit_kwarg_name(fn):
    params = inspect.signature(fn).parameters
    for name in ("time_limit_sec", "solver_time_limit_sec", "time_limit", "TimeLimit"):
        if name in params:
            return name
    return None


def _initial_state(data):
    return (
        (0,),             # path of node indices, starting at depot index 0
        0,                # pickup phase
        frozenset(),      # onboard pickup SIDs
        data["CapE"],     # remaining battery energy
        0.0,              # departure time
        frozenset(),      # seen pickups
        frozenset(),      # seen deliveries
        frozenset(),      # seen stations
        0,                # delivered count
        0.0,              # accumulated distance
    )


def _state_snapshot(data, state):
    path, phase, onboard, E, t_depart, seenP, seenD, seenS, deliv_count, distance = state
    cur_idx = path[-1]
    return {
        "path_indices": list(path),
        "path_sids": [data["nodes"][i][0] for i in path],
        "current_idx": cur_idx,
        "current_sid": data["nodes"][cur_idx][0],
        "phase": phase,
        "onboard": sorted(onboard),
        "E": E,
        "t_depart": t_depart,
        "seenP": sorted(seenP),
        "seenD": sorted(seenD),
        "seenS": sorted(seenS),
        "deliv_count": deliv_count,
        "distance": distance,
    }


def _step_sid(data, state, sid):
    from evrp_fragments.fragment_core import step

    prev_idx = state[0][-1]
    j = data["sid_to_i"][sid]
    before = _state_snapshot(data, state)
    travel_energy = data["energy"](prev_idx, j)
    travel_time = data["traveltime"](prev_idx, j)
    travel_distance = data["dist"](prev_idx, j)
    new_state, reason = step(data, state, j)
    return new_state, {
        "from_sid": before["current_sid"],
        "to_sid": sid,
        "from_idx": prev_idx,
        "to_idx": j,
        "energy_before": before["E"],
        "travel_energy": travel_energy,
        "travel_time": travel_time,
        "travel_distance": travel_distance,
        "energy_after_direct_travel_without_recharge": before["E"] - travel_energy,
        "step_ok": new_state is not None,
        "reason": reason,
        "state_before": before,
        "state_after": _state_snapshot(data, new_state) if new_state is not None else None,
    }


def _replay_fragment_from_current_state(data, state, fragment):
    """Replay one fragment's internal sequence from the current global state.

    The first SID in fragment['seq'] should match the current state's SID. If it does,
    only subsequent SIDs are stepped. If it does not, the mismatch is reported and the
    runner still attempts to step through the full sequence for diagnostic visibility.
    """
    seq = list(fragment["seq"])
    current_sid = data["nodes"][state[0][-1]][0]
    start_matches = bool(seq and seq[0] == current_sid)
    step_sids = seq[1:] if start_matches else seq
    trace = []
    st = state
    for sid in step_sids:
        st2, event = _step_sid(data, st, sid)
        trace.append(event)
        if st2 is None:
            return None, {
                "ok": False,
                "start_matches_current_sid": start_matches,
                "expected_start_sid": seq[0] if seq else None,
                "actual_current_sid": current_sid,
                "failed_sid": sid,
                "reason": event["reason"],
                "trace": trace,
            }
        st = st2
    return st, {
        "ok": True,
        "start_matches_current_sid": start_matches,
        "expected_start_sid": seq[0] if seq else None,
        "actual_current_sid": current_sid,
        "failed_sid": None,
        "reason": None,
        "trace": trace,
    }


def _handoff_record(data, *, route_pos, prev_aid, next_aid, prev_arc, next_arc, state_after_prev):
    residual = state_after_prev[3]
    next_emin = next_arc.get("Emin")
    try:
        deficit = float(next_emin) - float(residual)
        sufficient = deficit <= 1e-9
    except Exception:
        deficit = None
        sufficient = None
    return {
        "route_boundary_position": route_pos,
        "prev_arc_id": prev_aid,
        "next_arc_id": next_aid,
        "prev_end": prev_arc.get("End"),
        "next_start": next_arc.get("Start"),
        "boundary_matches": prev_arc.get("End") == next_arc.get("Start"),
        "residual_energy_after_prev_arc": residual,
        "next_fragment_Emin": next_emin,
        "energy_deficit_to_next_Emin": deficit,
        "sufficient_for_next_fragment_Emin": sufficient,
        "state_after_prev_arc": _state_snapshot(data, state_after_prev),
        "prev_arc": prev_arc,
        "next_arc": next_arc,
    }


def trace_interfragment_handoffs(data, route_arc_ids, arc_by_id):
    st = _initial_state(data)
    arc_reports = []
    handoffs = []
    ok = True
    failure = None

    for pos, aid in enumerate(route_arc_ids):
        arc = arc_by_id[aid]
        state_before_arc = st
        st_after, replay = _replay_fragment_from_current_state(data, st, arc)
        arc_report = {
            "route_position": pos,
            "arc_id": aid,
            "arc_start": arc.get("Start"),
            "arc_end": arc.get("End"),
            "arc_seq": arc.get("seq"),
            "arc_Emin": arc.get("Emin"),
            "state_before_arc": _state_snapshot(data, state_before_arc),
            "residual_energy_before_arc": state_before_arc[3],
            "energy_surplus_vs_arc_Emin_at_start": (state_before_arc[3] - arc.get("Emin")) if isinstance(arc.get("Emin"), (int, float)) else None,
            "replay": replay,
            "state_after_arc": _state_snapshot(data, st_after) if st_after is not None else None,
            "residual_energy_after_arc": st_after[3] if st_after is not None else None,
        }
        arc_reports.append(arc_report)
        if st_after is None:
            ok = False
            failure = {
                "type": "within_fragment_replay_failure",
                "route_position": pos,
                "arc_id": aid,
                "failed_sid": replay.get("failed_sid"),
                "reason": replay.get("reason"),
            }
            break

        if pos + 1 < len(route_arc_ids):
            next_aid = route_arc_ids[pos + 1]
            next_arc = arc_by_id[next_aid]
            handoffs.append(_handoff_record(data, route_pos=pos, prev_aid=aid, next_aid=next_aid, prev_arc=arc, next_arc=next_arc, state_after_prev=st_after))

        st = st_after

    problematic_handoffs = [
        h for h in handoffs
        if h.get("sufficient_for_next_fragment_Emin") is False or not h.get("boundary_matches")
    ]
    return {
        "ok": ok and not problematic_handoffs,
        "failure": failure,
        "arc_reports": arc_reports,
        "handoffs": handoffs,
        "problematic_handoffs": problematic_handoffs,
        "final_state": _state_snapshot(data, st) if ok else None,
    }


def main() -> int:
    parser = argparse.ArgumentParser(description="Trace inter-fragment energy handoffs for selected solver route")
    parser.add_argument("--instance", required=True)
    parser.add_argument("--max-base-path-len", type=int, required=True)
    parser.add_argument("--k-max", type=int, default=1)
    parser.add_argument("--use-callback", action="store_true")
    parser.add_argument("--time-limit-sec", type=float, default=300.0)
    parser.add_argument("--depot-sid", default="D0")
    parser.add_argument("--selected-arc-ids", default=None, help="Optional comma-separated selected arc ids; otherwise read from solver summary")
    parser.add_argument("--output", default="interfragment_energy_handoff_trace.json")
    args = parser.parse_args()

    from evrp_fragments import solution_helpers as helpers
    from evrp_fragments.fragment_core import read_instance
    from evrp_fragments.pipeline_v1e import run_solver
    import evrp_fragments.pipeline_v1e as pipeline_v1e

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

    if args.selected_arc_ids:
        selected_arc_ids = [int(x.strip()) for x in args.selected_arc_ids.split(",") if x.strip()]
    else:
        selected_arc_ids = summary.get("selected_arc_ids") or getattr(summary_obj, "selected_arc_ids", None) or []

    arc_by_id = getattr(getattr(pipeline_v1e, "cbcore", None), "arc_by_id", None)
    payload = {
        "instance": args.instance,
        "summary": summary,
        "selected_arc_ids": selected_arc_ids,
        "arc_by_id_found": isinstance(arc_by_id, dict),
        "arc_by_id_count": len(arc_by_id) if isinstance(arc_by_id, dict) else None,
        "routes": None,
        "route_handoff_traces": [],
        "error": None,
    }

    if not isinstance(arc_by_id, dict):
        payload["error"] = "pipeline_v1e.cbcore.arc_by_id was not available after run_solver"
        Path(args.output).write_text(json.dumps(_jsonable(payload), indent=2), encoding="utf-8")
        print(json.dumps(_jsonable(payload), indent=2))
        return 1

    missing = [aid for aid in selected_arc_ids if aid not in arc_by_id]
    if missing:
        payload["error"] = f"selected arc ids missing from arc_by_id: {missing}"
        Path(args.output).write_text(json.dumps(_jsonable(payload), indent=2), encoding="utf-8")
        print(json.dumps(_jsonable(payload), indent=2))
        return 1

    routes = helpers.extract_routes_from_solution(selected_arc_ids, arc_by_id, depot_sid=args.depot_sid)
    payload["routes"] = routes
    if not routes:
        payload["error"] = "selected arc ids were present but no depot-starting route was extracted"
        Path(args.output).write_text(json.dumps(_jsonable(payload), indent=2), encoding="utf-8")
        print(json.dumps(_jsonable(payload), indent=2))
        return 1

    for route in routes:
        trace = trace_interfragment_handoffs(data, route, arc_by_id)
        payload["route_handoff_traces"].append({
            "route_arc_ids": route,
            "trace": trace,
        })

    Path(args.output).write_text(json.dumps(_jsonable(payload), indent=2), encoding="utf-8")
    print(json.dumps(_jsonable({
        "selected_arc_ids": selected_arc_ids,
        "routes": routes,
        "route_handoff_summaries": [
            {
                "route_arc_ids": r["route_arc_ids"],
                "ok": r["trace"]["ok"],
                "failure": r["trace"]["failure"],
                "problematic_handoffs": [
                    {
                        "prev_arc_id": h["prev_arc_id"],
                        "next_arc_id": h["next_arc_id"],
                        "prev_end": h["prev_end"],
                        "next_start": h["next_start"],
                        "residual_energy_after_prev_arc": h["residual_energy_after_prev_arc"],
                        "next_fragment_Emin": h["next_fragment_Emin"],
                        "energy_deficit_to_next_Emin": h["energy_deficit_to_next_Emin"],
                        "sufficient_for_next_fragment_Emin": h["sufficient_for_next_fragment_Emin"],
                    }
                    for h in r["trace"]["problematic_handoffs"]
                ],
            }
            for r in payload["route_handoff_traces"]
        ],
    }), indent=2))
    print(f"[INTERFRAGMENT-HANDOFF-TRACE-WRITTEN] path={args.output}", flush=True)
    return 0 if all(r["trace"]["ok"] for r in payload["route_handoff_traces"]) else 1


if __name__ == "__main__":
    raise SystemExit(main())
