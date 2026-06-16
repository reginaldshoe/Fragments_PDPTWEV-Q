"""Synthetic Phase B tests for queueing no-overlap scheduling.

Environment:
- Python 3.12
- No Gurobi/model build required

Role:
- Unit-style diagnostic runner for Phase B queueing behaviour.
- Directly constructs QueueActivityExtraction objects.
- Exercises the delay branch and infeasible-overlap branch without fragment generation.
"""
from __future__ import annotations

import argparse
import json
from pathlib import Path
from typing import Any

from evrp_fragments import queueing_core as qcore


RESOURCE = "customer_location:42,10"


def _service_activity(
    *,
    activity_id: int,
    route_id: int,
    route_pos: int,
    arc_id: int,
    node_sid: str,
    base_start: float,
    duration: float,
    latest_start: float,
    resource_id: str = RESOURCE,
) -> qcore.QueueActivity:
    return qcore.QueueActivity(
        activity_id=activity_id,
        route_id=route_id,
        route_pos=route_pos,
        arc_id=arc_id,
        arc_pos=0,
        kind="service",
        sid_from=None,
        sid_to=None,
        node_sid=node_sid,
        resource_type="customer",
        resource_id=resource_id,
        duration=float(duration),
        earliest_start=0.0,
        latest_start=float(latest_start),
        base_start=float(base_start),
        base_end=float(base_start) + float(duration),
        slide=float(latest_start) - float(base_start),
        supporting_arc_ids=(int(arc_id),),
    )


def _movement_activity(
    *,
    activity_id: int,
    route_id: int,
    route_pos: int,
    arc_id: int,
    sid_from: str,
    sid_to: str,
    base_start: float,
    duration: float,
) -> qcore.QueueActivity:
    return qcore.QueueActivity(
        activity_id=activity_id,
        route_id=route_id,
        route_pos=route_pos,
        arc_id=arc_id,
        arc_pos=0,
        kind="movement",
        sid_from=sid_from,
        sid_to=sid_to,
        node_sid=None,
        resource_type="route",
        resource_id=f"route:{route_id}",
        duration=float(duration),
        earliest_start=float(base_start),
        latest_start=None,
        base_start=float(base_start),
        base_end=float(base_start) + float(duration),
        slide=None,
        supporting_arc_ids=(int(arc_id),),
    )


def make_resolvable_overlap_case() -> qcore.QueueActivityExtraction:
    """Two routes overlap at one customer resource but the second can slide.

    Activity A: service [100, 190]
    Activity B: service [150, 240], latest start 250

    Expected Phase B behaviour:
    - delay B by 40 to [190, 280]
    - result remains feasible
    """

    routes = (
        qcore.QueueRoute(route_id=0, arc_ids=(10,), sid_sequence=("D0", "C_A", "D0")),
        qcore.QueueRoute(route_id=1, arc_ids=(20,), sid_sequence=("D0", "C_B", "D0")),
    )
    activities = (
        _service_activity(
            activity_id=0,
            route_id=0,
            route_pos=0,
            arc_id=10,
            node_sid="C_A",
            base_start=100.0,
            duration=90.0,
            latest_start=300.0,
        ),
        _service_activity(
            activity_id=1,
            route_id=1,
            route_pos=0,
            arc_id=20,
            node_sid="C_B",
            base_start=150.0,
            duration=90.0,
            latest_start=250.0,
        ),
    )
    return qcore.QueueActivityExtraction(routes=routes, activities=activities, ignored_station_sids=())


def make_infeasible_overlap_case() -> qcore.QueueActivityExtraction:
    """Two routes overlap at one customer resource and the second cannot slide enough.

    Activity A: service [100, 190]
    Activity B: service [150, 240], latest start 180

    Expected Phase B behaviour:
    - delaying B to 190 would breach latest start 180
    - result is infeasible and reports arc 20
    """

    routes = (
        qcore.QueueRoute(route_id=0, arc_ids=(10,), sid_sequence=("D0", "C_A", "D0")),
        qcore.QueueRoute(route_id=1, arc_ids=(20,), sid_sequence=("D0", "C_B", "D0")),
    )
    activities = (
        _service_activity(
            activity_id=0,
            route_id=0,
            route_pos=0,
            arc_id=10,
            node_sid="C_A",
            base_start=100.0,
            duration=90.0,
            latest_start=300.0,
        ),
        _service_activity(
            activity_id=1,
            route_id=1,
            route_pos=0,
            arc_id=20,
            node_sid="C_B",
            base_start=150.0,
            duration=90.0,
            latest_start=180.0,
        ),
    )
    return qcore.QueueActivityExtraction(routes=routes, activities=activities, ignored_station_sids=())


def make_suffix_propagation_case() -> qcore.QueueActivityExtraction:
    """Overlap delay propagates to a later movement/service suffix on same route.

    Route 0:
      A at shared resource [100, 190]

    Route 1:
      B at shared resource [150, 240], can delay by 40
      movement [240, 250]
      C at separate resource [250, 260], latest start 310

    Expected Phase B behaviour:
    - delay B and all later route-1 activities by 40
    - B scheduled at 190
    - movement scheduled at 280
    - C scheduled at 290
    - result feasible
    """

    routes = (
        qcore.QueueRoute(route_id=0, arc_ids=(10,), sid_sequence=("D0", "C_A", "D0")),
        qcore.QueueRoute(route_id=1, arc_ids=(20, 21), sid_sequence=("D0", "C_B", "C_C", "D0")),
    )
    activities = (
        _service_activity(
            activity_id=0,
            route_id=0,
            route_pos=0,
            arc_id=10,
            node_sid="C_A",
            base_start=100.0,
            duration=90.0,
            latest_start=300.0,
        ),
        _service_activity(
            activity_id=1,
            route_id=1,
            route_pos=0,
            arc_id=20,
            node_sid="C_B",
            base_start=150.0,
            duration=90.0,
            latest_start=250.0,
        ),
        _movement_activity(
            activity_id=2,
            route_id=1,
            route_pos=1,
            arc_id=20,
            sid_from="C_B",
            sid_to="C_C",
            base_start=240.0,
            duration=10.0,
        ),
        _service_activity(
            activity_id=3,
            route_id=1,
            route_pos=2,
            arc_id=21,
            node_sid="C_C",
            base_start=250.0,
            duration=10.0,
            latest_start=310.0,
            resource_id="customer_location:99,99",
        ),
    )
    return qcore.QueueActivityExtraction(routes=routes, activities=activities, ignored_station_sids=())


def _scheduled_by_id(result: qcore.QueueingScheduleResult) -> dict[int, qcore.ScheduledQueueActivity]:
    return {activity.activity_id: activity for activity in result.scheduled_activities}


def run_cases() -> dict[str, Any]:
    cases = {
        "resolvable_overlap": make_resolvable_overlap_case(),
        "infeasible_overlap": make_infeasible_overlap_case(),
        "suffix_propagation": make_suffix_propagation_case(),
    }
    output: dict[str, Any] = {}

    # Case 1: resolvable overlap.
    extraction = cases["resolvable_overlap"]
    result = qcore.check_queueing_no_overlap(extraction)
    sched = _scheduled_by_id(result)
    assert result.ok, "resolvable_overlap should be feasible"
    assert abs(result.total_service_delay - 40.0) <= 1e-7, result.total_service_delay
    assert abs(sched[1].scheduled_start - 190.0) <= 1e-7, sched[1]
    output["resolvable_overlap"] = {
        "extraction": qcore.activities_to_jsonable(extraction),
        "schedule": qcore.schedule_result_to_jsonable(result),
    }

    # Case 2: infeasible overlap.
    extraction = cases["infeasible_overlap"]
    result = qcore.check_queueing_no_overlap(extraction)
    assert not result.ok, "infeasible_overlap should be infeasible"
    assert result.conflicts, "infeasible_overlap should report at least one conflict"
    assert 20 in result.infeasible_arc_ids, result.infeasible_arc_ids
    output["infeasible_overlap"] = {
        "extraction": qcore.activities_to_jsonable(extraction),
        "schedule": qcore.schedule_result_to_jsonable(result),
    }

    # Case 3: suffix propagation.
    extraction = cases["suffix_propagation"]
    result = qcore.check_queueing_no_overlap(extraction)
    sched = _scheduled_by_id(result)
    assert result.ok, "suffix_propagation should be feasible"
    assert abs(sched[1].scheduled_start - 190.0) <= 1e-7, sched[1]
    assert abs(sched[2].scheduled_start - 280.0) <= 1e-7, sched[2]
    assert abs(sched[3].scheduled_start - 290.0) <= 1e-7, sched[3]
    assert abs(result.total_service_delay - 80.0) <= 1e-7, result.total_service_delay
    output["suffix_propagation"] = {
        "extraction": qcore.activities_to_jsonable(extraction),
        "schedule": qcore.schedule_result_to_jsonable(result),
    }

    return output


def main() -> None:
    parser = argparse.ArgumentParser(description="Run synthetic Phase B queueing no-overlap tests.")
    parser.add_argument("--out", default="queueing_phase_b_synthetic_tests.json")
    args = parser.parse_args()

    output = run_cases()
    summary = {
        name: {
            "ok": payload["schedule"]["ok"],
            "total_delay": payload["schedule"]["total_delay"],
            "total_service_delay": payload["schedule"]["total_service_delay"],
            "conflict_count": len(payload["schedule"]["conflicts"]),
            "infeasible_arc_ids": payload["schedule"]["infeasible_arc_ids"],
        }
        for name, payload in output.items()
    }
    Path(args.out).write_text(json.dumps({"summary": summary, "cases": output}, indent=2), encoding="utf-8")
    print(json.dumps(summary, indent=2))


if __name__ == "__main__":
    main()
