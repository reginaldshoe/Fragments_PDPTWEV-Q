"""Regression gate for consolidated-v1b master split."""

from __future__ import annotations

import argparse
import math

from evrp_fragments_consolidated_v1.pipeline_v1b import run_solver, summary_to_json

EXPECTED = {
    "legacy_core_source_sha256": "e1db5b59971d852dc008ec6de1b9034ffb76b806092767222a95995d4f3950df",
    "base_paths": 126,
    "restricted_raw": 450,
    "restricted_dedup": 210,
    "restricted_meta": 210,
    "restricted_undominated": 28,
    "extended_raw": 112,
    "extended_dedup": 112,
    "extended_meta": 112,
    "extended_undominated": 112,
    "objective": 376.07267704045523,
    "theta": 65.73276248806948,
    "selected_arc_ids": [4, 12, 27, 67, 91, 103],
    "status": 2,
}


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Run consolidated-v1b c101C6_2 regression gate.")
    parser.add_argument("--instance", default="instances/c101C6_2.txt")
    parser.add_argument("--max-base-path-len", type=int, default=18)
    parser.add_argument("--k-max", type=int, default=1)
    parser.add_argument("--tolerance", type=float, default=1e-6)
    return parser.parse_args()


def assert_close(name: str, actual: float | None, expected: float, tolerance: float) -> None:
    if actual is None:
        raise AssertionError(f"{name} is None; expected {expected}")
    if not math.isclose(actual, expected, rel_tol=tolerance, abs_tol=tolerance):
        raise AssertionError(f"{name} mismatch: actual={actual}, expected={expected}")


def main() -> None:
    args = parse_args()
    summary = run_solver(
        instance=args.instance,
        max_base_path_len=args.max_base_path_len,
        k_max=args.k_max,
        force_exact_k=True,
        use_callback=True,
    )
    print(summary_to_json(summary))

    for key in (
        "legacy_core_source_sha256", "base_paths", "restricted_raw", "restricted_dedup",
        "restricted_meta", "restricted_undominated", "extended_raw", "extended_dedup",
        "extended_meta", "extended_undominated", "selected_arc_ids", "status",
    ):
        actual = getattr(summary, key)
        expected = EXPECTED[key]
        if actual != expected:
            raise AssertionError(f"{key} mismatch: actual={actual}, expected={expected}")

    assert_close("objective", summary.objective, EXPECTED["objective"], args.tolerance)
    assert_close("theta", summary.theta, EXPECTED["theta"], args.tolerance)
    print("consolidated_v1b master split regression gate passed")


if __name__ == "__main__":
    main()
