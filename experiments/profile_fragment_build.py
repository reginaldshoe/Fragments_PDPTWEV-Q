#!/usr/bin/env python3
"""Build-only fragment pipeline profiler.

This runner is intentionally separate from experiments/run_solver.py. It sets
fragment-build environment flags before importing fragment_core so CLI switches affect
module-level configuration.
"""
from __future__ import annotations

import argparse
import os
from time import perf_counter


def _phase(name, fn):
    print(f"[FRAGMENT-PROFILE-PHASE-START] phase={name}", flush=True)
    t0 = perf_counter()
    result = fn()
    print(f"[FRAGMENT-PROFILE-PHASE-END] phase={name} elapsed_sec={perf_counter() - t0:.3f}", flush=True)
    return result


def _split_base_path_result(result):
    if isinstance(result, tuple) and len(result) == 2 and isinstance(result[1], dict):
        return result[0], result[1]
    return result, None


def main() -> int:
    parser = argparse.ArgumentParser(description="Build-only fragment pipeline profiler")
    parser.add_argument("--instance", required=True)
    parser.add_argument("--max-base-path-len", type=int, required=True)
    parser.add_argument("--build-time-limit-sec", type=float, default=None)
    parser.add_argument("--progress-interval", type=int, default=100000)
    parser.add_argument("--base-path-only", action="store_true")
    parser.add_argument("--disable-base-path-dominance", action="store_true")
    args = parser.parse_args()

    os.environ["EVRP_FRAGMENT_BUILD_DIAGNOSTICS"] = "1"
    os.environ["EVRP_FRAGMENT_BUILD_PROGRESS_INTERVAL"] = str(max(1, args.progress_interval))
    if args.build_time_limit_sec is not None:
        os.environ["EVRP_FRAGMENT_BUILD_TIME_LIMIT_SEC"] = str(args.build_time_limit_sec)
    if args.disable_base_path_dominance:
        os.environ["EVRP_BASE_PATH_WORKING_SET_DOMINANCE"] = "0"

    # Import only after setting environment variables. fragment_core reads these at import time.
    from evrp_fragments.fragment_core import (
        read_instance,
        enumerate_base_paths,
        enumerate_fragments,
        dedup_exact,
        attach_metadata,
        dominance_filter,
        extend_all_fragments,
        dedup_by_signature,
    )

    print(
        f"[FRAGMENT-PROFILE-START] instance={args.instance} "
        f"max_base_path_len={args.max_base_path_len} "
        f"base_path_dominance={os.getenv('EVRP_BASE_PATH_WORKING_SET_DOMINANCE', '1')}",
        flush=True,
    )

    status = "completed"
    reason = None
    t0 = perf_counter()
    keys = [
        "base_paths",
        "restricted_raw",
        "restricted_dedup",
        "restricted_meta",
        "restricted_undominated",
        "extended_raw",
        "extended_meta",
        "extended_dedup",
        "extended_undominated",
    ]
    counts = {k: None for k in keys}

    try:
        data = _phase("read_instance", lambda: read_instance(args.instance))
        base_result = _phase(
            "enumerate_base_paths",
            lambda: enumerate_base_paths(data, args.max_base_path_len),
        )
        base_paths, base_pruned = _split_base_path_result(base_result)
        counts["base_paths"] = len(base_paths)
        print(f"[FRAGMENT-PROFILE-COUNT] base_paths={counts['base_paths']}", flush=True)
        if base_pruned is not None:
            print(f"[FRAGMENT-PROFILE-COUNT] base_path_pruned={base_pruned}", flush=True)

        if not args.base_path_only:
            restricted_raw = _phase("enumerate_restricted_fragments", lambda: enumerate_fragments(data, base_paths))
            counts["restricted_raw"] = len(restricted_raw)
            print(f"[FRAGMENT-PROFILE-COUNT] restricted_raw={len(restricted_raw)}", flush=True)

            restricted_dedup = _phase("dedup_restricted_exact", lambda: dedup_exact(restricted_raw))
            counts["restricted_dedup"] = len(restricted_dedup)
            print(f"[FRAGMENT-PROFILE-COUNT] restricted_dedup={len(restricted_dedup)}", flush=True)

            restricted_meta = _phase("attach_restricted_metadata", lambda: attach_metadata(data, restricted_dedup))
            counts["restricted_meta"] = len(restricted_meta)
            print(f"[FRAGMENT-PROFILE-COUNT] restricted_meta={len(restricted_meta)}", flush=True)

            restricted_undominated = _phase("dominance_filter_restricted", lambda: dominance_filter(restricted_meta))
            counts["restricted_undominated"] = len(restricted_undominated)
            print(f"[FRAGMENT-PROFILE-COUNT] restricted_undominated={len(restricted_undominated)}", flush=True)

            extended_raw = _phase("extend_fragments", lambda: extend_all_fragments(data, restricted_undominated))
            counts["extended_raw"] = len(extended_raw)
            print(f"[FRAGMENT-PROFILE-COUNT] extended_raw={len(extended_raw)}", flush=True)

            extended_meta = _phase("attach_extended_metadata", lambda: attach_metadata(data, extended_raw, exclude_last_ef=True))
            counts["extended_meta"] = len(extended_meta)
            print(f"[FRAGMENT-PROFILE-COUNT] extended_meta={len(extended_meta)}", flush=True)

            extended_dedup = _phase("dedup_extended_signature", lambda: dedup_by_signature(extended_meta))
            counts["extended_dedup"] = len(extended_dedup)
            print(f"[FRAGMENT-PROFILE-COUNT] extended_dedup={len(extended_dedup)}", flush=True)

            extended_undominated = _phase("dominance_filter_extended", lambda: dominance_filter(extended_dedup))
            counts["extended_undominated"] = len(extended_undominated)
            print(f"[FRAGMENT-PROFILE-COUNT] extended_undominated={len(extended_undominated)}", flush=True)

    except TimeoutError as exc:
        status = "aborted"
        reason = f"TimeoutError:{exc}"
    except Exception as exc:
        status = "failed"
        reason = f"{type(exc).__name__}:{exc}"

    elapsed = perf_counter() - t0
    print(
        "[FRAGMENT-PROFILE-SUMMARY] "
        + f"status={status} reason={reason} elapsed_sec={elapsed:.3f} "
        + " ".join(f"{k}={v}" for k, v in counts.items()),
        flush=True,
    )
    return 0 if status == "completed" else 1


if __name__ == "__main__":
    raise SystemExit(main())
