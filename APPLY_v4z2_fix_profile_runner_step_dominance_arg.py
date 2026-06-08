#!/usr/bin/env python3
from __future__ import annotations
import argparse
import ast
import shutil
from datetime import datetime
from pathlib import Path

OVERLAY_NAME = "v4z2_fix_profile_runner_step_dominance_arg"
RUNNER_TEXT = "#!/usr/bin/env python3\n\"\"\"Build-only fragment pipeline profiler.\n\nThis runner is intentionally separate from experiments/run_solver.py. It sets\nfragment-build environment flags before importing fragment_core so CLI switches affect\nmodule-level configuration.\n\"\"\"\nfrom __future__ import annotations\n\nimport argparse\nimport os\nfrom time import perf_counter\n\n\ndef _phase(name, fn):\n    print(f\"[FRAGMENT-PROFILE-PHASE-START] phase={name}\", flush=True)\n    t0 = perf_counter()\n    result = fn()\n    print(f\"[FRAGMENT-PROFILE-PHASE-END] phase={name} elapsed_sec={perf_counter() - t0:.3f}\", flush=True)\n    return result\n\n\ndef _split_base_path_result(result):\n    if isinstance(result, tuple) and len(result) == 2 and isinstance(result[1], dict):\n        return result[0], result[1]\n    return result, None\n\n\ndef main() -> int:\n    parser = argparse.ArgumentParser(description=\"Build-only fragment pipeline profiler\")\n    parser.add_argument(\"--instance\", required=True)\n    parser.add_argument(\"--max-base-path-len\", type=int, required=True)\n    parser.add_argument(\"--build-time-limit-sec\", type=float, default=None)\n    parser.add_argument(\"--progress-interval\", type=int, default=100000)\n    parser.add_argument(\"--base-path-only\", action=\"store_true\")\n    parser.add_argument(\"--disable-base-path-dominance\", action=\"store_true\")\n    args = parser.parse_args()\n\n    os.environ[\"EVRP_FRAGMENT_BUILD_DIAGNOSTICS\"] = \"1\"\n    os.environ[\"EVRP_FRAGMENT_BUILD_PROGRESS_INTERVAL\"] = str(max(1, args.progress_interval))\n    if args.build_time_limit_sec is not None:\n        os.environ[\"EVRP_FRAGMENT_BUILD_TIME_LIMIT_SEC\"] = str(args.build_time_limit_sec)\n    if args.disable_base_path_dominance:\n        os.environ[\"EVRP_BASE_PATH_WORKING_SET_DOMINANCE\"] = \"0\"\n\n    # Import only after setting environment variables. fragment_core reads these at import time.\n    from evrp_fragments.fragment_core import (\n        read_instance,\n        enumerate_base_paths,\n        enumerate_fragments,\n        dedup_exact,\n        attach_metadata,\n        dominance_filter,\n        extend_all_fragments,\n        dedup_by_signature,\n    )\n\n    print(\n        f\"[FRAGMENT-PROFILE-START] instance={args.instance} \"\n        f\"max_base_path_len={args.max_base_path_len} \"\n        f\"base_path_dominance={os.getenv('EVRP_BASE_PATH_WORKING_SET_DOMINANCE', '1')}\",\n        flush=True,\n    )\n\n    status = \"completed\"\n    reason = None\n    t0 = perf_counter()\n    keys = [\n        \"base_paths\",\n        \"restricted_raw\",\n        \"restricted_dedup\",\n        \"restricted_meta\",\n        \"restricted_undominated\",\n        \"extended_raw\",\n        \"extended_meta\",\n        \"extended_dedup\",\n        \"extended_undominated\",\n    ]\n    counts = {k: None for k in keys}\n\n    try:\n        data = _phase(\"read_instance\", lambda: read_instance(args.instance))\n        base_result = _phase(\n            \"enumerate_base_paths\",\n            lambda: enumerate_base_paths(data, args.max_base_path_len),\n        )\n        base_paths, base_pruned = _split_base_path_result(base_result)\n        counts[\"base_paths\"] = len(base_paths)\n        print(f\"[FRAGMENT-PROFILE-COUNT] base_paths={counts['base_paths']}\", flush=True)\n        if base_pruned is not None:\n            print(f\"[FRAGMENT-PROFILE-COUNT] base_path_pruned={base_pruned}\", flush=True)\n\n        if not args.base_path_only:\n            restricted_raw = _phase(\"enumerate_restricted_fragments\", lambda: enumerate_fragments(data, base_paths))\n            counts[\"restricted_raw\"] = len(restricted_raw)\n            print(f\"[FRAGMENT-PROFILE-COUNT] restricted_raw={len(restricted_raw)}\", flush=True)\n\n            restricted_dedup = _phase(\"dedup_restricted_exact\", lambda: dedup_exact(restricted_raw))\n            counts[\"restricted_dedup\"] = len(restricted_dedup)\n            print(f\"[FRAGMENT-PROFILE-COUNT] restricted_dedup={len(restricted_dedup)}\", flush=True)\n\n            restricted_meta = _phase(\"attach_restricted_metadata\", lambda: attach_metadata(data, restricted_dedup))\n            counts[\"restricted_meta\"] = len(restricted_meta)\n            print(f\"[FRAGMENT-PROFILE-COUNT] restricted_meta={len(restricted_meta)}\", flush=True)\n\n            restricted_undominated = _phase(\"dominance_filter_restricted\", lambda: dominance_filter(restricted_meta))\n            counts[\"restricted_undominated\"] = len(restricted_undominated)\n            print(f\"[FRAGMENT-PROFILE-COUNT] restricted_undominated={len(restricted_undominated)}\", flush=True)\n\n            extended_raw = _phase(\"extend_fragments\", lambda: extend_all_fragments(data, restricted_undominated))\n            counts[\"extended_raw\"] = len(extended_raw)\n            print(f\"[FRAGMENT-PROFILE-COUNT] extended_raw={len(extended_raw)}\", flush=True)\n\n            extended_meta = _phase(\"attach_extended_metadata\", lambda: attach_metadata(data, extended_raw, exclude_last_ef=True))\n            counts[\"extended_meta\"] = len(extended_meta)\n            print(f\"[FRAGMENT-PROFILE-COUNT] extended_meta={len(extended_meta)}\", flush=True)\n\n            extended_dedup = _phase(\"dedup_extended_signature\", lambda: dedup_by_signature(extended_meta))\n            counts[\"extended_dedup\"] = len(extended_dedup)\n            print(f\"[FRAGMENT-PROFILE-COUNT] extended_dedup={len(extended_dedup)}\", flush=True)\n\n            extended_undominated = _phase(\"dominance_filter_extended\", lambda: dominance_filter(extended_dedup))\n            counts[\"extended_undominated\"] = len(extended_undominated)\n            print(f\"[FRAGMENT-PROFILE-COUNT] extended_undominated={len(extended_undominated)}\", flush=True)\n\n    except TimeoutError as exc:\n        status = \"aborted\"\n        reason = f\"TimeoutError:{exc}\"\n    except Exception as exc:\n        status = \"failed\"\n        reason = f\"{type(exc).__name__}:{exc}\"\n\n    elapsed = perf_counter() - t0\n    print(\n        \"[FRAGMENT-PROFILE-SUMMARY] \"\n        + f\"status={status} reason={reason} elapsed_sec={elapsed:.3f} \"\n        + \" \".join(f\"{k}={v}\" for k, v in counts.items()),\n        flush=True,\n    )\n    return 0 if status == \"completed\" else 1\n\n\nif __name__ == \"__main__\":\n    raise SystemExit(main())\n"


def _write(path: Path, text: str) -> None:
    path.write_text(text, encoding="utf-8", newline="\n")


def _parse_ok(text: str):
    try:
        ast.parse(text)
    except SyntaxError as exc:
        return False, f"{type(exc).__name__}: {exc}"
    return True, None


def main() -> int:
    parser = argparse.ArgumentParser(description=OVERLAY_NAME)
    parser.add_argument("--audit-only", action="store_true")
    args = parser.parse_args()

    repo_root = Path(__file__).resolve().parent
    runner_path = repo_root / "experiments" / "profile_fragment_build.py"
    report_path = repo_root / f"{OVERLAY_NAME}_report.txt"
    report = [
        f"[{OVERLAY_NAME.upper()}]",
        f"repo_root={repo_root}",
        f"runner_path={runner_path}",
        f"audit_only={args.audit_only}",
    ]

    ok, err = _parse_ok(RUNNER_TEXT)
    report.append(f"replacement_runner_ast_parse_ok={ok}")
    if err:
        report.append(f"replacement_runner_ast_parse_error={err}")
    if not ok:
        report.append("patch_written=False")
        print("\n".join(report))
        return 2

    existing = runner_path.read_text(encoding="utf-8") if runner_path.exists() else ""
    report.append(f"runner_exists_before={runner_path.exists()}")
    report.append(f"runner_changed={existing != RUNNER_TEXT}")
    report.append(f"removes_enable_step_dominance_reference={'enable_step_dominance' not in RUNNER_TEXT}")
    report.append(f"imports_fragment_core_after_parse_args={RUNNER_TEXT.find('parse_args()') < RUNNER_TEXT.find('from evrp_fragments.fragment_core import')}")

    if args.audit_only:
        report.append("patch_written=False")
        report.append("scope_note=Audit only. No files modified.")
        print("\n".join(report))
        return 0

    runner_path.parent.mkdir(exist_ok=True)
    if runner_path.exists():
        stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        backup = runner_path.with_name(f"profile_fragment_build.py.bak_{OVERLAY_NAME}_{stamp}")
        shutil.copy2(runner_path, backup)
        report.append(f"runner_backup_created={backup}")
    _write(runner_path, RUNNER_TEXT)
    report.append("runner_written=True")
    report.append("patch_written=True")
    report.append("scope_note=Profile runner fixed: removed stale step-dominance argument reference and moved fragment_core import after environment setup.")
    _write(report_path, "\n".join(report) + "\n")
    print("\n".join(report))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
