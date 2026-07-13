"""Fragment-generation statistics runner with hard wall-clock timeout.

Environment:
- Python 3.12
- Standard library only, plus the local evrp_fragments package in worker mode
Target shape:
- Standalone script, run from the repository root

Role:
- Generate fragment-pipeline statistics without modifying or solving the master model.
- Does not import or modify experiments/run_solver.py.
- Does not build the Gurobi model.
- Enforces --fragment-time-limit-sec as a parent-process hard timeout.

Examples:
    python .\run_fragment_generation_stats.py --instance .\instances\c101C12.txt --max-base-path-len 12
    python .\run_fragment_generation_stats.py --instance .\instances\c101C12_4.txt --max-base-path-len 20 --fragment-time-limit-sec 60 --output-json .\fragment_stats_c101C12_4_L20.json
"""

from __future__ import annotations

import argparse
import contextlib
import io
import json
import os
import re
import subprocess
import sys
import traceback
from collections import Counter
from pathlib import Path
from time import perf_counter
from typing import Any, Callable


BASE_PATH_DOMINANCE_RE = re.compile(
    r"\[BASE-PATH-DOMINANCE\]\s+"
    r"source=(?P<source>\S+)\s+"
    r"depth=(?P<depth>-?\d+)\s+"
    r"before=(?P<before>\d+)\s+"
    r"after=(?P<after>\d+)\s+"
    r"removed=(?P<removed>\d+)\s+"
    r"keys=(?P<keys>\d+)\s+"
    r"max_bucket_size=(?P<max_bucket_size>\d+)"
)


frag: Any = None


def _jsonable(value: Any) -> Any:
    if value is None or isinstance(value, (str, int, float, bool)):
        return value
    if isinstance(value, Path):
        return str(value)
    if isinstance(value, dict):
        return {str(k): _jsonable(v) for k, v in value.items()}
    if isinstance(value, (list, tuple)):
        return [_jsonable(v) for v in value]
    if isinstance(value, (set, frozenset)):
        return sorted(_jsonable(v) for v in value)
    try:
        return float(value)
    except Exception:
        return repr(value)


def _seq_of(item: Any) -> Any:
    if isinstance(item, dict):
        return item.get("seq", item.get("Seq", item.get("path", ())))
    return item


def _length_distribution(items: list[Any] | None) -> dict[str, int] | None:
    if items is None:
        return None
    counts: Counter[str] = Counter()
    for item in items:
        seq = _seq_of(item)
        try:
            key = str(len(seq))
        except Exception:
            key = "unknown"
        counts[key] += 1
    return dict(sorted(counts.items(), key=lambda kv: (kv[0] == "unknown", int(kv[0]) if kv[0].isdigit() else 10**9)))


def _reduction_stats(before: list[Any] | None, after: list[Any] | None) -> dict[str, Any] | None:
    if before is None or after is None:
        return None
    before_n = len(before)
    after_n = len(after)
    removed = before_n - after_n
    return {
        "input": before_n,
        "output": after_n,
        "removed": removed,
        "removal_rate": None if before_n == 0 else removed / before_n,
    }


def _metadata_lookup(metadata: Any, *keys: str) -> Any:
    if not isinstance(metadata, dict):
        return None
    for key in keys:
        if key in metadata:
            return metadata[key]
    return None


def _time_like_metadata(metadata: Any) -> Any:
    if not isinstance(metadata, dict):
        return None
    out: dict[str, Any] = {}
    for key, value in metadata.items():
        key_lower = str(key).lower()
        if "time" in key_lower or "sec" in key_lower or "elapsed" in key_lower or "duration" in key_lower:
            out[str(key)] = _jsonable(value)
    return out or None


def _parse_base_path_dominance_events(log_text: str) -> list[dict[str, Any]]:
    events: list[dict[str, Any]] = []
    for line in log_text.splitlines():
        match = BASE_PATH_DOMINANCE_RE.search(line.strip())
        if not match:
            continue
        event = match.groupdict()
        event["depth"] = int(event["depth"])
        event["before"] = int(event["before"])
        event["after"] = int(event["after"])
        event["removed"] = int(event["removed"])
        event["keys"] = int(event["keys"])
        event["max_bucket_size"] = int(event["max_bucket_size"])
        before = event["before"]
        event["removal_rate"] = None if before == 0 else event["removed"] / before
        events.append(event)
    return events


def _safe_is_pickup(data: dict[str, Any] | None, sid: Any) -> bool:
    if data is None or frag is None:
        return False
    try:
        return bool(frag.is_pickup(data, sid))
    except Exception:
        return False


def _extension_stats(data: dict[str, Any] | None, extended_raw: list[dict[str, Any]] | None) -> dict[str, Any] | None:
    if extended_raw is None:
        return None

    depot = 0
    pickup = 0
    pickup_direct = 0
    pickup_with_station = 0
    other = 0
    missing_ext_to = 0

    for item in extended_raw:
        ext_to = item.get("ext_to")
        ext_station = item.get("ext_station")
        if ext_to is None:
            missing_ext_to += 1
            continue
        if ext_to == "D0":
            depot += 1
            continue
        if _safe_is_pickup(data, ext_to):
            pickup += 1
            if ext_station is None:
                pickup_direct += 1
            else:
                pickup_with_station += 1
        else:
            other += 1

    return {
        "accepted_total": len(extended_raw),
        "depot_extensions": depot,
        "pickup_extensions": pickup,
        "direct_pickup_extensions": pickup_direct,
        "station_mediated_pickup_extensions": pickup_with_station,
        "other_extensions": other,
        "missing_ext_to": missing_ext_to,
        "failed_extensions": None,
        "failed_extensions_note": "Not exposed by extend_all_fragments(); this script reports accepted extended fragments only.",
    }


class TeeCapture(io.TextIOBase):
    """Capture stdout in memory and flush it to a log file for hard-timeout recovery."""

    def __init__(self, log_path: Path | None):
        self.buffer = io.StringIO()
        self.log_path = log_path
        self.file = None
        if log_path is not None:
            log_path.parent.mkdir(parents=True, exist_ok=True)
            self.file = log_path.open("a", encoding="utf-8")

    def write(self, s: str) -> int:
        self.buffer.write(s)
        if self.file is not None:
            self.file.write(s)
            self.file.flush()
        return len(s)

    def flush(self) -> None:
        # TextIOBase.close() may call flush() after our file handle has already
        # been closed. Guard both None and closed states so cleanup is idempotent.
        if self.file is not None and not self.file.closed:
            self.file.flush()

    def close(self) -> None:
        # Make close() idempotent. Close and detach the backing file before
        # delegating to TextIOBase.close(), because the base class calls flush().
        file_handle = self.file
        self.file = None
        try:
            if file_handle is not None and not file_handle.closed:
                file_handle.flush()
                file_handle.close()
        finally:
            super().close()

    def getvalue(self) -> str:
        return self.buffer.getvalue()


def _atomic_write_json(path: Path, payload: dict[str, Any]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    tmp = path.with_suffix(path.suffix + ".tmp")
    tmp.write_text(json.dumps(payload, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    tmp.replace(path)


class FragmentStatsRun:
    """Stateful worker-side stats runner with checkpoint writes after safe stages."""

    def __init__(
        self,
        instance: str | Path,
        max_base_path_len: int,
        checkpoint_json: str | None,
        log_file: str | None,
    ):
        self.instance = str(instance)
        self.max_base_path_len = int(max_base_path_len)
        self.checkpoint_json = Path(checkpoint_json) if checkpoint_json else None
        self.log_file = Path(log_file) if log_file else None
        self.timings: dict[str, float] = {}
        self.completed_stages: list[str] = []
        self.current_stage: str | None = None
        self.pipeline_start = perf_counter()
        self.log_capture = TeeCapture(self.log_file)

        self.data: dict[str, Any] | None = None
        self.base_paths: list[Any] | None = None
        self.base_path_metadata: Any = None
        self.restricted_raw: list[dict[str, Any]] | None = None
        self.restricted_dedup: list[dict[str, Any]] | None = None
        self.restricted_meta: list[dict[str, Any]] | None = None
        self.restricted_undominated: list[dict[str, Any]] | None = None
        self.extended_raw: list[dict[str, Any]] | None = None
        self.extended_dedup: list[dict[str, Any]] | None = None
        self.extended_meta: list[dict[str, Any]] | None = None
        self.extended_undominated: list[dict[str, Any]] | None = None

    def write_checkpoint(self, status: str = "running", exception: BaseException | None = None) -> None:
        if self.checkpoint_json is None:
            return
        _atomic_write_json(self.checkpoint_json, self.to_payload(status=status, exception=exception))

    def _timed_call(self, label: str, fn: Callable[..., Any], *args: Any, **kwargs: Any) -> Any:
        self.current_stage = label
        self.write_checkpoint(status="running")
        start = perf_counter()
        try:
            with contextlib.redirect_stdout(self.log_capture):
                result = fn(*args, **kwargs)
        except Exception:
            self.timings[label] = perf_counter() - start
            self.write_checkpoint(status="failed")
            raise
        self.timings[label] = perf_counter() - start
        self.completed_stages.append(label)
        self.current_stage = None
        self.write_checkpoint(status="running")
        return result

    def run(self) -> dict[str, Any]:
        self.write_checkpoint(status="running")
        self.data = self._timed_call("read_instance", frag.read_instance, self.instance)
        self.base_paths, self.base_path_metadata = self._timed_call(
            "base_path_generation",
            frag.enumerate_base_paths,
            self.data,
            self.max_base_path_len,
        )
        self.restricted_raw = self._timed_call("restricted_generation", frag.enumerate_fragments, self.data, self.base_paths)
        self.restricted_dedup = self._timed_call("restricted_dedup", frag.dedup_exact, self.restricted_raw)
        self.restricted_meta = self._timed_call("restricted_metadata", frag.attach_metadata, self.data, self.restricted_dedup)
        self.restricted_undominated = self._timed_call("restricted_dominance", frag.dominance_filter, self.restricted_meta)
        self.extended_raw = self._timed_call("extended_generation", frag.extend_all_fragments, self.data, self.restricted_undominated)
        self.extended_dedup = self._timed_call("extended_dedup", frag.dedup_exact, self.extended_raw)
        self.extended_meta = self._timed_call(
            "extended_metadata",
            frag.attach_metadata,
            self.data,
            self.extended_dedup,
            exclude_last_ef=True,
        )
        self.extended_undominated = self._timed_call("extended_dominance", frag.dominance_filter, self.extended_meta)
        payload = self.to_payload(status="completed")
        self.write_checkpoint(status="completed")
        return payload

    def to_payload(self, status: str, exception: BaseException | None = None) -> dict[str, Any]:
        live_log_text = self.log_capture.getvalue()
        file_log_text = ""
        if self.log_file is not None and self.log_file.exists():
            try:
                file_log_text = self.log_file.read_text(encoding="utf-8")
            except Exception:
                file_log_text = ""
        log_text = file_log_text if len(file_log_text) >= len(live_log_text) else live_log_text
        base_path_dominance_events = _parse_base_path_dominance_events(log_text)
        total_time = perf_counter() - self.pipeline_start

        exception_payload = None
        if exception is not None:
            exception_payload = {
                "type": type(exception).__name__,
                "message": str(exception),
                "traceback": traceback.format_exception_only(type(exception), exception),
            }

        return {
            "script": "run_fragment_generation_stats.py",
            "stats_schema_version": "v6b1-hard-timeout",
            "status": status,
            "timeout_mode": "subprocess_wall_clock",
            "current_stage": self.current_stage,
            "failed_stage": self.current_stage if status in {"failed", "hard_timeout", "timeout"} else None,
            "completed_stages": list(self.completed_stages),
            "exception": exception_payload,
            "instance": self.instance,
            "max_base_path_len": self.max_base_path_len,
            "total_fragment_pipeline_time_sec": total_time,
            "stage_times_sec": self.timings,
            "base_paths": {
                "total": None if self.base_paths is None else len(self.base_paths),
                "length_distribution": _length_distribution(self.base_paths),
                "generation_time_sec": self.timings.get("base_path_generation"),
                "metadata": _jsonable(self.base_path_metadata),
                "metadata_time_like_fields": _time_like_metadata(self.base_path_metadata),
                "metadata_depth_layers": _jsonable(_metadata_lookup(self.base_path_metadata, "depth_layers", "layers", "layer_stats")),
                "metadata_pruning_by_reason": _jsonable(_metadata_lookup(self.base_path_metadata, "pruning_by_reason", "prune_reasons")),
                "metadata_dominance_statistics": _jsonable(_metadata_lookup(self.base_path_metadata, "dominance", "dominance_statistics")),
                "dominance_events_from_log": base_path_dominance_events,
            },
            "restricted_fragments": {
                "raw": None if self.restricted_raw is None else len(self.restricted_raw),
                "dedup": None if self.restricted_dedup is None else len(self.restricted_dedup),
                "metadata": None if self.restricted_meta is None else len(self.restricted_meta),
                "undominated": None if self.restricted_undominated is None else len(self.restricted_undominated),
                "generation_time_sec": self.timings.get("restricted_generation"),
                "total_processing_time_sec": sum(self.timings.get(k, 0.0) for k in ("restricted_generation", "restricted_dedup", "restricted_metadata", "restricted_dominance")) or None,
                "raw_length_distribution": _length_distribution(self.restricted_raw),
                "dedup_length_distribution": _length_distribution(self.restricted_dedup),
                "undominated_length_distribution": _length_distribution(self.restricted_undominated),
                "deduplication": _reduction_stats(self.restricted_raw, self.restricted_dedup),
                "dominance": _reduction_stats(self.restricted_meta, self.restricted_undominated),
                "pruning_by_reason": None,
                "pruning_by_reason_note": "Not exposed by the current restricted-fragment enumeration interface.",
            },
            "extended_fragments": {
                "raw": None if self.extended_raw is None else len(self.extended_raw),
                "dedup": None if self.extended_dedup is None else len(self.extended_dedup),
                "metadata": None if self.extended_meta is None else len(self.extended_meta),
                "undominated": None if self.extended_undominated is None else len(self.extended_undominated),
                "generation_time_sec": self.timings.get("extended_generation"),
                "total_processing_time_sec": sum(self.timings.get(k, 0.0) for k in ("extended_generation", "extended_dedup", "extended_metadata", "extended_dominance")) or None,
                "raw_length_distribution": _length_distribution(self.extended_raw),
                "dedup_length_distribution": _length_distribution(self.extended_dedup),
                "undominated_length_distribution": _length_distribution(self.extended_undominated),
                "deduplication": _reduction_stats(self.extended_raw, self.extended_dedup),
                "dominance": _reduction_stats(self.extended_meta, self.extended_undominated),
                "extension_statistics": _extension_stats(self.data, self.extended_raw),
            },
            "captured_log_summary": {
                "line_count": len(log_text.splitlines()),
                "base_path_dominance_event_count": len(base_path_dominance_events),
                "log_file": None if self.log_file is None else str(self.log_file),
            },
        }


def _load_json_if_exists(path: Path) -> dict[str, Any] | None:
    if not path.exists():
        return None
    try:
        return json.loads(path.read_text(encoding="utf-8"))
    except Exception:
        return None


def _write_payload(payload: dict[str, Any], output_json: str | None) -> None:
    text = json.dumps(payload, indent=2, sort_keys=True) + "\n"
    if output_json:
        Path(output_json).write_text(text, encoding="utf-8")
    else:
        print(text, end="")


def _supervise(args: argparse.Namespace) -> int:
    timeout = args.fragment_time_limit_sec
    output_path = Path(args.output_json) if args.output_json else None
    stem_base = output_path if output_path is not None else Path("fragment_generation_stats_stdout.json")
    checkpoint_path = stem_base.with_suffix(stem_base.suffix + ".checkpoint")
    log_path = stem_base.with_suffix(stem_base.suffix + ".log")

    for path in (checkpoint_path, log_path):
        try:
            if path.exists():
                path.unlink()
        except Exception:
            pass

    cmd = [
        sys.executable,
        str(Path(__file__).resolve()),
        "--_worker",
        "--instance",
        args.instance,
        "--max-base-path-len",
        str(args.max_base_path_len),
        "--checkpoint-json",
        str(checkpoint_path),
        "--log-file",
        str(log_path),
    ]

    started = perf_counter()
    proc = subprocess.Popen(cmd)
    try:
        return_code = proc.wait(timeout=timeout if timeout is not None else None)
    except subprocess.TimeoutExpired:
        proc.kill()
        proc.wait()
        checkpoint = _load_json_if_exists(checkpoint_path) or {}
        log_text = log_path.read_text(encoding="utf-8") if log_path.exists() else ""
        events = _parse_base_path_dominance_events(log_text)
        payload = checkpoint if checkpoint else {
            "script": "run_fragment_generation_stats.py",
            "stats_schema_version": "v6b1-hard-timeout",
            "instance": args.instance,
            "max_base_path_len": int(args.max_base_path_len),
            "completed_stages": [],
            "stage_times_sec": {},
            "base_paths": {"dominance_events_from_log": events},
        }
        payload["status"] = "hard_timeout"
        payload["timeout_mode"] = "subprocess_wall_clock"
        payload["fragment_time_limit_sec"] = float(timeout) if timeout is not None else None
        payload["hard_timeout_elapsed_sec"] = perf_counter() - started
        payload["failed_stage"] = payload.get("current_stage") or payload.get("failed_stage")
        payload["exception"] = {
            "type": "HardTimeout",
            "message": f"worker process killed after wall-clock timeout: limit={float(timeout):.3f}s",
        }
        payload.setdefault("captured_log_summary", {})
        payload["captured_log_summary"].update({
            "line_count": len(log_text.splitlines()),
            "base_path_dominance_event_count": len(events),
            "log_file": str(log_path),
        })
        payload.setdefault("base_paths", {})
        payload["base_paths"]["dominance_events_from_log"] = events
        _write_payload(payload, args.output_json)
        print(f"run_fragment_generation_stats.py: hard_timeout during {payload.get('failed_stage')}: limit={float(timeout):.3f}s", file=sys.stderr)
        return 2

    checkpoint = _load_json_if_exists(checkpoint_path)
    if checkpoint is not None:
        checkpoint["fragment_time_limit_sec"] = float(timeout) if timeout is not None else None
        _write_payload(checkpoint, args.output_json)
    return return_code


def _worker(args: argparse.Namespace) -> int:
    global frag
    from evrp_fragments import fragment_core as imported_frag
    frag = imported_frag

    run = FragmentStatsRun(args.instance, args.max_base_path_len, args.checkpoint_json, args.log_file)
    try:
        payload = run.run()
        if args.checkpoint_json:
            _atomic_write_json(Path(args.checkpoint_json), payload)
        return 0
    except Exception as exc:
        payload = run.to_payload(status="failed", exception=exc)
        if args.checkpoint_json:
            _atomic_write_json(Path(args.checkpoint_json), payload)
        print(f"run_fragment_generation_stats.py worker failed during {payload.get('failed_stage')}: {exc}", file=sys.stderr)
        return 1
    finally:
        run.log_capture.close()


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Generate fragment-pipeline statistics without solving the master model.")
    parser.add_argument("--instance", required=True, help="Path to the instance file.")
    parser.add_argument("--max-base-path-len", type=int, required=True, help="Maximum base-path length.")
    parser.add_argument(
        "--fragment-time-limit-sec",
        type=float,
        default=None,
        help="Optional hard wall-clock limit in seconds for the worker process.",
    )
    parser.add_argument(
        "--output-json",
        default=None,
        help="Optional path to write the JSON output. If omitted, JSON is printed to stdout after worker completion.",
    )
    parser.add_argument("--_worker", action="store_true", help=argparse.SUPPRESS)
    parser.add_argument("--checkpoint-json", default=None, help=argparse.SUPPRESS)
    parser.add_argument("--log-file", default=None, help=argparse.SUPPRESS)
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    if args.max_base_path_len <= 0:
        raise ValueError("--max-base-path-len must be positive")
    if args.fragment_time_limit_sec is not None and args.fragment_time_limit_sec <= 0:
        raise ValueError("--fragment-time-limit-sec must be positive when supplied")

    if args._worker:
        raise SystemExit(_worker(args))
    raise SystemExit(_supervise(args))


if __name__ == "__main__":
    main()
