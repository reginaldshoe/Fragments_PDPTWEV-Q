"""Compare/build the v1b fragment pipeline.

This script does not solve the master problem. It only runs the fragment build
sequence and writes a summary artefact.
"""

from __future__ import annotations

import argparse
import json
from dataclasses import asdict
from pathlib import Path
from typing import Any

from ..evrp_fragments_v1b.fragments.base_paths import read_instance
from ..evrp_fragments_v1b.fragments.pipeline import build_fragment_sets, summarise_fragments


def json_default(value: Any) -> Any:
    if isinstance(value, frozenset | set):
        return sorted(value)
    if isinstance(value, tuple):
        return list(value)
    return str(value)


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Build the v1b fragment-pipeline summary.")
    parser.add_argument("--source", default="ev_fragmentsv3.py")
    parser.add_argument("--instance", default="instances/c101C6_2.txt")
    parser.add_argument("--max-base-path-len", type=int, default=18)
    parser.add_argument(
        "--artefact",
        default="refactor_v1b_fragment_pipeline/artefacts/fragment_pipeline_summary.json",
    )
    return parser


def main(argv: list[str] | None = None) -> int:
    args = build_parser().parse_args(argv)
    data = read_instance(args.instance, source_path=args.source)
    result = build_fragment_sets(
        data=data,
        max_base_path_len=args.max_base_path_len,
        source_path=args.source,
    )

    payload = {
        "source": str(Path(args.source)),
        "instance": str(Path(args.instance)),
        "max_base_path_len": args.max_base_path_len,
        "counts": result.summary(),
        "restricted_raw_summary": asdict(summarise_fragments(result.restricted_raw)),
        "restricted_undominated_summary": asdict(summarise_fragments(result.restricted_undominated)),
        "extended_raw_summary": asdict(summarise_fragments(result.extended_raw)),
        "extended_undominated_summary": asdict(summarise_fragments(result.extended_undominated)),
    }

    artefact_path = Path(args.artefact)
    artefact_path.parent.mkdir(parents=True, exist_ok=True)
    artefact_path.write_text(json.dumps(payload, indent=2, default=json_default), encoding="utf-8")
    print(json.dumps(payload["counts"], indent=2))
    print(f"Wrote {artefact_path}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
