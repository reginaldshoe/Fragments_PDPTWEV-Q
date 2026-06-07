"""Run cleanup audit across the staged refactor packages."""
from __future__ import annotations

import argparse
import json
from pathlib import Path

from ..evrp_refactor_v1e.audit import audit_many
from ..evrp_refactor_v1e.cleanup_manifest import EXPECTED_REFACTOR_PACKAGES, TRANSITION_ADAPTER_FUNCTIONS
from ..evrp_refactor_v1e.quality_gates import compile_package, write_json


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description='Run cleanup audit for staged refactor packages.')
    parser.add_argument('--paths', nargs='*', default=EXPECTED_REFACTOR_PACKAGES)
    parser.add_argument('--artefact', default='refactor_v1e_cleanup/artefacts/cleanup_audit.json')
    parser.add_argument('--compile', action='store_true', help='Also run compileall for each existing package.')
    return parser


def main(argv: list[str] | None = None) -> int:
    args = build_parser().parse_args(argv)
    audit = audit_many(args.paths)
    audit['transition_adapter_functions'] = sorted(TRANSITION_ADAPTER_FUNCTIONS)
    if args.compile:
        audit['compile_results'] = [compile_package(path) for path in args.paths]
    write_json(args.artefact, audit)
    print(json.dumps(audit, indent=2, sort_keys=True))
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
