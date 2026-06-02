"""Compare legacy ev_fragmentsv3.py functions against integrated evrp_fragments functions.

This is an offline diagnostic. It does not solve the model and does not invoke
Gurobi. It extracts function source code using AST and emits line counts,
SHA-256 hashes, signatures where available, and unified diffs.

Usage:
    python -m experiments.compare_legacy_refactor_functions --legacy ev_fragmentsv3.py
"""
from __future__ import annotations

import argparse
import ast
import difflib
import hashlib
import inspect
import json
from pathlib import Path
from typing import Any


TARGETS = {
    'callback': 'evrp_fragments/callback/lazy_cuts.py',
    'dp_leg_frontier_charge_to_full': 'evrp_fragments/callback/energy_dp.py',
    'dp_route_min_dist': 'evrp_fragments/callback/energy_dp.py',
    'extract_routes_from_solution': 'evrp_fragments/callback/route_tools.py',
    'stitch_sid_sequence': 'evrp_fragments/callback/route_tools.py',
    'route_distance_from_sids': 'evrp_fragments/callback/route_tools.py',
    'build_master_model': 'evrp_fragments/master/model.py',
    'raw_depot_arcs': 'evrp_fragments/master/depot_arcs.py',
    'build_network': 'evrp_fragments/master/network.py',
}

# In the integrated implementation, the callback is a nested function returned
# by make_energy_callback, not a top-level `callback` function.
INTEGRATED_NAME_MAP = {
    'callback': 'make_energy_callback',
}


def _normalise_source(text: str) -> str:
    return '\n'.join(line.rstrip() for line in text.strip().splitlines()) + '\n'


def _sha(text: str | None) -> str | None:
    if text is None:
        return None
    return hashlib.sha256(_normalise_source(text).encode('utf-8')).hexdigest()


def _extract_function_source(path: Path, function_name: str) -> dict[str, Any]:
    if not path.exists():
        return {'exists': False, 'function_found': False, 'source': None, 'lineno': None, 'end_lineno': None, 'error': f'missing file: {path}'}
    text = path.read_text(encoding='utf-8')
    try:
        tree = ast.parse(text)
    except SyntaxError as exc:
        return {'exists': True, 'function_found': False, 'source': None, 'lineno': None, 'end_lineno': None, 'error': f'syntax error: {exc}'}
    lines = text.splitlines()
    for node in ast.walk(tree):
        if isinstance(node, (ast.FunctionDef, ast.AsyncFunctionDef)) and node.name == function_name:
            start = node.lineno
            end = getattr(node, 'end_lineno', node.lineno)
            source = '\n'.join(lines[start - 1:end]) + '\n'
            return {
                'exists': True,
                'function_found': True,
                'source': source,
                'lineno': start,
                'end_lineno': end,
                'line_count': end - start + 1,
                'sha256': _sha(source),
                'error': None,
            }
    return {'exists': True, 'function_found': False, 'source': None, 'lineno': None, 'end_lineno': None, 'error': f'function not found: {function_name}'}


def _signature_from_source(source: str | None, function_name: str) -> str | None:
    if source is None:
        return None
    try:
        module = ast.parse(source)
    except SyntaxError:
        return None
    fn = module.body[0] if module.body else None
    if not isinstance(fn, (ast.FunctionDef, ast.AsyncFunctionDef)):
        return None
    args = []
    all_args = list(fn.args.posonlyargs) + list(fn.args.args)
    default_offset = len(all_args) - len(fn.args.defaults)
    for idx, arg in enumerate(all_args):
        if idx >= default_offset:
            default = fn.args.defaults[idx - default_offset]
            args.append(f'{arg.arg}={ast.unparse(default)}')
        else:
            args.append(arg.arg)
    if fn.args.vararg:
        args.append('*' + fn.args.vararg.arg)
    if fn.args.kwonlyargs:
        args.append('*')
        for arg, default in zip(fn.args.kwonlyargs, fn.args.kw_defaults):
            if default is None:
                args.append(arg.arg)
            else:
                args.append(f'{arg.arg}={ast.unparse(default)}')
    if fn.args.kwarg:
        args.append('**' + fn.args.kwarg.arg)
    return f'{function_name}(' + ', '.join(args) + ')'


def _diff(a: str | None, b: str | None, fromfile: str, tofile: str, max_lines: int) -> list[str]:
    if a is None or b is None:
        return []
    diff = list(difflib.unified_diff(
        _normalise_source(a).splitlines(),
        _normalise_source(b).splitlines(),
        fromfile=fromfile,
        tofile=tofile,
        lineterm='',
    ))
    if len(diff) > max_lines:
        return diff[:max_lines] + [f'... diff truncated at {max_lines} lines ...']
    return diff


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description='Compare legacy and integrated callback/master functions.')
    parser.add_argument('--legacy', default='ev_fragmentsv3.py')
    parser.add_argument('--repo-root', default='.')
    parser.add_argument('--artefact', default='artefacts/legacy_refactor_function_diff.json')
    parser.add_argument('--max-diff-lines', type=int, default=220)
    return parser


def main(argv: list[str] | None = None) -> int:
    args = build_parser().parse_args(argv)
    repo = Path(args.repo_root)
    legacy_path = repo / args.legacy
    payload: dict[str, Any] = {
        'legacy': str(legacy_path),
        'comparisons': {},
        'summary': [],
    }

    for legacy_name, integrated_rel in TARGETS.items():
        integrated_path = repo / integrated_rel
        integrated_name = INTEGRATED_NAME_MAP.get(legacy_name, legacy_name)

        legacy = _extract_function_source(legacy_path, legacy_name)
        integrated = _extract_function_source(integrated_path, integrated_name)

        comparison = {
            'legacy_function': legacy_name,
            'integrated_function': integrated_name,
            'integrated_file': integrated_rel,
            'legacy_found': legacy['function_found'],
            'integrated_found': integrated['function_found'],
            'legacy_lineno': legacy.get('lineno'),
            'legacy_end_lineno': legacy.get('end_lineno'),
            'integrated_lineno': integrated.get('lineno'),
            'integrated_end_lineno': integrated.get('end_lineno'),
            'legacy_line_count': legacy.get('line_count'),
            'integrated_line_count': integrated.get('line_count'),
            'legacy_sha256': legacy.get('sha256'),
            'integrated_sha256': integrated.get('sha256'),
            'hash_match': legacy.get('sha256') == integrated.get('sha256') and legacy.get('sha256') is not None,
            'legacy_signature': _signature_from_source(legacy.get('source'), legacy_name),
            'integrated_signature': _signature_from_source(integrated.get('source'), integrated_name),
            'legacy_error': legacy.get('error'),
            'integrated_error': integrated.get('error'),
            'diff': _diff(
                legacy.get('source'),
                integrated.get('source'),
                f'legacy:{legacy_name}',
                f'integrated:{integrated_rel}:{integrated_name}',
                args.max_diff_lines,
            ),
        }
        payload['comparisons'][legacy_name] = comparison
        payload['summary'].append({
            'function': legacy_name,
            'integrated_file': integrated_rel,
            'legacy_found': comparison['legacy_found'],
            'integrated_found': comparison['integrated_found'],
            'hash_match': comparison['hash_match'],
            'legacy_signature': comparison['legacy_signature'],
            'integrated_signature': comparison['integrated_signature'],
            'legacy_line_count': comparison['legacy_line_count'],
            'integrated_line_count': comparison['integrated_line_count'],
        })

    out_path = repo / args.artefact
    out_path.parent.mkdir(parents=True, exist_ok=True)
    out_path.write_text(json.dumps(payload, indent=2), encoding='utf-8')
    print(json.dumps(payload['summary'], indent=2))
    print(f'Wrote {out_path}')
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
