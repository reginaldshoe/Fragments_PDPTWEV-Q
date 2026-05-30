"""Audit helpers for staged refactor cleanup.

The functions in this module use only the Python standard library. They do not
modify source files.
"""
from __future__ import annotations

import ast
from dataclasses import dataclass, asdict
from pathlib import Path
from typing import Any


@dataclass(frozen=True)
class FunctionDefinition:
    name: str
    file: str
    line: int


@dataclass(frozen=True)
class PackageAudit:
    path: str
    exists: bool
    python_file_count: int
    function_count: int
    duplicate_functions: dict[str, list[dict[str, Any]]]


def list_python_files(path: str | Path) -> list[Path]:
    root = Path(path)
    if not root.exists():
        return []
    return sorted(p for p in root.rglob('*.py') if '__pycache__' not in p.parts)


def extract_function_defs(file_path: str | Path) -> list[FunctionDefinition]:
    path = Path(file_path)
    try:
        tree = ast.parse(path.read_text(encoding='utf-8'))
    except SyntaxError:
        return []
    defs: list[FunctionDefinition] = []
    for node in ast.walk(tree):
        if isinstance(node, (ast.FunctionDef, ast.AsyncFunctionDef)):
            defs.append(FunctionDefinition(name=node.name, file=str(path), line=node.lineno))
    return defs


def find_duplicate_function_names(files: list[Path]) -> dict[str, list[FunctionDefinition]]:
    by_name: dict[str, list[FunctionDefinition]] = {}
    for file in files:
        for definition in extract_function_defs(file):
            by_name.setdefault(definition.name, []).append(definition)
    return {name: defs for name, defs in by_name.items() if len(defs) > 1}


def audit_package(path: str | Path) -> PackageAudit:
    root = Path(path)
    files = list_python_files(root)
    definitions = [definition for file in files for definition in extract_function_defs(file)]
    duplicates = find_duplicate_function_names(files)
    duplicate_payload = {
        name: [asdict(definition) for definition in defs]
        for name, defs in sorted(duplicates.items())
    }
    return PackageAudit(
        path=str(root),
        exists=root.exists(),
        python_file_count=len(files),
        function_count=len(definitions),
        duplicate_functions=duplicate_payload,
    )


def audit_many(paths: list[str | Path]) -> dict[str, Any]:
    return {'packages': [asdict(audit_package(path)) for path in paths]}
