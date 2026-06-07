#!/usr/bin/env python3
from __future__ import annotations

print("*** APPLY_v3h_canonicalise_feasibility_cut_names.py IS RUNNING ***", flush=True)

from pathlib import Path
import py_compile
import re
import shutil
import sys
import traceback

TARGET = Path("evrp_fragments") / "callback_core.py"
MARKER = "CANONICAL_FEASIBILITY_CUT_NAMING_APPLIED"

RENAMES = [
    ("V3A_FEAS_DIAGNOSTICS", "FEASIBILITY_CUT_DIAGNOSTICS_ENABLED"),
    ("V3A_FEAS_MAX_FRONT_PRUNE_TESTS", "FEASIBILITY_CUT_MAX_FRONT_PRUNE_TESTS"),
    ("_v3a_feas_arc_seq", "_fragment_arc_sid_sequence"),
    ("_v3a_feas_stitch_sid_sequence", "_stitch_fragment_sid_sequence"),
    ("_v3a_feas_skeleton_and_cover", "_build_route_skeleton_and_cover"),
    ("_v3a_feas_test_chain", "_test_fragment_chain_feasibility"),
    ("_v3a_feas_diagnose_infeasible_route", "_diagnose_infeasible_route"),
    ("V3B_FEAS_DRY_RUN_VERBOSE", "FEASIBILITY_CUT_VERBOSE_CANDIDATE_TESTS"),
    ("V3B_FEAS_DRY_RUN", "FEASIBILITY_CUT_DRY_RUN_ENABLED"),
    ("V3B_FEAS_MIN_CHAIN_SIZE", "FEASIBILITY_CUT_MIN_CHAIN_SIZE"),
    ("_v3b_feas_find_front_pruned_candidate", "_find_front_pruned_infeasible_chain"),
    ("_v3b_feas_dry_run", "_dry_run_front_pruned_feasibility_cut"),
    ("V3C_MANUAL_FEAS_ACTIVE", "ACTIVE_FEASIBILITY_CUT_ENABLED"),
    ("V3C_MANUAL_FEAS_PRINT", "FEASIBILITY_CUT_PRINT_PER_CUT"),
    ("V3D_FEAS_SUMMARY_ONLY", "FEASIBILITY_CUT_SUMMARY_ONLY"),
    ("V3D_FEAS_SUMMARY", "FEASIBILITY_CUT_SUMMARY"),
    ("_v3d_record_feas_cut", "_record_feasibility_cut_summary"),
    ("_v3d_print_feas_summary", "_print_feasibility_cut_summary"),
]

TAG_RENAMES = [
    ("[V3A-FEAS]", "[FEAS-CUT-DIAGNOSTIC]"),
    ("[V3B-FEAS-DRY]", "[FEAS-CUT-DRY-RUN]"),
    ("[V3C-FEAS-ACTIVE]", "[FEAS-CUT-ACTIVE]"),
    ("[V3D-FEAS-SUMMARY]", "[FEAS-CUT-SUMMARY]"),
]


def fail(msg: str) -> None:
    print("ERROR:", msg, flush=True)
    sys.exit(1)


def compile_ok(path: Path) -> tuple[bool, str]:
    try:
        py_compile.compile(str(path), doraise=True)
        return True, ""
    except Exception as exc:
        return False, "".join(traceback.format_exception_only(type(exc), exc)).strip()


def replace_identifier(text: str, old: str, new: str) -> tuple[str, int]:
    # Identifier-safe replacement: avoid accidentally replacing substrings inside longer names.
    pattern = re.compile(rf"(?<![A-Za-z0-9_]){re.escape(old)}(?![A-Za-z0-9_])")
    return pattern.subn(new, text)


def strip_troubleshooting_marker_comments(text: str) -> str:
    kept = []
    for line in text.splitlines(keepends=True):
        stripped = line.strip()
        if stripped.startswith("# === V3") or stripped.startswith("# v3") or stripped.startswith("# Active guarded front-pruned feasibility cut is frozen") or stripped.startswith("# See docs/NOTES_v3e") or stripped.startswith("# Behaviour is unchanged by v3e"):
            continue
        if "V3B2 suppressed noisy v3a call" in stripped:
            continue
        kept.append(line)
    return "".join(kept)


def replace_find_candidate_docstring(text: str) -> str:
    pattern = re.compile(
        r'(def _find_front_pruned_infeasible_chain\(route, arcs, data, fail_i\):\n)(?P<indent>\s+)""".*?"""',
        flags=re.DOTALL,
    )
    replacement = (
        r'\1'
        '    """Return the smallest front-pruned selected-arc chain that retests infeasible.\n\n'
        '    The candidate is built from the selected route prefix ending at the arc that\n'
        '    covers the first failed skeleton edge. Leading arcs are removed one at a\n'
        '    time, and each remaining suffix is retested with the route DP. The returned\n'
        '    chain is used by the active feasibility cut when it is valid and smaller\n'
        '    than the full selected route.\n'
        '    """'
    )
    text, _ = pattern.subn(replacement, text, count=1)
    return text


def replace_diagnostic_docstrings(text: str) -> str:
    replacements = {
        '"""Return the SID sequence for a selected fragment arc."""': '"""Return the SID sequence stored on a selected fragment arc."""',
        '"""Stitch selected fragment arc SID sequences into one route SID sequence."""': '"""Stitch selected fragment-arc SID sequences into a route SID sequence."""',
        '"""Build station-free skeleton and map each skeleton edge to route arc position."""': '"""Build the customer/depot skeleton and map each skeleton edge to its covering arc."""',
        '"""Return (ok, fail_i, skeleton) for a candidate contiguous selected-arc chain."""': '"""Retest a contiguous selected-arc chain with the route DP."""',
        '"""Diagnostic-only helper for infeasible DP route cases."""': '"""Print optional diagnostics for an infeasible selected route."""',
        '"""Emit compact dry-run diagnostics for a future front-pruned feasibility cut."""': '"""Print optional dry-run information for the front-pruned feasibility cut."""',
    }
    for old, new in replacements.items():
        text = text.replace(old, new)
    return text


def insert_canonical_header(text: str) -> str:
    if MARKER in text:
        return text
    header = (
        "\n# === CANONICAL_FEASIBILITY_CUT_NAMING_APPLIED: start ===\n"
        "# Feasibility-cut helpers use production names rather than troubleshooting labels.\n"
        "# The active cut remains the front-pruned infeasible-chain cut with summary output.\n"
        "# === CANONICAL_FEASIBILITY_CUT_NAMING_APPLIED: end ===\n"
    )
    idx = text.find("\ndef callback(")
    if idx == -1:
        idx = text.find("def callback(")
    if idx == -1:
        return header + text
    return text[:idx] + header + text[idx:]


def main() -> None:
    print(f"[{MARKER}] cwd={Path.cwd()}", flush=True)
    if not TARGET.exists():
        fail(f"target not found: {TARGET}; run from repository root")

    original = TARGET.read_text(encoding="utf-8")
    if MARKER in original:
        print(f"[{MARKER}] marker already present; no changes made", flush=True)
        return

    backup = TARGET.with_suffix(TARGET.suffix + f".bak_before_{MARKER.lower()}")
    shutil.copy2(TARGET, backup)
    print(f"[{MARKER}] backup written to {backup}", flush=True)

    text = original
    total_replacements = 0
    for old, new in RENAMES:
        text, n = replace_identifier(text, old, new)
        if n:
            print(f"[{MARKER}] renamed {old} -> {new}: {n}", flush=True)
            total_replacements += n

    for old, new in TAG_RENAMES:
        n = text.count(old)
        text = text.replace(old, new)
        if n:
            print(f"[{MARKER}] renamed output tag {old} -> {new}: {n}", flush=True)
            total_replacements += n

    text = replace_find_candidate_docstring(text)
    text = replace_diagnostic_docstrings(text)
    text = strip_troubleshooting_marker_comments(text)
    text = insert_canonical_header(text)

    # Clarify that singleton candidates are permitted or not via the value, without v3 naming.
    text = text.replace(
        "# singleton candidates are not used as proposed cuts in v3b.",
        "# Candidate length is controlled by FEASIBILITY_CUT_MIN_CHAIN_SIZE.",
    )

    TARGET.write_text(text, encoding="utf-8")

    ok, err = compile_ok(TARGET)
    if not ok:
        shutil.copy2(backup, TARGET)
        fail(f"canonical naming patch did not compile; restored backup. Error: {err}")

    final = TARGET.read_text(encoding="utf-8")
    residuals = [old for old, _new in RENAMES if old in final]
    residual_tags = [old for old, _new in TAG_RENAMES if old in final]

    print(f"[{MARKER}] compile check passed", flush=True)
    print(f"[{MARKER}] total_identifier_or_tag_replacements={total_replacements}", flush=True)
    print(f"[{MARKER}] residual_old_identifiers={residuals}", flush=True)
    print(f"[{MARKER}] residual_old_output_tags={residual_tags}", flush=True)
    print(f"[{MARKER}] applied successfully", flush=True)
    print("Expected runtime summary tag after this patch: [FEAS-CUT-SUMMARY]", flush=True)


if __name__ == "__main__":
    main()
