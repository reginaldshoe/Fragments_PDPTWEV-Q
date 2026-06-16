from __future__ import annotations

import argparse
import py_compile
from pathlib import Path

RUN_SOLVER_PATH = Path("experiments/run_solver.py")
MASTER_CORE_PATH = Path("evrp_fragments/master_core.py")
START_MARKER = "# --- v5k queueing-enabled master objective plumbing ---"
END_MARKER = "# --- end v5k queueing-enabled master objective plumbing ---"
MASTER_BLOCK = '\n# --- v5k queueing-enabled master objective plumbing ---\n# Default behaviour is unchanged. If EVRP_QUEUEING_ENABLED=1, the legacy theta\n# remains the energy-DP theta and a parallel non-negative theta_queue variable is\n# added to the objective with coefficient EVRP_QUEUE_WEIGHT.\n\nif "_build_master_model_v5k_base" not in globals():\n    _build_master_model_v5k_base = build_master_model\n\n\ndef _v5k_env_bool(name: str, default: bool = False) -> bool:\n    import os\n    raw = os.environ.get(name)\n    if raw is None:\n        return default\n    return str(raw).strip().lower() in {"1", "true", "yes", "on"}\n\n\ndef _v5k_env_float(name: str, default: float) -> float:\n    import os\n    raw = os.environ.get(name)\n    if raw is None or str(raw).strip() == "":\n        return float(default)\n    return float(raw)\n\n\ndef build_master_model(*args, **kwargs):\n    """Build the master model, optionally adding queueing objective plumbing."""\n    result = _build_master_model_v5k_base(*args, **kwargs)\n    queueing_enabled = _v5k_env_bool("EVRP_QUEUEING_ENABLED", False)\n    queue_weight = _v5k_env_float("EVRP_QUEUE_WEIGHT", 1.0)\n\n    try:\n        model = result[0]\n        theta_energy = result[6]\n    except Exception:\n        if queueing_enabled:\n            raise RuntimeError("v5k expected legacy build_master_model to return tuple with model and theta")\n        return result\n\n    if not queueing_enabled:\n        model._queueing_enabled = False\n        model._theta_energy = theta_energy\n        model._theta_queue = None\n        model._queue_weight = 0.0\n        return result\n\n    if not isinstance(result, tuple) or len(result) < 8:\n        raise RuntimeError("v5k expected legacy build_master_model to return at least 8 tuple entries")\n\n    try:\n        import gurobipy as gp_local\n    except ImportError as exc:  # pragma: no cover\n        raise RuntimeError("v5k queueing-enabled master objective requires gurobipy") from exc\n\n    theta_queue = model.addVar(lb=0.0, name="theta_queue")\n    model.update()\n    model.setObjective(model.getObjective() + float(queue_weight) * theta_queue, gp_local.GRB.MINIMIZE)\n    model.update()\n\n    model._queueing_enabled = True\n    model._theta_energy = theta_energy\n    model._theta_queue = theta_queue\n    model._queue_weight = float(queue_weight)\n\n    print(\n        "[QUEUEING-MASTER-CONFIG]",\n        "queueing_enabled=", True,\n        "queue_weight=", float(queue_weight),\n        "theta_energy_name=", getattr(theta_energy, "VarName", "theta"),\n        "theta_queue_name=", getattr(theta_queue, "VarName", "theta_queue"),\n    )\n    return result\n\n\ndef get_queueing_master_metadata(model) -> dict:\n    """Return queueing master metadata for optional runner/reporting use."""\n    theta_energy = getattr(model, "_theta_energy", None)\n    theta_queue = getattr(model, "_theta_queue", None)\n\n    def _value(var):\n        if var is None:\n            return None\n        for attr in ("X", "x"):\n            if hasattr(var, attr):\n                try:\n                    return float(getattr(var, attr))\n                except Exception:\n                    return None\n        return None\n\n    return {\n        "queueing_enabled": bool(getattr(model, "_queueing_enabled", False)),\n        "queue_weight": float(getattr(model, "_queue_weight", 0.0) or 0.0),\n        "theta_energy": _value(theta_energy),\n        "theta_queue": _value(theta_queue),\n    }\n# --- end v5k queueing-enabled master objective plumbing ---\n'
PARSER_INSERTION = '    parser.add_argument("--queueing-enabled", action="store_true", help="Enable queueing objective plumbing by adding theta_queue in the master.")\n    parser.add_argument("--queue-weight", type=float, default=1.0, help="Objective coefficient for theta_queue when --queueing-enabled is used.")\n'
ENV_BLOCK = '    # --- v5k queueing-enabled env bridge ---\n    if getattr(args, "queueing_enabled", False):\n        os.environ["EVRP_QUEUEING_ENABLED"] = "1"\n        os.environ["EVRP_QUEUE_WEIGHT"] = str(float(getattr(args, "queue_weight", 1.0)))\n    else:\n        os.environ.pop("EVRP_QUEUEING_ENABLED", None)\n        os.environ.pop("EVRP_QUEUE_WEIGHT", None)\n    # --- end v5k queueing-enabled env bridge ---\n'
NL = chr(10)

def _replace_or_append_master_block(text: str) -> str:
    if START_MARKER in text and END_MARKER in text:
        start = text.index(START_MARKER)
        end = text.index(END_MARKER) + len(END_MARKER)
        return text[:start].rstrip() + NL + NL + MASTER_BLOCK.strip() + NL + text[end:].lstrip(NL)
    return text.rstrip() + NL + NL + MASTER_BLOCK.strip() + NL

def _ensure_os_import(text: str) -> str:
    if 'import os' in text:
        return text
    lines = text.splitlines()
    insert_at = 0
    for idx, line in enumerate(lines):
        stripped = line.strip()
        if stripped.startswith('import ') or stripped.startswith('from '):
            insert_at = idx + 1
        elif stripped and not stripped.startswith('#'):
            break
    lines.insert(insert_at, 'import os')
    return NL.join(lines) + NL

def _patch_run_solver_parser(text: str) -> str:
    if '--queueing-enabled' in text:
        return text
    anchors = ['    parser.add_argument("--use-callback"', "    parser.add_argument('--use-callback'"]
    for anchor in anchors:
        idx = text.find(anchor)
        if idx != -1:
            line_end = text.find(NL, idx)
            if line_end == -1:
                line_end = len(text)
            return text[:line_end + 1] + PARSER_INSERTION + text[line_end + 1:]
    parse_idx = text.find('args = parser.parse_args()')
    if parse_idx == -1:
        raise RuntimeError('Could not find parser insertion point in experiments/run_solver.py')
    return text[:parse_idx] + PARSER_INSERTION + text[parse_idx:]

def _patch_run_solver_env(text: str) -> str:
    marker = '# --- v5k queueing-enabled env bridge ---'
    if marker in text:
        return text
    parse_stmt = 'args = parser.parse_args()'
    idx = text.find(parse_stmt)
    if idx == -1:
        raise RuntimeError('Could not find args = parser.parse_args() in experiments/run_solver.py')
    line_end = text.find(NL, idx)
    if line_end == -1:
        line_end = len(text)
    return text[:line_end + 1] + ENV_BLOCK + text[line_end + 1:]

def main() -> None:
    parser = argparse.ArgumentParser(description='Apply v5k queueing-enabled master objective plumbing overlay.')
    parser.add_argument('--audit-only', action='store_true', help='Report intended changes without writing files.')
    parser.add_argument('--compile', action='store_true', help='Compile patched files after writing them.')
    args = parser.parse_args()

    print('[v5k] target:', MASTER_CORE_PATH)
    print('[v5k] target:', RUN_SOLVER_PATH)
    print('[v5k] action: add --queueing-enabled flag and optional theta_queue master plumbing')
    print('[v5k] legacy theta/energy-DP callback behaviour is preserved by default')

    if args.audit_only:
        print('[v5k] audit-only: no files written')
        return

    master_text = MASTER_CORE_PATH.read_text(encoding='utf-8')
    master_text = _replace_or_append_master_block(master_text)
    MASTER_CORE_PATH.write_text(master_text, encoding='utf-8')
    print('[v5k] wrote', MASTER_CORE_PATH)

    run_text = RUN_SOLVER_PATH.read_text(encoding='utf-8')
    run_text = _ensure_os_import(run_text)
    run_text = _patch_run_solver_parser(run_text)
    run_text = _patch_run_solver_env(run_text)
    RUN_SOLVER_PATH.write_text(run_text, encoding='utf-8')
    print('[v5k] wrote', RUN_SOLVER_PATH)

    if args.compile:
        py_compile.compile(str(MASTER_CORE_PATH), doraise=True)
        py_compile.compile(str(RUN_SOLVER_PATH), doraise=True)
        print('[v5k] py_compile ok')

if __name__ == '__main__':
    main()
