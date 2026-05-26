"""Configuration flags and solver constants for the EV fragment model."""
from pathlib import Path

INSTANCE_DIR = Path.cwd() / "instances"

DEBUG = False
DEBUG_CALLBACK = False
DEBUG_DIAGNOSTICS = True

FORCE_CHAIN = False
RUN_OPTIONA = True
DIAG_SKELETON = False

DP_MAX_STATION_VISITS_PER_LEG = 6
DP_MAX_LABELS_PER_NODE = 500
