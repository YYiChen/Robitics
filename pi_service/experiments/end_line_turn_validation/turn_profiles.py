"""Compatibility import for the promoted formal turn-profile helpers."""
from __future__ import annotations

import sys
from pathlib import Path

ROBOT_WEB = Path(__file__).resolve().parents[2] / "robot_web"
if str(ROBOT_WEB) not in sys.path:
    sys.path.insert(0, str(ROBOT_WEB))

from routes.end_line.turn_profiles import *  # noqa: F401,F403,E402
