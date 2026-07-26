"""Compatibility import for the promoted formal end-line perception logic."""
from __future__ import annotations

import sys
from pathlib import Path

ROBOT_WEB = Path(__file__).resolve().parents[2] / "robot_web"
if str(ROBOT_WEB) not in sys.path:
    sys.path.insert(0, str(ROBOT_WEB))

from routes.end_line.perception import *  # noqa: F401,F403,E402
