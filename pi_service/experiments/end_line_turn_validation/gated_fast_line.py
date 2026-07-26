"""Compatibility import for the promoted formal green-gated line follower."""
from __future__ import annotations

import sys
from pathlib import Path

ROBOT_WEB = Path(__file__).resolve().parents[2] / "robot_web"
if str(ROBOT_WEB) not in sys.path:
    sys.path.insert(0, str(ROBOT_WEB))

from routes.end_line.line_following import *  # noqa: F401,F403,E402
