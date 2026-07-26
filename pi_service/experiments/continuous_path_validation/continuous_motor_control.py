"""Compatibility import for promoted generic-route control."""
import sys
from pathlib import Path

ROBOT_WEB = Path(__file__).resolve().parents[2] / "robot_web"
if str(ROBOT_WEB) not in sys.path:
    sys.path.insert(0, str(ROBOT_WEB))

from routes.generic.control import *  # noqa: F401,F403
