"""Compatibility alias for the formal scanline route package."""
from __future__ import annotations

import sys

from routes.scanline import tracker as _tracker

sys.modules[__name__] = _tracker
