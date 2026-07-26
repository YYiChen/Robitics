"""Compatibility alias for the formal end-line route package."""
from __future__ import annotations

import sys

from routes.end_line import tracker as _tracker

# Preserve legacy imports and test patching of module-level profile paths.
sys.modules[__name__] = _tracker
