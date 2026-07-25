"""Port-5000 adapter for green-floor / white-tape I-shape turnaround."""
from __future__ import annotations

from pathlib import Path
import sys
from dataclasses import replace

from scanline_i_route import ScanlineIShapeRouteTracker


GREEN_WHITE_EXPERIMENT = Path(__file__).resolve().parents[1] / "experiments" / "i_shape_green_white_turnaround_validation"
if str(GREEN_WHITE_EXPERIMENT) not in sys.path:
    sys.path.insert(0, str(GREEN_WHITE_EXPERIMENT))

from green_white_scanline_i_logic import GreenWhiteHybridScanlineAnalyzer  # noqa: E402


class GreenWhiteScanlineIShapeRouteTracker(ScanlineIShapeRouteTracker):
    """The existing I-turn controls with HSV green-floor/white-tape evidence."""
    route_mode = "scanline_i_green_white"
    route_variant = "hybrid_green_white"
    route_ready_detail = "绿地白线 Hybrid 扫描线 I 型识别运行中；按 M 开启自动行驶"

    def _create_analyzer(self, config):
        return GreenWhiteHybridScanlineAnalyzer()

    @staticmethod
    def _planner_config(config):
        """Enable the red-bottom exit only for this calibrated green course."""
        return replace(
            ScanlineIShapeRouteTracker._planner_config(config),
            red_exit_enabled=True,
        )
