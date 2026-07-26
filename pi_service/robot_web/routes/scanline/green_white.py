"""Port-5000 adapter for green-floor / white-tape I-shape turnaround."""
from __future__ import annotations

from dataclasses import replace

from .green_white_perception import GreenWhiteHybridScanlineAnalyzer
from .tracker import ScanlineIShapeRouteTracker


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
