"""Aggregated read-only service status endpoint."""
from __future__ import annotations

from flask import jsonify


def register_status_api(app, controller, camera, system_metrics, oled, route_tracker) -> None:
    @app.get("/api/status")
    def status():
        return jsonify(
            api_version="robot-console-2026-07-24-card-combined-v4",
            capabilities={
                "system_metrics": True,
                "highres_fps_control": hasattr(camera, "set_highres_fps"),
                "highres_fps_max": 30,
                "card_deal": True,
                "card_feed": True,
            },
            robot=controller.status(),
            camera=camera.status_dict(),
            system=system_metrics.status_dict(),
            oled=oled.status_dict() if oled else {
                "online": False, "disabled": True, "error": "已通过启动参数关闭"
            },
            autonomous=route_tracker.status_dict() if route_tracker else {
                "available": False, "enabled": False,
                "state": "disabled", "detail": "未通过启动参数开启"
            },
        )
