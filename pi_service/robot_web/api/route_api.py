"""Route preview, remote-vision, tuning, and turn endpoints."""
from __future__ import annotations

from flask import Response, jsonify, request

from routes.end_line.tracker import EndLineTurnAdaptorRouteTracker
from routes.pc_adaptor.tracker import PcVisionAdaptorRouteTracker


def register_route_api(app, route_preview, route_tracker) -> None:
    @app.get("/route_preview_feed")
    def route_preview_feed():
        if route_preview is None:
            return jsonify(error="路线预判未通过启动参数开启"), 404
        return Response(route_preview.iter_mjpeg(), mimetype="multipart/x-mixed-replace; boundary=frame")

    @app.get("/api/vision-adaptor/frame")
    def vision_adaptor_frame():
        if not isinstance(route_tracker, PcVisionAdaptorRouteTracker):
            return jsonify(ok=False, error="当前 route mode 不是 pc_vision_adaptor"), 409
        jpeg, sequence, captured_at_ms, fast_center, fast_confidence, fast_centers = route_tracker.frame_snapshot()
        if jpeg is None:
            return jsonify(ok=False, error="Pi 尚未发布 adaptor 帧"), 503
        fast_rows = ";".join(f"{y},{x:.1f},{width}" for y, x, width in fast_centers)
        return Response(jpeg, mimetype="image/jpeg", headers={
            "Cache-Control": "no-store, max-age=0",
            "X-Vision-Frame-Seq": str(sequence),
            "X-Vision-Captured-At-Ms": str(captured_at_ms),
            "X-Vision-Pi-Fast-Centre": "" if fast_center is None else f"{fast_center:.1f}",
            "X-Vision-Pi-Fast-Confidence": f"{fast_confidence:.2f}",
            "X-Vision-Pi-Fast-Centers": fast_rows,
        })

    @app.post("/api/vision-adaptor/event")
    def vision_adaptor_event():
        if not isinstance(route_tracker, PcVisionAdaptorRouteTracker):
            return jsonify(ok=False, error="当前 route mode 不是 pc_vision_adaptor"), 409
        try:
            return jsonify(ok=True, **route_tracker.submit_remote_event(request.get_json(silent=True) or {}))
        except ValueError as exc:
            return jsonify(ok=False, error=str(exc)), 400

    @app.post("/api/vision-adaptor/preview")
    def vision_adaptor_preview():
        if not isinstance(route_tracker, PcVisionAdaptorRouteTracker):
            return jsonify(ok=False, error="当前 route mode 不是 pc_vision_adaptor"), 409
        try:
            return jsonify(ok=True, **route_tracker.submit_remote_preview(request.get_data(cache=False), request.headers))
        except ValueError as exc:
            return jsonify(ok=False, error=str(exc)), 400

    @app.post("/api/autonomous/toggle")
    def autonomous_toggle():
        if route_tracker is None:
            return jsonify(ok=False, error="路线预判未通过启动参数开启"), 409
        return jsonify(ok=True, autonomous=route_tracker.toggle_drive())

    @app.post("/api/autonomous/tuning")
    def autonomous_tuning():
        if route_tracker is None:
            return jsonify(ok=False, error="路线预判未通过启动参数开启"), 409
        try:
            return jsonify(ok=True, autonomous=route_tracker.update_tuning(request.get_json(silent=True) or {}))
        except ValueError as exc:
            return jsonify(ok=False, error=str(exc)), 400

    @app.post("/api/autonomous/manual-turn")
    def autonomous_manual_turn():
        if not isinstance(route_tracker, EndLineTurnAdaptorRouteTracker):
            return jsonify(ok=False, error="当前路线模式不支持 Q/E/U/I 视觉转向"), 409
        try:
            return jsonify(ok=True, autonomous=route_tracker.request_manual_turn((request.get_json(silent=True) or {}).get("command", "")))
        except ValueError as exc:
            return jsonify(ok=False, error=str(exc)), 400

    @app.post("/api/autonomous/face-turn")
    def autonomous_face_turn():
        if not isinstance(route_tracker, EndLineTurnAdaptorRouteTracker):
            return jsonify(ok=False, error="当前路线模式不支持电脑人脸居中转向"), 409
        try:
            return jsonify(ok=True, autonomous=route_tracker.request_face_center_turn((request.get_json(silent=True) or {}).get("command", "")))
        except ValueError as exc:
            return jsonify(ok=False, error=str(exc)), 400

    @app.post("/api/autonomous/line-turn")
    def autonomous_line_turn():
        if not isinstance(route_tracker, EndLineTurnAdaptorRouteTracker):
            return jsonify(ok=False, error="当前路线模式不支持白线居中转向"), 409
        try:
            return jsonify(ok=True, autonomous=route_tracker.request_line_center_turn((request.get_json(silent=True) or {}).get("command", "")))
        except ValueError as exc:
            return jsonify(ok=False, error=str(exc)), 400

    @app.post("/api/autonomous/roundtrip/start")
    def autonomous_roundtrip_start():
        if not isinstance(route_tracker, EndLineTurnAdaptorRouteTracker):
            return jsonify(ok=False, error="当前路线模式不支持双人脸往返序列"), 409
        try:
            return jsonify(ok=True, autonomous=route_tracker.request_roundtrip_start((request.get_json(silent=True) or {}).get("sweep_side", "")))
        except ValueError as exc:
            return jsonify(ok=False, error=str(exc)), 400

    @app.post("/api/autonomous/roundtrip/return")
    def autonomous_roundtrip_return():
        if not isinstance(route_tracker, EndLineTurnAdaptorRouteTracker):
            return jsonify(ok=False, error="当前路线模式不支持双人脸往返序列"), 409
        try:
            return jsonify(ok=True, autonomous=route_tracker.request_roundtrip_return())
        except ValueError as exc:
            return jsonify(ok=False, error=str(exc)), 400

    @app.post("/api/autonomous/roundtrip/stop")
    def autonomous_roundtrip_stop():
        if not isinstance(route_tracker, EndLineTurnAdaptorRouteTracker):
            return jsonify(ok=False, error="当前路线模式不支持双人脸往返序列"), 409
        return jsonify(ok=True, autonomous=route_tracker.request_roundtrip_stop())

    @app.post("/api/autonomous/follow-to-end")
    def autonomous_follow_to_end():
        if not isinstance(route_tracker, EndLineTurnAdaptorRouteTracker):
            return jsonify(ok=False, error="当前路线模式不支持 N 自动巡线"), 409
        try:
            return jsonify(ok=True, autonomous=route_tracker.request_follow_to_end())
        except ValueError as exc:
            return jsonify(ok=False, error=str(exc)), 400

    @app.post("/api/autonomous/return")
    def autonomous_return():
        if not isinstance(route_tracker, EndLineTurnAdaptorRouteTracker):
            return jsonify(ok=False, error="当前路线模式不支持分段视觉返程"), 409
        try:
            return jsonify(ok=True, autonomous=route_tracker.request_return())
        except ValueError as exc:
            return jsonify(ok=False, error=str(exc)), 400
