"""Camera feeds, profiles, exposure, and image-processing endpoints."""
from __future__ import annotations

from flask import Response, jsonify, render_template, request


def register_camera_api(app, camera) -> None:
    def mjpeg_only():
        if camera.transport == "mjpeg":
            return None
        return jsonify(ok=False, error="当前为 H.264/WebRTC 模式；请使用 start_robot.sh 启动 MJPEG 后再调整该项"), 409

    def highres_available():
        if getattr(camera, "highres_available", False) or camera.transport == "mjpeg":
            return None
        return jsonify(ok=False, error="当前视频模式未启用高清 JPEG 通道"), 409

    @app.get("/")
    def index():
        return render_template("index.html")

    @app.get("/video_feed")
    def video_feed():
        unavailable = mjpeg_only()
        if unavailable is not None:
            return unavailable
        if not camera.online:
            return jsonify(error=camera.error or camera.status), 503
        return Response(camera.iter_mjpeg(), mimetype="multipart/x-mixed-replace; boundary=frame")

    @app.get("/highres_feed")
    def highres_feed():
        unavailable = highres_available()
        if unavailable is not None:
            return unavailable
        if not camera.online:
            return jsonify(error=camera.error or camera.status), 503
        return Response(camera.iter_highres_mjpeg(), mimetype="multipart/x-mixed-replace; boundary=frame")

    @app.get("/api/camera/highres/latest")
    def latest_highres_image():
        unavailable = highres_available()
        if unavailable is not None:
            return unavailable
        if not camera.online:
            return jsonify(error=camera.error or camera.status), 503
        jpeg = camera.latest_highres_jpeg()
        if jpeg is None:
            return jsonify(error="高清图片尚未生成"), 503
        return Response(jpeg, mimetype="image/jpeg", headers={"Cache-Control": "no-store, max-age=0"})

    @app.post("/api/camera/mode")
    def camera_mode():
        unavailable = mjpeg_only()
        if unavailable is not None:
            return unavailable
        payload = request.get_json(silent=True) or {}
        try:
            return jsonify(ok=True, camera=camera.set_mode(str(payload.get("mode", ""))))
        except ValueError as exc:
            return jsonify(ok=False, error=str(exc)), 400
        except RuntimeError as exc:
            return jsonify(ok=False, error=str(exc), camera=camera.status_dict()), 503

    @app.post("/api/camera/exposure")
    def camera_exposure():
        unavailable = mjpeg_only()
        if unavailable is not None:
            return unavailable
        try:
            return jsonify(ok=True, camera=camera.set_exposure(request.get_json(silent=True) or {}))
        except ValueError as exc:
            return jsonify(ok=False, error=str(exc)), 400
        except RuntimeError as exc:
            return jsonify(ok=False, error=str(exc), camera=camera.status_dict()), 503

    @app.post("/api/camera/stream-profile")
    def camera_stream_profile():
        unavailable = mjpeg_only()
        if unavailable is not None:
            return unavailable
        payload = request.get_json(silent=True) or {}
        try:
            return jsonify(ok=True, camera=camera.set_stream_profile(str(payload.get("profile", ""))))
        except ValueError as exc:
            return jsonify(ok=False, error=str(exc)), 400

    @app.post("/api/camera/highres-profile")
    def camera_highres_profile():
        unavailable = highres_available()
        if unavailable is not None:
            return unavailable
        payload = request.get_json(silent=True) or {}
        try:
            return jsonify(ok=True, camera=camera.set_highres_profile(str(payload.get("profile", ""))))
        except ValueError as exc:
            return jsonify(ok=False, error=str(exc)), 400

    @app.post("/api/camera/highres-fps")
    def camera_highres_fps():
        unavailable = highres_available()
        if unavailable is not None:
            return unavailable
        if not hasattr(camera, "set_highres_fps"):
            return jsonify(ok=False, error="当前相机后端不支持动态调整高清图片帧率"), 409
        payload = request.get_json(silent=True) or {}
        try:
            return jsonify(ok=True, camera=camera.set_highres_fps(payload.get("fps")))
        except ValueError as exc:
            return jsonify(ok=False, error=str(exc)), 400

    @app.post("/api/camera/color-correction")
    def camera_color_correction():
        unavailable = mjpeg_only()
        if unavailable is not None:
            return unavailable
        try:
            return jsonify(ok=True, camera=camera.set_color_correction(request.get_json(silent=True) or {}))
        except ValueError as exc:
            return jsonify(ok=False, error=str(exc)), 400
