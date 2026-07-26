"""Drive, servo, reconnect, and card-motor HTTP endpoints."""
from __future__ import annotations

from flask import jsonify, request


def register_control_api(app, controller) -> None:
    @app.post("/api/action")
    def action():
        try:
            return jsonify(ok=True, action=controller.select_action((request.get_json(silent=True) or {}).get("action", "STOP")))
        except ValueError as exc:
            return jsonify(ok=False, error=str(exc)), 400

    @app.post("/api/drive")
    def drive():
        payload = request.get_json(silent=True) or {}
        try:
            right, left = controller.set_direct_drive(payload.get("right_pwm"), payload.get("left_pwm"))
            return jsonify(ok=True, right_pwm=right, left_pwm=left)
        except ValueError as exc:
            return jsonify(ok=False, error=str(exc)), 400

    @app.post("/api/keys")
    def keys():
        payload = request.get_json(silent=True) or {}
        try:
            return jsonify(
                ok=True,
                action=controller.update_keys(payload),
                deal=controller.deal_from_key_request(payload.get("deal_request")),
            )
        except ValueError as exc:
            return jsonify(ok=False, error=str(exc)), 400
        except RuntimeError as exc:
            return jsonify(ok=False, error=str(exc)), 503

    @app.post("/api/heartbeat")
    def heartbeat():
        controller.heartbeat()
        return jsonify(ok=True)

    @app.post("/api/stop")
    def stop():
        controller.stop_now()
        return jsonify(ok=True)

    @app.post("/api/deal")
    def deal():
        payload = request.get_json(silent=True) or {}
        try:
            return jsonify(ok=True, state=controller.deal_card(payload.get("pwm", 255), payload.get("duration_ms", 1000)))
        except ValueError as exc:
            return jsonify(ok=False, error=str(exc)), 400
        except RuntimeError as exc:
            return jsonify(ok=False, error=str(exc)), 503

    @app.post("/api/feed")
    def feed():
        payload = request.get_json(silent=True) or {}
        try:
            return jsonify(ok=True, state=controller.feed_cards(payload.get("pwm", -255), payload.get("duration_ms", 5000)))
        except ValueError as exc:
            return jsonify(ok=False, error=str(exc)), 400
        except RuntimeError as exc:
            return jsonify(ok=False, error=str(exc)), 503

    @app.post("/api/servo")
    def servo():
        payload = request.get_json(silent=True) or {}
        try:
            return jsonify(ok=True, requested_angle=controller.set_servo_angle(payload.get("angle"), fast=bool(payload.get("fast", False))))
        except ValueError as exc:
            return jsonify(ok=False, error=str(exc)), 400
        except RuntimeError as exc:
            return jsonify(ok=False, error=str(exc)), 503

    @app.post("/api/reconnect")
    def reconnect():
        status = controller.reconnect()
        return jsonify(ok=status["arduino_online"], robot=status)

    @app.post("/api/config")
    def config():
        return jsonify(ok=True, config=controller.update_config(request.get_json(silent=True) or {}))
