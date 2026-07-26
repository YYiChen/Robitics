"""Drive, servo, reconnect, and card-motor HTTP endpoints."""
from __future__ import annotations

from flask import jsonify, request


FROZEN_CARD_FEED_PWM = -150
FROZEN_CARD_FEED_DURATION_MS = 1500
FROZEN_CARD_DEAL_PWM = 150
FROZEN_CARD_DEAL_DURATION_MS = 400


def _frozen_deal_request(raw_request):
    if raw_request is None or not isinstance(raw_request, dict):
        return raw_request
    return {
        "token": raw_request.get("token"),
        "feed_pwm": FROZEN_CARD_FEED_PWM,
        "feed_duration_ms": FROZEN_CARD_FEED_DURATION_MS,
        "deal_pwm": FROZEN_CARD_DEAL_PWM,
        "deal_duration_ms": FROZEN_CARD_DEAL_DURATION_MS,
    }


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
                deal=controller.deal_from_key_request(
                    _frozen_deal_request(payload.get("deal_request"))
                ),
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
        try:
            return jsonify(
                ok=True,
                state=controller.deal_card(
                    FROZEN_CARD_DEAL_PWM, FROZEN_CARD_DEAL_DURATION_MS
                ),
            )
        except ValueError as exc:
            return jsonify(ok=False, error=str(exc)), 400
        except RuntimeError as exc:
            return jsonify(ok=False, error=str(exc)), 503

    @app.post("/api/feed")
    def feed():
        try:
            return jsonify(
                ok=True,
                state=controller.feed_cards(
                    FROZEN_CARD_FEED_PWM, FROZEN_CARD_FEED_DURATION_MS
                ),
            )
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
