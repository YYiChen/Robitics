"""Stable semantic HTTP facade for the DeskMate state machine."""

from __future__ import annotations

from flask import jsonify, request

from control.robotics_gateway import RoboticsGateway


def register_robotics_api(app, controller, route_tracker) -> None:
    gateway = RoboticsGateway(controller, route_tracker)
    app.extensions["robotics_gateway"] = gateway

    @app.get("/api/robotics/v1/capabilities")
    def robotics_capabilities():
        return jsonify(ok=True, capabilities=gateway.capabilities())

    @app.get("/api/robotics/v1/status")
    def robotics_status():
        return jsonify(ok=True, status=gateway.status())

    @app.post("/api/robotics/v1/gate")
    def robotics_gate():
        try:
            return jsonify(ok=True, gate=gateway.set_gate(request.get_json(silent=True) or {}))
        except ValueError as exc:
            return jsonify(ok=False, error=str(exc)), 400
        except RuntimeError as exc:
            return jsonify(ok=False, error=str(exc)), 409

    @app.get("/api/robotics/v1/config")
    def robotics_config():
        try:
            return jsonify(ok=True, config=gateway.config())
        except RuntimeError as exc:
            return jsonify(ok=False, error=str(exc)), 409

    @app.post("/api/robotics/v1/config")
    def robotics_config_update():
        try:
            return jsonify(
                ok=True,
                config=gateway.update_config(request.get_json(silent=True) or {}),
            )
        except ValueError as exc:
            return jsonify(ok=False, error=str(exc)), 400
        except RuntimeError as exc:
            return jsonify(ok=False, error=str(exc)), 409

    @app.post("/api/robotics/v1/actions")
    def robotics_action():
        try:
            return jsonify(
                ok=True,
                result=gateway.execute(request.get_json(silent=True) or {}),
            )
        except ValueError as exc:
            return jsonify(ok=False, error=str(exc)), 400
        except RuntimeError as exc:
            return jsonify(ok=False, error=str(exc)), 409

    @app.get("/api/robotics/v1/requests/<request_id>")
    def robotics_request_result(request_id: str):
        try:
            result = gateway.request_result(request_id)
        except ValueError as exc:
            return jsonify(ok=False, error=str(exc)), 400
        if result is None:
            return jsonify(
                ok=False,
                error="request_id is not known by this service process",
                request_id=request_id,
            ), 404
        return jsonify(ok=True, result=result)


__all__ = ["register_robotics_api"]
