"""Static path/import validation for the Pi service; never opens hardware."""
from __future__ import annotations

import ast
import importlib
from pathlib import Path
import sys


SERVICE_ROOT = Path(__file__).resolve().parent
WEB_ROOT = SERVICE_ROOT / "robot_web"

REQUIRED_PATHS = (
    "AGENTS.md",
    "README.md",
    "config/defaults.json",
    "robot_web/app.py",
    "robot_web/api/camera_api.py",
    "robot_web/api/control_api.py",
    "robot_web/api/route_api.py",
    "robot_web/api/status_api.py",
    "robot_web/control/drive_config.py",
    "robot_web/control/motor_commands.py",
    "robot_web/control/protocol.py",
    "robot_web/control/card_control.py",
    "robot_web/control/servo_control.py",
    "robot_web/media/profiles.py",
    "robot_web/media/settings.py",
    "robot_web/media/metrics.py",
    "robot_web/media/color_correction.py",
    "robot_web/routes/common.py",
    "robot_web/routes/end_line/tracker.py",
    "robot_web/routes/scanline/models.py",
    "robot_web/routes/scanline/perception.py",
    "robot_web/routes/scanline/planner.py",
    "robot_web/routes/scanline/tracker.py",
    "robot_web/routes/generic/tracker.py",
    "robot_web/routes/pc_adaptor/tracker.py",
)

COMPATIBILITY_SHIMS = (
    "robot_web/autonomous_route.py",
    "robot_web/end_line_turn_adaptor.py",
    "robot_web/scanline_i_route.py",
    "robot_web/scanline_i_logic.py",
    "robot_web/green_white_scanline_i_route.py",
    "robot_web/four_endpoint_validation_route.py",
    "robot_web/pc_vision_adaptor_route.py",
)

PURE_IMPORTS = (
    "control.drive_config",
    "control.motor_commands",
    "control.protocol",
    "media.metrics",
    "media.profiles",
    "media.settings",
    "routes.common",
    "routes.end_line.turn_profiles",
    "routes.generic.marker_counter",
    "routes.generic.planner",
    "routes.pc_adaptor.protocol",
    "routes.scanline.config",
    "routes.scanline.control",
    "routes.scanline.logging",
    "routes.scanline.models",
    "routes.scanline.perception",
    "routes.scanline.planner",
)


def validate_paths() -> list[str]:
    errors: list[str] = []
    for relative in (*REQUIRED_PATHS, *COMPATIBILITY_SHIMS):
        if not (SERVICE_ROOT / relative).is_file():
            errors.append(f"missing required file: {relative}")

    for launcher in ("start_robot.sh", "start_webrtc.sh"):
        path = SERVICE_ROOT / launcher
        if not path.is_file():
            errors.append(f"missing launcher: {launcher}")
            continue
        source = path.read_text(encoding="utf-8")
        if "robot_web" not in source or "app.py" not in source:
            errors.append(f"launcher does not resolve robot_web/app.py: {launcher}")

    for path in WEB_ROOT.rglob("*.py"):
        relative = path.relative_to(SERVICE_ROOT).as_posix()
        source = path.read_text(encoding="utf-8")
        try:
            tree = ast.parse(source, filename=str(path))
            compile(tree, str(path), "exec")
        except SyntaxError as exc:
            errors.append(f"syntax error: {relative}: {exc}")
            continue
        if "/routes/" in f"/{relative}":
            for node in ast.walk(tree):
                if isinstance(node, ast.ImportFrom) and (
                    node.module or ""
                ).startswith("experiments"):
                    errors.append(f"formal route imports experiments: {relative}")
                if isinstance(node, ast.Import):
                    for alias in node.names:
                        if alias.name.startswith("experiments"):
                            errors.append(
                                f"formal route imports experiments: {relative}"
                            )

    inserted = False
    web_text = str(WEB_ROOT)
    if web_text not in sys.path:
        sys.path.insert(0, web_text)
        inserted = True
    try:
        for module_name in PURE_IMPORTS:
            try:
                importlib.import_module(module_name)
            except Exception as exc:
                errors.append(f"pure import failed: {module_name}: {exc}")
    finally:
        if inserted:
            sys.path.remove(web_text)
    return errors


def main() -> int:
    errors = validate_paths()
    if errors:
        for error in errors:
            print(f"FAIL: {error}")
        return 1
    print(
        "OK: pi_service module paths, compatibility shims, launchers, "
        "syntax, and pure imports are valid"
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
