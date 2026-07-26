"""Versioned JSONL audit logging for the formal scanline route."""
from __future__ import annotations

from dataclasses import fields
from datetime import datetime, timezone
import json
import os
from pathlib import Path
import time
from uuid import uuid4

from .config import ScanlineIRouteConfig


RUN_LOG_SCHEMA_VERSION = 1


class ScanlineLoggingMixin:
    @staticmethod
    def _config_snapshot(config: ScanlineIRouteConfig) -> dict[str, object]:
        return {
            field.name: str(value) if isinstance(value := getattr(config, field.name), Path) else value
            for field in fields(config)
        }

    def _base_log_record(
        self, event: str, frame_id: int | None, now: float | None = None
    ) -> dict[str, object]:
        return {
            "schema_version": RUN_LOG_SCHEMA_VERSION,
            "event": event,
            "run_id": self._run_id,
            "drive_session_id": self._drive_session_id,
            "time_utc": datetime.now(timezone.utc).isoformat(),
            "monotonic_seconds": time.monotonic() if now is None else now,
            "frame_id": frame_id,
            "config_revision": self._config_revision,
        }

    def _write_event(
        self, event: str, *, frame_id: int | None = None,
        now: float | None = None, **payload: object,
    ) -> None:
        record = self._base_log_record(event, frame_id, now)
        record.update(payload)
        with self._log_lock:
            handle = self._run_log
            if handle is None:
                return
            handle.write(json.dumps(record, ensure_ascii=False) + "\n")
            handle.flush()

    def _open_run_log(
        self, config: ScanlineIRouteConfig, frame_shape: tuple[int, ...]
    ) -> None:
        if not config.tuning_path:
            return
        log_dir = config.tuning_path.parent / "runtime_logs"
        log_dir.mkdir(parents=True, exist_ok=True)
        stamp = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ")
        self._run_id = f"{stamp}-{uuid4().hex[:8]}"
        handle = (
            log_dir / f"{self.route_mode}_{self._run_id}.jsonl"
        ).open("a", encoding="utf-8")
        self._session_started = True
        self._session_ended = False
        height, width = frame_shape[:2]
        start_record = self._base_log_record("session_start", 0)
        start_record.update({
            "route_mode": self.route_mode,
            "route_variant": self.route_variant if config.use_hybrid else "legacy",
            "software_version": os.environ.get("ROBOT_GIT_COMMIT", "unknown"),
            "frame_width_px": width,
            "frame_height_px": height,
            "config": self._config_snapshot(config),
        })
        with self._log_lock:
            handle.write(json.dumps(start_record, ensure_ascii=False) + "\n")
            handle.flush()
            self._run_log = handle

    @staticmethod
    def _reason_code(reason: str) -> str:
        exact = {
            "lower_transverse_bar_marked_follow_until_stem_lost": "WHITE_BAR_CONFIRMED",
            "early_prediction_confirmed_by_bar_rows_endpoint": "WHITE_BAR_CONFIRMED",
            "confirmed_white_bar_red_marker_exited_bottom_braking": "RED_EXIT_CONFIRMED",
            "longitudinal_stem_lost_after_bar_fast_braking": "LINE_LOST_CONFIRMED_FAST",
            "longitudinal_stem_lost_after_bar_braking": "LINE_LOST_CONFIRMED",
            "stem_lost_from_early_prediction_braking": "LINE_LOST_CONFIRMED",
            "brake_complete_starting_right_pivot": "PIVOT_STARTED",
            "longitudinal_line_reacquired": "REACQUIRE_CONFIRMED",
            "pivot_timeout_without_longitudinal_reacquire": "PIVOT_LIMIT_REACHED",
            "bar_mark_timeout_returning_to_follow": "BAR_TIMEOUT",
            "early_prediction_false_alarm_junction_lost": "EARLY_PREDICTION_CLEARED",
        }
        if reason in exact:
            return exact[reason]
        if reason.startswith("junction_at_y="):
            return "EARLY_BAR_PREDICTED"
        return "STATE_PROGRESS"

    def _write_run_log(
        self, evidence, decision, planner, frame_index: int, now: float,
        motor_text: str, commanded: tuple[int, int] | None,
        control: dict[str, object], frame_shape: tuple[int, ...],
    ) -> None:
        if self._run_log is None:
            return
        controller_status = self.controller.status() if hasattr(self.controller, "status") else {}
        height, width = frame_shape[:2]
        scanlines = [
            {
                "y_px": int(y), "row_ratio": float(y) / max(1, height),
                "detected": True, "center_x_px": x, "width_px": line_width,
            }
            for y, x, line_width in evidence.line_centers
        ]
        counters = {
            "endpoint_frames": decision.endpoint_frames,
            "line_lost_frames": decision.line_lost_frames,
            "reacquire_frames": decision.reacquire_frames,
            "junction_frames": decision.junction_frames,
            "pivot_elapsed_seconds": decision.pivot_elapsed_seconds,
        }
        diagnostics = planner.diagnostics()
        record = {
            "frame_width_px": width,
            "frame_height_px": height,
            "vision": {
                "confidence": evidence.confidence,
                "valid_line": evidence.valid_line,
                "line_lost": evidence.line_lost,
                "line_center_x_px": evidence.line_center_x,
                "valid_bands": len(evidence.line_centers),
                "scanlines": scanlines,
                "endpoint": {"detected": evidence.endpoint_detected, "y_px": evidence.endpoint_y, "width_px": evidence.endpoint_width},
                "junction": {"detected": evidence.junction_detected, "y_px": evidence.junction_y, "arms": evidence.junction_arm_count},
                "red_band": {"detected": evidence.red_marker_detected, "y_px": evidence.red_marker_y, "span_px": evidence.red_marker_span},
                "lookahead": {"x_px": evidence.lookahead_x, "y_px": evidence.lookahead_y, "path_length_px": evidence.path_length_px},
            },
            "planner": {
                "state": decision.state.value,
                "reason_code": self._reason_code(decision.reason),
                "reason_detail": decision.reason,
                "counters": counters,
                "diagnostics": diagnostics,
            },
            "control": control,
            "commanded_pwm": None if commanded is None else {"right": commanded[0], "left": commanded[1]},
            "motor_text": motor_text,
            "controller": {
                "motor_output": controller_status.get("motor_output"),
                "serial_open": controller_status.get("serial"),
                "arduino_online": controller_status.get("arduino_online"),
                "last_rx_age_seconds": controller_status.get("last_rx_age"),
                "error": controller_status.get("error"),
            },
        }
        self._write_event("frame_observation", frame_id=frame_index, now=now, **record)
        current_state = decision.state.value
        if (
            self._previous_logged_state is not None
            and current_state != self._previous_logged_state
        ):
            self._write_event(
                "state_transition",
                frame_id=frame_index,
                now=now,
                from_state=self._previous_logged_state,
                to_state=current_state,
                reason_code=self._reason_code(decision.reason),
                reason_detail=decision.reason,
                evidence_summary={
                    "valid_bands": len(evidence.line_centers),
                    "line_lost": evidence.line_lost,
                    "endpoint_detected": evidence.endpoint_detected,
                    "red_marker_detected": evidence.red_marker_detected,
                    "red_exit_armed": diagnostics["red_exit_armed"],
                },
                counters=counters,
            )
        self._previous_logged_state = current_state

    def _close_run_log(self, *, outcome: str, reason_code: str) -> None:
        end_record = self._base_log_record("session_end", self._last_frame_index)
        end_record.update({"outcome": outcome, "reason_code": reason_code})
        with self._log_lock:
            handle = self._run_log
            if handle is None:
                return
            if self._session_started and not self._session_ended:
                handle.write(json.dumps(end_record, ensure_ascii=False) + "\n")
                handle.flush()
                self._session_ended = True
            handle.close()
            self._run_log = None
