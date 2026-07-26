"""Steering-servo command and heartbeat synchronization."""
from __future__ import annotations


class ServoControlMixin:
    def set_servo_angle(self, raw_angle: object, *, fast: bool = False, track_target: bool = True) -> int:
        try:
            angle = int(raw_angle)
        except (TypeError, ValueError):
            raise ValueError("舵机角度必须是 0 到 180 的整数") from None
        if not 0 <= angle <= 180:
            raise ValueError("舵机角度必须在 0 到 180 之间")
        self._write(f"SVF,{angle}" if fast else f"SV,{angle}")
        if self.error:
            raise RuntimeError(f"发送舵机命令失败：{self.error}")
        with self.lock:
            self.servo_angle = angle
            if track_target:
                self._servo_target_angle = float(angle)
        return angle

    def _sync_steering(self, now: float, config) -> None:
        with self.lock:
            direction = (
                self.steering_direction
                if now - self.last_steering_seen <= self.client_timeout_seconds
                else 0
            )
            if config.servo_qe_reversed:
                direction = -direction
            limits = (config.servo_speed_dps, config.servo_acceleration_dps2)
            send_limits = limits != self._last_sent_servo_limits
            send_direction = (
                direction != self._last_sent_steering_direction
                or (
                    direction != 0
                    and now - self._last_steering_sent_at >= self.heartbeat_seconds
                )
            )
            if send_limits:
                self._last_sent_servo_limits = limits
            if send_direction:
                self._last_sent_steering_direction = direction
                self._last_steering_sent_at = now
        if send_limits:
            self._write(f"SVC,{limits[0]:.1f},{limits[1]:.1f}")
        if send_direction:
            self._write(f"SVD,{direction}")
