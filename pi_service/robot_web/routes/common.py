"""Route-wide run gate and latest-frame preview publisher."""
from __future__ import annotations

import threading
from typing import Iterator


class AutonomousRunGate:
    """Motor permission; deliberately starts paused."""

    def __init__(self) -> None:
        self._enabled = False
        self._lock = threading.Lock()

    def toggle(self) -> bool:
        with self._lock:
            self._enabled = not self._enabled
            return self._enabled

    def set_enabled(self, enabled: bool) -> bool:
        """Set motor permission idempotently and return the resulting value."""

        if not isinstance(enabled, bool):
            raise ValueError("enabled must be a boolean")
        with self._lock:
            self._enabled = enabled
            return self._enabled

    def enabled(self) -> bool:
        with self._lock:
            return self._enabled


class RoutePreviewPublisher:
    """Latest-frame MJPEG publisher; slow clients never build a queue."""

    def __init__(self) -> None:
        self._condition = threading.Condition()
        self._jpeg: bytes | None = None
        self._sequence = 0

    def publish(self, jpeg: bytes) -> None:
        with self._condition:
            self._jpeg, self._sequence = jpeg, self._sequence + 1
            self._condition.notify_all()

    def iter_mjpeg(self) -> Iterator[bytes]:
        sequence = -1
        while True:
            with self._condition:
                self._condition.wait_for(
                    lambda: self._sequence != sequence, timeout=1.0
                )
                jpeg, sequence = self._jpeg, self._sequence
            if jpeg:
                yield (
                    b"--frame\r\nContent-Type: image/jpeg\r\n\r\n"
                    + jpeg
                    + b"\r\n"
                )
