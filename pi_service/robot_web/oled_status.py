"""Non-blocking OLED status page for the robot web service."""
from __future__ import annotations

import threading
from typing import Any

from oled_display import OledDisplay


class OledStatusService:
    """Render robot health to OLED without affecting the control path."""

    def __init__(
        self,
        controller: Any,
        camera: Any,
        *,
        address: int = 0x3C,
        i2c_port: int = 1,
        interval_seconds: float = 1.0,
        display: OledDisplay | None = None,
    ) -> None:
        self._controller = controller
        self._camera = camera
        self._display = display or OledDisplay(address=address, i2c_port=i2c_port)
        self._interval_seconds = max(0.2, float(interval_seconds))
        self._stop = threading.Event()
        self._thread: threading.Thread | None = None
        self._running = False
        self.error = ""

    def start(self) -> None:
        if self._running:
            return
        if not self._display.begin():
            self.error = self._display.error or "OLED 初始化失败"
            return
        self._stop.clear()
        self._running = True
        self._thread = threading.Thread(target=self._run, name="oled-status", daemon=True)
        self._thread.start()

    def _run(self) -> None:
        while not self._stop.is_set():
            try:
                self._render()
                self.error = ""
            except Exception as exc:
                self.error = str(exc)
                break
            self._stop.wait(self._interval_seconds)
        self._running = False

    def _render(self) -> None:
        robot = self._controller.status()
        camera = self._camera.status_dict()
        arduino_online = bool(robot.get("arduino_online"))
        camera_online = bool(camera.get("online"))
        if not arduino_online or not camera_online:
            self._display.show_warning()
            return
        ultrasonic = robot.get("ultrasonic")
        front = ultrasonic[1] if isinstance(ultrasonic, (list, tuple)) and len(ultrasonic) > 1 else None
        distance = "--"
        try:
            if float(front) >= 0:
                distance = f"{float(front):.0f}cm"
        except (TypeError, ValueError):
            pass
        action = str(robot.get("action") or "STOP")[:6]
        self._display.show_text(["ROBITICS", f"A:{'OK' if arduino_online else 'OFF'} C:{'OK' if camera_online else 'OFF'}", f"F:{distance}", action])

    def status_dict(self) -> dict[str, object]:
        return {
            "online": self._running and self._display.online and not self.error,
            "address": f"0x{self._display.address:02X}",
            "i2c_port": self._display.i2c_port,
            "error": self.error or self._display.error,
        }

    def stop(self) -> None:
        self._stop.set()
        thread = self._thread
        if thread and thread is not threading.current_thread():
            thread.join(timeout=2.0)
        try:
            self._display.stop()
        except Exception as exc:
            self.error = str(exc)
        self._running = False
