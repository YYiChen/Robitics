"""Linux system metrics for the Raspberry Pi web status endpoint."""
from __future__ import annotations

import os
import shutil
import time
from pathlib import Path


class SystemMetrics:
    """Read lightweight `/proc` metrics without adding a psutil dependency."""

    def __init__(self) -> None:
        self._previous_cpu: tuple[int, int] | None = None
        self._started_at = time.monotonic()

    @staticmethod
    def _cpu_ticks() -> tuple[int, int] | None:
        try:
            fields = Path("/proc/stat").read_text(encoding="utf-8").splitlines()[0].split()[1:]
            ticks = [int(value) for value in fields]
            total = sum(ticks)
            idle = ticks[3] + (ticks[4] if len(ticks) > 4 else 0)
            return total, idle
        except (FileNotFoundError, IndexError, OSError, ValueError):
            return None

    @staticmethod
    def _memory_bytes() -> tuple[int, int] | None:
        try:
            values = {}
            for line in Path("/proc/meminfo").read_text(encoding="utf-8").splitlines():
                key, raw = line.split(":", 1)
                values[key] = int(raw.split()[0]) * 1024
            total = values["MemTotal"]
            available = values.get("MemAvailable", values.get("MemFree", 0))
            return total, max(0, total - available)
        except (FileNotFoundError, KeyError, OSError, ValueError):
            return None

    @staticmethod
    def _temperature_c() -> float | None:
        for path in Path("/sys/class/thermal").glob("thermal_zone*/temp"):
            try:
                return int(path.read_text(encoding="utf-8").strip()) / 1000.0
            except (OSError, ValueError):
                continue
        return None

    def status_dict(self) -> dict:
        ticks = self._cpu_ticks()
        cpu_percent = None
        if ticks is not None and self._previous_cpu is not None:
            total_delta = ticks[0] - self._previous_cpu[0]
            idle_delta = ticks[1] - self._previous_cpu[1]
            if total_delta > 0:
                cpu_percent = max(0.0, min(100.0, (1.0 - idle_delta / total_delta) * 100.0))
        self._previous_cpu = ticks
        memory = self._memory_bytes()
        try:
            disk = shutil.disk_usage("/")
            disk_total, disk_used = disk.total, disk.used
        except OSError:
            disk_total = disk_used = None
        try:
            load_1m = os.getloadavg()[0]
        except (AttributeError, OSError):
            load_1m = None
        return {
            "cpu_percent": cpu_percent,
            "load_1m": load_1m,
            "memory_total_bytes": memory[0] if memory else None,
            "memory_used_bytes": memory[1] if memory else None,
            "disk_total_bytes": disk_total,
            "disk_used_bytes": disk_used,
            "cpu_temperature_c": self._temperature_c(),
            "uptime_seconds": time.monotonic() - self._started_at,
        }
