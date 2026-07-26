"""Checkpointed drive recording and vision-primary return replay.

The recorder stores only M1/M2 commands.  Card commands are never replayed.
Recorded PWM is a bounded feed-forward hint; live white-line vision remains
the primary return controller.
"""
from __future__ import annotations

from dataclasses import asdict, dataclass
import json
from pathlib import Path
import threading
import time


@dataclass(frozen=True)
class DriveSample:
    duration_seconds: float
    right_pwm: int
    left_pwm: int
    line_center_x: float | None
    confidence: float
    right_speed_pps: float | None = None
    left_speed_pps: float | None = None


@dataclass(frozen=True)
class RouteSegment:
    index: int
    checkpoint: str
    estimated_right_ticks: float
    estimated_left_ticks: float
    samples: tuple[DriveSample, ...]


@dataclass(frozen=True)
class ReplayStep:
    sample: DriveSample | None
    segment_index: int | None
    checkpoint: str | None
    segment_changed: bool = False
    complete: bool = False

    @property
    def forward_facing_pwm(self) -> tuple[int, int] | None:
        """Swap the recorded wheels after the vehicle has turned 180 degrees."""
        if self.sample is None:
            return None
        return self.sample.left_pwm, self.sample.right_pwm


class ReturnReplay:
    """Time-preserving reverse traversal of recorded route segments."""

    def __init__(self, segments: list[RouteSegment]) -> None:
        self._segments = list(reversed(segments))
        self._segment_cursor = 0
        self._sample_cursor = 0
        self._deadline: float | None = None
        self._complete = not bool(self._segments)

    def status_dict(self) -> dict:
        return {
            "complete": self._complete,
            "segments_remaining": max(0, len(self._segments) - self._segment_cursor),
            "segment_index": (
                None if self._complete else self._segments[self._segment_cursor].index
            ),
        }

    def step(self, now: float | None = None) -> ReplayStep:
        now = time.monotonic() if now is None else float(now)
        if self._complete:
            return ReplayStep(None, None, None, complete=True)
        segment_changed = False
        while True:
            segment = self._segments[self._segment_cursor]
            reverse_samples = segment.samples
            if self._sample_cursor >= len(reverse_samples):
                self._segment_cursor += 1
                self._sample_cursor = 0
                self._deadline = None
                segment_changed = True
                if self._segment_cursor >= len(self._segments):
                    self._complete = True
                    return ReplayStep(
                        None, segment.index, segment.checkpoint,
                        segment_changed=True, complete=True,
                    )
                continue
            sample = reverse_samples[-1 - self._sample_cursor]
            if self._deadline is None:
                self._deadline = now + max(.01, sample.duration_seconds)
            elif now >= self._deadline:
                self._sample_cursor += 1
                self._deadline = None
                continue
            return ReplayStep(
                sample,
                segment.index,
                segment.checkpoint,
                segment_changed=segment_changed,
            )


class ReturnRouteRecorder:
    """Record forward-follow commands and split them on durable landmarks."""

    def __init__(
        self,
        output_path: Path,
        *,
        nominal_sample_seconds: float = .05,
    ) -> None:
        self.output_path = Path(output_path)
        self.nominal_sample_seconds = max(.01, float(nominal_sample_seconds))
        self._lock = threading.RLock()
        self._segments: list[RouteSegment] = []
        self._current: list[DriveSample] = []
        self._last_at: float | None = None
        self._estimated_right_ticks = 0.0
        self._estimated_left_ticks = 0.0
        self._active = False

    def start_recording(self) -> None:
        with self._lock:
            self._segments = []
            self._current = []
            self._last_at = None
            self._estimated_right_ticks = 0.0
            self._estimated_left_ticks = 0.0
            self._active = True

    def record(
        self,
        right_pwm: int,
        left_pwm: int,
        *,
        line_center_x: float | None,
        confidence: float,
        wheel_speed: list[float] | tuple[float, ...] | None = None,
        now: float | None = None,
    ) -> None:
        with self._lock:
            if not self._active:
                return
            now = time.monotonic() if now is None else float(now)
            duration = (
                self.nominal_sample_seconds
                if self._last_at is None
                else max(.01, min(.25, now - self._last_at))
            )
            self._last_at = now
            right_speed = left_speed = None
            if wheel_speed is not None and len(wheel_speed) >= 2:
                # Arduino SPD order is left, right.
                left_speed, right_speed = float(wheel_speed[0]), float(wheel_speed[1])
                self._estimated_right_ticks += right_speed * duration
                self._estimated_left_ticks += left_speed * duration
            self._current.append(DriveSample(
                duration,
                max(-255, min(255, int(right_pwm))),
                max(-255, min(255, int(left_pwm))),
                None if line_center_x is None else float(line_center_x),
                max(0.0, min(1.0, float(confidence))),
                right_speed,
                left_speed,
            ))

    def checkpoint(self, name: str) -> bool:
        with self._lock:
            if not self._active or not self._current:
                return False
            self._segments.append(RouteSegment(
                len(self._segments),
                str(name),
                self._estimated_right_ticks,
                self._estimated_left_ticks,
                tuple(self._current),
            ))
            self._current = []
            self._last_at = None
            self._persist()
            return True

    def prepare_return(self) -> ReturnReplay:
        with self._lock:
            self.checkpoint("return_start")
            self._active = False
            self._persist()
            return ReturnReplay(list(self._segments))

    def has_samples(self) -> bool:
        with self._lock:
            return bool(self._current or any(segment.samples for segment in self._segments))

    def status_dict(self) -> dict:
        with self._lock:
            return {
                "recording": self._active,
                "segments": len(self._segments) + int(bool(self._current)),
                "checkpoints": [segment.checkpoint for segment in self._segments],
                "samples": len(self._current) + sum(len(item.samples) for item in self._segments),
                "estimated_right_ticks": round(self._estimated_right_ticks, 2),
                "estimated_left_ticks": round(self._estimated_left_ticks, 2),
                "path": str(self.output_path),
            }

    def _persist(self) -> None:
        payload = {
            "schema_version": 1,
            "recording": self._active,
            "segments": [
                {
                    **{key: value for key, value in asdict(segment).items() if key != "samples"},
                    "samples": [asdict(sample) for sample in segment.samples],
                }
                for segment in self._segments
            ],
        }
        self.output_path.parent.mkdir(parents=True, exist_ok=True)
        temporary = self.output_path.with_suffix(".tmp")
        temporary.write_text(
            json.dumps(payload, ensure_ascii=False, indent=2) + "\n",
            encoding="utf-8",
        )
        temporary.replace(self.output_path)


def blend_return_pwm(
    vision_pwm: tuple[int, int],
    replay_pwm: tuple[int, int],
    replay_weight: float,
) -> tuple[int, int]:
    """Blend bounded replay feed-forward under a vision-majority controller."""
    weight = max(0.0, min(.5, float(replay_weight)))
    right = round((1.0 - weight) * vision_pwm[0] + weight * replay_pwm[0])
    left = round((1.0 - weight) * vision_pwm[1] + weight * replay_pwm[1])
    return (
        max(-255, min(255, int(right))),
        max(-255, min(255, int(left))),
    )
