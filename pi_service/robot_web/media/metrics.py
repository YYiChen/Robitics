"""Pure rolling-window calculations for camera transport metrics."""
from __future__ import annotations

from collections import deque


def window_stats(events: deque, now: float) -> tuple[int, int, float]:
    cutoff = now - 1.0
    while events and events[0][0] < cutoff:
        events.popleft()
    if not events:
        return 0, 0, 1.0
    return len(events), sum(item[1] for item in events), 1.0


def compact_window_stats(events: deque, now: float) -> tuple[int, int]:
    count, total, _elapsed = window_stats(events, now)
    return count, total
