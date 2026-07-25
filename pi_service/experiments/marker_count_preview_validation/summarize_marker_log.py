"""Print a compact, human-readable marker-event report from preview JSON logs."""

from __future__ import annotations

import json
from pathlib import Path
import sys


def main() -> int:
    if len(sys.argv) != 2:
        raise SystemExit("usage: summarize_marker_log.py /path/to/latest.log")
    path = Path(sys.argv[1])
    events: list[dict[str, object]] = []
    last: dict[str, object] | None = None
    for raw in path.read_text(encoding="utf-8", errors="replace").splitlines():
        try:
            item = json.loads(raw)
        except json.JSONDecodeError:
            continue
        last = item
        if item.get("marker_event"):
            events.append(item)
    print(f"marker events: {len(events)}")
    for item in events:
        print(
            "frame={frame} time={wall_time} marker={marker_in_lap}/4 lap={lap_count}".format(
                frame=item.get("frame", "?"),
                wall_time=item.get("wall_time", "?"),
                marker_in_lap=item.get("marker_in_lap", "?"),
                lap_count=item.get("lap_count", "?"),
            )
        )
    if last is not None:
        print(
            "last: frame={frame} marker={marker_in_lap}/4 lap={lap_count} intent={intent}".format(
                frame=last.get("frame", "?"),
                marker_in_lap=last.get("marker_in_lap", "?"),
                lap_count=last.get("lap_count", "?"),
                intent=last.get("intent", "?"),
            )
        )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
