"""Pure parsing of Arduino replies into controller state changes."""
from __future__ import annotations


def parse_protocol_line(text: str, motor_output: list[int] | None) -> dict:
    changes: dict = {}
    parts = text.split(",")
    try:
        if text.startswith("READY:MOTOR_BRIDGE"):
            if "DEAL_ADJUSTABLE" in text:
                changes["card_motor_protocol"] = "adjustable"
            elif "DEAL_1000MS" in text:
                changes["card_motor_protocol"] = "legacy"
        elif text.startswith("IMU,") and len(parts) == 4:
            changes["imu"] = [float(value) for value in parts[1:]]
        elif text.startswith("SPD,") and len(parts) == 7:
            changes["speed"] = [float(value) for value in parts[1:]]
        elif (text.startswith("OK:M,") or text.startswith("OUT,")) and len(parts) == 5:
            changes["motor_output"] = [int(value) for value in parts[1:]]
        elif text.startswith("US,") and len(parts) == 2:
            changes["ultrasonic"] = float(parts[1])
        elif text.startswith(("OK:SV,", "SVP,")):
            changes["servo_angle"] = int(text.split(",", 1)[1])
        elif text.startswith("OK:DEAL") or text == "BUSY:DEAL":
            changes["card_deal_state"] = "running"
        elif text == "DEAL:DONE":
            changes["card_deal_state"] = "idle"
        elif text.startswith("OK:FEED") or text == "BUSY:FEED":
            changes["card_feed_state"] = "running"
        elif text == "FEED:DONE":
            changes["card_feed_state"] = "idle"
        elif text in {"OK:STOP", "STATUS:STOPPED", "STATUS:DRIVE_STOPPED", "TIMEOUT:STOP"} or text.startswith("BLOCK:"):
            outputs = list(motor_output or [0, 0, 0, 0])
            outputs[0] = outputs[1] = 0
            changes["motor_output"] = outputs
    except ValueError:
        return {}
    return changes
