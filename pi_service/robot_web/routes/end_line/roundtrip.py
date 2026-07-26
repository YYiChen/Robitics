"""Pure two-face/white-line landmark sequence for the end-line route."""
from __future__ import annotations

from dataclasses import dataclass
from enum import Enum


class RoundtripPhase(str, Enum):
    IDLE = "IDLE"
    FOLLOW_OUTBOUND = "FOLLOW_OUTBOUND"
    TURN_FACE_1 = "TURN_FACE_1"
    TURN_LINE_MIDDLE = "TURN_LINE_MIDDLE"
    TURN_FACE_2 = "TURN_FACE_2"
    TURN_LINE_FINAL = "TURN_LINE_FINAL"
    READY_RETURN = "READY_RETURN"
    FOLLOW_RETURN = "FOLLOW_RETURN"
    COMPLETE = "COMPLETE"


class LandmarkTarget(str, Enum):
    FACE = "FACE"
    WHITE_LINE = "WHITE_LINE"


@dataclass(frozen=True)
class TurnInstruction:
    side: str
    target: LandmarkTarget
    label: str


class TwoFaceRoundtripPlanner:
    """Deterministic route memory; perception and motors stay outside."""

    def __init__(self) -> None:
        self.phase = RoundtripPhase.IDLE
        self.sweep_side: str | None = None

    def reset(self) -> None:
        self.phase = RoundtripPhase.IDLE
        self.sweep_side = None

    def start(self, sweep_side: str) -> None:
        side = str(sweep_side).upper()
        if side not in {"LEFT", "RIGHT"}:
            raise ValueError("sweep_side 只支持 LEFT 或 RIGHT")
        if self.phase not in {RoundtripPhase.IDLE, RoundtripPhase.COMPLETE}:
            raise ValueError(f"当前序列状态 {self.phase.value} 不能重新开始")
        self.sweep_side = side
        self.phase = RoundtripPhase.FOLLOW_OUTBOUND

    def endpoint_reached(self) -> None:
        if self.phase == RoundtripPhase.FOLLOW_OUTBOUND:
            self.phase = RoundtripPhase.TURN_FACE_1
        elif self.phase == RoundtripPhase.FOLLOW_RETURN:
            self.phase = RoundtripPhase.COMPLETE
        else:
            raise ValueError(f"{self.phase.value} 状态不等待白线端点")

    def expected_turn(self) -> TurnInstruction | None:
        if self.sweep_side is None:
            return None
        opposite = "RIGHT" if self.sweep_side == "LEFT" else "LEFT"
        mapping = {
            RoundtripPhase.TURN_FACE_1: TurnInstruction(self.sweep_side, LandmarkTarget.FACE, "FACE_1"),
            RoundtripPhase.TURN_LINE_MIDDLE: TurnInstruction(self.sweep_side, LandmarkTarget.WHITE_LINE, "LINE_180"),
            RoundtripPhase.TURN_FACE_2: TurnInstruction(self.sweep_side, LandmarkTarget.FACE, "FACE_2"),
            RoundtripPhase.TURN_LINE_FINAL: TurnInstruction(opposite, LandmarkTarget.WHITE_LINE, "LINE_RETURN"),
        }
        return mapping.get(self.phase)

    def target_reached(self, target: LandmarkTarget | str) -> None:
        actual = target if isinstance(target, LandmarkTarget) else LandmarkTarget(str(target))
        expected = self.expected_turn()
        if expected is None:
            raise ValueError(f"{self.phase.value} 状态不等待视觉地标")
        if actual != expected.target:
            raise ValueError(f"当前需要 {expected.target.value}，不能用 {actual.value} 完成本阶段")
        transitions = {
            RoundtripPhase.TURN_FACE_1: RoundtripPhase.TURN_LINE_MIDDLE,
            RoundtripPhase.TURN_LINE_MIDDLE: RoundtripPhase.TURN_FACE_2,
            RoundtripPhase.TURN_FACE_2: RoundtripPhase.TURN_LINE_FINAL,
            RoundtripPhase.TURN_LINE_FINAL: RoundtripPhase.READY_RETURN,
        }
        self.phase = transitions[self.phase]

    def start_return(self) -> None:
        if self.phase != RoundtripPhase.READY_RETURN:
            raise ValueError("只有 READY_RETURN 状态可以开始返程巡线")
        self.phase = RoundtripPhase.FOLLOW_RETURN

    def status_dict(self) -> dict:
        instruction = self.expected_turn()
        return {
            "phase": self.phase.value,
            "sweep_side": self.sweep_side,
            "expected_side": None if instruction is None else instruction.side,
            "expected_target": None if instruction is None else instruction.target.value,
            "expected_label": None if instruction is None else instruction.label,
        }
