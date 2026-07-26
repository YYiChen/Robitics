"""Card-motor command workflow mixed into the single serial controller."""
from __future__ import annotations

import threading
import time


CARD_COMMAND_ACK_TIMEOUT_SECONDS = 1.20


class CardControlMixin:
    @staticmethod
    def _timed_motor_parameters(raw_pwm: object, raw_duration_ms: object) -> tuple[int, int]:
        try:
            pwm = int(raw_pwm)
            duration_ms = int(raw_duration_ms)
        except (TypeError, ValueError):
            raise ValueError("电机功率和运行时间必须是整数") from None
        if pwm == 0 or not -255 <= pwm <= 255:
            raise ValueError("电机 PWM 必须在 -255 到 255 之间，正负号控制方向；0 不会驱动电机")
        if not 100 <= duration_ms <= 60000:
            raise ValueError("电机运行时间必须在 100 到 60000 毫秒之间")
        return pwm, duration_ms

    def _send_card_command_and_wait(self, motor: str, command: str) -> str:
        with self._card_command_lock:
            with self._card_reply_condition:
                before = self._card_reply_sequence
                self._card_reply_waiting_for = motor
            if not self._write(command):
                with self._card_reply_condition:
                    self._card_reply_waiting_for = None
                raise RuntimeError(f"串口写入失败：{self.error or 'Arduino 串口未打开'}")
            deadline = time.monotonic() + CARD_COMMAND_ACK_TIMEOUT_SECONDS
            with self._card_reply_condition:
                while self._card_reply_sequence == before:
                    remaining = deadline - time.monotonic()
                    if remaining <= 0:
                        self._card_reply_waiting_for = None
                        raise TimeoutError(f"Arduino 未确认 {command}")
                    self._card_reply_condition.wait(remaining)
                self._card_reply_waiting_for = None
                return self.card_command_reply

    @staticmethod
    def _card_result_state(reply: str) -> str:
        if reply.startswith("OK:"):
            return "running"
        if reply.startswith("BUSY:"):
            return "busy"
        raise RuntimeError(f"Arduino 拒绝卡牌电机命令：{reply}")

    def deal_card(self, raw_pwm: object = 255, raw_duration_ms: object = 1000) -> str:
        pwm, duration_ms = self._timed_motor_parameters(raw_pwm, raw_duration_ms)
        with self.lock:
            self.card_deal_state = "requested"
            protocol = self.card_motor_protocol
        command = "DEAL" if protocol == "legacy" else f"DEAL,{pwm},{duration_ms}"
        try:
            reply = self._send_card_command_and_wait("DEAL", command)
            state = self._card_result_state(reply)
        except (TimeoutError, RuntimeError) as first_error:
            if protocol != "unknown":
                with self.lock:
                    self.card_deal_state = "error"
                raise RuntimeError(f"发送出牌命令失败：{first_error}") from first_error
            try:
                reply = self._send_card_command_and_wait("DEAL", "DEAL")
                state = self._card_result_state(reply)
                with self.lock:
                    if reply == "OK:DEAL":
                        self.card_motor_protocol = "legacy"
                    elif reply.startswith("OK:DEAL,"):
                        self.card_motor_protocol = "adjustable"
                    protocol = self.card_motor_protocol
            except (TimeoutError, RuntimeError) as fallback_error:
                with self.lock:
                    self.card_deal_state = "error"
                raise RuntimeError(f"新版和旧版出牌命令均未被 Arduino 接受：{fallback_error}") from fallback_error
        with self.lock:
            self.card_deal_state = "running" if state in {"running", "busy"} else state
        return "legacy" if protocol == "legacy" and state == "running" else state

    def feed_cards(self, raw_pwm: object = -255, raw_duration_ms: object = 5000) -> str:
        pwm, duration_ms = self._timed_motor_parameters(raw_pwm, raw_duration_ms)
        with self.lock:
            protocol = self.card_motor_protocol
        if protocol == "legacy":
            raise RuntimeError("Arduino 固件过旧，不支持网页触发 M3；请重新烧录新版 motor_bridge 固件")
        with self.lock:
            self.card_feed_state = "requested"
        try:
            reply = self._send_card_command_and_wait("FEED", f"FEED,{pwm},{duration_ms}")
            state = self._card_result_state(reply)
        except (TimeoutError, RuntimeError) as exc:
            with self.lock:
                self.card_feed_state = "error"
            raise RuntimeError(f"发送送牌命令失败：{exc}") from exc
        with self.lock:
            self.card_feed_state = "running" if state in {"running", "busy"} else state
        return state

    def deal_from_key_request(self, request: object) -> dict | None:
        if not isinstance(request, dict):
            return None
        token = str(request.get("token", "")).strip()
        if not token:
            raise ValueError("出牌事件缺少 token")
        feed_pwm, feed_duration_ms = self._timed_motor_parameters(
            request.get("feed_pwm", -255), request.get("feed_duration_ms", 5000)
        )
        deal_pwm, deal_duration_ms = self._timed_motor_parameters(
            request.get("deal_pwm", request.get("pwm", 255)),
            request.get("deal_duration_ms", request.get("duration_ms", 1000)),
        )
        with self._deal_request_lock:
            if token == self._last_deal_request_token:
                return self._last_deal_request_result
            self._last_deal_request_token = token
            self._last_deal_request_result = {"token": token, "state": "pending", "reply": ""}
            threading.Thread(
                target=self._complete_deal_request,
                args=(token, feed_pwm, feed_duration_ms, deal_pwm, deal_duration_ms),
                daemon=True,
                name="card-deal-request",
            ).start()
            return self._last_deal_request_result

    def deal_request_status(self, token: object) -> dict | None:
        """Return a prior key-deal result without issuing another motor command."""

        normalized = str(token or "").strip()
        if not normalized:
            raise ValueError("出牌事件缺少 token")
        with self._deal_request_lock:
            if normalized != self._last_deal_request_token:
                return None
            return (
                dict(self._last_deal_request_result)
                if self._last_deal_request_result is not None
                else None
            )

    def _complete_deal_request(
        self, token: str, feed_pwm: int, feed_duration_ms: int,
        deal_pwm: int, deal_duration_ms: int,
    ) -> None:
        try:
            feed_result = {"state": self.feed_cards(feed_pwm, feed_duration_ms), "reply": self.card_command_reply}
        except (RuntimeError, ValueError) as exc:
            feed_result = {"state": "error", "reply": self.card_command_reply, "error": str(exc)}
        try:
            deal_result = {"state": self.deal_card(deal_pwm, deal_duration_ms), "reply": self.card_command_reply}
        except (RuntimeError, ValueError) as exc:
            deal_result = {"state": "error", "reply": self.card_command_reply, "error": str(exc)}
        failed = sum(item["state"] == "error" for item in (feed_result, deal_result))
        result = {
            "token": token,
            "state": "error" if failed == 2 else "partial" if failed == 1 else "running",
            "feed": feed_result,
            "deal": deal_result,
        }
        with self._deal_request_lock:
            if token == self._last_deal_request_token:
                self._last_deal_request_result = result
