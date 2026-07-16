"""SSD1306 64×48 OLED driver for the Raspberry Pi I2C bus."""
from __future__ import annotations

import time


class OledDisplay:
    """Small 64×48 SSD1306 panel using the COM-pin layout required by this module."""

    def __init__(self, address: int = 0x3C, i2c_port: int = 1) -> None:
        self.address = address
        self.i2c_port = i2c_port
        self._device = None
        self.error = ""

    @property
    def online(self) -> bool:
        return self._device is not None and not self.error

    def begin(self) -> bool:
        try:
            from luma.core.interface.serial import i2c
            from luma.oled.device import ssd1306
            serial = i2c(port=self.i2c_port, address=self.address)
            device = ssd1306(serial, width=64, height=48)
            # This 64×48 panel needs an alternative COM pin configuration.
            device.command(0xDA)
            device.command(0x12)
            device.command(0x81)
            device.command(0xCF)
            self._device = device
            self.error = ""
            self.clear()
            return True
        except Exception as exc:
            self._device = None
            self.error = str(exc)
            return False

    def clear(self) -> None:
        if not self._device:
            return
        from luma.core.render import canvas
        with canvas(self._device):
            pass

    def show_text(self, lines: list[str]) -> None:
        if not self._device:
            return
        from luma.core.render import canvas
        from PIL import ImageFont
        font = ImageFont.load_default()
        with canvas(self._device) as draw:
            for index, line in enumerate(lines[:4]):
                draw.text((0, index * 11), line[:10], fill="white", font=font)

    def show_warning(self) -> None:
        if not self._device:
            return
        from luma.core.render import canvas
        with canvas(self._device) as draw:
            draw.polygon([(32, 4), (5, 43), (59, 43)], outline="white")
            draw.rectangle((30, 15, 33, 31), fill="white")
            draw.rectangle((30, 35, 33, 38), fill="white")

    def show_smile(self) -> None:
        if not self._device:
            return
        from luma.core.render import canvas
        with canvas(self._device) as draw:
            draw.ellipse((12, 4, 52, 44), outline="white")
            draw.ellipse((18, 14, 23, 19), fill="white")
            draw.ellipse((41, 14, 46, 19), fill="white")
            draw.arc((22, 26, 42, 42), 20, 160, fill="white")

    def show_heart(self) -> None:
        if not self._device:
            return
        from luma.core.render import canvas
        with canvas(self._device) as draw:
            draw.ellipse((14, 4, 30, 20), fill="white")
            draw.ellipse((34, 4, 50, 20), fill="white")
            draw.polygon([(12, 16), (52, 16), (32, 43)], fill="white")

    def stop(self) -> None:
        self.clear()
        self._device = None


def _test() -> None:
    oled = OledDisplay()
    if not oled.begin():
        raise SystemExit(f"OLED 初始化失败：{oled.error}")
    for draw in (lambda: oled.show_text(["ROBITICS", "OLED 64x48", "I2C OK"]), oled.show_smile, oled.show_heart, oled.show_warning):
        draw()
        time.sleep(1.5)
    oled.stop()


if __name__ == "__main__":
    _test()
