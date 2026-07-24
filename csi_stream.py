import threading

import cv2
import numpy as np
from flask import Flask, Response, jsonify, request
from picamera2 import Picamera2

app = Flask(__name__)

# These values are applied after Picamera2's automatic white balance. Keep the
# defaults neutral; use the web controls only to compensate for the real light.
color_lock = threading.Lock()
color = {
    "red": 1.00,
    "green": 1.00,
    "blue": 1.00,
    "saturation": 1.00,
    "brightness": 0,
    "contrast": 1.00,
}

print("正在初始化树莓派5 CSI 摄像头…")
picam2 = Picamera2()
config = picam2.create_video_configuration(
    main={"size": (1640, 1232), "format": "RGB888"},
    controls={
        "FrameDurationLimits": (25000, 25000),
        "AwbEnable": True,
        "AeEnable": True,
    },
    buffer_count=4,
)
picam2.configure(config)
picam2.start()
print("摄像头已成功启动；自动白平衡已启用。")


def apply_color_calibration(bgr: np.ndarray) -> np.ndarray:
    """Apply small browser-controlled corrections to an OpenCV BGR image."""
    with color_lock:
        settings = color.copy()

    adjusted = bgr.astype(np.float32)
    # OpenCV stores channels as B, G, R after the conversion below.
    adjusted[:, :, 0] *= settings["blue"]
    adjusted[:, :, 1] *= settings["green"]
    adjusted[:, :, 2] *= settings["red"]
    adjusted = np.clip(adjusted, 0, 255).astype(np.uint8)

    if settings["saturation"] != 1.0:
        hsv = cv2.cvtColor(adjusted, cv2.COLOR_BGR2HSV)
        hsv[:, :, 1] = np.clip(
            hsv[:, :, 1].astype(np.float32) * settings["saturation"], 0, 255
        ).astype(np.uint8)
        adjusted = cv2.cvtColor(hsv, cv2.COLOR_HSV2BGR)

    if settings["contrast"] != 1.0 or settings["brightness"] != 0:
        adjusted = cv2.convertScaleAbs(
            adjusted, alpha=settings["contrast"], beta=settings["brightness"]
        )
    return adjusted


def generate_frames():
    while True:
        try:
            # Picamera2 gives RGB888; OpenCV/JPEG expects BGR channel order.
            rgb = picam2.capture_array()
            bgr = cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)
            bgr = apply_color_calibration(bgr)
            success, buffer = cv2.imencode(
                ".jpg", bgr, [cv2.IMWRITE_JPEG_QUALITY, 80]
            )
            if success:
                yield (
                    b"--frame\r\n"
                    b"Content-Type: image/jpeg\r\n\r\n"
                    + buffer.tobytes()
                    + b"\r\n"
                )
        except Exception as exc:
            print(f"读取画面或串流出错: {exc}")
            break


@app.get("/video_feed")
def video_feed():
    return Response(
        generate_frames(),
        mimetype="multipart/x-mixed-replace; boundary=frame",
        headers={"Cache-Control": "no-store, max-age=0"},
    )


@app.get("/api/color")
def get_color():
    with color_lock:
        return jsonify(color)


@app.post("/api/color")
def set_color():
    payload = request.get_json(silent=True) or {}
    limits = {
        "red": (0.50, 1.80),
        "green": (0.50, 1.80),
        "blue": (0.50, 1.80),
        "saturation": (0.00, 2.00),
        "brightness": (-80, 80),
        "contrast": (0.50, 1.80),
    }
    with color_lock:
        for name, (low, high) in limits.items():
            if name in payload:
                color[name] = max(low, min(high, float(payload[name])))
        current = color.copy()
    return jsonify({"ok": True, "color": current})


@app.post("/api/color/reset")
def reset_color():
    with color_lock:
        color.update(red=1.0, green=1.0, blue=1.0, saturation=1.0, brightness=0, contrast=1.0)
        current = color.copy()
    return jsonify({"ok": True, "color": current})


@app.get("/")
def index():
    return """<!doctype html><html lang='zh-CN'><meta charset='utf-8'>
<title>CSI 摄像头颜色校准</title>
<style>body{font:16px system-ui;margin:20px;max-width:950px;background:#111;color:#eee}img{width:100%;max-width:820px;background:#222;border-radius:8px}.row{display:grid;grid-template-columns:110px 1fr 60px;gap:10px;align-items:center;margin:10px 0}input{width:100%}button{padding:9px 14px}</style>
<h2>CSI 摄像头颜色校准</h2><img src='/video_feed' alt='实时画面'>
<p>先等待约 3 秒让自动白平衡稳定。偏蓝则降低蓝色；偏红则降低红色。默认值均为中性值。</p>
<div id='controls'></div><button id='reset'>恢复中性值</button>
<script>
const defs={red:[.5,1.8,.01,'红色'],green:[.5,1.8,.01,'绿色'],blue:[.5,1.8,.01,'蓝色'],saturation:[0,2,.01,'饱和度'],brightness:[-80,80,1,'亮度'],contrast:[.5,1.8,.01,'对比度']};
const root=document.querySelector('#controls');
for(const [key,[min,max,step,label]] of Object.entries(defs)){root.insertAdjacentHTML('beforeend',`<div class='row'><label>${label}</label><input id='${key}' type='range' min='${min}' max='${max}' step='${step}'><output id='${key}Out'></output></div>`)}
async function load(){const s=await (await fetch('/api/color')).json();for(const k in defs){const e=document.querySelector('#'+k);e.value=s[k];document.querySelector('#'+k+'Out').textContent=s[k]}}
async function save(){const v={};for(const k in defs){v[k]=Number(document.querySelector('#'+k).value);document.querySelector('#'+k+'Out').textContent=v[k]}await fetch('/api/color',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify(v)})}
root.addEventListener('input',save);document.querySelector('#reset').onclick=async()=>{await fetch('/api/color/reset',{method:'POST'});load()};load();
</script></html>"""


if __name__ == "__main__":
    try:
        app.run(host="0.0.0.0", port=5000, threaded=True)
    finally:
        print("正在关闭摄像头…")
        picam2.stop()
