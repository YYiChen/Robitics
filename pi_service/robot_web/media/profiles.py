"""Shared camera mode and output-profile definitions."""

CAMERA_MODES = {
    "fast_1640": {
        "label": "1640×1232", "width": 1640, "height": 1232,
        "sensor_fps": 30.0, "stream_fps": 30.0,
    },
    "full_3280": {
        "label": "3280×2464", "width": 3280, "height": 2464,
        "sensor_fps": 30.0, "stream_fps": 30.0,
    },
}
DEFAULT_CAMERA_MODE = "fast_1640"
DEFAULT_EXPOSURE = {"auto": True, "ev": 0.0, "shutter_denominator": 200}
STREAM_PROFILES = {
    "low_latency": {
        "label": "低延迟 · 640×480 · JPEG 60", "max_width": 640,
        "size": (640, 480), "quality": 60,
    },
    "balanced": {"label": "平衡 · 最宽 1230 px · JPEG 70", "max_width": 1230, "quality": 70},
    "source": {"label": "原始尺寸 · JPEG 70", "max_width": None, "quality": 70},
}
DEFAULT_STREAM_PROFILE = "low_latency"
DEFAULT_HIGHRES_FPS = 2.0
MIN_HIGHRES_FPS = 1.0
MAX_HIGHRES_FPS = 30.0
HIGHRES_JPEG_QUALITY = 75
HIGHRES_CACHE_MAX_AGE = 0.45
HIGHRES_PROFILES = {
    "source": {"label": "原始尺寸 · JPEG 75", "max_width": None},
    "medium_1640": {"label": "高清平衡 · 最宽 1640 px · JPEG 75", "max_width": 1640},
    "compact_1280": {"label": "高清轻量 · 最宽 1280 px · JPEG 75", "max_width": 1280},
}
DEFAULT_HIGHRES_PROFILE = "source"
DEFAULT_COLOR_CORRECTION = {"enabled": True, "strength": 1.0}
EDGE_BGR_GAINS = (0.92, 1.06, 0.78)
RADIAL_FALLOFF_EXPONENT = 0.55
