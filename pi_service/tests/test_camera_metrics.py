import sys
import tempfile
import unittest
import importlib.util
import re
from pathlib import Path

sys.path.insert(0, str(Path(__file__).parents[1] / "robot_web"))

from camera import CameraStreamer
from dual_stream_camera import DualStreamCamera
from webrtc_stream import WebRTCStreamer


class CameraMetricsTests(unittest.TestCase):
    def test_template_defines_every_id_referenced_by_frontend_script(self) -> None:
        web_root = Path(__file__).parents[1] / "robot_web"
        script = (web_root / "static" / "app.js").read_text(encoding="utf-8")
        template = (web_root / "templates" / "index.html").read_text(encoding="utf-8")
        referenced_ids = set(re.findall(r"#([A-Za-z][A-Za-z0-9_-]+)", script))
        missing = sorted(item for item in referenced_ids if f'id="{item}"' not in template)
        self.assertEqual(missing, [])

    def test_reports_encoded_and_stream_bandwidth(self) -> None:
        camera = CameraStreamer(width=1280, height=720, fps=15)
        camera._record_encoded(100_000, 12.5)
        camera._record_stream(100_050)

        status = camera.status_dict()
        self.assertEqual(status["resolution"], "1280x720")
        self.assertEqual(status["capture_fps"], 1)
        self.assertEqual(status["jpeg_bytes"], 100_000)
        self.assertAlmostEqual(status["jpeg_kBps"], 100.0)
        self.assertAlmostEqual(status["jpeg_kbps"], 800.0)
        self.assertAlmostEqual(status["stream_kbps"], 800.4)
        self.assertAlmostEqual(status["encode_ms"], 12.5)

    def test_camera_mode_is_persisted(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            config_path = Path(directory) / "camera_config.json"
            first = CameraStreamer(config_path=config_path)
            first.mode_key = "full_3280"
            first.auto_exposure = False
            first.exposure_ev = 1.2
            first.shutter_denominator = 250
            first.stream_profile_key = "balanced"
            first._save_settings()
            second = CameraStreamer(config_path=config_path)
            self.assertEqual(second.mode_key, "full_3280")
            self.assertEqual(second.status_dict()["resolution"], "3280x2464")
            self.assertFalse(second.auto_exposure)
            self.assertEqual(second.exposure_ev, 1.2)
            self.assertEqual(second.shutter_denominator, 250)
            self.assertEqual(second.stream_profile_key, "balanced")

    def test_low_latency_profile_downscales_transport_not_sensor_mode(self) -> None:
        camera = CameraStreamer(width=1640, height=1232)
        status = camera.set_stream_profile("low_latency")
        self.assertEqual(status["resolution"], "1640x1232")
        self.assertEqual(status["stream_profile"]["resolution"], "640x480")
        self.assertEqual(status["stream_profile"]["quality"], 60)

    def test_highres_profile_scales_only_the_two_fps_channel(self) -> None:
        camera = CameraStreamer(width=3280, height=2464)
        status = camera.set_highres_profile("medium_1640")
        self.assertEqual(status["resolution"], "3280x2464")
        self.assertEqual(status["highres_profile"]["resolution"], "1640x1232")
        self.assertEqual(status["highres_profile"]["quality"], 75)
        self.assertEqual(status["highres_profile"]["target_fps"], 2.0)

    def test_mjpeg_highres_fps_is_configurable_and_persisted(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            config_path = Path(directory) / "camera_config.json"
            first = CameraStreamer(config_path=config_path)
            status = first.set_highres_fps(5)
            self.assertEqual(status["highres"]["target_fps"], 5.0)
            self.assertEqual(status["highres_profile"]["target_fps"], 5.0)
            second = CameraStreamer(config_path=config_path)
            self.assertEqual(second.highres_fps, 5.0)
            with self.assertRaises(ValueError):
                second.set_highres_fps(16)

    def test_mjpeg_highres_work_requires_an_active_preview_client(self) -> None:
        camera = CameraStreamer()
        self.assertFalse(camera._highres_is_due(1.0))
        camera._highres_client_started()
        self.assertTrue(camera._highres_is_due(1.0))
        self.assertFalse(camera._highres_is_due(1.0))
        camera._highres_client_stopped()

    def test_exposure_uses_one_over_n_shutter_unit(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            camera = CameraStreamer(config_path=Path(directory) / "camera_config.json")
            status = camera.set_exposure({"auto": False, "ev": -0.5, "shutter_denominator": 200})
            self.assertFalse(status["exposure"]["auto"])
            self.assertEqual(status["exposure"]["shutter_denominator"], 200)
            self.assertEqual(status["exposure"]["shutter_us"], 5000)

    def test_webrtc_status_declares_h264_transport(self) -> None:
        stream = WebRTCStreamer(1280, 720, 30, 2_500_000, 8889, "cam")
        stream._media_server_online = lambda: True
        status = stream.status_dict()
        self.assertTrue(status["online"])
        self.assertEqual(status["transport"], "webrtc")
        self.assertEqual(status["resolution"], "1280x720")
        self.assertEqual(status["webrtc_port"], 8889)
        self.assertEqual(status["rtsp_url_template"], "rtsp://{host}:8554/cam")

    def test_dual_webrtc_camera_keeps_low_video_and_highres_jpeg_separate(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            camera = DualStreamCamera(
                video_width=640, video_height=480, video_fps=30, video_bitrate=1_500_000,
                highres_width=1640, highres_height=1232,
                config_path=Path(directory) / "camera_config.json",
            )
            status = camera.set_highres_profile("medium_1640")
            self.assertEqual(status["transport"], "webrtc")
            self.assertEqual(status["resolution"], "640x480")
            self.assertEqual(status["highres_profile"]["resolution"], "1640x1232")
            self.assertEqual(status["highres_profile"]["target_fps"], 2.0)
            self.assertTrue(status["highres_available"])
            self.assertEqual(status["stream_profile"]["gop_frames"], 8)
            self.assertEqual(status["stream_profile"]["keyframe_interval_ms"], 267)
            self.assertEqual(status["stream_profile"]["profile"], "baseline")
            self.assertTrue(status["stream_profile"]["low_latency_mux"])
            self.assertEqual(status["stream_profile"]["camera_buffer_count"], 2)

    def test_dual_webrtc_camera_only_encodes_highres_when_a_client_is_present(self) -> None:
        camera = DualStreamCamera()
        self.assertFalse(camera._highres_has_clients())
        camera._highres_client_started()
        self.assertTrue(camera._highres_has_clients())
        self.assertTrue(camera._highres_wakeup.is_set())
        camera._highres_client_stopped()
        self.assertFalse(camera._highres_has_clients())

    def test_dual_webrtc_camera_persists_configurable_highres_fps(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            config_path = Path(directory) / "camera_config.json"
            first = DualStreamCamera(config_path=config_path)
            status = first.set_highres_fps(5)
            self.assertEqual(status["highres"]["target_fps"], 5.0)
            second = DualStreamCamera(config_path=config_path)
            self.assertEqual(second.status_dict()["highres"]["target_fps"], 5.0)
            with self.assertRaises(ValueError):
                second.set_highres_fps(16)

    @unittest.skipUnless(importlib.util.find_spec("flask"), "Flask is installed on the Raspberry Pi deployment target")
    def test_webrtc_mode_rejects_mjpeg_endpoints(self) -> None:
        from app import create_app

        class Controller:
            pass

        stream = WebRTCStreamer(1280, 720, 30, 2_500_000, 8889, "cam")
        app = create_app(Controller(), stream)
        client = app.test_client()
        self.assertEqual(client.get("/video_feed").status_code, 409)
        self.assertEqual(client.post("/api/camera/mode", json={"mode": "fast_1640"}).status_code, 409)

    @unittest.skipUnless(importlib.util.find_spec("flask"), "Flask is installed on the Raspberry Pi deployment target")
    def test_mjpeg_status_declares_metrics_and_highres_fps_capabilities(self) -> None:
        from app import create_app

        class Controller:
            def status(self):
                return {"arduino_online": False, "serial": False, "action": "STOP", "config": {}}

        app = create_app(Controller(), CameraStreamer())
        response = app.test_client().get("/api/status")
        self.assertEqual(response.status_code, 200)
        body = response.get_json()
        self.assertEqual(body["api_version"], "mjpeg-console-2026-07-16")
        self.assertTrue(body["capabilities"]["system_metrics"])
        self.assertTrue(body["capabilities"]["highres_fps_control"])


if __name__ == "__main__":
    unittest.main()
