from __future__ import annotations

import unittest

import numpy as np

from pathlib import Path

PROJECT_ROOT = Path(__file__).resolve().parents[3]

from deskmate_face_position_server import (
    DEFAULT_CENTER_DEADBAND_NORMALIZED,
    DEFAULT_DETECTOR_SCORE_THRESHOLD,
    DESKMATE_FACE_CONFIG,
    DEFAULT_LOCAL_BACKEND,
    DEFAULT_MINIMUM_FACE_SIZE_PX,
    DEFAULT_SERVER_PORT,
    DEFAULT_SOURCE,
    _command_runs_this_server,
    _configured_port,
    annotate_face_preview,
    camera_config,
    face_identity_config,
    face_payload,
    select_primary_feature,
)
from poker_dealer.domain import ColorSpace, FramePacket
from poker_dealer.perception.identity import (
    DetectedFaceFeature,
    FaceFrameEvidence,
    FaceIdentityConfig,
    OpenCvFaceIdentityAdapter,
)


def feature(
    bbox: tuple[int, int, int, int],
    score: float,
    seed: int,
) -> DetectedFaceFeature:
    embedding = np.zeros(8, dtype=np.float32)
    embedding[seed] = 1.0
    embedding.setflags(write=False)
    return DetectedFaceFeature(
        observed_at_ns=10,
        bbox_xywh=bbox,
        detection_score=score,
        embedding=embedding,
    )


class DeskMateFacePositionServerTests(unittest.TestCase):
    def test_single_instance_match_is_limited_to_same_script_and_port(self) -> None:
        script = (
            "pi_service/experiments/deskmate_face_position_bridge/"
            "deskmate_face_position_server.py"
        )
        command = ["python.exe", script, "--port", "5060"]
        self.assertEqual(_configured_port(command), 5060)
        self.assertTrue(_command_runs_this_server(command, str(PROJECT_ROOT)))
        self.assertFalse(
            _command_runs_this_server(
                ["python.exe", "somewhere/face_position_server.py"],
                str(PROJECT_ROOT),
            )
        )

    def test_single_instance_port_parser_supports_default_and_equals_form(self) -> None:
        self.assertEqual(
            _configured_port(["python.exe", "server.py"]),
            DEFAULT_SERVER_PORT,
        )
        self.assertEqual(
            _configured_port(["python.exe", "server.py", "--port=5061"]),
            5061,
        )

    def test_submodule_assets_verify_and_official_models_load(self) -> None:
        config = FaceIdentityConfig.from_json(DESKMATE_FACE_CONFIG)
        detector_hash, embedder_hash = config.verify_assets()
        self.assertEqual(detector_hash, config.detector.sha256)
        self.assertEqual(embedder_hash, config.embedder.sha256)

        image = np.zeros((240, 320, 3), dtype=np.uint8)
        image.setflags(write=False)
        frame = FramePacket(
            sequence_id=0,
            captured_at_ns=10,
            source_id="test",
            device_index=0,
            width=320,
            height=240,
            color_space=ColorSpace.BGR,
            nominal_fps=30.0,
            dropped_before=0,
            image=image,
        )
        evidence = OpenCvFaceIdentityAdapter(config).analyze(frame)
        self.assertEqual(evidence.detected_face_count, 0)
        self.assertEqual(evidence.features, ())

    def test_largest_face_is_primary_and_payload_matches_existing_bridge(self) -> None:
        small = feature((10, 20, 40, 40), 0.99, 0)
        large = feature((200, 100, 100, 120), 0.91, 1)
        evidence = FaceFrameEvidence(
            observed_at_ns=10,
            detected_face_count=2,
            low_quality_face_count=0,
            features=(small, large),
            inference_latency_ms=7.25,
        )
        self.assertIs(select_primary_feature(evidence), large)

        payload = face_payload(
            evidence,
            frame_index=4,
            frame_width=640,
            frame_height=480,
            source=DEFAULT_SOURCE,
        )
        self.assertTrue(payload["detected"])
        self.assertEqual(payload["center_x"], 250.0)
        self.assertEqual(payload["offset_x"], -70.0)
        self.assertEqual(payload["offset_x_normalized"], -0.2188)
        self.assertEqual(payload["score"], 0.91)
        self.assertEqual(payload["usable_face_count"], 2)
        self.assertEqual(payload["model"], "deskmate-opencv-yunet-sface")

    def test_no_usable_face_is_fail_closed(self) -> None:
        evidence = FaceFrameEvidence(
            observed_at_ns=10,
            detected_face_count=1,
            low_quality_face_count=1,
            features=(),
            inference_latency_ms=3.0,
        )
        payload = face_payload(
            evidence,
            frame_index=1,
            frame_width=640,
            frame_height=480,
            source=DEFAULT_SOURCE,
        )
        self.assertFalse(payload["detected"])
        self.assertEqual(payload["score"], 0.0)
        self.assertIsNone(payload["offset_x_normalized"])

    def test_preview_draws_face_box_and_exact_center_gate(self) -> None:
        primary = feature((140, 70, 80, 100), 0.94, 0)
        evidence = FaceFrameEvidence(
            observed_at_ns=10,
            detected_face_count=1,
            low_quality_face_count=0,
            features=(primary,),
            inference_latency_ms=4.0,
        )
        payload = face_payload(
            evidence,
            frame_index=2,
            frame_width=400,
            frame_height=240,
            source=DEFAULT_SOURCE,
        )
        image = np.zeros((240, 400, 3), dtype=np.uint8)
        annotated = annotate_face_preview(image, evidence, payload)

        self.assertEqual(annotated.shape, image.shape)
        self.assertFalse(np.shares_memory(annotated, image))
        self.assertGreater(np.count_nonzero(annotated), 0)
        gate_half_width = int(400 * DEFAULT_CENTER_DEADBAND_NORMALIZED / 2)
        self.assertGreater(np.count_nonzero(annotated[:, 200 - gate_half_width]), 0)
        self.assertGreater(np.count_nonzero(annotated[:, 200 + gate_half_width]), 0)

    def test_camera_config_uses_deskmate_network_adapter_for_pi_stream(self) -> None:
        network_source = "http://100.80.46.54:5000/video_feed"
        network = camera_config(network_source)
        self.assertEqual(network.stream_url, network_source)
        self.assertTrue(network.is_network_stream)
        self.assertEqual(network.backend, "auto")

        local = camera_config("1")
        self.assertEqual(local.device_index, 1)
        self.assertIsNone(local.stream_url)
        self.assertEqual(local.backend, DEFAULT_LOCAL_BACKEND)

        directshow = camera_config("1", local_backend="dshow")
        self.assertEqual(directshow.backend, "dshow")

    def test_car_following_thresholds_override_only_detector_options(self) -> None:
        config = face_identity_config()
        self.assertEqual(
            config.detector_options["score_threshold"],
            DEFAULT_DETECTOR_SCORE_THRESHOLD,
        )
        self.assertEqual(
            config.detector_options["minimum_face_size_px"],
            DEFAULT_MINIMUM_FACE_SIZE_PX,
        )
        self.assertEqual(
            DEFAULT_SOURCE,
            "http://100.93.97.117:4747/video",
        )
        config.verify_assets()


if __name__ == "__main__":
    unittest.main()
