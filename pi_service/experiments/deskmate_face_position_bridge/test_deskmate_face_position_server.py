from __future__ import annotations

import unittest

import numpy as np

from deskmate_face_position_server import (
    DESKMATE_FACE_CONFIG,
    DEFAULT_LOCAL_BACKEND,
    DEFAULT_SOURCE,
    camera_config,
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

    def test_camera_config_uses_deskmate_network_adapter_for_pi_stream(self) -> None:
        network_source = "http://100.80.46.54:5000/video_feed"
        network = camera_config(network_source)
        self.assertEqual(network.stream_url, network_source)
        self.assertTrue(network.is_network_stream)
        self.assertEqual(network.backend, "auto")

        local = camera_config(DEFAULT_SOURCE)
        self.assertEqual(local.device_index, 1)
        self.assertIsNone(local.stream_url)
        self.assertEqual(local.backend, DEFAULT_LOCAL_BACKEND)

        directshow = camera_config("1", local_backend="dshow")
        self.assertEqual(directshow.backend, "dshow")


if __name__ == "__main__":
    unittest.main()
