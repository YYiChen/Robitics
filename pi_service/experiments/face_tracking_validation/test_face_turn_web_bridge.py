import unittest
from datetime import datetime, timedelta, timezone

from face_turn_web_bridge import DEFAULT_FACE_DEADBAND_NORMALIZED, FaceStopArmer, is_fresh_and_centred


class FaceTurnWebBridgeTests(unittest.TestCase):
    def test_default_stop_zone_is_twenty_percent(self):
        self.assertEqual(DEFAULT_FACE_DEADBAND_NORMALIZED, .20)

    def test_initially_centred_face_must_depart_before_it_can_stop_return(self):
        armer = FaceStopArmer()
        self.assertFalse(armer.should_stop(True))
        self.assertFalse(armer.should_stop(False))
        self.assertTrue(armer.should_stop(True))
        armer.reset()
        self.assertFalse(armer.should_stop(True))

    def test_only_fresh_confident_centred_face_stops_turn(self):
        now = datetime.now(timezone.utc).isoformat()
        face = {"detected": True, "score": .9, "offset_x_normalized": .06, "time": now}
        self.assertTrue(is_fresh_and_centred(face, minimum_score=.5, deadband_normalized=.08, max_age_ms=450))
        face["offset_x_normalized"] = .3
        self.assertFalse(is_fresh_and_centred(face, minimum_score=.5, deadband_normalized=.08, max_age_ms=450))
        face["offset_x_normalized"] = 0.0
        face["time"] = (datetime.now(timezone.utc) - timedelta(seconds=1)).isoformat()
        self.assertFalse(is_fresh_and_centred(face, minimum_score=.5, deadband_normalized=.08, max_age_ms=450))


if __name__ == "__main__":
    unittest.main()
