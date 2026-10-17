import unittest

from .evaluate_camera_transport import (evaluate_camera_transport,
                                        render_report)


class CameraTransportEvaluationTests(unittest.TestCase):
    def test_scene_sweep_reconstructs_within_transfer_bound(self):
        result = evaluate_camera_transport(timing_iterations=3)
        self.assertEqual(result["scene_count"], 60)
        self.assertTrue(result["default_round_trip_exact"])
        self.assertLessEqual(result["scene_payload_max_bytes"],
                             result["max_encoded_bytes"])
        self.assertEqual(result["default_payload_bytes"], 91)

    def test_rendered_report_labels_host_only_timing_and_transport_limits(self):
        report = render_report(timing_iterations=2)
        self.assertIn("60 frames", report)
        self.assertIn("does not", report)
        self.assertIn("does not\nmeasure HTTP, Wi-Fi", report)
        self.assertIn("Camera_Transport_Results.md", report)


if __name__ == "__main__":
    unittest.main()
