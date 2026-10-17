import unittest

from .evaluate_camera_detector_timing import benchmark_camera_detector


class CameraDetectorTimingTests(unittest.TestCase):
    def test_benchmark_covers_normal_noisy_and_dense_frames(self):
        report = benchmark_camera_detector(iterations=2)
        cases = {row["case"] for row in report["results"]}
        self.assertEqual(cases, {
            "clean target", "σ=40 target", "σ=50 target", "blank scene",
            "all dark", "checkerboard", "half dark",
        })
        self.assertEqual(report["width_px"], 320)
        self.assertEqual(report["height_px"], 240)
        self.assertTrue(all(row["median_ms"] >= 0
                            for row in report["results"]))


if __name__ == "__main__":
    unittest.main()
