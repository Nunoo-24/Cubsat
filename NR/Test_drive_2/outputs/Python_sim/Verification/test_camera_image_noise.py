import unittest

from .evaluate_camera_image_noise import (add_gaussian_intensity_noise,
                                          evaluate_camera_image_noise)


class CameraImageNoiseTests(unittest.TestCase):
    def test_image_noise_is_deterministic_and_keeps_qvga_length(self):
        frame = bytes([190]) * (320 * 240)
        first = add_gaussian_intensity_noise(frame, 20.0, seed=7)
        second = add_gaussian_intensity_noise(frame, 20.0, seed=7)
        self.assertEqual(first, second)
        self.assertEqual(len(first), len(frame))
        self.assertTrue(all(0 <= value <= 255 for value in first))

    def test_clean_target_is_detected_without_blank_scene_false_positive(self):
        result = evaluate_camera_image_noise(
            noise_levels=(0.0,), bearings_deg=(0.0,), seed_count=2
        )["results"][0]
        self.assertEqual(result["target_detections"], 2)
        self.assertEqual(result["target_trials"], 2)
        self.assertEqual(result["no_target_false_positives"], 0)
        self.assertEqual(result["centroid_error_max_px"], 0.0)

    def test_neighbour_filter_rejects_blank_noise_and_retains_target(self):
        result = evaluate_camera_image_noise(
            noise_levels=(0.0, 50.0), bearings_deg=(0.0,), seed_count=3
        )["results"][1]
        self.assertEqual(result["no_target_trials"], 3)
        self.assertEqual(result["no_target_false_positives"], 0)
        self.assertEqual(result["target_detections"], 3)


if __name__ == "__main__":
    unittest.main()
