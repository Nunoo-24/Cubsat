import unittest

from .evaluate_camera_geometry import (evaluate_camera_geometry,
                                       evaluate_camera_pose_grid,
                                       render_report)


class CameraGeometrySweepTests(unittest.TestCase):
    def test_selected_size_depth_cases_match_rendered_detector(self):
        results = evaluate_camera_geometry(
            diameters_m=(0.05, 0.10),
            depths_m=(1.0, 2.0),
            bearings_deg=(-30.0, 0.0, 30.0),
            elevations_deg=(0.0,),
        )
        by_geometry = {
            (row["diameter_m"], row["depth_m"]): row for row in results
        }
        self.assertEqual(by_geometry[(0.05, 1.0)]["valid"], 3)
        self.assertEqual(by_geometry[(0.05, 2.0)]["valid"], 0)
        self.assertEqual(by_geometry[(0.10, 1.0)]["valid"], 0)
        self.assertEqual(by_geometry[(0.10, 2.0)]["valid"], 3)
        self.assertEqual(by_geometry[(0.10, 2.0)]["coherent_pixels_min"], 74)

    def test_vertical_target_motion_matches_image_y_with_documented_sign(self):
        result = evaluate_camera_geometry(
            diameters_m=(0.10,), depths_m=(2.0,), bearings_deg=(0.0,),
            elevations_deg=(-10.0, 0.0, 10.0),
        )[0]
        self.assertEqual(result["valid"], 3)
        self.assertEqual(result["centroid_error_max_px"], 0.0)
        self.assertLessEqual(result["centroid_y_error_max_px"], 0.5)

    def test_rendered_camera_detector_tracks_reference_across_body_poses(self):
        poses = evaluate_camera_pose_grid()
        self.assertEqual(len(poses), 5)
        self.assertTrue(all(item["valid"] for item in poses))
        self.assertEqual(max(item["error_x_px"] for item in poses), 0.0)
        self.assertEqual(max(item["error_y_px"] for item in poses), 0.0)

    def test_report_explains_raster_scope_and_area_counts(self):
        report = render_report(evaluate_camera_geometry(
            diameters_m=(0.10,), depths_m=(2.0,), bearings_deg=(0.0,),
            elevations_deg=(0.0,),
        ))
        self.assertIn("not a physical-camera measurement", report)
        self.assertIn("Raw dark pixels", report)
        self.assertIn("coherent pixels", report)


if __name__ == "__main__":
    unittest.main()
