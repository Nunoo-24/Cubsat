import unittest

from .evaluate_axis_control import (
    evaluate_combined_camera_pointing,
    evaluate_camera_pointing_axes,
    evaluate_camera_roll_both_directions,
    evaluate_point_target_roll_observability,
    evaluate_simultaneous_stabilization,
    evaluate_stabilization_axes,
)


class ThreeAxisControlEvidenceTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.stabilization = evaluate_stabilization_axes()
        cls.simultaneous = evaluate_simultaneous_stabilization()
        cls.pointing = evaluate_camera_pointing_axes()
        cls.roll_cue = evaluate_point_target_roll_observability()
        cls.camera_roll_both = evaluate_camera_roll_both_directions()
        cls.camera_roll = cls.camera_roll_both[0]
        cls.combined = evaluate_combined_camera_pointing()

    def test_each_axis_damps_after_target_loss_within_simulated_authority(self):
        self.assertEqual([item["axis"] for item in self.stabilization],
                         ["X / pitch", "Y / yaw", "Z / roll"])
        for item in self.stabilization:
            with self.subTest(axis=item["axis"]):
                self.assertTrue(item["target_lost"])
                self.assertEqual(item["mode"], "RATE_DAMP")
                self.assertEqual(item["fault"], "")
                self.assertIsNotNone(item["rate_settle_after_pulse_s"])
                self.assertLess(item["rate_settle_after_pulse_s"], 2.5)
                self.assertLess(item["peak_excursion_deg"], 5.0)
        self.assertIsNotNone(self.stabilization[0]["attitude_settle_after_pulse_s"])
        self.assertIsNone(self.stabilization[1]["attitude_settle_after_pulse_s"])
        self.assertIsNotNone(self.stabilization[2]["attitude_settle_after_pulse_s"])

    def test_all_three_sensor_loops_stabilize_together_during_target_loss(self):
        result = self.simultaneous
        self.assertTrue(result["target_lost"])
        self.assertEqual(result["mode"], "RATE_DAMP")
        self.assertEqual(result["fault"], "")
        self.assertIsNotNone(result["settle_after_pulse_s"])
        self.assertLess(result["settle_after_pulse_s"], 2.5)
        self.assertTrue(all(value > 0.0
                            for value in result["torque_peaks_nm"].values()))
        self.assertLess(abs(result["final_yaw_rate_deg_s"]), 0.5)
        self.assertLess(abs(result["final_pitch_deg"]), 1.0)
        self.assertLess(abs(result["final_pitch_rate_deg_s"]), 0.5)
        self.assertLess(abs(result["final_roll_deg"]), 1.0)
        self.assertLess(abs(result["final_roll_rate_deg_s"]), 0.5)

    def test_camera_steers_yaw_and_pitch_to_center(self):
        self.assertEqual([item["axis"] for item in self.pointing],
                         ["Yaw / image X (+)", "Yaw / image X (−)",
                          "Pitch / image Y (+)", "Pitch / image Y (−)"])
        for item in self.pointing:
            with self.subTest(axis=item["axis"]):
                self.assertIsNotNone(item["settle_to_2deg_s"])
                self.assertLess(item["settle_to_2deg_s"], 4.0)
                self.assertLess(item["final_error_deg"], 2.0)
                self.assertEqual(item["valid_samples"], item["sample_count"])
        self.assertLess(self.pointing[0]["initial_error_x_rad"] *
                        self.pointing[1]["initial_error_x_rad"], 0.0)
        self.assertLess(self.pointing[2]["initial_error_y_rad"] *
                        self.pointing[3]["initial_error_y_rad"], 0.0)
        for item in self.pointing[:2]:
            self.assertLess(abs(item["initial_error_y_rad"]), 1e-5)
        for item in self.pointing[2:]:
            self.assertLess(abs(item["initial_error_x_rad"]), 1e-5)

    def test_oriented_camera_marker_controls_roll_without_accelerometer_roll(self):
        self.assertTrue(self.camera_roll["accelerometer_roll_hidden"])
        self.assertEqual(self.camera_roll["initial_roll_reference_source"], "CAMERA")
        self.assertEqual(self.camera_roll["final_roll_reference_source"], "CAMERA")
        self.assertLess(abs(self.camera_roll["initial_camera_roll_estimate_deg"] - 15.0), 2.0)
        self.assertLess(self.camera_roll["initial_roll_torque_nm"], 0.0)
        self.assertIsNotNone(self.camera_roll["settle_to_1deg_0_5deg_s"])
        self.assertLess(self.camera_roll["settle_to_1deg_0_5deg_s"], 10.0)
        self.assertLess(abs(self.camera_roll["final_roll_deg"]), 1.0)
        self.assertLess(abs(self.camera_roll["final_roll_rate_deg_s"]), 0.5)

    def test_oriented_camera_marker_corrects_both_roll_directions(self):
        self.assertEqual([row["initial_roll_deg"]
                          for row in self.camera_roll_both], [15.0, -15.0])
        for row in self.camera_roll_both:
            with self.subTest(initial_roll_deg=row["initial_roll_deg"]):
                self.assertTrue(row["accelerometer_roll_hidden"])
                self.assertEqual(row["initial_roll_reference_source"], "CAMERA")
                self.assertEqual(row["final_roll_reference_source"], "CAMERA")
                self.assertLess(
                    abs(row["initial_camera_roll_estimate_deg"] -
                        row["initial_roll_deg"]), 2.0)
                self.assertLess(row["initial_roll_torque_nm"] *
                                row["initial_roll_deg"], 0.0)
                self.assertIsNotNone(row["settle_to_1deg_0_5deg_s"])
                self.assertLess(row["settle_to_1deg_0_5deg_s"], 10.0)
                self.assertLess(abs(row["final_roll_deg"]), 1.0)
                self.assertLess(abs(row["final_roll_rate_deg_s"]), 0.5)

    def test_point_only_camera_has_no_roll_cue_and_imu_levels_roll(self):
        self.assertTrue(self.roll_cue["target_valid"])
        self.assertEqual(self.roll_cue["pixel_error_x"], 0)
        self.assertEqual(self.roll_cue["pixel_error_y"], 0)
        self.assertLess(self.roll_cue["imu_roll_torque_nm"], 0.0)

    def test_camera_centres_and_controls_roll_from_combined_pose(self):
        result = self.combined
        self.assertGreater(result["initial_image_error_deg"], 5.0)
        self.assertEqual(result["initial_roll_reference_source"], "CAMERA")
        self.assertIsNotNone(result["settle_to_all_axes_s"])
        self.assertLess(result["settle_to_all_axes_s"], 4.0)
        self.assertLess(result["final_image_error_deg"], 2.0)
        self.assertLess(abs(result["final_roll_deg"]), 1.0)
        self.assertLess(abs(result["final_roll_rate_deg_s"]), 0.5)
        self.assertEqual(result["final_roll_reference_source"], "CAMERA")
        self.assertTrue(result["final_target_valid"])


if __name__ == "__main__":
    unittest.main()
