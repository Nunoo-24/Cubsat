from pathlib import Path
import unittest

from .evaluate_scenarios import (evaluate_yaw_attitude_estimator,
                                 evaluate_passive_roll_pitch_decay,
                                 evaluate_tilt_compensated_yaw_grid,
                                 evaluate_tilted_camera_projection_grid,
                                 evaluate_synthetic_camera_detector_sweep,
                                 evaluate_sil_sensor_noise_latency_screen,
                                 evaluate_sil_hold_seed_sensitivity,
                                 evaluate_yaw_parameter_corner_sweep,
                                 evaluate_full_direction_tilted_pointing,
                                 evaluate_yaw_step_45_degrees,
                                 evaluate_wheel_speed_torque_bound,
                                 optimistic_wheel_speed_after_half_second)


class MotorSpeedEvidenceTests(unittest.TestCase):
    def test_current_placeholder_wheel_model_cannot_reach_3000_rpm_in_half_second(self):
        rpm = optimistic_wheel_speed_after_half_second()
        self.assertGreater(rpm, 430.0)
        self.assertLess(rpm, 432.0)
        self.assertLess(rpm, 3000.0)

    def test_physical_velocity_torque_cap_bounds_the_model_inertia(self):
        result = evaluate_wheel_speed_torque_bound()
        self.assertAlmostEqual(result["required_ideal_torque_nm"],
                               0.062831853, places=8)
        default = result["caps"]["default"]
        maximum = result["caps"]["kconfig_max"]
        self.assertAlmostEqual(default["torque_nm"], 0.001)
        self.assertAlmostEqual(maximum["torque_nm"], 0.020)
        self.assertAlmostEqual(maximum["ideal_speed_rpm"], 954.92966,
                               places=4)
        self.assertAlmostEqual(maximum["max_inertia_kg_m2"],
                               3.18309886e-5, places=12)
        kconfig = (Path(__file__).resolve().parents[1] /
                   "CubeSat_ESP32/firmware/Kconfig.projbuild")
        source = (Path(__file__).resolve().parents[1] /
                  "CubeSat_ESP32/firmware/main/moteus_uart.c")
        kconfig_text = kconfig.read_text(encoding="utf-8")
        torque_config = kconfig_text.split(
            "config CUBESAT_MOTEUS_MAX_TORQUE_MNM", 1)[1].split(
                "endmenu", 1)[0]
        self.assertIn("range 1 20", torque_config)
        self.assertIn("default 1", torque_config)
        velocity_builder = source.read_text(encoding="utf-8").split(
            "static bool make_velocity_frame", 1)[1].split(
                "\nstatic ", 1)[0]
        self.assertIn("CONFIG_CUBESAT_MOTEUS_MAX_TORQUE_MNM / 1000.0f",
                      velocity_builder)


class AttitudeEstimateEvidenceTests(unittest.TestCase):
    def test_yaw_error_and_drift_meet_model_target_with_synthetic_field(self):
        peak_error_deg, drift_deg = evaluate_yaw_attitude_estimator()
        self.assertLess(peak_error_deg, 2.0)
        self.assertLess(drift_deg, 2.0)

    def test_tilt_compensated_yaw_matches_truth_across_combined_pose_grid(self):
        peak_error_deg, pose_count = evaluate_tilt_compensated_yaw_grid()
        self.assertEqual(pose_count, 5)
        self.assertLessEqual(peak_error_deg, 0.000001)

    def test_camera_projection_matches_rotation_matrix_across_pose_grid(self):
        max_pixel_error, pose_count = evaluate_tilted_camera_projection_grid()
        self.assertEqual(pose_count, 5)
        self.assertEqual(max_pixel_error, 0)

    def test_full_direction_control_centres_tilted_pointing_case(self):
        result = evaluate_full_direction_tilted_pointing()
        self.assertEqual(result["sample_count"], 300)
        self.assertEqual(result["target_valid_count"], 300)
        self.assertLess(abs(result["horizontal_error_deg"]), 2.0)
        self.assertLess(abs(result["vertical_error_deg"]), 2.0)
        self.assertLess(abs(result["final_pitch_deg"]), 1.0)
        self.assertLess(abs(result["final_roll_deg"]), 1.0)
        self.assertLess(abs(result["final_yaw_deg"]), 2.0)

    def test_commanded_45_degree_yaw_step_settles_before_timeout(self):
        settle_time_s, completion_time_s = evaluate_yaw_step_45_degrees()
        self.assertIsNotNone(settle_time_s)
        self.assertIsNotNone(completion_time_s)
        self.assertLess(settle_time_s, 10.0)


class CameraDetectorEvidenceTests(unittest.TestCase):
    def test_default_marker_is_detected_across_nominal_yaw_fov(self):
        valid_count, sample_count, max_centroid_error_px = (
            evaluate_synthetic_camera_detector_sweep())
        self.assertEqual((valid_count, sample_count), (13, 13))
        self.assertLessEqual(max_centroid_error_px, 3.0)


class PassiveAxisEvidenceTests(unittest.TestCase):
    def test_provisional_roll_and_pitch_modes_lose_energy_without_disturbance(self):
        result = evaluate_passive_roll_pitch_decay()
        self.assertAlmostEqual(result["duration_s"], 20.0)
        self.assertLess(result["roll_energy_ratio"], 1.0)
        self.assertLess(result["pitch_energy_ratio"], 1.0)
        self.assertLess(abs(result["final_roll_rad"]),
                        abs(result["initial_roll_rad"]))
        self.assertLess(abs(result["final_pitch_rad"]),
                        abs(result["initial_pitch_rad"]))


class YawParameterSensitivityTests(unittest.TestCase):
    def test_acquisition_and_nudge_meet_timing_targets_at_parameter_corners(self):
        result = evaluate_yaw_parameter_corner_sweep()
        self.assertEqual(result["total_corner_count"], 32)
        self.assertEqual(result["passing_corner_count"], 32)
        self.assertLess(result["worst_acquisition_s"], 6.0)
        self.assertLess(result["worst_nudge_recovery_s"], 4.0)


class ObservationNoiseLatencyTests(unittest.TestCase):
    def test_control_response_screen_reports_deterministic_stress_results(self):
        result = evaluate_sil_sensor_noise_latency_screen()
        self.assertEqual(result["case_count"], 81)
        self.assertEqual(result["acquisition_pass_count"], 81)
        self.assertEqual(result["nudge_pass_count"], 81)
        self.assertEqual(result["hold_pass_count"], 80)
        self.assertGreater(result["worst_hold_peak_deg"], 2.0)
        self.assertLess(result["worst_hold_peak_deg"], 2.1)

    def test_higher_provisional_proportional_gain_passes_assumed_screen(self):
        candidate = evaluate_sil_sensor_noise_latency_screen(
            kp_nm_per_rad=0.10, kd_nm_per_rad_s=0.04)
        self.assertEqual(candidate["case_count"], 81)
        self.assertEqual(candidate["acquisition_pass_count"], 81)
        self.assertEqual(candidate["nudge_pass_count"], 81)
        self.assertEqual(candidate["hold_pass_count"], 81)
        self.assertLess(candidate["worst_hold_peak_deg"], 2.0)

    def test_neighboring_gain_pair_results_match_reported_sensitivity(self):
        lower = evaluate_sil_sensor_noise_latency_screen(
            kp_nm_per_rad=0.10, kd_nm_per_rad_s=0.05)
        stronger = evaluate_sil_sensor_noise_latency_screen(
            kp_nm_per_rad=0.14, kd_nm_per_rad_s=0.05)
        self.assertEqual(lower["hold_pass_count"], 80)
        self.assertGreater(lower["worst_hold_peak_deg"], 2.0)
        self.assertLess(lower["worst_hold_peak_deg"], 2.1)
        self.assertEqual(stronger["hold_pass_count"], 81)
        self.assertLess(stronger["worst_hold_peak_deg"], 1.95)

    def test_gain_candidate_does_not_improve_critical_corner_seed_tail(self):
        baseline = evaluate_sil_hold_seed_sensitivity(0.08, 0.04, 50)
        candidate = evaluate_sil_hold_seed_sensitivity(0.14, 0.05, 50)
        self.assertEqual(baseline["hold_pass_count"], 49)
        self.assertEqual(candidate["hold_pass_count"], 49)
        self.assertLess(candidate["median_hold_peak_deg"],
                        baseline["median_hold_peak_deg"])
        self.assertGreater(candidate["p95_hold_peak_deg"],
                           baseline["p95_hold_peak_deg"])
        self.assertGreater(candidate["worst_hold_peak_deg"],
                           baseline["worst_hold_peak_deg"])


if __name__ == "__main__":
    unittest.main()
