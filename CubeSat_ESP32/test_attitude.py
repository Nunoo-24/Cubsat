import math
import unittest

from .attitude import YawAttitudeEstimator, wrap_pi


def sensor_sample(yaw, roll=0.0, pitch=0.0):
    gravity = 9.80665
    world_field = (20.0, 0.0, 45.0)
    cy, sy = math.cos(yaw), math.sin(yaw)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cr, sr = math.cos(roll), math.sin(roll)
    bx = cy * world_field[0] - sy * world_field[2]
    by = world_field[1]
    bz = sy * world_field[0] + cy * world_field[2]
    by2 = cp * by + sp * bz
    bz2 = -sp * by + cp * bz
    mag = (cr * bx + sr * by2, -sr * bx + cr * by2, bz2)
    accel = (gravity * sr * cp, gravity * cr * cp, -gravity * sp)
    return accel, mag


class YawAttitudeEstimatorTests(unittest.TestCase):
    def test_tilt_compensated_magnetic_heading_recovers_yaw(self):
        for yaw, roll, pitch in (
                (0.0, 0.0, 0.0),
                (math.radians(15.0), 0.0, 0.0),
                (math.radians(-28.0), math.radians(10.0), math.radians(-7.0))):
            with self.subTest(yaw=yaw, roll=roll, pitch=pitch):
                estimator = YawAttitudeEstimator()
                accel, mag = sensor_sample(yaw, roll, pitch)
                estimate, valid, source = estimator.update(
                    0.0, accel, mag, 0.02)
                self.assertTrue(valid)
                self.assertEqual(source, "MAG_FUSED")
                self.assertAlmostEqual(wrap_pi(estimate - yaw), 0.0, places=10)

    def test_magnetic_reference_corrects_gyro_bias_over_one_minute(self):
        estimator = YawAttitudeEstimator()
        accel, mag = sensor_sample(0.0)
        estimate = 0.0
        for _ in range(3000):
            estimate, valid, _ = estimator.update(0.001, accel, mag, 0.02)
        self.assertTrue(valid)
        self.assertLess(abs(math.degrees(estimate)), 0.2)

    def test_without_magnetometer_estimator_reports_relative_gyro_only_yaw(self):
        estimator = YawAttitudeEstimator()
        accel, mag = sensor_sample(0.0)
        _, valid, source = estimator.update(0.01, accel, mag, 0.02,
                                            magnetic_valid=False)
        self.assertTrue(valid)
        self.assertEqual(source, "GYRO_ONLY")

    def test_invalid_sample_does_not_initialize_heading(self):
        estimator = YawAttitudeEstimator()
        estimate, valid, source = estimator.update(
            0.0, (0.0, 0.0, 0.0), (20.0, 0.0, 45.0), 0.02)
        self.assertFalse(valid)
        self.assertEqual(source, "INVALID")
        self.assertEqual(estimate, 0.0)
        self.assertFalse(estimator.initialized)

    def test_nonfinite_acceleration_rejects_sample_without_gyro_integration(self):
        estimator = YawAttitudeEstimator()
        accel, mag = sensor_sample(0.0)
        _, valid, _ = estimator.update(0.0, accel, mag, 0.02,
                                       magnetic_valid=False)
        self.assertTrue(valid)
        yaw_before = estimator.yaw_rad

        estimate, valid, source = estimator.update(
            1.0, (math.nan, accel[1], accel[2]), mag, 0.02)

        self.assertFalse(valid)
        self.assertEqual(source, "INVALID")
        self.assertEqual(estimate, yaw_before)
        self.assertEqual(estimator.yaw_rad, yaw_before)

    def test_sil_attitude_telemetry_matches_known_simulated_yaw(self):
        from Sim_Environment.main import Environment
        from .sil import OnboardSoftware

        env = Environment()
        env.plant.yaw_rad = math.radians(12.0)
        onboard = OnboardSoftware()
        onboard.mode = "SAFE"
        telemetry = onboard.process_observation(env.observation())
        self.assertTrue(telemetry["yaw_estimate_valid"])
        self.assertEqual(telemetry["yaw_estimate_source"], "MAG_FUSED")
        self.assertAlmostEqual(
            math.degrees(telemetry["yaw_estimate_rad"]), 12.0, places=5)


if __name__ == "__main__":
    unittest.main()
