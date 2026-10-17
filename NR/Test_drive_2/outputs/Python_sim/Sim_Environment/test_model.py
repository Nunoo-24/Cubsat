import math
import unittest

from .model import ReactionWheel, YawPlant, target_bearing_error, wrap_pi


class YawPlantTests(unittest.TestCase):
    def test_positive_torque_increases_yaw(self):
        plant = YawPlant()
        yaw, rate = plant.step(external_torque_nm=0.01, dt_s=0.01)
        self.assertGreater(yaw, 0.0)
        self.assertGreater(rate, 0.0)

    def test_zero_torque_keeps_stationary_plant_stationary(self):
        plant = YawPlant()
        self.assertEqual(plant.step(), (0.0, 0.0))

    def test_wrap_and_target_error(self):
        self.assertAlmostEqual(wrap_pi(3 * math.pi), -math.pi)
        self.assertAlmostEqual(
            target_bearing_error(math.radians(-179), math.radians(179)),
            math.radians(2),
        )

    def test_rejects_unstable_step_size(self):
        with self.assertRaises(ValueError):
            YawPlant().step(dt_s=0.2)

    def test_wheel_torque_reacts_on_body_and_preserves_angular_momentum(self):
        plant = YawPlant(damping_n_m_per_rad_s=0.0)
        for _ in range(10):
            plant.step(body_wheel_torque_nm=0.005, dt_s=0.01)
        total_momentum = (plant.inertia_kg_m2 * plant.rate_rad_s
                          + plant.wheel.inertia_kg_m2 * plant.wheel.speed_rad_s)
        self.assertGreater(plant.rate_rad_s, 0.0)
        self.assertLess(plant.wheel.speed_rad_s, 0.0)
        self.assertAlmostEqual(total_momentum, 0.0, places=12)

    def test_wheel_motor_torque_and_speed_are_bounded(self):
        wheel = ReactionWheel(inertia_kg_m2=1e-4, max_motor_torque_nm=0.01,
                              max_speed_rad_s=10.0, motor_time_constant_s=0.01,
                              speed_rad_s=9.9)
        body_torque = wheel.step(-0.05, 0.01)
        self.assertLessEqual(abs(wheel.motor_torque_nm), 0.01)
        self.assertLessEqual(abs(wheel.speed_rad_s), 10.0)
        self.assertLess(body_torque, 0.0)

    def test_reaction_wheel_rejects_invalid_direct_step_inputs(self):
        wheel = ReactionWheel()
        for torque, dt_s in ((0.01, 0.0), (0.01, 0.2), (math.nan, 0.01)):
            with self.subTest(torque=torque, dt_s=dt_s), self.assertRaises(ValueError):
                wheel.step(torque, dt_s)
        with self.assertRaises(ValueError):
            wheel.step_speed_target(10.0, 0.0)

    def test_velocity_target_accelerates_wheel_and_reacts_on_body(self):
        plant = YawPlant(damping_n_m_per_rad_s=0.0)
        for _ in range(100):
            plant.step(dt_s=0.01, motor_speed_target_rev_s=10.0)
        self.assertGreater(plant.wheel.speed_rad_s, 0.0)
        self.assertLess(plant.rate_rad_s, 0.0)
        self.assertAlmostEqual(plant.wheel.speed_rad_s,
                               10.0 * 2.0 * math.pi / 60.0)

    def test_external_xyz_torques_act_on_their_gimbal_axes(self):
        plant = YawPlant()
        plant.step(external_torque_nm={"x": 0.01, "y": -0.01, "z": 0.02})
        self.assertGreater(plant.pitch_rate_rad_s, 0.0)
        self.assertLess(plant.rate_rad_s, 0.0)
        self.assertGreater(plant.roll_rate_rad_s, 0.0)

    def test_xyz_torque_inputs_do_not_drive_other_axes_in_decoupled_model(self):
        cases = (
            ({"x": 0.01, "y": 0.0, "z": 0.0}, "pitch_rate_rad_s"),
            ({"x": 0.0, "y": 0.01, "z": 0.0}, "rate_rad_s"),
            ({"x": 0.0, "y": 0.0, "z": 0.01}, "roll_rate_rad_s"),
        )
        for torque, driven_axis in cases:
            with self.subTest(torque=torque):
                plant = YawPlant()
                plant.step(external_torque_nm=torque)
                self.assertGreater(getattr(plant, driven_axis), 0.0)
                for axis in ("rate_rad_s", "roll_rate_rad_s",
                             "pitch_rate_rad_s"):
                    if axis != driven_axis:
                        self.assertEqual(getattr(plant, axis), 0.0)

    def test_roll_and_pitch_pendulum_terms_restore_toward_center(self):
        plant = YawPlant(roll_rad=0.1, pitch_rad=-0.1)
        plant.step(dt_s=0.01)
        self.assertLess(plant.roll_rate_rad_s, 0.0)
        self.assertGreater(plant.pitch_rate_rad_s, 0.0)
        self.assertLess(plant.roll_rad, 0.1)
        self.assertGreater(plant.pitch_rad, -0.1)


if __name__ == "__main__":
    unittest.main()
