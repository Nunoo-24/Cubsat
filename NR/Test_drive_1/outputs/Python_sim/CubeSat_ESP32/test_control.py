import json
import math
import unittest

from .control import PitchRollController, YawController
from .sil import OnboardSoftware, PERIOD_S, _next_cycle_deadline
from Sim_Environment.main import Environment
from Sim_Environment.model import YawPlant, target_bearing_error


class SILSchedulerTests(unittest.TestCase):
    def test_sil_cycle_scheduler_keeps_period_and_rebases_after_overrun(self):
        self.assertAlmostEqual(_next_cycle_deadline(1.0, 1.005),
                               1.0 + PERIOD_S)
        self.assertAlmostEqual(_next_cycle_deadline(1.0, 1.05), 1.05)


class PitchRollControllerTests(unittest.TestCase):
    def test_camera_vertical_error_drives_pitch_with_project_axis_sign(self):
        controller = PitchRollController()
        # Positive image-down error means target below centre: positive X
        # (pitch) torque moves the camera boresight down in this body frame.
        out = controller.update(0.1, 0.0, 0.0, 0.0, 0.0,
                                target_valid=True)
        self.assertGreater(out.pitch_body_torque_nm, 0.0)
        self.assertEqual(out.roll_body_torque_nm, 0.0)

    def test_roll_leveling_torque_is_bounded_and_reducing_error(self):
        out = PitchRollController().update(
            None, 0.0, math.radians(10), 0.0, 0.0, target_valid=False)
        self.assertLess(out.roll_body_torque_nm, 0.0)
        self.assertLessEqual(abs(out.roll_body_torque_nm), 0.05)

    def test_pitch_roll_control_fails_closed_on_stale_samples(self):
        out = PitchRollController().update(
            0.1, 0.0, 0.0, 0.0, 0.0, observation_age_s=0.2)
        self.assertEqual(out.fault, "STALE_OBSERVATION")
        self.assertEqual((out.pitch_body_torque_nm, out.roll_body_torque_nm),
                         (0.0, 0.0))

    def test_sil_centres_target_above_on_pitch_and_levels_roll(self):
        env = Environment()
        env.apply({"target_xyz_m": {"x": 0.0, "y": 0.5, "z": 2.0}})
        onboard = OnboardSoftware()
        initial_pitch = env.plant.pitch_rad
        for _ in range(1000):
            observation = env.observation()
            frame = env.camera_frame_snapshot(observation["sequence"])
            output = onboard.process_observation(observation,
                                                camera_frame=frame)
            env.accept_actuator(output)
            env.advance(dt_s=0.01)
        final = env.snapshot()
        self.assertNotEqual(env.plant.pitch_rad, initial_pitch)
        self.assertLess(abs(final["camera_truth_error_y_rad"]),
                        math.radians(2.0))
        self.assertTrue(output["target_valid"])
        self.assertLess(abs(env.plant.roll_rad), math.radians(1.0))
        self.assertGreater(abs(output["pitch_body_torque_nm"]), 0.0)
        self.assertIn("virtual_axis_torque_nm", output)

    def test_sil_recovers_quickly_from_moderate_roll_axis_torque(self):
        env = Environment()
        env.apply({"external_torque_nm": {
            "x": 0.0, "y": 0.0, "z": 0.05}, "duration_s": 0.5})
        onboard = OnboardSoftware()
        first_in_band_s = None
        settle_s = None
        peak_roll_rad = 0.0
        for _ in range(400):
            observation = env.observation()
            frame = env.camera_frame_snapshot(observation["sequence"])
            output = onboard.process_observation(observation,
                                                 camera_frame=frame)
            env.accept_actuator(output)
            env.advance(dt_s=0.01)
            peak_roll_rad = max(peak_roll_rad, abs(env.plant.roll_rad))
            within_band = (
                env.plant.time_s >= 0.5 and
                abs(env.plant.roll_rad) < math.radians(1.0) and
                abs(env.plant.roll_rate_rad_s) < math.radians(0.5)
            )
            if within_band:
                if first_in_band_s is None:
                    first_in_band_s = env.plant.time_s
                if env.plant.time_s - first_in_band_s >= 0.5:
                    settle_s = first_in_band_s - 0.5
                    break
            else:
                first_in_band_s = None

        self.assertIsNotNone(settle_s)
        self.assertLess(settle_s, 2.0)
        self.assertLess(peak_roll_rad, math.radians(5.0))
        self.assertLessEqual(abs(output["roll_body_torque_nm"]), 0.05)

class YawControllerTests(unittest.TestCase):
    def test_sil_rejects_noninteger_or_unsafe_command_ids(self):
        onboard = OnboardSoftware()
        for command_id in (True, 1.5, 1 << 53):
            with self.subTest(command_id=command_id), self.assertRaises(ValueError):
                onboard.receive_command({"command_id": command_id,
                                         "command": "PING"})

    def test_sil_requires_exact_numeric_environment_protocol_version(self):
        base = {
            "protocol_version": 1,
            "message_type": "environment_observation",
            "sensors": {},
        }
        for invalid_version in (True, 1.5, 2, "1"):
            packet = {**base, "protocol_version": invalid_version}
            with self.subTest(version=invalid_version), self.assertRaisesRegex(
                    ValueError, "protocol version"):
                OnboardSoftware().process_observation(packet)

        packet = {**base, "protocol_version": 1.0}
        # Version 1 encoded as an integral JSON number is accepted by cJSON too.
        output = OnboardSoftware().process_observation(packet)
        self.assertEqual(output["protocol_version"], 1)

    def test_sil_rejects_invalid_mode_with_firmware_reason(self):
        onboard = OnboardSoftware()
        onboard.receive_command({"command_id": 1, "command": "SET_MODE",
                                 "arguments": {"mode": "FLIGHT"}})
        self.assertEqual(onboard.pending_ack["status"], "REJECTED")
        self.assertEqual(onboard.pending_ack["reason"], "INVALID_MODE")

    def test_sil_rejects_extra_speed_test_arguments_like_firmware(self):
        onboard = OnboardSoftware()
        onboard.mode = "SAFE"
        onboard.receive_command({
            "command_id": 1,
            "command": "SET_WHEEL_SPEED",
            "arguments": {"target_rev_s": 10.0, "duration_s": 1.0,
                          "unreviewed_override": True},
        })
        self.assertEqual(onboard.mode, "SAFE")
        self.assertEqual(onboard.pending_ack["status"], "REJECTED")
        self.assertEqual(onboard.pending_ack["reason"],
                         "INVALID_SPEED_TEST_ARGUMENTS")

    @staticmethod
    def run_environment_step(env, onboard):
        observation = env.observation()
        frame = env.camera_frame_snapshot(observation["sequence"])
        output = onboard.process_observation(observation, camera_frame=frame)
        env.plant.step(body_wheel_torque_nm=output["body_torque_nm"], dt_s=0.02)
        error_deg = abs(math.degrees(env.snapshot()["target_error_rad"]))
        return error_deg, output

    def test_closed_loop_reduces_30_degree_error(self):
        plant = YawPlant()
        controller = YawController()
        target = math.radians(30)
        for _ in range(600):
            error = target_bearing_error(target, plant.yaw_rad)
            result = controller.update(error, plant.rate_rad_s)
            plant.step(body_wheel_torque_nm=result.body_torque_nm)
        self.assertLess(abs(target_bearing_error(target, plant.yaw_rad)),
                        math.radians(1.0))
        self.assertEqual(result.mode, "TRACK")

    def test_lost_target_only_rate_damps(self):
        output = YawController().update(None, 0.5, target_valid=False)
        self.assertEqual(output.mode, "RATE_DAMP")
        self.assertAlmostEqual(output.body_torque_nm, -0.02)

    def test_sil_rate_damps_when_fresh_camera_frame_has_no_target(self):
        result = OnboardSoftware().process_observation({
            "protocol_version": 1,
            "message_type": "environment_observation",
            "simulation_time_s": 0.0,
            "sensors": {
                "camera": {"valid": False, "age_s": 0.0},
                "imu": {
                    "gyro_rad_s": {"x": 0.0, "y": 0.5, "z": 0.0},
                    "accel_m_s2": {"x": 0.0, "y": 0.0, "z": 9.80665},
                    "age_s": 0.0,
                },
            },
        }, now=1.0)

        self.assertEqual(result["mode"], "RATE_DAMP")
        self.assertEqual(result["system_state"], "NOMINAL")
        self.assertEqual(result["fault"], "")
        self.assertAlmostEqual(result["body_torque_nm"], -0.02)
        self.assertEqual(result["camera_age_s"], 0.0)

    def test_stale_observation_fails_closed(self):
        output = YawController().update(0.2, 0.0, observation_age_s=0.2)
        self.assertEqual(output.mode, "FAULT")
        self.assertEqual(output.body_torque_nm, 0.0)

    def test_sil_reports_camera_and_imu_sample_ages_independently(self):
        packet = OnboardSoftware().process_observation({
            "protocol_version": 1,
            "message_type": "environment_observation",
            "simulation_time_s": 0.0,
            "sensors": {
                "camera": {"valid": True, "pixel_error_x": 0,
                           "focal_length_x_px": 277, "age_s": 0.04},
                "imu": {"gyro_rad_s": {"x": 0.0, "y": 0.0, "z": 0.0},
                        "accel_m_s2": {"x": 0.0, "y": 0.0, "z": 9.8},
                        "age_s": 0.01},
            },
        })
        self.assertEqual(packet["camera_age_s"], 0.04)
        self.assertEqual(packet["imu_age_s"], 0.01)
        self.assertEqual(packet["system_state"], "NOMINAL")

    def test_sil_fails_closed_on_negative_age_even_if_other_sensor_is_fresh(self):
        packet = OnboardSoftware().process_observation({
            "protocol_version": 1,
            "message_type": "environment_observation",
            "simulation_time_s": 0.0,
            "sensors": {
                "camera": {"valid": True, "pixel_error_x": 0,
                           "focal_length_x_px": 277, "age_s": 0.0},
                "imu": {"gyro_rad_s": {"x": 0.0, "y": 0.0, "z": 0.0},
                        "accel_m_s2": {"x": 0.0, "y": 0.0, "z": 9.8},
                        "age_s": -0.01},
            },
        })
        self.assertEqual(packet["system_state"], "FAULT")
        self.assertEqual(packet["fault"], "NONFINITE_SENSOR")
        self.assertEqual(packet["body_torque_nm"], 0.0)
        self.assertFalse(packet["imu_valid"])
        self.assertIsNone(packet["imu_age_s"])
        json.dumps(packet, allow_nan=False)

    def test_sil_fails_closed_on_unparseable_sample_age(self):
        packet = OnboardSoftware().process_observation({
            "protocol_version": 1,
            "message_type": "environment_observation",
            "simulation_time_s": 0.0,
            "sensors": {
                "camera": {"valid": True, "pixel_error_x": 0,
                           "focal_length_x_px": 277, "age_s": 0.0},
                "imu": {"gyro_rad_s": {"x": 0.0, "y": 0.0, "z": 0.0},
                        "accel_m_s2": {"x": 0.0, "y": 0.0, "z": 9.8},
                        "age_s": None},
            },
        })
        self.assertEqual(packet["system_state"], "FAULT")
        self.assertEqual(packet["body_torque_nm"], 0.0)
        self.assertIsNone(packet["imu_age_s"])
        json.dumps(packet, allow_nan=False)

    def test_output_is_bounded(self):
        output = YawController().update(math.pi, -10.0)
        self.assertEqual(output.body_torque_nm, 0.05)

    def test_invalid_controller_configuration_fails_closed(self):
        controller = YawController(max_body_torque_nm=0.0)
        output = controller.update(0.2, 0.0)
        self.assertEqual(output.mode, "FAULT")
        self.assertEqual(output.fault, "INVALID_CONFIG")
        self.assertEqual(output.body_torque_nm, 0.0)

        controller = YawController(max_abs_rate_rad_s=0.0)
        self.assertEqual(controller.update(0.0, 0.0).fault, "INVALID_CONFIG")

    def test_controller_rate_limit_is_configurable_and_inclusive_at_limit(self):
        controller = YawController(max_abs_rate_rad_s=2.0)
        at_limit = controller.update(None, 2.0, target_valid=False)
        over_limit = controller.update(None, 2.0001, target_valid=False)
        self.assertEqual(at_limit.mode, "RATE_DAMP")
        self.assertEqual(over_limit.fault, "RATE_LIMIT")

    def test_invalid_config_takes_precedence_over_nonfinite_sensor(self):
        output = YawController(kp_nm_per_rad=-0.01).update(0.1, math.nan)
        self.assertEqual(output.fault, "INVALID_CONFIG")

    def test_simulated_observation_closes_yaw_loop_in_software(self):
        from Sim_Environment.main import Environment

        env = Environment()
        env.apply({"target_xyz_m": {"x": 1.154700538, "y": 0.0, "z": 2.0}})
        onboard = OnboardSoftware()
        for _ in range(500):
            observation = env.observation()
            frame = env.camera_frame_snapshot(observation["sequence"])
            output = onboard.process_observation(observation, camera_frame=frame)
            env.accept_actuator(output)
            env.plant.step(body_wheel_torque_nm=output["body_torque_nm"],
                           dt_s=0.02)
        self.assertLess(abs(env.snapshot()["target_error_rad"]), math.radians(1.0))

    def test_project_acquisition_target_within_six_simulated_seconds(self):
        from Sim_Environment.main import Environment

        env = Environment()
        env.apply({"target_xyz_m": {"x": 1.154700538, "y": 0.0, "z": 2.0}})
        onboard = OnboardSoftware()
        acquisition_time_s = None
        for _ in range(300):
            error_deg, _ = self.run_environment_step(env, onboard)
            if error_deg <= 2.0:
                acquisition_time_s = env.plant.time_s
                break
        self.assertIsNotNone(acquisition_time_s)
        self.assertLess(acquisition_time_s, 6.0)

    def test_project_one_minute_pointing_hold_within_two_degrees(self):
        from Sim_Environment.main import Environment

        env = Environment()
        onboard = OnboardSoftware()
        peak_error_deg = 0.0
        for _ in range(3000):
            error_deg, _ = self.run_environment_step(env, onboard)
            peak_error_deg = max(peak_error_deg, error_deg)
        self.assertGreaterEqual(env.plant.time_s, 59.99)
        self.assertLessEqual(peak_error_deg, 2.0)

    def test_project_twenty_degree_nudge_recovers_within_four_seconds(self):
        from Sim_Environment.main import Environment

        env = Environment()
        env.plant.yaw_rad = math.radians(20.0)
        onboard = OnboardSoftware()
        recovery_time_s = None
        start_time_s = env.plant.time_s
        for _ in range(200):
            error_deg, _ = self.run_environment_step(env, onboard)
            if error_deg <= 2.0:
                recovery_time_s = env.plant.time_s - start_time_s
                break
        self.assertIsNotNone(recovery_time_s)
        self.assertLess(recovery_time_s, 4.0)

    def test_slowly_moving_target_remains_in_field_and_is_tracked(self):
        from Sim_Environment.main import Environment

        env = Environment()
        onboard = OnboardSoftware()
        peak_error_deg = 0.0
        for index in range(3000):
            bearing_rad = math.radians(0.2 * (index + 1) * 0.02)
            env.apply({"target_xyz_m": {
                "x": 2.0 * math.tan(bearing_rad), "y": 0.0, "z": 2.0
            }})
            error_deg, packet = self.run_environment_step(env, onboard)
            self.assertTrue(packet["target_valid"])
            peak_error_deg = max(peak_error_deg, error_deg)
        self.assertGreaterEqual(env.plant.time_s, 59.99)
        self.assertLessEqual(peak_error_deg, 2.0)

    def test_project_recovers_after_two_second_camera_occlusion(self):
        from Sim_Environment.main import Environment

        env = Environment()
        onboard = OnboardSoftware()
        env.plant.yaw_rad = math.radians(5.0)
        env.apply({"camera_occlusion_s": 2.0})
        peak_rate_rad_s = 0.0
        for index in range(100):
            _, packet = self.run_environment_step(env, onboard)
            self.assertFalse(packet["target_valid"])
            peak_rate_rad_s = max(peak_rate_rad_s, abs(env.plant.rate_rad_s))
        self.assertGreaterEqual(env.plant.time_s, 1.99)
        self.assertLess(peak_rate_rad_s, 20.0)

        recovery_time_s = None
        for index in range(200):
            error_deg, packet = self.run_environment_step(env, onboard)
            if packet["target_valid"] and error_deg <= 2.0:
                recovery_time_s = env.plant.time_s - 2.0
                break
        self.assertIsNotNone(recovery_time_s)
        self.assertLess(recovery_time_s, 4.0)

    def test_supervisory_mode_and_ping_commands(self):
        onboard = OnboardSoftware()
        onboard.receive_command({"command_id": 1, "command": "SET_MODE",
                                 "arguments": {"mode": "SAFE"}})
        onboard.receive_command({"command_id": 2, "command": "PING"})
        packet = onboard.process_observation({
            "protocol_version": 1,
            "message_type": "environment_observation",
            "simulation_time_s": 0.0,
            "sensors": {
                "camera": {"valid": True, "pixel_error_x": 50,
                           "focal_length_x_px": 277, "age_s": 0.0},
                "imu": {"gyro_rad_s": {"z": 0.0}, "age_s": 0.0},
            },
        })
        self.assertEqual(packet["mode"], "SAFE")
        self.assertEqual(packet["body_torque_nm"], 0.0)
        self.assertEqual(packet["command_ack"]["command_id"], 2)

    def test_wheel_speed_test_requires_safe_mode_and_stops_after_duration(self):
        onboard = OnboardSoftware()
        onboard.mode = "SAFE"
        onboard.receive_command({"command_id": 1, "command": "SET_WHEEL_SPEED",
                                 "arguments": {"target_rev_s": 50.0,
                                               "duration_s": 1.0}}, now=10.0)
        packet = {
            "protocol_version": 1,
            "message_type": "environment_observation",
            "simulation_time_s": 0.0,
            "actuator_feedback": {"wheel_speed_rad_s": 0.0},
            "sensors": {
                "camera": {"valid": True, "pixel_error_x": 0.0,
                           "pixel_error_y": 0.0,
                           "focal_length_x_px": 277.0, "age_s": 0.0},
                "imu": {"gyro_rad_s": {"x": 0.0, "y": 0.0, "z": 0.0},
                        "accel_m_s2": {"x": 0.0, "y": 9.80665, "z": 0.0},
                        "age_s": 0.0},
            },
        }
        output = onboard.process_observation(packet, now=10.1)
        self.assertEqual(output["mode"], "WHEEL_SPEED_TEST")
        self.assertEqual(output["motor_speed_target_rev_s"], 50.0)
        self.assertEqual(output["body_torque_nm"], 0.0)
        self.assertEqual(output["command_ack"]["status"], "ACKNOWLEDGED")

        packet["actuator_feedback"]["wheel_speed_rad_s"] = 40.0 * 2.0 * math.pi / 60.0
        stopping = onboard.process_observation(packet, now=11.1)
        self.assertEqual(stopping["mode"], "WHEEL_SPEED_STOPPING")
        self.assertEqual(stopping["motor_speed_target_rev_s"], 0.0)

        packet["actuator_feedback"]["wheel_speed_rad_s"] = 0.0
        stopped = onboard.process_observation(packet, now=12.0)
        self.assertEqual(stopped["mode"], "SAFE")
        self.assertEqual(stopped["motor_speed_test_status"], "STOPPED")
        self.assertEqual(stopped["motor_speed_target_rev_s"], 0.0)

    def test_wheel_speed_terminal_status_remains_visible_after_stop(self):
        onboard = OnboardSoftware()
        onboard.receive_command({"command_id": 1, "command": "SET_MODE",
                                  "arguments": {"mode": "SAFE"}}, now=0.0)
        packet = {
            "protocol_version": 1,
            "message_type": "environment_observation",
            "simulation_time_s": 0.0,
            "actuator_feedback": {"wheel_speed_rad_s": 0.0},
            "sensors": {
                "camera": {"valid": False, "age_s": 0.0},
                "imu": {"gyro_rad_s": {"x": 0.0, "y": 0.0, "z": 0.0},
                        "accel_m_s2": {"x": 0.0, "y": 0.0, "z": 9.8},
                        "age_s": 0.0},
            },
        }
        onboard.receive_command({"command_id": 2, "command": "SET_WHEEL_SPEED",
                                 "arguments": {"target_rev_s": 1.0,
                                               "duration_s": 0.5}}, now=0.0)
        completed = onboard.process_observation(packet, now=0.5)
        self.assertEqual(completed["motor_speed_test_status"], "STOPPED")
        later = onboard.process_observation(packet, now=0.52)
        self.assertEqual(later["motor_speed_test_status"], "STOPPED")

        onboard.receive_command({"command_id": 3, "command": "SET_WHEEL_SPEED",
                                 "arguments": {"target_rev_s": 1.0,
                                               "duration_s": 0.5}}, now=20.0)
        packet["actuator_feedback"]["wheel_speed_rad_s"] = 2.0
        onboard.process_observation(packet, now=20.5)
        timed_out = onboard.process_observation(packet, now=30.5)
        self.assertEqual(timed_out["mode"], "SAFE")
        self.assertEqual(timed_out["motor_speed_test_status"], "STOP_TIMEOUT")
        later_timeout = onboard.process_observation(packet, now=30.52)
        self.assertEqual(later_timeout["motor_speed_test_status"], "STOP_TIMEOUT")

    def test_wheel_speed_test_is_rejected_while_attitude_tracking(self):
        onboard = OnboardSoftware()
        onboard.receive_command({"command_id": 1, "command": "SET_WHEEL_SPEED",
                                 "arguments": {"target_rev_s": 50.0,
                                               "duration_s": 1.0}}, now=5.0)
        packet = onboard.process_observation({
            "protocol_version": 1,
            "message_type": "environment_observation",
            "simulation_time_s": 0.0,
            "actuator_feedback": {"wheel_speed_rad_s": 0.0},
            "sensors": {
                "camera": {"valid": True, "pixel_error_x": 0.0,
                           "pixel_error_y": 0.0,
                           "focal_length_x_px": 277.0, "age_s": 0.0},
                "imu": {"gyro_rad_s": {"x": 0.0, "y": 0.0, "z": 0.0},
                        "accel_m_s2": {"x": 0.0, "y": 9.80665, "z": 0.0},
                        "age_s": 0.0},
            },
        }, now=5.1)
        self.assertEqual(packet["mode"], "TRACK")
        self.assertEqual(packet["motor_speed_target_rev_s"], None)
        self.assertEqual(packet["command_ack"]["status"], "REJECTED")
        self.assertEqual(packet["command_ack"]["reason"],
                         "SPEED_TEST_REQUIRES_SAFE_OR_STANDBY")

    def test_commanded_45_degree_yaw_step_settles_and_returns_to_standby(self):
        from Sim_Environment.main import Environment

        env = Environment()
        onboard = OnboardSoftware()
        onboard.mode = "SAFE"
        first = onboard.process_observation(env.observation(), now=100.0)
        self.assertTrue(first["yaw_estimate_valid"])
        onboard.receive_command({
            "command_id": 1,
            "command": "SET_YAW_STEP",
            "arguments": {"delta_deg": 45.0},
        }, now=100.02)

        completed = None
        for index in range(1, 500):
            now = 100.02 + index * 0.02
            output = onboard.process_observation(env.observation(), now=now)
            env.plant.step(body_wheel_torque_nm=output["body_torque_nm"],
                           dt_s=0.02)
            if output["yaw_step_status"] == "COMPLETE":
                completed = output
                break

        self.assertIsNotNone(completed)
        self.assertEqual(completed["mode"], "STANDBY")
        final_error = target_bearing_error(completed["yaw_step_target_rad"],
                                           env.plant.yaw_rad)
        self.assertLessEqual(abs(math.degrees(final_error)), 1.0)
        self.assertLess(completed["yaw_step_elapsed_s"], 10.0)

    def test_commanded_yaw_step_requires_safe_mode_and_valid_estimate(self):
        onboard = OnboardSoftware()
        onboard.receive_command({"command_id": 1, "command": "SET_YAW_STEP",
                                 "arguments": {"delta_deg": 45.0}}, now=0.0)
        self.assertEqual(onboard.pending_ack["status"], "REJECTED")
        self.assertEqual(onboard.pending_ack["reason"],
                         "YAW_STEP_REQUIRES_SAFE_OR_STANDBY")

        onboard.mode = "SAFE"
        onboard.last_command_id = 1
        onboard.receive_command({"command_id": 2, "command": "SET_YAW_STEP",
                                 "arguments": {"delta_deg": 45.0}}, now=0.02)
        self.assertEqual(onboard.pending_ack["status"], "REJECTED")
        self.assertEqual(onboard.pending_ack["reason"],
                         "ATTITUDE_ESTIMATE_UNAVAILABLE")

    def test_explicit_yaw_step_abort_clears_the_reported_target(self):
        from Sim_Environment.main import Environment

        onboard = OnboardSoftware()
        onboard.mode = "SAFE"
        env = Environment()
        initial = onboard.process_observation(env.observation(), now=10.0)
        self.assertTrue(initial["yaw_estimate_valid"])
        onboard.receive_command({"command_id": 1, "command": "SET_YAW_STEP",
                                 "arguments": {"delta_deg": 20.0}}, now=10.02)
        active = onboard.process_observation(env.observation(), now=10.02)
        self.assertEqual(active["mode"], "YAW_STEP")
        self.assertIsNotNone(active["yaw_step_target_rad"])

        onboard.receive_command({"command_id": 2, "command": "SET_MODE",
                                 "arguments": {"mode": "STANDBY"}}, now=10.04)
        aborted = onboard.process_observation(env.observation(), now=10.04)
        self.assertEqual(aborted["mode"], "STANDBY")
        self.assertEqual(aborted["yaw_step_status"], "ABORTED")
        self.assertIsNone(aborted["yaw_step_target_rad"])

    def test_stale_imu_aborts_yaw_step_without_integrating_the_sample(self):
        from Sim_Environment.main import Environment

        onboard = OnboardSoftware()
        onboard.mode = "SAFE"
        env = Environment()
        initial = onboard.process_observation(env.observation(), now=10.0)
        self.assertTrue(initial["yaw_estimate_valid"])
        yaw_before_stale_sample = onboard.attitude_estimator.yaw_rad
        onboard.receive_command({"command_id": 1, "command": "SET_YAW_STEP",
                                 "arguments": {"delta_deg": 20.0}}, now=10.02)

        stale_observation = env.observation()
        stale_observation["sensors"]["imu"]["age_s"] = 0.2
        stale_observation["sensors"]["imu"]["gyro_rad_s"]["y"] = 1.0
        result = onboard.process_observation(stale_observation, now=10.04)

        self.assertEqual(result["mode"], "SAFE")
        self.assertEqual(result["yaw_step_status"], "ATTITUDE_INVALID")
        self.assertIsNone(result["yaw_step_target_rad"])
        self.assertIsNone(result["yaw_step_error_rad"])
        self.assertFalse(result["yaw_estimate_valid"])
        self.assertEqual(result["body_torque_nm"], 0.0)
        self.assertEqual(onboard.attitude_estimator.yaw_rad,
                         yaw_before_stale_sample)

    def test_yaw_step_timeout_returns_safe_with_null_target_and_error(self):
        from Sim_Environment.main import Environment

        onboard = OnboardSoftware()
        onboard.mode = "SAFE"
        env = Environment()
        onboard.process_observation(env.observation(), now=10.0)
        onboard.receive_command({"command_id": 1, "command": "SET_YAW_STEP",
                                 "arguments": {"delta_deg": 20.0}}, now=10.02)
        timed_out = onboard.process_observation(env.observation(), now=20.02)
        self.assertEqual(timed_out["mode"], "SAFE")
        self.assertEqual(timed_out["yaw_step_status"], "TIMEOUT")
        self.assertIsNone(timed_out["yaw_step_target_rad"])
        self.assertIsNone(timed_out["yaw_step_error_rad"])
        self.assertEqual(timed_out["body_torque_nm"], 0.0)

    def test_telemetry_contains_all_synthetic_imu_axes(self):
        packet = OnboardSoftware().process_observation({
            "protocol_version": 1,
            "message_type": "environment_observation",
            "simulation_time_s": 0.0,
            "sensors": {
                "camera": {"valid": True, "pixel_error_x": 0,
                           "focal_length_x_px": 277, "age_s": 0.0},
                "imu": {"gyro_rad_s": {"x": 0.1, "y": -0.2, "z": 0.3},
                        "accel_m_s2": {"x": 1.0, "y": 2.0, "z": 9.0},
                        "age_s": 0.0},
            },
        })
        self.assertTrue(packet["imu_valid"])
        self.assertEqual(packet["gyro_y_rad_s"], -0.2)
        self.assertEqual(packet["accel_z_m_s2"], 9.0)

    def test_magnetometer_vector_is_telemetry_only_and_invalid_values_are_rejected(self):
        packet = OnboardSoftware().process_observation({
            "protocol_version": 1,
            "message_type": "environment_observation",
            "simulation_time_s": 0.0,
            "sensors": {
                "camera": {"valid": False, "age_s": 0.0},
                "imu": {"gyro_rad_s": {"x": 0.0, "y": 0.0, "z": 0.0},
                        "accel_m_s2": {"x": 0.0, "y": 9.8, "z": 0.0},
                        "age_s": 0.0},
                "magnetometer": {
                    "valid": True,
                    "magnetic_field_uT": {"x": 20.0, "y": -5.0, "z": 45.0},
                    "age_s": 0.05,
                },
            },
        })
        self.assertTrue(packet["mag_valid"])
        self.assertEqual(packet["mag_frame"], "body")
        self.assertEqual(packet["mag_x_uT"], 20.0)
        self.assertEqual(packet["mag_y_uT"], -5.0)
        self.assertEqual(packet["mag_z_uT"], 45.0)
        self.assertAlmostEqual(packet["body_torque_nm"], 0.0)

        packet = OnboardSoftware().process_observation({
            "protocol_version": 1,
            "message_type": "environment_observation",
            "simulation_time_s": 0.0,
            "sensors": {
                "camera": {"valid": False, "age_s": 0.0},
                "imu": {"gyro_rad_s": {"x": 0.0, "y": 0.0, "z": 0.0},
                        "accel_m_s2": {"x": 0.0, "y": 9.8, "z": 0.0},
                        "age_s": 0.0},
                "magnetometer": {
                    "valid": True,
                    "magnetic_field_uT": {"x": float("nan"), "y": 0.0, "z": 0.0},
                    "age_s": 0.0,
                },
            },
        })
        self.assertFalse(packet["mag_valid"])
        self.assertIsNone(packet["mag_x_uT"])


if __name__ == "__main__":
    unittest.main()
