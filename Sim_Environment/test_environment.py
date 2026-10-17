import math
import json
import time
import unittest
from unittest.mock import patch
from urllib.parse import urlparse

from . import main as environment_main
from .camera_transport import (MAX_ENCODED_BYTES, decode_rle_frame,
                               encode_rle_frame)
from .main import (CAMERA_FRAME_HISTORY, CAMERA_HEIGHT_PX,
                   CAMERA_PREVIEW_BYTES, CAMERA_PREVIEW_HEIGHT,
                   CAMERA_PREVIEW_WIDTH, CAMERA_WIDTH_PX, Environment, Handler)
from CubeSat_ESP32.camera import detect_black_target
from CubeSat_ESP32 import sil
from CubeSat_ESP32.sil import OnboardSoftware
from http_test_support import dispatch


class EnvironmentTests(unittest.TestCase):
    def test_versioned_bridge_inputs_reject_boolean_and_fractional_versions(self):
        invalid_versions = (True, 1.5, 2, "1")
        for version in invalid_versions:
            with self.subTest(version=version):
                env = Environment()
                with self.assertRaisesRegex(ValueError, "protocol_version"):
                    env.queue_command({
                        "protocol_version": version,
                        "message_type": "telecommand",
                        "command_id": 1,
                        "command": "PING",
                    })
                with self.assertRaisesRegex(ValueError, "protocol_version"):
                    env.accept_actuator({"protocol_version": version})

        env = Environment()
        env.queue_command({
            "protocol_version": 1.0,
            "message_type": "telecommand",
            "command_id": 1,
            "command": "PING",
        })
        env.accept_actuator({
            "protocol_version": 1.0,
            "message_type": "actuator_telemetry",
            "body_torque_nm": 0.0,
        })

    def test_environment_ui_serves_explicit_controls_and_reports_link_failure(self):
        status, _, body = dispatch(Handler, "GET", "/")
        self.assertEqual(status, 200)
        self.assertIn(b"numberInput('tx')", body)
        self.assertIn(b"numberInput('duration')", body)
        self.assertIn(b"numberInput('occlusion')", body)
        self.assertIn(b"Simulation link unavailable; input was not applied.", body)

    def test_environment_frame_can_be_processed_by_host_detector(self):
        env = Environment()
        env.apply({"target_xyz_m": {"x": 0.25, "y": -0.1, "z": 2.0}})
        observation = env.observation()
        frame = env.camera_frame_snapshot(observation["sequence"])
        self.assertEqual(len(frame), CAMERA_WIDTH_PX * CAMERA_HEIGHT_PX)
        detection = detect_black_target(frame, CAMERA_WIDTH_PX, CAMERA_HEIGHT_PX)
        camera = observation["sensors"]["camera"]
        self.assertTrue(detection["valid"])
        self.assertTrue(50 <= detection["dark_pixel_count"] <= 200)
        self.assertAlmostEqual(detection["error_x_px"], camera["pixel_error_x"], delta=0.5)
        self.assertAlmostEqual(detection["error_y_px"], -camera["pixel_error_y"], delta=0.5)
        telemetry = OnboardSoftware().process_observation(
            observation, camera_frame=frame
        )
        self.assertTrue(telemetry["target_valid"])
        self.assertAlmostEqual(telemetry["camera_error_x_px"],
                               detection["error_x_px"], delta=1e-6)
        self.assertGreater(telemetry["body_torque_nm"], 0.0)
        env.accept_actuator(telemetry)
        preview = env.camera_preview_snapshot()
        self.assertEqual(preview[4], "simulated")
        self.assertEqual(env.camera_preview_gray,
                         env._downsample_camera_frame(frame))
        self.assertEqual(preview[1:4], (
            True, telemetry["camera_error_x_px"], telemetry["camera_error_y_px"]
        ))

    def test_old_rendered_frame_fails_closed_in_onboard_camera_control(self):
        env = Environment()
        observation = env.observation()
        frame = env.camera_frame_snapshot(observation["sequence"])
        telemetry = OnboardSoftware().process_observation(
            observation, camera_frame=frame,
            camera_sample_started=time.monotonic() - 0.101
        )
        self.assertEqual(telemetry["mode"], "FAULT")
        self.assertEqual(telemetry["fault"], "STALE_OBSERVATION")
        self.assertEqual(telemetry["body_torque_nm"], 0.0)

    def test_future_rendered_frame_timestamp_fails_closed(self):
        env = Environment()
        observation = env.observation()
        frame = env.camera_frame_snapshot(observation["sequence"])
        telemetry = OnboardSoftware().process_observation(
            observation, camera_frame=frame,
            camera_sample_started=time.monotonic() + 1.0
        )
        self.assertEqual(telemetry["mode"], "FAULT")
        self.assertEqual(telemetry["body_torque_nm"], 0.0)

    def test_camera_frame_endpoint_pairs_raw_qvga_bytes_with_observation_sequence(self):
        env = Environment()
        observation = env.observation()
        sequence = observation["sequence"]
        with patch.object(environment_main, "environment", env):
            status, headers, body = dispatch(
                Handler, "GET", f"/api/camera-frame.raw?sequence={sequence}")
        self.assertEqual(status, 200)
        self.assertEqual(len(body), CAMERA_WIDTH_PX * CAMERA_HEIGHT_PX)
        self.assertIn(f"X-Camera-Frame-Sequence: {sequence}".encode(), headers)

    def test_camera_rle_endpoint_preserves_frame_and_sequence_with_small_payload(self):
        env = Environment()
        observation = env.observation()
        sequence = observation["sequence"]
        original = env.camera_frame_snapshot(sequence)
        with patch.object(environment_main, "environment", env):
            status, headers, body = dispatch(
                Handler, "GET", f"/api/camera-frame.rle?sequence={sequence}")
        self.assertEqual(status, 200)
        self.assertIn(f"X-Camera-Frame-Sequence: {sequence}".encode(), headers)
        self.assertIn(b"X-Camera-Frame-Encoding: RLE1", headers)
        self.assertIn(b"X-Camera-Frame-Width: 320", headers)
        self.assertIn(b"X-Camera-Frame-Height: 240", headers)
        self.assertLess(len(body), len(original) // 20)
        self.assertEqual(len(body), 91)
        self.assertEqual(decode_rle_frame(body, len(original)), original)

    def test_rle_bridge_to_sil_reconstructs_and_detects_paired_image(self):
        env = Environment()
        env.apply({"target_xyz_m": {"x": 0.25, "y": -0.1, "z": 2.0}})
        observation = env.observation()
        sequence = observation["sequence"]
        original = env.camera_frame_snapshot(sequence)

        class MemoryResponse:
            def __init__(self, response_headers, body):
                self.headers = response_headers
                self.body = body

            def __enter__(self):
                return self

            def __exit__(self, *_args):
                return False

            def read(self, size):
                return self.body[:size]

        def bridge_get(request, timeout):
            parsed = urlparse(request.full_url)
            status, raw_headers, body = dispatch(
                Handler, "GET", parsed.path + "?" + parsed.query
            )
            if status != 200:
                raise AssertionError(f"bridge returned HTTP {status}")
            response_headers = {}
            for line in raw_headers.split(b"\r\n")[1:]:
                if b": " in line:
                    key, value = line.decode("ascii").split(": ", 1)
                    response_headers[key] = value
            return MemoryResponse(response_headers, body)

        with patch.object(environment_main, "environment", env), \
                patch.object(sil, "BASE_URL", "http://memory-bridge"), \
                patch.object(sil, "urlopen", side_effect=bridge_get):
            frame = sil.request_camera_frame(sequence)

        self.assertEqual(frame, original)
        detection = detect_black_target(frame, CAMERA_WIDTH_PX,
                                        CAMERA_HEIGHT_PX)
        telemetry = OnboardSoftware().process_observation(
            observation, camera_frame=frame
        )
        self.assertTrue(detection["valid"])
        self.assertTrue(telemetry["target_valid"])
        self.assertEqual(telemetry["camera_frame_sequence"], sequence)
        self.assertAlmostEqual(telemetry["camera_error_x_px"],
                               detection["error_x_px"])
        env.accept_actuator(telemetry)
        preview = env.camera_preview_snapshot()
        self.assertEqual(preview[4], "simulated")
        self.assertEqual(env.camera_preview_gray,
                         env._downsample_camera_frame(original))
        self.assertEqual(preview[1:4], (
            True, telemetry["camera_error_x_px"], telemetry["camera_error_y_px"]
        ))

    def test_camera_rle_endpoint_rejects_payload_above_transfer_limit(self):
        env = Environment()
        sequence = env.observation()["sequence"]
        with patch.object(environment_main, "environment", env), patch.object(
                environment_main, "encode_rle_frame", return_value=bytes(8193)):
            status, _, body = dispatch(
                Handler, "GET", f"/api/camera-frame.rle?sequence={sequence}")
        self.assertEqual(status, 413)
        self.assertIn(b"transfer bound", body)

    def test_old_camera_frame_expires_from_bounded_sequence_history(self):
        env = Environment()
        first = env.observation()["sequence"]
        for _ in range(CAMERA_FRAME_HISTORY):
            env.observation()
        self.assertIsNone(env.camera_frame_snapshot(first))

    def test_target_xyz_and_scale_update(self):
        env = Environment()
        env.apply({"target_xyz_m": {"x": 1.0, "y": 0.2, "z": 3.0},
                   "target_scale": 0.25})
        state = env.snapshot()
        self.assertEqual(state["target_xyz_m"]["z"], 3.0)
        self.assertAlmostEqual(state["target_bearing_rad"], 0.3217505544)
        self.assertEqual(state["target_scale_m"], 0.25)

    def test_torque_pulse_records_all_axes(self):
        env = Environment()
        env.apply({"external_torque_nm": {"x": 0.01, "y": 0.02, "z": -0.03},
                   "duration_s": 1})
        state = env.snapshot()
        self.assertEqual(state["external_torque_nm"]["x"], 0.01)
        self.assertTrue(state["torque_active"])

    def test_short_torque_pulse_preserves_impulse_on_simulation_clock(self):
        env = Environment()
        env.plant.damping_n_m_per_rad_s = 0.0
        env.apply({"external_torque_nm": {"x": 0.0, "y": 0.01, "z": 0.0},
                   "duration_s": 0.015})

        # A 15 ms pulse crossing a 20 ms step has the same angular impulse as
        # 15 ms at full torque. Wall-clock time is deliberately far ahead.
        env.advance(dt_s=0.02, now=1_000_000.0)

        self.assertAlmostEqual(env.plant.rate_rad_s, 0.01 / 0.02 * 0.015)
        self.assertAlmostEqual(env.plant.time_s, 0.02)
        self.assertFalse(env.snapshot()["torque_active"])

    def test_camera_occlusion_duration_uses_simulation_time(self):
        env = Environment()
        env.apply({"camera_occlusion_s": 0.5})
        self.assertFalse(env.observation()["sensors"]["camera"]["valid"])

        for _ in range(5):
            env.advance(dt_s=0.1, now=1_000_000.0)
        self.assertTrue(env.observation()["sensors"]["camera"]["valid"])

    def test_advance_rejects_invalid_step_before_applying_timed_events(self):
        for dt_s in (0.0, -0.01, 0.11):
            with self.subTest(dt_s=dt_s), self.assertRaisesRegex(
                    ValueError, r"dt_s must be finite"):
                Environment().advance(dt_s=dt_s)

    def test_rejects_target_behind_camera(self):
        env = Environment()
        with self.assertRaises(ValueError):
            env.apply({"target_xyz_m": {"x": 0, "y": 0, "z": -1}})

    def test_rejects_out_of_range_torque(self):
        env = Environment()
        with self.assertRaises(ValueError):
            env.apply({"external_torque_nm": {"x": 2, "y": 0, "z": 0},
                       "duration_s": 1})

    def test_invalid_scene_request_does_not_partially_apply_valid_fields(self):
        env = Environment()
        original = env.snapshot()
        with patch.object(environment_main, "environment", env):
            status, _, _ = dispatch(Handler, "POST", "/api/control", {
                "target_xyz_m": {"x": 1.0, "y": 0.0, "z": 2.0},
                "external_torque_nm": {"x": 2.0, "y": 0.0, "z": 0.0},
                "duration_s": 1.0,
            })
        self.assertEqual(status, 400)
        state = env.snapshot()
        self.assertEqual(state["target_xyz_m"], original["target_xyz_m"])
        self.assertEqual(state["external_torque_nm"],
                         original["external_torque_nm"])
        self.assertFalse(state["torque_active"])

    def test_scene_control_rejects_unused_duration_and_unknown_fields(self):
        env = Environment()
        for payload, message in (
                ({"duration_s": 1.0}, "requires external_torque_nm"),
                ({"target_scale_m": 0.5}, "unsupported control field")):
            with self.subTest(payload=payload), self.assertRaisesRegex(ValueError, message):
                env.apply(payload)

    def test_observation_schema_contains_synthetic_camera_and_imu(self):
        packet = Environment().observation()
        self.assertEqual(packet["protocol_version"], 1)
        self.assertEqual(packet["message_type"], "environment_observation")
        self.assertIn("camera", packet["sensors"])
        self.assertIn("imu", packet["sensors"])
        self.assertIn("magnetometer", packet["sensors"])
        self.assertTrue(packet["sensors"]["camera"]["valid"])
        self.assertIn("wheel_speed_rad_s", packet["actuator_feedback"])

    def test_sil_camera_y_telemetry_uses_image_down_coordinates(self):
        env = Environment()
        env.apply({"target_xyz_m": {"x": 0.0, "y": 0.2, "z": 2.0}})
        observation = env.observation()
        self.assertGreater(observation["sensors"]["camera"]["pixel_error_y"], 0)
        telemetry = OnboardSoftware().process_observation(observation)
        self.assertLess(telemetry["camera_error_y_px"], 0)

    def test_speed_telecommand_is_bounded_and_normalized(self):
        env = Environment()
        env.queue_command({"protocol_version": 1, "message_type": "telecommand",
                           "command_id": 1, "command": "SET_WHEEL_SPEED",
                           "arguments": {"target_rev_s": 50, "duration_s": 1.0}})
        command = env.commands_after(0)[0]
        self.assertEqual(command["arguments"],
                         {"target_rev_s": 50.0, "duration_s": 1.0})

    def test_speed_telecommand_rejects_out_of_range_and_unknown_arguments(self):
        for args in (
                {"target_rev_s": 50.1, "duration_s": 1.0},
                {"target_rev_s": 10.0, "duration_s": 0.1},
                {"target_rev_s": 10.0, "duration_s": 1.0, "torque_nm": 0.1}):
            with self.subTest(args=args), self.assertRaises(ValueError):
                Environment().queue_command({
                    "protocol_version": 1, "message_type": "telecommand",
                    "command_id": 1, "command": "SET_WHEEL_SPEED",
                    "arguments": args,
                })

    def test_speed_target_actuator_packet_drives_simulated_wheel(self):
        env = Environment()
        env.accept_actuator({"protocol_version": 1,
                             "message_type": "actuator_telemetry",
                             "sequence": 1, "body_torque_nm": 0.0,
                             "motor_speed_target_rev_s": 10.0})
        env.advance(0.02)
        self.assertGreater(env.plant.wheel.speed_rad_s, 0.0)
        self.assertAlmostEqual(env.snapshot()["motor_speed_target_rev_s"], 10.0)

    def test_synthetic_magnetic_field_rotates_with_body_and_preserves_magnitude(self):
        env = Environment()
        field = env.observation()["sensors"]["magnetometer"]["magnetic_field_uT"]
        initial_norm = math.sqrt(sum(value * value for value in field.values()))
        self.assertAlmostEqual(field["x"], 20.0)
        self.assertAlmostEqual(field["z"], 45.0)
        env.plant.yaw_rad = math.pi / 2
        rotated = env.observation()["sensors"]["magnetometer"]["magnetic_field_uT"]
        rotated_norm = math.sqrt(sum(value * value for value in rotated.values()))
        self.assertAlmostEqual(rotated["x"], -45.0)
        self.assertAlmostEqual(rotated["z"], 20.0)
        self.assertAlmostEqual(rotated_norm, initial_norm)

    def test_tilted_synthetic_sensors_and_estimator_match_combined_attitudes(self):
        poses_deg = (
            (-35.0, -25.0, -140.0),
            (-20.0, 30.0, -60.0),
            (0.0, 0.0, 0.0),
            (15.0, -20.0, 45.0),
            (35.0, 25.0, 140.0),
        )
        gravity = 9.80665
        world_field_norm = math.sqrt(20.0 ** 2 + 45.0 ** 2)
        for roll_deg, pitch_deg, yaw_deg in poses_deg:
            with self.subTest(roll=roll_deg, pitch=pitch_deg, yaw=yaw_deg):
                env = Environment()
                env.plant.roll_rad = math.radians(roll_deg)
                env.plant.pitch_rad = math.radians(pitch_deg)
                env.plant.yaw_rad = math.radians(yaw_deg)
                env.plant.roll_rate_rad_s = 0.03
                env.plant.rate_rad_s = -0.02
                env.plant.pitch_rate_rad_s = 0.01
                with patch.object(env.rng, "gauss", return_value=0.0):
                    observation = env.observation()

                sensors = observation["sensors"]
                acceleration = sensors["imu"]["accel_m_s2"]
                magnetic = sensors["magnetometer"]["magnetic_field_uT"]
                self.assertAlmostEqual(
                    math.sqrt(sum(v * v for v in acceleration.values())),
                    gravity, places=8)
                self.assertAlmostEqual(
                    math.sqrt(sum(v * v for v in magnetic.values())),
                    world_field_norm, places=8)
                gyro = sensors["imu"]["gyro_rad_s"]
                self.assertAlmostEqual(gyro["x"], 0.011)
                self.assertAlmostEqual(gyro["y"], -0.021)
                self.assertAlmostEqual(gyro["z"], 0.031)

                telemetry = OnboardSoftware().process_observation(observation)
                self.assertTrue(telemetry["yaw_estimate_valid"])
                self.assertAlmostEqual(
                    telemetry["pitch_estimate_rad"],
                    math.radians(pitch_deg), places=10)
                self.assertAlmostEqual(
                    telemetry["roll_estimate_rad"],
                    math.radians(roll_deg), places=10)
                yaw_error = math.atan2(
                    math.sin(telemetry["yaw_estimate_rad"] -
                             env.plant.yaw_rad),
                    math.cos(telemetry["yaw_estimate_rad"] -
                             env.plant.yaw_rad),
                )
                self.assertAlmostEqual(yaw_error, 0.0, places=10)

    def test_camera_pixel_error_is_target_bearing_minus_body_yaw(self):
        env = Environment()
        target_x = 2.0 * math.tan(math.radians(20.0))
        env.apply({"target_xyz_m": {"x": target_x, "y": 0.0, "z": 2.0}})
        offset = env.observation()["sensors"]["camera"]["pixel_error_x"]
        self.assertGreater(offset, 0)
        env.plant.yaw_rad = math.atan2(target_x, 2.0)
        centered = env.observation()["sensors"]["camera"]["pixel_error_x"]
        self.assertEqual(centered, 0)

    def test_tilted_camera_projection_matches_independent_rotation_matrix(self):
        poses_deg = (
            (-15.0, -10.0, -20.0),
            (-10.0, 15.0, -10.0),
            (0.0, 0.0, 0.0),
            (10.0, -15.0, 10.0),
            (15.0, 10.0, 20.0),
        )
        target_world = (0.2, 0.1, 2.0)
        focal_px = 277.0

        def multiply(left, right):
            return [[sum(left[row][k] * right[k][col]
                         for k in range(3))
                     for col in range(3)] for row in range(3)]

        for roll_deg, pitch_deg, yaw_deg in poses_deg:
            with self.subTest(roll=roll_deg, pitch=pitch_deg, yaw=yaw_deg):
                roll, pitch, yaw = map(
                    math.radians, (roll_deg, pitch_deg, yaw_deg))
                cr, sr = math.cos(roll), math.sin(roll)
                cp, sp = math.cos(pitch), math.sin(pitch)
                cy, sy = math.cos(yaw), math.sin(yaw)
                rotate_z = [[cr, -sr, 0.0], [sr, cr, 0.0],
                            [0.0, 0.0, 1.0]]
                rotate_x = [[1.0, 0.0, 0.0],
                            [0.0, cp, -sp], [0.0, sp, cp]]
                rotate_y = [[cy, 0.0, sy], [0.0, 1.0, 0.0],
                            [-sy, 0.0, cy]]
                body_to_world = multiply(
                    rotate_y, multiply(rotate_x, rotate_z))
                camera_xyz = [
                    sum(body_to_world[row][col] * target_world[row]
                        for row in range(3))
                    for col in range(3)
                ]
                expected_x = round(focal_px * camera_xyz[0] / camera_xyz[2])
                expected_y = round(focal_px * camera_xyz[1] / camera_xyz[2])

                env = Environment()
                env.plant.roll_rad = roll
                env.plant.pitch_rad = pitch
                env.plant.yaw_rad = yaw
                env.apply({"target_xyz_m": dict(zip(
                    ("x", "y", "z"), target_world))})
                camera = env.observation()["sensors"]["camera"]
                self.assertTrue(camera["valid"])
                self.assertEqual(camera["pixel_error_x"], expected_x)
                self.assertEqual(camera["pixel_error_y"], expected_y)

    def test_camera_facing_target_size_uses_optical_depth_not_slant_range(self):
        counts = []
        for bearing_deg in (0.0, 15.0):
            env = Environment()
            env.apply({"target_xyz_m": {
                "x": 2.0 * math.tan(math.radians(bearing_deg)),
                "y": 0.0,
                "z": 2.0,
            }, "target_scale": 0.2})
            observation = env.observation()
            counts.append(observation["sensors"]["camera"][
                "target_dark_pixel_area"])
        self.assertEqual(counts[0], counts[1])

    def test_camera_target_behind_body_does_not_divide_by_nonpositive_depth(self):
        env = Environment()
        env.plant.yaw_rad = math.pi
        observation = env.observation()
        camera = observation["sensors"]["camera"]
        frame = env.camera_frame_snapshot(observation["sequence"])
        self.assertFalse(camera["valid"])
        self.assertEqual(camera["target_dark_pixel_area"], 0)
        self.assertFalse(detect_black_target(
            frame, CAMERA_WIDTH_PX, CAMERA_HEIGHT_PX
        )["valid"])

    def test_near_camera_target_clamps_to_full_frame_without_overflow(self):
        env = Environment()
        env.apply({"target_xyz_m": {"x": 0.0, "y": 0.0, "z": 1e-300},
                   "target_scale": 2.0})
        observation = env.observation()
        frame = env.camera_frame_snapshot(observation["sequence"])
        self.assertEqual(frame.count(12), CAMERA_WIDTH_PX * CAMERA_HEIGHT_PX)
        self.assertLessEqual(len(encode_rle_frame(frame)), MAX_ENCODED_BYTES)
        self.assertFalse(observation["sensors"]["camera"]["valid"])

    def test_rle_transfer_bound_covers_supported_scene_grid(self):
        for depth_m in (0.001, 0.1, 2.0, 1000.0):
            for bearing_deg in (-30.0, 0.0, 30.0):
                for target_scale in (0.001, 0.015, 0.1, 0.5, 2.0):
                    with self.subTest(depth_m=depth_m, bearing_deg=bearing_deg,
                                      target_scale=target_scale):
                        env = Environment()
                        env.apply({
                            "target_xyz_m": {
                                "x": depth_m * math.tan(
                                    math.radians(bearing_deg)),
                                "y": 0.0,
                                "z": depth_m,
                            },
                            "target_scale": target_scale,
                        })
                        observation = env.observation()
                        frame = env.camera_frame_snapshot(
                            observation["sequence"]
                        )
                        encoded = encode_rle_frame(frame)
                        self.assertLessEqual(len(encoded), MAX_ENCODED_BYTES)
                        self.assertEqual(
                            decode_rle_frame(encoded, len(frame)), frame
                        )

    def test_camera_rejects_target_outside_synthetic_pixel_area_limits(self):
        env = Environment()
        env.apply({"target_scale": 0.015})
        observation = env.observation()
        camera = observation["sensors"]["camera"]
        frame = env.camera_frame_snapshot(observation["sequence"])
        detection = detect_black_target(frame, CAMERA_WIDTH_PX, CAMERA_HEIGHT_PX)
        self.assertFalse(camera["valid"])
        self.assertIsNone(camera["pixel_error_x"])
        self.assertLess(camera["target_dark_pixel_area"], 50.0)
        self.assertGreater(camera["target_dark_pixel_area"], 0)
        self.assertFalse(detection["valid"])
        self.assertEqual(camera["target_dark_pixel_area"],
                         detection["dark_pixel_count"])

    def test_camera_validity_uses_rendered_area_when_target_is_clipped(self):
        env = Environment()
        env.apply({"target_xyz_m": {
            "x": 2.0 * math.tan(math.radians(30.0)),
            "y": 0.0,
            "z": 2.0,
        }, "target_scale": 0.067})
        observation = env.observation()
        camera = observation["sensors"]["camera"]
        frame = env.camera_frame_snapshot(observation["sequence"])
        detection = detect_black_target(frame, CAMERA_WIDTH_PX, CAMERA_HEIGHT_PX)

        self.assertFalse(detection["valid"])
        self.assertFalse(camera["valid"])
        self.assertIsNone(camera["pixel_error_x"])
        self.assertEqual(camera["target_dark_pixel_area"],
                         detection["dark_pixel_count"])

    def test_oversized_in_view_target_is_rendered_but_rejected_by_area_limit(self):
        env = Environment()
        env.apply({"target_scale": 0.5})
        observation = env.observation()
        camera = observation["sensors"]["camera"]
        frame = env.camera_frame_snapshot(observation["sequence"])
        detection = detect_black_target(frame, CAMERA_WIDTH_PX, CAMERA_HEIGHT_PX)

        self.assertFalse(camera["valid"])
        self.assertIsNone(camera["pixel_error_x"])
        self.assertGreater(camera["target_dark_pixel_area"], 200)
        self.assertFalse(detection["valid"])
        self.assertEqual(camera["target_dark_pixel_area"],
                         detection["dark_pixel_count"])

    def test_camera_requires_target_inside_field_of_view(self):
        env = Environment()
        for bearing_deg, expected in ((25.0, True), (29.9, True), (30.1, False)):
            with self.subTest(bearing_deg=bearing_deg):
                env.apply({"target_xyz_m": {
                    "x": 2.0 * math.tan(math.radians(bearing_deg)),
                    "y": 0.0,
                    "z": 2.0,
                }})
                camera = env.observation()["sensors"]["camera"]
                self.assertEqual(camera["valid"], expected)

    def test_roll_marker_toggle_is_boolean_and_preserved_in_scene_state(self):
        env = Environment()
        self.assertFalse(env.snapshot()["camera_roll_marker_enabled"])
        env.apply({"camera_roll_marker_enabled": True})
        self.assertTrue(env.snapshot()["camera_roll_marker_enabled"])
        with self.assertRaisesRegex(ValueError, "must be boolean"):
            env.apply({"camera_roll_marker_enabled": 1})

    def test_valid_actuator_packet_updates_telemetry_and_plant_command(self):
        env = Environment()
        packet = {"protocol_version": 1, "source": "cubesat_sil",
                  "message_type": "actuator_telemetry", "sequence": 1,
                  "body_torque_nm": 0.01, "mode": "TRACK",
                  "system_state": "NOMINAL"}
        env.accept_actuator(packet)
        self.assertEqual(env.snapshot()["body_torque_nm"], 0.01)
        self.assertEqual(env.telemetry_snapshot()["mode"], "TRACK")

    def test_virtual_pitch_and_roll_torques_use_separate_bounds(self):
        env = Environment()
        env.accept_actuator({
            "protocol_version": 1, "source": "cubesat_sil",
            "message_type": "actuator_telemetry", "sequence": 1,
            "body_torque_nm": 0.0,
            "virtual_axis_torque_nm": {"pitch": 0.02, "roll": 0.035},
        })
        self.assertEqual(env.virtual_pitch_torque_nm, 0.02)
        self.assertEqual(env.virtual_roll_torque_nm, 0.035)
        for axis, invalid_value in (("pitch", 0.0201), ("roll", 0.0351)):
            with self.subTest(axis=axis):
                with self.assertRaisesRegex(ValueError, f"virtual {axis} torque"):
                    Environment().accept_actuator({
                        "protocol_version": 1, "source": "cubesat_sil",
                        "message_type": "actuator_telemetry", "sequence": 1,
                        "body_torque_nm": 0.0,
                        "virtual_axis_torque_nm": {axis: invalid_value},
                    })

    def test_esp32_virtual_axis_torque_is_accepted_only_for_simulated_camera(self):
        env = Environment()
        env.accept_actuator({
            "protocol_version": 1, "source": "cubesat_esp32",
            "message_type": "actuator_telemetry", "sequence": 1,
            "body_torque_nm": 0.0, "camera_frame_source": "simulated",
            "virtual_axis_torque_nm": {"pitch": 0.01, "roll": -0.02},
        })
        self.assertEqual(env.virtual_pitch_torque_nm, 0.01)
        self.assertEqual(env.virtual_roll_torque_nm, -0.02)
        env.advance(dt_s=0.01)
        self.assertGreater(env.plant.pitch_rate_rad_s, 0.0)
        self.assertLess(env.plant.roll_rate_rad_s, 0.0)
        telemetry = env.telemetry_snapshot()
        self.assertEqual(telemetry["simulated_pitch_body_torque_nm"], 0.01)
        self.assertEqual(telemetry["simulated_roll_body_torque_nm"], -0.02)
        for frame_source in ("physical", "simulated-extra", "unavailable", None):
            with self.subTest(frame_source=frame_source):
                packet = {
                    "protocol_version": 1, "source": "cubesat_esp32",
                    "message_type": "actuator_telemetry", "sequence": 1,
                    "body_torque_nm": 0.0,
                    "camera_frame_source": frame_source,
                    "virtual_axis_torque_nm": {"roll": 0.01},
                }
                with self.assertRaisesRegex(
                    ValueError, "host-SIL or simulated ESP32"
                ):
                    Environment().accept_actuator(packet)

    def test_ground_telemetry_includes_separate_simulated_wheel_outputs(self):
        env = Environment()
        env.accept_actuator({"protocol_version": 1, "source": "cubesat_sil",
                             "message_type": "actuator_telemetry",
                             "sequence": 1, "body_torque_nm": 0.01,
                             "mode": "TRACK", "system_state": "NOMINAL"})
        env.plant.step(body_wheel_torque_nm=0.01, dt_s=0.01)
        packet = env.telemetry_snapshot()
        self.assertIn("simulated_wheel_speed_rad_s", packet)
        self.assertIn("simulated_wheel_motor_torque_nm", packet)
        self.assertIn("simulated_body_torque_nm", packet)
        self.assertNotEqual(packet["simulated_body_torque_nm"],
                            packet["body_torque_nm"])

    def test_optional_moteus_status_passes_through_without_driving_simulated_plant(self):
        env = Environment()
        env.accept_actuator({"protocol_version": 1, "source": "cubesat_esp32",
                             "message_type": "actuator_telemetry", "sequence": 2,
                             "body_torque_nm": 0.0, "mode": "SAFE",
                             "system_state": "NOMINAL", "moteus_enabled": True,
                             "moteus_arm_active": True, "moteus_link_valid": True,
                             "moteus_mode": 10, "moteus_fault": 0,
                             "moteus_velocity_rev_s": 1.25,
                             "moteus_motor_torque_nm": 0.001})
        packet = env.telemetry_snapshot()
        self.assertTrue(packet["moteus_link_valid"])
        self.assertEqual(packet["moteus_velocity_rev_s"], 1.25)
        self.assertEqual(packet["simulated_body_torque_nm"], 0.0)

    def test_stale_telemetry_is_not_reported_as_live(self):
        env = Environment()
        env.accept_actuator({"protocol_version": 1, "source": "cubesat_sil",
                             "message_type": "actuator_telemetry",
                             "sequence": 1, "body_torque_nm": 0.0,
                             "mode": "TRACK", "system_state": "NOMINAL"})
        stale_time = env.last_actuator_time + 2.0
        self.assertIsNone(env.telemetry_snapshot(now=stale_time))

    def test_camera_preview_is_wrapped_as_grayscale_bmp_and_expires(self):
        env = Environment()
        frame = bytes(range(256)) * 12
        env.accept_actuator({"protocol_version": 1, "source": "cubesat_esp32",
                             "message_type": "actuator_telemetry", "sequence": 1,
                             "body_torque_nm": 0.0, "target_valid": True,
                             "camera_error_x_px": 12.5,
                             "camera_error_y_px": -3.25})
        env.accept_camera_preview(frame)
        bmp, target_valid, error_x_px, error_y_px, source = env.camera_preview_snapshot()
        self.assertEqual(bmp[:2], b"BM")
        self.assertEqual(int.from_bytes(bmp[18:22], "little"), 64)
        self.assertEqual(int.from_bytes(bmp[22:26], "little"), 48)
        self.assertEqual(int.from_bytes(bmp[2:6], "little"), len(bmp))
        self.assertTrue(target_valid)
        self.assertEqual(error_x_px, 12.5)
        self.assertEqual(error_y_px, -3.25)
        self.assertEqual(source, "physical")
        self.assertIsNone(env.camera_preview_bmp(now=env.camera_preview_time + 2.1))

    def test_sil_telemetry_produces_synthetic_camera_frame_with_paired_error(self):
        env = Environment()
        env.accept_actuator({"protocol_version": 1, "source": "cubesat_sil",
                             "message_type": "actuator_telemetry", "sequence": 1,
                             "body_torque_nm": 0.0, "target_valid": True,
                             "camera_error_x_px": 10.0,
                             "camera_error_y_px": -5.0})
        bmp, target_valid, error_x_px, error_y_px, source = env.camera_preview_snapshot()
        self.assertEqual(source, "simulated")
        self.assertTrue(target_valid)
        self.assertEqual((error_x_px, error_y_px), (10.0, -5.0))
        self.assertEqual(bmp[:2], b"BM")
        # Decode the bottom-up 8-bit BMP pixel payload and confirm a dark target.
        pixel_offset = int.from_bytes(bmp[10:14], "little")
        self.assertEqual(pixel_offset, 14 + 40 + 256 * 4)
        raster = bmp[pixel_offset:]
        dark = []
        for bmp_row in range(CAMERA_PREVIEW_HEIGHT):
            image_y = CAMERA_PREVIEW_HEIGHT - 1 - bmp_row
            row_start = bmp_row * CAMERA_PREVIEW_WIDTH
            for x, value in enumerate(raster[row_start:row_start + CAMERA_PREVIEW_WIDTH]):
                if value == 12:
                    dark.append((x, image_y))
        self.assertTrue(dark)
        self.assertIn(190, raster)
        self.assertGreater(sum(x for x, _ in dark) / len(dark), 31.5)
        self.assertLess(sum(y for _, y in dark) / len(dark), 23.5)

    def test_missing_physical_preview_is_not_synthesized_for_firmware_telemetry(self):
        env = Environment()
        env.accept_actuator({"protocol_version": 1, "source": "cubesat_esp32",
                             "message_type": "actuator_telemetry", "sequence": 1,
                             "body_torque_nm": 0.0})
        self.assertIsNone(env.camera_preview_snapshot())

    def test_physical_firmware_transition_clears_previous_simulated_preview(self):
        env = Environment()
        observation = env.observation()
        frame = env.camera_frame_snapshot(observation["sequence"])
        onboard = OnboardSoftware()
        simulated = onboard.process_observation(observation, camera_frame=frame)
        env.accept_actuator(simulated)
        self.assertEqual(env.camera_preview_snapshot()[4], "simulated")
        env.accept_actuator({"protocol_version": 1, "source": "cubesat_esp32",
                             "message_type": "actuator_telemetry", "sequence": 1,
                             "body_torque_nm": 0.0,
                             "camera_frame_source": "physical"})
        self.assertIsNone(env.camera_preview_snapshot())

    def test_camera_preview_http_headers_match_frame_telemetry(self):
        env = Environment()
        env.accept_actuator({"protocol_version": 1, "source": "cubesat_esp32",
                             "message_type": "actuator_telemetry", "sequence": 1,
                             "body_torque_nm": 0.0, "target_valid": True,
                             "camera_error_x_px": -7.25,
                             "camera_error_y_px": 4.5})
        env.accept_camera_preview(bytes(CAMERA_PREVIEW_BYTES))
        with patch.object(environment_main, "environment", env):
            status, headers, body = dispatch(Handler, "GET", "/api/camera-preview.bmp")
        self.assertEqual(status, 200)
        self.assertTrue(body.startswith(b"BM"))
        self.assertIn(b"X-Camera-Target-Valid: true", headers)
        self.assertIn(b"X-Camera-Error-X-Px: -7.25", headers)
        self.assertIn(b"X-Camera-Error-Y-Px: 4.5", headers)
        self.assertIn(b"X-Camera-Preview-Source: physical", headers)

    def test_camera_preview_endpoint_labels_sil_frame_and_matching_error(self):
        env = Environment()
        env.accept_actuator({"protocol_version": 1, "source": "cubesat_sil",
                             "message_type": "actuator_telemetry", "sequence": 1,
                             "body_torque_nm": 0.0, "target_valid": True,
                             "camera_error_x_px": 5.0,
                             "camera_error_y_px": -10.0})
        with patch.object(environment_main, "environment", env):
            status, headers, body = dispatch(
                Handler, "GET", "/api/camera-preview.bmp")
        self.assertEqual(status, 200)
        self.assertTrue(body.startswith(b"BM"))
        self.assertIn(b"X-Camera-Preview-Source: simulated", headers)
        self.assertIn(b"X-Camera-Error-X-Px: 5.0", headers)
        self.assertIn(b"X-Camera-Error-Y-Px: -10.0", headers)

    def test_camera_preview_requires_exact_frame_size(self):
        with self.assertRaises(ValueError):
            Environment().accept_camera_preview(b"short")

    def test_simulation_watchdog_zeros_stale_body_torque(self):
        env = Environment()
        env.body_torque_nm = 0.02
        env.last_actuator_time = 0.0
        env.advance(dt_s=0.01, now=1.0)
        self.assertEqual(env.snapshot()["body_torque_nm"], 0.0)

    def test_supervisory_commands_are_limited_to_safe_set(self):
        env = Environment()
        command_id = env.queue_command({
            "protocol_version": 1,
            "message_type": "telecommand",
            "command_id": 7,
            "command": "SET_MODE",
            "arguments": {"mode": "RATE_DAMP"},
        })
        self.assertEqual(command_id, 7)
        self.assertEqual(env.commands_after(6)[0]["arguments"]["mode"], "RATE_DAMP")
        retry = {"protocol_version": 1, "message_type": "telecommand",
                 "command_id": 7, "command": "SET_MODE",
                 "arguments": {"mode": "RATE_DAMP"}}
        self.assertEqual(env.queue_command(retry), 7)
        with self.assertRaisesRegex(ValueError, "different content"):
            env.queue_command({**retry, "command": "PING", "arguments": {}})
        env.commands[0]["issued_unix_s"] -= 11.0
        with self.assertRaisesRegex(ValueError, "has expired"):
            env.queue_command(retry)
        with self.assertRaisesRegex(ValueError, "greater than the last accepted ID"):
            env.queue_command({**retry, "command_id": 6})
        with self.assertRaises(ValueError):
            env.queue_command({"protocol_version": 1,
                               "message_type": "telecommand",
                               "command_id": 8, "command": "SET_WHEEL_TORQUE"})
        for invalid_id in (True, 1.5, 1 << 53):
            with self.subTest(command_id=invalid_id), self.assertRaises(ValueError):
                env.queue_command({"protocol_version": 1,
                                   "message_type": "telecommand",
                                   "command_id": invalid_id,
                                   "command": "PING"})

    def test_command_queue_offers_oldest_pending_command_one_at_a_time(self):
        env = Environment()
        for command_id, name in ((21, "PING"), (22, "SET_MODE")):
            arguments = {"mode": "STANDBY"} if name == "SET_MODE" else {}
            env.queue_command({"protocol_version": 1,
                               "message_type": "telecommand",
                               "command_id": command_id,
                               "command": name,
                               "arguments": arguments})

        first_cycle = env.cycle_packet(20)
        self.assertEqual([item["command_id"] for item in first_cycle["commands"]],
                         [21])
        next_cycle = env.cycle_packet(first_cycle["commands"][0]["command_id"])
        self.assertEqual([item["command_id"] for item in next_cycle["commands"]],
                         [22])
        self.assertEqual(next_cycle["commands"][0]["command"], "SET_MODE")


class EnvironmentHTTPHandlerTests(unittest.TestCase):
    def test_environment_sil_environment_cycle_through_http_handlers(self):
        env = Environment()
        onboard = OnboardSoftware()
        with patch.object(environment_main, "environment", env):
            cycle_status, _, cycle_body = dispatch(
                Handler, "GET", "/api/cycle?after=0")
            cycle = json.loads(cycle_body)
            actuator = onboard.process_observation(cycle["observation"])
            actuator_status, _, _ = dispatch(
                Handler, "POST", "/api/actuator", actuator)
        self.assertEqual(cycle_status, 200)
        self.assertEqual(actuator_status, 200)
        self.assertEqual(env.telemetry["source"], "cubesat_sil")
        self.assertEqual(env.body_torque_nm, actuator["body_torque_nm"])

    def test_cycle_get_returns_versioned_observation_json(self):
        env = Environment()
        with patch.object(environment_main, "environment", env):
            status, _, body = dispatch(Handler, "GET", "/api/cycle?after=0")
        self.assertEqual(status, 200)
        cycle = json.loads(body)
        self.assertEqual(cycle["protocol_version"], 1)
        self.assertEqual(cycle["observation"]["message_type"],
                         "environment_observation")

    def test_command_post_queues_only_supervisory_mode_command(self):
        env = Environment()
        with patch.object(environment_main, "environment", env):
            status, _, body = dispatch(Handler, "POST", "/api/command", {
                "protocol_version": 1,
                "message_type": "telecommand",
                "command_id": 18,
                "command": "SET_MODE",
                "arguments": {"mode": "RATE_DAMP"},
            })
        self.assertEqual(status, 202)
        self.assertTrue(json.loads(body)["accepted"])
        self.assertEqual(env.commands_after(17)[0]["arguments"]["mode"],
                         "RATE_DAMP")

    def test_actuator_post_updates_telemetry_and_ack(self):
        env = Environment()
        env.queue_command({"protocol_version": 1,
                           "message_type": "telecommand", "command_id": 18,
                           "command": "PING"})
        packet = {"protocol_version": 1, "message_type": "actuator_telemetry",
                  "sequence": 1, "body_torque_nm": 0.005, "mode": "TRACK",
                  "command_ack": {"command_id": 18, "status": "ACKNOWLEDGED"}}
        with patch.object(environment_main, "environment", env):
            status, _, _ = dispatch(Handler, "POST", "/api/actuator", packet)
        self.assertEqual(status, 200)
        self.assertEqual(env.telemetry["body_torque_nm"], 0.005)
        self.assertEqual(env.latest_ack["command_id"], 18)

    def test_invalid_actuator_ack_does_not_partially_accept_telemetry(self):
        env = Environment()
        env.queue_command({"protocol_version": 1,
                           "message_type": "telecommand", "command_id": 1,
                           "command": "PING"})
        packet = {"protocol_version": 1, "message_type": "actuator_telemetry",
                  "sequence": 1, "body_torque_nm": 0.005, "mode": "TRACK",
                  "command_ack": {"command_id": 1, "status": "INVALID"}}
        with patch.object(environment_main, "environment", env):
            status, _, _ = dispatch(Handler, "POST", "/api/actuator", packet)
        self.assertEqual(status, 400)
        self.assertIsNone(env.telemetry)
        self.assertEqual(env.telemetry_sequence, 0)

    def test_ack_endpoint_rejects_id_not_issued_by_bridge(self):
        env = Environment()
        env.queue_command({"protocol_version": 1,
                           "message_type": "telecommand", "command_id": 1,
                           "command": "PING"})
        with patch.object(environment_main, "environment", env):
            status, _, body = dispatch(Handler, "POST", "/api/ack", {
                "command_id": 2, "status": "ACKNOWLEDGED"})
        self.assertEqual(status, 400)
        self.assertIn("does not match", json.loads(body)["error"])
        self.assertIsNone(env.latest_ack)

    def test_invalid_command_packet_returns_bad_request(self):
        with patch.object(environment_main, "environment", Environment()):
            status, _, body = dispatch(Handler, "POST", "/api/command", {
                "protocol_version": 1,
                "message_type": "telecommand",
                "command_id": 2,
                "command": "SET_WHEEL_TORQUE",
            })
        self.assertEqual(status, 400)
        self.assertIn("supported commands", json.loads(body)["error"])


if __name__ == "__main__":
    unittest.main()
