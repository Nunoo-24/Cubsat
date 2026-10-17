import json
import math
import time
import unittest
from unittest.mock import patch

from CubeSat_ESP32.sil import OnboardSoftware
from Ground_Station.main import make_command_packet
from Ground_Station import main as ground_station_main
from Ground_Station.main import Handler as GroundStationHandler
from Sim_Environment.camera_transport import decode_rle_frame
from Sim_Environment.main import Environment
from Sim_Environment import main as environment_main
from Sim_Environment.main import Handler as EnvironmentHandler
from http_test_support import dispatch


class WheelSpeedCommandIntegrationTests(unittest.TestCase):
    def test_ground_station_speed_request_reaches_sil_and_simulated_wheel(self):
        environment = Environment()
        onboard = OnboardSoftware()
        now = 100.0

        safe = make_command_packet("SET_MODE", "SAFE")
        environment.queue_command(safe)
        cycle = environment.cycle_packet(onboard.last_command_id)
        for command in cycle["commands"]:
            onboard.receive_command(command, now=now)
        observation = cycle["observation"]
        frame = environment.camera_frame_snapshot(observation["sequence"])
        actuator = onboard.process_observation(
            observation, now=now, camera_frame=frame
        )
        environment.accept_actuator(actuator)
        environment.advance(0.02, now=time.monotonic())
        self.assertEqual(environment.latest_ack["status"], "ACKNOWLEDGED")

        speed = make_command_packet("SET_WHEEL_SPEED", speed_rpm=3000,
                                    duration_s=2.0)
        environment.queue_command(speed)
        cycle = environment.cycle_packet(onboard.last_command_id)
        for command in cycle["commands"]:
            onboard.receive_command(command, now=now + 0.02)
        observation = cycle["observation"]
        frame = environment.camera_frame_snapshot(observation["sequence"])
        actuator = onboard.process_observation(
            observation, now=now + 0.02, camera_frame=frame
        )
        environment.accept_actuator(actuator)
        environment.advance(0.02, now=time.monotonic())

        self.assertEqual(actuator["motor_speed_target_rev_s"], 50.0)
        self.assertEqual(actuator["mode"], "WHEEL_SPEED_TEST")
        self.assertEqual(actuator["command_ack"]["status"], "ACKNOWLEDGED")
        self.assertGreater(environment.plant.wheel.speed_rad_s, 0.0)
        self.assertLess(environment.plant.rate_rad_s, 0.0)
        self.assertEqual(environment.telemetry_snapshot()["motor_speed_test_status"],
                         "RUNNING")


class YawStepCommandIntegrationTests(unittest.TestCase):
    def test_ground_station_yaw_step_crosses_http_handlers_and_settles(self):
        environment = Environment()
        onboard = OnboardSoftware()
        current_time = time.monotonic()

        def bridge_request(method, path, payload=None):
            return dispatch(EnvironmentHandler, method, path, payload)

        def ground_station_bridge_request(path, method="GET", payload=None,
                                          timeout=1.0):
            status, _, body = dispatch(EnvironmentHandler, method, path, payload)
            return status, body

        def send_ground_station_command(payload):
            status, _, body = dispatch(
                GroundStationHandler, "POST", "/api/command", payload)
            return status, json.loads(body)

        def exchange_one_cycle():
            nonlocal current_time
            status, _, body = bridge_request(
                "GET", f"/api/cycle?after={onboard.last_command_id}")
            self.assertEqual(status, 200)
            cycle = json.loads(body)
            for command in cycle["commands"]:
                onboard.receive_command(command, now=current_time)
            observation = cycle["observation"]
            frame = environment.camera_frame_snapshot(observation["sequence"])
            actuator = onboard.process_observation(
                observation, now=current_time, camera_frame=frame)
            status, _, body = bridge_request("POST", "/api/actuator", actuator)
            self.assertEqual(status, 200)
            environment.advance(0.02)
            current_time += 0.02
            return actuator, json.loads(body)

        with (patch.object(environment_main, "environment", environment),
              patch.object(ground_station_main, "bridge_request",
                           side_effect=ground_station_bridge_request),
              patch.object(environment_main.time, "monotonic",
                           side_effect=lambda: current_time)):
            status, _ = send_ground_station_command(
                {"command": "SET_MODE", "mode": "SAFE"})
            self.assertEqual(status, 202)
            safe_telemetry, _ = exchange_one_cycle()
            self.assertEqual(safe_telemetry["command_ack"]["status"],
                             "ACKNOWLEDGED")
            self.assertTrue(onboard.last_attitude_valid)

            status, _ = send_ground_station_command(
                {"command": "SET_YAW_STEP", "delta_deg": 45.0})
            self.assertEqual(status, 202)
            yaw_command_id = environment.commands[-1]["command_id"]

            final = None
            yaw_step_ack = None
            for _ in range(500):
                actuator, _ = exchange_one_cycle()
                if actuator.get("command_ack", {}).get("command_id") == yaw_command_id:
                    yaw_step_ack = actuator["command_ack"]
                if actuator["yaw_step_status"] == "COMPLETE":
                    final = actuator
                    break

        self.assertIsNotNone(final, "yaw step must complete before timeout")
        self.assertEqual(yaw_step_ack["status"], "ACKNOWLEDGED")
        self.assertLessEqual(abs(final["yaw_step_error_rad"]), math.radians(1.0))
        self.assertGreaterEqual(final["yaw_step_elapsed_s"], 1.0)
        self.assertLess(final["yaw_step_elapsed_s"], 10.0)
        self.assertEqual(final["mode"], "STANDBY")
        self.assertAlmostEqual(
            environment.telemetry_snapshot()["yaw_step_target_rad"],
            final["yaw_step_target_rad"])


class VisualPointingIntegrationTests(unittest.TestCase):
    def test_environment_image_sil_control_plant_and_ground_telemetry(self):
        environment = Environment()
        environment.apply({"target_xyz_m": {"x": 1.154700538,
                                             "y": 0.0, "z": 2.0}})
        onboard = OnboardSoftware()
        current_time = 100.0

        def environment_request(path, method="GET", payload=None):
            return dispatch(EnvironmentHandler, method, path, payload)

        def ground_station_request(path, method="GET", payload=None,
                                  timeout=1.0):
            status, _, body = dispatch(EnvironmentHandler, method, path, payload)
            return status, body

        final_actuator = None
        with (patch.object(environment_main, "environment", environment),
              patch.object(environment_main.time, "monotonic",
                           side_effect=lambda: current_time),
              patch.object(ground_station_main, "bridge_request",
                           side_effect=ground_station_request),
              patch.object(ground_station_main, "log_telemetry")):
            for _ in range(300):
                status, _, body = environment_request(
                    f"/api/cycle?after={onboard.last_command_id}")
                self.assertEqual(status, 200)
                cycle = json.loads(body)
                observation = cycle["observation"]
                status, headers, encoded_frame = environment_request(
                    "/api/camera-frame.rle?sequence=" +
                    str(observation["sequence"]))
                self.assertEqual(status, 200)
                self.assertIn(b"X-Camera-Frame-Encoding: RLE1", headers)
                self.assertIn(
                    f"X-Camera-Frame-Sequence: {observation['sequence']}".encode(),
                    headers)
                frame = decode_rle_frame(
                    encoded_frame,
                    environment_main.CAMERA_WIDTH_PX *
                    environment_main.CAMERA_HEIGHT_PX)
                final_actuator = onboard.process_observation(
                    observation, now=current_time, camera_frame=frame)
                status, _, body = environment_request(
                    "/api/actuator", "POST", final_actuator)
                self.assertEqual(status, 200)
                environment.advance(0.02, now=current_time)
                current_time += 0.02
                measured_error_deg = abs(math.degrees(
                    environment.snapshot()["target_error_rad"]))
                camera_error_deg = abs(math.degrees(math.atan(
                    final_actuator["camera_error_x_px"] /
                    final_actuator["camera_focal_length_x_px"])))
                if camera_error_deg <= 2.0:
                    break

            self.assertIsNotNone(final_actuator)
            self.assertLessEqual(measured_error_deg, 2.0)
            self.assertLess(environment.plant.time_s, 6.0)
            self.assertEqual(final_actuator["camera_frame_source"], "simulated")
            self.assertTrue(final_actuator["target_valid"])
            self.assertLessEqual(camera_error_deg, 2.0)
            self.assertLessEqual(measured_error_deg, 2.0)
            self.assertLess(final_actuator["simulation_time_s"], 6.0)
            self.assertLessEqual(abs(final_actuator["body_torque_nm"]), 0.05)

            status, _, body = dispatch(GroundStationHandler, "GET",
                                       "/api/telemetry")
            self.assertEqual(status, 200)
            ground_telemetry = json.loads(body)
            self.assertEqual(ground_telemetry["source"], "cubesat_sil")
            self.assertEqual(ground_telemetry["ground_sequence"],
                             environment.telemetry_sequence)
            self.assertTrue(ground_telemetry["target_valid"])
            self.assertLessEqual(abs(math.degrees(math.atan(
                ground_telemetry["camera_error_x_px"] /
                ground_telemetry["camera_focal_length_x_px"]))), 2.0)


if __name__ == "__main__":
    unittest.main()
