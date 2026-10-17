import unittest
import csv
import json
from pathlib import Path
from tempfile import TemporaryDirectory
from unittest.mock import patch

from . import main as ground_main
from .main import Handler, LOG_FIELDS, make_command_packet
from http_test_support import dispatch


class GroundStationCommandTests(unittest.TestCase):
    def test_ping_command_has_versioned_supervisory_envelope(self):
        packet = make_command_packet("PING")
        self.assertEqual(packet["protocol_version"], 1)
        self.assertEqual(packet["source"], "ground_station")
        self.assertEqual(packet["message_type"], "telecommand")
        self.assertEqual(packet["command"], "PING")

    def test_mode_command_is_restricted_to_documented_modes(self):
        packet = make_command_packet("SET_MODE", "SAFE")
        self.assertEqual(packet["arguments"], {"mode": "SAFE"})
        with self.assertRaises(ValueError):
            make_command_packet("SET_MODE", "DIRECT_TORQUE")

    def test_speed_test_packet_converts_rpm_to_bounded_rev_per_second(self):
        packet = make_command_packet("SET_WHEEL_SPEED", speed_rpm=3000,
                                     duration_s=2.0)
        self.assertEqual(packet["command"], "SET_WHEEL_SPEED")
        self.assertEqual(packet["arguments"],
                         {"target_rev_s": 50.0, "duration_s": 2.0})

    def test_speed_test_packet_rejects_out_of_range_request(self):
        for rpm, duration in ((3001, 2.0), (-1, 2.0), (100, 0.1),
                              (float("nan"), 2.0)):
            with self.subTest(rpm=rpm, duration=duration), self.assertRaises(ValueError):
                make_command_packet("SET_WHEEL_SPEED", speed_rpm=rpm,
                                    duration_s=duration)

    def test_yaw_step_packet_is_a_bounded_relative_angle(self):
        packet = make_command_packet("SET_YAW_STEP", delta_deg=45.0)
        self.assertEqual(packet["arguments"], {"delta_deg": 45.0})
        for delta in (-181.0, 181.0, float("nan"), True):
            with self.subTest(delta=delta), self.assertRaises(ValueError):
                make_command_packet("SET_YAW_STEP", delta_deg=delta)

    def test_telemetry_csv_schema_includes_magnetometer_frame_and_vector(self):
        for field in ("mag_valid", "mag_frame", "mag_age_s", "mag_x_uT",
                      "mag_y_uT", "mag_z_uT"):
            with self.subTest(field=field):
                self.assertIn(field, LOG_FIELDS)
        self.assertIn("camera_error_y_px", LOG_FIELDS)
        self.assertIn("camera_focal_length_x_px", LOG_FIELDS)
        self.assertIn("camera_frame_sequence", LOG_FIELDS)
        self.assertIn("camera_frame_source", LOG_FIELDS)

    def test_telemetry_csv_schema_includes_yaw_step_result(self):
        for field in ("yaw_estimate_rad", "yaw_estimate_source",
                      "yaw_step_target_rad", "yaw_step_error_rad",
                      "yaw_step_status", "yaw_step_elapsed_s"):
            self.assertIn(field, LOG_FIELDS)

    def test_telemetry_logging_preserves_legacy_file_when_schema_changes(self):
        with TemporaryDirectory() as temp_dir:
            log_path = Path(temp_dir) / "telemetry.csv"
            legacy_header = ["ground_sequence", "mode"]
            with log_path.open("w", newline="", encoding="utf-8") as stream:
                writer = csv.writer(stream)
                writer.writerow(legacy_header)
                writer.writerow([1, "TRACK"])
            with (patch.object(ground_main, "LOG_PATH", log_path),
                  patch.object(ground_main, "_last_logged_sequence", -1)):
                ground_main.log_telemetry({
                    "ground_sequence": 2, "sequence": 2, "mode": "TRACK",
                    "virtual_axis_torque_nm": {"pitch": 0.0, "roll": 0.01},
                })

            legacy_paths = list(Path(temp_dir).glob("telemetry.legacy-*.csv"))
            self.assertEqual(len(legacy_paths), 1)
            with legacy_paths[0].open(newline="", encoding="utf-8") as stream:
                old_rows = list(csv.DictReader(stream))
            self.assertEqual(old_rows, [{"ground_sequence": "1", "mode": "TRACK"}])
            with log_path.open(newline="", encoding="utf-8") as stream:
                reader = csv.DictReader(stream)
                self.assertEqual(reader.fieldnames, LOG_FIELDS)
                new_rows = list(reader)
            self.assertEqual(len(new_rows), 1)
            self.assertEqual(new_rows[0]["virtual_axis_torque_nm"],
                             '{"pitch": 0.0, "roll": 0.01}')


class GroundStationHTTPHandlerTests(unittest.TestCase):
    def test_camera_preview_proxy_preserves_frame_matched_error_headers(self):
        with patch.object(ground_main, "bridge_request", return_value=(
                200, b"BMimage", {"x-camera-target-valid": "true",
                                  "x-camera-error-x-px": "12.5",
                                  "x-camera-error-y-px": "-4.5",
                                  "x-camera-preview-source": "simulated"})) as bridge:
            status, headers, body = dispatch(
                Handler, "GET", "/api/camera-preview.bmp")
        self.assertEqual(status, 200)
        self.assertTrue(body.startswith(b"BM"))
        self.assertIn(b"X-Camera-Target-Valid: true", headers)
        self.assertIn(b"X-Camera-Error-X-Px: 12.5", headers)
        self.assertIn(b"X-Camera-Error-Y-Px: -4.5", headers)
        self.assertIn(b"X-Camera-Preview-Source: simulated", headers)
        self.assertTrue(bridge.call_args.kwargs["include_headers"])

    def test_camera_preview_proxy_handles_invalid_target_without_error_headers(self):
        with patch.object(ground_main, "bridge_request", return_value=(
                200, b"BMimage", {"x-camera-target-valid": "false",
                                  "x-camera-preview-source": "simulated"})):
            status, headers, body = dispatch(
                Handler, "GET", "/api/camera-preview.bmp")
        self.assertEqual(status, 200)
        self.assertTrue(body.startswith(b"BM"))
        self.assertIn(b"X-Camera-Target-Valid: false", headers)
        self.assertNotIn(b"X-Camera-Error-X-Px", headers)

    def test_dashboard_yaw_rate_fallback_uses_project_yaw_axis(self):
        status, _, body = dispatch(Handler, "GET", "/")
        self.assertEqual(status, 200)
        self.assertIn(b"t.yaw_rate_rad_s ?? t.gyro_y_rad_s", body)

    def test_dashboard_history_preserves_occlusion_gaps_on_simulation_time_axis(self):
        status, _, body = dispatch(Handler, "GET", "/")
        self.assertEqual(status, 200)
        self.assertIn(b"time_s:Number(t.simulation_time_s)", body)
        self.assertIn(b"?t.target_error_rad:null", body)
        self.assertIn(b"if(sample.error_rad===null){drawing=false;continue;}", body)

    def test_dashboard_marks_stale_sensor_data_and_breaks_error_history(self):
        status, _, body = dispatch(Handler, "GET", "/")
        self.assertEqual(status, 200)
        self.assertIn(b"t.camera_age_s <= 0.1", body)
        self.assertIn(b"t.imu_age_s <= 0.1", body)
        self.assertIn(b"STALE (", body)
        self.assertIn(b"targetFresh&&imuFresh", body)
        self.assertIn(b"target_valid: targetFresh && imuFresh", body)

    def test_dashboard_preview_overlay_scales_qvga_error_to_thumbnail_size(self):
        status, _, body = dispatch(Handler, "GET", "/")
        self.assertEqual(status, 200)
        self.assertIn(b"lastPreviewError.errorX*c.width/320", body)
        self.assertIn(b"lastPreviewError.errorY*c.height/240", body)
        self.assertIn(b"Camera frame:", body)
        self.assertIn(b"X-Camera-Preview-Source", body)

    def test_telemetry_get_proxies_packet_and_logs_csv_row(self):
        packet = {"ground_sequence": 4, "sequence": 7,
                  "simulation_time_s": 1.2, "mode": "TRACK",
                  "body_torque_nm": 0.01,
                  "camera_focal_length_x_px": 277.0}
        with TemporaryDirectory() as temp_dir:
            log_path = Path(temp_dir) / "telemetry.csv"
            with (patch.object(ground_main, "LOG_PATH", log_path),
                  patch.object(ground_main, "_last_logged_sequence", -1),
                  patch.object(ground_main, "bridge_request",
                               return_value=(200, json.dumps(packet).encode()))):
                status, _, body = dispatch(Handler, "GET", "/api/telemetry")
            self.assertEqual(status, 200)
            self.assertEqual(json.loads(body)["ground_sequence"], 4)
            with log_path.open(newline="", encoding="utf-8") as stream:
                rows = list(csv.DictReader(stream))
            self.assertEqual(len(rows), 1)
            self.assertEqual(rows[0]["mode"], "TRACK")
            self.assertEqual(rows[0]["camera_focal_length_x_px"], "277.0")

    def test_command_post_forwards_versioned_envelope_to_bridge(self):
        with patch.object(ground_main, "bridge_request",
                          return_value=(202, b'{"accepted":true}')) as bridge:
            status, _, body = dispatch(Handler, "POST", "/api/command", {
                "command": "SET_MODE", "mode": "SAFE"})
        self.assertEqual(status, 202)
        self.assertTrue(json.loads(body)["accepted"])
        args, kwargs = bridge.call_args
        self.assertEqual(args[:2], ("/api/command", "POST"))
        forwarded = args[2]
        self.assertEqual(forwarded["protocol_version"], 1)
        self.assertEqual(forwarded["arguments"], {"mode": "SAFE"})

    def test_speed_test_post_forwards_velocity_and_duration(self):
        with patch.object(ground_main, "bridge_request",
                          return_value=(202, b'{"accepted":true}')) as bridge:
            status, _, body = dispatch(Handler, "POST", "/api/command", {
                "command": "SET_WHEEL_SPEED", "speed_rpm": 1800,
                "duration_s": 1.5})
        self.assertEqual(status, 202)
        self.assertTrue(json.loads(body)["accepted"])
        self.assertEqual(bridge.call_args.args[2]["arguments"],
                         {"target_rev_s": 30.0, "duration_s": 1.5})

    def test_yaw_step_post_forwards_relative_angle(self):
        with patch.object(ground_main, "bridge_request",
                          return_value=(202, b'{"accepted":true}')) as bridge:
            status, _, body = dispatch(Handler, "POST", "/api/command", {
                "command": "SET_YAW_STEP", "delta_deg": 45.0})
        self.assertEqual(status, 202)
        self.assertTrue(json.loads(body)["accepted"])
        self.assertEqual(bridge.call_args.args[2]["arguments"],
                         {"delta_deg": 45.0})

    def test_command_post_rejects_unsupported_mode(self):
        status, _, body = dispatch(Handler, "POST", "/api/command", {
            "command": "SET_MODE", "mode": "DIRECT_TORQUE"})
        self.assertEqual(status, 400)
        self.assertIn("supported commands", json.loads(body)["error"])

    def test_command_post_requires_json_object(self):
        for payload in ([], b"null"):
            with self.subTest(payload=payload):
                status, _, body = dispatch(Handler, "POST", "/api/command",
                                          payload)
                self.assertEqual(status, 400)
                self.assertEqual(json.loads(body)["error"],
                                 "request body must be a JSON object")

    def test_telemetry_csv_schema_includes_physical_moteus_status(self):
        for field in ("moteus_enabled", "moteus_link_valid", "moteus_arm_active", "moteus_mode",
                      "moteus_fault", "moteus_position_rev",
                      "moteus_velocity_rev_s", "moteus_motor_torque_nm",
                      "moteus_commanded_motor_torque_nm"):
            with self.subTest(field=field):
                self.assertIn(field, LOG_FIELDS)


if __name__ == "__main__":
    unittest.main()
