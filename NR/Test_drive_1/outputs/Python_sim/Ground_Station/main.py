"""Local web Ground Station for the simulation bridge (standard library)."""

from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import csv
import json
import math
import os
from pathlib import Path
import threading
import time
from urllib.error import HTTPError, URLError
from urllib.request import Request, urlopen


HOST = "127.0.0.1"
PORT = 8001
ENV_URL = os.environ.get("CUBESAT_SIM_URL", "http://127.0.0.1:8000").rstrip("/")
UI_PATH = Path(__file__).with_name("ui.html")
LOG_PATH = Path(__file__).with_name("logs") / "telemetry.csv"
LOG_FIELDS = ["ground_sequence", "sequence", "simulation_time_s", "mode",
              "system_state", "target_valid", "target_error_rad",
              "camera_error_x_px", "camera_error_y_px", "imu_valid",
              "camera_frame_sequence",
              "camera_frame_source",
              "camera_focal_length_x_px",
              "camera_error_y_rad", "pitch_estimate_rad", "roll_estimate_rad",
              "virtual_axis_torque_nm",
              "gyro_x_rad_s", "gyro_y_rad_s",
              "gyro_z_rad_s", "yaw_rate_rad_s", "accel_x_m_s2", "accel_y_m_s2", "accel_z_m_s2",
              "yaw_estimate_rad", "yaw_estimate_valid", "yaw_estimate_source",
              "yaw_step_target_rad", "yaw_step_error_rad", "yaw_step_status",
              "yaw_step_elapsed_s",
              "mag_valid", "mag_frame", "mag_age_s", "mag_x_uT", "mag_y_uT", "mag_z_uT",
              "camera_age_s", "imu_age_s", "body_torque_nm",
              "pitch_body_torque_nm", "roll_body_torque_nm",
              "simulated_body_torque_nm", "simulated_wheel_motor_torque_nm",
              "simulated_pitch_body_torque_nm", "simulated_roll_body_torque_nm",
              "simulated_wheel_speed_rad_s", "simulated_wheel_saturated",
              "moteus_enabled", "moteus_link_valid", "moteus_arm_active", "moteus_mode",
              "moteus_fault", "moteus_position_rev", "moteus_velocity_rev_s",
              "moteus_motor_torque_nm", "moteus_commanded_motor_torque_nm",
              "motor_speed_target_rev_s", "motor_speed_measured_rev_s",
              "motor_speed_error_percent", "motor_speed_test_status",
              "motor_speed_test_elapsed_s",
              "fault", "command_ack"]

_command_id = int(time.time() * 1000)
_command_lock = threading.Lock()
_log_lock = threading.Lock()
_last_logged_sequence = -1


def bridge_request(path, method="GET", payload=None, timeout=1.0,
                   include_headers=False):
    data = None if payload is None else json.dumps(payload).encode("utf-8")
    request = Request(ENV_URL + path, data=data, method=method,
                      headers={"Content-Type": "application/json"})
    with urlopen(request, timeout=timeout) as response:
        result = (response.status, response.read())
        if include_headers:
            return (*result, dict(response.headers.items()))
        return result


def make_command_packet(command, mode=None, speed_rpm=None, duration_s=None,
                        delta_deg=None):
    global _command_id
    if command == "PING":
        arguments = {}
    elif command == "SET_MODE" and mode in {
        "SAFE", "STANDBY", "RATE_DAMP", "ACQUIRE", "TRACK"
    }:
        arguments = {"mode": mode}
    elif command == "SET_WHEEL_SPEED":
        if (isinstance(speed_rpm, bool) or not isinstance(speed_rpm, (int, float))
                or not 0.0 <= speed_rpm <= 3000.0):
            raise ValueError("speed_rpm must be a number in [0, 3000]")
        if (isinstance(duration_s, bool) or not isinstance(duration_s, (int, float))
                or not 0.5 <= duration_s <= 5.0):
            raise ValueError("duration_s must be a number in [0.5, 5]")
        arguments = {"target_rev_s": float(speed_rpm) / 60.0,
                    "duration_s": float(duration_s)}
    elif command == "SET_YAW_STEP":
        if (isinstance(delta_deg, bool) or not isinstance(delta_deg, (int, float))
                or not math.isfinite(delta_deg) or not -180.0 <= delta_deg <= 180.0):
            raise ValueError("delta_deg must be a number in [-180, 180]")
        arguments = {"delta_deg": float(delta_deg)}
    else:
        raise ValueError("supported commands: PING, SET_MODE, SET_WHEEL_SPEED, SET_YAW_STEP")
    with _command_lock:
        _command_id = max(_command_id + 1, int(time.time() * 1000))
        command_id = _command_id
    return {
        "protocol_version": 1,
        "source": "ground_station",
        "message_type": "telecommand",
        "command_id": command_id,
        "command": command,
        "arguments": arguments,
        "issued_unix_s": time.time(),
    }


def log_telemetry(packet):
    global _last_logged_sequence
    sequence = packet.get("ground_sequence", packet.get("sequence"))
    if not isinstance(sequence, int):
        return
    with _log_lock:
        if sequence <= _last_logged_sequence:
            return
        LOG_PATH.parent.mkdir(parents=True, exist_ok=True)
        if LOG_PATH.exists():
            with LOG_PATH.open(newline="", encoding="utf-8") as stream:
                existing_fields = next(csv.reader(stream), [])
            if existing_fields != LOG_FIELDS:
                stamp = time.strftime("%Y%m%d-%H%M%S")
                legacy_path = LOG_PATH.with_name(
                    f"{LOG_PATH.stem}.legacy-{stamp}{LOG_PATH.suffix}")
                suffix = 1
                while legacy_path.exists():
                    legacy_path = LOG_PATH.with_name(
                        f"{LOG_PATH.stem}.legacy-{stamp}-{suffix}"
                        f"{LOG_PATH.suffix}")
                    suffix += 1
                LOG_PATH.replace(legacy_path)
        new_file = not LOG_PATH.exists()
        with LOG_PATH.open("a", newline="", encoding="utf-8") as stream:
            writer = csv.DictWriter(stream, fieldnames=LOG_FIELDS,
                                    extrasaction="ignore")
            if new_file:
                writer.writeheader()
            row = dict(packet)
            if isinstance(row.get("command_ack"), dict):
                row["command_ack"] = json.dumps(row["command_ack"], sort_keys=True)
            if isinstance(row.get("virtual_axis_torque_nm"), dict):
                row["virtual_axis_torque_nm"] = json.dumps(
                    row["virtual_axis_torque_nm"], sort_keys=True)
            writer.writerow(row)
        _last_logged_sequence = sequence


class Handler(BaseHTTPRequestHandler):
    def send_bytes(self, status, body, content_type, response_headers=None):
        self.send_response(status)
        self.send_header("Content-Type", content_type)
        self.send_header("Content-Length", str(len(body)))
        self.send_header("Cache-Control", "no-store")
        for name, value in (response_headers or {}).items():
            if name in {"X-Camera-Target-Valid", "X-Camera-Error-X-Px",
                        "X-Camera-Error-Y-Px", "X-Camera-Preview-Source"}:
                self.send_header(name, value)
        self.end_headers()
        self.wfile.write(body)

    def send_json(self, status, payload):
        self.send_bytes(status, json.dumps(payload).encode("utf-8"),
                        "application/json; charset=utf-8")

    def do_GET(self):
        if self.path == "/":
            self.send_bytes(200, UI_PATH.read_bytes(), "text/html; charset=utf-8")
        elif self.path == "/api/telemetry":
            try:
                status, body = bridge_request("/api/telemetry")
                if status == 200:
                    log_telemetry(json.loads(body.decode("utf-8")))
                self.send_bytes(status, body, "application/json; charset=utf-8")
            except (HTTPError, URLError, TimeoutError) as exc:
                code = exc.code if isinstance(exc, HTTPError) else 503
                self.send_json(code, {"error": str(exc)})
        elif self.path == "/api/state":
            try:
                status, body = bridge_request("/api/state")
                self.send_bytes(status, body, "application/json; charset=utf-8")
            except (HTTPError, URLError, TimeoutError) as exc:
                self.send_json(503, {"error": str(exc)})
        elif self.path == "/api/camera-preview.bmp":
            try:
                status, body, headers = bridge_request(
                    "/api/camera-preview.bmp", include_headers=True)
                headers = {name.lower(): value for name, value in headers.items()}
                preview_headers = {
                    "X-Camera-Target-Valid": headers.get("x-camera-target-valid"),
                    "X-Camera-Error-X-Px": headers.get("x-camera-error-x-px"),
                    "X-Camera-Error-Y-Px": headers.get("x-camera-error-y-px"),
                    "X-Camera-Preview-Source": headers.get("x-camera-preview-source"),
                }
                preview_headers = {name: value for name, value in preview_headers.items()
                                   if value is not None}
                self.send_bytes(status, body, "image/bmp" if status == 200 else
                                "application/json; charset=utf-8",
                                response_headers=preview_headers)
            except (HTTPError, URLError, TimeoutError) as exc:
                self.send_json(503, {"error": str(exc)})
        elif self.path == "/telemetry.csv":
            if LOG_PATH.exists():
                self.send_bytes(200, LOG_PATH.read_bytes(), "text/csv; charset=utf-8")
            else:
                self.send_json(404, {"error": "no telemetry has been logged"})
        else:
            self.send_json(404, {"error": "not found"})

    def do_POST(self):
        if self.path != "/api/command":
            self.send_json(404, {"error": "not found"})
            return
        try:
            size = int(self.headers.get("Content-Length", "0"))
            if not 0 < size <= 4096:
                raise ValueError("request body must be 1..4096 bytes")
            request = json.loads(self.rfile.read(size))
            if not isinstance(request, dict):
                raise ValueError("request body must be a JSON object")
            packet = make_command_packet(request.get("command"),
                                         request.get("mode"),
                                         request.get("speed_rpm"),
                                         request.get("duration_s"),
                                         request.get("delta_deg"))
            status, body = bridge_request("/api/command", "POST", packet)
            self.send_bytes(status, body, "application/json; charset=utf-8")
        except (ValueError, TypeError, json.JSONDecodeError) as exc:
            self.send_json(400, {"error": str(exc)})
        except (HTTPError, URLError, TimeoutError) as exc:
            self.send_json(503, {"error": str(exc)})

    def log_message(self, _format, *_args):
        pass


def main():
    server = ThreadingHTTPServer((HOST, PORT), Handler)
    print(f"Ground Station: http://{HOST}:{PORT}")
    print(f"Simulation bridge: {ENV_URL}")
    print(f"Telemetry CSV: {LOG_PATH}")
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        print("\nGround Station stopped.")
    finally:
        server.server_close()


if __name__ == "__main__":
    main()
