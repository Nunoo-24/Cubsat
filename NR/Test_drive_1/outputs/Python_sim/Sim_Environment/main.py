"""Browser-based three-axis gimbal simulation environment (stdlib only)."""

from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import json
import math
from pathlib import Path
import random
import struct
import threading
import time
from urllib.parse import parse_qs, urlparse

from .camera_transport import MAX_ENCODED_BYTES, encode_rle_frame
from .model import YawPlant, target_bearing_error


HOST = "127.0.0.1"
PORT = 8000
DT_S = 0.01
PROTOCOL_VERSION = 1
MAX_SAFE_COMMAND_ID = (1 << 53) - 1
CAMERA_WIDTH_PX = 320
CAMERA_HEIGHT_PX = 240
CAMERA_FOCAL_PX = 277.0  # Placeholder for a 60-degree horizontal FOV.
CAMERA_HALF_FOV_RAD = math.atan((CAMERA_WIDTH_PX / 2) / CAMERA_FOCAL_PX)
CAMERA_HALF_VERTICAL_FOV_RAD = math.atan((CAMERA_HEIGHT_PX / 2) /
                                         CAMERA_FOCAL_PX)
TELEMETRY_STALE_AFTER_S = 1.5
CAMERA_PREVIEW_WIDTH = 64
CAMERA_PREVIEW_HEIGHT = 48
CAMERA_PREVIEW_BYTES = CAMERA_PREVIEW_WIDTH * CAMERA_PREVIEW_HEIGHT
CAMERA_PREVIEW_STALE_AFTER_S = 2.0
CAMERA_FRAME_HISTORY = 8
# Generic fixed field for sensor-interface testing; not a location model.
MAGNETIC_FIELD_WORLD_UT = {"x": 20.0, "y": 0.0, "z": 45.0}
UI_PATH = Path(__file__).with_name("ui.html")


def _supported_protocol_version(value):
    return (not isinstance(value, bool) and
            isinstance(value, (int, float)) and value == PROTOCOL_VERSION)


class Environment:
    def __init__(self):
        self.plant = YawPlant()
        self.target_xyz_m = {"x": 0.0, "y": 0.0, "z": 2.0}
        self.target_scale = 0.1
        self.external_torque_nm = {"x": 0.0, "y": 0.0, "z": 0.0}
        self.torque_until = 0.0
        self.body_torque_nm = 0.0
        self.virtual_pitch_torque_nm = 0.0
        self.virtual_roll_torque_nm = 0.0
        self.motor_speed_target_rev_s = None
        self.last_actuator_time = 0.0
        self.camera_occluded_until = 0.0
        self.observation_sequence = 0
        self.telemetry_sequence = 0
        self.telemetry = None
        self.camera_frames = {}
        self.camera_preview_gray = None
        self.camera_preview_time = 0.0
        self.camera_preview_source = None
        self.camera_preview_target_valid = False
        self.camera_preview_error_x_px = None
        self.camera_preview_error_y_px = None
        self.commands = []
        self.last_command_id = 0
        self.latest_ack = None
        self.rng = random.Random(20261016)
        self.lock = threading.RLock()

    def apply(self, payload):
        if not isinstance(payload, dict):
            raise ValueError("control payload must be an object")
        allowed_fields = {"target_xyz_m", "target_scale", "external_torque_nm",
                          "duration_s", "camera_occlusion_s"}
        unknown_fields = set(payload) - allowed_fields
        if unknown_fields:
            names = ", ".join(sorted(unknown_fields))
            raise ValueError(f"unsupported control field(s): {names}")
        if "duration_s" in payload and "external_torque_nm" not in payload:
            raise ValueError("duration_s requires external_torque_nm")
        with self.lock:
            new_point = None
            new_scale = None
            new_torque = None
            torque_duration = None
            occlusion_duration = None
            if "target_xyz_m" in payload:
                point = payload["target_xyz_m"]
                if not isinstance(point, dict):
                    raise ValueError("target_xyz_m must be an object")
                new_point = {}
                for axis in ("x", "y", "z"):
                    value = float(point[axis])
                    if not math.isfinite(value):
                        raise ValueError("target coordinates must be finite")
                    new_point[axis] = value
                if new_point["z"] <= 0:
                    raise ValueError("target z must be in front of the camera")
            if "target_scale" in payload:
                scale = float(payload["target_scale"])
                if not math.isfinite(scale) or not 0.001 <= scale <= 2.0:
                    raise ValueError("target_scale must be between 0.001 and 2 m")
                new_scale = scale
            if "external_torque_nm" in payload:
                torque = payload["external_torque_nm"]
                if not isinstance(torque, dict):
                    raise ValueError("external_torque_nm must be an object")
                new_torque = {}
                for axis in ("x", "y", "z"):
                    value = float(torque[axis])
                    if not math.isfinite(value) or abs(value) > 1.0:
                        raise ValueError("torque components must be within ±1 N m")
                    new_torque[axis] = value
                duration = float(payload.get("duration_s", 0.0))
                if not math.isfinite(duration) or not 0.0 <= duration <= 60.0:
                    raise ValueError("duration_s must be between 0 and 60 s")
                torque_duration = duration
            if "camera_occlusion_s" in payload:
                duration = float(payload["camera_occlusion_s"])
                if not math.isfinite(duration) or not 0.0 <= duration <= 60.0:
                    raise ValueError("camera_occlusion_s must be between 0 and 60 s")
                occlusion_duration = duration

            # Commit only after every requested field has passed validation.
            # Scene events use simulation time so a pause or a slow host does
            # not change their duration relative to the displayed plant clock.
            if new_point is not None:
                self.target_xyz_m = new_point
            if new_scale is not None:
                self.target_scale = new_scale
            if new_torque is not None:
                self.external_torque_nm = new_torque
                self.torque_until = self.plant.time_s + torque_duration
            if occlusion_duration is not None:
                self.camera_occluded_until = (
                    self.plant.time_s + occlusion_duration
                )

    def observation(self, now=None):
        """Create a versioned synthetic sensor packet for the onboard loop."""
        with self.lock:
            now = self.plant.time_s if now is None else now
            target = self.target_xyz_m
            cy, sy = math.cos(self.plant.yaw_rad), math.sin(self.plant.yaw_rad)
            cp, sp = math.cos(self.plant.pitch_rad), math.sin(self.plant.pitch_rad)
            cr, sr = math.cos(self.plant.roll_rad), math.sin(self.plant.roll_rad)
            # Body-to-world rotation is Ry(yaw) * Rx(pitch) * Rz(roll).
            # Apply its transpose to project the inertial target into the camera.
            x1 = cy * target["x"] - sy * target["z"]
            y1 = target["y"]
            z1 = sy * target["x"] + cy * target["z"]
            x2 = x1
            y2 = cp * y1 + sp * z1
            z2 = -sp * y1 + cp * z1
            camera_x = cr * x2 + sr * y2
            camera_y = -sr * x2 + cr * y2
            camera_z = z2
            camera_error = math.atan2(camera_x, camera_z)
            # The synthetic target is a camera-facing circular marker. Its
            # apparent size follows pinhole projection using optical-axis
            # depth, not slant range (which would shrink it off-axis).
            apparent_diameter_px = (
                CAMERA_FOCAL_PX * self.target_scale / camera_z
                if camera_z > 0.0 else 0.0
            )
            vertical_error = math.atan2(camera_y, camera_z)
            in_view = (now >= self.camera_occluded_until and camera_z > 0.0
                       and abs(camera_error) <= CAMERA_HALF_FOV_RAD
                       and abs(vertical_error) <= CAMERA_HALF_VERTICAL_FOV_RAD)
            pixel_error = None
            pixel_error_y = None
            if in_view:
                pixel_error = round(CAMERA_FOCAL_PX * math.tan(camera_error))
                pixel_error_y = round(CAMERA_FOCAL_PX * camera_y / camera_z)
            camera_frame = self._render_camera_frame(
                pixel_error, pixel_error_y, apparent_diameter_px
            )
            dark_pixel_area = camera_frame.count(12)
            visible = in_view and 50 <= dark_pixel_area <= 200
            if not visible:
                pixel_error = None
                pixel_error_y = None
            self.observation_sequence += 1
            self.camera_frames[self.observation_sequence] = camera_frame
            while len(self.camera_frames) > CAMERA_FRAME_HISTORY:
                del self.camera_frames[min(self.camera_frames)]
            gyro = {
                "x": self.plant.pitch_rate_rad_s + 0.001 + self.rng.gauss(0.0, 0.001),
                "y": self.plant.rate_rad_s - 0.001 + self.rng.gauss(0.0, 0.001),
                "z": self.plant.roll_rate_rad_s + 0.001 + self.rng.gauss(0.0, 0.001),
            }
            gravity = 9.80665
            accel = {
                "x": gravity * sr * cp,
                "y": gravity * cr * cp,
                "z": -gravity * sp,
            }
            field = MAGNETIC_FIELD_WORLD_UT
            bx1 = cy * field["x"] - sy * field["z"]
            by1 = field["y"]
            bz1 = sy * field["x"] + cy * field["z"]
            bx2 = bx1
            by2 = cp * by1 + sp * bz1
            bz2 = -sp * by1 + cp * bz1
            magnetic_field = {
                "x": cr * bx2 + sr * by2,
                "y": -sr * bx2 + cr * by2,
                "z": bz2,
            }
            return {
                "protocol_version": PROTOCOL_VERSION,
                "source": "sim_environment",
                "message_type": "environment_observation",
                "sequence": self.observation_sequence,
                "simulation_time_s": self.plant.time_s,
                "sensors": {
                    "camera": {
                        "valid": visible,
                        "pixel_error_x": pixel_error,
                        "pixel_error_y": pixel_error_y,
                        "image_width_px": CAMERA_WIDTH_PX,
                        "image_height_px": CAMERA_HEIGHT_PX,
                        "focal_length_x_px": CAMERA_FOCAL_PX,
                        "target_dark_pixel_area": dark_pixel_area,
                        "age_s": 0.0,
                    },
                    "imu": {
                        "gyro_rad_s": gyro,
                        "accel_m_s2": accel,
                        "age_s": 0.0,
                    },
                    "magnetometer": {
                        "valid": True,
                        "magnetic_field_uT": magnetic_field,
                        "age_s": 0.0,
                    },
                },
                "actuator_feedback": {
                    "wheel_speed_rad_s": self.plant.wheel.speed_rad_s,
                },
                "model_note": "Synthetic sensor data; camera intrinsics and fixed-world magnetic field are placeholders.",
            }

    @staticmethod
    def _render_camera_frame(pixel_error_x, pixel_error_y, apparent_diameter_px):
        """Render a bright QVGA image with one dark target disk, if visible."""
        pixels = bytearray([190] * (CAMERA_WIDTH_PX * CAMERA_HEIGHT_PX))
        if pixel_error_x is None or pixel_error_y is None:
            return bytes(pixels)
        center_x = (CAMERA_WIDTH_PX - 1) / 2 + pixel_error_x
        # Sensor-plane Y is positive up; image rows and detector error are down.
        center_y = (CAMERA_HEIGHT_PX - 1) / 2 - pixel_error_y
        radius = apparent_diameter_px / 2
        # Beyond this bound the disk covers every pixel for any projected
        # center inside the camera FOV. Clamping also handles finite/infinite
        # apparent sizes for a target very close to the camera plane.
        radius = min(radius, 2 * max(CAMERA_WIDTH_PX, CAMERA_HEIGHT_PX))
        radius_squared = radius * radius
        min_x = max(0, math.floor(center_x - radius))
        max_x = min(CAMERA_WIDTH_PX - 1, math.ceil(center_x + radius))
        min_y = max(0, math.floor(center_y - radius))
        max_y = min(CAMERA_HEIGHT_PX - 1, math.ceil(center_y + radius))
        dark_pixel = b"\x0c"
        for y in range(min_y, max_y + 1):
            vertical_squared = (y - center_y) ** 2
            if vertical_squared > radius_squared:
                continue
            horizontal_reach = math.sqrt(radius_squared - vertical_squared)
            start_x = max(min_x, math.ceil(center_x - horizontal_reach))
            end_x = min(max_x, math.floor(center_x + horizontal_reach))
            if start_x <= end_x:
                row = y * CAMERA_WIDTH_PX
                pixels[row + start_x:row + end_x + 1] = dark_pixel * (
                    end_x - start_x + 1
                )
        return bytes(pixels)

    def camera_frame_snapshot(self, sequence):
        """Return the exact raw QVGA frame paired with an observation ID."""
        if isinstance(sequence, bool) or not isinstance(sequence, int) or sequence < 1:
            raise ValueError("camera frame sequence must be a positive integer")
        with self.lock:
            return self.camera_frames.get(sequence)

    def accept_actuator(self, payload):
        if not _supported_protocol_version(payload.get("protocol_version")):
            raise ValueError("unsupported protocol_version")
        if payload.get("message_type") != "actuator_telemetry":
            raise ValueError("message_type must be actuator_telemetry")
        torque = float(payload.get("body_torque_nm"))
        if not math.isfinite(torque) or abs(torque) > 0.1:
            raise ValueError("body_torque_nm must be within ±0.1 N m")
        raw_speed_target = payload.get("motor_speed_target_rev_s")
        if raw_speed_target is None:
            speed_target = None
        else:
            if isinstance(raw_speed_target, bool):
                raise ValueError("motor_speed_target_rev_s must be a finite number in [0, 50]")
            speed_target = float(raw_speed_target)
            if not math.isfinite(speed_target) or not 0.0 <= speed_target <= 50.0:
                raise ValueError("motor_speed_target_rev_s must be a finite number in [0, 50]")
            if torque != 0.0:
                raise ValueError("motor speed target and body torque are mutually exclusive")
        raw_axis_torques = payload.get("virtual_axis_torque_nm", {})
        if not isinstance(raw_axis_torques, dict) or set(raw_axis_torques) - {"pitch", "roll"}:
            raise ValueError("virtual_axis_torque_nm must contain pitch and/or roll")
        axis_torques = {"pitch": 0.0, "roll": 0.0}
        axis_limits = {"pitch": 0.02, "roll": 0.05}
        for axis in axis_torques:
            value = raw_axis_torques.get(axis, 0.0)
            if (isinstance(value, bool) or not isinstance(value, (int, float)) or
                    not math.isfinite(value) or abs(value) > axis_limits[axis]):
                raise ValueError(
                    f"virtual {axis} torque must be finite and within "
                    f"±{axis_limits[axis]:.2f} N m")
            axis_torques[axis] = float(value)
        if raw_axis_torques and payload.get("source") != "cubesat_sil":
            raise ValueError("virtual axis torques are reserved for host SIL")
        if speed_target is not None and any(axis_torques.values()):
            raise ValueError("speed test and virtual axis torque cannot be active together")
        ack = (self._parse_ack(payload["command_ack"])
               if "command_ack" in payload else None)
        with self.lock:
            if ack is not None:
                self._check_ack_id_locked(ack["command_id"])
            self.body_torque_nm = torque
            self.virtual_pitch_torque_nm = axis_torques["pitch"]
            self.virtual_roll_torque_nm = axis_torques["roll"]
            self.motor_speed_target_rev_s = speed_target
            self.last_actuator_time = time.monotonic()
            self.telemetry_sequence += 1
            self.telemetry = dict(payload)
            self.telemetry["ground_sequence"] = self.telemetry_sequence
            self.telemetry["received_time_s"] = self.plant.time_s
            if (payload.get("source") in {"cubesat_sil", "cubesat_esp32"} and
                    payload.get("camera_frame_source") == "simulated"):
                frame_sequence = payload.get("camera_frame_sequence")
                if (isinstance(frame_sequence, int) and
                        not isinstance(frame_sequence, bool)):
                    camera_frame = self.camera_frames.get(frame_sequence)
                    physical_preview_is_fresh = (
                        self.camera_preview_source == "physical" and
                        time.monotonic() - self.camera_preview_time <=
                        CAMERA_PREVIEW_STALE_AFTER_S
                    )
                    if camera_frame is not None and not physical_preview_is_fresh:
                        self.camera_preview_gray = self._downsample_camera_frame(
                            camera_frame
                        )
                        self.camera_preview_time = self.last_actuator_time
                        self.camera_preview_source = "simulated"
                        self._pair_preview_error_locked(payload)
            elif (payload.get("source") == "cubesat_esp32" and
                  payload.get("camera_frame_source") == "physical" and
                  self.camera_preview_source == "simulated"):
                # Do not leave a recent emulated image visible after switching
                # to physical firmware before its first real frame upload.
                self.camera_preview_gray = None
                self.camera_preview_source = None
                self.camera_preview_target_valid = False
                self.camera_preview_error_x_px = None
                self.camera_preview_error_y_px = None
            if ack is not None:
                self.latest_ack = ack

    @staticmethod
    def _downsample_camera_frame(frame):
        """Average 5x5 QVGA blocks into the Ground Station's 64x48 image."""
        if not isinstance(frame, bytes) or len(frame) != CAMERA_WIDTH_PX * CAMERA_HEIGHT_PX:
            raise ValueError("camera frame must contain a raw QVGA grayscale image")
        pixels = bytearray(CAMERA_PREVIEW_BYTES)
        for row in range(CAMERA_PREVIEW_HEIGHT):
            source_row = row * 5 * CAMERA_WIDTH_PX
            for col in range(CAMERA_PREVIEW_WIDTH):
                total = 0
                source_col = col * 5
                for offset_y in range(5):
                    start = source_row + offset_y * CAMERA_WIDTH_PX + source_col
                    total += sum(frame[start:start + 5])
                pixels[row * CAMERA_PREVIEW_WIDTH + col] = round(total / 25)
        return bytes(pixels)

    def _pair_preview_error_locked(self, telemetry):
        error_x = telemetry.get("camera_error_x_px")
        error_y = telemetry.get("camera_error_y_px")
        valid = (
            telemetry.get("target_valid") is True and
            isinstance(error_x, (int, float)) and not isinstance(error_x, bool) and
            math.isfinite(error_x) and
            isinstance(error_y, (int, float)) and not isinstance(error_y, bool) and
            math.isfinite(error_y)
        )
        self.camera_preview_target_valid = valid
        self.camera_preview_error_x_px = float(error_x) if valid else None
        self.camera_preview_error_y_px = float(error_y) if valid else None

    def telemetry_snapshot(self, now=None):
        with self.lock:
            now = time.monotonic() if now is None else now
            if (self.telemetry is None or
                    now - self.last_actuator_time > TELEMETRY_STALE_AFTER_S):
                return None
            snapshot = dict(self.telemetry)
            snapshot["simulated_wheel_speed_rad_s"] = self.plant.wheel.speed_rad_s
            snapshot["simulated_wheel_motor_torque_nm"] = self.plant.wheel.motor_torque_nm
            snapshot["simulated_body_torque_nm"] = self.plant.actual_body_torque_nm
            snapshot["simulated_wheel_saturated"] = self.plant.wheel.saturated
            snapshot["simulated_motor_speed_target_rev_s"] = self.motor_speed_target_rev_s
            snapshot["simulated_pitch_body_torque_nm"] = self.virtual_pitch_torque_nm
            snapshot["simulated_roll_body_torque_nm"] = self.virtual_roll_torque_nm
            return snapshot

    def accept_camera_preview(self, frame):
        if not isinstance(frame, bytes) or len(frame) != CAMERA_PREVIEW_BYTES:
            raise ValueError(f"camera preview must be {CAMERA_PREVIEW_BYTES} grayscale bytes")
        with self.lock:
            self.camera_preview_gray = frame
            self.camera_preview_time = time.monotonic()
            self.camera_preview_source = "physical"
            telemetry = self.telemetry or {}
            self._pair_preview_error_locked(telemetry)

    def camera_preview_snapshot(self, now=None):
        with self.lock:
            now = time.monotonic() if now is None else now
            preview_fresh = (
                self.camera_preview_gray is not None and
                now - self.camera_preview_time <= CAMERA_PREVIEW_STALE_AFTER_S
            )
            if preview_fresh:
                pixels = self.camera_preview_gray
                target_valid = self.camera_preview_target_valid
                error_x_px = self.camera_preview_error_x_px
                error_y_px = self.camera_preview_error_y_px
                source = self.camera_preview_source or "physical"
            else:
                telemetry_fresh = (
                    self.telemetry is not None and
                    now - self.last_actuator_time <= TELEMETRY_STALE_AFTER_S
                )
                # Render a synthetic image only for host SIL telemetry. This
                # keeps missing physical-camera frames visible as unavailable.
                if not telemetry_fresh or self.telemetry.get("source") != "cubesat_sil":
                    return None
                telemetry = self.telemetry
                target_valid = telemetry.get("target_valid") is True
                raw_x = telemetry.get("camera_error_x_px")
                raw_y = telemetry.get("camera_error_y_px")
                target_valid = (
                    target_valid and
                    isinstance(raw_x, (int, float)) and not isinstance(raw_x, bool) and
                    math.isfinite(raw_x) and
                    isinstance(raw_y, (int, float)) and not isinstance(raw_y, bool) and
                    math.isfinite(raw_y)
                )
                error_x_px = float(raw_x) if target_valid else None
                error_y_px = float(raw_y) if target_valid else None
                pixels = bytearray([190] * CAMERA_PREVIEW_BYTES)
                if target_valid:
                    center_x = (CAMERA_PREVIEW_WIDTH - 1) / 2 + error_x_px / 5.0
                    center_y = (CAMERA_PREVIEW_HEIGHT - 1) / 2 + error_y_px / 5.0
                    # A small dark disk represents the black point at 5x scale.
                    for row in range(CAMERA_PREVIEW_HEIGHT):
                        for col in range(CAMERA_PREVIEW_WIDTH):
                            if (col - center_x) ** 2 + (row - center_y) ** 2 <= 2.25:
                                pixels[row * CAMERA_PREVIEW_WIDTH + col] = 12
                pixels = bytes(pixels)
                source = "simulated"
        palette = b"".join(bytes((value, value, value, 0)) for value in range(256))
        bottom_up = b"".join(
            pixels[row * CAMERA_PREVIEW_WIDTH:(row + 1) * CAMERA_PREVIEW_WIDTH]
            for row in range(CAMERA_PREVIEW_HEIGHT - 1, -1, -1)
        )
        pixel_offset = 14 + 40 + len(palette)
        file_header = struct.pack("<2sIHHI", b"BM", pixel_offset + len(bottom_up),
                                  0, 0, pixel_offset)
        dib_header = struct.pack("<IiiHHIIiiII", 40, CAMERA_PREVIEW_WIDTH,
                                 CAMERA_PREVIEW_HEIGHT, 1, 8, 0,
                                 len(bottom_up), 2835, 2835, 256, 256)
        return (file_header + dib_header + palette + bottom_up,
                target_valid, error_x_px, error_y_px, source)

    def camera_preview_bmp(self, now=None):
        snapshot = self.camera_preview_snapshot(now)
        return None if snapshot is None else snapshot[0]

    def queue_command(self, packet):
        if not _supported_protocol_version(packet.get("protocol_version")):
            raise ValueError("unsupported protocol_version")
        if packet.get("message_type") != "telecommand":
            raise ValueError("message_type must be telecommand")
        command_id = packet.get("command_id")
        if (isinstance(command_id, bool) or not isinstance(command_id, int) or
                not 1 <= command_id <= MAX_SAFE_COMMAND_ID):
            raise ValueError("command_id must be an integer from 1 to 2^53-1")
        issued = float(packet.get("issued_unix_s", time.time()))
        if not math.isfinite(issued) or issued > time.time() + 1.0 or time.time() - issued > 10.0:
            raise ValueError("telecommand is expired or has an invalid issue time")
        command = packet.get("command")
        arguments = packet.get("arguments", {})
        if command == "PING":
            arguments = {}
        elif command == "SET_MODE":
            mode = arguments.get("mode") if isinstance(arguments, dict) else None
            if mode not in {"SAFE", "STANDBY", "RATE_DAMP", "ACQUIRE", "TRACK"}:
                raise ValueError("SET_MODE mode is not supported")
            arguments = {"mode": mode}
        elif command == "SET_WHEEL_SPEED":
            if not isinstance(arguments, dict):
                raise ValueError("SET_WHEEL_SPEED arguments must be an object")
            if set(arguments) != {"target_rev_s", "duration_s"}:
                raise ValueError("SET_WHEEL_SPEED requires only target_rev_s and duration_s")
            target = arguments.get("target_rev_s")
            duration = arguments.get("duration_s")
            if (isinstance(target, bool) or not isinstance(target, (int, float)) or
                    not math.isfinite(target) or not 0.0 <= target <= 50.0):
                raise ValueError("target_rev_s must be a finite number in [0, 50]")
            if (isinstance(duration, bool) or not isinstance(duration, (int, float)) or
                    not math.isfinite(duration) or not 0.5 <= duration <= 5.0):
                raise ValueError("duration_s must be a finite number in [0.5, 5]")
            arguments = {"target_rev_s": float(target),
                         "duration_s": float(duration)}
        elif command == "SET_YAW_STEP":
            if not isinstance(arguments, dict) or set(arguments) != {"delta_deg"}:
                raise ValueError("SET_YAW_STEP requires only delta_deg")
            delta = arguments.get("delta_deg")
            if (isinstance(delta, bool) or not isinstance(delta, (int, float)) or
                    not math.isfinite(delta) or not -180.0 <= delta <= 180.0):
                raise ValueError("delta_deg must be a finite number in [-180, 180]")
            arguments = {"delta_deg": float(delta)}
        else:
            raise ValueError("supported commands: PING, SET_MODE, SET_WHEEL_SPEED, SET_YAW_STEP")
        with self.lock:
            if command_id <= self.last_command_id:
                queued = next((item for item in self.commands
                               if item["command_id"] == command_id), None)
                if queued is not None:
                    if time.time() - queued["issued_unix_s"] > 10.0:
                        raise ValueError("duplicate command_id has expired")
                    if (queued["command"] == command and
                            queued["arguments"] == arguments):
                        return command_id  # Identical retry while still live.
                    raise ValueError("duplicate command_id has different content")
                raise ValueError("command_id must be greater than the last accepted ID")
            self.last_command_id = command_id
            self.commands.append({"command_id": command_id,
                                  "command": command,
                                  "arguments": arguments,
                                  "issued_unix_s": issued})
            self.commands = self.commands[-50:]
            return command_id

    def commands_after(self, command_id):
        with self.lock:
            pending = [item for item in self.commands
                       if item["command_id"] > command_id
                       and time.time() - item["issued_unix_s"] <= 10.0]
            return [dict(pending[0])] if pending else []

    def cycle_packet(self, command_id):
        return {
            "protocol_version": PROTOCOL_VERSION,
            "message_type": "simulation_cycle",
            "observation": self.observation(),
            "commands": self.commands_after(command_id),
        }

    @staticmethod
    def _parse_ack(ack):
        if not isinstance(ack, dict):
            raise ValueError("command_ack must be an object")
        command_id = ack.get("command_id")
        if (isinstance(command_id, bool) or not isinstance(command_id, int) or
                not 1 <= command_id <= MAX_SAFE_COMMAND_ID):
            raise ValueError("command_ack command_id must be an integer from 1 to 2^53-1")
        status = ack.get("status")
        if status not in {"ACKNOWLEDGED", "REJECTED"}:
            raise ValueError("command_ack status must be ACKNOWLEDGED or REJECTED")
        reason = ack.get("reason", "")
        if not isinstance(reason, str) or len(reason) > 120:
            raise ValueError("command_ack reason must be text of at most 120 characters")
        if status == "REJECTED" and not reason:
            raise ValueError("rejected command_ack requires a reason")
        return {"command_id": command_id, "status": status, "reason": reason}

    def _check_ack_id_locked(self, command_id):
        if not any(item["command_id"] == command_id for item in self.commands):
            raise ValueError("command_ack ID does not match a command issued by this bridge")

    def acknowledge(self, ack):
        parsed = self._parse_ack(ack)
        with self.lock:
            self._check_ack_id_locked(parsed["command_id"])
            self.latest_ack = parsed

    def snapshot(self):
        with self.lock:
            torque_active = self.plant.time_s < self.torque_until
            bearing = math.atan2(self.target_xyz_m["x"], self.target_xyz_m["z"])
            cy, sy = math.cos(self.plant.yaw_rad), math.sin(self.plant.yaw_rad)
            cp, sp = math.cos(self.plant.pitch_rad), math.sin(self.plant.pitch_rad)
            cr, sr = math.cos(self.plant.roll_rad), math.sin(self.plant.roll_rad)
            target = self.target_xyz_m
            x1 = cy * target["x"] - sy * target["z"]
            y1 = target["y"]
            z1 = sy * target["x"] + cy * target["z"]
            y2 = cp * y1 + sp * z1
            z2 = -sp * y1 + cp * z1
            camera_x = cr * x1 + sr * y2
            camera_y = -sr * x1 + cr * y2
            return {
                "time_s": self.plant.time_s,
                "yaw_rad": self.plant.yaw_rad,
                "yaw_rate_rad_s": self.plant.rate_rad_s,
                "roll_rad": self.plant.roll_rad,
                "pitch_rad": self.plant.pitch_rad,
                "roll_rate_rad_s": self.plant.roll_rate_rad_s,
                "pitch_rate_rad_s": self.plant.pitch_rate_rad_s,
                "target_xyz_m": dict(self.target_xyz_m),
                "target_scale_m": self.target_scale,
                "target_bearing_rad": bearing,
                "target_error_rad": target_bearing_error(
                    bearing, self.plant.yaw_rad
                ),
                "camera_truth_error_x_rad": math.atan2(camera_x, z2),
                "camera_truth_error_y_rad": -math.atan2(camera_y, z2),
                "external_torque_nm": dict(self.external_torque_nm),
                "torque_active": torque_active,
                "body_torque_nm": self.body_torque_nm,
                "simulated_pitch_body_torque_nm": self.virtual_pitch_torque_nm,
                "simulated_roll_body_torque_nm": self.virtual_roll_torque_nm,
                "motor_speed_target_rev_s": self.motor_speed_target_rev_s,
                "actual_body_torque_nm": self.plant.actual_body_torque_nm,
                "wheel_speed_rad_s": self.plant.wheel.speed_rad_s,
                "wheel_motor_torque_nm": self.plant.wheel.motor_torque_nm,
                "wheel_saturated": self.plant.wheel.saturated,
                "attitude_rad": {"roll": self.plant.roll_rad,
                                 "pitch": self.plant.pitch_rad,
                                 "yaw": self.plant.yaw_rad},
                "actuator_fresh": time.monotonic() - self.last_actuator_time <= 0.1,
                "model_limits": ["euler_rate_coupling", "gimbal_cross_axis_inertia",
                                 "large_angle_pendulum_dynamics"],
            }

    def run(self, stop_event):
        previous = time.monotonic()
        while not stop_event.is_set():
            now = time.monotonic()
            dt = min(DT_S, max(0.0001, now - previous))
            previous = now
            self.advance(dt, now)
            stop_event.wait(max(0.0, DT_S - (time.monotonic() - now)))

    def advance(self, dt_s=DT_S, now=None):
        if not math.isfinite(dt_s) or dt_s <= 0.0 or dt_s > 0.1:
            raise ValueError("dt_s must be finite and in (0, 0.1]")
        now = time.monotonic() if now is None else now
        with self.lock:
            sim_start = self.plant.time_s
            pulse_duration = max(
                0.0, min(sim_start + dt_s, self.torque_until) - sim_start
            )
            pulse_fraction = pulse_duration / dt_s
            ext_torque = {
                axis: value * pulse_fraction
                for axis, value in self.external_torque_nm.items()
            }
            if now - self.last_actuator_time > 0.1:
                self.body_torque_nm = 0.0
                self.virtual_pitch_torque_nm = 0.0
                self.virtual_roll_torque_nm = 0.0
                self.motor_speed_target_rev_s = None
            self.plant.step(ext_torque, self.body_torque_nm, dt_s,
                            motor_speed_target_rev_s=self.motor_speed_target_rev_s,
                            body_pitch_torque_nm=self.virtual_pitch_torque_nm,
                            body_roll_torque_nm=self.virtual_roll_torque_nm)
            return self.snapshot()


environment = Environment()


class Handler(BaseHTTPRequestHandler):
    def _send(self, status, body, content_type="application/json", headers=None):
        data = body if isinstance(body, bytes) else body.encode("utf-8")
        self.send_response(status)
        self.send_header("Content-Type", content_type)
        self.send_header("Content-Length", str(len(data)))
        self.send_header("Cache-Control", "no-store")
        for name, value in (headers or {}).items():
            self.send_header(name, str(value))
        self.end_headers()
        self.wfile.write(data)

    def do_GET(self):
        if self.path == "/":
            self._send(200, UI_PATH.read_bytes(), "text/html; charset=utf-8")
        elif self.path == "/api/state":
            self._send(200, json.dumps(environment.snapshot()))
        elif self.path == "/api/observation":
            self._send(200, json.dumps(environment.observation()))
        elif urlparse(self.path).path in {
                "/api/camera-frame.raw", "/api/camera-frame.rle"}:
            parsed_path = urlparse(self.path)
            query = parse_qs(parsed_path.query)
            try:
                sequence = int(query.get("sequence", [""])[0])
                frame = environment.camera_frame_snapshot(sequence)
                if frame is None:
                    self._send(404, json.dumps({"error": "camera frame is unavailable or expired"}))
                else:
                    is_rle = parsed_path.path.endswith(".rle")
                    payload = encode_rle_frame(frame) if is_rle else frame
                    if is_rle and len(payload) > MAX_ENCODED_BYTES:
                        self._send(413, json.dumps({
                            "error": "encoded camera frame exceeds RLE1 transfer bound"
                        }))
                        return
                    headers = {
                        "X-Camera-Frame-Sequence": sequence,
                        "X-Camera-Frame-Width": CAMERA_WIDTH_PX,
                        "X-Camera-Frame-Height": CAMERA_HEIGHT_PX,
                    }
                    if is_rle:
                        headers["X-Camera-Frame-Encoding"] = "RLE1"
                    self._send(200, payload, "application/octet-stream", headers)
            except ValueError as exc:
                self._send(400, json.dumps({"error": str(exc)}))
        elif self.path.startswith("/api/cycle"):
            query = parse_qs(urlparse(self.path).query)
            try:
                after = int(query.get("after", ["0"])[0])
                if after < 0:
                    raise ValueError("after cannot be negative")
                self._send(200, json.dumps(environment.cycle_packet(after)))
            except ValueError:
                self._send(400, json.dumps({"error": "after must be an integer"}))
        elif self.path == "/api/telemetry":
            telemetry = environment.telemetry_snapshot()
            if telemetry is None:
                self._send(503, json.dumps({"error": "no onboard telemetry yet"}))
            else:
                self._send(200, json.dumps(telemetry))
        elif self.path == "/api/camera-preview.bmp":
            snapshot = environment.camera_preview_snapshot()
            if snapshot is None:
                self._send(503, json.dumps({"error": "no recent camera preview"}))
            else:
                preview, target_valid, error_x_px, error_y_px, source = snapshot
                preview_headers = {"X-Camera-Target-Valid": str(target_valid).lower()}
                preview_headers["X-Camera-Preview-Source"] = source
                if error_x_px is not None:
                    preview_headers["X-Camera-Error-X-Px"] = error_x_px
                    preview_headers["X-Camera-Error-Y-Px"] = error_y_px
                self._send(200, preview, "image/bmp", preview_headers)
        elif self.path.startswith("/api/commands"):
            query = parse_qs(urlparse(self.path).query)
            try:
                after = int(query.get("after", ["0"])[0])
                if after < 0:
                    raise ValueError("after cannot be negative")
                self._send(200, json.dumps({"commands": environment.commands_after(after)}))
            except ValueError:
                self._send(400, json.dumps({"error": "after must be an integer"}))
        elif self.path == "/api/ack":
            with environment.lock:
                self._send(200, json.dumps({"latest_ack": environment.latest_ack}))
        else:
            self._send(404, json.dumps({"error": "not found"}))

    def do_POST(self):
        if self.path == "/api/camera-preview":
            try:
                size = int(self.headers.get("Content-Length", "0"))
                if size != CAMERA_PREVIEW_BYTES:
                    raise ValueError(f"request body must be exactly {CAMERA_PREVIEW_BYTES} bytes")
                environment.accept_camera_preview(self.rfile.read(size))
            except (ValueError, TypeError) as exc:
                self._send(400, json.dumps({"error": str(exc)}))
                return
            self._send(204, b"")
            return
        if self.path not in ("/api/control", "/api/actuator", "/api/command", "/api/ack"):
            self._send(404, json.dumps({"error": "not found"}))
            return
        try:
            size = int(self.headers.get("Content-Length", "0"))
            if not 0 < size <= 4096:
                raise ValueError("request body must be 1..4096 bytes")
            payload = json.loads(self.rfile.read(size))
            if not isinstance(payload, dict):
                raise ValueError("request body must be a JSON object")
            if self.path == "/api/control":
                environment.apply(payload)
            elif self.path == "/api/actuator":
                environment.accept_actuator(payload)
            elif self.path == "/api/command":
                command_id = environment.queue_command(payload)
                self._send(202, json.dumps({"accepted": True,
                                            "command_id": command_id}))
                return
            else:
                environment.acknowledge(payload)
        except (ValueError, TypeError, KeyError, json.JSONDecodeError) as exc:
            self._send(400, json.dumps({"error": str(exc)}))
            return
        self._send(200, json.dumps(environment.snapshot()))

    def log_message(self, _format, *_args):
        pass


def main():
    stop_event = threading.Event()
    model_thread = threading.Thread(
        target=environment.run, args=(stop_event,), daemon=True
    )
    model_thread.start()
    server = ThreadingHTTPServer((HOST, PORT), Handler)
    print(f"Simulation environment: http://{HOST}:{PORT}")
    print("Three-axis gimbal plant: yaw wheel plus host-SIL virtual pitch/roll control.")
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        print("\nSimulation environment stopped.")
    finally:
        server.server_close()
        stop_event.set()
        model_thread.join(timeout=1.0)


if __name__ == "__main__":
    main()
