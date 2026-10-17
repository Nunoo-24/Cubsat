"""HTTP software-in-the-loop runner for the CubeSat onboard control logic.

This is a host-side reference runner, not ESP32 firmware. It uses the same
sensor-to-control flow and simulation bridge intended for an ESP-IDF adapter.
"""

import json
import math
import os
import time
from urllib.error import URLError, HTTPError
from urllib.request import Request, urlopen

from Sim_Environment.camera_transport import (MAX_ENCODED_BYTES,
                                              decode_rle_frame)

from .attitude import YawAttitudeEstimator, wrap_pi
from .camera import detect_black_target
from .control import PitchRollController, YawController


BASE_URL = os.environ.get("CUBESAT_SIM_URL", "http://127.0.0.1:8000").rstrip("/")
PERIOD_S = 0.02
CAMERA_WIDTH_PX = 320
CAMERA_HEIGHT_PX = 240
CAMERA_FRAME_BYTES = CAMERA_WIDTH_PX * CAMERA_HEIGHT_PX
CAMERA_RLE_MAX_BYTES = MAX_ENCODED_BYTES


def _next_cycle_deadline(previous_deadline, now):
    deadline = previous_deadline + PERIOD_S
    return now if deadline <= now else deadline


def _sample_age(sensor):
    try:
        value = float(sensor.get("age_s", math.inf))
    except (TypeError, ValueError):
        return math.inf, False
    return value, math.isfinite(value) and value >= 0.0


class OnboardSoftware:
    def __init__(self, controller=None, pitch_roll_controller=None):
        self.controller = controller or YawController()
        self.pitch_roll_controller = pitch_roll_controller or PitchRollController()
        self.sequence = 0
        self.mode = "TRACK"
        self.last_command_id = 0
        self.pending_ack = None
        self.wheel_speed_target_rev_s = None
        self.wheel_speed_test_deadline = None
        self.wheel_speed_stop_deadline = None
        self.wheel_speed_test_started = None
        self.wheel_speed_test_status = "IDLE"
        self.attitude_estimator = YawAttitudeEstimator()
        self.last_attitude_sim_time_s = None
        self.last_attitude_valid = False
        self.yaw_step_target_rad = None
        self.yaw_step_started = None
        self.yaw_step_deadline = None
        self.yaw_step_settle_since = None
        self.yaw_step_status = "IDLE"

    def receive_command(self, command, now=None):
        now = time.monotonic() if now is None else now
        raw_command_id = command.get("command_id")
        if (isinstance(raw_command_id, bool) or
                not isinstance(raw_command_id, int) or
                not 1 <= raw_command_id <= (1 << 53) - 1):
            raise ValueError("command_id must be an integer from 1 to 2^53-1")
        command_id = raw_command_id
        if command_id <= self.last_command_id:
            return
        kind = command.get("command")
        arguments = command.get("arguments", {})
        if not isinstance(arguments, dict):
            arguments = None
        status, reason = "ACKNOWLEDGED", ""
        if kind == "PING":
            pass
        elif kind == "SET_MODE" and isinstance(arguments, dict) and arguments.get("mode") in {
            "SAFE", "STANDBY", "RATE_DAMP", "ACQUIRE", "TRACK"
        }:
            requested_mode = arguments["mode"]
            if self.mode in ("WHEEL_SPEED_TEST", "WHEEL_SPEED_STOPPING"):
                if requested_mode == "SAFE":
                    self.mode = "WHEEL_SPEED_STOPPING"
                    self.wheel_speed_stop_deadline = now + 10.0
                else:
                    status, reason = "REJECTED", "SPEED_TEST_ACTIVE"
            elif self.mode == "YAW_STEP":
                if requested_mode in ("SAFE", "STANDBY"):
                    self.mode = requested_mode
                    self.yaw_step_status = "ABORTED"
                    self.yaw_step_target_rad = None
                    self.yaw_step_deadline = None
                    self.yaw_step_settle_since = None
                else:
                    status, reason = "REJECTED", "YAW_STEP_ACTIVE"
            else:
                self.mode = requested_mode
        elif kind == "SET_MODE":
            status, reason = "REJECTED", "INVALID_MODE"
        elif kind == "SET_YAW_STEP":
            delta_deg = arguments.get("delta_deg") if isinstance(arguments, dict) else None
            valid_arguments = (
                isinstance(arguments, dict) and set(arguments) == {"delta_deg"} and
                not isinstance(delta_deg, bool) and
                isinstance(delta_deg, (int, float)) and
                math.isfinite(delta_deg) and -180.0 <= delta_deg <= 180.0
            )
            if not valid_arguments:
                status, reason = "REJECTED", "INVALID_YAW_STEP_ARGUMENTS"
            elif self.mode not in ("SAFE", "STANDBY"):
                status, reason = "REJECTED", "YAW_STEP_REQUIRES_SAFE_OR_STANDBY"
            elif not self.last_attitude_valid:
                status, reason = "REJECTED", "ATTITUDE_ESTIMATE_UNAVAILABLE"
            else:
                self.yaw_step_target_rad = wrap_pi(
                    self.attitude_estimator.yaw_rad + math.radians(delta_deg))
                self.yaw_step_started = now
                self.yaw_step_deadline = now + 10.0
                self.yaw_step_settle_since = None
                self.yaw_step_status = "RUNNING"
                self.mode = "YAW_STEP"
        elif kind == "SET_WHEEL_SPEED":
            target = arguments.get("target_rev_s") if isinstance(arguments, dict) else None
            duration = arguments.get("duration_s") if isinstance(arguments, dict) else None
            valid = (
                isinstance(arguments, dict) and
                set(arguments) == {"target_rev_s", "duration_s"} and
                not isinstance(target, bool) and isinstance(target, (int, float))
                and math.isfinite(target) and 0.0 <= target <= 50.0
                and not isinstance(duration, bool)
                and isinstance(duration, (int, float))
                and math.isfinite(duration) and 0.5 <= duration <= 5.0
            )
            if not valid:
                status, reason = "REJECTED", "INVALID_SPEED_TEST_ARGUMENTS"
            elif self.mode not in ("SAFE", "STANDBY"):
                status, reason = "REJECTED", "SPEED_TEST_REQUIRES_SAFE_OR_STANDBY"
            else:
                self.mode = "WHEEL_SPEED_TEST"
                self.wheel_speed_target_rev_s = float(target)
                self.wheel_speed_test_started = now
                self.wheel_speed_test_deadline = now + float(duration)
                self.wheel_speed_stop_deadline = None
                self.wheel_speed_test_status = "RUNNING"
        else:
            status, reason = "REJECTED", "UNSUPPORTED_COMMAND"
        self.last_command_id = command_id
        self.pending_ack = {"command_id": command_id,
                            "status": status,
                            "reason": reason}

    def process_observation(self, packet, now=None, camera_frame=None,
                            camera_age_s=None, camera_sample_started=None):
        now = time.monotonic() if now is None else now
        protocol_version = packet.get("protocol_version")
        if (isinstance(protocol_version, bool) or
                not isinstance(protocol_version, (int, float)) or
                protocol_version != 1):
            raise ValueError("unsupported environment protocol version")
        if packet.get("message_type") != "environment_observation":
            raise ValueError("unexpected environment message type")
        sensors = packet.get("sensors", {})
        camera = dict(sensors.get("camera", {}))
        if camera_age_s is not None:
            camera["age_s"] = camera_age_s
        if camera_frame is not None:
            detection = detect_black_target(
                camera_frame, CAMERA_WIDTH_PX, CAMERA_HEIGHT_PX
            )
            camera["valid"] = detection["valid"]
            camera["pixel_error_x"] = (
                detection["error_x_px"] if detection["valid"] else None
            )
            # Environment image coordinates use positive Y up; detector rows
            # and telemetry use positive Y down.
            camera["pixel_error_y"] = (
                -detection["error_y_px"] if detection["valid"] else None
            )
            camera["target_dark_pixel_area"] = detection["dark_pixel_count"]
            camera["image_width_px"] = CAMERA_WIDTH_PX
            camera["image_height_px"] = CAMERA_HEIGHT_PX
            if camera_sample_started is not None:
                try:
                    if isinstance(camera_sample_started, bool):
                        raise ValueError("sample start cannot be boolean")
                    sample_started = float(camera_sample_started)
                    camera["age_s"] = time.monotonic() - sample_started
                except (TypeError, ValueError):
                    camera["age_s"] = math.nan
        imu = sensors.get("imu", {})
        magnetometer = sensors.get("magnetometer", {})
        magnetic_field = magnetometer.get("magnetic_field_uT", {})
        magnetic_values = {
            axis: float(magnetic_field.get(axis, math.nan))
            for axis in ("x", "y", "z")
        }
        magnetic_age = float(magnetometer.get("age_s", math.inf))
        magnetometer_valid = bool(magnetometer.get("valid", False)) and all(
            math.isfinite(value) for value in magnetic_values.values()
        ) and math.isfinite(magnetic_age) and 0.0 <= magnetic_age <= 0.1
        camera_age, camera_age_valid = _sample_age(camera)
        imu_age, imu_age_valid = _sample_age(imu)
        gyro = imu.get("gyro_rad_s", {})
        rate = float(gyro.get("y", math.nan))
        accel = imu.get("accel_m_s2", {})
        accel_values = {
            axis: float(accel.get(axis, math.nan)) for axis in ("x", "y", "z")
        }
        sim_time = packet.get("simulation_time_s", 0.0)
        try:
            sim_time = float(sim_time)
        except (TypeError, ValueError):
            sim_time = math.nan
        attitude_dt_s = PERIOD_S
        if (math.isfinite(sim_time) and self.last_attitude_sim_time_s is not None
                and sim_time > self.last_attitude_sim_time_s):
            attitude_dt_s = sim_time - self.last_attitude_sim_time_s
        if math.isfinite(sim_time):
            self.last_attitude_sim_time_s = sim_time
        imu_sample_valid = (imu_age_valid and imu_age <= 0.1 and
                            all(math.isfinite(value) for value in
                                (rate, *accel_values.values())))
        accel_norm = math.sqrt(sum(value * value for value in accel_values.values()))
        tilt_valid = (imu_sample_valid and math.isfinite(accel_norm) and
                      0.8 * 9.80665 <= accel_norm <= 1.2 * 9.80665)
        pitch_estimate_rad = (
            math.atan2(-accel_values["z"],
                       math.hypot(accel_values["x"], accel_values["y"]))
            if tilt_valid else None
        )
        roll_estimate_rad = (
            math.atan2(accel_values["x"], accel_values["y"])
            if tilt_valid else None
        )
        if imu_sample_valid:
            yaw_estimate_rad, attitude_valid, attitude_source = (
                self.attitude_estimator.update(
                    rate,
                    tuple(accel_values[axis] for axis in ("x", "y", "z")),
                    tuple(magnetic_values[axis] for axis in ("x", "y", "z")),
                    attitude_dt_s,
                    magnetic_valid=(magnetometer_valid and imu_age_valid and
                                    imu_age <= 0.1),
                )
            )
        else:
            yaw_estimate_rad, attitude_valid, attitude_source = (
                self.attitude_estimator.yaw_rad, False, "INVALID")
        self.last_attitude_valid = attitude_valid
        target_valid = bool(camera.get("valid", False)) and camera_age_valid
        error_rad = None
        camera_error_y_rad = None
        if target_valid:
            pixel_error = float(camera.get("pixel_error_x", math.nan))
            focal_px = float(camera.get("focal_length_x_px", math.nan))
            if not math.isfinite(pixel_error) or not math.isfinite(focal_px) or focal_px <= 0:
                target_valid = False
            else:
                # Pinhole bearing from image-plane displacement, with zero at
                # the optical centre. Focal length is provisional simulation data.
                error_rad = math.atan(pixel_error / focal_px)
                pixel_error_y = float(camera.get("pixel_error_y", math.nan))
                if not math.isfinite(pixel_error_y):
                    target_valid = False
                    error_rad = None
                else:
                    # Bridge pixel_error_y is positive up. Convert to the
                    # camera/telemetry positive-down convention; positive
                    # pitch moves a fixed target upward in the image.
                    camera_error_y_rad = -math.atan(pixel_error_y / focal_px)

        # Both sensor streams contribute to the onboard observation and must
        # have valid ages. max() alone would hide a negative age in one stream.
        observation_age = (max(camera_age, imu_age)
                           if camera_age_valid and imu_age_valid else math.inf)

        feedback = packet.get("actuator_feedback", {})
        wheel_speed_rad_s = feedback.get("wheel_speed_rad_s", math.nan)
        try:
            wheel_speed_rad_s = float(wheel_speed_rad_s)
        except (TypeError, ValueError):
            wheel_speed_rad_s = math.nan
        measured_motor_speed_rev_s = (
            wheel_speed_rad_s * 60.0 / (2.0 * math.pi)
            if math.isfinite(wheel_speed_rad_s) else None
        )
        motor_speed_target_rev_s = None
        motor_speed_test_status = self.wheel_speed_test_status
        motor_speed_error_percent = None
        motor_speed_test_elapsed_s = None
        if (self.mode == "WHEEL_SPEED_TEST" and
                self.wheel_speed_test_deadline is not None and
                now >= self.wheel_speed_test_deadline):
            self.mode = "WHEEL_SPEED_STOPPING"
            self.wheel_speed_stop_deadline = now + 10.0
            self.wheel_speed_test_status = "STOPPING"

        if self.mode in ("WHEEL_SPEED_TEST", "WHEEL_SPEED_STOPPING"):
            if self.mode == "WHEEL_SPEED_TEST":
                motor_speed_target_rev_s = self.wheel_speed_target_rev_s
                motor_speed_test_status = "RUNNING"
                self.wheel_speed_test_status = motor_speed_test_status
                if self.wheel_speed_target_rev_s:
                    motor_speed_error_percent = 100.0 * abs(
                        self.wheel_speed_target_rev_s -
                        (measured_motor_speed_rev_s or 0.0)
                    ) / self.wheel_speed_target_rev_s
                if self.wheel_speed_test_started is not None:
                    motor_speed_test_elapsed_s = max(0.0, now - self.wheel_speed_test_started)
            else:
                motor_speed_target_rev_s = 0.0
                motor_speed_test_status = "STOPPING"
                self.wheel_speed_test_status = motor_speed_test_status
                stopped = (measured_motor_speed_rev_s is not None and
                           abs(measured_motor_speed_rev_s) <= 0.5)
                timed_out = (self.wheel_speed_stop_deadline is not None and
                             now >= self.wheel_speed_stop_deadline)
                if stopped or timed_out:
                    self.mode = "SAFE"
                    motor_speed_test_status = "STOPPED" if stopped else "STOP_TIMEOUT"
                    self.wheel_speed_test_status = motor_speed_test_status
                    self.wheel_speed_target_rev_s = None
                    self.wheel_speed_test_deadline = None
                    self.wheel_speed_stop_deadline = None
                    self.wheel_speed_test_started = None
        yaw_step_error_rad = None
        yaw_step_elapsed_s = None
        if self.mode == "YAW_STEP":
            yaw_step_elapsed_s = max(0.0, now - self.yaw_step_started)
            if now >= self.yaw_step_deadline:
                self.mode = "SAFE"
                self.yaw_step_status = "TIMEOUT"
                self.yaw_step_target_rad = None
                self.yaw_step_settle_since = None
            elif attitude_valid:
                yaw_step_error_rad = wrap_pi(
                    self.yaw_step_target_rad - yaw_estimate_rad)
                settled = (abs(yaw_step_error_rad) <= math.radians(1.0) and
                           abs(rate) <= math.radians(0.5))
                if settled:
                    if self.yaw_step_settle_since is None:
                        self.yaw_step_settle_since = now
                    self.yaw_step_status = "SETTLING"
                    if now - self.yaw_step_settle_since >= 1.0:
                        self.mode = "STANDBY"
                        self.yaw_step_status = "COMPLETE"
                else:
                    self.yaw_step_settle_since = None
                    self.yaw_step_status = "RUNNING"
            else:
                self.mode = "SAFE"
                self.yaw_step_status = "ATTITUDE_INVALID"
                self.yaw_step_target_rad = None
                self.yaw_step_settle_since = None
        if self.mode in ("SAFE", "STANDBY", "WHEEL_SPEED_TEST", "WHEEL_SPEED_STOPPING"):
            output_torque = 0.0
            mode = "SAFE" if self.mode == "SAFE" else self.mode
            fault = ""
        elif self.mode == "YAW_STEP":
            if attitude_valid and yaw_step_error_rad is not None:
                output = self.controller.update(
                    yaw_step_error_rad, rate, target_valid=True,
                    observation_age_s=imu_age)
                output_torque = output.body_torque_nm
                mode = "FAULT" if output.mode == "FAULT" else "YAW_STEP"
                fault = output.fault
            else:
                output_torque = 0.0
                mode = "YAW_STEP"
                fault = "ATTITUDE_ESTIMATE_UNAVAILABLE"
        else:
            output = self.controller.update(
                error_rad if target_valid and self.mode in ("TRACK", "ACQUIRE") else None,
                rate,
                target_valid=target_valid and self.mode in ("TRACK", "ACQUIRE"),
                observation_age_s=observation_age,
            )
            output_torque, mode, fault = output.body_torque_nm, output.mode, output.fault
            if self.mode == "RATE_DAMP" and output.mode != "FAULT":
                mode = "RATE_DAMP"
            elif self.mode == "ACQUIRE" and output.mode == "TRACK":
                mode = "ACQUIRE"

        pitch_body_torque_nm = 0.0
        roll_body_torque_nm = 0.0
        if mode not in ("SAFE", "STANDBY", "WHEEL_SPEED_TEST",
                        "WHEEL_SPEED_STOPPING", "FAULT") and not fault:
            tilt_output = self.pitch_roll_controller.update(
                camera_error_y_rad,
                pitch_estimate_rad if pitch_estimate_rad is not None else math.nan,
                roll_estimate_rad if roll_estimate_rad is not None else math.nan,
                float(gyro.get("x", math.nan)),
                float(gyro.get("z", math.nan)),
                target_valid=target_valid,
                observation_age_s=observation_age,
            )
            pitch_body_torque_nm = tilt_output.pitch_body_torque_nm
            roll_body_torque_nm = tilt_output.roll_body_torque_nm
            if tilt_output.fault:
                output_torque = 0.0
                pitch_body_torque_nm = 0.0
                roll_body_torque_nm = 0.0
                fault = tilt_output.fault
                mode = "FAULT"

        self.sequence += 1
        result = {
            "protocol_version": 1,
            "source": "cubesat_sil",
            "message_type": "actuator_telemetry",
            "sequence": self.sequence,
            "simulation_time_s": float(packet.get("simulation_time_s", 0.0)),
            "mode": mode,
            "system_state": "FAULT" if fault else "NOMINAL",
            "fault": fault,
            "target_valid": target_valid,
            "target_error_rad": error_rad if target_valid else None,
            "camera_error_x_px": (float(camera["pixel_error_x"])
                                  if target_valid else None),
            # Environment camera Y is positive up; telemetry follows image
            # coordinates and is positive down, matching the physical detector.
            "camera_error_y_px": (-float(camera["pixel_error_y"])
                                  if target_valid else None),
            "camera_error_y_rad": camera_error_y_rad if target_valid else None,
            "pitch_estimate_rad": pitch_estimate_rad,
            "roll_estimate_rad": roll_estimate_rad,
            "camera_focal_length_x_px": float(camera.get("focal_length_x_px", 0.0)),
            "camera_age_s": camera_age if camera_age_valid else None,
            "imu_valid": imu_age_valid and all(
                math.isfinite(v) for v in (rate, *accel_values.values())
            ),
            "imu_age_s": imu_age if imu_age_valid else None,
            "gyro_x_rad_s": float(gyro.get("x", 0.0)),
            "gyro_y_rad_s": float(gyro.get("y", 0.0)),
            "gyro_z_rad_s": float(gyro.get("z", 0.0)),
            "yaw_rate_rad_s": rate,
            "yaw_estimate_rad": yaw_estimate_rad if attitude_valid else None,
            "yaw_estimate_valid": attitude_valid,
            "yaw_estimate_source": attitude_source,
            "yaw_step_target_rad": self.yaw_step_target_rad,
            "yaw_step_error_rad": yaw_step_error_rad,
            "yaw_step_status": self.yaw_step_status,
            "yaw_step_elapsed_s": yaw_step_elapsed_s,
            "accel_x_m_s2": accel_values["x"],
            "accel_y_m_s2": accel_values["y"],
            "accel_z_m_s2": accel_values["z"],
            "mag_valid": magnetometer_valid,
            "mag_frame": "body" if magnetometer_valid else None,
            "mag_age_s": magnetic_age if math.isfinite(magnetic_age) else None,
            "mag_x_uT": magnetic_values["x"] if magnetometer_valid else None,
            "mag_y_uT": magnetic_values["y"] if magnetometer_valid else None,
            "mag_z_uT": magnetic_values["z"] if magnetometer_valid else None,
            "body_torque_nm": output_torque,
            "pitch_body_torque_nm": pitch_body_torque_nm,
            "roll_body_torque_nm": roll_body_torque_nm,
            "virtual_axis_torque_nm": {
                "pitch": pitch_body_torque_nm,
                "roll": roll_body_torque_nm,
            },
            "motor_speed_target_rev_s": motor_speed_target_rev_s,
            "motor_speed_measured_rev_s": measured_motor_speed_rev_s,
            "motor_speed_error_percent": motor_speed_error_percent,
            "motor_speed_test_status": motor_speed_test_status,
            "motor_speed_test_elapsed_s": motor_speed_test_elapsed_s,
        }
        if camera_frame is not None:
            result["camera_frame_source"] = "simulated"
            frame_sequence = packet.get("sequence")
            if (isinstance(frame_sequence, int) and
                    not isinstance(frame_sequence, bool) and frame_sequence > 0):
                result["camera_frame_sequence"] = frame_sequence
        if self.pending_ack is not None:
            result["command_ack"] = self.pending_ack
            self.pending_ack = None
        return result


def request_json(path, payload=None, timeout=0.5):
    url = BASE_URL + path
    data = None if payload is None else json.dumps(payload).encode("utf-8")
    req = Request(url, data=data, headers={"Content-Type": "application/json"})
    with urlopen(req, timeout=timeout) as response:
        return json.loads(response.read().decode("utf-8"))


def request_camera_frame(sequence, timeout=0.5):
    """Fetch and decode the sequence-paired synthetic QVGA frame."""
    url = f"{BASE_URL}/api/camera-frame.rle?sequence={int(sequence)}"
    with urlopen(Request(url), timeout=timeout) as response:
        paired_sequence = response.headers.get("X-Camera-Frame-Sequence")
        if paired_sequence != str(sequence):
            raise ValueError("camera frame sequence does not match observation")
        if response.headers.get("X-Camera-Frame-Encoding") != "RLE1":
            raise ValueError("camera frame has an unsupported encoding")
        if (response.headers.get("X-Camera-Frame-Width") !=
                str(CAMERA_WIDTH_PX) or
                response.headers.get("X-Camera-Frame-Height") !=
                str(CAMERA_HEIGHT_PX)):
            raise ValueError("camera frame dimensions do not match QVGA")
        encoded = response.read(CAMERA_RLE_MAX_BYTES + 1)
    if len(encoded) > CAMERA_RLE_MAX_BYTES:
        raise ValueError("camera frame RLE1 payload exceeds transfer bound")
    return decode_rle_frame(encoded, CAMERA_FRAME_BYTES)


def main():
    onboard = OnboardSoftware()
    print(f"CubeSat SIL connected to simulation bridge at {BASE_URL}")
    print("This host process exercises the onboard path; it is not ESP32 emulation.")
    next_cycle = time.monotonic()
    try:
        while True:
            try:
                cycle_started = time.monotonic()
                cycle = request_json(f"/api/cycle?after={onboard.last_command_id}")
                for command in cycle.get("commands", []):
                    onboard.receive_command(command)
                observation = cycle["observation"]
                frame = request_camera_frame(observation["sequence"])
                actuator_packet = onboard.process_observation(
                    observation, camera_frame=frame,
                    camera_sample_started=cycle_started
                )
                request_json("/api/actuator", actuator_packet)
            except (URLError, HTTPError, TimeoutError, ValueError, TypeError,
                    KeyError) as exc:
                print(f"SIL cycle failed: {exc}")
            now = time.monotonic()
            # Match the firmware: after a deadline miss, resume from now
            # instead of running back-to-back cycles to catch up.
            next_cycle = _next_cycle_deadline(next_cycle, now)
            time.sleep(max(0.0, next_cycle - time.monotonic()))
    except KeyboardInterrupt:
        print("\nCubeSat SIL stopped.")


if __name__ == "__main__":
    main()
