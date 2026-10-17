"""Portable yaw control-law reference for software/model-level validation.

Positive output means positive torque applied to the spacecraft body. The
hardware adapter must apply the opposite torque to the reaction wheel and use
the actuator's own verified command convention. Gains and limits are initial
simulation placeholders, not tuned hardware values.
"""

from dataclasses import dataclass
import math


@dataclass(frozen=True)
class ControlOutput:
    body_torque_nm: float
    mode: str
    fault: str = ""


@dataclass
class YawController:
    kp_nm_per_rad: float = 0.08
    kd_nm_per_rad_s: float = 0.04
    max_body_torque_nm: float = 0.05
    max_observation_age_s: float = 0.1
    max_abs_rate_rad_s: float = 20.0

    def update(self, target_error_rad, yaw_rate_rad_s, *, target_valid=True,
               observation_age_s=0.0):
        config = (self.kp_nm_per_rad, self.kd_nm_per_rad_s,
                  self.max_body_torque_nm, self.max_observation_age_s,
                  self.max_abs_rate_rad_s)
        if (not all(math.isfinite(v) for v in config)
                or self.kp_nm_per_rad < 0 or self.kd_nm_per_rad_s < 0
                or self.max_body_torque_nm <= 0
                or self.max_observation_age_s <= 0
                or self.max_abs_rate_rad_s <= 0):
            return ControlOutput(0.0, "FAULT", "INVALID_CONFIG")
        values = (yaw_rate_rad_s, observation_age_s)
        if not all(math.isfinite(v) for v in values):
            return ControlOutput(0.0, "FAULT", "NONFINITE_SENSOR")
        if abs(yaw_rate_rad_s) > self.max_abs_rate_rad_s:
            return ControlOutput(0.0, "FAULT", "RATE_LIMIT")
        if observation_age_s < 0 or observation_age_s > self.max_observation_age_s:
            return ControlOutput(0.0, "FAULT", "STALE_OBSERVATION")
        if target_valid:
            if target_error_rad is None or not math.isfinite(target_error_rad):
                return ControlOutput(0.0, "FAULT", "INVALID_TARGET_ERROR")
            command = (self.kp_nm_per_rad * target_error_rad
                       - self.kd_nm_per_rad_s * yaw_rate_rad_s)
            mode = "TRACK"
        else:
            # A temporary camera loss must not create a stale target command.
            command = -self.kd_nm_per_rad_s * yaw_rate_rad_s
            mode = "RATE_DAMP"
        command = max(-self.max_body_torque_nm,
                      min(self.max_body_torque_nm, command))
        return ControlOutput(command, mode)


@dataclass(frozen=True)
class PitchRollControlOutput:
    pitch_body_torque_nm: float
    roll_body_torque_nm: float
    fault: str = ""


@dataclass
class PitchRollController:
    """Host-SIL pitch tracking and roll-leveling controller.

    Camera image Y error steers pitch. Accelerometer-derived roll holds the
    body level because a single point target cannot observe rotation about the
    camera boresight. Gains and virtual actuator limits are simulation
    placeholders; this controller is not connected to physical actuators.
    """

    kp_pitch_nm_per_rad: float = 0.08
    kd_pitch_nm_per_rad_s: float = 0.04
    kp_roll_nm_per_rad: float = 0.4
    kd_roll_nm_per_rad_s: float = 0.3
    max_pitch_torque_nm: float = 0.02
    max_roll_torque_nm: float = 0.05
    pitch_stiffness_compensation_nm_per_rad: float = 0.02
    max_observation_age_s: float = 0.1
    max_abs_rate_rad_s: float = 20.0

    def update(self, camera_error_y_rad, pitch_rad, roll_rad,
               pitch_rate_rad_s, roll_rate_rad_s, *, target_valid=True,
               observation_age_s=0.0):
        config = (self.kp_pitch_nm_per_rad, self.kd_pitch_nm_per_rad_s,
                  self.kp_roll_nm_per_rad, self.kd_roll_nm_per_rad_s,
                  self.max_pitch_torque_nm, self.max_roll_torque_nm,
                  self.pitch_stiffness_compensation_nm_per_rad,
                  self.max_observation_age_s, self.max_abs_rate_rad_s)
        if (not all(math.isfinite(value) for value in config) or
                min(self.kp_pitch_nm_per_rad, self.kd_pitch_nm_per_rad_s,
                    self.kp_roll_nm_per_rad, self.kd_roll_nm_per_rad_s,
                    self.pitch_stiffness_compensation_nm_per_rad) < 0 or
                self.max_pitch_torque_nm <= 0 or
                self.max_roll_torque_nm <= 0 or
                self.max_observation_age_s <= 0 or
                self.max_abs_rate_rad_s <= 0):
            return PitchRollControlOutput(0.0, 0.0, "INVALID_CONFIG")
        values = (pitch_rad, roll_rad, pitch_rate_rad_s,
                  roll_rate_rad_s, observation_age_s)
        if target_valid:
            values += (camera_error_y_rad,)
        if not all(math.isfinite(value) for value in values):
            return PitchRollControlOutput(0.0, 0.0, "NONFINITE_SENSOR")
        if observation_age_s < 0 or observation_age_s > self.max_observation_age_s:
            return PitchRollControlOutput(0.0, 0.0, "STALE_OBSERVATION")
        if max(abs(pitch_rate_rad_s), abs(roll_rate_rad_s)) > self.max_abs_rate_rad_s:
            return PitchRollControlOutput(0.0, 0.0, "RATE_LIMIT")

        pitch_error = (camera_error_y_rad if target_valid else -pitch_rad)
        pitch_torque = (
            self.kp_pitch_nm_per_rad * pitch_error
            - self.kd_pitch_nm_per_rad_s * pitch_rate_rad_s
            + self.pitch_stiffness_compensation_nm_per_rad * pitch_rad
        )
        roll_torque = (
            -self.kp_roll_nm_per_rad * roll_rad
            -self.kd_roll_nm_per_rad_s * roll_rate_rad_s
        )
        return PitchRollControlOutput(
            max(-self.max_pitch_torque_nm,
                min(self.max_pitch_torque_nm, pitch_torque)),
            max(-self.max_roll_torque_nm,
                min(self.max_roll_torque_nm, roll_torque)),
        )
