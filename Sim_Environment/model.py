"""Small, dependency-free gimbal plant for simulation and model checks.

Sign convention: positive yaw rotates the body counter-clockwise viewed from
above. Positive external torque increases positive yaw. Positive wheel torque
is the torque applied to the body (the wheel receives the opposite torque).
All parameters are explicit placeholders pending measurement.
"""

from dataclasses import dataclass, field
import math


@dataclass
class ReactionWheel:
    """One-axis reaction-wheel actuator model; all values are placeholders."""

    inertia_kg_m2: float = 1.0e-4
    max_motor_torque_nm: float = 0.01
    max_speed_rad_s: float = 500.0
    motor_time_constant_s: float = 0.05
    speed_rad_s: float = 0.0
    motor_torque_nm: float = 0.0
    saturated: bool = False

    def __post_init__(self):
        values = (self.inertia_kg_m2, self.max_motor_torque_nm,
                  self.max_speed_rad_s, self.motor_time_constant_s)
        if not all(math.isfinite(value) for value in values):
            raise ValueError("wheel parameters must be finite")
        if self.inertia_kg_m2 <= 0 or self.max_motor_torque_nm <= 0:
            raise ValueError("wheel inertia and max torque must be positive")
        if self.max_speed_rad_s <= 0 or self.motor_time_constant_s <= 0:
            raise ValueError("wheel speed and motor time constant must be positive")

    @staticmethod
    def _validate_step(dt_s):
        if not math.isfinite(dt_s) or dt_s <= 0.0 or dt_s > 0.1:
            raise ValueError("dt_s must be finite and in (0, 0.1]")

    def step(self, desired_body_torque_nm, dt_s):
        """Return actual body torque after motor lag, torque and speed limits."""
        self._validate_step(dt_s)
        if not math.isfinite(desired_body_torque_nm):
            raise ValueError("desired body torque must be finite")
        requested_motor_torque = max(
            -self.max_motor_torque_nm,
            min(self.max_motor_torque_nm, -desired_body_torque_nm),
        )
        alpha = min(1.0, dt_s / self.motor_time_constant_s)
        motor_torque = self.motor_torque_nm + alpha * (
            requested_motor_torque - self.motor_torque_nm
        )
        proposed_speed = self.speed_rad_s + (
            motor_torque / self.inertia_kg_m2
        ) * dt_s
        limited_speed = max(-self.max_speed_rad_s,
                            min(self.max_speed_rad_s, proposed_speed))
        self.saturated = limited_speed != proposed_speed
        if self.saturated:
            # Do not apply a torque that would push the wheel beyond its speed limit.
            motor_torque = ((limited_speed - self.speed_rad_s)
                            * self.inertia_kg_m2 / dt_s)
        self.motor_torque_nm = motor_torque
        self.speed_rad_s = limited_speed
        return -motor_torque

    def step_speed_target(self, target_rev_s, dt_s):
        """Simulate an optimistic acceleration-limited native speed servo.

        The speed loop is not modeled. This uses the motor torque ceiling and
        wheel inertia to limit acceleration, then tracks the velocity target
        exactly. It is an optimistic actuator bound, not a calibrated response.
        """
        self._validate_step(dt_s)
        if not math.isfinite(target_rev_s):
            raise ValueError("target_rev_s must be finite")
        target_rad_s = target_rev_s * (2.0 * math.pi / 60.0)
        requested_delta = target_rad_s - self.speed_rad_s
        max_delta = self.max_motor_torque_nm / self.inertia_kg_m2 * dt_s
        applied_delta = max(-max_delta, min(max_delta, requested_delta))
        proposed_speed = self.speed_rad_s + applied_delta
        limited_speed = max(-self.max_speed_rad_s,
                           min(self.max_speed_rad_s, proposed_speed))
        self.saturated = (applied_delta != requested_delta or
                          limited_speed != proposed_speed)
        self.motor_torque_nm = ((limited_speed - self.speed_rad_s)
                                * self.inertia_kg_m2 / dt_s)
        self.speed_rad_s = limited_speed
        return -self.motor_torque_nm


@dataclass
class YawPlant:
    """Decoupled 3-axis gimbal model with yaw and virtual tilt actuation.

    Simulation body frame maps X torque to pitch, Y torque to yaw, and Z
    torque to roll. The Y axis uses a reaction wheel; X and Z accept optional
    externally supplied simulated body torques atop linear pendulum restoring
    terms. Euler-rate coupling, cross-axis inertia, and large-angle dynamics
    are omitted.
    """

    inertia_kg_m2: float = 0.02
    damping_n_m_per_rad_s: float = 0.002
    yaw_rad: float = 0.0
    rate_rad_s: float = 0.0
    time_s: float = 0.0
    wheel: ReactionWheel = field(default_factory=ReactionWheel)
    actual_body_torque_nm: float = 0.0
    roll_rad: float = 0.0
    pitch_rad: float = 0.0
    roll_rate_rad_s: float = 0.0
    pitch_rate_rad_s: float = 0.0
    roll_inertia_kg_m2: float = 0.02
    pitch_inertia_kg_m2: float = 0.02
    roll_damping_n_m_per_rad_s: float = 0.002
    pitch_damping_n_m_per_rad_s: float = 0.002
    pendulum_stiffness_nm_per_rad: float = 0.02

    def __post_init__(self):
        positive = (self.inertia_kg_m2, self.roll_inertia_kg_m2,
                    self.pitch_inertia_kg_m2)
        nonnegative = (self.damping_n_m_per_rad_s,
                       self.roll_damping_n_m_per_rad_s,
                       self.pitch_damping_n_m_per_rad_s,
                       self.pendulum_stiffness_nm_per_rad)
        if not all(math.isfinite(value) for value in positive + nonnegative):
            raise ValueError("plant parameters must be finite")
        if min(positive) <= 0 or min(nonnegative) < 0:
            raise ValueError("inertias must be positive and damping/restoring nonnegative")

    def step(self, external_torque_nm=0.0, body_wheel_torque_nm=0.0, dt_s=0.01,
             motor_speed_target_rev_s=None, body_pitch_torque_nm=0.0,
             body_roll_torque_nm=0.0):
        """Advance state; external torque may be scalar yaw or an XYZ mapping.

        ``body_pitch_torque_nm`` and ``body_roll_torque_nm`` represent virtual
        simulation actuators and are not connected to the physical hardware
        adapter.
        """
        if not math.isfinite(dt_s) or dt_s <= 0 or dt_s > 0.1:
            raise ValueError("dt_s must be finite and in (0, 0.1]")
        if isinstance(external_torque_nm, dict):
            torque_x = float(external_torque_nm.get("x", 0.0))
            torque_y = float(external_torque_nm.get("y", 0.0))
            torque_z = float(external_torque_nm.get("z", 0.0))
        elif isinstance(external_torque_nm, (tuple, list)):
            if len(external_torque_nm) != 3:
                raise ValueError("external torque sequence must contain XYZ")
            torque_x, torque_y, torque_z = map(float, external_torque_nm)
        else:
            # Backward-compatible scalar input means yaw-axis disturbance.
            torque_x, torque_y, torque_z = 0.0, float(external_torque_nm), 0.0
        if not all(math.isfinite(v) for v in (torque_x, torque_y, torque_z,
                                               body_wheel_torque_nm,
                                               body_pitch_torque_nm,
                                               body_roll_torque_nm)):
            raise ValueError("torques must be finite")

        # Model frame: X is pitch, Y is yaw (reaction-wheel axis), Z is roll.
        roll_accel = (torque_z + body_roll_torque_nm
                      - self.pendulum_stiffness_nm_per_rad * self.roll_rad
                      - self.roll_damping_n_m_per_rad_s * self.roll_rate_rad_s
                      ) / self.roll_inertia_kg_m2
        pitch_accel = (torque_x + body_pitch_torque_nm
                       - self.pendulum_stiffness_nm_per_rad * self.pitch_rad
                       - self.pitch_damping_n_m_per_rad_s * self.pitch_rate_rad_s
                       ) / self.pitch_inertia_kg_m2
        self.roll_rate_rad_s += roll_accel * dt_s
        self.pitch_rate_rad_s += pitch_accel * dt_s
        self.roll_rad = wrap_pi(self.roll_rad + self.roll_rate_rad_s * dt_s)
        self.pitch_rad = wrap_pi(self.pitch_rad + self.pitch_rate_rad_s * dt_s)

        if motor_speed_target_rev_s is None:
            self.actual_body_torque_nm = self.wheel.step(body_wheel_torque_nm, dt_s)
        else:
            self.actual_body_torque_nm = self.wheel.step_speed_target(
                motor_speed_target_rev_s, dt_s
            )
        torque = (torque_y + self.actual_body_torque_nm
                  - self.damping_n_m_per_rad_s * self.rate_rad_s)
        self.rate_rad_s += torque / self.inertia_kg_m2 * dt_s
        self.yaw_rad = wrap_pi(self.yaw_rad + self.rate_rad_s * dt_s)
        self.time_s += dt_s
        return self.yaw_rad, self.rate_rad_s


def wrap_pi(angle_rad):
    """Wrap radians into [-pi, pi)."""
    return (angle_rad + math.pi) % (2.0 * math.pi) - math.pi


def target_bearing_error(target_bearing_rad, body_yaw_rad):
    """Return signed shortest yaw error from body boresight to target."""
    return wrap_pi(target_bearing_rad - body_yaw_rad)
