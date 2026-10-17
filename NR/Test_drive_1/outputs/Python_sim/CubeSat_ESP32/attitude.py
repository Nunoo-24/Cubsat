"""Yaw attitude estimator for the project body-frame convention.

Gyro Y is integrated at the control cadence. A tilt-compensated magnetic
heading corrects yaw drift when fresh, calibrated-enough sensor vectors are
available. This is a one-axis demonstrator estimator, not a general AHRS.
"""

import math


GRAVITY_M_S2 = 9.80665
WORLD_MAGNETIC_FIELD_UT = (20.0, 0.0, 45.0)
MAG_CORRECTION_TAU_S = 1.0


def wrap_pi(angle):
    return (angle + math.pi) % (2.0 * math.pi) - math.pi


class YawAttitudeEstimator:
    """Fuse body-Y gyro with gravity and magnetic vectors for yaw only."""

    def __init__(self, correction_tau_s=MAG_CORRECTION_TAU_S):
        if not math.isfinite(correction_tau_s) or correction_tau_s <= 0.0:
            raise ValueError("correction_tau_s must be finite and positive")
        self.correction_tau_s = correction_tau_s
        self.yaw_rad = 0.0
        self.initialized = False

    def update(self, gyro_y_rad_s, accel_m_s2, magnetic_field_uT,
               dt_s, magnetic_valid=True):
        """Return ``(yaw_rad, valid, source)`` for one sensor sample.

        Accelerometer correction is used only while its norm is within 20% of
        gravity. Magnetometer values are assumed hard/soft-iron calibrated;
        those calibration parameters are not yet available for the project.
        """
        if (not math.isfinite(gyro_y_rad_s) or not math.isfinite(dt_s) or
                dt_s <= 0.0 or dt_s > 0.1):
            return self.yaw_rad, False, "INVALID"
        ax, ay, az = accel_m_s2
        if not all(math.isfinite(value) for value in (ax, ay, az)):
            return self.yaw_rad, False, "INVALID"
        accel_norm = math.sqrt(ax * ax + ay * ay + az * az)
        accel_valid = (0.8 * GRAVITY_M_S2 <= accel_norm <=
                       1.2 * GRAVITY_M_S2)
        mag_valid = False
        mag_yaw = None
        if magnetic_valid and accel_valid:
            mx, my, mz = magnetic_field_uT
            mag_norm = math.sqrt(mx * mx + my * my + mz * mz)
            mag_valid = (all(math.isfinite(v) for v in (mx, my, mz)) and
                         1.0 <= mag_norm <= 1000.0)
            if mag_valid:
                # For R_body_to_world = Ry(yaw) Rx(pitch) Rz(roll),
                # gravity provides roll/pitch and the tilted field is rotated
                # back into the horizontal plane before extracting yaw.
                roll = math.atan2(ax, ay)
                pitch = math.asin(max(-1.0, min(1.0, -az / accel_norm)))
                cr, sr = math.cos(roll), math.sin(roll)
                cp, sp = math.cos(pitch), math.sin(pitch)
                mx1 = cr * mx - sr * my
                my1 = sr * mx + cr * my
                mz1 = mz
                mxh = mx1
                mzh = sp * my1 + cp * mz1
                field_x, _, field_z = WORLD_MAGNETIC_FIELD_UT
                mag_yaw = wrap_pi(math.atan2(mzh, mxh) -
                                  math.atan2(field_z, field_x))

        self.yaw_rad = wrap_pi(self.yaw_rad + gyro_y_rad_s * dt_s)
        if mag_valid and mag_yaw is not None:
            if not self.initialized:
                self.yaw_rad = mag_yaw
                self.initialized = True
            else:
                alpha = 1.0 - math.exp(-dt_s / self.correction_tau_s)
                self.yaw_rad = wrap_pi(
                    self.yaw_rad + alpha * wrap_pi(mag_yaw - self.yaw_rad)
                )
            return self.yaw_rad, True, "MAG_FUSED"
        if accel_valid:
            # Relative yaw is defined as zero at the first valid sample; it
            # remains gyro-only until a trustworthy magnetic reference exists.
            self.initialized = True
        if self.initialized:
            return self.yaw_rad, True, "GYRO_ONLY"
        return self.yaw_rad, False, "INVALID"
