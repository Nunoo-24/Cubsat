#include "attitude_estimator.h"

#include <math.h>

#define ESTIMATOR_GRAVITY_M_S2 9.80665f
#define ESTIMATOR_MAG_CORRECTION_TAU_S 1.0f
#define ESTIMATOR_PI 3.14159265358979323846f
#define ESTIMATOR_WORLD_MAG_X_UT 20.0f
#define ESTIMATOR_WORLD_MAG_Z_UT 45.0f

static float wrap_pi(float angle)
{
    const float two_pi = 2.0f * ESTIMATOR_PI;
    angle = fmodf(angle + ESTIMATOR_PI, two_pi);
    if (angle < 0.0f) angle += two_pi;
    return angle - ESTIMATOR_PI;
}

bool cube_yaw_estimator_update(cube_yaw_estimator_t *state,
                               float gyro_y_rad_s,
                               const float accel_m_s2[3],
                               const float magnetic_field_uT[3],
                               bool magnetic_valid,
                               float dt_s,
                               cube_attitude_source_t *source)
{
    if (source != NULL) *source = CUBE_ATTITUDE_INVALID;
    if (state == NULL || accel_m_s2 == NULL || source == NULL ||
        !isfinite(gyro_y_rad_s) || !isfinite(dt_s) || dt_s <= 0.0f ||
        dt_s > 0.1f) {
        return false;
    }
    for (int i = 0; i < 3; ++i) {
        if (!isfinite(accel_m_s2[i])) return false;
    }

    const float ax = accel_m_s2[0];
    const float ay = accel_m_s2[1];
    const float az = accel_m_s2[2];
    const float accel_norm = sqrtf(ax * ax + ay * ay + az * az);
    const bool accel_valid = isfinite(accel_norm) &&
        accel_norm >= 0.8f * ESTIMATOR_GRAVITY_M_S2 &&
        accel_norm <= 1.2f * ESTIMATOR_GRAVITY_M_S2;

    bool mag_is_valid = false;
    float mag_yaw = 0.0f;
    if (magnetic_valid && accel_valid && magnetic_field_uT != NULL) {
        const float mx = magnetic_field_uT[0];
        const float my = magnetic_field_uT[1];
        const float mz = magnetic_field_uT[2];
        if (isfinite(mx) && isfinite(my) && isfinite(mz)) {
            const float mag_norm = sqrtf(mx * mx + my * my + mz * mz);
            mag_is_valid = isfinite(mag_norm) && mag_norm >= 1.0f &&
                           mag_norm <= 1000.0f;
            if (mag_is_valid) {
                const float roll = atan2f(ax, ay);
                const float ratio = fmaxf(-1.0f, fminf(1.0f, -az / accel_norm));
                const float pitch = asinf(ratio);
                const float cr = cosf(roll), sr = sinf(roll);
                const float cp = cosf(pitch), sp = sinf(pitch);
                const float mx1 = cr * mx - sr * my;
                const float my1 = sr * mx + cr * my;
                const float mzh = sp * my1 + cp * mz;
                const float world_angle = atan2f(ESTIMATOR_WORLD_MAG_Z_UT,
                                                  ESTIMATOR_WORLD_MAG_X_UT);
                mag_yaw = wrap_pi(atan2f(mzh, mx1) - world_angle);
            }
        }
    }

    state->yaw_rad = wrap_pi(state->yaw_rad + gyro_y_rad_s * dt_s);
    if (mag_is_valid) {
        if (!state->initialized) {
            state->yaw_rad = mag_yaw;
            state->initialized = true;
        } else {
            const float alpha = 1.0f - expf(
                -dt_s / ESTIMATOR_MAG_CORRECTION_TAU_S);
            state->yaw_rad = wrap_pi(state->yaw_rad + alpha *
                                     wrap_pi(mag_yaw - state->yaw_rad));
        }
        *source = CUBE_ATTITUDE_MAG_FUSED;
        return true;
    }
    if (accel_valid) {
        /* Relative yaw is zeroed at first valid sample without a calibrated
         * magnetic reference, and then integrated from the gyro only. */
        state->initialized = true;
    }
    if (state->initialized) {
        *source = CUBE_ATTITUDE_GYRO_ONLY;
        return true;
    }
    return false;
}
