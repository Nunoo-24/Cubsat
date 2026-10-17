#include "pitch_roll_controller.h"

#include <math.h>
#include <stddef.h>

cube_pitch_roll_controller_config_t cube_pitch_roll_controller_default_config(void)
{
    return (cube_pitch_roll_controller_config_t){
        .kp_pitch_nm_per_rad = 0.08f,
        .kd_pitch_nm_per_rad_s = 0.04f,
        .kp_roll_nm_per_rad = 0.24f,
        .kd_roll_nm_per_rad_s = 0.19f,
        .pitch_stiffness_compensation_nm_per_rad = 0.02f,
        .max_pitch_torque_nm = 0.02f,
        .max_roll_torque_nm = 0.035f,
        .max_observation_age_s = 0.1f,
        .max_abs_rate_rad_s = 20.0f,
    };
}

static float clamp_torque(float value, float limit)
{
    if (value > limit) return limit;
    if (value < -limit) return -limit;
    return value;
}

cube_pitch_roll_control_output_t cube_pitch_roll_controller_update(
    const cube_pitch_roll_controller_config_t *config,
    float camera_error_y_rad, bool target_valid,
    float pitch_rad, float roll_rad,
    float pitch_rate_rad_s, float roll_rate_rad_s,
    float observation_age_s)
{
    cube_pitch_roll_control_output_t out = {0};
    if (config == NULL) return out;
    const float values[] = {
        config->kp_pitch_nm_per_rad, config->kd_pitch_nm_per_rad_s,
        config->kp_roll_nm_per_rad, config->kd_roll_nm_per_rad_s,
        config->pitch_stiffness_compensation_nm_per_rad,
        config->max_pitch_torque_nm, config->max_roll_torque_nm,
        config->max_observation_age_s, config->max_abs_rate_rad_s,
        pitch_rad, roll_rad, pitch_rate_rad_s, roll_rate_rad_s,
        observation_age_s,
    };
    for (size_t i = 0; i < sizeof(values) / sizeof(values[0]); ++i) {
        if (!isfinite(values[i])) return out;
    }
    if (target_valid && !isfinite(camera_error_y_rad)) return out;
    if (config->kp_pitch_nm_per_rad < 0.0f ||
        config->kd_pitch_nm_per_rad_s < 0.0f ||
        config->kp_roll_nm_per_rad < 0.0f ||
        config->kd_roll_nm_per_rad_s < 0.0f ||
        config->pitch_stiffness_compensation_nm_per_rad < 0.0f ||
        config->max_pitch_torque_nm <= 0.0f ||
        config->max_roll_torque_nm <= 0.0f ||
        config->max_observation_age_s <= 0.0f ||
        config->max_abs_rate_rad_s <= 0.0f ||
        observation_age_s < 0.0f ||
        observation_age_s > config->max_observation_age_s ||
        fabsf(pitch_rate_rad_s) > config->max_abs_rate_rad_s ||
        fabsf(roll_rate_rad_s) > config->max_abs_rate_rad_s) {
        return out;
    }
    const float pitch_error = target_valid ? camera_error_y_rad : -pitch_rad;
    const float pitch_torque = config->kp_pitch_nm_per_rad * pitch_error -
        config->kd_pitch_nm_per_rad_s * pitch_rate_rad_s +
        config->pitch_stiffness_compensation_nm_per_rad * pitch_rad;
    const float roll_torque = -config->kp_roll_nm_per_rad * roll_rad -
        config->kd_roll_nm_per_rad_s * roll_rate_rad_s;
    out.pitch_body_torque_nm = clamp_torque(pitch_torque,
                                             config->max_pitch_torque_nm);
    out.roll_body_torque_nm = clamp_torque(roll_torque,
                                            config->max_roll_torque_nm);
    out.valid = true;
    return out;
}
