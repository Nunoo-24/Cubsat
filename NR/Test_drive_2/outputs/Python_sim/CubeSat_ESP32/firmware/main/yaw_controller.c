#include "yaw_controller.h"

#include <math.h>
#include <stddef.h>

cube_yaw_controller_config_t cube_yaw_controller_default_config(void)
{
    return (cube_yaw_controller_config_t){
        .kp_nm_per_rad = 0.08f,
        .kd_nm_per_rad_s = 0.04f,
        .kd_lost_target_nm_per_rad_s = 0.06f,
        .max_body_torque_nm = 0.05f,
        .max_observation_age_s = 0.1f,
        .max_abs_rate_rad_s = 20.0f,
    };
}

static cube_yaw_control_output_t fault_output(cube_control_fault_t fault)
{
    return (cube_yaw_control_output_t){
        .body_torque_nm = 0.0f,
        .mode = CUBE_CONTROL_FAULT,
        .fault = fault,
    };
}

cube_yaw_control_output_t cube_yaw_controller_update(
    const cube_yaw_controller_config_t *config,
    float target_error_rad,
    bool target_valid,
    float yaw_rate_rad_s,
    float observation_age_s)
{
    if (config == NULL) {
        return fault_output(CUBE_CONTROL_INVALID_CONFIG);
    }
    if (!isfinite(config->kp_nm_per_rad) ||
        !isfinite(config->kd_nm_per_rad_s) ||
        !isfinite(config->kd_lost_target_nm_per_rad_s) ||
        !isfinite(config->max_body_torque_nm) ||
        !isfinite(config->max_observation_age_s) ||
        !isfinite(config->max_abs_rate_rad_s)) {
        return fault_output(CUBE_CONTROL_INVALID_CONFIG);
    }
    if (config->kp_nm_per_rad < 0.0f ||
        config->kd_nm_per_rad_s < 0.0f ||
        config->kd_lost_target_nm_per_rad_s < 0.0f ||
        config->max_body_torque_nm <= 0.0f ||
        config->max_observation_age_s <= 0.0f ||
        config->max_abs_rate_rad_s <= 0.0f) {
        return fault_output(CUBE_CONTROL_INVALID_CONFIG);
    }
    if (!isfinite(yaw_rate_rad_s) || !isfinite(observation_age_s)) {
        return fault_output(CUBE_CONTROL_NONFINITE_SENSOR);
    }
    if (fabsf(yaw_rate_rad_s) > config->max_abs_rate_rad_s) {
        return fault_output(CUBE_CONTROL_RATE_LIMIT);
    }
    if (observation_age_s < 0.0f ||
        observation_age_s > config->max_observation_age_s) {
        return fault_output(CUBE_CONTROL_STALE_OBSERVATION);
    }

    const float kd_rate = target_valid
        ? config->kd_nm_per_rad_s
        : config->kd_lost_target_nm_per_rad_s;
    float command_nm = -kd_rate * yaw_rate_rad_s;
    cube_control_mode_t mode = CUBE_CONTROL_RATE_DAMP;
    if (target_valid) {
        if (!isfinite(target_error_rad)) {
            return fault_output(CUBE_CONTROL_INVALID_TARGET_ERROR);
        }
        command_nm += config->kp_nm_per_rad * target_error_rad;
        mode = CUBE_CONTROL_TRACK;
    }

    const float limit = fabsf(config->max_body_torque_nm);
    if (command_nm > limit) {
        command_nm = limit;
    } else if (command_nm < -limit) {
        command_nm = -limit;
    }

    return (cube_yaw_control_output_t){
        .body_torque_nm = command_nm,
        .mode = mode,
        .fault = CUBE_CONTROL_NO_FAULT,
    };
}
