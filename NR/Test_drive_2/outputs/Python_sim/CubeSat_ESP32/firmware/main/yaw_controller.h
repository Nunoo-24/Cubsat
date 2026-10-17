#ifndef AE_YAW_CONTROLLER_H
#define AE_YAW_CONTROLLER_H

#include <stdbool.h>

typedef enum {
    CUBE_CONTROL_TRACK = 0,
    CUBE_CONTROL_RATE_DAMP,
    CUBE_CONTROL_FAULT
} cube_control_mode_t;

typedef enum {
    CUBE_CONTROL_NO_FAULT = 0,
    CUBE_CONTROL_NONFINITE_SENSOR,
    CUBE_CONTROL_RATE_LIMIT,
    CUBE_CONTROL_STALE_OBSERVATION,
    CUBE_CONTROL_INVALID_TARGET_ERROR,
    CUBE_CONTROL_INVALID_CONFIG
} cube_control_fault_t;

typedef struct {
    float kp_nm_per_rad;
    float kd_nm_per_rad_s;
    float kd_lost_target_nm_per_rad_s;
    float max_body_torque_nm;
    float max_observation_age_s;
    float max_abs_rate_rad_s;
} cube_yaw_controller_config_t;

typedef struct {
    float body_torque_nm;
    cube_control_mode_t mode;
    cube_control_fault_t fault;
} cube_yaw_control_output_t;

cube_yaw_controller_config_t cube_yaw_controller_default_config(void);

cube_yaw_control_output_t cube_yaw_controller_update(
    const cube_yaw_controller_config_t *config,
    float target_error_rad,
    bool target_valid,
    float yaw_rate_rad_s,
    float observation_age_s);

#endif
