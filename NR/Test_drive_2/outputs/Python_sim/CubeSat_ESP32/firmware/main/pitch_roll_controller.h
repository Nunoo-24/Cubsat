#ifndef AE_PITCH_ROLL_CONTROLLER_H
#define AE_PITCH_ROLL_CONTROLLER_H

#include <stdbool.h>

typedef struct {
    float kp_pitch_nm_per_rad;
    float kd_pitch_nm_per_rad_s;
    float kp_roll_nm_per_rad;
    float kd_roll_nm_per_rad_s;
    float pitch_stiffness_compensation_nm_per_rad;
    float max_pitch_torque_nm;
    float max_roll_torque_nm;
    float max_observation_age_s;
    float max_abs_rate_rad_s;
} cube_pitch_roll_controller_config_t;

typedef struct {
    float pitch_body_torque_nm;
    float roll_body_torque_nm;
    bool valid;
} cube_pitch_roll_control_output_t;

cube_pitch_roll_controller_config_t cube_pitch_roll_controller_default_config(void);
cube_pitch_roll_control_output_t cube_pitch_roll_controller_update(
    const cube_pitch_roll_controller_config_t *config,
    float camera_error_y_rad, bool target_valid,
    float pitch_rad, float roll_rad,
    float pitch_rate_rad_s, float roll_rate_rad_s,
    float observation_age_s);

#endif
