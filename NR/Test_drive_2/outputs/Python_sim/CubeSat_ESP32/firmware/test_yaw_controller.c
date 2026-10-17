#include "yaw_controller.h"

#include <assert.h>
#include <math.h>
#include <stddef.h>

int main(void)
{
    const cube_yaw_controller_config_t config =
        cube_yaw_controller_default_config();

    cube_yaw_control_output_t out = cube_yaw_controller_update(
        &config, 0.2f, true, 0.0f, 0.0f);
    assert(out.mode == CUBE_CONTROL_TRACK);
    assert(out.body_torque_nm > 0.0f);

    out = cube_yaw_controller_update(&config, NAN, false, 0.5f, 0.0f);
    assert(out.mode == CUBE_CONTROL_RATE_DAMP);
    assert(fabsf(out.body_torque_nm + 0.03f) < 1e-6f);

    out = cube_yaw_controller_update(&config, 0.1f, true, 0.0f, 0.2f);
    assert(out.mode == CUBE_CONTROL_FAULT);
    assert(out.fault == CUBE_CONTROL_STALE_OBSERVATION);
    assert(out.body_torque_nm == 0.0f);

    out = cube_yaw_controller_update(&config, 0.1f, true, 21.0f, 0.0f);
    assert(out.fault == CUBE_CONTROL_RATE_LIMIT);

    out = cube_yaw_controller_update(&config, NAN, false, 20.0f, 0.0f);
    assert(out.mode == CUBE_CONTROL_RATE_DAMP);
    out = cube_yaw_controller_update(&config, NAN, false, 20.01f, 0.0f);
    assert(out.fault == CUBE_CONTROL_RATE_LIMIT);

    out = cube_yaw_controller_update(&config, 1.0f, true, -10.0f, 0.0f);
    assert(out.body_torque_nm == 0.05f);

    out = cube_yaw_controller_update(NULL, 0.0f, true, 0.0f, 0.0f);
    assert(out.mode == CUBE_CONTROL_FAULT);
    assert(out.fault == CUBE_CONTROL_INVALID_CONFIG);
    cube_yaw_controller_config_t invalid = config;
    invalid.max_body_torque_nm = 0.0f;
    out = cube_yaw_controller_update(&invalid, 0.1f, true, 0.0f, 0.0f);
    assert(out.fault == CUBE_CONTROL_INVALID_CONFIG);
    assert(out.body_torque_nm == 0.0f);
    invalid = config;
    invalid.kp_nm_per_rad = -0.01f;
    out = cube_yaw_controller_update(&invalid, 0.1f, true, NAN, 0.0f);
    assert(out.fault == CUBE_CONTROL_INVALID_CONFIG);
    return 0;
}
