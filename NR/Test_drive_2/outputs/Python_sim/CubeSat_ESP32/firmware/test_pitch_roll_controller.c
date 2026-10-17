#include "pitch_roll_controller.h"

#include <assert.h>
#include <math.h>

int main(void)
{
    const cube_pitch_roll_controller_config_t config =
        cube_pitch_roll_controller_default_config();
    cube_pitch_roll_control_output_t out = cube_pitch_roll_controller_update(
        &config, 0.1f, true, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f);
    assert(out.valid && out.pitch_body_torque_nm > 0.0f);
    assert(out.roll_body_torque_nm == 0.0f);

    out = cube_pitch_roll_controller_update(
        &config, 0.0f, true, 0.0f, 0.2f, 0.0f, 0.0f, 0.0f);
    assert(out.valid && out.roll_body_torque_nm < 0.0f);
    assert(fabsf(out.roll_body_torque_nm) <= config.max_roll_torque_nm);

    out = cube_pitch_roll_controller_update(
        &config, 0.0f, false, 0.1f, -0.1f, 0.0f, 0.0f, 0.0f);
    assert(out.valid && out.pitch_body_torque_nm < 0.0f);
    assert(out.roll_body_torque_nm > 0.0f);

    out = cube_pitch_roll_controller_update(
        &config, NAN, false, 0.0f, 0.0f, 0.0f, 0.0f, 0.2f);
    assert(!out.valid && out.pitch_body_torque_nm == 0.0f &&
           out.roll_body_torque_nm == 0.0f);
    out = cube_pitch_roll_controller_update(
        &config, NAN, true, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f);
    assert(!out.valid);
    out = cube_pitch_roll_controller_update(
        NULL, 0.0f, false, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f);
    assert(!out.valid);
    return 0;
}
