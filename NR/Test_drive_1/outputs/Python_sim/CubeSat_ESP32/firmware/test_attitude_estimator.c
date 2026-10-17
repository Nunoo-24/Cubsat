#include "attitude_estimator.h"

#include <assert.h>
#include <math.h>

static void make_sample(float yaw, float roll, float pitch,
                        float accel[3], float mag[3])
{
    const float g = 9.80665f;
    const float cy = cosf(yaw), sy = sinf(yaw);
    const float cp = cosf(pitch), sp = sinf(pitch);
    const float cr = cosf(roll), sr = sinf(roll);
    const float bx = cy * 20.0f - sy * 45.0f;
    const float bz = sy * 20.0f + cy * 45.0f;
    const float by2 = sp * bz;
    const float bz2 = cp * bz;
    accel[0] = g * sr * cp;
    accel[1] = g * cr * cp;
    accel[2] = -g * sp;
    mag[0] = cr * bx + sr * by2;
    mag[1] = -sr * bx + cr * by2;
    mag[2] = bz2;
}

int main(void)
{
    cube_yaw_estimator_t estimator = {0};
    cube_attitude_source_t source;
    float accel[3], mag[3];

    make_sample(0.261799388f, 0.174532925f, -0.122173048f, accel, mag);
    assert(cube_yaw_estimator_update(&estimator, 0.0f, accel, mag, true,
                                     0.02f, &source));
    assert(source == CUBE_ATTITUDE_MAG_FUSED);
    assert(fabsf(estimator.yaw_rad - 0.261799388f) < 1e-5f);

    make_sample(estimator.yaw_rad, 0.0f, 0.0f, accel, mag);
    for (int i = 0; i < 3000; ++i) {
        assert(cube_yaw_estimator_update(&estimator, 0.001f, accel, mag,
                                         true, 0.02f, &source));
    }
    assert(source == CUBE_ATTITUDE_MAG_FUSED);
    assert(fabsf(estimator.yaw_rad - 0.261799388f) < 0.01f);

    assert(cube_yaw_estimator_update(&estimator, 0.01f, accel, mag,
                                     false, 0.02f, &source));
    assert(source == CUBE_ATTITUDE_GYRO_ONLY);
    assert(!cube_yaw_estimator_update(&estimator, NAN, accel, mag,
                                      false, 0.02f, &source));
    assert(source == CUBE_ATTITUDE_INVALID);
    const float yaw_before_invalid_accel = estimator.yaw_rad;
    accel[0] = NAN;
    assert(!cube_yaw_estimator_update(&estimator, 1.0f, accel, mag,
                                      false, 0.02f, &source));
    assert(source == CUBE_ATTITUDE_INVALID);
    assert(estimator.yaw_rad == yaw_before_invalid_accel);
    return 0;
}
