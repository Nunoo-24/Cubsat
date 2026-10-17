#ifndef CUBESAT_ATTITUDE_ESTIMATOR_H
#define CUBESAT_ATTITUDE_ESTIMATOR_H

#include <stdbool.h>

typedef enum {
    CUBE_ATTITUDE_INVALID = 0,
    CUBE_ATTITUDE_MAG_FUSED,
    CUBE_ATTITUDE_GYRO_ONLY,
} cube_attitude_source_t;

typedef struct {
    float yaw_rad;
    bool initialized;
} cube_yaw_estimator_t;

/* One-axis demonstrator estimator. Acceleration and magnetometer inputs are
 * assumed calibrated and expressed in the project's X=pitch, Y=yaw, Z=roll
 * body frame. Returns whether the current estimate is usable. */
bool cube_yaw_estimator_update(cube_yaw_estimator_t *state,
                               float gyro_y_rad_s,
                               const float accel_m_s2[3],
                               const float magnetic_field_uT[3],
                               bool magnetic_valid,
                               float dt_s,
                               cube_attitude_source_t *source);

#endif
