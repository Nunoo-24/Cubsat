#ifndef AE_MPU6050_H
#define AE_MPU6050_H

#include <stdbool.h>
#include <stdint.h>

typedef struct {
    float gyro_rad_s[3];
    float accel_m_s2[3];
    int64_t sampled_at_us;
} cube_mpu6050_sample_t;

/* Configure the MPU-6050 and estimate gyro bias while stationary. */
bool cube_mpu6050_init(void);

/* Read timestamped, bias-corrected gyro and scaled accelerometer samples. */
bool cube_mpu6050_read(cube_mpu6050_sample_t *sample);

#endif
