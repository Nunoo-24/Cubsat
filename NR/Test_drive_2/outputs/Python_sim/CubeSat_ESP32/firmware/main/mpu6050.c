#include "mpu6050.h"

#include <math.h>
#include <stddef.h>
#include <string.h>

#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "sensor_bus.h"
#include "sdkconfig.h"

#define MPU6050_REG_SMPLRT_DIV 0x19u
#define MPU6050_REG_CONFIG 0x1Au
#define MPU6050_REG_GYRO_CONFIG 0x1Bu
#define MPU6050_REG_ACCEL_CONFIG 0x1Cu
#define MPU6050_REG_ACCEL_XOUT_H 0x3Bu
#define MPU6050_REG_GYRO_XOUT_H 0x43u
#define MPU6050_REG_PWR_MGMT_1 0x6Bu
#define MPU6050_REG_WHO_AM_I 0x75u
#define MPU6050_WHO_AM_I_VALUE 0x68u
#define MPU6050_I2C_HZ 100000u
#define MPU6050_I2C_TIMEOUT_MS 100
#define MPU6050_CALIBRATION_SAMPLES 200u
#define MPU6050_CALIBRATION_PERIOD_MS 5u
#define MPU6050_FILTER_DELAY_US 4900LL
#define MPU6050_OUTPUT_PERIOD_US 5000LL
#define MPU6050_GYRO_LSB_PER_DPS 131.0f
#define MPU6050_ACCEL_LSB_PER_G 16384.0f
#define STANDARD_GRAVITY_M_S2 9.80665f
#define DEGREES_TO_RADIANS 0.01745329251994329577f

static const char *TAG = "CUBESAT_MPU6050";
static i2c_master_dev_handle_t device_handle;
static float gyro_bias_raw[3];

static bool write_register(uint8_t reg, uint8_t value)
{
    const uint8_t packet[] = {reg, value};
    return i2c_master_transmit(device_handle, packet, sizeof(packet),
                               MPU6050_I2C_TIMEOUT_MS) == ESP_OK;
}

static bool read_registers(uint8_t reg, uint8_t *data, size_t length)
{
    return i2c_master_transmit_receive(device_handle, &reg, sizeof(reg), data,
                                       length, MPU6050_I2C_TIMEOUT_MS) == ESP_OK;
}

static int16_t decode_i16_be(const uint8_t *bytes)
{
    const uint16_t bits = ((uint16_t)bytes[0] << 8) | bytes[1];
    return (int16_t)bits;
}

static bool calibrate_gyro(void)
{
    int64_t sums[3] = {0, 0, 0};
    uint8_t bytes[6];
    for (unsigned sample = 0; sample < MPU6050_CALIBRATION_SAMPLES; ++sample) {
        if (!read_registers(MPU6050_REG_GYRO_XOUT_H, bytes, sizeof(bytes))) {
            return false;
        }
        for (unsigned axis = 0; axis < 3; ++axis) {
            sums[axis] += decode_i16_be(&bytes[axis * 2]);
        }
        TickType_t delay_ticks = pdMS_TO_TICKS(MPU6050_CALIBRATION_PERIOD_MS);
        vTaskDelay(delay_ticks > 0 ? delay_ticks : 1);
    }
    for (unsigned axis = 0; axis < 3; ++axis) {
        gyro_bias_raw[axis] = (float)sums[axis] /
                              (float)MPU6050_CALIBRATION_SAMPLES;
    }
    ESP_LOGI(TAG, "Startup gyro bias raw: x=%.1f y=%.1f z=%.1f",
             gyro_bias_raw[0], gyro_bias_raw[1], gyro_bias_raw[2]);
    return true;
}

bool cube_mpu6050_init(void)
{
    i2c_master_bus_handle_t bus = NULL;
    if (cube_sensor_bus_get(&bus) != ESP_OK) {
        ESP_LOGE(TAG, "Shared sensor I2C bus initialization failed");
        return false;
    }

    const i2c_device_config_t device_config = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = CONFIG_CUBESAT_MPU6050_ADDRESS,
        .scl_speed_hz = MPU6050_I2C_HZ,
    };
    if (i2c_master_bus_add_device(bus, &device_config,
                                  &device_handle) != ESP_OK) {
        ESP_LOGE(TAG, "MPU-6050 I2C device registration failed");
        return false;
    }

    uint8_t identity = 0;
    if (!read_registers(MPU6050_REG_WHO_AM_I, &identity, 1) ||
        (identity & 0x7Eu) != MPU6050_WHO_AM_I_VALUE) {
        ESP_LOGE(TAG, "MPU-6050 not found (WHO_AM_I=0x%02x)", identity);
        return false;
    }

    /* Wake the device and use the X gyro PLL as the clock reference. */
    if (!write_register(MPU6050_REG_PWR_MGMT_1, 0x01u) ||
        !write_register(MPU6050_REG_CONFIG, 0x03u) ||
        !write_register(MPU6050_REG_SMPLRT_DIV, 4u) ||
        !write_register(MPU6050_REG_GYRO_CONFIG, 0x00u) ||
        !write_register(MPU6050_REG_ACCEL_CONFIG, 0x00u)) {
        ESP_LOGE(TAG, "MPU-6050 configuration failed");
        return false;
    }

    vTaskDelay(pdMS_TO_TICKS(100));
    if (!calibrate_gyro()) {
        ESP_LOGE(TAG, "MPU-6050 startup gyro calibration failed");
        return false;
    }
    ESP_LOGI(TAG, "MPU-6050 ready at I2C address 0x%02x",
             CONFIG_CUBESAT_MPU6050_ADDRESS);
    return true;
}

bool cube_mpu6050_read(cube_mpu6050_sample_t *sample)
{
    if (sample == NULL || device_handle == NULL) {
        return false;
    }
    uint8_t bytes[14];
    if (!read_registers(MPU6050_REG_ACCEL_XOUT_H, bytes, sizeof(bytes))) {
        return false;
    }
    /* DLPF_CFG=3 contributes about 4.8 ms gyro / 4.9 ms accel delay. With
     * SMPLRT_DIV=4 and DLPF enabled, output registers update at 200 Hz, so the newest
     * register sample can also be nearly 5 ms old when this read completes.
     * The MPU has no per-sample timestamp here; subtract both terms as a
     * conservative effective-sample-time estimate for freshness checks. */
    const int64_t sampled_at_us = esp_timer_get_time() -
        MPU6050_FILTER_DELAY_US - MPU6050_OUTPUT_PERIOD_US;
    for (unsigned axis = 0; axis < 3; ++axis) {
        const int16_t accel_raw = decode_i16_be(&bytes[axis * 2]);
        const int16_t gyro_raw = decode_i16_be(&bytes[8 + axis * 2]);
        sample->accel_m_s2[axis] = ((float)accel_raw / MPU6050_ACCEL_LSB_PER_G) *
                                   STANDARD_GRAVITY_M_S2;
        sample->gyro_rad_s[axis] =
            (((float)gyro_raw - gyro_bias_raw[axis]) / MPU6050_GYRO_LSB_PER_DPS) *
            DEGREES_TO_RADIANS;
    }
    sample->sampled_at_us = sampled_at_us;
    return true;
}
