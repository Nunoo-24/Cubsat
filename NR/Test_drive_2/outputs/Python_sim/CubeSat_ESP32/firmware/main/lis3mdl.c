#include "lis3mdl.h"

#include <stddef.h>

#include "driver/i2c_master.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "sensor_bus.h"
#include "sdkconfig.h"

#define LIS3MDL_WHO_AM_I_REG 0x0Fu
#define LIS3MDL_WHO_AM_I_VALUE 0x3Du
#define LIS3MDL_CTRL_REG1 0x20u
#define LIS3MDL_CTRL_REG2 0x21u
#define LIS3MDL_CTRL_REG3 0x22u
#define LIS3MDL_CTRL_REG4 0x23u
#define LIS3MDL_CTRL_REG5 0x24u
#define LIS3MDL_STATUS_REG 0x27u
#define LIS3MDL_OUT_X_L 0x28u
#define LIS3MDL_I2C_HZ 100000u
#define LIS3MDL_TIMEOUT_MS 100
#define LIS3MDL_ODR_PERIOD_US 12500
#define LIS3MDL_MAX_SAMPLE_AGE_US 100000

#if CONFIG_CUBESAT_LIS3MDL_FS_8G
#define LIS3MDL_CTRL_REG2_FS 0x20u
#define LIS3MDL_RANGE_G 8u
#define LIS3MDL_MICROTESLA_PER_LSB (100.0f / 3421.0f)
#elif CONFIG_CUBESAT_LIS3MDL_FS_12G
#define LIS3MDL_CTRL_REG2_FS 0x40u
#define LIS3MDL_RANGE_G 12u
#define LIS3MDL_MICROTESLA_PER_LSB (100.0f / 2281.0f)
#elif CONFIG_CUBESAT_LIS3MDL_FS_16G
#define LIS3MDL_CTRL_REG2_FS 0x60u
#define LIS3MDL_RANGE_G 16u
#define LIS3MDL_MICROTESLA_PER_LSB (100.0f / 1711.0f)
#else
#define LIS3MDL_CTRL_REG2_FS 0x00u
#define LIS3MDL_RANGE_G 4u
#define LIS3MDL_MICROTESLA_PER_LSB (100.0f / 6842.0f)
#endif

static const char *TAG = "CUBESAT_LIS3MDL";
static i2c_master_dev_handle_t device_handle;
static cube_lis3mdl_sample_t latest_sample;
static bool have_sample;

uint8_t cube_lis3mdl_full_scale_gauss(void)
{
    return (uint8_t)LIS3MDL_RANGE_G;
}

static bool write_register(uint8_t reg, uint8_t value)
{
    const uint8_t packet[] = {reg, value};
    return i2c_master_transmit(device_handle, packet, sizeof(packet),
                               LIS3MDL_TIMEOUT_MS) == ESP_OK;
}

static bool read_registers(uint8_t reg, uint8_t *data, size_t length)
{
    return i2c_master_transmit_receive(device_handle, &reg, sizeof(reg), data,
                                       length, LIS3MDL_TIMEOUT_MS) == ESP_OK;
}

static int16_t decode_i16_le(const uint8_t *bytes)
{
    const uint16_t bits = (uint16_t)bytes[0] | ((uint16_t)bytes[1] << 8);
    return (int16_t)bits;
}

bool cube_lis3mdl_init(void)
{
    i2c_master_bus_handle_t bus = NULL;
    if (cube_sensor_bus_get(&bus) != ESP_OK) {
        ESP_LOGE(TAG, "Shared sensor I2C bus initialization failed");
        return false;
    }
#if CONFIG_CUBESAT_LIS3MDL_ADDRESS_1E
    const uint16_t address = 0x1Eu;
#else
    const uint16_t address = 0x1Cu;
#endif
    const i2c_device_config_t device_config = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = address,
        .scl_speed_hz = LIS3MDL_I2C_HZ,
    };
    if (i2c_master_bus_add_device(bus, &device_config, &device_handle) != ESP_OK) {
        ESP_LOGE(TAG, "LIS3MDL I2C device registration failed");
        return false;
    }

    uint8_t identity = 0;
    if (!read_registers(LIS3MDL_WHO_AM_I_REG, &identity, 1) ||
        identity != LIS3MDL_WHO_AM_I_VALUE) {
        ESP_LOGE(TAG, "LIS3MDL not found (WHO_AM_I=0x%02x)", identity);
        return false;
    }

    /* 80 Hz, high XY/Z performance, continuous conversion, selected FS, BDU. */
    if (!write_register(LIS3MDL_CTRL_REG1, 0xFCu) ||
        !write_register(LIS3MDL_CTRL_REG2, LIS3MDL_CTRL_REG2_FS) ||
        !write_register(LIS3MDL_CTRL_REG3, 0x00u) ||
        !write_register(LIS3MDL_CTRL_REG4, 0x0Cu) ||
        !write_register(LIS3MDL_CTRL_REG5, 0x40u)) {
        ESP_LOGE(TAG, "LIS3MDL configuration failed");
        return false;
    }
    ESP_LOGI(TAG, "LIS3MDL ready at I2C address 0x%02x, range +/- %u G",
             address, LIS3MDL_RANGE_G);
    return true;
}

bool cube_lis3mdl_read(cube_lis3mdl_sample_t *sample)
{
    if (sample == NULL || device_handle == NULL) {
        return false;
    }
    uint8_t status = 0;
    if (read_registers(LIS3MDL_STATUS_REG, &status, 1) && (status & 0x08u)) {
        uint8_t bytes[6];
        const uint8_t first_output_register = LIS3MDL_OUT_X_L | 0x80u;
        if (read_registers(first_output_register, bytes, sizeof(bytes))) {
            const int64_t read_time_us = esp_timer_get_time();
            for (unsigned axis = 0; axis < 3; ++axis) {
                latest_sample.magnetic_field_uT[axis] =
                    (float)decode_i16_le(&bytes[axis * 2]) *
                    LIS3MDL_MICROTESLA_PER_LSB;
            }
            /* Conservative upper bound: one complete 80 Hz conversion period. */
            latest_sample.sampled_at_us = read_time_us - LIS3MDL_ODR_PERIOD_US;
            have_sample = true;
        }
    }
    if (!have_sample ||
        esp_timer_get_time() - latest_sample.sampled_at_us >
            LIS3MDL_MAX_SAMPLE_AGE_US) {
        return false;
    }
    *sample = latest_sample;
    return true;
}
