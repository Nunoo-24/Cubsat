#include "sensor_bus.h"

#include "driver/gpio.h"

static i2c_master_bus_handle_t shared_bus;

esp_err_t cube_sensor_bus_get(i2c_master_bus_handle_t *bus)
{
    if (bus == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    if (shared_bus == NULL) {
        const i2c_master_bus_config_t config = {
            .clk_source = I2C_CLK_SRC_DEFAULT,
            .i2c_port = I2C_NUM_0,
            .sda_io_num = GPIO_NUM_5,
            .scl_io_num = GPIO_NUM_6,
            .glitch_ignore_cnt = 7,
            .flags.enable_internal_pullup = false,
        };
        const esp_err_t result = i2c_new_master_bus(&config, &shared_bus);
        if (result != ESP_OK) {
            return result;
        }
    }
    *bus = shared_bus;
    return ESP_OK;
}
