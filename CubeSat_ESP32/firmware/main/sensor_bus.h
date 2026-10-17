#pragma once

#include "esp_err.h"
#include "driver/i2c_master.h"

/* Shared XIAO D4/GPIO5 SDA and D5/GPIO6 SCL bus for onboard sensors. */
esp_err_t cube_sensor_bus_get(i2c_master_bus_handle_t *bus);
