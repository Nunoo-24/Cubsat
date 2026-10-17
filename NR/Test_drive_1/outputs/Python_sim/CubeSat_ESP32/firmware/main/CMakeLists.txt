set(cubesat_sources "cubesat_main.c" "yaw_controller.c" "attitude_estimator.c"
                    "black_target.c")
set(cubesat_requirements esp_http_client esp_wifi esp_event esp_netif
                         esp_timer nvs_flash)
if(CONFIG_CUBESAT_MPU6050_ENABLED OR CONFIG_CUBESAT_LIS3MDL_ENABLED)
    list(APPEND cubesat_sources "sensor_bus.c")
    list(APPEND cubesat_requirements esp_driver_i2c esp_driver_gpio)
endif()
if(CONFIG_CUBESAT_CAMERA_ENABLED)
    list(APPEND cubesat_sources "xiao_camera.c")
endif()
if(CONFIG_CUBESAT_MPU6050_ENABLED)
    list(APPEND cubesat_sources "mpu6050.c")
endif()
if(CONFIG_CUBESAT_LIS3MDL_ENABLED)
    list(APPEND cubesat_sources "lis3mdl.c")
endif()
if(CONFIG_CUBESAT_MOTEUS_ENABLED)
    list(APPEND cubesat_sources "moteus_uart.c")
    list(APPEND cubesat_requirements esp_driver_uart esp_driver_gpio)
endif()

idf_component_register(SRCS ${cubesat_sources}
                       REQUIRES ${cubesat_requirements}
                       INCLUDE_DIRS "")
