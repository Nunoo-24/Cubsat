#include <inttypes.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"

static const char *TAG = "CUBE_FW";

void app_main(void)
{
    uint32_t heartbeat = 0;

    ESP_LOGI(TAG, "Cube firmware — Level 0 startup complete");

    while (true) {
        ESP_LOGI(
            TAG,
            "heartbeat=%" PRIu32 " | mode=STARTUP | state=NOMINAL",
            heartbeat++
        );

        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}