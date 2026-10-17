#include "xiao_camera.h"

#include "black_target.h"

#include <string.h>

#include "esp_camera.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "sdkconfig.h"

#define CAMERA_WIDTH_PX 320u
#define CAMERA_HEIGHT_PX 240u
#define PREVIEW_WIDTH_PX 64u
#define PREVIEW_HEIGHT_PX 48u
#define PREVIEW_BYTES (PREVIEW_WIDTH_PX * PREVIEW_HEIGHT_PX)

static const char *TAG = "CUBESAT_CAMERA";
static unsigned char preview_gray[PREVIEW_BYTES];
static bool preview_available;

static const camera_config_t camera_config = {
    .pin_pwdn = -1,
    .pin_reset = -1,
    .pin_xclk = 10,
    .pin_sccb_sda = 40,
    .pin_sccb_scl = 39,
    .pin_d7 = 48,
    .pin_d6 = 11,
    .pin_d5 = 12,
    .pin_d4 = 14,
    .pin_d3 = 16,
    .pin_d2 = 18,
    .pin_d1 = 17,
    .pin_d0 = 15,
    .pin_vsync = 38,
    .pin_href = 47,
    .pin_pclk = 13,
    .xclk_freq_hz = 20000000,
    .ledc_timer = LEDC_TIMER_0,
    .ledc_channel = LEDC_CHANNEL_0,
    .pixel_format = PIXFORMAT_GRAYSCALE,
    .frame_size = FRAMESIZE_QVGA,
    .jpeg_quality = 12,
    .fb_count = 1,
    .fb_location = CAMERA_FB_IN_PSRAM,
    .grab_mode = CAMERA_GRAB_WHEN_EMPTY,
};

bool cube_xiao_camera_init(void)
{
    const esp_err_t result = esp_camera_init(&camera_config);
    if (result != ESP_OK) {
        ESP_LOGE(TAG, "Camera initialization failed: %s", esp_err_to_name(result));
        return false;
    }
    ESP_LOGI(TAG, "Grayscale QVGA camera initialized");
    return true;
}

bool cube_xiao_camera_observe(cube_camera_observation_t *observation)
{
    if (observation == NULL) {
        return false;
    }
    memset(observation, 0, sizeof(*observation));
    observation->width_px = CAMERA_WIDTH_PX;
    observation->focal_length_x_px = CONFIG_CUBESAT_CAMERA_FOCAL_LENGTH_PX;

    camera_fb_t *frame = esp_camera_fb_get();
    if (frame == NULL) {
        ESP_LOGW(TAG, "Frame capture failed");
        return false;
    }
    const int64_t frame_time_us = (int64_t)frame->timestamp.tv_sec * 1000000LL +
                                  (int64_t)frame->timestamp.tv_usec;
    const int64_t now_us = esp_timer_get_time();
    if (frame_time_us <= 0 || frame_time_us > now_us) {
        ESP_LOGW(TAG, "Frame has invalid capture timestamp");
        esp_camera_fb_return(frame);
        return false;
    }
    observation->sampled_at_us = frame_time_us;

    bool success = false;
    if (frame->format == PIXFORMAT_GRAYSCALE &&
        frame->width == CAMERA_WIDTH_PX && frame->height == CAMERA_HEIGHT_PX &&
        frame->len >= CAMERA_WIDTH_PX * CAMERA_HEIGHT_PX) {
        const cube_target_detector_config_t detector = {
            .threshold = CONFIG_CUBESAT_CAMERA_THRESHOLD,
            .min_dark_pixels = CONFIG_CUBESAT_CAMERA_MIN_DARK_PIXELS,
            .max_dark_pixels = CONFIG_CUBESAT_CAMERA_MAX_DARK_PIXELS,
        };
        cube_target_detection_t result;
        success = cube_detect_black_target(frame->buf, frame->width, frame->height,
                                           frame->width, &detector, &result);
        for (unsigned y = 0; y < PREVIEW_HEIGHT_PX; ++y) {
            for (unsigned x = 0; x < PREVIEW_WIDTH_PX; ++x) {
                unsigned sum = 0;
                for (unsigned dy = 0; dy < 5; ++dy) {
                    for (unsigned dx = 0; dx < 5; ++dx) {
                        sum += frame->buf[(y * 5 + dy) * CAMERA_WIDTH_PX + x * 5 + dx];
                    }
                }
                preview_gray[y * PREVIEW_WIDTH_PX + x] = (unsigned char)(sum / 25);
            }
        }
        preview_available = true;
        if (success) {
            observation->valid = result.valid;
            observation->error_x_px = result.error_x_px;
            observation->error_y_px = result.error_y_px;
        }
    }
    esp_camera_fb_return(frame);
    return success;
}

bool cube_xiao_camera_copy_preview(unsigned char *destination, unsigned capacity)
{
    if (destination == NULL || capacity < PREVIEW_BYTES || !preview_available) {
        return false;
    }
    memcpy(destination, preview_gray, PREVIEW_BYTES);
    return true;
}
