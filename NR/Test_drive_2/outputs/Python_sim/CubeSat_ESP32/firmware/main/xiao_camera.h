#ifndef AE_XIAO_CAMERA_H
#define AE_XIAO_CAMERA_H

#include <stdbool.h>
#include <stdint.h>

typedef struct {
    bool valid;
    float error_x_px;
    float error_y_px;
    float focal_length_x_px;
    unsigned width_px;
    int64_t sampled_at_us;
} cube_camera_observation_t;

/* Initialize the XIAO ESP32-S3 Sense DVP camera. */
bool cube_xiao_camera_init(void);

/* Capture one grayscale QVGA frame and locate the configured dark target. */
bool cube_xiao_camera_observe(cube_camera_observation_t *observation);
/* Copy the latest 64x48 grayscale thumbnail; returns false before first frame. */
bool cube_xiao_camera_copy_preview(unsigned char *destination, unsigned capacity);

#endif
