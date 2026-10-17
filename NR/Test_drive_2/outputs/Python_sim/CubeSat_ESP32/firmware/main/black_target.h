#ifndef AE_BLACK_TARGET_H
#define AE_BLACK_TARGET_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#define CUBE_RLE1_MAX_BYTES 8192u

typedef struct {
    uint8_t threshold;
    uint32_t min_dark_pixels;
    uint32_t max_dark_pixels;
} cube_target_detector_config_t;

typedef struct {
    bool valid;
    /* All pixels at/below threshold, before the local-coherence filter. */
    uint32_t dark_pixel_count;
    /* Pixels retained by the 8-neighbour rule; area limits use this count. */
    uint32_t coherent_pixel_count;
    float centroid_x_px;
    float centroid_y_px;
    float error_x_px;
    float error_y_px;
} cube_target_detection_t;

/*
 * Compute the centroid of coherent thresholded dark pixels in a grayscale
 * frame. A dark pixel is retained only when at least two other dark pixels
 * occur in its 8-neighbourhood, rejecting isolated threshold noise. This is
 * not full connected-component segmentation; configure ROI and area bounds
 * from the actual camera image. Border-clipped masks can bias the centroid.
 */
bool cube_detect_black_target(const uint8_t *gray, size_t width, size_t height,
                              size_t stride,
                              const cube_target_detector_config_t *config,
                              cube_target_detection_t *result);

/* Decode RLE1: ASCII magic followed by little-endian u16 run + gray byte. */
bool cube_decode_rle1_grayscale(const uint8_t *encoded, size_t encoded_length,
                                uint8_t *gray, size_t expected_pixels);

#endif
