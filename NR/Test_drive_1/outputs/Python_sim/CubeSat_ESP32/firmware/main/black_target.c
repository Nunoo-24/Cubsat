#include "black_target.h"

#include <math.h>
#include <string.h>

#define CUBE_MAX_IMAGE_WIDTH 640u
#define CUBE_MAX_IMAGE_HEIGHT 480u

bool cube_decode_rle1_grayscale(const uint8_t *encoded, size_t encoded_length,
                                uint8_t *gray, size_t expected_pixels)
{
    static const uint8_t magic[] = {'R', 'L', 'E', '1'};
    const size_t header_size = sizeof(magic);
    const size_t run_size = 3u;
    if (encoded == NULL || gray == NULL || expected_pixels == 0 ||
        encoded_length < header_size ||
        encoded_length > CUBE_RLE1_MAX_BYTES ||
        memcmp(encoded, magic, header_size) != 0 ||
        (encoded_length - header_size) % run_size != 0) {
        return false;
    }

    size_t produced = 0;
    for (size_t offset = header_size; offset < encoded_length;
         offset += run_size) {
        const size_t run_length = (size_t)encoded[offset] |
            ((size_t)encoded[offset + 1u] << 8);
        if (run_length == 0 || run_length > expected_pixels - produced) {
            return false;
        }
        memset(gray + produced, encoded[offset + 2u], run_length);
        produced += run_length;
    }
    return produced == expected_pixels;
}

bool cube_detect_black_target(const uint8_t *gray, size_t width, size_t height,
                              size_t stride,
                              const cube_target_detector_config_t *config,
                              cube_target_detection_t *result)
{
    if (gray == NULL || config == NULL || result == NULL ||
        width == 0 || height == 0 || width > CUBE_MAX_IMAGE_WIDTH ||
        height > CUBE_MAX_IMAGE_HEIGHT || stride < width ||
        config->min_dark_pixels == 0 ||
        config->max_dark_pixels < config->min_dark_pixels ||
        config->max_dark_pixels > width * height) {
        return false;
    }

    memset(result, 0, sizeof(*result));
    uint64_t sum_x = 0;
    uint64_t sum_y = 0;
    uint32_t raw_count = 0;
    uint32_t coherent_count = 0;
    for (size_t y = 0; y < height; ++y) {
        const uint8_t *row = gray + y * stride;
        for (size_t x = 0; x < width; ++x) {
            if (row[x] > config->threshold) {
                continue;
            }
            ++raw_count;
            uint32_t neighbours = 0;
            const size_t y_start = y == 0 ? 0 : y - 1u;
            const size_t y_end = y + 1u < height ? y + 1u : y;
            const size_t x_start = x == 0 ? 0 : x - 1u;
            const size_t x_end = x + 1u < width ? x + 1u : x;
            for (size_t ny = y_start; ny <= y_end; ++ny) {
                const uint8_t *neighbour_row = gray + ny * stride;
                for (size_t nx = x_start; nx <= x_end; ++nx) {
                    if ((nx != x || ny != y) &&
                        neighbour_row[nx] <= config->threshold) {
                        ++neighbours;
                    }
                }
            }
            if (neighbours >= 2u) {
                sum_x += x;
                sum_y += y;
                ++coherent_count;
            }
        }
    }

    result->dark_pixel_count = raw_count;
    result->coherent_pixel_count = coherent_count;
    if (coherent_count < config->min_dark_pixels ||
        coherent_count > config->max_dark_pixels) {
        return true;
    }
    result->centroid_x_px = (float)sum_x / (float)coherent_count;
    result->centroid_y_px = (float)sum_y / (float)coherent_count;
    result->error_x_px = result->centroid_x_px - ((float)width - 1.0f) * 0.5f;
    result->error_y_px = result->centroid_y_px - ((float)height - 1.0f) * 0.5f;
    result->valid = isfinite(result->error_x_px) && isfinite(result->error_y_px);
    return true;
}
