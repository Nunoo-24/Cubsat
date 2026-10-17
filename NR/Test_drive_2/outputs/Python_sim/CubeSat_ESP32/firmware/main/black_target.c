#include "black_target.h"

#include <math.h>
#include <string.h>

bool cube_decode_rle1_grayscale(const uint8_t *encoded, size_t encoded_length,
                                uint8_t *gray, size_t expected_pixels)
{
    static const uint8_t magic[] = {'R', 'L', 'E', '1'};
    const size_t header_size = sizeof(magic);
    const size_t run_size = 3u;
    if (encoded == NULL || gray == NULL || expected_pixels == 0 ||
        expected_pixels > CUBE_RLE1_MAX_PIXELS ||
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

bool cube_detect_black_target(const uint8_t *gray, size_t gray_length,
                              size_t width, size_t height, size_t stride,
                              const cube_target_detector_config_t *config,
                              cube_target_detection_t *result)
{
    if (gray == NULL || config == NULL || result == NULL ||
        width == 0 || height == 0 ||
        width > CUBE_TARGET_MAX_IMAGE_WIDTH ||
        height > CUBE_TARGET_MAX_IMAGE_HEIGHT || stride < width ||
        stride > CUBE_TARGET_MAX_IMAGE_WIDTH ||
        gray_length < stride * height ||
        config->min_dark_pixels == 0 ||
        config->max_dark_pixels < config->min_dark_pixels ||
        config->max_dark_pixels > width * height) {
        return false;
    }

    memset(result, 0, sizeof(*result));
    uint64_t sum_x = 0;
    uint64_t sum_y = 0;
    uint64_t sum_xx = 0, sum_yy = 0, sum_xy = 0;
    uint64_t sum_xxx = 0, sum_yyy = 0, sum_xxy = 0, sum_xyy = 0;
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
                sum_xx += (uint64_t)x * x;
                sum_yy += (uint64_t)y * y;
                sum_xy += (uint64_t)x * y;
                sum_xxx += (uint64_t)x * x * x;
                sum_yyy += (uint64_t)y * y * y;
                sum_xxy += (uint64_t)x * x * y;
                sum_xyy += (uint64_t)x * y * y;
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
    const double n = (double)coherent_count;
    const double cx = (double)sum_x / n;
    const double cy = (double)sum_y / n;
    const double cov_xx = (double)sum_xx / n - cx * cx;
    const double cov_yy = (double)sum_yy / n - cy * cy;
    const double cov_xy = (double)sum_xy / n - cx * cy;
    const double trace = cov_xx + cov_yy;
    const double anisotropy = hypot(cov_xx - cov_yy, 2.0 * cov_xy);
    result->centroid_x_px = (float)cx;
    result->centroid_y_px = (float)cy;
    result->error_x_px = result->centroid_x_px - ((float)width - 1.0f) * 0.5f;
    result->error_y_px = result->centroid_y_px - ((float)height - 1.0f) * 0.5f;
    result->roll_orientation_valid = false;
    result->roll_orientation_rad = 0.0f;
    if (trace > 0.0 && anisotropy / trace >= 0.18) {
        double angle = 0.5 * atan2(2.0 * cov_xy, cov_xx - cov_yy);
        const double m30 = (double)sum_xxx - 3.0 * cx * (double)sum_xx +
                           2.0 * n * cx * cx * cx;
        const double m03 = (double)sum_yyy - 3.0 * cy * (double)sum_yy +
                           2.0 * n * cy * cy * cy;
        const double m21 = (double)sum_xxy - cy * (double)sum_xx -
                           2.0 * cx * (double)sum_xy + n * cx * cx * cy +
                           cx * cx * (double)sum_y;
        const double m12 = (double)sum_xyy - cx * (double)sum_yy -
                           2.0 * cy * (double)sum_xy + n * cx * cy * cy +
                           cy * cy * (double)sum_x;
        const double c = cos(angle), sn = sin(angle);
        const double skew = c * c * c * m30 + 3.0 * c * c * sn * m21 +
                            3.0 * c * sn * sn * m12 + sn * sn * sn * m03;
        if (fabs(skew) > 0.05) {
            if (skew < 0.0) angle += 3.14159265358979323846;
            /* The directed target axis points toward its smaller satellite;
             * skew points to the larger end, so reverse it. */
            angle += 3.14159265358979323846;
            result->roll_orientation_rad = (float)atan2(sin(angle), cos(angle));
            result->roll_orientation_valid = isfinite(result->roll_orientation_rad);
        }
    }
    result->valid = isfinite(result->error_x_px) && isfinite(result->error_y_px);
    return true;
}
