#include "black_target.h"

#include <assert.h>
#include <stdint.h>
#include <math.h>
#include <string.h>

static void draw_directed_fiducial(uint8_t *frame, size_t stride,
                                   double angle)
{
    memset(frame, 190, stride * 240u);
    const double radius = 5.2;
    const double small_radius = 1.664;
    const double large_radius = 2.392;
    const double offset = radius * 1.55;
    const double correction = (large_radius * large_radius -
        small_radius * small_radius) * offset / (radius * radius);
    const double cx = 159.5, cy = 119.5;
    const double disks[3][3] = {
        {cx + correction * cos(angle), cy + correction * sin(angle), radius},
        {cx + offset * cos(angle), cy + offset * sin(angle), small_radius},
        {cx - offset * cos(angle), cy - offset * sin(angle), large_radius},
    };
    for (size_t disk = 0; disk < 3; ++disk) {
        for (int y = 110; y < 130; ++y) {
            for (int x = 145; x < 175; ++x) {
                const double dx = x - disks[disk][0];
                const double dy = y - disks[disk][1];
                if (dx * dx + dy * dy <= disks[disk][2] * disks[disk][2]) {
                    frame[(size_t)y * stride + (size_t)x] = 12;
                }
            }
        }
    }
}

int main(void)
{
    const uint8_t encoded[] = {
        'R', 'L', 'E', '1', 3, 0, 12, 2, 0, 190,
    };
    uint8_t decoded[5] = {0};
    assert(cube_decode_rle1_grayscale(encoded, sizeof(encoded), decoded,
                                      sizeof(decoded)));
    assert(decoded[0] == 12 && decoded[2] == 12 &&
           decoded[3] == 190 && decoded[4] == 190);
    const uint8_t zero_run[] = {'R', 'L', 'E', '1', 0, 0, 12};
    const uint8_t oversized_run[] = {'R', 'L', 'E', '1', 6, 0, 12};
    const uint8_t partial_run[] = {'R', 'L', 'E', '1', 1, 0};
    assert(!cube_decode_rle1_grayscale(zero_run, sizeof(zero_run), decoded,
                                       sizeof(decoded)));
    assert(!cube_decode_rle1_grayscale(oversized_run, sizeof(oversized_run),
                                       decoded, sizeof(decoded)));
    assert(!cube_decode_rle1_grayscale(partial_run, sizeof(partial_run),
                                       decoded, sizeof(decoded)));
    uint8_t untouched[5] = {1, 2, 3, 4, 5};
    assert(!cube_decode_rle1_grayscale(
        encoded, sizeof(encoded), untouched, CUBE_RLE1_MAX_PIXELS + 1u));
    assert(untouched[0] == 1 && untouched[1] == 2 && untouched[2] == 3 &&
           untouched[3] == 4 && untouched[4] == 5);

    enum { WIDTH = 320, HEIGHT = 240, STRIDE = 324 };
    uint8_t frame[STRIDE * HEIGHT];
    memset(frame, 255, sizeof(frame));
    for (int y = 95; y < 105; ++y) {
        for (int x = 170; x < 180; ++x) {
            frame[y * STRIDE + x] = 0;
        }
    }
    const cube_target_detector_config_t config = {
        .threshold = 40,
        .min_dark_pixels = 50,
        .max_dark_pixels = 200,
    };
    cube_target_detection_t result;
    assert(!cube_detect_black_target(frame, sizeof(frame) - 1u,
                                    WIDTH, HEIGHT, STRIDE,
                                    &config, &result));
    assert(cube_detect_black_target(frame, sizeof(frame), WIDTH, HEIGHT, STRIDE,
                                   &config, &result));
    assert(result.valid);
    assert(result.dark_pixel_count == 100);
    assert(result.error_x_px == 15.0f);
    assert(result.error_y_px == -20.0f);

    memset(frame, 190, sizeof(frame));
    for (size_t y = 2; y < 32; y += 4) {
        for (size_t x = 2; x < 32; x += 4) {
            frame[y * STRIDE + x] = 0;
        }
    }
    assert(cube_detect_black_target(frame, sizeof(frame), WIDTH, HEIGHT, STRIDE,
                                   &config, &result));
    assert(!result.valid && result.dark_pixel_count == 64 &&
           result.coherent_pixel_count == 0);

    memset(frame, 190, sizeof(frame));
    for (size_t y = 115; y < 125; ++y) {
        for (size_t x = 155; x < 165; ++x) {
            frame[y * STRIDE + x] = 0;
        }
    }
    assert(cube_detect_black_target(frame, sizeof(frame), WIDTH, HEIGHT, STRIDE,
                                   &config, &result));
    assert(result.valid && result.dark_pixel_count == 100 &&
           result.coherent_pixel_count == 100);
    assert(result.error_x_px == 0.0f && result.error_y_px == 0.0f);
    assert(!result.roll_orientation_valid);

    /* Three dark disks form a directed fiducial: small end at +X, large at -X.
     * Sweep both signs so image angle and camera-roll sign cannot regress. */
    const double marker_angles[] = {
        -0.5235987755982988, -0.2617993877991494, 0.0,
        0.2617993877991494, 0.5235987755982988,
    };
    for (size_t i = 0; i < sizeof(marker_angles) / sizeof(marker_angles[0]); ++i) {
        draw_directed_fiducial(frame, STRIDE, marker_angles[i]);
        assert(cube_detect_black_target(frame, sizeof(frame), WIDTH, HEIGHT, STRIDE,
                                       &config, &result));
        assert(result.valid && result.roll_orientation_valid);
        assert(fabsf(result.roll_orientation_rad - (float)marker_angles[i]) < 0.05f);
        assert(fabsf(result.error_x_px) < 0.5f &&
               fabsf(result.error_y_px) < 0.5f);
    }

    memset(frame, 255, sizeof(frame));
    assert(cube_detect_black_target(frame, sizeof(frame), WIDTH, HEIGHT, STRIDE,
                                   &config, &result));
    assert(!result.valid);

    assert(!cube_detect_black_target(frame, sizeof(frame), WIDTH, HEIGHT, WIDTH - 1,
                                    &config, &result));
    assert(!cube_detect_black_target(frame, sizeof(frame), WIDTH, HEIGHT,
                                    CUBE_TARGET_MAX_IMAGE_WIDTH + 1u,
                                    &config, &result));
    cube_target_detector_config_t oversized_area = config;
    oversized_area.max_dark_pixels = WIDTH * HEIGHT + 1u;
    assert(!cube_detect_black_target(frame, sizeof(frame), WIDTH, HEIGHT, STRIDE,
                                    &oversized_area, &result));

    return 0;
}
