#include "black_target.h"

#include <assert.h>
#include <stdint.h>
#include <string.h>

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
    assert(cube_detect_black_target(frame, WIDTH, HEIGHT, STRIDE,
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
    assert(cube_detect_black_target(frame, WIDTH, HEIGHT, STRIDE,
                                   &config, &result));
    assert(!result.valid && result.dark_pixel_count == 64 &&
           result.coherent_pixel_count == 0);

    memset(frame, 190, sizeof(frame));
    for (size_t y = 115; y < 125; ++y) {
        for (size_t x = 155; x < 165; ++x) {
            frame[y * STRIDE + x] = 0;
        }
    }
    assert(cube_detect_black_target(frame, WIDTH, HEIGHT, STRIDE,
                                   &config, &result));
    assert(result.valid && result.dark_pixel_count == 100 &&
           result.coherent_pixel_count == 100);
    assert(result.error_x_px == 0.0f && result.error_y_px == 0.0f);

    memset(frame, 255, sizeof(frame));
    assert(cube_detect_black_target(frame, WIDTH, HEIGHT, STRIDE,
                                   &config, &result));
    assert(!result.valid);

    assert(!cube_detect_black_target(frame, WIDTH, HEIGHT, WIDTH - 1,
                                    &config, &result));
    cube_target_detector_config_t oversized_area = config;
    oversized_area.max_dark_pixels = WIDTH * HEIGHT + 1u;
    assert(!cube_detect_black_target(frame, WIDTH, HEIGHT, STRIDE,
                                    &oversized_area, &result));

    return 0;
}
