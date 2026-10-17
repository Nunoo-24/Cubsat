#pragma once

#include <stdbool.h>
#include <stdint.h>

typedef struct {
    float magnetic_field_uT[3];
    int64_t sampled_at_us;
} cube_lis3mdl_sample_t;

bool cube_lis3mdl_init(void);
bool cube_lis3mdl_read(cube_lis3mdl_sample_t *sample);
