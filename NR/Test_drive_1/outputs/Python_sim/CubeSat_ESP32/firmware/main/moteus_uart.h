#pragma once

#include <stdbool.h>

typedef struct {
    bool valid;
    int mode;
    int fault;
    float position_rev;
    float velocity_rev_s;
    float motor_torque_nm;
    float commanded_motor_torque_nm;
} cube_moteus_status_t;

/* Initialize only after physical sensor adapters are online and the arm input
 * is asserted. The UART protocol path is compile-time disabled by default. */
bool cube_moteus_uart_init(void);
bool cube_moteus_arm_active(void);

/* Sends a bounded motor-side torque and requires a fresh status/fault reply. */
bool cube_moteus_command_torque(float motor_torque_nm,
                                cube_moteus_status_t *status);

/* Native moteus velocity mode for the bounded commissioning state, accepted
 * only in SAFE/STANDBY. Target and hard cap are in rev/s (50 rev/s = 3000 rpm). */
bool cube_moteus_command_velocity(float target_rev_s,
                                  cube_moteus_status_t *status);
