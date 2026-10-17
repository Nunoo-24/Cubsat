#include "moteus_uart.h"

#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "driver/gpio.h"
#include "driver/uart.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "sdkconfig.h"

#define MOTEUS_UART UART_NUM_1
#define MOTEUS_BAUD 921600
#define MOTEUS_LINE_CAPACITY 256
#define MOTEUS_FRAME_BYTES 64
#define MOTEUS_STATUS_TIMEOUT_MS 40
#define MOTEUS_MAX_COMMAND_VELOCITY_REV_S 50.0f

static const char *TAG = "MOTEUS_UART";
static bool uart_ready;

static uint8_t crc8_97(const uint8_t *data, size_t length)
{
    static const uint8_t table[16] = {
        0x00, 0x97, 0xb9, 0x2e, 0xe5, 0x72, 0x5c, 0xcb,
        0x5d, 0xca, 0xe4, 0x73, 0xb8, 0x2f, 0x01, 0x96,
    };
    uint8_t crc = 0;
    for (size_t i = 0; i < length; ++i) {
        crc = table[((crc >> 4) ^ (data[i] >> 4)) & 0x0f] ^ (uint8_t)(crc << 4);
        crc = table[((crc >> 4) ^ (data[i] & 0x0f)) & 0x0f] ^ (uint8_t)(crc << 4);
    }
    return crc;
}

static bool write_checked_line(const char *line)
{
    char output[MOTEUS_LINE_CAPACITY];
    const size_t length = strlen(line);
    if (length + 7 >= sizeof(output)) return false;
    uint8_t crc_input[MOTEUS_LINE_CAPACITY];
    memcpy(crc_input, line, length);
    crc_input[length] = ' ';
    const uint8_t crc = crc8_97(crc_input, length + 1);
    const int written = snprintf(output, sizeof(output), "%s *%02X\n", line, crc);
    if (written <= 0 || (size_t)written >= sizeof(output)) return false;
    return uart_write_bytes(MOTEUS_UART, output, (size_t)written) == written;
}

static bool read_checked_line(char *line, size_t capacity, int timeout_ms)
{
    size_t used = 0;
    const int64_t deadline = (int64_t)xTaskGetTickCount() * portTICK_PERIOD_MS + timeout_ms;
    while ((int64_t)xTaskGetTickCount() * portTICK_PERIOD_MS < deadline) {
        uint8_t ch;
        const int count = uart_read_bytes(MOTEUS_UART, &ch, 1, pdMS_TO_TICKS(3));
        if (count <= 0) continue;
        if (ch == '\n') {
            while (used > 0 && line[used - 1] == '\r') --used;
            line[used] = '\0';
            char *star = strrchr(line, '*');
            if (star == NULL || strlen(star + 1) != 2) return false;
            char *end = NULL;
            const long expected = strtol(star + 1, &end, 16);
            if (end == star + 1 || *end != '\0' || expected < 0 || expected > 255)
                return false;
            const uint8_t actual = crc8_97((const uint8_t *)line,
                                            (size_t)(star - line));
            if (actual != (uint8_t)expected) return false;
            while (star > line && star[-1] == ' ') --star;
            *star = '\0';
            return true;
        }
        if (used + 1 >= capacity) return false;
        line[used++] = (char)ch;
    }
    return false;
}

static bool command_ok(const char *command, bool accept_already_on)
{
    char line[MOTEUS_LINE_CAPACITY];
    if (!write_checked_line(command)) return false;
    if (!read_checked_line(line, sizeof(line), MOTEUS_STATUS_TIMEOUT_MS)) return false;
    if (strcmp(line, "OK") == 0) return true;
    return accept_already_on && strcmp(line, "ERR already in BusOn") == 0;
}

static void put_float_le(uint8_t *dst, float value)
{
    uint32_t bits;
    memcpy(&bits, &value, sizeof(bits));
    dst[0] = (uint8_t)bits;
    dst[1] = (uint8_t)(bits >> 8);
    dst[2] = (uint8_t)(bits >> 16);
    dst[3] = (uint8_t)(bits >> 24);
}

static float get_float_le(const uint8_t *src)
{
    const uint32_t bits = (uint32_t)src[0] | ((uint32_t)src[1] << 8) |
        ((uint32_t)src[2] << 16) | ((uint32_t)src[3] << 24);
    float value;
    memcpy(&value, &bits, sizeof(value));
    return value;
}

static size_t make_torque_frame(float torque_nm, uint8_t frame[MOTEUS_FRAME_BYTES])
{
    size_t i = 0;
    /* Write Mode=Position first, as required by the moteus register protocol. */
    frame[i++] = 0x01; frame[i++] = 0x00; frame[i++] = 0x0a;

    /* F32 write of registers 0x020..0x02b (12 registers). */
    frame[i++] = 0x08; frame[i++] = 0x0c; frame[i++] = 0x20;
    const float nan_value = NAN;
    const float command[12] = {
        nan_value,                              /* position: stay here */
        0.0f,                                   /* velocity */
        torque_nm,                              /* feedforward torque */
        0.0f, 0.0f,                             /* kp and kd scales */
        CONFIG_CUBESAT_MOTEUS_MAX_TORQUE_MNM / 1000.0f,
        nan_value,                              /* stop position */
        0.1f,                                   /* finite watchdog timeout */
        nan_value, nan_value, nan_value,        /* velocity/accel/voltage overrides */
        0.0f,                                   /* integral limit scale */
    };
    for (size_t j = 0; j < 12; ++j, i += 4) put_float_le(&frame[i], command[j]);

    /* Query mode, fault, and measured position/velocity/torque in the same FD frame. */
    frame[i++] = 0x11; frame[i++] = 0x00;
    frame[i++] = 0x11; frame[i++] = 0x0f;
    frame[i++] = 0x1f; frame[i++] = 0x01;
    while (i < MOTEUS_FRAME_BYTES) frame[i++] = 0x50; /* NOP padding */
    return i;
}

static bool make_velocity_frame(float target_rev_s,
                                uint8_t frame[MOTEUS_FRAME_BYTES])
{
    if (!isfinite(target_rev_s) ||
        fabsf(target_rev_s) > MOTEUS_MAX_COMMAND_VELOCITY_REV_S) {
        return false;
    }
    size_t i = 0;
    /* Position=NaN selects native velocity mode in position mode. */
    frame[i++] = 0x01; frame[i++] = 0x00; frame[i++] = 0x0a;
    /* F32 write of registers 0x020..0x02b (12 registers). */
    frame[i++] = 0x08; frame[i++] = 0x0c; frame[i++] = 0x20;
    const float nan_value = NAN;
    const float command[12] = {
        nan_value,                              /* position: velocity mode */
        target_rev_s,                           /* velocity setpoint (rev/s) */
        0.0f,                                   /* feedforward torque */
        nan_value, nan_value,                   /* retain internal PID scales */
        CONFIG_CUBESAT_MOTEUS_MAX_TORQUE_MNM / 1000.0f,
        nan_value,                              /* no stop position */
        0.1f,                                   /* finite watchdog timeout */
        MOTEUS_MAX_COMMAND_VELOCITY_REV_S,      /* hard velocity cap */
        nan_value, nan_value,                   /* accel/voltage overrides */
        nan_value,                              /* retain integral limit scale */
    };
    for (size_t j = 0; j < 12; ++j, i += 4) put_float_le(&frame[i], command[j]);
    frame[i++] = 0x11; frame[i++] = 0x00;
    frame[i++] = 0x11; frame[i++] = 0x0f;
    frame[i++] = 0x1f; frame[i++] = 0x01;
    while (i < MOTEUS_FRAME_BYTES) frame[i++] = 0x50;
    return i == MOTEUS_FRAME_BYTES;
}

static bool parse_hex_bytes(const char *hex, uint8_t *bytes, size_t capacity,
                            size_t *length)
{
    const size_t hex_length = strlen(hex);
    if ((hex_length & 1u) != 0 || hex_length / 2 > capacity) return false;
    for (size_t i = 0; i < hex_length / 2; ++i) {
        char pair[3] = {hex[2 * i], hex[2 * i + 1], '\0'};
        char *end = NULL;
        const long value = strtol(pair, &end, 16);
        if (end != pair + 2 || value < 0 || value > 255) return false;
        bytes[i] = (uint8_t)value;
    }
    *length = hex_length / 2;
    return true;
}

static bool parse_reply(const char *line, cube_moteus_status_t *status)
{
    if (strncmp(line, "rcv ", 4) != 0) return false;
    char copy[MOTEUS_LINE_CAPACITY];
    snprintf(copy, sizeof(copy), "%s", line + 4);
    char *save = NULL;
    char *id_text = strtok_r(copy, " ", &save);
    char *data_text = strtok_r(NULL, " ", &save);
    if (id_text == NULL || data_text == NULL) return false;
    char *end = NULL;
    const unsigned long id = strtoul(id_text, &end, 16);
    if (end == id_text || *end != '\0' ||
        id != ((unsigned long)CONFIG_CUBESAT_MOTEUS_CONTROLLER_ID << 8))
        return false;

    uint8_t data[MOTEUS_FRAME_BYTES];
    size_t length = 0;
    if (!parse_hex_bytes(data_text, data, sizeof(data), &length) || length < 20)
        return false;
    /* Strictly check the queried register reply sequence and lengths. */
    if (data[0] != 0x21 || data[1] != 0x00 ||
        data[3] != 0x21 || data[4] != 0x0f ||
        data[6] != 0x2f || data[7] != 0x01)
        return false;
    status->mode = data[2];
    status->fault = data[5];
    status->position_rev = get_float_le(&data[8]);
    status->velocity_rev_s = get_float_le(&data[12]);
    status->motor_torque_nm = get_float_le(&data[16]);
    status->valid = isfinite(status->position_rev) &&
                    isfinite(status->velocity_rev_s) &&
                    isfinite(status->motor_torque_nm) &&
                    status->mode == 10 && status->fault == 0;
    return isfinite(status->position_rev) && isfinite(status->velocity_rev_s) &&
           isfinite(status->motor_torque_nm);
}

static bool exchange_frame(const uint8_t frame[MOTEUS_FRAME_BYTES],
                           cube_moteus_status_t *status)
{
    char command[2 * MOTEUS_FRAME_BYTES + 48];
    int n = snprintf(command, sizeof(command), "can send %04X ",
                     0x8000u | CONFIG_CUBESAT_MOTEUS_CONTROLLER_ID);
    if (n < 0 || (size_t)n >= sizeof(command)) return false;
    size_t used = (size_t)n;
    for (size_t i = 0; i < MOTEUS_FRAME_BYTES; ++i) {
        if (used + 3 >= sizeof(command)) return false;
        n = snprintf(command + used, sizeof(command) - used, "%02X", frame[i]);
        if (n != 2) return false;
        used += 2;
    }
    n = snprintf(command + used, sizeof(command) - used, " BF");
    if (n < 0 || (size_t)n >= sizeof(command) - used) return false;
    if (!write_checked_line(command)) return false;

    bool got_reply = false;
    bool got_ok = false;
    const int64_t deadline = (int64_t)xTaskGetTickCount() * portTICK_PERIOD_MS +
                             MOTEUS_STATUS_TIMEOUT_MS;
    while ((int64_t)xTaskGetTickCount() * portTICK_PERIOD_MS < deadline) {
        char line[MOTEUS_LINE_CAPACITY];
        if (!read_checked_line(line, sizeof(line), 5)) continue;
        if (strncmp(line, "ERR", 3) == 0) return false;
        if (strcmp(line, "OK") == 0) {
            got_ok = true;
        } else if (strncmp(line, "rcv ", 4) == 0) {
            if (!parse_reply(line, status)) return false;
            got_reply = true;
        }
        if (got_ok && got_reply) return true;
    }
    return false;
}

bool cube_moteus_uart_init(void)
{
    if (CONFIG_CUBESAT_MOTEUS_UART_TX_GPIO == CONFIG_CUBESAT_MOTEUS_UART_RX_GPIO ||
        CONFIG_CUBESAT_MOTEUS_ARM_GPIO == CONFIG_CUBESAT_MOTEUS_UART_TX_GPIO ||
        CONFIG_CUBESAT_MOTEUS_ARM_GPIO == CONFIG_CUBESAT_MOTEUS_UART_RX_GPIO) {
        ESP_LOGE(TAG, "UART TX, RX, and arm GPIO must be distinct");
        return false;
    }
    if (gpio_set_direction(CONFIG_CUBESAT_MOTEUS_ARM_GPIO, GPIO_MODE_INPUT) != ESP_OK ||
        gpio_set_pull_mode(CONFIG_CUBESAT_MOTEUS_ARM_GPIO, GPIO_PULLDOWN_ONLY) != ESP_OK ||
        uart_driver_install(MOTEUS_UART, 2048, 0, 0, NULL, 0) != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize actuator GPIO/UART");
        return false;
    }
    const uart_config_t config = {
        .baud_rate = MOTEUS_BAUD,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };
    if (uart_param_config(MOTEUS_UART, &config) != ESP_OK ||
        uart_set_pin(MOTEUS_UART, CONFIG_CUBESAT_MOTEUS_UART_TX_GPIO,
                     CONFIG_CUBESAT_MOTEUS_UART_RX_GPIO,
                     UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE) != ESP_OK) {
        ESP_LOGE(TAG, "Failed to configure moteus UART");
        uart_driver_delete(MOTEUS_UART);
        return false;
    }
    uart_ready = true;
    if (gpio_get_level(CONFIG_CUBESAT_MOTEUS_ARM_GPIO) != 1) {
        ESP_LOGW(TAG, "Actuator remains disarmed: arm input is low");
        uart_ready = false;
        return false;
    }
    uart_flush_input(MOTEUS_UART);
    if (!command_ok("can on", true)) {
        ESP_LOGE(TAG, "moteus UART did not enter CAN bus-on state");
        return false;
    }
    cube_moteus_status_t status = {0};
    uint8_t frame[MOTEUS_FRAME_BYTES];
    make_torque_frame(0.0f, frame);
    if (!exchange_frame(frame, &status) || !status.valid) {
        ESP_LOGE(TAG, "moteus zero-torque/status handshake failed");
        return false;
    }
    ESP_LOGI(TAG, "moteus UART active, mode=%d fault=%d", status.mode, status.fault);
    return true;
}

bool cube_moteus_command_torque(float motor_torque_nm,
                                cube_moteus_status_t *status)
{
    if (!uart_ready || status == NULL || !isfinite(motor_torque_nm) ||
        gpio_get_level(CONFIG_CUBESAT_MOTEUS_ARM_GPIO) != 1)
        return false;
    const float limit_nm = CONFIG_CUBESAT_MOTEUS_MAX_TORQUE_MNM / 1000.0f;
    if (motor_torque_nm > limit_nm) motor_torque_nm = limit_nm;
    if (motor_torque_nm < -limit_nm) motor_torque_nm = -limit_nm;
    memset(status, 0, sizeof(*status));
    status->commanded_motor_torque_nm = motor_torque_nm;
    uint8_t frame[MOTEUS_FRAME_BYTES];
    make_torque_frame(motor_torque_nm, frame);
    return exchange_frame(frame, status);
}

bool cube_moteus_command_velocity(float target_rev_s,
                                  cube_moteus_status_t *status)
{
    if (!uart_ready || status == NULL || !isfinite(target_rev_s) ||
        fabsf(target_rev_s) > MOTEUS_MAX_COMMAND_VELOCITY_REV_S ||
        gpio_get_level(CONFIG_CUBESAT_MOTEUS_ARM_GPIO) != 1)
        return false;
    memset(status, 0, sizeof(*status));
    status->commanded_motor_torque_nm = 0.0f;
    uint8_t frame[MOTEUS_FRAME_BYTES];
    if (!make_velocity_frame(target_rev_s, frame)) return false;
    return exchange_frame(frame, status);
}

bool cube_moteus_arm_active(void)
{
    return gpio_get_level(CONFIG_CUBESAT_MOTEUS_ARM_GPIO) == 1;
}
