#include "yaw_controller.h"
#include "attitude_estimator.h"
#include "black_target.h"
#include "xiao_camera.h"
#include "mpu6050.h"
#include "lis3mdl.h"
#include "sdkconfig.h"
#if CONFIG_CUBESAT_MOTEUS_ENABLED
#include "moteus_uart.h"
#endif

#include <math.h>
#include <float.h>
#include <inttypes.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "cJSON.h"
#include "esp_event.h"
#include "esp_http_client.h"
#include "esp_log.h"
#include "esp_netif.h"
#include "esp_timer.h"
#include "esp_wifi.h"
#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"
#include "freertos/task.h"
#include "nvs_flash.h"

#define WIFI_CONNECTED_BIT BIT0
#define WIFI_FAIL_BIT BIT1
#define HTTP_BODY_CAPACITY 2048
#define CONTROL_PERIOD_MS 20
#define CAMERA_PREVIEW_BYTES (64 * 48)
#define SIM_CAMERA_WIDTH_PX 320u
#define SIM_CAMERA_HEIGHT_PX 240u
#define SIM_CAMERA_FRAME_BYTES (SIM_CAMERA_WIDTH_PX * SIM_CAMERA_HEIGHT_PX)
#define SIM_CAMERA_RLE_MAX_BYTES CUBE_RLE1_MAX_BYTES

#if CONFIG_FREERTOS_HZ < 50 || (CONFIG_FREERTOS_HZ % 50) != 0
#error "Configure FreeRTOS tick rate as a multiple of 50 Hz for a 20 ms control period"
#endif

static const char *TAG = "CUBESAT";
static EventGroupHandle_t wifi_event_group;
static int wifi_retry_count;
static uint32_t telemetry_sequence;
static uint64_t last_command_id;
static char operating_mode[24] = "TRACK";
static bool have_command_ack;
static uint64_t acknowledged_command_id;
static char command_ack_status[16];
static char command_ack_reason[40];
static float wheel_speed_target_rev_s;
static int64_t wheel_speed_test_started_us;
static int64_t wheel_speed_test_deadline_us;
static int64_t wheel_speed_stop_deadline_us;
static char wheel_speed_test_status[16] = "IDLE";
static float last_motor_speed_rev_s;
static bool last_motor_speed_valid;
static cube_yaw_estimator_t yaw_estimator;
static int64_t last_attitude_update_us;
static bool last_attitude_valid;
static float yaw_step_target_rad;
static bool yaw_step_target_valid;
static int64_t yaw_step_started_us;
static int64_t yaw_step_deadline_us;
static int64_t yaw_step_settle_started_us;
static float yaw_step_error_rad;
static char yaw_step_status[20] = "IDLE";
#if CONFIG_CUBESAT_CAMERA_ENABLED
static bool camera_online;
#endif
#if CONFIG_CUBESAT_MOTEUS_ENABLED
static bool moteus_online;
#endif
#if CONFIG_CUBESAT_CAMERA_ENABLED
static unsigned char camera_preview_upload_buffer[CAMERA_PREVIEW_BYTES];
static int64_t last_preview_upload_us;
#else
static uint8_t simulated_camera_frame[SIM_CAMERA_FRAME_BYTES];
static uint8_t simulated_camera_rle[SIM_CAMERA_RLE_MAX_BYTES];
#endif

static const char *fault_name(cube_control_fault_t fault)
{
    switch (fault) {
    case CUBE_CONTROL_NO_FAULT: return "";
    case CUBE_CONTROL_NONFINITE_SENSOR: return "NONFINITE_SENSOR";
    case CUBE_CONTROL_RATE_LIMIT: return "RATE_LIMIT";
    case CUBE_CONTROL_STALE_OBSERVATION: return "STALE_OBSERVATION";
    case CUBE_CONTROL_INVALID_TARGET_ERROR: return "INVALID_TARGET_ERROR";
    case CUBE_CONTROL_INVALID_CONFIG: return "INVALID_CONFIG";
    default: return "UNKNOWN_FAULT";
    }
}

static bool json_age_valid(const cJSON *item)
{
    return cJSON_IsNumber(item) && isfinite(item->valuedouble) &&
           item->valuedouble >= 0.0 && item->valuedouble <= FLT_MAX;
}

static bool wheel_speed_mode_active(void)
{
    return strcmp(operating_mode, "WHEEL_SPEED_TEST") == 0 ||
           strcmp(operating_mode, "WHEEL_SPEED_STOPPING") == 0;
}

static void begin_wheel_speed_stop(int64_t now_us)
{
    snprintf(operating_mode, sizeof(operating_mode), "WHEEL_SPEED_STOPPING");
    snprintf(wheel_speed_test_status, sizeof(wheel_speed_test_status), "STOPPING");
    wheel_speed_target_rev_s = 0.0f;
    wheel_speed_stop_deadline_us = now_us + 10000000;
}

static float wrap_pi(float angle)
{
    while (angle > 3.14159265359f) angle -= 6.28318530718f;
    while (angle < -3.14159265359f) angle += 6.28318530718f;
    return angle;
}

typedef struct {
    char body[HTTP_BODY_CAPACITY];
    size_t length;
    bool overflow;
} http_body_t;

#if !CONFIG_CUBESAT_CAMERA_ENABLED
typedef struct {
    uint8_t *data;
    size_t capacity;
    size_t length;
    bool overflow;
    uint64_t expected_sequence;
    bool sequence_header_valid;
    bool encoding_header_valid;
    bool width_header_valid;
    bool height_header_valid;
} http_binary_body_t;

static bool http_header_name_is(const char *actual, const char *expected)
{
    if (actual == NULL || expected == NULL) return false;
    while (*actual != '\0' && *expected != '\0') {
        char actual_char = *actual++;
        char expected_char = *expected++;
        if (actual_char >= 'A' && actual_char <= 'Z') {
            actual_char = (char)(actual_char + ('a' - 'A'));
        }
        if (expected_char >= 'A' && expected_char <= 'Z') {
            expected_char = (char)(expected_char + ('a' - 'A'));
        }
        if (actual_char != expected_char) return false;
    }
    return *actual == '\0' && *expected == '\0';
}

static esp_err_t collect_http_binary(esp_http_client_event_t *event)
{
    http_binary_body_t *body = event->user_data;
    if (event->event_id == HTTP_EVENT_ON_HEADER &&
        event->header_key != NULL && event->header_value != NULL) {
        char sequence_text[24];
        snprintf(sequence_text, sizeof(sequence_text), "%" PRIu64,
                 body->expected_sequence);
        if (http_header_name_is(event->header_key,
                                "X-Camera-Frame-Sequence")) {
            body->sequence_header_valid =
                strcmp(event->header_value, sequence_text) == 0;
        } else if (http_header_name_is(event->header_key,
                                      "X-Camera-Frame-Encoding")) {
            body->encoding_header_valid = strcmp(event->header_value, "RLE1") == 0;
        } else if (http_header_name_is(event->header_key,
                                      "X-Camera-Frame-Width")) {
            body->width_header_valid =
                strcmp(event->header_value, "320") == 0;
        } else if (http_header_name_is(event->header_key,
                                      "X-Camera-Frame-Height")) {
            body->height_header_valid =
                strcmp(event->header_value, "240") == 0;
        }
    } else if (event->event_id == HTTP_EVENT_ON_DATA && event->data_len > 0) {
        size_t count = (size_t)event->data_len;
        const size_t available = body->capacity - body->length;
        if (count > available) {
            count = available;
            body->overflow = true;
        }
        memcpy(body->data + body->length, event->data, count);
        body->length += count;
    }
    return ESP_OK;
}

static bool fetch_simulated_camera_frame(uint64_t sequence,
                                         size_t *encoded_length)
{
    if (encoded_length == NULL) return false;
    char url[320];
    snprintf(url, sizeof(url), "%s/api/camera-frame.rle?sequence=%" PRIu64,
             CONFIG_CUBESAT_SIM_BRIDGE_URL, sequence);
    http_binary_body_t body = {
        .data = simulated_camera_rle,
        .capacity = sizeof(simulated_camera_rle),
        .expected_sequence = sequence,
    };
    const esp_http_client_config_t config = {
        .url = url,
        .timeout_ms = 500,
        .event_handler = collect_http_binary,
        .user_data = &body,
    };
    esp_http_client_handle_t client = esp_http_client_init(&config);
    if (client == NULL) return false;
    esp_http_client_set_method(client, HTTP_METHOD_GET);
    const esp_err_t result = esp_http_client_perform(client);
    const int status = esp_http_client_get_status_code(client);
    esp_http_client_cleanup(client);
    if (result != ESP_OK || status != 200 || body.overflow || body.length <= 4u ||
        !body.sequence_header_valid || !body.encoding_header_valid ||
        !body.width_header_valid || !body.height_header_valid) {
        return false;
    }
    *encoded_length = body.length;
    return true;
}
#endif

static esp_err_t collect_http_body(esp_http_client_event_t *event)
{
    http_body_t *body = event->user_data;
    if (event->event_id == HTTP_EVENT_ON_DATA && event->data_len > 0) {
        const size_t available = sizeof(body->body) - body->length - 1;
        size_t count = (size_t)event->data_len;
        if (count > available) {
            count = available;
            body->overflow = true;
        }
        memcpy(body->body + body->length, event->data, count);
        body->length += count;
        body->body[body->length] = '\0';
    }
    return ESP_OK;
}

static bool http_request(const char *path, const char *post_body,
                         http_body_t *response)
{
    char url[256];
    snprintf(url, sizeof(url), "%s%s", CONFIG_CUBESAT_SIM_BRIDGE_URL, path);
    memset(response, 0, sizeof(*response));
    const esp_http_client_config_t config = {
        .url = url,
        .timeout_ms = 500,
        .event_handler = collect_http_body,
        .user_data = response,
    };
    esp_http_client_handle_t client = esp_http_client_init(&config);
    if (client == NULL) {
        return false;
    }
    esp_http_client_set_header(client, "Content-Type", "application/json");
    if (post_body != NULL) {
        esp_http_client_set_method(client, HTTP_METHOD_POST);
        esp_http_client_set_post_field(client, post_body, (int)strlen(post_body));
    } else {
        esp_http_client_set_method(client, HTTP_METHOD_GET);
    }
    const esp_err_t err = esp_http_client_perform(client);
    const int status = esp_http_client_get_status_code(client);
    esp_http_client_cleanup(client);
    return err == ESP_OK && status >= 200 && status < 300 && !response->overflow;
}

#if CONFIG_CUBESAT_CAMERA_ENABLED
static bool http_post_camera_preview(const unsigned char *frame, size_t length)
{
    char url[256];
    snprintf(url, sizeof(url), "%s/api/camera-preview",
             CONFIG_CUBESAT_SIM_BRIDGE_URL);
    const esp_http_client_config_t config = {.url = url, .timeout_ms = 700};
    esp_http_client_handle_t client = esp_http_client_init(&config);
    if (client == NULL) return false;
    esp_http_client_set_method(client, HTTP_METHOD_POST);
    esp_http_client_set_header(client, "Content-Type", "application/octet-stream");
    esp_http_client_set_post_field(client, (const char *)frame, (int)length);
    const esp_err_t err = esp_http_client_perform(client);
    const int status = esp_http_client_get_status_code(client);
    esp_http_client_cleanup(client);
    return err == ESP_OK && status == 204;
}
#endif

static void handle_command(cJSON *command)
{
    cJSON *id_json = cJSON_GetObjectItemCaseSensitive(command, "command_id");
    cJSON *name_json = cJSON_GetObjectItemCaseSensitive(command, "command");
    if (!cJSON_IsNumber(id_json) || !cJSON_IsString(name_json) ||
        !isfinite(id_json->valuedouble) || id_json->valuedouble < 1.0 ||
        id_json->valuedouble > 9007199254740991.0 ||
        floor(id_json->valuedouble) != id_json->valuedouble) {
        return;
    }
    const uint64_t command_id = (uint64_t)id_json->valuedouble;
    if (command_id <= last_command_id) {
        return;
    }
    const char *name = name_json->valuestring;
    const char *status = "ACKNOWLEDGED";
    const char *reason = "";
    if (strcmp(name, "PING") == 0) {
        // Health request: the next telemetry packet is the response.
    } else if (strcmp(name, "SET_MODE") == 0) {
        cJSON *arguments = cJSON_GetObjectItemCaseSensitive(command, "arguments");
        cJSON *mode_json = cJSON_GetObjectItemCaseSensitive(arguments, "mode");
        const char *mode = cJSON_IsString(mode_json) ? mode_json->valuestring : "";
        if (strcmp(mode, "SAFE") == 0 || strcmp(mode, "STANDBY") == 0 ||
            strcmp(mode, "RATE_DAMP") == 0 || strcmp(mode, "ACQUIRE") == 0 ||
            strcmp(mode, "TRACK") == 0) {
            if (strcmp(operating_mode, "YAW_STEP") == 0 &&
                strcmp(mode, "SAFE") != 0 && strcmp(mode, "STANDBY") != 0) {
                status = "REJECTED";
                reason = "YAW_STEP_ACTIVE";
            } else if (wheel_speed_mode_active() && strcmp(mode, "SAFE") != 0) {
                status = "REJECTED";
                reason = "SPEED_TEST_ACTIVE";
            } else if (wheel_speed_mode_active()) {
                begin_wheel_speed_stop(esp_timer_get_time());
            } else {
                if (strcmp(operating_mode, "YAW_STEP") == 0) {
                    snprintf(yaw_step_status, sizeof(yaw_step_status), "ABORTED");
                    yaw_step_target_valid = false;
                    yaw_step_settle_started_us = 0;
                }
                snprintf(operating_mode, sizeof(operating_mode), "%s", mode);
            }
        } else {
            status = "REJECTED";
            reason = "INVALID_MODE";
        }
    } else if (strcmp(name, "SET_WHEEL_SPEED") == 0) {
        cJSON *arguments = cJSON_GetObjectItemCaseSensitive(command, "arguments");
        cJSON *target_json = cJSON_GetObjectItemCaseSensitive(arguments,
                                                               "target_rev_s");
        cJSON *duration_json = cJSON_GetObjectItemCaseSensitive(arguments,
                                                                 "duration_s");
        const bool valid_arguments = cJSON_IsObject(arguments) &&
            cJSON_GetArraySize(arguments) == 2 &&
            cJSON_IsNumber(target_json) && cJSON_IsNumber(duration_json) &&
            isfinite(target_json->valuedouble) &&
            target_json->valuedouble >= 0.0 && target_json->valuedouble <= 50.0 &&
            isfinite(duration_json->valuedouble) &&
            duration_json->valuedouble >= 0.5 && duration_json->valuedouble <= 5.0;
        if (!valid_arguments) {
            status = "REJECTED";
            reason = "INVALID_SPEED_TEST_ARGUMENTS";
        } else if (strcmp(operating_mode, "SAFE") != 0 &&
                   strcmp(operating_mode, "STANDBY") != 0) {
            status = "REJECTED";
            reason = "SPEED_TEST_REQUIRES_SAFE_OR_STANDBY";
#if CONFIG_CUBESAT_MOTEUS_ENABLED
        } else if (!moteus_online || !cube_moteus_arm_active()) {
            status = "REJECTED";
            reason = "ACTUATOR_NOT_ARMED";
#endif
        } else {
            const int64_t now_us = esp_timer_get_time();
            wheel_speed_target_rev_s = (float)target_json->valuedouble;
            wheel_speed_test_started_us = now_us;
            wheel_speed_test_deadline_us = now_us +
                (int64_t)(duration_json->valuedouble * 1000000.0);
            wheel_speed_stop_deadline_us = 0;
            snprintf(wheel_speed_test_status, sizeof(wheel_speed_test_status), "RUNNING");
            snprintf(operating_mode, sizeof(operating_mode), "WHEEL_SPEED_TEST");
        }
    } else if (strcmp(name, "SET_YAW_STEP") == 0) {
        cJSON *arguments = cJSON_GetObjectItemCaseSensitive(command, "arguments");
        cJSON *delta_json = cJSON_GetObjectItemCaseSensitive(arguments, "delta_deg");
        if (!cJSON_IsObject(arguments) || cJSON_GetArraySize(arguments) != 1 ||
            !cJSON_IsNumber(delta_json) || !isfinite(delta_json->valuedouble) ||
            delta_json->valuedouble < -180.0 || delta_json->valuedouble > 180.0) {
            status = "REJECTED";
            reason = "INVALID_YAW_STEP_ARGUMENTS";
        } else if (strcmp(operating_mode, "SAFE") != 0 &&
                   strcmp(operating_mode, "STANDBY") != 0) {
            status = "REJECTED";
            reason = "YAW_STEP_REQUIRES_SAFE_OR_STANDBY";
        } else if (!last_attitude_valid) {
            status = "REJECTED";
            reason = "ATTITUDE_ESTIMATE_UNAVAILABLE";
        } else {
            const int64_t now_us = esp_timer_get_time();
            yaw_step_target_rad = wrap_pi(yaw_estimator.yaw_rad +
                (float)delta_json->valuedouble * 0.01745329252f);
            yaw_step_target_valid = true;
            yaw_step_started_us = now_us;
            yaw_step_deadline_us = now_us + 10000000;
            yaw_step_settle_started_us = 0;
            yaw_step_error_rad = 0.0f;
            snprintf(yaw_step_status, sizeof(yaw_step_status), "RUNNING");
            snprintf(operating_mode, sizeof(operating_mode), "YAW_STEP");
        }
    } else {
        status = "REJECTED";
        reason = "UNSUPPORTED_COMMAND";
    }
    last_command_id = command_id;
    acknowledged_command_id = command_id;
    snprintf(command_ack_status, sizeof(command_ack_status), "%s", status);
    snprintf(command_ack_reason, sizeof(command_ack_reason), "%s", reason);
    have_command_ack = true;
}

static void process_commands(cJSON *commands)
{
    if (!cJSON_IsArray(commands)) {
        return;
    }
    cJSON *item = NULL;
    cJSON_ArrayForEach(item, commands) {
        handle_command(item);
        if (have_command_ack) {
            break; // Preserve ordered command acknowledgement semantics.
        }
    }
}

static cube_yaw_controller_config_t controller_config;
#if CONFIG_CUBESAT_MPU6050_ENABLED
static bool imu_online;
#endif
#if CONFIG_CUBESAT_LIS3MDL_ENABLED
static bool magnetometer_online;
#endif

static bool run_control_cycle(void)
{
    http_body_t response;
    const int64_t cycle_request_started_us = esp_timer_get_time();
    char path[96];
    snprintf(path, sizeof(path), "/api/cycle?after=%" PRIu64,
             last_command_id);
    if (!http_request(path, NULL, &response)) {
        ESP_LOGW(TAG, "Environment observation unavailable; bridge watchdog will zero actuator output");
        return false;
    }
    cJSON *root = cJSON_Parse(response.body);
    if (!cJSON_IsObject(root)) {
        cJSON_Delete(root);
        ESP_LOGW(TAG, "Invalid JSON returned by the environment bridge");
        return false;
    }
    cJSON *version = cJSON_GetObjectItemCaseSensitive(root, "protocol_version");
    cJSON *message_type = cJSON_GetObjectItemCaseSensitive(root, "message_type");
    cJSON *commands = cJSON_GetObjectItemCaseSensitive(root, "commands");
    cJSON *observation = cJSON_GetObjectItemCaseSensitive(root, "observation");
    cJSON *sensors = cJSON_GetObjectItemCaseSensitive(observation, "sensors");
    cJSON *actuator_feedback = cJSON_GetObjectItemCaseSensitive(
        observation, "actuator_feedback");
    cJSON *sim_wheel_speed_json = cJSON_GetObjectItemCaseSensitive(
        actuator_feedback, "wheel_speed_rad_s");
    cJSON *camera = cJSON_GetObjectItemCaseSensitive(sensors, "camera");
    cJSON *imu = cJSON_GetObjectItemCaseSensitive(sensors, "imu");
    cJSON *magnetometer = cJSON_GetObjectItemCaseSensitive(sensors, "magnetometer");
    cJSON *gyro = cJSON_GetObjectItemCaseSensitive(imu, "gyro_rad_s");
    cJSON *camera_age_json = cJSON_GetObjectItemCaseSensitive(camera, "age_s");
#if !CONFIG_CUBESAT_CAMERA_ENABLED
    cJSON *focal_json = cJSON_GetObjectItemCaseSensitive(camera, "focal_length_x_px");
#endif
    cJSON *imu_age_json = cJSON_GetObjectItemCaseSensitive(imu, "age_s");
    cJSON *accel = cJSON_GetObjectItemCaseSensitive(imu, "accel_m_s2");
    cJSON *gyro_x_json = cJSON_GetObjectItemCaseSensitive(gyro, "x");
    cJSON *gyro_y_json = cJSON_GetObjectItemCaseSensitive(gyro, "y");
    cJSON *gyro_z_json = cJSON_GetObjectItemCaseSensitive(gyro, "z");
    cJSON *accel_x_json = cJSON_GetObjectItemCaseSensitive(accel, "x");
    cJSON *accel_y_json = cJSON_GetObjectItemCaseSensitive(accel, "y");
    cJSON *accel_z_json = cJSON_GetObjectItemCaseSensitive(accel, "z");
    cJSON *mag_valid_json = cJSON_GetObjectItemCaseSensitive(magnetometer, "valid");
    cJSON *mag_field = cJSON_GetObjectItemCaseSensitive(magnetometer,
                                                        "magnetic_field_uT");
    cJSON *mag_x_json = cJSON_GetObjectItemCaseSensitive(mag_field, "x");
    cJSON *mag_y_json = cJSON_GetObjectItemCaseSensitive(mag_field, "y");
    cJSON *mag_z_json = cJSON_GetObjectItemCaseSensitive(mag_field, "z");
    cJSON *mag_age_json = cJSON_GetObjectItemCaseSensitive(magnetometer, "age_s");
    cJSON *sim_time_json = cJSON_GetObjectItemCaseSensitive(observation, "simulation_time_s");
#if !CONFIG_CUBESAT_CAMERA_ENABLED
    cJSON *observation_sequence_json = cJSON_GetObjectItemCaseSensitive(
        observation, "sequence");
#endif
    const bool packet_valid = cJSON_IsNumber(version) && version->valueint == 1 &&
        version->valuedouble == 1.0 &&
        cJSON_IsString(message_type) &&
        strcmp(message_type->valuestring, "simulation_cycle") == 0 &&
        cJSON_IsNumber(gyro_x_json) && cJSON_IsNumber(gyro_y_json) &&
        cJSON_IsNumber(gyro_z_json) && cJSON_IsNumber(accel_x_json) &&
        cJSON_IsNumber(accel_y_json) && cJSON_IsNumber(accel_z_json);
    if (!packet_valid) {
        cJSON_Delete(root);
        ESP_LOGW(TAG, "Invalid environment observation packet");
        return false;
    }
    process_commands(commands);

    const int64_t cycle_now_us = esp_timer_get_time();
    float measured_motor_speed_rev_s = NAN;
    bool measured_motor_speed_valid = false;
    if (cJSON_IsNumber(sim_wheel_speed_json) &&
        isfinite(sim_wheel_speed_json->valuedouble)) {
        measured_motor_speed_rev_s = (float)sim_wheel_speed_json->valuedouble *
                                     60.0f / 6.28318530718f;
        measured_motor_speed_valid = isfinite(measured_motor_speed_rev_s);
    }
#if CONFIG_CUBESAT_MOTEUS_ENABLED
    if (last_motor_speed_valid) {
        measured_motor_speed_rev_s = last_motor_speed_rev_s;
        measured_motor_speed_valid = true;
    }
#endif
    if (strcmp(operating_mode, "WHEEL_SPEED_TEST") == 0 &&
        cycle_now_us >= wheel_speed_test_deadline_us) {
        begin_wheel_speed_stop(cycle_now_us);
    }
    const bool speed_command_active = wheel_speed_mode_active();
    const bool speed_stop_timed_out =
        strcmp(operating_mode, "WHEEL_SPEED_STOPPING") == 0 &&
        cycle_now_us >= wheel_speed_stop_deadline_us;
    const bool speed_stop_reached =
        strcmp(operating_mode, "WHEEL_SPEED_STOPPING") == 0 &&
        measured_motor_speed_valid && fabsf(measured_motor_speed_rev_s) <= 0.5f;
    const bool finish_speed_stop_this_cycle =
        speed_stop_timed_out || speed_stop_reached;

    bool camera_age_valid = json_age_valid(camera_age_json);
    float camera_age = camera_age_valid ? (float)camera_age_json->valuedouble : NAN;
    bool target_valid;
    float target_error_px;
    float target_error_y_px;
    float focal_length_px;
    const char *camera_frame_source = "unavailable";
    uint64_t camera_frame_sequence = 0;
    bool camera_frame_sequence_valid = false;
#if CONFIG_CUBESAT_CAMERA_ENABLED
    cube_camera_observation_t camera_observation;
    const bool camera_ok = cube_xiao_camera_observe(&camera_observation);
    camera_age = camera_ok
        ? (float)(esp_timer_get_time() - camera_observation.sampled_at_us) / 1000000.0f
        : NAN;
    camera_age_valid = camera_ok && isfinite(camera_age) && camera_age >= 0.0f;
    target_valid = camera_age_valid && camera_observation.valid;
    target_error_px = camera_observation.error_x_px;
    target_error_y_px = camera_observation.error_y_px;
    focal_length_px = camera_observation.focal_length_x_px;
    camera_frame_source = "physical";
#else
    cube_target_detection_t detection = {0};
    const double raw_sequence = cJSON_IsNumber(observation_sequence_json)
        ? observation_sequence_json->valuedouble : 0.0;
    const bool sequence_ok = isfinite(raw_sequence) && raw_sequence >= 1.0 &&
        raw_sequence <= 9007199254740991.0 && trunc(raw_sequence) == raw_sequence;
    size_t encoded_frame_length = 0;
    const bool frame_payload_received = sequence_ok &&
        fetch_simulated_camera_frame((uint64_t)raw_sequence,
                                     &encoded_frame_length);
    const bool frame_received = frame_payload_received &&
        cube_decode_rle1_grayscale(simulated_camera_rle,
                                   encoded_frame_length,
                                   simulated_camera_frame,
                                   SIM_CAMERA_FRAME_BYTES);
    const cube_target_detector_config_t detector_config = {
        .threshold = 40,
        .min_dark_pixels = 50,
        .max_dark_pixels = 200,
    };
    const bool detector_ok = frame_received && cube_detect_black_target(
        simulated_camera_frame, SIM_CAMERA_WIDTH_PX, SIM_CAMERA_HEIGHT_PX,
        SIM_CAMERA_WIDTH_PX, &detector_config, &detection);
    camera_age = detector_ok
        ? (float)(esp_timer_get_time() - cycle_request_started_us) / 1000000.0f
        : NAN;
    camera_age_valid = detector_ok && isfinite(camera_age) && camera_age >= 0.0f;
    target_valid = camera_age_valid && detection.valid;
    target_error_px = target_valid ? detection.error_x_px : 0.0f;
    target_error_y_px = target_valid ? detection.error_y_px : 0.0f;
    focal_length_px = cJSON_IsNumber(focal_json) &&
        isfinite(focal_json->valuedouble) && focal_json->valuedouble > 0.0
        ? (float)focal_json->valuedouble : 0.0f;
    target_valid = target_valid && focal_length_px > 0.0f;
    if (frame_received && sequence_ok) {
        camera_frame_source = "simulated";
        camera_frame_sequence = (uint64_t)raw_sequence;
        camera_frame_sequence_valid = true;
    }
#endif
    const float error_rad = target_valid && focal_length_px > 0.0f
        ? atanf(target_error_px / focal_length_px) : 0.0f;
    float gyro_x = (float)gyro_x_json->valuedouble;
    float gyro_y = (float)gyro_y_json->valuedouble;
    float gyro_z = (float)gyro_z_json->valuedouble;
    float yaw_rate = (float)gyro_y_json->valuedouble;
    float accel_x = (float)accel_x_json->valuedouble;
    float accel_y = (float)accel_y_json->valuedouble;
    float accel_z = (float)accel_z_json->valuedouble;
    bool imu_age_valid = json_age_valid(imu_age_json);
    float imu_age = imu_age_valid ? (float)imu_age_json->valuedouble : NAN;
    bool imu_valid = imu_age_valid && isfinite(gyro_x) && isfinite(gyro_y) &&
        isfinite(gyro_z) && isfinite(accel_x) && isfinite(accel_y) &&
        isfinite(accel_z);
    float mag_x = cJSON_IsNumber(mag_x_json) ? (float)mag_x_json->valuedouble : 0.0f;
    float mag_y = cJSON_IsNumber(mag_y_json) ? (float)mag_y_json->valuedouble : 0.0f;
    float mag_z = cJSON_IsNumber(mag_z_json) ? (float)mag_z_json->valuedouble : 0.0f;
    float mag_age = cJSON_IsNumber(mag_age_json) ? (float)mag_age_json->valuedouble : 0.0f;
    bool mag_valid = cJSON_IsTrue(mag_valid_json) &&
        cJSON_IsNumber(mag_x_json) && cJSON_IsNumber(mag_y_json) &&
        cJSON_IsNumber(mag_z_json) && isfinite(mag_x) && isfinite(mag_y) &&
        isfinite(mag_z) && isfinite(mag_age) && mag_age >= 0.0f &&
        mag_age <= 0.1f;
    const char *mag_frame = "body";
#if CONFIG_CUBESAT_MPU6050_ENABLED
    cube_mpu6050_sample_t imu_sample;
    imu_valid = imu_online && cube_mpu6050_read(&imu_sample);
    if (imu_valid) {
        gyro_x = imu_sample.gyro_rad_s[0];
        gyro_y = imu_sample.gyro_rad_s[1];
        gyro_z = imu_sample.gyro_rad_s[2];
        /* Project body frame defines +Y as the controlled yaw axis. */
        yaw_rate = imu_sample.gyro_rad_s[1];
        accel_x = imu_sample.accel_m_s2[0];
        accel_y = imu_sample.accel_m_s2[1];
        accel_z = imu_sample.accel_m_s2[2];
        imu_age = (float)(esp_timer_get_time() - imu_sample.sampled_at_us) /
                  1000000.0f;
        imu_age_valid = isfinite(imu_age) && imu_age >= 0.0f;
        imu_valid = imu_age_valid && isfinite(gyro_x) && isfinite(gyro_y) &&
            isfinite(gyro_z) && isfinite(accel_x) && isfinite(accel_y) &&
            isfinite(accel_z);
    }
    if (!imu_valid) {
        /* Invalid local samples inhibit output without inventing an age. */
        imu_age_valid = false;
        imu_age = NAN;
    }
#endif
#if CONFIG_CUBESAT_LIS3MDL_ENABLED
    cube_lis3mdl_sample_t mag_sample;
    mag_valid = magnetometer_online && cube_lis3mdl_read(&mag_sample);
    if (mag_valid) {
        mag_x = mag_sample.magnetic_field_uT[0];
        mag_y = mag_sample.magnetic_field_uT[1];
        mag_z = mag_sample.magnetic_field_uT[2];
        mag_age = (float)(esp_timer_get_time() - mag_sample.sampled_at_us) /
                  1000000.0f;
        mag_valid = mag_age <= 0.1f;
        mag_frame = "sensor";
    } else {
        /* Do not substitute a synthetic field for a failed physical sensor. */
        mag_frame = NULL;
    }
#endif
    const float accel_for_attitude[3] = {accel_x, accel_y, accel_z};
    const float mag_for_attitude[3] = {mag_x, mag_y, mag_z};
    bool attitude_magnetometer_valid = mag_valid;
#if CONFIG_CUBESAT_MPU6050_ENABLED || CONFIG_CUBESAT_LIS3MDL_ENABLED
    /* Physical LIS3MDL data is raw and uncalibrated in this build. Do not
     * apply either it or the bridge's synthetic field as an absolute heading. */
    attitude_magnetometer_valid = false;
#endif
    const int64_t attitude_now_us = esp_timer_get_time();
    const float attitude_dt_s = last_attitude_update_us > 0
        ? (float)(attitude_now_us - last_attitude_update_us) / 1000000.0f
        : (float)CONTROL_PERIOD_MS / 1000.0f;
    last_attitude_update_us = attitude_now_us;
    cube_attitude_source_t attitude_source = CUBE_ATTITUDE_INVALID;
    const bool attitude_valid = imu_valid && imu_age_valid && imu_age <= 0.1f &&
        cube_yaw_estimator_update(&yaw_estimator, yaw_rate,
            accel_for_attitude, mag_for_attitude,
            attitude_magnetometer_valid && mag_age >= 0.0f && mag_age <= 0.1f,
            attitude_dt_s, &attitude_source);
    last_attitude_valid = attitude_valid;
    bool yaw_step_active = strcmp(operating_mode, "YAW_STEP") == 0;
    bool yaw_step_error_valid = false;
    if (yaw_step_active && !attitude_valid) {
        snprintf(yaw_step_status, sizeof(yaw_step_status), "ATTITUDE_INVALID");
        yaw_step_target_valid = false;
        yaw_step_settle_started_us = 0;
        yaw_step_active = false;
        snprintf(operating_mode, sizeof(operating_mode), "SAFE");
    }
    if (yaw_step_active && cycle_now_us >= yaw_step_deadline_us) {
        snprintf(yaw_step_status, sizeof(yaw_step_status), "TIMEOUT");
        yaw_step_target_valid = false;
        yaw_step_settle_started_us = 0;
        yaw_step_active = false;
        snprintf(operating_mode, sizeof(operating_mode), "SAFE");
    }
    if (yaw_step_active) {
        yaw_step_error_rad = wrap_pi(yaw_step_target_rad - yaw_estimator.yaw_rad);
        yaw_step_error_valid = true;
        if (fabsf(yaw_step_error_rad) <= 0.01745329252f &&
            fabsf(yaw_rate) <= 0.00872664626f) {
            if (yaw_step_settle_started_us == 0) yaw_step_settle_started_us = cycle_now_us;
            if (cycle_now_us - yaw_step_settle_started_us >= 1000000) {
                snprintf(yaw_step_status, sizeof(yaw_step_status), "COMPLETE");
                snprintf(operating_mode, sizeof(operating_mode), "STANDBY");
                yaw_step_active = false;
            }
        } else {
            yaw_step_settle_started_us = 0;
        }
    }
    /* Both contributing observation streams must be fresh. A fresh frame with
     * no detected target still permits rate damping; a missing/stale camera
     * sample fails closed rather than silently bypassing the age guard. */
    const float observation_age = camera_age_valid && imu_age_valid
        ? fmaxf(camera_age, imu_age) : INFINITY;

    cube_yaw_control_output_t output = {0};
    if (speed_command_active) {
        output.body_torque_nm = 0.0f;
        output.mode = CUBE_CONTROL_RATE_DAMP;
        output.fault = CUBE_CONTROL_NO_FAULT;
    } else if (yaw_step_active) {
        output = cube_yaw_controller_update(&controller_config,
            yaw_step_error_rad, true, yaw_rate, imu_age);
    } else if (strcmp(operating_mode, "SAFE") == 0 ||
        strcmp(operating_mode, "STANDBY") == 0) {
        output.mode = CUBE_CONTROL_RATE_DAMP;
        output.fault = CUBE_CONTROL_NO_FAULT;
    } else {
        const bool use_target = target_valid &&
            (strcmp(operating_mode, "TRACK") == 0 ||
             strcmp(operating_mode, "ACQUIRE") == 0);
        output = cube_yaw_controller_update(&controller_config,
            use_target ? error_rad : 0.0f, use_target, yaw_rate,
            observation_age);
    }
    if (output.mode == CUBE_CONTROL_FAULT) {
        ESP_LOGE(TAG, "Control inhibited by sensor/observation fault %d", output.fault);
    }

#if CONFIG_CUBESAT_MOTEUS_ENABLED
    cube_moteus_status_t moteus_status = {0};
    bool moteus_reply_received = false;
    bool moteus_command_ok = false;
    float motor_command_nm = -output.body_torque_nm; /* reaction-wheel sign */
    if (output.mode == CUBE_CONTROL_FAULT ||
        strcmp(operating_mode, "SAFE") == 0 ||
        strcmp(operating_mode, "STANDBY") == 0) {
        motor_command_nm = 0.0f;
    }
    if (!cube_moteus_arm_active()) {
        /* Dropping the hardware arm input requires a firmware restart before
         * any later re-arm can restore motor commands. */
        moteus_online = false;
    }
    if (moteus_online) {
        if (speed_command_active) {
            moteus_reply_received = cube_moteus_command_velocity(
                wheel_speed_target_rev_s, &moteus_status);
        } else {
            moteus_reply_received = cube_moteus_command_torque(
                motor_command_nm, &moteus_status);
        }
        moteus_command_ok = moteus_reply_received && moteus_status.valid;
        if (moteus_reply_received && moteus_status.valid) {
            last_motor_speed_rev_s = moteus_status.velocity_rev_s;
            last_motor_speed_valid = isfinite(last_motor_speed_rev_s);
            measured_motor_speed_rev_s = last_motor_speed_rev_s;
            measured_motor_speed_valid = last_motor_speed_valid;
        } else {
            last_motor_speed_valid = false;
        }
        if (moteus_reply_received && !moteus_status.valid) {
            /* A reported drive fault/mode mismatch latches the physical output
             * off until a deliberate firmware restart and re-arm. */
            moteus_online = false;
        }
        if (!moteus_command_ok) {
            ESP_LOGE(TAG, "moteus command/status exchange failed; controller watchdog will expire");
        }
    }
#endif

    float motor_speed_error_percent = NAN;
    float motor_speed_test_elapsed_s = NAN;
    if (strcmp(operating_mode, "WHEEL_SPEED_TEST") == 0) {
        snprintf(wheel_speed_test_status, sizeof(wheel_speed_test_status), "RUNNING");
        motor_speed_test_elapsed_s =
            (float)(cycle_now_us - wheel_speed_test_started_us) / 1000000.0f;
        if (measured_motor_speed_valid && wheel_speed_target_rev_s > 0.0f) {
            motor_speed_error_percent = 100.0f *
                fabsf(wheel_speed_target_rev_s - measured_motor_speed_rev_s) /
                wheel_speed_target_rev_s;
        }
    } else if (strcmp(operating_mode, "WHEEL_SPEED_STOPPING") == 0) {
        snprintf(wheel_speed_test_status, sizeof(wheel_speed_test_status), "STOPPING");
    }
    if (finish_speed_stop_this_cycle) {
        snprintf(wheel_speed_test_status, sizeof(wheel_speed_test_status), "%s",
                 speed_stop_reached ? "STOPPED" : "STOP_TIMEOUT");
        snprintf(operating_mode, sizeof(operating_mode), "SAFE");
        wheel_speed_target_rev_s = 0.0f;
        wheel_speed_test_deadline_us = 0;
        wheel_speed_stop_deadline_us = 0;
    }

    const char *reported_mode = operating_mode;
    if (output.mode == CUBE_CONTROL_RATE_DAMP &&
        strcmp(operating_mode, "SAFE") != 0 &&
        strcmp(operating_mode, "STANDBY") != 0) {
        reported_mode = "RATE_DAMP";
    }
    cJSON *packet = cJSON_CreateObject();
    cJSON_AddNumberToObject(packet, "protocol_version", 1);
    cJSON_AddStringToObject(packet, "source", "cubesat_esp32");
    cJSON_AddStringToObject(packet, "message_type", "actuator_telemetry");
    cJSON_AddNumberToObject(packet, "sequence", ++telemetry_sequence);
    cJSON_AddNumberToObject(packet, "simulation_time_s",
                            cJSON_IsNumber(sim_time_json) ? sim_time_json->valuedouble : 0.0);
    cJSON_AddStringToObject(packet, "mode", reported_mode);
    const char *system_state = output.mode == CUBE_CONTROL_FAULT ? "FAULT" : "NOMINAL";
#if CONFIG_CUBESAT_MOTEUS_ENABLED
    if (!moteus_online || !moteus_command_ok) system_state = "ACTUATOR_FAULT";
#endif
    cJSON_AddStringToObject(packet, "system_state", system_state);
    cJSON_AddStringToObject(packet, "fault", fault_name(output.fault));
    cJSON_AddBoolToObject(packet, "target_valid", target_valid);
    cJSON_AddStringToObject(packet, "camera_frame_source", camera_frame_source);
    if (camera_frame_sequence_valid) {
        cJSON_AddNumberToObject(packet, "camera_frame_sequence",
                                (double)camera_frame_sequence);
    } else {
        cJSON_AddNullToObject(packet, "camera_frame_sequence");
    }
    cJSON_AddNumberToObject(packet, "camera_error_x_px",
                            target_valid ? target_error_px : 0.0f);
    cJSON_AddNumberToObject(packet, "camera_error_y_px",
                            target_valid ? target_error_y_px : 0.0f);
    cJSON_AddNumberToObject(packet, "camera_focal_length_x_px", focal_length_px);
    if (camera_age_valid) {
        cJSON_AddNumberToObject(packet, "camera_age_s", camera_age);
    } else {
        cJSON_AddNullToObject(packet, "camera_age_s");
    }
    cJSON_AddBoolToObject(packet, "imu_valid", imu_valid);
    if (imu_age_valid) {
        cJSON_AddNumberToObject(packet, "imu_age_s", imu_age);
    } else {
        cJSON_AddNullToObject(packet, "imu_age_s");
    }
    if (target_valid) {
        cJSON_AddNumberToObject(packet, "target_error_rad", error_rad);
    } else {
        cJSON_AddNullToObject(packet, "target_error_rad");
    }
    cJSON_AddNumberToObject(packet, "gyro_x_rad_s", isfinite(gyro_x) ? gyro_x : 0.0f);
    cJSON_AddNumberToObject(packet, "gyro_y_rad_s", isfinite(gyro_y) ? gyro_y : 0.0f);
    cJSON_AddNumberToObject(packet, "gyro_z_rad_s", isfinite(gyro_z) ? gyro_z : 0.0f);
    cJSON_AddNumberToObject(packet, "yaw_rate_rad_s", isfinite(yaw_rate) ? yaw_rate : 0.0f);
    if (attitude_valid) {
        cJSON_AddNumberToObject(packet, "yaw_estimate_rad", yaw_estimator.yaw_rad);
    } else {
        cJSON_AddNullToObject(packet, "yaw_estimate_rad");
    }
    cJSON_AddBoolToObject(packet, "yaw_estimate_valid", attitude_valid);
    if (yaw_step_target_valid) {
        cJSON_AddNumberToObject(packet, "yaw_step_target_rad", yaw_step_target_rad);
    } else {
        cJSON_AddNullToObject(packet, "yaw_step_target_rad");
    }
    if (yaw_step_error_valid) {
        cJSON_AddNumberToObject(packet, "yaw_step_error_rad", yaw_step_error_rad);
    } else {
        cJSON_AddNullToObject(packet, "yaw_step_error_rad");
    }
    cJSON_AddStringToObject(packet, "yaw_step_status", yaw_step_status);
    cJSON_AddNumberToObject(packet, "yaw_step_elapsed_s",
        yaw_step_started_us > 0
            ? (float)(cycle_now_us - yaw_step_started_us) / 1000000.0f : 0.0f);
    const char *attitude_source_name = "INVALID";
    if (attitude_source == CUBE_ATTITUDE_MAG_FUSED) {
        attitude_source_name = "MAG_FUSED";
    } else if (attitude_source == CUBE_ATTITUDE_GYRO_ONLY) {
        attitude_source_name = "GYRO_ONLY";
    }
    cJSON_AddStringToObject(packet, "yaw_estimate_source", attitude_source_name);
    cJSON_AddNumberToObject(packet, "accel_x_m_s2", isfinite(accel_x) ? accel_x : 0.0f);
    cJSON_AddNumberToObject(packet, "accel_y_m_s2", isfinite(accel_y) ? accel_y : 0.0f);
    cJSON_AddNumberToObject(packet, "accel_z_m_s2", isfinite(accel_z) ? accel_z : 0.0f);
    cJSON_AddBoolToObject(packet, "mag_valid", mag_valid);
    if (mag_valid) {
        cJSON_AddStringToObject(packet, "mag_frame", mag_frame);
        cJSON_AddNumberToObject(packet, "mag_x_uT", mag_x);
        cJSON_AddNumberToObject(packet, "mag_y_uT", mag_y);
        cJSON_AddNumberToObject(packet, "mag_z_uT", mag_z);
        cJSON_AddNumberToObject(packet, "mag_age_s", mag_age);
    } else {
        cJSON_AddNullToObject(packet, "mag_frame");
        cJSON_AddNullToObject(packet, "mag_x_uT");
        cJSON_AddNullToObject(packet, "mag_y_uT");
        cJSON_AddNullToObject(packet, "mag_z_uT");
        cJSON_AddNullToObject(packet, "mag_age_s");
    }
    cJSON_AddNumberToObject(packet, "body_torque_nm", output.body_torque_nm);
    if (speed_command_active) {
        cJSON_AddNumberToObject(packet, "motor_speed_target_rev_s",
                                wheel_speed_target_rev_s);
    } else {
        cJSON_AddNullToObject(packet, "motor_speed_target_rev_s");
    }
    if (measured_motor_speed_valid) {
        cJSON_AddNumberToObject(packet, "motor_speed_measured_rev_s",
                                measured_motor_speed_rev_s);
    } else {
        cJSON_AddNullToObject(packet, "motor_speed_measured_rev_s");
    }
    if (isfinite(motor_speed_error_percent)) {
        cJSON_AddNumberToObject(packet, "motor_speed_error_percent",
                                motor_speed_error_percent);
    } else {
        cJSON_AddNullToObject(packet, "motor_speed_error_percent");
    }
    cJSON_AddStringToObject(packet, "motor_speed_test_status",
                            wheel_speed_test_status);
    if (isfinite(motor_speed_test_elapsed_s)) {
        cJSON_AddNumberToObject(packet, "motor_speed_test_elapsed_s",
                                motor_speed_test_elapsed_s);
    } else {
        cJSON_AddNullToObject(packet, "motor_speed_test_elapsed_s");
    }
#if CONFIG_CUBESAT_MOTEUS_ENABLED
    cJSON_AddBoolToObject(packet, "moteus_enabled", true);
    cJSON_AddBoolToObject(packet, "moteus_link_valid", moteus_command_ok);
    cJSON_AddBoolToObject(packet, "moteus_arm_active", cube_moteus_arm_active());
    if (moteus_reply_received) {
        cJSON_AddNumberToObject(packet, "moteus_mode", moteus_status.mode);
        cJSON_AddNumberToObject(packet, "moteus_fault", moteus_status.fault);
        cJSON_AddNumberToObject(packet, "moteus_position_rev", moteus_status.position_rev);
        cJSON_AddNumberToObject(packet, "moteus_velocity_rev_s", moteus_status.velocity_rev_s);
        cJSON_AddNumberToObject(packet, "moteus_motor_torque_nm", moteus_status.motor_torque_nm);
        cJSON_AddNumberToObject(packet, "moteus_commanded_motor_torque_nm",
                                moteus_status.commanded_motor_torque_nm);
    } else {
        cJSON_AddNullToObject(packet, "moteus_mode");
        cJSON_AddNullToObject(packet, "moteus_fault");
        cJSON_AddNullToObject(packet, "moteus_position_rev");
        cJSON_AddNullToObject(packet, "moteus_velocity_rev_s");
        cJSON_AddNullToObject(packet, "moteus_motor_torque_nm");
        cJSON_AddNullToObject(packet, "moteus_commanded_motor_torque_nm");
    }
#else
    cJSON_AddBoolToObject(packet, "moteus_enabled", false);
    cJSON_AddBoolToObject(packet, "moteus_link_valid", false);
    cJSON_AddBoolToObject(packet, "moteus_arm_active", false);
#endif
    if (have_command_ack) {
        cJSON *ack = cJSON_CreateObject();
        cJSON_AddNumberToObject(ack, "command_id", (double)acknowledged_command_id);
        cJSON_AddStringToObject(ack, "status", command_ack_status);
        cJSON_AddStringToObject(ack, "reason", command_ack_reason);
        cJSON_AddItemToObject(packet, "command_ack", ack);
    }
    char *serialized = cJSON_PrintUnformatted(packet);
    cJSON_Delete(packet);
    cJSON_Delete(root);

    const bool sent = serialized != NULL &&
        http_request("/api/actuator", serialized, &response);
#if CONFIG_CUBESAT_CAMERA_ENABLED
    if (sent && esp_timer_get_time() - last_preview_upload_us >= 1000000) {
        if (cube_xiao_camera_copy_preview(camera_preview_upload_buffer,
                                          sizeof(camera_preview_upload_buffer))) {
            if (!http_post_camera_preview(camera_preview_upload_buffer,
                                          sizeof(camera_preview_upload_buffer))) {
                ESP_LOGW(TAG, "Camera preview upload failed");
            } else {
                last_preview_upload_us = esp_timer_get_time();
            }
        }
    }
#endif
    if (sent) {
        have_command_ack = false;
    }
    if (serialized != NULL) {
        cJSON_free(serialized);
    }
    return sent;
}

static void wifi_event_handler(void *arg, esp_event_base_t event_base,
                               int32_t event_id, void *event_data)
{
    (void)arg;
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        xEventGroupClearBits(wifi_event_group, WIFI_CONNECTED_BIT);
        if (wifi_retry_count++ < 8) {
            esp_wifi_connect();
        } else {
            xEventGroupSetBits(wifi_event_group, WIFI_FAIL_BIT);
        }
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        wifi_retry_count = 0;
        xEventGroupSetBits(wifi_event_group, WIFI_CONNECTED_BIT);
    }
    (void)event_data;
}

static void connect_wifi(void)
{
    wifi_event_group = xEventGroupCreate();
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_sta();
    const wifi_init_config_t init_config = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&init_config));
    ESP_ERROR_CHECK(esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID,
                                                &wifi_event_handler, NULL));
    ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP,
                                                &wifi_event_handler, NULL));
    wifi_config_t wifi_config = {0};
    snprintf((char *)wifi_config.sta.ssid, sizeof(wifi_config.sta.ssid), "%s",
             CONFIG_CUBESAT_WIFI_SSID);
    snprintf((char *)wifi_config.sta.password, sizeof(wifi_config.sta.password), "%s",
             CONFIG_CUBESAT_WIFI_PASSWORD);
    wifi_config.sta.threshold.authmode = WIFI_AUTH_OPEN;
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());
    const EventBits_t bits = xEventGroupWaitBits(wifi_event_group,
        WIFI_CONNECTED_BIT | WIFI_FAIL_BIT, pdFALSE, pdFALSE, pdMS_TO_TICKS(20000));
    if ((bits & WIFI_CONNECTED_BIT) == 0) {
        ESP_LOGE(TAG, "Wi-Fi connection failed; simulation watchdog will keep body torque at zero");
    }
}

void app_main(void)
{
    esp_err_t nvs_status = nvs_flash_init();
    if (nvs_status == ESP_ERR_NVS_NO_FREE_PAGES ||
        nvs_status == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        nvs_status = nvs_flash_init();
    }
    ESP_ERROR_CHECK(nvs_status);
    controller_config = cube_yaw_controller_default_config();
#if CONFIG_CUBESAT_CAMERA_ENABLED
    camera_online = cube_xiao_camera_init();
    if (!camera_online) {
        ESP_LOGE(TAG, "Camera unavailable; active control will fail closed");
    }
#endif
#if CONFIG_CUBESAT_MPU6050_ENABLED
    imu_online = cube_mpu6050_init();
    if (!imu_online) {
        ESP_LOGE(TAG, "MPU-6050 unavailable; control will fail closed");
    }
#endif
#if CONFIG_CUBESAT_LIS3MDL_ENABLED
    magnetometer_online = cube_lis3mdl_init();
    if (!magnetometer_online) {
        ESP_LOGW(TAG, "LIS3MDL unavailable; continuing without magnetic telemetry");
    }
#endif
#if CONFIG_CUBESAT_MOTEUS_ENABLED
    /* Physical actuation is armed only after both physical sensors initialize.
     * The default Wokwi/SIL profile cannot enable this Kconfig option. */
    moteus_online = camera_online && imu_online && cube_moteus_uart_init();
    if (!moteus_online) {
        ESP_LOGE(TAG, "moteus actuator unavailable/disarmed; physical torque is inhibited");
    }
#endif
    ESP_LOGI(TAG, "CubeSat ESP32-S3 simulation firmware starting; initial mode=%s",
             operating_mode);
    connect_wifi();
    while ((xEventGroupGetBits(wifi_event_group) & WIFI_CONNECTED_BIT) == 0) {
        if ((xEventGroupGetBits(wifi_event_group) & WIFI_FAIL_BIT) != 0) {
            xEventGroupClearBits(wifi_event_group, WIFI_FAIL_BIT);
            wifi_retry_count = 0;
            ESP_LOGW(TAG, "Retrying Wi-Fi connection");
            esp_wifi_connect();
        }
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
    ESP_LOGI(TAG, "Simulation bridge at %s", CONFIG_CUBESAT_SIM_BRIDGE_URL);
    TickType_t last_wake_tick = xTaskGetTickCount();
    const TickType_t control_period_ticks = pdMS_TO_TICKS(CONTROL_PERIOD_MS);
    uint32_t cycles_since_stack_report = 0;
    while (true) {
        if (!run_control_cycle()) {
            ESP_LOGW(TAG, "No actuator update this cycle");
        }
        if (++cycles_since_stack_report >= 250) {
            cycles_since_stack_report = 0;
            ESP_LOGD(TAG, "Main/control task minimum free stack: %u bytes",
                     (unsigned)uxTaskGetStackHighWaterMark(NULL));
        }
        if (xTaskDelayUntil(&last_wake_tick, control_period_ticks) == pdFALSE) {
            /* Rebase after a late cycle instead of rapidly catching up. */
            last_wake_tick = xTaskGetTickCount();
            ESP_LOGW(TAG, "Control cycle exceeded its 20 ms deadline");
        }
    }
}
