/**
  ******************************************************************************
  * @file    dbus.c
  * @brief   DBUS 帧位域解析与逻辑输入映射。
  *
  * 位域顺序（协议 V1.4 附录，低字节在前）：
  *   ch0[0..10] ch1[11..21] ch2[22..32] ch3[33..43]
  *   S1[44..45] S2[46..47]
  *   byte6..7 鼠标 X，byte8..9 Y，byte10..11 Z，
  *   byte12 左键，byte13 右键，byte14..15 键盘位。
  ******************************************************************************
  */
#include "Remote/dbus.h"

#include <math.h>
#include <stddef.h>
#include <string.h>

#define DBUS_CHANNEL_HALF_RANGE 660.0f

static bool channel_valid(int16_t value)
{
    return value >= DBUS_CHANNEL_MIN && value <= DBUS_CHANNEL_MAX;
}

static bool switch_valid(uint8_t value)
{
    return value == DBUS_SWITCH_UP || value == DBUS_SWITCH_DOWN || value == DBUS_SWITCH_MID;
}

app_status_t dbus_decode(const uint8_t *frame, size_t size, uint32_t sample_ms,
                         app_dbus_frame_t *out)
{
    app_dbus_frame_t parsed = {0};

    if (out == NULL) {
        return APP_STATUS_INVALID_PARAM;
    }
    *out = parsed;
    if (frame == NULL || size != DBUS_FRAME_SIZE) {
        return APP_STATUS_INVALID_PARAM;
    }

    parsed.channel[0] = (int16_t)(((uint32_t)frame[0] | ((uint32_t)frame[1] << 8)) & 0x7FFu);
    parsed.channel[1] = (int16_t)((((uint32_t)frame[1] >> 3) | ((uint32_t)frame[2] << 5)) & 0x7FFu);
    parsed.channel[2] = (int16_t)((((uint32_t)frame[2] >> 6) | ((uint32_t)frame[3] << 2) |
                                  ((uint32_t)frame[4] << 10)) & 0x7FFu);
    parsed.channel[3] = (int16_t)((((uint32_t)frame[4] >> 1) | ((uint32_t)frame[5] << 7)) & 0x7FFu);
    parsed.switch_s1 = (uint8_t)((frame[5] >> 4) & 0x03u);
    parsed.switch_s2 = (uint8_t)((frame[5] >> 6) & 0x03u);

    parsed.mouse_x = (int16_t)((uint16_t)frame[6] | ((uint16_t)frame[7] << 8));
    parsed.mouse_y = (int16_t)((uint16_t)frame[8] | ((uint16_t)frame[9] << 8));
    parsed.mouse_z = (int16_t)((uint16_t)frame[10] | ((uint16_t)frame[11] << 8));
    parsed.mouse_press_left = frame[12];
    parsed.mouse_press_right = frame[13];
    parsed.keyboard = (uint16_t)((uint16_t)frame[14] | ((uint16_t)frame[15] << 8));
    parsed.sample_ms = sample_ms;

    for (unsigned int i = 0; i < 4u; i++) {
        if (!channel_valid(parsed.channel[i])) {
            return APP_STATUS_INVALID_PARAM;
        }
    }
    if (!switch_valid(parsed.switch_s1) || !switch_valid(parsed.switch_s2)) {
        return APP_STATUS_INVALID_PARAM;
    }
    if (parsed.mouse_press_left > 1u || parsed.mouse_press_right > 1u) {
        return APP_STATUS_INVALID_PARAM;
    }

    *out = parsed;
    return APP_STATUS_OK;
}

static bool mapping_config_valid(const app_remote_config_t *config)
{
    if (config == NULL) {
        return false;
    }
    if (!config->valid || !config->mapping_valid || !config->channel_map_valid ||
        !config->switch_map_valid) {
        return false;
    }
    if (config->channel_forward > 3u || config->channel_turn > 3u ||
        config->channel_aux1 > 3u || config->channel_aux2 > 3u) {
        return false;
    }
    if (config->gear_switch > 1u || config->enable_switch > 1u) {
        return false;
    }
    const int8_t signs[4] = {config->sign_forward, config->sign_turn, config->sign_aux1,
                             config->sign_aux2};
    for (unsigned int i = 0; i < 4u; i++) {
        if (signs[i] != 1 && signs[i] != -1) {
            return false;
        }
    }
    return true;
}

static float normalize_axis(int16_t value, float deadband)
{
    float normalized = ((float)value - (float)DBUS_CHANNEL_MID) / DBUS_CHANNEL_HALF_RANGE;
    normalized = app_clampf(normalized, -1.0f, 1.0f);
    if (fabsf(normalized) <= deadband) {
        normalized = 0.0f;
    }
    return normalized;
}

static app_mode_t gear_mode(uint8_t gear_position)
{
    if (gear_position == DBUS_SWITCH_UP) {
        return APP_MODE_VEHICLE_UP;
    }
    if (gear_position == DBUS_SWITCH_DOWN) {
        return APP_MODE_VEHICLE_DOWN;
    }
    return APP_MODE_CHASSIS;
}

app_status_t dbus_to_input(const app_dbus_frame_t *frame,
                           const app_remote_config_t *config,
                           app_remote_input_t *out)
{
    if (frame == NULL || out == NULL) {
        return APP_STATUS_INVALID_PARAM;
    }
    if (!mapping_config_valid(config)) {
        *out = (app_remote_input_t){0};
        return APP_STATUS_UNCONFIGURED;
    }

    const float deadband = config->deadband_valid
                               ? app_clampf(config->channel_deadband, 0.0f, 1.0f)
                               : 0.0f;
    const uint8_t gear_position =
        (config->gear_switch == 0u) ? frame->switch_s1 : frame->switch_s2;
    const uint8_t enable_position =
        (config->enable_switch == 0u) ? frame->switch_s1 : frame->switch_s2;
    if (!switch_valid(gear_position) || !switch_valid(enable_position)) {
        return APP_STATUS_INVALID_PARAM;
    }

    app_remote_input_t input = {0};
    input.forward = (float)config->sign_forward *
                    normalize_axis(frame->channel[config->channel_forward], deadband);
    input.turn = (float)config->sign_turn *
                 normalize_axis(frame->channel[config->channel_turn], deadband);
    input.aux1 = (float)config->sign_aux1 *
                 normalize_axis(frame->channel[config->channel_aux1], deadband);
    input.aux2 = (float)config->sign_aux2 *
                 normalize_axis(frame->channel[config->channel_aux2], deadband);
    input.mode = gear_mode(gear_position);
    /* 简单电平使能：右开关上档请求使能，中/下档停用；下档同时产生复位请求。
     * 这是本方案的新约定，不复用旧工程“下档回中后上档”的重连资格序列。 */
    input.arm_request = enable_position == DBUS_SWITCH_UP;
    input.reset_request = enable_position == DBUS_SWITCH_DOWN;
    input.sample_ms = frame->sample_ms;

    *out = input;
    return APP_STATUS_OK;
}
