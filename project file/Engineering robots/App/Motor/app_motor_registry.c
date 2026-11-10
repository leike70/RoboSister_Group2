/**
  ******************************************************************************
  * @file    app_motor_registry.c
  * @brief   电机设备注册与反馈快照。
  *
  * 首轮不实例化任何 DJI/达妙电机对象：只按配置表登记“逻辑设备”，反馈通道
  * 标记为未实现。这样未确认配置不会创建会等待 CAN 反馈的硬件对象，也不会
  * 把 0 反馈当成有效测量。
  ******************************************************************************
  */
#include "Motor/app_motor_internal.h"

#include <stddef.h>

#include "Common/app_time.h"
#include "app_config.h"

app_motor_slot_t g_app_motor_slots[APP_MOTOR_COUNT];
int g_app_motor_registered_count = 0;
app_status_t g_app_motor_status = APP_STATUS_UNCONFIGURED;
static bool g_motor_adapter_ready;

app_status_t app_motor_init(uint32_t now_ms)
{
    uint32_t index;

    (void)now_ms;

    g_app_motor_registered_count = 0;
    g_motor_adapter_ready = false;

    for (index = 0u; index < APP_MOTOR_COUNT; index++) {
        app_motor_slot_t *slot = &g_app_motor_slots[index];
        const app_motor_config_t *config = app_config_motor((int)index);

        slot->status = APP_STATUS_UNCONFIGURED;
        slot->registered = false;
        slot->comms_usable = false;
        slot->motion_usable = false;
        slot->missing_flags = 0xFFFFFFFFu;
        slot->last_feedback_ms = APP_TIME_NEVER;
        slot->feedback.position_rad = 0.0f;
        slot->feedback.velocity_rad_s = 0.0f;
        slot->feedback.current_norm = 0.0f;
        slot->feedback.valid = false;
        slot->feedback.timestamp_ms = APP_TIME_NEVER;

        if (config == NULL) {
            continue;
        }

        slot->comms_usable = app_config_motor_comms_usable((int)index);
        slot->motion_usable = app_config_motor_motion_usable((int)index);
        slot->missing_flags = app_config_motor_missing((int)index);
        slot->registered = slot->comms_usable;

        if (slot->registered) {
            /* 硬件适配层（DJI/达妙驱动对象）尚未实现，登记后也只能报告未实现。 */
            slot->status = APP_STATUS_NOT_IMPLEMENTED;
            g_app_motor_registered_count++;
        }
    }

    if (g_app_motor_registered_count == 0) {
        g_app_motor_status = APP_STATUS_UNCONFIGURED;
    } else {
        g_app_motor_status = APP_STATUS_NOT_IMPLEMENTED;
    }

    return g_app_motor_status;
}

app_status_t app_motor_update_feedback(uint32_t now_ms)
{
    (void)now_ms;

    if (g_app_motor_registered_count == 0) {
        g_app_motor_status = APP_STATUS_UNCONFIGURED;
        return g_app_motor_status;
    }

    /* 没有注册任何硬件对象，也没有 CAN 反馈适配：不伪造快照。 */
    g_app_motor_status = APP_STATUS_NOT_IMPLEMENTED;
    return g_app_motor_status;
}

app_status_t app_motor_feedback_get(int slot, app_motor_feedback_t *out)
{
    if ((out == NULL) || (slot < 0) || (slot >= (int)APP_MOTOR_COUNT)) {
        return APP_STATUS_INVALID_PARAM;
    }

    *out = g_app_motor_slots[slot].feedback;

    if (!g_app_motor_slots[slot].registered) {
        return APP_STATUS_UNCONFIGURED;
    }
    if (!out->valid) {
        return APP_STATUS_NOT_IMPLEMENTED;
    }
    return APP_STATUS_OK;
}

app_status_t app_motor_device_get(int slot, app_motor_device_t *out)
{
    if ((out == NULL) || (slot < 0) || (slot >= (int)APP_MOTOR_COUNT)) {
        return APP_STATUS_INVALID_PARAM;
    }

    out->status = g_app_motor_slots[slot].status;
    out->registered = g_app_motor_slots[slot].registered;
    out->comms_usable = g_app_motor_slots[slot].comms_usable;
    out->motion_usable = g_app_motor_slots[slot].motion_usable;
    out->missing_flags = g_app_motor_slots[slot].missing_flags;
    out->last_feedback_ms = g_app_motor_slots[slot].last_feedback_ms;

    return out->status;
}

int app_motor_registered_count(void)
{
    return g_app_motor_registered_count;
}

void app_motor_set_adapter_ready(bool ready)
{
    g_motor_adapter_ready = ready;
}

bool app_motor_adapter_ready(void)
{
    return g_motor_adapter_ready;
}

app_status_t app_motor_status(void)
{
    return g_app_motor_status;
}
