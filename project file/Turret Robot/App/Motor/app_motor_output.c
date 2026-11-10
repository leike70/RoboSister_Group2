/**
  ******************************************************************************
  * @file    app_motor_output.c
  * @brief   统一电机请求收集与唯一输出提交入口。
  *
  * 每个控制周期：
  *   1. 调度器调用 app_motor_clear_requests() 清空旧请求；
  *   2. 各应用模块调用 app_motor_request() 写本周期请求；
  *   3. 调度器只调用一次 app_motor_submit()：配置门控 → 保护门控 → 功率预算
  *      → 机构单位换算 → 写每台电机唯一最终命令。
  *
  * 任何环节不满足时最终命令为 disabled，绝不保留上一周期命令。
  * 最终命令只由适配层通过 app_motor_output_get() 读取；首轮没有适配层，因此
  * 不会向 CAN 写入任何报文。
  ******************************************************************************
  */
#include "Motor/app_motor_internal.h"

#include <math.h>
#include <stddef.h>

#include "Common/app_time.h"
#include "Motor/app_unit.h"
#include "Power/app_power.h"
#include "Protection/app_protection.h"
#include "app_config.h"

static app_motor_request_t g_app_motor_requests[APP_MOTOR_COUNT];
static app_motor_output_t g_app_motor_outputs[APP_MOTOR_COUNT];

static bool output_is_norm_mode(app_motor_cmd_mode_t mode)
{
    return (mode == APP_MOTOR_CMD_CURRENT_NORM) || (mode == APP_MOTOR_CMD_VOLTAGE_NORM);
}

static bool is_chassis_slot(int slot)
{
    const app_motor_config_t *config = app_config_motor(slot);
    return config != NULL &&
           (config->role == APP_MOTOR_ROLE_TRACK_LEFT ||
            config->role == APP_MOTOR_ROLE_TRACK_RIGHT);
}

void app_motor_clear_requests(void)
{
    uint32_t index;

    for (index = 0u; index < APP_MOTOR_COUNT; index++) {
        g_app_motor_requests[index].mode = APP_MOTOR_CMD_NONE;
        g_app_motor_requests[index].value = 0.0f;
        g_app_motor_requests[index].valid = false;
    }
}

app_status_t app_motor_request(int slot, app_motor_cmd_mode_t mode, float value)
{
    if ((slot < 0) || (slot >= (int)APP_MOTOR_COUNT) || (mode == APP_MOTOR_CMD_NONE) ||
        !isfinite(value)) {
        return APP_STATUS_INVALID_PARAM;
    }

    if (!g_app_motor_slots[slot].motion_usable) {
        return APP_STATUS_UNCONFIGURED;
    }

    if (mode == APP_MOTOR_CMD_POSITION_RAD) {
        const app_motor_config_t *config = app_config_motor(slot);
        float limited = 0.0f;
        if (app_unit_limit_rad(config, value, &limited) != APP_STATUS_OK) {
            return APP_STATUS_UNCONFIGURED;
        }
        value = limited;
    }

    g_app_motor_requests[slot].mode = mode;
    g_app_motor_requests[slot].value = value;
    g_app_motor_requests[slot].valid = true;
    return APP_STATUS_OK;
}

app_status_t app_motor_request_get(int slot, app_motor_request_t *out)
{
    if ((out == NULL) || (slot < 0) || (slot >= (int)APP_MOTOR_COUNT)) {
        return APP_STATUS_INVALID_PARAM;
    }
    *out = g_app_motor_requests[slot];
    return g_app_motor_requests[slot].valid ? APP_STATUS_OK : APP_STATUS_NOT_READY;
}

/**
 * @brief 把一条应用层请求换算成驱动侧数值。
 * @return true 表示换算成功，driver_value 有效。
 */
static bool convert_request(const app_motor_config_t *config,
                            const app_motor_request_t *request,
                            float power_scale,
                            float *driver_value)
{
    app_status_t status;

    switch (request->mode) {
    case APP_MOTOR_CMD_CURRENT_NORM:
    case APP_MOTOR_CMD_VOLTAGE_NORM:
        *driver_value = app_clampf(request->value * power_scale, -1.0f, 1.0f);
        return true;

    case APP_MOTOR_CMD_SPEED_RAD_S:
        status = app_unit_rad_s_to_rotor_rpm(config, request->value, driver_value);
        return status == APP_STATUS_OK;

    case APP_MOTOR_CMD_POSITION_RAD:
        status = app_unit_rad_to_rotor_rounds(config, request->value, driver_value);
        return status == APP_STATUS_OK;

    case APP_MOTOR_CMD_NONE:
    default:
        return false;
    }
}

app_status_t app_motor_submit(uint32_t now_ms)
{
    const bool output_allowed = app_protection_output_allowed();
    float requested_total_norm = 0.0f;
    float power_scale = 0.0f;
    bool power_ok;
    app_power_policy_t power_policy = {0};
    bool chassis_power_ready;
    uint32_t index;
    int enabled_count = 0;

    /* 没有真实驱动适配层时，不允许任何最终命令被标记为 enabled。 */
    if (!app_motor_adapter_ready()) {
        for (index = 0u; index < APP_MOTOR_COUNT; index++) {
            app_motor_output_t *out = &g_app_motor_outputs[index];
            out->mode = APP_MOTOR_CMD_NONE;
            out->value = 0.0f;
            out->enabled = false;
            out->driver_valid = false;
            out->driver_value = 0.0f;
            out->current_limit_norm = 0.0f;
            out->power_limited = false;
            out->committed_ms = now_ms;
        }
        return APP_STATUS_NOT_IMPLEMENTED;
    }

    /* 1) 功率预算只对归一化电流/电压代理量限幅；速度/位置指令由驱动闭环执行，
     *    其功率约束尚未实现（见 App/Power/README.md）。 */
    for (index = 0u; index < APP_MOTOR_COUNT; index++) {
        if (g_app_motor_requests[index].valid &&
            output_is_norm_mode(g_app_motor_requests[index].mode)) {
            requested_total_norm += fabsf(g_app_motor_requests[index].value);
        }
    }
    power_ok = (app_power_limit_scale(requested_total_norm, &power_scale) == APP_STATUS_OK);
    chassis_power_ready =
        (app_power_get_policy(&power_policy) == APP_STATUS_OK) && power_policy.source_ready;

    /* 2) 每台电机生成本周期唯一最终命令；先全部失效，再逐台填写。 */
    for (index = 0u; index < APP_MOTOR_COUNT; index++) {
        app_motor_output_t *output = &g_app_motor_outputs[index];
        const app_motor_request_t *request = &g_app_motor_requests[index];

        output->mode = APP_MOTOR_CMD_NONE;
        output->value = 0.0f;
        output->enabled = false;
        output->driver_valid = false;
        output->driver_value = 0.0f;
        output->current_limit_norm = 1.0f;
        output->power_limited = false;
        output->committed_ms = now_ms;

        if (!output_allowed) {
            continue;
        }
        if (!request->valid || !g_app_motor_slots[index].motion_usable) {
            continue;
        }

        const bool chassis = is_chassis_slot((int)index);
        const app_motor_config_t *config = app_config_motor((int)index);
        float driver_value = 0.0f;
        float effective_value = request->value;
        float effective_scale = 1.0f;

        if (chassis) {
            /* 底盘只接受 0x31 策略许可；源未配置/未核验/超时/低能量时不放行。 */
            if (!chassis_power_ready || power_policy.chassis_scale <= 0.0f) {
                continue;
            }
            if (request->mode == APP_MOTOR_CMD_POSITION_RAD) {
                /* 位置目标绝不能乘功率系数；受限时直接不输出。 */
                continue;
            }
            effective_scale = power_policy.chassis_scale;
            effective_value = request->value * effective_scale;
            output->power_limited = effective_scale < 1.0f;
        } else if (!power_ok) {
            /* 非底盘沿用旧代理预算；源缺失不把它解释为整车故障。 */
            continue;
        }

        const float driver_scale = chassis ? 1.0f : power_scale;
        if (!convert_request(config, request, driver_scale, &driver_value)) {
            continue;
        }
        if (chassis) {
            /* Re-convert with the actual speed/current value; position is blocked above. */
            app_motor_request_t scaled = *request;
            scaled.value = effective_value;
            if (!convert_request(config, &scaled, 1.0f, &driver_value)) {
                continue;
            }
            output->current_limit_norm = effective_scale;
        }

        output->mode = request->mode;
        output->value = effective_value;
        output->driver_valid = true;
        output->driver_value = driver_value;
        output->enabled = true;
        enabled_count++;
    }

    if (!output_allowed || (enabled_count == 0)) {
        return APP_STATUS_NOT_READY;
    }
    return APP_STATUS_OK;
}

app_status_t app_motor_output_get(int slot, app_motor_output_t *out)
{
    if ((out == NULL) || (slot < 0) || (slot >= (int)APP_MOTOR_COUNT)) {
        return APP_STATUS_INVALID_PARAM;
    }
    *out = g_app_motor_outputs[slot];
    return out->enabled ? APP_STATUS_OK : APP_STATUS_NOT_READY;
}

void app_motor_disable_all(uint32_t now_ms)
{
    uint32_t index;

    app_motor_clear_requests();

    for (index = 0u; index < APP_MOTOR_COUNT; index++) {
        g_app_motor_outputs[index].mode = APP_MOTOR_CMD_NONE;
        g_app_motor_outputs[index].value = 0.0f;
        g_app_motor_outputs[index].enabled = false;
        g_app_motor_outputs[index].driver_valid = false;
        g_app_motor_outputs[index].driver_value = 0.0f;
        g_app_motor_outputs[index].current_limit_norm = 0.0f;
        g_app_motor_outputs[index].power_limited = false;
        g_app_motor_outputs[index].committed_ms = now_ms;
    }
}
