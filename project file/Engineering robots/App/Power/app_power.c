/**
  ******************************************************************************
  * @file    app_power.c
  * @brief   0x31 功率/能量反馈、底盘策略与旧代理预算。
  *
  * 0x31 定义来自 docs/RULES_REQUIREMENTS.md：标准数据帧、DLC=4；
  * 字节 0–1 小端 int16 底盘功率（0.1 W）；字节 2–3 小端 int16 缓冲能量（0.1 J）。
  * 负值保留，不取绝对值、不截成 0。
  *
  * 并发约定：
   *  - ISR 只更新独立 raw 帧（短临界区）；
   *  - update 在局部变量完成解析/策略，最后一次短临界区发布整体快照；
   *  - 所有读取接口只在短临界区复制快照，不做重试自旋。
  ******************************************************************************
  */
#include "Power/app_power.h"

#include <math.h>
#include <string.h>

#include "Common/app_critical.h"
#include "Common/app_time.h"

#define POWER_SCALE_0_1 0.1f

typedef struct {
    uint32_t sequence;
    uint8_t data[8];
    uint8_t dlc;
    uint8_t bus;
    uint32_t arrival_ms;
    bool valid;
} power_raw_frame_t;

typedef struct {
    app_power_feedback_t feedback;
    app_power_policy_t policy;
    app_power_budget_t budget;
    app_status_t status;
} power_snapshot_t;

static power_raw_frame_t g_raw;
static power_snapshot_t g_snapshot;
static uint32_t g_processed_sequence;
static uint32_t g_invalid_frames;
static uint32_t g_bus_mismatch;
static app_can_bus_t g_source_bus = APP_CAN_BUS_NONE;
static bool g_was_online;

#ifdef APP_POWER_TEST
static void (*g_test_before_publish)(void);

void app_power_test_set_before_publish(void (*hook)(void))
{
    g_test_before_publish = hook;
}
#endif

app_status_t app_power_init(uint32_t now_ms)
{
    power_snapshot_t initial;
    (void)memset(&initial, 0, sizeof(initial));
    initial.feedback.source_bus = APP_CAN_BUS_NONE;
    initial.feedback.frame_arrival_ms = APP_TIME_NEVER;
    initial.feedback.status = APP_STATUS_UNCONFIGURED;
    initial.feedback.updated_ms = now_ms;
    initial.policy.updated_ms = now_ms;
    initial.budget.updated_ms = now_ms;
    initial.budget.available_power_w = 0.0f;
    initial.budget.source_valid = false;
    initial.budget.valid = false;
    initial.status = APP_STATUS_UNCONFIGURED;

    uint32_t key = app_critical_enter();
    (void)memset(&g_raw, 0, sizeof(g_raw));
    g_raw.arrival_ms = APP_TIME_NEVER;
    g_processed_sequence = 0u;
    g_invalid_frames = 0u;
    g_bus_mismatch = 0u;
    g_was_online = false;
    g_source_bus = app_config_get()->power.source_bus;
    g_snapshot = initial;
    app_critical_exit(key);
    return initial.status;
}

void app_power_on_can_frame(uint8_t bus, uint32_t id, const uint8_t *data,
                            uint8_t dlc, uint32_t arrival_ms)
{
    if (id != APP_POWER_CAN_ID || data == NULL) {
        return;
    }

    uint32_t key = app_critical_enter();
    if (g_source_bus == APP_CAN_BUS_NONE || bus != (uint8_t)g_source_bus) {
        g_bus_mismatch++;
        app_critical_exit(key);
        return;
    }
    if (dlc != APP_POWER_FRAME_DLC) {
        g_invalid_frames++;
        app_critical_exit(key);
        return;
    }

    g_raw.sequence++;
    g_raw.bus = bus;
    g_raw.dlc = dlc;
    g_raw.arrival_ms = arrival_ms;
    g_raw.valid = true;
    (void)memcpy(g_raw.data, data, APP_POWER_FRAME_DLC);
    app_critical_exit(key);
}

static app_power_budget_t make_old_proxy_budget(const app_power_config_t *power,
                                                uint32_t now_ms)
{
    app_power_budget_t budget;
    (void)memset(&budget, 0, sizeof(budget));
    budget.updated_ms = now_ms;
    budget.available_power_w = 0.0f;
    budget.source_valid = false;
    if (power->valid && isfinite(power->max_total_output_norm) &&
        power->max_total_output_norm > 0.0f) {
        budget.max_total_output_norm = power->max_total_output_norm;
        budget.valid = true;
    }
    return budget;
}

app_status_t app_power_update(uint32_t now_ms)
{
    const app_power_config_t *power = &app_config_get()->power;
    power_raw_frame_t raw;
    uint32_t invalid;
    uint32_t bus_mismatch;
    uint32_t key = app_critical_enter();
    raw = g_raw;
    invalid = g_invalid_frames;
    bus_mismatch = g_bus_mismatch;
    app_power_feedback_t feedback = g_snapshot.feedback;
    app_critical_exit(key);

    if (raw.sequence != g_processed_sequence) {
        g_processed_sequence = raw.sequence;
        if (raw.valid && raw.dlc == APP_POWER_FRAME_DLC &&
            raw.bus == (uint8_t)power->source_bus) {
            const int16_t power_raw =
                (int16_t)((uint16_t)raw.data[0] | ((uint16_t)raw.data[1] << 8));
            const int16_t energy_raw =
                (int16_t)((uint16_t)raw.data[2] | ((uint16_t)raw.data[3] << 8));
            feedback.measured_chassis_power_w = (float)power_raw * POWER_SCALE_0_1;
            feedback.buffer_energy_j = (float)energy_raw * POWER_SCALE_0_1;
            feedback.frame_arrival_ms = raw.arrival_ms;
            feedback.source_bus = (app_can_bus_t)raw.bus;
            feedback.frame_valid = true;
            feedback.frame_count++;
        }
    }

    feedback.invalid_frame_count = invalid;
    feedback.config_valid = power->source_bus_valid &&
                            power->source_bus != APP_CAN_BUS_NONE &&
                            power->source_timeout_valid &&
                            (power->source_timeout_ms > 0u);
    feedback.limits_valid = power->chassis_power_limit_valid &&
                            isfinite(power->chassis_power_limit_w) &&
                            (power->chassis_power_limit_w > 0.0f) &&
                            power->buffer_energy_limit_valid &&
                            isfinite(power->buffer_energy_limit_j) &&
                            (power->buffer_energy_limit_j > 0.0f);
    feedback.hardware_verified = power->hardware_verified;
    feedback.source_bus = power->source_bus;
    feedback.updated_ms = now_ms;
    feedback.source_online =
        feedback.config_valid && feedback.frame_valid &&
        app_time_is_fresh(now_ms, feedback.frame_arrival_ms,
                          power->source_timeout_ms);
    if (g_was_online && !feedback.source_online) {
        feedback.timeout_count++;
    }
    g_was_online = feedback.source_online;

    feedback.error_flags = APP_POWER_ERR_NONE;
    if (!feedback.config_valid) { feedback.error_flags |= APP_POWER_ERR_CONFIG; }
    if (!feedback.frame_valid) { feedback.error_flags |= APP_POWER_ERR_NO_FRAME; }
    if (feedback.frame_valid && !feedback.source_online) {
        feedback.error_flags |= APP_POWER_ERR_TIMEOUT;
    }
    if (invalid > 0u) { feedback.error_flags |= APP_POWER_ERR_BAD_FRAME; }
    if (bus_mismatch > 0u) { feedback.error_flags |= APP_POWER_ERR_BUS_MISMATCH; }
    if (!feedback.hardware_verified) { feedback.error_flags |= APP_POWER_ERR_UNVERIFIED; }

    app_status_t status;
    if (!feedback.config_valid) {
        status = APP_STATUS_UNCONFIGURED;
    } else if (!feedback.hardware_verified) {
        status = APP_STATUS_NOT_READY;
    } else if (!feedback.frame_valid || !feedback.source_online) {
        status = APP_STATUS_OFFLINE;
    } else {
        status = APP_STATUS_OK;
    }
    feedback.status = status;

    app_power_policy_t policy;
    (void)memset(&policy, 0, sizeof(policy));
    policy.updated_ms = now_ms;
    policy.measured_power_w = feedback.measured_chassis_power_w;
    policy.buffer_energy_j = feedback.buffer_energy_j;
    policy.chassis_power_limit_w = power->chassis_power_limit_w;
    policy.buffer_energy_limit_j = power->buffer_energy_limit_j;
    policy.chassis_scale = 0.0f;
    policy.source_online = feedback.source_online;
    policy.source_ready = feedback.config_valid && feedback.limits_valid &&
                          feedback.hardware_verified && feedback.source_online;
    if (!policy.source_ready) {
        policy.state = APP_POWER_POLICY_UNAVAILABLE;
    } else {
        policy.allowed_power_w = power->chassis_power_limit_w;
        policy.power_margin_w = power->chassis_power_limit_w -
                                feedback.measured_chassis_power_w;
        policy.over_power = feedback.measured_chassis_power_w >
                            power->chassis_power_limit_w;
        policy.low_energy = power->low_buffer_energy_valid &&
                            isfinite(power->low_buffer_energy_j) &&
                            feedback.buffer_energy_j <= power->low_buffer_energy_j;
        policy.buffer_margin_j = power->low_buffer_energy_valid
                                     ? feedback.buffer_energy_j - power->low_buffer_energy_j
                                     : feedback.buffer_energy_j;
        policy.state = APP_POWER_POLICY_NORMAL;
        policy.chassis_scale = 1.0f;
        if (policy.over_power) {
            policy.state = APP_POWER_POLICY_LIMITED;
            policy.chassis_scale = power->chassis_power_limit_w /
                                   feedback.measured_chassis_power_w;
            if (policy.chassis_scale < 0.0f) { policy.chassis_scale = 0.0f; }
            if (policy.chassis_scale > 1.0f) { policy.chassis_scale = 1.0f; }
        }
        if (policy.low_energy) {
            policy.state = APP_POWER_POLICY_LOW_ENERGY;
            policy.chassis_scale = 0.0f;
        }
    }

    app_power_budget_t budget = make_old_proxy_budget(power, now_ms);

#ifdef APP_POWER_TEST
    if (g_test_before_publish != NULL) {
        g_test_before_publish();
    }
#endif

    /* Single short publication: readers see either the previous complete
     * snapshot or this complete snapshot, never a partially written one. */
    key = app_critical_enter();
    g_snapshot.feedback = feedback;
    g_snapshot.policy = policy;
    g_snapshot.budget = budget;
    g_snapshot.status = status;
    g_source_bus = power->source_bus;
    app_critical_exit(key);
    return status;
}

app_status_t app_power_get_feedback(app_power_feedback_t *out)
{
    if (out == NULL) { return APP_STATUS_INVALID_PARAM; }
    const uint32_t key = app_critical_enter();
    *out = g_snapshot.feedback;
    app_critical_exit(key);
    return out->status;
}

app_status_t app_power_get_policy(app_power_policy_t *out)
{
    if (out == NULL) { return APP_STATUS_INVALID_PARAM; }
    app_status_t status;
    const uint32_t key = app_critical_enter();
    *out = g_snapshot.policy;
    status = g_snapshot.status;
    app_critical_exit(key);
    return out->source_ready ? APP_STATUS_OK : status;
}

app_status_t app_power_get_budget(app_power_budget_t *out)
{
    if (out == NULL) { return APP_STATUS_INVALID_PARAM; }
    const uint32_t key = app_critical_enter();
    *out = g_snapshot.budget;
    app_critical_exit(key);
    return out->valid ? APP_STATUS_OK : APP_STATUS_UNCONFIGURED;
}

app_status_t app_power_limit_scale(float requested_total_norm, float *scale_out)
{
    if (scale_out == NULL) { return APP_STATUS_INVALID_PARAM; }
    *scale_out = 0.0f;

    app_power_budget_t budget;
    const uint32_t key = app_critical_enter();
    budget = g_snapshot.budget;
    app_critical_exit(key);

    /* The old proxy budget is independent of the 0x31 chassis source. */
    if (!budget.valid) { return APP_STATUS_UNCONFIGURED; }
    if (!isfinite(requested_total_norm)) { return APP_STATUS_INVALID_PARAM; }
    if (requested_total_norm <= 0.0f) { *scale_out = 1.0f; return APP_STATUS_OK; }
    if (requested_total_norm <= budget.max_total_output_norm) {
        *scale_out = 1.0f;
        return APP_STATUS_OK;
    }
    *scale_out = app_clampf(budget.max_total_output_norm / requested_total_norm,
                            0.0f, 1.0f);
    return APP_STATUS_OK;
}

app_status_t app_power_status(void)
{
    app_status_t status;
    const uint32_t key = app_critical_enter();
    status = g_snapshot.status;
    app_critical_exit(key);
    return status;
}
