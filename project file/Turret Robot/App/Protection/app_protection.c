/**
  ******************************************************************************
  * @file    app_protection.c
  * @brief   整车保护与运行许可（两车公共）。
  *
  * 每个周期只读各模块状态并给出唯一输出许可；不触发动作、不创建 RTOS 对象、
  * 不阻塞。首轮配置未确认、遥控离线、无设备反馈，因此永远不会允许输出。
  *
  * 故障策略：
  *  - raise() 同时置 active 与 latched，并立即撤销输出许可；
  *  - 活动故障未清零时 clear_latched() 拒绝清除并返回 false；
  *  - 故障不会因条件消失自动解除，必须由上层监督逻辑显式清除。
  ******************************************************************************
  */
#include "Protection/app_protection.h"

#include <string.h>

#include "Motor/app_motor.h"
#include "Power/app_power.h"
#include "Remote/app_remote.h"
#include "app_config.h"

static app_protection_report_t g_report;
static bool g_arm_request;
static bool g_mode_active;
static bool g_estop_active;
static app_status_t g_status = APP_STATUS_NOT_READY;

/* ------------------------------------------------------------------------- */
/* 内部工具                                                                   */
/* ------------------------------------------------------------------------- */

/** 运行状态 → 接口返回码；详细状态仍以 app_protection_report() 为准。 */
static app_status_t protection_state_to_status(app_run_state_t state)
{
    switch (state) {
    case APP_RUN_STATE_ARMED:
    case APP_RUN_STATE_RUNNING:
        return APP_STATUS_OK;
    case APP_RUN_STATE_FAULT:
        return APP_STATUS_FAULT;
    case APP_RUN_STATE_WAIT_CONFIG:
        return APP_STATUS_UNCONFIGURED;
    case APP_RUN_STATE_WAIT_INPUT:
    case APP_RUN_STATE_WAIT_DEVICE:
    case APP_RUN_STATE_WAIT_POWER:
    case APP_RUN_STATE_DISABLED:
    default:
        return APP_STATUS_NOT_READY;
    }
}

/* ------------------------------------------------------------------------- */
/* 公开接口                                                                   */
/* ------------------------------------------------------------------------- */

app_status_t app_protection_init(uint32_t now_ms)
{
    (void)memset(&g_report, 0, sizeof(g_report));
    g_arm_request = false;
    g_mode_active = false;
    g_estop_active = false;

    g_report.state = APP_RUN_STATE_DISABLED;
    g_report.output_allowed = false;
    g_report.active_faults = APP_FAULT_NONE;
    g_report.latched_faults = APP_FAULT_NONE;
    g_report.config_status = app_config_status();
    g_report.remote_status = app_remote_status();
    g_report.device_status = app_motor_status();
    g_report.power_status = app_power_status();
    g_report.updated_ms = now_ms;

    g_status = APP_STATUS_NOT_READY;
    return g_status;
}

app_status_t app_protection_evaluate(uint32_t now_ms)
{
    g_report.updated_ms = now_ms;

    /* 急停是持久的逻辑故障；即使上层调用 resolve(MANUAL) 也不能绕过。 */
    if (g_estop_active) { g_report.active_faults |= APP_FAULT_MANUAL; }

    /* 当前活动故障并入锁存位：故障不会自动消失，须显式清除。 */
    g_report.latched_faults |= g_report.active_faults;

    /* 只读各模块状态，不触发任何动作。 */
    g_report.config_status = app_config_status();
    g_report.remote_status = app_remote_status();
    g_report.device_status = app_motor_status();
    g_report.power_status = app_power_status();

    /* 1) 故障优先：任一活动或锁存故障都禁止输出。 */
    if (g_report.active_faults != APP_FAULT_NONE ||
        g_report.latched_faults != APP_FAULT_NONE) {
        g_report.state = APP_RUN_STATE_FAULT;
        g_report.output_allowed = false;
        g_status = APP_STATUS_FAULT;
        return g_status;
    }

    /* 2) 依次检查配置 → 遥控输入 → 设备，再判断使能与动作模式。 */
    if (g_report.config_status != APP_STATUS_OK) {
        g_report.state = APP_RUN_STATE_WAIT_CONFIG;
    } else if (g_report.remote_status != APP_STATUS_OK) {
        g_report.state = APP_RUN_STATE_WAIT_INPUT;
    } else if (g_report.device_status != APP_STATUS_OK) {
        g_report.state = APP_RUN_STATE_WAIT_DEVICE;
    } else if (g_arm_request && g_mode_active) {
        g_report.state = APP_RUN_STATE_RUNNING;
    } else if (g_arm_request) {
        g_report.state = APP_RUN_STATE_ARMED;
    } else {
        g_report.state = APP_RUN_STATE_DISABLED;
    }

    /* 3) 唯一输出许可：仅在已满足条件且未处于故障时成立。功率限幅由
     *    app_motor_submit() 在保护门控之后执行，不在本处伪造许可。 */
    g_report.output_allowed = (g_report.state == APP_RUN_STATE_ARMED ||
                               g_report.state == APP_RUN_STATE_RUNNING);

    g_status = protection_state_to_status(g_report.state);
    return g_status;
}

void app_protection_set_arm_request(bool requested)
{
    /* 只存标志，不在此处评估：评估唯一入口是 app_protection_evaluate()。 */
    g_arm_request = requested && !g_estop_active;
}

void app_protection_set_mode_active(bool active)
{
    g_mode_active = active && !g_estop_active;
}

bool app_protection_output_allowed(void)
{
    return g_report.output_allowed;
}

void app_protection_raise(uint32_t fault_flags)
{
    if (fault_flags == APP_FAULT_NONE) {
        return;
    }

    g_report.active_faults |= fault_flags;
    g_report.latched_faults |= fault_flags;

    /* 故障即刻撤销输出许可，不等下一次 evaluate。 */
    g_report.state = APP_RUN_STATE_FAULT;
    g_report.output_allowed = false;
    g_status = APP_STATUS_FAULT;
}

void app_protection_resolve(uint32_t fault_flags)
{
    /* 原因解除后仅清活动位，锁存仍禁止输出，必须另行显式复位。 */
    g_report.active_faults &= ~fault_flags;
}

bool app_protection_clear_latched(void)
{
    /* 仍有活动故障时拒绝清除：故障源未消失，清锁存只会掩盖问题。 */
    if (g_report.active_faults != APP_FAULT_NONE) {
        return false;
    }

    g_report.latched_faults = APP_FAULT_NONE;
    if (g_report.state == APP_RUN_STATE_FAULT) {
        g_arm_request = false;
        g_mode_active = false;
        g_report.state = APP_RUN_STATE_DISABLED;
        g_report.output_allowed = false;
        g_status = APP_STATUS_NOT_READY;
    }
    return true;
}

void app_protection_emergency_stop(void)
{
    g_estop_active = true;
    g_arm_request = false;
    g_mode_active = false;
    app_protection_raise(APP_FAULT_MANUAL);
}

void app_protection_release_emergency_stop(void)
{
    g_estop_active = false;
    /* 仅释放 MANUAL 活动位；锁存仍保持，必须由上层显式 clear_latched。 */
    app_protection_resolve(APP_FAULT_MANUAL);
}

bool app_protection_emergency_stop_active(void)
{
    return g_estop_active;
}

const app_protection_report_t *app_protection_report(void)
{
    return &g_report;
}
