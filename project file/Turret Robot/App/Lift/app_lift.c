/**
  ******************************************************************************
  * @file    app_lift.c
  * @brief   前后抬升机构位置请求（两车公共）。
  *
  * 输入前/后机构目标角度 rad，输出两台 6020 的位置请求。只写 app_motor_request()，
  * 不直接操作驱动；本文件不创建 RTOS 对象、不阻塞。
  *
  * 首轮边界：
  *  - lift.valid=false 时 update 返回 UNCONFIGURED，不写任何请求；
  *  - 无反馈接入，前/后实际角度与同步偏差无法计算：sync_error_rad 保持 0、
  *    front_valid/rear_valid=false，不把 0 当作有效测量；
  *  - 不实现自动越障，也不做“目标归零”式停机（机械自锁/制动策略待定）。
  ******************************************************************************
  */
#include "Lift/app_lift.h"

#include <math.h>
#include <string.h>

#include "Motor/app_motor.h"
#include "app_config.h"

static app_lift_state_t g_state;
static app_status_t g_status = APP_STATUS_NOT_READY;

/* ------------------------------------------------------------------------- */
/* 内部工具                                                                   */
/* ------------------------------------------------------------------------- */

/** 清空快照：位置/同步偏差清零并标为无效，状态记录拒绝原因。 */
static void lift_clear_state(app_status_t status, uint32_t now_ms)
{
    g_state.front_position_rad = 0.0f;
    g_state.rear_position_rad = 0.0f;
    g_state.sync_error_rad = 0.0f;
    g_state.front_valid = false;
    g_state.rear_valid = false;
    g_state.status = status;
    g_state.updated_ms = now_ms;
    g_status = status;
}

/* ------------------------------------------------------------------------- */
/* 公开接口                                                                   */
/* ------------------------------------------------------------------------- */

app_status_t app_lift_init(uint32_t now_ms)
{
    (void)memset(&g_state, 0, sizeof(g_state));
    lift_clear_state(APP_STATUS_NOT_READY, now_ms);
    return g_status;
}

app_status_t app_lift_update(const app_lift_request_t *request, uint32_t now_ms)
{
    const app_config_t *config;
    const app_lift_config_t *lift;
    int front_slot;
    int rear_slot;
    app_status_t front_status;
    app_status_t rear_status;

    g_state.updated_ms = now_ms;

    /* 无请求：清空快照，不写任何电机请求。 */
    if (request == NULL || !request->valid) {
        lift_clear_state(APP_STATUS_NOT_READY, now_ms);
        return g_status;
    }

    config = app_config_get();
    lift = &config->lift;

    if (!lift->valid) {
        lift_clear_state(APP_STATUS_UNCONFIGURED, now_ms);
        return g_status;
    }

    if (!isfinite(request->front_target_rad) || !isfinite(request->rear_target_rad)) {
        lift_clear_state(APP_STATUS_INVALID_PARAM, now_ms);
        return g_status;
    }

    /* 行程未确认时无法校验目标，也就不能生成位置请求。 */
    if (!lift->travel_valid || lift->travel_max_rad <= lift->travel_min_rad) {
        lift_clear_state(APP_STATUS_UNCONFIGURED, now_ms);
        return g_status;
    }

    /* 超行程整体拒绝，不做静默限幅：限幅会掩盖上层目标错误。 */
    if (request->front_target_rad < lift->travel_min_rad ||
        request->front_target_rad > lift->travel_max_rad ||
        request->rear_target_rad < lift->travel_min_rad ||
        request->rear_target_rad > lift->travel_max_rad) {
        lift_clear_state(APP_STATUS_INVALID_PARAM, now_ms);
        return g_status;
    }

    front_slot = app_config_find_motor(APP_MOTOR_ROLE_LIFT_FRONT);
    rear_slot = app_config_find_motor(APP_MOTOR_ROLE_LIFT_REAR);
    if (front_slot < 0 || rear_slot < 0 ||
        !app_config_motor_motion_usable(front_slot) ||
        !app_config_motor_motion_usable(rear_slot)) {
        lift_clear_state(APP_STATUS_NOT_READY, now_ms);
        return g_status;
    }

    front_status =
        app_motor_request(front_slot, APP_MOTOR_CMD_POSITION_RAD, request->front_target_rad);
    rear_status =
        app_motor_request(rear_slot, APP_MOTOR_CMD_POSITION_RAD, request->rear_target_rad);
    if (front_status != APP_STATUS_OK || rear_status != APP_STATUS_OK) {
        lift_clear_state((front_status != APP_STATUS_OK) ? front_status : rear_status,
                         now_ms);
        return g_status;
    }

    /* 无反馈：实际角度与同步偏差不能计算，保持 0 且标为无效。 */
    g_state.front_position_rad = 0.0f;
    g_state.rear_position_rad = 0.0f;
    g_state.sync_error_rad = 0.0f;
    g_state.front_valid = false;
    g_state.rear_valid = false;
    g_state.status = APP_STATUS_OK;
    g_state.updated_ms = now_ms;
    g_status = APP_STATUS_OK;
    return g_status;
}

app_status_t app_lift_get_state(app_lift_state_t *out)
{
    if (out == NULL) {
        return APP_STATUS_INVALID_PARAM;
    }
    *out = g_state;
    return g_status;
}

app_status_t app_lift_stop(uint32_t now_ms)
{
    /* 只清目标/状态：机械自锁、制动与零位标定未实现，这里不做“位置归零”。
     * 本周期请求已由调度器的 app_motor_clear_requests() 统一清空。 */
    lift_clear_state(APP_STATUS_NOT_READY, now_ms);
    return g_status;
}
