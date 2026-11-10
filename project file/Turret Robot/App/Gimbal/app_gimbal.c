/**
  ******************************************************************************
  * @file    app_gimbal.c
  * @brief   炮台车云台 Yaw/Pitch 目标记录与统一电机请求生成（首轮无反馈、无闭环）。
  *
  * 单位：角度 rad、角速度 rad/s。Yaw 为 GM6020、Pitch 为达妙 6006；本文件只使用
  * 逻辑角色，不接触具体驱动，Pitch 不得被 DJI 驱动错误适配。
  *
  * 首轮边界：
  *  - 不创建电机对象、不换算反馈、不做速度环/姿态解算/前馈/反陀螺；
  *  - 云台配置（Config/app_config.c 的 gimbal 段）未确认时拒绝目标，且不写任何
  *    电机请求；
  *  - 只有“目标已设置且该轴电机运动配置齐全”的轴才写
  *    APP_MOTOR_CMD_POSITION_RAD 请求；反馈接口显式返回 NOT_IMPLEMENTED。
  *
  * 调度约定：周期任务先调用 app_motor_clear_requests()，本文件只在条件满足时
  * 写本周期请求，不保存、不重发上一周期命令。
  ******************************************************************************
  */
#include "Gimbal/app_gimbal.h"

#include <math.h>
#include <stddef.h>

#include "Common/app_time.h"
#include "Motor/app_motor.h"
#include "app_config.h"

/* 内部状态：只保存目标与状态，不保存任何伪造反馈。 */
typedef struct {
    float target_rad[APP_GIMBAL_AXIS_COUNT];
    bool target_set[APP_GIMBAL_AXIS_COUNT];
    app_status_t status;
    uint32_t updated_ms;
} app_gimbal_internal_t;

static app_gimbal_internal_t g_gimbal;

static bool gimbal_axis_valid(app_gimbal_axis_t axis)
{
    return (unsigned)axis < (unsigned)APP_GIMBAL_AXIS_COUNT;
}

static app_motor_role_t gimbal_axis_role(app_gimbal_axis_t axis)
{
    if (axis == APP_GIMBAL_AXIS_PITCH) {
        return APP_MOTOR_ROLE_GIMBAL_PITCH;
    }
    return APP_MOTOR_ROLE_GIMBAL_YAW;
}

static void gimbal_clear_axis_targets(void)
{
    uint32_t axis;

    for (axis = 0u; axis < (uint32_t)APP_GIMBAL_AXIS_COUNT; axis++) {
        g_gimbal.target_set[axis] = false;
        g_gimbal.target_rad[axis] = 0.0f;
    }
}

app_status_t app_gimbal_init(uint32_t now_ms)
{
    const app_config_t *config = app_config_get();

    g_gimbal = (app_gimbal_internal_t){0};
    g_gimbal.status =
        config->gimbal.valid ? APP_STATUS_NOT_READY : APP_STATUS_UNCONFIGURED;
    g_gimbal.updated_ms = now_ms;

    /* 首轮不注册、不构造任何电机对象；配置未确认时保持 UNCONFIGURED。 */
    return g_gimbal.status;
}

app_status_t app_gimbal_update(uint32_t now_ms)
{
    const app_config_t *config = app_config_get();
    bool wrote_any = false;
    uint32_t axis;

    if (!config->gimbal.valid) {
        g_gimbal.status = APP_STATUS_UNCONFIGURED;
        g_gimbal.updated_ms = now_ms;
        return g_gimbal.status;
    }

    for (axis = 0u; axis < (uint32_t)APP_GIMBAL_AXIS_COUNT; axis++) {
        const app_gimbal_axis_t axis_id = (app_gimbal_axis_t)axis;
        int slot;

        if (!g_gimbal.target_set[axis]) {
            continue;
        }
        slot = app_config_find_motor(gimbal_axis_role(axis_id));
        if ((slot < 0) || !app_config_motor_motion_usable(slot)) {
            /* 报文/方向/减速比/零位/限位/PID 不齐：不下发角位置请求。 */
            continue;
        }
        if (app_motor_request(slot, APP_MOTOR_CMD_POSITION_RAD,
                              g_gimbal.target_rad[axis]) == APP_STATUS_OK) {
            wrote_any = true;
        }
    }

    /* 单轴调试允许只输出已设置且配置齐全的轴。 */
    g_gimbal.status = wrote_any ? APP_STATUS_OK : APP_STATUS_NOT_READY;
    g_gimbal.updated_ms = now_ms;
    return g_gimbal.status;
}

app_status_t app_gimbal_set_target(app_gimbal_axis_t axis, float angle_rad)
{
    const app_config_t *config = app_config_get();
    float min_rad;
    float max_rad;

    if (!gimbal_axis_valid(axis) || !isfinite(angle_rad)) {
        return APP_STATUS_INVALID_PARAM;
    }

    if (!config->gimbal.valid) {
        /* 配置未确认：不保存目标，也不修改上一次状态以外的任何东西。 */
        g_gimbal.status = APP_STATUS_UNCONFIGURED;
        g_gimbal.updated_ms = app_time_now_ms();
        return g_gimbal.status;
    }

    if (!config->gimbal.limits_valid) {
        /* 限位未标定时无法判断是否越界，本次目标被拒绝。 */
        return APP_STATUS_INVALID_PARAM;
    }

    if (axis == APP_GIMBAL_AXIS_YAW) {
        min_rad = config->gimbal.yaw_min_rad;
        max_rad = config->gimbal.yaw_max_rad;
    } else {
        min_rad = config->gimbal.pitch_min_rad;
        max_rad = config->gimbal.pitch_max_rad;
    }
    if ((min_rad > max_rad) || (angle_rad < min_rad) || (angle_rad > max_rad)) {
        return APP_STATUS_INVALID_PARAM;
    }

    g_gimbal.target_rad[axis] = angle_rad;
    g_gimbal.target_set[axis] = true;
    g_gimbal.status = APP_STATUS_NOT_READY;
    g_gimbal.updated_ms = app_time_now_ms();

    /* 返回 OK 仅表示目标已记录；无反馈、无闭环，不声称已运动到位。 */
    return APP_STATUS_OK;
}

app_status_t app_gimbal_get_target(app_gimbal_axis_t axis, float *angle_rad)
{
    if (!gimbal_axis_valid(axis) || (angle_rad == NULL)) {
        return APP_STATUS_INVALID_PARAM;
    }
    if (!g_gimbal.target_set[axis]) {
        *angle_rad = 0.0f;
        return APP_STATUS_NOT_READY;
    }
    *angle_rad = g_gimbal.target_rad[axis];
    return APP_STATUS_OK;
}

void app_gimbal_clear_targets(void)
{
    gimbal_clear_axis_targets();
    g_gimbal.status = APP_STATUS_NOT_READY;
    g_gimbal.updated_ms = app_time_now_ms();
}

app_status_t app_gimbal_get_feedback(app_gimbal_axis_t axis,
                                     app_gimbal_feedback_t *out)
{
    if (!gimbal_axis_valid(axis) || (out == NULL)) {
        return APP_STATUS_INVALID_PARAM;
    }

    /* 首轮没有任何反馈适配：不填角度/角速度，valid 保持 false。 */
    out->angle_rad = 0.0f;
    out->rate_rad_s = 0.0f;
    out->valid = false;
    out->timestamp_ms = APP_TIME_NEVER;
    return APP_STATUS_NOT_IMPLEMENTED;
}

app_status_t app_gimbal_stop(uint32_t now_ms)
{
    gimbal_clear_axis_targets();
    g_gimbal.updated_ms = now_ms;
    g_gimbal.status = APP_STATUS_NOT_READY;
    return g_gimbal.status;
}

app_status_t app_gimbal_get_state(app_gimbal_state_t *out)
{
    const app_config_t *config = app_config_get();
    uint32_t axis;

    if (out == NULL) {
        return APP_STATUS_INVALID_PARAM;
    }

    out->yaw_target_rad = g_gimbal.target_set[APP_GIMBAL_AXIS_YAW]
                              ? g_gimbal.target_rad[APP_GIMBAL_AXIS_YAW]
                              : 0.0f;
    out->pitch_target_rad = g_gimbal.target_set[APP_GIMBAL_AXIS_PITCH]
                                ? g_gimbal.target_rad[APP_GIMBAL_AXIS_PITCH]
                                : 0.0f;
    out->target_valid = g_gimbal.target_set[APP_GIMBAL_AXIS_YAW] &&
                        g_gimbal.target_set[APP_GIMBAL_AXIS_PITCH];

    for (axis = 0u; axis < (uint32_t)APP_GIMBAL_AXIS_COUNT; axis++) {
        out->axis_configured[axis] =
            (config->gimbal.valid && config->gimbal.limits_valid &&
             (app_config_find_motor(gimbal_axis_role((app_gimbal_axis_t)axis)) >= 0));
    }

    out->status = g_gimbal.status;
    out->updated_ms = g_gimbal.updated_ms;
    return APP_STATUS_OK;
}
