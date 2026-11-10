/**
  ******************************************************************************
  * @file    app_chassis.c
  * @brief   履带底盘差速运动学与左右履带速度请求（两车公共）。
  *
  * 输入 m/s、rad/s，输出左右履带机构角速度 rad/s。只写 app_motor_request()，
  * 不直接操作任何驱动；本文件不创建 RTOS 对象、不阻塞。
  *
  * 差速换算（正方向约定**待实车确认**）：
  *   v_left  = forward - turn * track_width / 2
  *   v_right = forward + turn * track_width / 2
  *   w_left  = v_left / wheel_radius, w_right = v_right / wheel_radius
  * 当前约定 turn > 0 为左转（右履带更快）；实车方向与电机 direction 一起核验。
  *
  * 首轮配置未确认（chassis.valid=false），update 返回 UNCONFIGURED 且不产生请求。
  ******************************************************************************
  */
#include "Chassis/app_chassis.h"

#include <math.h>
#include <string.h>

#include "Motor/app_motor.h"
#include "app_config.h"

static app_chassis_state_t g_state;
static app_status_t g_status = APP_STATUS_NOT_READY;

/* ------------------------------------------------------------------------- */
/* 内部工具                                                                   */
/* ------------------------------------------------------------------------- */

/** 清空目标并记录状态，保证任何拒绝路径都不留下旧目标。 */
static void chassis_clear_targets(app_status_t status, uint32_t now_ms)
{
    g_state.left_target_rad_s = 0.0f;
    g_state.right_target_rad_s = 0.0f;
    g_state.status = status;
    g_state.updated_ms = now_ms;
    g_status = status;
}

/* ------------------------------------------------------------------------- */
/* 公开接口                                                                   */
/* ------------------------------------------------------------------------- */

app_status_t app_chassis_init(uint32_t now_ms)
{
    (void)memset(&g_state, 0, sizeof(g_state));
    g_state.status = APP_STATUS_NOT_READY;
    g_state.updated_ms = now_ms;
    g_status = APP_STATUS_NOT_READY;
    return g_status;
}

app_status_t app_chassis_update(const app_chassis_request_t *request, uint32_t now_ms)
{
    const app_config_t *config;
    const app_chassis_config_t *chassis;
    int left_slot;
    int right_slot;
    float left_target;
    float right_target;
    app_status_t left_status;
    app_status_t right_status;

    g_state.updated_ms = now_ms;

    /* 无请求：清空目标，不写任何电机请求。 */
    if (request == NULL || !request->valid) {
        chassis_clear_targets(APP_STATUS_NOT_READY, now_ms);
        return g_status;
    }

    config = app_config_get();
    chassis = &config->chassis;

    if (!chassis->valid) {
        chassis_clear_targets(APP_STATUS_UNCONFIGURED, now_ms);
        return g_status;
    }

    /* 换算与校验依赖轮距、轮径和两个上限；缺任一项都不能生成可信目标。 */
    if (!chassis->track_width_valid || !chassis->wheel_radius_valid ||
        !chassis->max_linear_speed_valid || !chassis->max_turn_rate_valid ||
        chassis->wheel_radius_m <= 0.0f || chassis->track_width_m < 0.0f) {
        chassis_clear_targets(APP_STATUS_UNCONFIGURED, now_ms);
        return g_status;
    }

    if (!isfinite(request->forward_mps) || !isfinite(request->turn_rad_s)) {
        chassis_clear_targets(APP_STATUS_INVALID_PARAM, now_ms);
        return g_status;
    }

    /* 超限请求整体拒绝，不做静默限幅：限幅会让上层以为满值已被接受。 */
    if (fabsf(request->forward_mps) > chassis->max_linear_speed_mps ||
        fabsf(request->turn_rad_s) > chassis->max_turn_rate_rad_s) {
        chassis_clear_targets(APP_STATUS_INVALID_PARAM, now_ms);
        return g_status;
    }

    left_slot = app_config_find_motor(APP_MOTOR_ROLE_TRACK_LEFT);
    right_slot = app_config_find_motor(APP_MOTOR_ROLE_TRACK_RIGHT);
    if (left_slot < 0 || right_slot < 0 ||
        !app_config_motor_motion_usable(left_slot) ||
        !app_config_motor_motion_usable(right_slot)) {
        /* 机构参数齐全但电机注册项未齐：不写请求，等设备侧就绪。 */
        chassis_clear_targets(APP_STATUS_NOT_READY, now_ms);
        return g_status;
    }

    /* 差速运动学：turn > 0 为左转（右履带更快），正方向约定待实车确认。 */
    left_target = (request->forward_mps -
                   request->turn_rad_s * chassis->track_width_m * 0.5f) /
                  chassis->wheel_radius_m;
    right_target = (request->forward_mps +
                    request->turn_rad_s * chassis->track_width_m * 0.5f) /
                   chassis->wheel_radius_m;

    if (!isfinite(left_target) || !isfinite(right_target)) {
        chassis_clear_targets(APP_STATUS_INVALID_PARAM, now_ms);
        return g_status;
    }

    left_status = app_motor_request(left_slot, APP_MOTOR_CMD_SPEED_RAD_S, left_target);
    right_status = app_motor_request(right_slot, APP_MOTOR_CMD_SPEED_RAD_S, right_target);
    if (left_status != APP_STATUS_OK || right_status != APP_STATUS_OK) {
        /* 统一出口拒绝（未注册/配置门控未开等）：不保留任何一侧目标。 */
        chassis_clear_targets((left_status != APP_STATUS_OK) ? left_status : right_status,
                              now_ms);
        return g_status;
    }

    g_state.left_target_rad_s = left_target;
    g_state.right_target_rad_s = right_target;
    g_state.status = APP_STATUS_OK;
    g_state.updated_ms = now_ms;
    g_status = APP_STATUS_OK;
    return g_status;
}

app_status_t app_chassis_get_state(app_chassis_state_t *out)
{
    if (out == NULL) {
        return APP_STATUS_INVALID_PARAM;
    }
    *out = g_state;
    return g_status;
}

app_status_t app_chassis_stop(uint32_t now_ms)
{
    /* 只清本模块目标；不写 0 速度请求，避免把“停止”伪装成有效运动命令。 */
    chassis_clear_targets(APP_STATUS_NOT_READY, now_ms);
    return g_status;
}
