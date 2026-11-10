/**
  ******************************************************************************
  * @file    app_shooting.c
  * @brief   炮台车摩擦轮与拨弹目标记录、统一电机请求生成（首轮无反馈、无卡弹检测）。
  *
  * 单位：角速度 rad/s。摩擦轮为两只 3508（转速闭环），拨弹为 2006；本文件只使用
  * 逻辑角色，不接触具体驱动。
  *
  * 首轮边界：
  *  - 不创建电机对象、不换算反馈、不做卡弹检测/定量拨弹/发射成功判定；
  *  - config.shooting.valid 未确认时清空目标并返回 UNCONFIGURED，不写请求；
  *  - 摩擦轮目标必须成对有效且不超 config.shooting.friction_max_rad_s；
  *  - 拨弹转速来源未确认：摩擦轮请求生效后只记录“收到发射请求”，不写拨弹占位请求；
  *  - 卡弹状态恒为 APP_SHOOT_JAM_UNKNOWN，jam_reset 只记录请求，不声称恢复。
  *
  * 调度约定：周期任务先调用 app_motor_clear_requests()，本文件只写本周期请求。
  ******************************************************************************
  */
#include "Shooting/app_shooting.h"

#include <math.h>
#include <stddef.h>

#include "Common/app_time.h"
#include "Motor/app_motor.h"
#include "app_config.h"

/* 内部状态：只保存本周期目标、卡弹状态与请求记录，不保存伪造反馈。 */
typedef struct {
    float friction_left_rad_s;
    float friction_right_rad_s;
    bool friction_valid;
    bool firing;
    app_shoot_jam_state_t jam;
    bool jam_reset_recorded; /* 仅记录“收到复位请求”，无恢复判定 */
    app_status_t status;
    uint32_t updated_ms;
} app_shooting_internal_t;

static app_shooting_internal_t g_shooting;

static void shooting_clear_targets(void)
{
    g_shooting.friction_left_rad_s = 0.0f;
    g_shooting.friction_right_rad_s = 0.0f;
    g_shooting.friction_valid = false;
    g_shooting.firing = false;
}

app_status_t app_shooting_init(uint32_t now_ms)
{
    const app_config_t *config = app_config_get();

    g_shooting = (app_shooting_internal_t){0};
    g_shooting.jam = APP_SHOOT_JAM_UNKNOWN;
    g_shooting.status =
        config->shooting.valid ? APP_STATUS_NOT_READY : APP_STATUS_UNCONFIGURED;
    g_shooting.updated_ms = now_ms;

    /* 首轮不注册、不构造任何电机对象；配置未确认时保持 UNCONFIGURED。 */
    return g_shooting.status;
}

app_status_t app_shooting_update(const app_shooting_request_t *request,
                                 uint32_t now_ms)
{
    const app_config_t *config = app_config_get();
    int left_slot;
    int right_slot;
    bool friction_written = false;

    if ((request == NULL) || !request->valid) {
        /* 无请求/请求整体无效：清空目标，保持不拨弹。 */
        shooting_clear_targets();
        g_shooting.status = APP_STATUS_NOT_READY;
        g_shooting.updated_ms = now_ms;
        return g_shooting.status;
    }

    if (request->jam_reset) {
        /* 只记录复位请求。没有卡弹检测实现，不改变 jam 结论、不声称已恢复。 */
        g_shooting.jam_reset_recorded = true;
    }

    if (!config->shooting.valid) {
        shooting_clear_targets();
        g_shooting.status = APP_STATUS_UNCONFIGURED;
        g_shooting.updated_ms = now_ms;
        return g_shooting.status;
    }

    if (!request->friction_valid) {
        shooting_clear_targets();
        g_shooting.status = APP_STATUS_NOT_READY;
        g_shooting.updated_ms = now_ms;
        return g_shooting.status;
    }

    if (!isfinite(request->friction_left_rad_s) ||
        !isfinite(request->friction_right_rad_s) ||
        (fabsf(request->friction_left_rad_s) > config->shooting.friction_max_rad_s) ||
        (fabsf(request->friction_right_rad_s) > config->shooting.friction_max_rad_s)) {
        /* 目标非法或超速：本周期不写摩擦轮、不拨弹，并清掉旧目标避免残留。 */
        shooting_clear_targets();
        g_shooting.status = APP_STATUS_INVALID_PARAM;
        g_shooting.updated_ms = now_ms;
        return g_shooting.status;
    }

    left_slot = app_config_find_motor(APP_MOTOR_ROLE_FRICTION_LEFT);
    right_slot = app_config_find_motor(APP_MOTOR_ROLE_FRICTION_RIGHT);
    if ((left_slot >= 0) && (right_slot >= 0) &&
        app_config_motor_motion_usable(left_slot) &&
        app_config_motor_motion_usable(right_slot) &&
        (app_motor_request(left_slot, APP_MOTOR_CMD_SPEED_RAD_S,
                           request->friction_left_rad_s) == APP_STATUS_OK) &&
        (app_motor_request(right_slot, APP_MOTOR_CMD_SPEED_RAD_S,
                           request->friction_right_rad_s) == APP_STATUS_OK)) {
        friction_written = true;
    }

    if (!friction_written) {
        /* 两只摩擦轮没有成对写入：保持不通弹，不假装发射链条可用。 */
        shooting_clear_targets();
        g_shooting.status = APP_STATUS_NOT_READY;
        g_shooting.updated_ms = now_ms;
        return g_shooting.status;
    }

    g_shooting.friction_left_rad_s = request->friction_left_rad_s;
    g_shooting.friction_right_rad_s = request->friction_right_rad_s;
    g_shooting.friction_valid = true;

    if (!request->fire) {
        g_shooting.firing = false;
        g_shooting.status = APP_STATUS_OK;
        g_shooting.updated_ms = now_ms;
        return g_shooting.status;
    }

    /* 拨弹转速来源未确认：2006 的减速比、拨弹定量方式、trigger_rate_hz 与转速的
     * 换算关系都没有标定。此时**不写任何拨弹占位请求**（写 0.0f 也会被误读成
     * “已给出转速”），只记录“收到发射请求但无法执行”，绝不声称发射成功。 */
    g_shooting.firing = false;
    g_shooting.status = APP_STATUS_NOT_READY;

    g_shooting.updated_ms = now_ms;
    return g_shooting.status;
}

app_status_t app_shooting_get_state(app_shooting_state_t *out)
{
    if (out == NULL) {
        return APP_STATUS_INVALID_PARAM;
    }

    out->friction_left_rad_s = g_shooting.friction_left_rad_s;
    out->friction_right_rad_s = g_shooting.friction_right_rad_s;
    out->friction_valid = g_shooting.friction_valid;
    out->firing = g_shooting.firing;
    out->jam = g_shooting.jam; /* 无检测实现：恒为 APP_SHOOT_JAM_UNKNOWN */
    out->status = g_shooting.status;
    out->updated_ms = g_shooting.updated_ms;
    return APP_STATUS_OK;
}

app_status_t app_shooting_stop(uint32_t now_ms)
{
    shooting_clear_targets();
    g_shooting.jam_reset_recorded = false;
    g_shooting.updated_ms = now_ms;
    g_shooting.status = APP_STATUS_NOT_READY;
    return g_shooting.status;
}
