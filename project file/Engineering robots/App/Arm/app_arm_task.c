/**
  ******************************************************************************
  * @file    app_arm_task.c
  * @brief   工程车抓取/搬运/倾倒任务状态机（首轮只记录状态，不执行动作）。
  *
  * 任务状态机只有“记录请求”的能力：合法任务请求进入 UNSUPPORTED 并返回
  * NOT_IMPLEMENTED，绝不会进入 COMPLETED；取消回到 IDLE。
  * 真正的动作序列依赖运动学、关节限位与夹持参数，全部未确认，故不实现。
  * 不创建 RTOS 对象、不阻塞。
  ******************************************************************************
  */
#include "Arm/app_arm_task.h"

#include <stddef.h>

#include "Common/app_time.h"

/* ------------------------------------------------------------------------- */
/* 内部状态                                                                   */
/* ------------------------------------------------------------------------- */

typedef struct {
    app_arm_task_state_t state;
    app_arm_task_id_t task;
    uint32_t updated_ms;
} app_arm_task_internal_t;

static app_arm_task_internal_t g_task = {
    .state = APP_ARM_TASK_STATE_IDLE,
    .task = APP_ARM_TASK_NONE,
    .updated_ms = APP_TIME_NEVER,
};

/** 只接受 app_arm_task.h 中已定义的作业任务。 */
static bool arm_task_id_is_valid(app_arm_task_id_t task)
{
    return (task == APP_ARM_TASK_GRAB_DART) ||
           (task == APP_ARM_TASK_CARRY_MAGAZINE) ||
           (task == APP_ARM_TASK_POUR_AMMO);
}

/* ------------------------------------------------------------------------- */
/* 公开接口                                                                   */
/* ------------------------------------------------------------------------- */

app_status_t app_arm_task_request(app_arm_task_id_t task)
{
    if (!arm_task_id_is_valid(task)) {
        /* 非法任务号：状态保持原样，不登记请求。 */
        return APP_STATUS_INVALID_PARAM;
    }

    /*
     * 首轮没有任何动作序列：登记任务号，明确标记 UNSUPPORTED，
     * 返回 NOT_IMPLEMENTED。禁止把未执行的任务报成 COMPLETED。
     */
    g_task.task = task;
    g_task.state = APP_ARM_TASK_STATE_UNSUPPORTED;
    g_task.updated_ms = app_time_now_ms();
    return APP_STATUS_NOT_IMPLEMENTED;
}

app_status_t app_arm_task_cancel(void)
{
    g_task.state = APP_ARM_TASK_STATE_IDLE;
    g_task.task = APP_ARM_TASK_NONE;
    g_task.updated_ms = app_time_now_ms();
    return APP_STATUS_OK;
}

app_arm_task_state_t app_arm_task_get_state(app_arm_task_id_t *out_task)
{
    if (out_task != NULL) {
        *out_task = g_task.task; /* 无任务时恒为 APP_ARM_TASK_NONE */
    }
    return g_task.state;
}

app_status_t app_arm_task_status(void)
{
    /* 接口已定义、动作序列与验证均不存在。 */
    return APP_STATUS_NOT_IMPLEMENTED;
}
