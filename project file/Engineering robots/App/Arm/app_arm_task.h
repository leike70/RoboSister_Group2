/**
  ******************************************************************************
  * @file    app_arm_task.h
  * @brief   工程车抓取/搬运/倾倒任务状态接口。
  *
  * 首轮只允许请求和查询，不执行动作：任务请求返回 NOT_IMPLEMENTED，状态保持
  * IDLE/UNSUPPORTED，绝不报告 COMPLETED。
  ******************************************************************************
  */
#ifndef APP_ARM_TASK_H
#define APP_ARM_TASK_H

#include <stdbool.h>
#include <stdint.h>

#include "Common/app_types.h"

#ifdef __cplusplus
extern "C" {
#endif

/** 作业任务种类。 */
typedef enum {
    APP_ARM_TASK_NONE = 0,
    APP_ARM_TASK_GRAB_DART,       /**< 抓取飞镖 */
    APP_ARM_TASK_CARRY_MAGAZINE,  /**< 拿起并搬运弹夹 */
    APP_ARM_TASK_POUR_AMMO        /**< 向炮台车倾倒弹丸 */
} app_arm_task_id_t;

/** 任务状态。只有真正实现并验证后才允许进入 COMPLETED。 */
typedef enum {
    APP_ARM_TASK_STATE_IDLE = 0,
    APP_ARM_TASK_STATE_REQUESTED,
    APP_ARM_TASK_STATE_RUNNING,
    APP_ARM_TASK_STATE_COMPLETED,
    APP_ARM_TASK_STATE_FAILED,
    APP_ARM_TASK_STATE_UNSUPPORTED
} app_arm_task_state_t;

/** 请求一个作业任务；首轮返回 NOT_IMPLEMENTED 并置 UNSUPPORTED。 */
app_status_t app_arm_task_request(app_arm_task_id_t task);

/** 取消当前任务并回到 IDLE。 */
app_status_t app_arm_task_cancel(void);

/** 查询任务状态与当前任务种类。 */
app_arm_task_state_t app_arm_task_get_state(app_arm_task_id_t *out_task);

/** 任务状态机接口状态：UNCONFIGURED / NOT_IMPLEMENTED / OK。 */
app_status_t app_arm_task_status(void);

#ifdef __cplusplus
}
#endif

#endif /* APP_ARM_TASK_H */
