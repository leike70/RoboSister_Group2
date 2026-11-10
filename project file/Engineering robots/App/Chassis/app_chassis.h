/**
  ******************************************************************************
  * @file    app_chassis.h
  * @brief   履带底盘接口（两车公共）。
  *
  * 输入：行进/转向请求（m/s、rad/s）；输出：左右履带目标角速度（rad/s）。
  * 只生成电机请求，不直接操作驱动；方向、减速比和轮径来自本车配置。
  * 首轮配置未确认，update 返回未配置且不产生任何请求。
  ******************************************************************************
  */
#ifndef APP_CHASSIS_H
#define APP_CHASSIS_H

#include <stdbool.h>
#include <stdint.h>

#include "Common/app_types.h"

#ifdef __cplusplus
extern "C" {
#endif

/** 底盘请求：线速度与转向角速度。 */
typedef struct {
    float forward_mps;   /**< 前进速度，m/s，正为前进 */
    float turn_rad_s;    /**< 转向角速度，rad/s，正为左转（待实车确认） */
    bool valid;          /**< false 表示本周期无请求，模块必须清空输出 */
} app_chassis_request_t;

/** 底盘状态快照。 */
typedef struct {
    float left_target_rad_s;  /**< 左履带目标角速度，rad/s */
    float right_target_rad_s; /**< 右履带目标角速度，rad/s */
    app_status_t status;
    uint32_t updated_ms;
} app_chassis_state_t;

/** 初始化。 */
app_status_t app_chassis_init(uint32_t now_ms);

/**
 * @brief 周期更新：把请求换算成左右履带角速度请求并交给统一电机输出。
 * @param request 本周期请求；NULL 或 valid=false 时清空目标
 * @return UNCONFIGURED / NOT_READY / OK
 */
app_status_t app_chassis_update(const app_chassis_request_t *request, uint32_t now_ms);

/** 读取状态快照。 */
app_status_t app_chassis_get_state(app_chassis_state_t *out);

/** 停用：清空目标（不阻塞、不等待硬件）。 */
app_status_t app_chassis_stop(uint32_t now_ms);

#ifdef __cplusplus
}
#endif

#endif /* APP_CHASSIS_H */
