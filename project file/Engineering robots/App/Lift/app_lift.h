/**
  ******************************************************************************
  * @file    app_lift.h
  * @brief   前后抬升机构接口（两车公共）。
  *
  * 输入：前/后机构目标角度（rad）；输出：两台 6020 的位置请求。
  * 同步关系、行程、零位与载荷参数待测定，首轮不实现自动越障。
  *
  * 故障/停用策略：本模块只清目标，机械自锁/制动/支撑策略是明确待办，
  * 不能用“位置归零”当作停止。
  ******************************************************************************
  */
#ifndef APP_LIFT_H
#define APP_LIFT_H

#include <stdbool.h>
#include <stdint.h>

#include "Common/app_types.h"

#ifdef __cplusplus
extern "C" {
#endif

/** 抬升请求：前后机构目标角度。 */
typedef struct {
    float front_target_rad; /**< 前抬升机构目标角度，rad */
    float rear_target_rad;  /**< 后抬升机构目标角度，rad */
    bool valid;             /**< false 表示本周期无请求 */
} app_lift_request_t;

/** 抬升状态快照。 */
typedef struct {
    float front_position_rad; /**< 前机构角度，rad；valid=false 时无意义 */
    float rear_position_rad;  /**< 后机构角度，rad；valid=false 时无意义 */
    float sync_error_rad;     /**< 前后偏差，rad；无效时为 0 且 front/rear_valid=false */
    bool front_valid;
    bool rear_valid;
    app_status_t status;
    uint32_t updated_ms;
} app_lift_state_t;

/** 初始化。 */
app_status_t app_lift_init(uint32_t now_ms);

/**
 * @brief 周期更新：生成前后抬升位置请求。
 * @param request 本周期请求；NULL 或 valid=false 时清空目标
 */
app_status_t app_lift_update(const app_lift_request_t *request, uint32_t now_ms);

/** 读取状态快照（反馈未接入时 front/rear_valid=false）。 */
app_status_t app_lift_get_state(app_lift_state_t *out);

/**
 * @brief 停用：清空目标并标记未就绪。
 * @note  机械自锁/制动策略未实现，本函数不做任何“位置归零”。
 */
app_status_t app_lift_stop(uint32_t now_ms);

#ifdef __cplusplus
}
#endif

#endif /* APP_LIFT_H */
