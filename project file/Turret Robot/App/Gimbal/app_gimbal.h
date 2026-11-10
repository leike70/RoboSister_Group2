/**
  ******************************************************************************
  * @file    app_gimbal.h
  * @brief   炮台车云台 Yaw/Pitch 接口。
  *
  * 单位：角度 rad、角速度 rad/s。Yaw 为 6020、Pitch 为 6006（达妙协议，不能用
  * DJI 驱动代替）。首轮配置未确认：不生成目标、不换算反馈。
  ******************************************************************************
  */
#ifndef APP_GIMBAL_H
#define APP_GIMBAL_H

#include <stdbool.h>
#include <stdint.h>

#include "Common/app_types.h"

#ifdef __cplusplus
extern "C" {
#endif

/** 云台轴。 */
typedef enum {
    APP_GIMBAL_AXIS_YAW = 0,
    APP_GIMBAL_AXIS_PITCH,
    APP_GIMBAL_AXIS_COUNT
} app_gimbal_axis_t;

/** 单轴反馈快照；valid=false 时数值无意义。 */
typedef struct {
    float angle_rad;
    float rate_rad_s;
    bool valid;
    uint32_t timestamp_ms;
} app_gimbal_feedback_t;

/** 云台状态快照。 */
typedef struct {
    float yaw_target_rad;   /**< 当前 Yaw 目标，rad；target_valid=false 时无意义 */
    float pitch_target_rad;
    bool target_valid;
    bool axis_configured[APP_GIMBAL_AXIS_COUNT];
    app_status_t status;
    uint32_t updated_ms;
} app_gimbal_state_t;

/** 初始化；不创建电机对象、不阻塞。 */
app_status_t app_gimbal_init(uint32_t now_ms);

/** 周期更新：把目标转成统一电机请求；目标无效时不产生请求。 */
app_status_t app_gimbal_update(uint32_t now_ms);

/** 设置单轴角度目标，rad；超出机械限位或配置未确认时被拒绝。 */
app_status_t app_gimbal_set_target(app_gimbal_axis_t axis, float angle_rad);

/** 读取单轴目标；未设置过时返回 NOT_READY。 */
app_status_t app_gimbal_get_target(app_gimbal_axis_t axis, float *angle_rad);

/** 清空两轴目标。 */
void app_gimbal_clear_targets(void);

/** 读取单轴反馈；反馈未接入时 valid=false。 */
app_status_t app_gimbal_get_feedback(app_gimbal_axis_t axis,
                                     app_gimbal_feedback_t *out);

/** 停用：清空目标。 */
app_status_t app_gimbal_stop(uint32_t now_ms);

/** 读取状态快照。 */
app_status_t app_gimbal_get_state(app_gimbal_state_t *out);

#ifdef __cplusplus
}
#endif

#endif /* APP_GIMBAL_H */
