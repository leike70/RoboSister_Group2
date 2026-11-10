/**
  ******************************************************************************
  * @file    app_arm.h
  * @brief   工程车机械臂五关节接口。
  *
  * 单位：关节角度 rad、角速度 rad/s。关节顺序固定为 yaw / 大臂 pitch /
  * 小臂 pitch / 爪子 roll / 爪子开合。
  * 首轮配置未确认：不产生关节目标、不换算反馈、不声称动作完成。
  ******************************************************************************
  */
#ifndef APP_ARM_H
#define APP_ARM_H

#include <stdbool.h>
#include <stdint.h>

#include "Common/app_types.h"

#ifdef __cplusplus
extern "C" {
#endif

/** 机械臂关节下标。 */
typedef enum {
    APP_ARM_JOINT_YAW = 0,       /**< 底座 Yaw（4310） */
    APP_ARM_JOINT_SHOULDER,      /**< 大臂 Pitch（4310） */
    APP_ARM_JOINT_ELBOW,         /**< 小臂 Pitch（3508） */
    APP_ARM_JOINT_WRIST_ROLL,    /**< 爪子 Roll（3508） */
    APP_ARM_JOINT_GRIPPER,       /**< 爪子开合（2006） */
    APP_ARM_JOINT_COUNT
} app_arm_joint_t;

/** 单关节反馈快照。valid=false 时数值无意义。 */
typedef struct {
    float position_rad;
    float velocity_rad_s;
    bool valid;
    uint32_t timestamp_ms;
} app_arm_joint_feedback_t;

/** 末端目标（抓取/搬运/倾倒任务使用）。姿态要求待确认。 */
typedef struct {
    float x_m;
    float y_m;
    float z_m;
    float roll_rad;
    float pitch_rad;
    bool valid;
} app_arm_end_target_t;

/** 机械臂状态快照。 */
typedef struct {
    app_status_t status;            /**< 机械臂整体状态 */
    app_status_t kinematics_status; /**< 运动学接口状态 */
    app_status_t task_status;       /**< 任务状态机状态 */
    bool joint_configured[APP_ARM_JOINT_COUNT]; /**< 各关节配置是否齐全 */
    uint32_t updated_ms;
} app_arm_state_t;

/** 初始化；不创建电机对象、不阻塞。 */
app_status_t app_arm_init(uint32_t now_ms);

/** 周期更新：把当前关节/末端目标转成统一电机请求。 */
app_status_t app_arm_update(uint32_t now_ms);

/** 单关节调试入口：设置某关节目标角度，rad。 */
app_status_t app_arm_set_joint_target(app_arm_joint_t joint, float position_rad);

/** 读取单关节调试目标；未设置过时返回 NOT_READY。 */
app_status_t app_arm_get_joint_target(app_arm_joint_t joint, float *position_rad);

/** 清空全部关节调试目标。 */
void app_arm_clear_joint_targets(void);

/** 读取单关节反馈；反馈未接入时 valid=false。 */
app_status_t app_arm_get_joint_feedback(app_arm_joint_t joint,
                                        app_arm_joint_feedback_t *out);

/** 设置末端目标（任务模式）；参数非法或运动学未实现时返回相应状态。 */
app_status_t app_arm_set_end_target(const app_arm_end_target_t *target);

/** 清空末端目标。 */
void app_arm_clear_end_target(void);

/**
 * @brief 停用：清空关节/末端目标。
 * @note  机械臂受重力影响，实车支撑/制动策略未确定，本函数不做“归零”。
 */
app_status_t app_arm_stop(uint32_t now_ms);

/** 读取状态快照。 */
app_status_t app_arm_get_state(app_arm_state_t *out);

#ifdef __cplusplus
}
#endif

#endif /* APP_ARM_H */
