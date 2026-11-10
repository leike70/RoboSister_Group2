/**
  ******************************************************************************
  * @file    app_arm_kinematics.h
  * @brief   工程车机械臂运动学求解接口（正解/逆解）。
  *
  * 首轮只定义接口与单位，不实现算法：连杆尺寸、坐标系、零位与限位未确认，
  * 任何求解都返回 NOT_IMPLEMENTED 或 UNCONFIGURED，绝不返回“成功”的关节角。
  ******************************************************************************
  */
#ifndef APP_ARM_KINEMATICS_H
#define APP_ARM_KINEMATICS_H

#include <stdbool.h>

#include "Common/app_types.h"
#include "Arm/app_arm.h"

#ifdef __cplusplus
extern "C" {
#endif

/** 末端位姿（位置 m、姿态 rad）。 */
typedef struct {
    float x_m;
    float y_m;
    float z_m;
    float roll_rad;
    float pitch_rad;
    bool valid;
} app_arm_pose_t;

/** 运动学接口当前状态：UNCONFIGURED / NOT_IMPLEMENTED / OK。 */
app_status_t app_arm_kinematics_status(void);

/**
 * @brief 正运动学：关节角 → 末端位姿。
 * @param joint_rad 关节角数组，rad，长度 APP_ARM_JOINT_COUNT
 * @param out_pose  输出位姿；失败时 valid=false
 * @return NOT_IMPLEMENTED（首轮）；不得输出伪位姿
 */
app_status_t app_arm_kinematics_forward(const float joint_rad[APP_ARM_JOINT_COUNT],
                                        app_arm_pose_t *out_pose);

/**
 * @brief 逆运动学：末端位姿 → 关节角。
 * @param target_pose 目标位姿
 * @param joint_rad   输出关节角数组，rad；失败时全部置 0 且返回非 OK
 * @return NOT_IMPLEMENTED（首轮）；不得输出伪关节角
 */
app_status_t app_arm_kinematics_inverse(const app_arm_pose_t *target_pose,
                                        float joint_rad[APP_ARM_JOINT_COUNT]);

#ifdef __cplusplus
}
#endif

#endif /* APP_ARM_KINEMATICS_H */
