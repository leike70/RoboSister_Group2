/**
  ******************************************************************************
  * @file    app_arm_kinematics.c
  * @brief   工程车机械臂运动学占位实现：只定义接口行为，不做任何求解。
  *
  * 连杆尺寸、坐标系、零位与关节限位在 Config/app_config.c 中全部未确认
  * （arm.links_valid / reach_valid / limits_valid 均为 false）。因此本文件固定
  * 返回 NOT_IMPLEMENTED，并把输出置为“无效”或全零，绝不输出伪位姿、伪关节角。
  * 实现顺序：先标定零位与限位，再用实测关节角验证正解，最后才做逆解可达性，
  * 详见 App/Arm/README.md。
  ******************************************************************************
  */
#include "Arm/app_arm_kinematics.h"

#include <stddef.h>

/** 失败时把位姿整份标记为无效，避免调用方误用残留数值。 */
static void pose_mark_invalid(app_arm_pose_t *pose)
{
    pose->x_m = 0.0f;
    pose->y_m = 0.0f;
    pose->z_m = 0.0f;
    pose->roll_rad = 0.0f;
    pose->pitch_rad = 0.0f;
    pose->valid = false;
}

app_status_t app_arm_kinematics_status(void)
{
    /* 无连杆/坐标系参数，接口存在但实现不存在。 */
    return APP_STATUS_NOT_IMPLEMENTED;
}

app_status_t app_arm_kinematics_forward(const float joint_rad[APP_ARM_JOINT_COUNT],
                                        app_arm_pose_t *out_pose)
{
    if (out_pose == NULL) {
        return APP_STATUS_INVALID_PARAM;
    }

    pose_mark_invalid(out_pose);

    if (joint_rad == NULL) {
        return APP_STATUS_INVALID_PARAM;
    }

    /* 没有连杆几何与零位，不计算任何位姿。 */
    return APP_STATUS_NOT_IMPLEMENTED;
}

app_status_t app_arm_kinematics_inverse(const app_arm_pose_t *target_pose,
                                        float joint_rad[APP_ARM_JOINT_COUNT])
{
    int joint;

    if (joint_rad == NULL) {
        return APP_STATUS_INVALID_PARAM;
    }

    for (joint = 0; joint < (int)APP_ARM_JOINT_COUNT; joint++) {
        joint_rad[joint] = 0.0f;
    }

    if (target_pose == NULL) {
        return APP_STATUS_INVALID_PARAM;
    }

    /* 不求解、不给出可执行的关节角。 */
    return APP_STATUS_NOT_IMPLEMENTED;
}
