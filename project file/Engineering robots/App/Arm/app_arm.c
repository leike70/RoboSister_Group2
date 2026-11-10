/**
  ******************************************************************************
  * @file    app_arm.c
  * @brief   工程车机械臂五关节实现（首轮无动作骨架）。
  *
  * 约定见 App/Arm/app_arm.h：关节角度 rad、角速度 rad/s；关节顺序固定为
  * yaw / 大臂 pitch / 小臂 pitch / 爪子 roll / 爪子开合。
  *
  * 本文件只做三件事：保存单关节调试目标、按配置门控把目标转成统一电机请求、
  * 维护可读状态。不创建电机对象、不阻塞、不发报文、不换算反馈。
  * 配置未确认（Config/app_config.c 中 arm.valid==false）时不写任何电机请求；
  * 连杆/坐标系/零位/限位/PID 未确认，因此不求解运动学、不声称动作完成。
  ******************************************************************************
  */
#include "Arm/app_arm.h"

#include <float.h>
#include <stddef.h>

#include "Arm/app_arm_kinematics.h"
#include "Arm/app_arm_task.h"
#include "Common/app_time.h"
#include "Motor/app_motor.h"
#include "app_config.h"

/* ------------------------------------------------------------------------- */
/* 内部状态                                                                   */
/* ------------------------------------------------------------------------- */

/** 关节下标到本车电机角色的固定映射，顺序同 app_arm.h，不得调换。 */
static const app_motor_role_t k_joint_role[APP_ARM_JOINT_COUNT] = {
    APP_MOTOR_ROLE_ARM_YAW,
    APP_MOTOR_ROLE_ARM_SHOULDER,
    APP_MOTOR_ROLE_ARM_ELBOW,
    APP_MOTOR_ROLE_ARM_WRIST_ROLL,
    APP_MOTOR_ROLE_ARM_GRIPPER,
};

typedef struct {
    app_status_t status;
    bool joint_target_set[APP_ARM_JOINT_COUNT];
    float joint_target_rad[APP_ARM_JOINT_COUNT];
    bool joint_configured[APP_ARM_JOINT_COUNT];
    app_arm_end_target_t end_target;
    bool end_target_set;
    uint32_t updated_ms;
} app_arm_internal_t;

static app_arm_internal_t g_arm;

/* ------------------------------------------------------------------------- */
/* 工具函数                                                                   */
/* ------------------------------------------------------------------------- */

/** 不依赖 libm：NaN 与自身不相等，正负无穷超出 FLT_MAX 范围。 */
static bool arm_float_is_finite(float value)
{
    return (value == value) && (value <= FLT_MAX) && (value >= -FLT_MAX);
}

static bool arm_joint_is_valid(app_arm_joint_t joint)
{
    /* 用无符号比较，负数（越界强转）会变成大值而被拒绝。 */
    return (unsigned int)joint < (unsigned int)APP_ARM_JOINT_COUNT;
}

/** 配置表已校验且机械臂机构整体已确认时才允许动作。 */
static bool arm_config_is_valid(void)
{
    const app_config_t *config = app_config_get();

    if (app_config_status() == APP_STATUS_UNCONFIGURED) {
        return false;
    }
    return config->arm.valid;
}

static void arm_clear_joint_targets_internal(void)
{
    int joint;

    for (joint = 0; joint < (int)APP_ARM_JOINT_COUNT; joint++) {
        g_arm.joint_target_set[joint] = false;
        g_arm.joint_target_rad[joint] = 0.0f;
    }
}

static void arm_clear_end_target_internal(void)
{
    g_arm.end_target_set = false;
    g_arm.end_target.x_m = 0.0f;
    g_arm.end_target.y_m = 0.0f;
    g_arm.end_target.z_m = 0.0f;
    g_arm.end_target.roll_rad = 0.0f;
    g_arm.end_target.pitch_rad = 0.0f;
    g_arm.end_target.valid = false;
}

static void arm_clear_targets_internal(void)
{
    arm_clear_joint_targets_internal();
    arm_clear_end_target_internal();
}

/**
 * @brief 按实际配置刷新各关节“配置是否齐全”，并给出机械臂整体状态。
 * @note  只读配置，不写电机请求。
 */
static app_status_t arm_refresh_configuration(void)
{
    const bool config_valid = arm_config_is_valid();
    bool all_configured = true;
    int joint;

    for (joint = 0; joint < (int)APP_ARM_JOINT_COUNT; joint++) {
        const int slot = app_config_find_motor(k_joint_role[joint]);
        const bool configured = config_valid && (slot >= 0) &&
                                app_config_motor_motion_usable(slot);

        g_arm.joint_configured[joint] = configured;
        if (!configured) {
            all_configured = false;
        }
    }

    if (!config_valid) {
        return APP_STATUS_UNCONFIGURED;
    }
    return all_configured ? APP_STATUS_OK : APP_STATUS_NOT_READY;
}

/**
 * @brief 把单关节目标限幅到已确认的关节限位（或该电机限位）。
 * @return true 表示得到可用目标；false 表示没有任何已确认限位，禁止输出。
 */
static bool arm_clamp_joint_target(int joint, int slot, float *position_rad)
{
    const app_config_t *config = app_config_get();
    const app_motor_config_t *motor = app_config_motor(slot);
    float target = *position_rad;

    if (motor == NULL) {
        return false;
    }

    if (config->arm.limits_valid &&
        (config->arm.joint_max_rad[joint] > config->arm.joint_min_rad[joint])) {
        target = app_clampf(target, config->arm.joint_min_rad[joint],
                            config->arm.joint_max_rad[joint]);
    } else if (motor->limit_valid &&
               (motor->limit_max_rad > motor->limit_min_rad)) {
        /* 关节级限位未确认时退回到该电机已确认的限位；两者都没有则不输出。 */
        target = app_clampf(target, motor->limit_min_rad, motor->limit_max_rad);
    } else {
        return false;
    }

    *position_rad = target;
    return true;
}

/* ------------------------------------------------------------------------- */
/* 公开接口                                                                   */
/* ------------------------------------------------------------------------- */

app_status_t app_arm_init(uint32_t now_ms)
{
    g_arm.status = APP_STATUS_UNCONFIGURED;
    arm_clear_targets_internal();
    g_arm.status = arm_refresh_configuration();
    g_arm.updated_ms = now_ms;
    return g_arm.status;
}

app_status_t app_arm_update(uint32_t now_ms)
{
    const app_status_t config_status = arm_refresh_configuration();
    bool any_target = false;
    bool all_applied = true;
    int joint;

    if (config_status == APP_STATUS_UNCONFIGURED) {
        /* 配置无效：清空目标，且不写任何电机请求。 */
        arm_clear_targets_internal();
        g_arm.status = APP_STATUS_UNCONFIGURED;
        g_arm.updated_ms = now_ms;
        return g_arm.status;
    }

    for (joint = 0; joint < (int)APP_ARM_JOINT_COUNT; joint++) {
        const int slot = app_config_find_motor(k_joint_role[joint]);
        float target;

        if (!g_arm.joint_target_set[joint]) {
            continue;
        }
        any_target = true;

        if (!g_arm.joint_configured[joint] || (slot < 0)) {
            /* 关节配置不齐：不写请求；目标保留，配置补全后可用。 */
            all_applied = false;
            continue;
        }

        target = g_arm.joint_target_rad[joint];
        if (!arm_clamp_joint_target(joint, slot, &target)) {
            /* 没有任何已确认限位：拒绝输出，避免越界运动。 */
            all_applied = false;
            continue;
        }

        if (app_motor_request(slot, APP_MOTOR_CMD_POSITION_RAD, target) !=
            APP_STATUS_OK) {
            /* 电机模块因配置/参数拒绝，本关节保持无输出。 */
            all_applied = false;
            continue;
        }
    }

    /* 末端目标只保存请求：逆解未实现，本周期不产生任何关节输出。 */
    g_arm.status = (any_target && all_applied) ? APP_STATUS_OK
                                               : APP_STATUS_NOT_READY;
    g_arm.updated_ms = now_ms;
    return g_arm.status;
}

app_status_t app_arm_set_joint_target(app_arm_joint_t joint, float position_rad)
{
    if (!arm_joint_is_valid(joint) || !arm_float_is_finite(position_rad)) {
        return APP_STATUS_INVALID_PARAM;
    }

    if (!arm_config_is_valid()) {
        /* 配置未确认：不保存，避免“看起来已接受”的假象。 */
        return APP_STATUS_UNCONFIGURED;
    }

    g_arm.joint_target_rad[joint] = position_rad;
    g_arm.joint_target_set[joint] = true;
    g_arm.updated_ms = app_time_now_ms();
    return APP_STATUS_OK;
}

app_status_t app_arm_get_joint_target(app_arm_joint_t joint, float *position_rad)
{
    if (!arm_joint_is_valid(joint) || (position_rad == NULL)) {
        return APP_STATUS_INVALID_PARAM;
    }
    if (!g_arm.joint_target_set[joint]) {
        return APP_STATUS_NOT_READY;
    }

    *position_rad = g_arm.joint_target_rad[joint];
    return APP_STATUS_OK;
}

void app_arm_clear_joint_targets(void)
{
    arm_clear_joint_targets_internal();
    g_arm.updated_ms = app_time_now_ms();
}

app_status_t app_arm_get_joint_feedback(app_arm_joint_t joint,
                                        app_arm_joint_feedback_t *out)
{
    if (!arm_joint_is_valid(joint) || (out == NULL)) {
        return APP_STATUS_INVALID_PARAM;
    }

    /* 首轮没有反馈适配层：不伪造测量值。 */
    out->position_rad = 0.0f;
    out->velocity_rad_s = 0.0f;
    out->valid = false;
    out->timestamp_ms = APP_TIME_NEVER;
    return APP_STATUS_NOT_IMPLEMENTED;
}

app_status_t app_arm_set_end_target(const app_arm_end_target_t *target)
{
    if (target == NULL) {
        return APP_STATUS_INVALID_PARAM;
    }
    if (!arm_float_is_finite(target->x_m) || !arm_float_is_finite(target->y_m) ||
        !arm_float_is_finite(target->z_m) ||
        !arm_float_is_finite(target->roll_rad) ||
        !arm_float_is_finite(target->pitch_rad)) {
        return APP_STATUS_INVALID_PARAM;
    }

    /* 运动学未实现：只保存请求，不求解、不产生关节目标。 */
    g_arm.end_target = *target;
    g_arm.end_target_set = true;
    g_arm.updated_ms = app_time_now_ms();
    return APP_STATUS_NOT_IMPLEMENTED;
}

void app_arm_clear_end_target(void)
{
    arm_clear_end_target_internal();
    g_arm.updated_ms = app_time_now_ms();
}

app_status_t app_arm_stop(uint32_t now_ms)
{
    /*
     * 机械臂受重力影响：实车支撑/制动策略（机械自锁、抱闸、支撑台）尚未确定，
     * 本函数只清空目标，不做“位置归零”，也不能把“无请求”当作机构已制动。
     * 恢复输出前必须人工确认支撑条件，见 App/Arm/README.md 的故障行为一节。
     */
    arm_clear_targets_internal();
    g_arm.status = APP_STATUS_NOT_READY;
    g_arm.updated_ms = now_ms;
    return g_arm.status;
}

app_status_t app_arm_get_state(app_arm_state_t *out)
{
    int joint;

    if (out == NULL) {
        return APP_STATUS_INVALID_PARAM;
    }

    out->status = g_arm.status;
    out->kinematics_status = app_arm_kinematics_status();
    out->task_status = app_arm_task_status();
    for (joint = 0; joint < (int)APP_ARM_JOINT_COUNT; joint++) {
        out->joint_configured[joint] = g_arm.joint_configured[joint];
    }
    out->updated_ms = g_arm.updated_ms;
    return APP_STATUS_OK;
}
