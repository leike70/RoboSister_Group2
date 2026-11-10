/**
  ******************************************************************************
  * @file    app_config.c
  * @brief   炮台车配置表（本车专用）。
  *
  * 本文件是**炮台车**的硬件映射与机构参数表。所有未确认项保留占位数值，但
  * configured / valid_* 一律为 false；占位值因此不会参与设备注册或单位换算。
  * 确认一项就打开对应 valid_*，全部齐全后再把 configured 置 true。
  *
  * 与工程车的差异：本车 9 台电机 = 公共底盘（2×3508 + 2×6020）+
  * 云台（Yaw 1×6020、Pitch 1×达妙 6006）+ 摩擦轮 2×3508 + 拨弹 1×2006。
  * 机械臂配置（arm）保持 invalid 且不会被使用；Pitch 不得用 DJI 驱动代替。
  ******************************************************************************
  */
#include "app_config.h"

#include <stddef.h>

#include "Common/app_time.h"

/* ------------------------------------------------------------------------- */
/* 工具：占位电机配置                                                         */
/* ------------------------------------------------------------------------- */

#define PLACEHOLDER_MOTOR(logical_role, logical_name, motor_model)                 \
    {                                                                              \
        .name = (logical_name),                                                    \
        .role = (logical_role),                                                    \
        .model = (motor_model),                                                    \
        .bus = APP_CAN_BUS_NONE,                                                   \
        .motor_id = 0u,                                                            \
        .tx_msg_id = 0u,                                                           \
        .rx_msg_id = 0u,                                                           \
        .control_mode = APP_MOTOR_CONTROL_NONE,                                    \
        .direction = 0,                                                            \
        .gear_ratio = 1.0f,                                                        \
        .zero_offset_rad = 0.0f,                                                   \
        .limit_min_rad = 0.0f,                                                     \
        .limit_max_rad = 0.0f,                                                     \
        .pid = {0.0f, 0.0f, 0.0f, 1.0f, 0.1f, false},                              \
        .configured = false,                                                       \
        .bus_valid = false,                                                        \
        .id_valid = false,                                                         \
        .msg_id_valid = false,                                                     \
        .control_mode_valid = false,                                               \
        .direction_valid = false,                                                  \
        .gear_ratio_valid = false,                                                 \
        .zero_offset_valid = false,                                                \
        .limit_valid = false                                                       \
    }

/* ------------------------------------------------------------------------- */
/* 本车配置表                                                                 */
/* ------------------------------------------------------------------------- */

static const app_config_t g_config = {
    .vehicle_kind = APP_VEHICLE_KIND_TURRET,

    /* 公共底盘（2×3508 + 2×6020）+ 炮台车专有机构
     * （云台 6020 + 达妙 6006、摩擦轮 2×3508、拨弹 2006）。
     * 总线、ID、报文 ID、控制模式、方向、减速比、零位、限位、PID 全部待确认。 */
    .motors = {
        PLACEHOLDER_MOTOR(APP_MOTOR_ROLE_TRACK_LEFT, "track_left", APP_MOTOR_MODEL_DJI_M3508),
        PLACEHOLDER_MOTOR(APP_MOTOR_ROLE_TRACK_RIGHT, "track_right", APP_MOTOR_MODEL_DJI_M3508),
        PLACEHOLDER_MOTOR(APP_MOTOR_ROLE_LIFT_FRONT, "lift_front", APP_MOTOR_MODEL_DJI_GM6020),
        PLACEHOLDER_MOTOR(APP_MOTOR_ROLE_LIFT_REAR, "lift_rear", APP_MOTOR_MODEL_DJI_GM6020),
        PLACEHOLDER_MOTOR(APP_MOTOR_ROLE_GIMBAL_YAW, "gimbal_yaw", APP_MOTOR_MODEL_DJI_GM6020),
        PLACEHOLDER_MOTOR(APP_MOTOR_ROLE_GIMBAL_PITCH, "gimbal_pitch", APP_MOTOR_MODEL_DAMIAO_6006),
        PLACEHOLDER_MOTOR(APP_MOTOR_ROLE_FRICTION_LEFT, "friction_left", APP_MOTOR_MODEL_DJI_M3508),
        PLACEHOLDER_MOTOR(APP_MOTOR_ROLE_FRICTION_RIGHT, "friction_right", APP_MOTOR_MODEL_DJI_M3508),
        PLACEHOLDER_MOTOR(APP_MOTOR_ROLE_TRIGGER, "trigger", APP_MOTOR_MODEL_DJI_M2006),
    },

    /* 履带底盘：轮距/轮径/最高速度全部待测。 */
    .chassis = {
        .track_width_m = 0.0f,
        .wheel_radius_m = 0.0f,
        .max_linear_speed_mps = 0.0f,
        .max_turn_rate_rad_s = 0.0f,
        .track_width_valid = false,
        .wheel_radius_valid = false,
        .max_linear_speed_valid = false,
        .max_turn_rate_valid = false,
        .valid = false,
    },

    /* 抬升：行程、同步偏差待测。 */
    .lift = {
        .travel_min_rad = 0.0f,
        .travel_max_rad = 0.0f,
        .sync_tolerance_rad = 0.0f,
        .travel_valid = false,
        .sync_valid = false,
        .valid = false,
    },

    /* 本车无机械臂，arm 配置保持 invalid。 */
    .arm = {
        .link_base_to_shoulder_m = 0.0f,
        .link_shoulder_elbow_m = 0.0f,
        .link_elbow_wrist_m = 0.0f,
        .reach_max_m = 0.0f,
        .joint_min_rad = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f},
        .joint_max_rad = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f},
        .links_valid = false,
        .reach_valid = false,
        .limits_valid = false,
        .valid = false,
    },

    /* 云台：机械限位、零位、方向均未确认；占位 0 不生效。 */
    .gimbal = {
        .yaw_min_rad = 0.0f,
        .yaw_max_rad = 0.0f,
        .pitch_min_rad = 0.0f,
        .pitch_max_rad = 0.0f,
        .limits_valid = false,
        .valid = false,
    },

    /* 发射：摩擦轮最大转速、拨弹额定速率均未标定；占位 0 不生效。 */
    .shooting = {
        .friction_max_rad_s = 0.0f,
        .trigger_rate_hz = 0.0f,
        .friction_valid = false,
        .trigger_valid = false,
        .valid = false,
    },

    /* DR16/DT7 DBUS 接收链路已接入（USART3 + DMA/IDLE）。
     * valid=true 仅表示映射参数结构完整；hardware_verified=false 表示尚未上板核验。
     * 通道/开关映射取协议手册与参考工程的初始值，需上板逐项核验后修改；
     * 失联门限 42 ms = 参考工程实测帧周期 14 ms 的 3 倍，本车尚未实测。 */
    .remote = {
        .link_timeout_ms = 42u,
        .channel_deadband = 0.03f,
        .channel_forward = 3u,
        .channel_turn = 2u,
        .channel_aux1 = 0u,
        .channel_aux2 = 1u,
        .sign_forward = 1,
        .sign_turn = -1,
        .sign_aux1 = -1,
        .sign_aux2 = 1,
        .gear_switch = 0u,
        .enable_switch = 1u,
        .mapping_valid = true,
        .timeout_valid = true,
        .deadband_valid = true,
        .channel_map_valid = true,
        .switch_map_valid = true,
        .valid = true,
        .hardware_verified = false,
    },

    /* 功率/能量：规则值来自 docs/RULES_REQUIREMENTS.md 当前版本；
     * 0x31 来源总线、超时和上板核验仍未确认，保持 invalid。 */
    .power = {
        .max_total_output_norm = 0.0f,
        .valid = false,
        .source_bus = APP_CAN_BUS_NONE,
        .source_bus_valid = false,
        .source_timeout_ms = 0u,
        .source_timeout_valid = false,
        .hardware_verified = false,
        .chassis_power_limit_w = 40.0f,
        .chassis_power_limit_valid = true,
        .buffer_energy_limit_j = 400.0f,
        .buffer_energy_limit_valid = true,
        .low_buffer_energy_j = 0.0f, /* 规则下限：能量 ≤ 0 J 时底盘断电 */
        .low_buffer_energy_valid = true,
    },
};

/* ------------------------------------------------------------------------- */
/* 校验与查询                                                                 */
/* ------------------------------------------------------------------------- */

#define COMMS_MISSING_MASK                                                        \
    (APP_CFG_MISSING_CONFIGURED | APP_CFG_MISSING_BUS | APP_CFG_MISSING_MOTOR_ID | \
     APP_CFG_MISSING_MSG_ID | APP_CFG_MISSING_CONTROL_MODE | APP_CFG_MISSING_DIRECTION)

#define MOTION_MISSING_MASK                                                       \
    (COMMS_MISSING_MASK | APP_CFG_MISSING_GEAR_RATIO | APP_CFG_MISSING_ZERO_OFFSET | \
     APP_CFG_MISSING_LIMIT | APP_CFG_MISSING_PID)

static app_status_t g_status = APP_STATUS_UNCONFIGURED;
static app_config_report_t g_report = {0};

static uint32_t config_motor_missing(const app_motor_config_t *motor)
{
    uint32_t missing = APP_CFG_MISSING_NONE;

    if (!motor->configured) {
        missing |= APP_CFG_MISSING_CONFIGURED;
    }
    if (!motor->bus_valid || motor->bus == APP_CAN_BUS_NONE) {
        missing |= APP_CFG_MISSING_BUS;
    }
    if (!motor->id_valid) {
        missing |= APP_CFG_MISSING_MOTOR_ID;
    }
    if (!motor->msg_id_valid || motor->tx_msg_id == 0u || motor->rx_msg_id == 0u) {
        missing |= APP_CFG_MISSING_MSG_ID;
    }
    if (!motor->control_mode_valid || motor->control_mode == APP_MOTOR_CONTROL_NONE) {
        missing |= APP_CFG_MISSING_CONTROL_MODE;
    }
    if (!motor->direction_valid || (motor->direction != 1 && motor->direction != -1)) {
        missing |= APP_CFG_MISSING_DIRECTION;
    }
    if (!motor->gear_ratio_valid || motor->gear_ratio <= 0.0f) {
        missing |= APP_CFG_MISSING_GEAR_RATIO;
    }
    if (!motor->zero_offset_valid) {
        missing |= APP_CFG_MISSING_ZERO_OFFSET;
    }
    if (!motor->limit_valid || motor->limit_max_rad <= motor->limit_min_rad) {
        missing |= APP_CFG_MISSING_LIMIT;
    }
    if ((motor->control_mode == APP_MOTOR_CONTROL_SPEED ||
         motor->control_mode == APP_MOTOR_CONTROL_POSITION) &&
        !motor->pid.valid) {
        missing |= APP_CFG_MISSING_PID;
    }

    return missing;
}

const app_config_t *app_config_get(void)
{
    return &g_config;
}

app_status_t app_config_init(void)
{
    uint32_t index;
    bool mechanisms_ready;
    bool all_motors_ready;

    /* 允许重复调用：先清零累积字段。 */
    g_report.motors_total = APP_MOTOR_COUNT;
    g_report.motors_configured = 0u;
    g_report.motors_motion_usable = 0u;
    g_report.first_incomplete_motor = -1;

    for (index = 0u; index < APP_MOTOR_COUNT; index++) {
        const app_motor_config_t *motor = &g_config.motors[index];
        const uint32_t missing = config_motor_missing(motor);

        g_report.missing[index] = missing;

        if ((missing & COMMS_MISSING_MASK) == 0u) {
            g_report.motors_configured++;
        }
        if ((missing & MOTION_MISSING_MASK) == 0u) {
            g_report.motors_motion_usable++;
        }
        if (g_report.first_incomplete_motor < 0 &&
            (missing & MOTION_MISSING_MASK) != 0u) {
            g_report.first_incomplete_motor = (int32_t)index;
        }
    }

    g_report.chassis_valid = g_config.chassis.valid;
    g_report.lift_valid = g_config.lift.valid;
    g_report.arm_valid = g_config.arm.valid;
    g_report.gimbal_valid = g_config.gimbal.valid;
    g_report.shooting_valid = g_config.shooting.valid;
    g_report.remote_valid = g_config.remote.valid;
    g_report.power_valid = g_config.power.valid;
    g_report.updated_ms = app_time_now_ms();

    /* 底盘与抬升是两车公共必配机构；专有机构按本车种类判定。
     * 炮台车：底盘 + 抬升 + 云台 + 发射全部确认才算机构就绪。 */
    if (g_config.vehicle_kind == APP_VEHICLE_KIND_ENGINEER) {
        mechanisms_ready = g_config.chassis.valid && g_config.lift.valid && g_config.arm.valid;
    } else if (g_config.vehicle_kind == APP_VEHICLE_KIND_TURRET) {
        mechanisms_ready = g_config.chassis.valid && g_config.lift.valid &&
                           g_config.gimbal.valid && g_config.shooting.valid;
    } else {
        mechanisms_ready = false;
    }
    all_motors_ready = (g_report.motors_configured == APP_MOTOR_COUNT);

    if ((g_report.motors_configured == 0u) && !mechanisms_ready) {
        g_status = APP_STATUS_UNCONFIGURED;
    } else if (all_motors_ready && mechanisms_ready) {
        g_status = APP_STATUS_OK;
    } else {
        g_status = APP_STATUS_NOT_READY;
    }
    g_report.status = g_status;

    return g_status;
}

app_status_t app_config_status(void)
{
    return g_status;
}

const app_config_report_t *app_config_report(void)
{
    return &g_report;
}

app_vehicle_kind_t app_config_vehicle_kind(void)
{
    return g_config.vehicle_kind;
}

const app_motor_config_t *app_config_motor(int slot)
{
    if (slot < 0 || slot >= (int)APP_MOTOR_COUNT) {
        return NULL;
    }
    return &g_config.motors[slot];
}

uint32_t app_config_motor_missing(int slot)
{
    if (slot < 0 || slot >= (int)APP_MOTOR_COUNT) {
        return 0xFFFFFFFFu;
    }
    return g_report.missing[slot];
}

bool app_config_motor_comms_usable(int slot)
{
    if (slot < 0 || slot >= (int)APP_MOTOR_COUNT) {
        return false;
    }
    return (g_report.missing[slot] & COMMS_MISSING_MASK) == 0u;
}

bool app_config_motor_motion_usable(int slot)
{
    if (slot < 0 || slot >= (int)APP_MOTOR_COUNT) {
        return false;
    }
    return (g_report.missing[slot] & MOTION_MISSING_MASK) == 0u;
}

int app_config_find_motor(app_motor_role_t role)
{
    uint32_t index;

    if (role == APP_MOTOR_ROLE_NONE) {
        return -1;
    }
    for (index = 0u; index < APP_MOTOR_COUNT; index++) {
        if (g_config.motors[index].role == role) {
            return (int)index;
        }
    }
    return -1;
}
