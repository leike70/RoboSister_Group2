/**
  ******************************************************************************
  * @file    app_config.h
  * @brief   本车配置表接口：逻辑电机、机构尺寸、遥控/功率配置及有效性标记。
  *
  * 两车都有一份**本工程内**的 Config/app_config.c（内容不同），本头文件两车一致。
  *
  * 有效性规则（根 README 第 8 节）：
  *  - 每个未确认项都有独立 valid/valid_* 标记，占位数值不会自动生效。
  *  - configured 是整台电机的总开关；它和分项标记都为真才允许注册/使用。
  *  - 未确认减速比/零位/方向时，禁止把反馈换算成“机构角度”这类伪造测量。
  ******************************************************************************
  */
#ifndef APP_CONFIG_H
#define APP_CONFIG_H

#include <stdbool.h>
#include <stdint.h>

#include "Common/app_types.h"

#ifdef __cplusplus
extern "C" {
#endif

/** 本车种类，用于区分专有机构和公共配置的适用性。 */
typedef enum {
    APP_VEHICLE_KIND_UNKNOWN = 0,
    APP_VEHICLE_KIND_ENGINEER,   /**< 工程车：机械臂 */
    APP_VEHICLE_KIND_TURRET      /**< 炮台车：云台/摩擦轮/拨弹 */
} app_vehicle_kind_t;

/** 电机型号。达妙电机不能复用 DJI 驱动。 */
typedef enum {
    APP_MOTOR_MODEL_UNKNOWN = 0,
    APP_MOTOR_MODEL_DJI_M3508,
    APP_MOTOR_MODEL_DJI_M2006,
    APP_MOTOR_MODEL_DJI_GM6020,
    APP_MOTOR_MODEL_DAMIAO_4310,
    APP_MOTOR_MODEL_DAMIAO_6006
} app_motor_model_t;

/** CAN 总线编号。总线分配待确认。 */
typedef enum {
    APP_CAN_BUS_NONE = 0,
    APP_CAN_BUS_1,
    APP_CAN_BUS_2
} app_can_bus_t;

/** 驱动控制模式。应用层请求的单位由 action 决定，见 app_motor.h。 */
typedef enum {
    APP_MOTOR_CONTROL_NONE = 0,
    APP_MOTOR_CONTROL_CURRENT,   /**< 归一化电流 [-1,1] */
    APP_MOTOR_CONTROL_VOLTAGE,   /**< 归一化电压 [-1,1] */
    APP_MOTOR_CONTROL_SPEED,     /**< 角速度 rad/s（关节/轮） */
    APP_MOTOR_CONTROL_POSITION   /**< 角度 rad（关节/轮） */
} app_motor_control_mode_t;

/** 逻辑电机角色（两车全部角色；本车只用其中 9 个）。 */
typedef enum {
    APP_MOTOR_ROLE_NONE = 0,
    /* 公共底盘 */
    APP_MOTOR_ROLE_TRACK_LEFT,
    APP_MOTOR_ROLE_TRACK_RIGHT,
    APP_MOTOR_ROLE_LIFT_FRONT,
    APP_MOTOR_ROLE_LIFT_REAR,
    /* 工程车 */
    APP_MOTOR_ROLE_ARM_YAW,
    APP_MOTOR_ROLE_ARM_SHOULDER,
    APP_MOTOR_ROLE_ARM_ELBOW,
    APP_MOTOR_ROLE_ARM_WRIST_ROLL,
    APP_MOTOR_ROLE_ARM_GRIPPER,
    /* 炮台车 */
    APP_MOTOR_ROLE_GIMBAL_YAW,
    APP_MOTOR_ROLE_GIMBAL_PITCH,
    APP_MOTOR_ROLE_FRICTION_LEFT,
    APP_MOTOR_ROLE_FRICTION_RIGHT,
    APP_MOTOR_ROLE_TRIGGER
} app_motor_role_t;

/** 单电机 PID 配置；驱动内置默认值不是本车调参结果，valid=false 表示未整定。 */
typedef struct {
    float kp;
    float ki;
    float kd;
    float max_output_norm;      /**< 输出限幅，归一化 */
    float integral_limit_norm;  /**< 积分限幅，归一化 */
    bool valid;                 /**< true 才允许用于闭环 */
} app_pid_config_t;

/**
 * @brief 单个逻辑电机的完整配置。
 *
 * 未知项保留占位数值，但对应 valid_* 必须为 false；任何占位值都不会自动生效。
 */
typedef struct {
    const char *name;                      /**< 逻辑名称，调试显示用 */
    app_motor_role_t role;                 /**< 逻辑角色 */
    app_motor_model_t model;               /**< 型号 */
    app_can_bus_t bus;                     /**< CAN 总线 */
    uint32_t motor_id;                     /**< 电机 ID（含组合帧槽位含义） */
    uint32_t tx_msg_id;                    /**< 控制报文 ID */
    uint32_t rx_msg_id;                    /**< 反馈报文 ID */
    app_motor_control_mode_t control_mode; /**< 驱动控制模式 */
    int8_t direction;                      /**< +1 / -1，机构正方向与转子正方向的关系 */
    float gear_ratio;                      /**< 减速比：转子圈数 / 机构圈数，>0 */
    float zero_offset_rad;                 /**< 机构零位对应的转子位置，rad（机构侧） */
    float limit_min_rad;                   /**< 机构下限，rad */
    float limit_max_rad;                   /**< 机构上限，rad */
    app_pid_config_t pid;                  /**< 速度/位置环 PID */

    bool configured;                       /**< 总开关：确认为 true 才可能参与注册 */
    bool bus_valid;
    bool id_valid;
    bool msg_id_valid;
    bool control_mode_valid;
    bool direction_valid;
    bool gear_ratio_valid;
    bool zero_offset_valid;
    bool limit_valid;
} app_motor_config_t;

/** 履带底盘机构配置。 */
typedef struct {
    float track_width_m;         /**< 左右履带中心距，m */
    float wheel_radius_m;        /**< 驱动轮半径，m */
    float max_linear_speed_mps;  /**< 允许的最大线速度，m/s */
    float max_turn_rate_rad_s;   /**< 允许的最大转向角速度，rad/s */
    bool track_width_valid;
    bool wheel_radius_valid;
    bool max_linear_speed_valid;
    bool max_turn_rate_valid;
    bool valid;                  /**< 机构整体有效标记 */
} app_chassis_config_t;

/** 前后抬升机构配置。 */
typedef struct {
    float travel_min_rad;    /**< 机构行程下限，rad */
    float travel_max_rad;    /**< 机构行程上限，rad */
    float sync_tolerance_rad;/**< 前后同步允许偏差，rad */
    bool travel_valid;
    bool sync_valid;
    bool valid;
} app_lift_config_t;

/** 机械臂配置：连杆尺寸、关节限位、可达范围。 */
typedef struct {
    float link_base_to_shoulder_m; /**< 底座到肩关节高度，m */
    float link_shoulder_elbow_m;   /**< 大臂长度，m */
    float link_elbow_wrist_m;      /**< 小臂长度，m */
    float reach_max_m;             /**< 末端最大可达半径，m */
    float joint_min_rad[5];        /**< 五关节下限，rad（顺序同 app_arm.h） */
    float joint_max_rad[5];        /**< 五关节上限，rad */
    bool links_valid;
    bool reach_valid;
    bool limits_valid;
    bool valid;
} app_arm_config_t;

/** 云台配置。 */
typedef struct {
    float yaw_min_rad;
    float yaw_max_rad;
    float pitch_min_rad;
    float pitch_max_rad;
    bool limits_valid;
    bool valid;
} app_gimbal_config_t;

/** 发射机构配置。 */
typedef struct {
    float friction_max_rad_s; /**< 摩擦轮允许最大角速度，rad/s */
    float trigger_rate_hz;    /**< 拨弹额定速率，Hz */
    bool friction_valid;
    bool trigger_valid;
    bool valid;
} app_shooting_config_t;

/** 遥控配置：DR16/DT7 DBUS 接收、通道/开关映射与失联门限。 */
typedef struct {
    uint32_t link_timeout_ms; /**< 超过该时间无有效帧判离线，ms */
    float channel_deadband;   /**< 归一化死区，[0,1] 内取值 */
    uint8_t channel_forward;  /**< 前进通道下标 0..3（DBUS ch0..ch3） */
    uint8_t channel_turn;     /**< 转向通道下标 0..3 */
    uint8_t channel_aux1;     /**< 本车功能轴 1 通道下标 0..3 */
    uint8_t channel_aux2;     /**< 本车功能轴 2 通道下标 0..3 */
    int8_t sign_forward;      /**< 前进方向 +1/-1 */
    int8_t sign_turn;         /**< 转向方向 +1/-1 */
    int8_t sign_aux1;         /**< 功能轴 1 方向 +1/-1 */
    int8_t sign_aux2;         /**< 功能轴 2 方向 +1/-1 */
    uint8_t gear_switch;      /**< 档位开关：0=S1(左)、1=S2(右) */
    uint8_t enable_switch;    /**< 使能/复位开关：0=S1、1=S2 */
    bool mapping_valid;       /**< 逻辑输入映射是否已确认 */
    bool timeout_valid;       /**< 失联门限是否已确认 */
    bool deadband_valid;      /**< 死区是否已确认 */
    bool channel_map_valid;   /**< 通道下标/方向是否已确认 */
    bool switch_map_valid;    /**< 开关职责是否已确认 */
    bool valid;
    bool hardware_verified;  /**< 通道/开关已在本车接收机上实测确认 */
} app_remote_config_t;

/** 功率/能量来源与规则限制配置。 */
typedef struct {
    float max_total_output_norm; /**< 旧的归一化输出预算（代理量，不是瓦数） */
    bool valid;                  /**< 旧代理预算是否启用 */

    app_can_bus_t source_bus;    /**< 0x31 来源总线：NONE/1/2 */
    bool source_bus_valid;       /**< 来源总线是否已确认 */
    uint32_t source_timeout_ms;  /**< 有效帧超时，ms；0 表示未配置 */
    bool source_timeout_valid;   /**< 超时是否已确认 */
    bool hardware_verified;      /**< 0x31 是否已在本车上板核验 */

    float chassis_power_limit_w;      /**< 底盘功率上限 W，规则值 */
    bool chassis_power_limit_valid;   /**< 规则值是否适用当前规则版本 */
    float buffer_energy_limit_j;      /**< 缓冲能量上限 J，规则值 */
    bool buffer_energy_limit_valid;
    float low_buffer_energy_j;        /**< 低能量策略阈值 J（不填则不做低能量降额） */
    bool low_buffer_energy_valid;
} app_power_config_t;

/** 本车完整配置。公共机构与专有机构放在同一张表，未适用项保持 invalid。 */
typedef struct {
    app_vehicle_kind_t vehicle_kind;
    app_motor_config_t motors[APP_MOTOR_COUNT];
    app_chassis_config_t chassis;
    app_lift_config_t lift;
    app_arm_config_t arm;
    app_gimbal_config_t gimbal;
    app_shooting_config_t shooting;
    app_remote_config_t remote;
    app_power_config_t power;
} app_config_t;

/** 电机缺失项位标志，用于报告“哪些未确认项还没填”。 */
typedef enum {
    APP_CFG_MISSING_NONE = 0u,
    APP_CFG_MISSING_BUS = 1u << 0,
    APP_CFG_MISSING_MOTOR_ID = 1u << 1,
    APP_CFG_MISSING_MSG_ID = 1u << 2,
    APP_CFG_MISSING_CONTROL_MODE = 1u << 3,
    APP_CFG_MISSING_DIRECTION = 1u << 4,
    APP_CFG_MISSING_GEAR_RATIO = 1u << 5,
    APP_CFG_MISSING_ZERO_OFFSET = 1u << 6,
    APP_CFG_MISSING_LIMIT = 1u << 7,
    APP_CFG_MISSING_PID = 1u << 8,
    APP_CFG_MISSING_CONFIGURED = 1u << 9
} app_config_missing_t;

/** 配置检查结果，供调试器/后续遥测读取。 */
typedef struct {
    app_status_t status;                 /**< 整体状态：UNCONFIGURED / NOT_READY / OK */
    uint32_t motors_total;               /**< 逻辑电机总数，应为 APP_MOTOR_COUNT */
    uint32_t motors_configured;          /**< 通信必需项齐全的电机数 */
    uint32_t motors_motion_usable;       /**< 含减速比/零位/限位（及闭环 PID）的电机数 */
    int32_t first_incomplete_motor;      /**< 第一台缺项电机下标，-1 表示无 */
    uint32_t missing[APP_MOTOR_COUNT];   /**< 每台电机的缺失项位标志 */
    bool chassis_valid;
    bool lift_valid;
    bool arm_valid;
    bool gimbal_valid;
    bool shooting_valid;
    bool remote_valid;
    bool power_valid;
    uint32_t updated_ms;
} app_config_report_t;

/** 取得本车只读配置表。 */
const app_config_t *app_config_get(void);

/**
 * @brief 校验配置表并缓存结果；只读、不创建硬件对象、不发报文。
 * @return 整体状态（UNCONFIGURED / NOT_READY / OK）。
 */
app_status_t app_config_init(void);

/** 最近一次 app_config_init 的整体状态。 */
app_status_t app_config_status(void);

/** 最近一次校验的详细结果。 */
const app_config_report_t *app_config_report(void);

/** 本车种类。 */
app_vehicle_kind_t app_config_vehicle_kind(void);

/** 按下标取电机配置，越界返回 NULL。 */
const app_motor_config_t *app_config_motor(int slot);

/** 该电机的缺失项位标志。 */
uint32_t app_config_motor_missing(int slot);

/** 通信必需项（总线/ID/报文ID/控制模式/方向）是否齐全。 */
bool app_config_motor_comms_usable(int slot);

/** 机构运动必需项（通信项 + 减速比 + 零位 + 限位，闭环还需 PID）是否齐全。 */
bool app_config_motor_motion_usable(int slot);

/** 按角色查电机下标，未找到返回 -1。 */
int app_config_find_motor(app_motor_role_t role);

#ifdef __cplusplus
}
#endif

#endif /* APP_CONFIG_H */
