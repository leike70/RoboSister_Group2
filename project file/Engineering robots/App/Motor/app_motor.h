/**
  ******************************************************************************
  * @file    app_motor.h
  * @brief   设备注册与统一电机输出接口（两车公共）。
  *
  * 职责（根 README 第 5、6 节）：
  *  - 按配置表注册逻辑电机，维护每台设备的可用性/在线/故障状态；
  *  - 每周期收集各应用模块的请求（staging），再由唯一入口 app_motor_submit()
  *    结合配置门控、保护状态和功率预算，提交每台电机唯一的最终命令；
  *  - 提供反馈快照（有效性 + 时间戳），不伪造任何测量值。
  *
  * 首轮边界：不实例化 DJI/达妙驱动对象，不发送任何 CAN 报文；submit 只生成
  * 软件侧最终命令，供后续适配层唯一读取。未配置的电机不会被注册，也不会输出。
  ******************************************************************************
  */
#ifndef APP_MOTOR_H
#define APP_MOTOR_H

#include <stdbool.h>
#include <stdint.h>

#include "Common/app_types.h"
#include "app_config.h"

#ifdef __cplusplus
extern "C" {
#endif

/** 单台电机反馈快照（机构侧单位）。valid=false 时其余字段无意义。 */
typedef struct {
    float position_rad;    /**< 机构角度，rad */
    float velocity_rad_s;  /**< 机构角速度，rad/s */
    float current_norm;    /**< 归一化电流，[-1,1]；物理安培值待核验 */
    bool valid;
    uint32_t timestamp_ms;
} app_motor_feedback_t;

/** 应用层命令类型。value 的单位随类型变化。 */
typedef enum {
    APP_MOTOR_CMD_NONE = 0,       /**< 无请求 */
    APP_MOTOR_CMD_CURRENT_NORM,   /**< 归一化电流 [-1,1] */
    APP_MOTOR_CMD_VOLTAGE_NORM,   /**< 归一化电压 [-1,1] */
    APP_MOTOR_CMD_SPEED_RAD_S,    /**< 机构角速度，rad/s */
    APP_MOTOR_CMD_POSITION_RAD    /**< 机构角度，rad */
} app_motor_cmd_mode_t;

/** 应用模块写入的每周期请求（不直接生效）。 */
typedef struct {
    app_motor_cmd_mode_t mode;
    float value;
    bool valid;
} app_motor_request_t;

/** 唯一提交入口产出的最终命令，每台电机每周期只有一份。 */
typedef struct {
    app_motor_cmd_mode_t mode;  /**< 应用层命令类型 */
    float value;                /**< 应用层单位数值 */
    bool enabled;               /**< true 才允许后续适配层输出 */
    bool driver_valid;          /**< 驱动侧换算是否成功 */
    float driver_value;         /**< 驱动侧数值：归一化量 / 转子 rpm / 转子圈数 */
    float current_limit_norm;   /**< 驱动最终电流/力矩上限，0..1；1=不额外限幅 */
    bool power_limited;         /**< 本周期是否因底盘功率策略降额 */
    uint32_t committed_ms;
} app_motor_output_t;

/** 单台电机运行状态。 */
typedef struct {
    app_status_t status;        /**< UNCONFIGURED / NOT_READY / OFFLINE / OK / NOT_IMPLEMENTED */
    bool registered;            /**< 是否已按配置注册 */
    bool comms_usable;
    bool motion_usable;
    uint32_t missing_flags;     /**< app_config_missing_t 位组合 */
    uint32_t last_feedback_ms;
} app_motor_device_t;

/* ------------------------------------------------------------------------- */
/* 初始化与反馈                                                               */
/* ------------------------------------------------------------------------- */

/** 按配置表注册电机；未确认配置一律不注册、不构造硬件对象。 */
app_status_t app_motor_init(uint32_t now_ms);

/** 真实驱动适配层就绪后才允许最终命令 enabled；默认 false。 */
void app_motor_set_adapter_ready(bool ready);
bool app_motor_adapter_ready(void);

/** 刷新反馈快照；首轮无硬件适配，返回 NOT_IMPLEMENTED 并保持 valid=false。 */
app_status_t app_motor_update_feedback(uint32_t now_ms);

/** 读单台电机反馈快照；未注册/无适配时 valid=false 且返回相应状态。 */
app_status_t app_motor_feedback_get(int slot, app_motor_feedback_t *out);

/** 单台电机运行状态。 */
app_status_t app_motor_device_get(int slot, app_motor_device_t *out);

/** 已注册电机数量（首轮为 0）。 */
int app_motor_registered_count(void);

/** 整体电机模块状态：全部未配置 → UNCONFIGURED；已配置但无反馈 → NOT_READY。 */
app_status_t app_motor_status(void);

/* ------------------------------------------------------------------------- */
/* 请求收集（各应用模块调用）                                                  */
/* ------------------------------------------------------------------------- */

/** 清空本周期所有请求，避免旧请求残留。由周期任务在每个周期开始时调用。 */
void app_motor_clear_requests(void);

/**
 * @brief 提交一台电机的本周期请求（staging）。
 * @param slot 电机下标 [0, APP_MOTOR_COUNT)
 * @param mode 命令类型
 * @param value 数值，单位随 mode
 * @return 参数/配置检查结果；被拒绝时不会留下请求。
 */
app_status_t app_motor_request(int slot, app_motor_cmd_mode_t mode, float value);

/** 读取当前 staging 请求（调试用）。 */
app_status_t app_motor_request_get(int slot, app_motor_request_t *out);

/* ------------------------------------------------------------------------- */
/* 唯一输出提交入口（每周期只调用一次）                                        */
/* ------------------------------------------------------------------------- */

/**
 * @brief 本周期唯一的整车输出提交入口。
 *
 * 顺序：配置门控 → 保护门控 → 功率预算 → 单位换算 → 写入每台电机唯一最终命令。
 * 任一环节不满足时该电机输出 disabled，不会保留上一周期命令。
 * @return 提交结果：APP_STATUS_OK 表示至少一台电机生成命令；
 *         UNCONFIGURED/NOT_READY 表示整车仍处于未使能/未就绪状态。
 */
app_status_t app_motor_submit(uint32_t now_ms);

/** 读取某台电机的最终命令（供适配层/调试器读取，唯一数据来源）。 */
app_status_t app_motor_output_get(int slot, app_motor_output_t *out);

/** 立即失效所有最终命令（失联、模式切换、故障时使用）。 */
void app_motor_disable_all(uint32_t now_ms);

#ifdef __cplusplus
}
#endif

#endif /* APP_MOTOR_H */
