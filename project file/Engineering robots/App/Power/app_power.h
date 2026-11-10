/**
  ******************************************************************************
  * @file    app_power.h
  * @brief   裁判 0x31 功率/能量反馈、底盘功率策略与输出预算接口。
  *
  * 数据链路：CAN BSP 接收 → app_power_on_can_frame() 只保存原始帧/到达时间
  * → app_power_update() 在任务上下文解析、判超时并计算策略 → app_motor_submit()
  * 只对底盘电机施加最终约束。
  *
  * `measured_*` 是裁判反馈真值；`allowed_*` 是规则允许值；`max_total_output_norm`
  * 仍是旧的归一化代理预算，三者不能混用。
  ******************************************************************************
  */
#ifndef APP_POWER_H
#define APP_POWER_H

#include <stdbool.h>
#include <stdint.h>

#include "Common/app_types.h"
#include "app_config.h"

#ifdef __cplusplus
extern "C" {
#endif

#define APP_POWER_CAN_ID 0x31u
#define APP_POWER_FRAME_DLC 4u

/** 0x31 原始反馈错误位。 */
typedef enum {
    APP_POWER_ERR_NONE = 0u,
    APP_POWER_ERR_CONFIG = 1u << 0,
    APP_POWER_ERR_NO_FRAME = 1u << 1,
    APP_POWER_ERR_TIMEOUT = 1u << 2,
    APP_POWER_ERR_BAD_FRAME = 1u << 3,
    APP_POWER_ERR_BUS_MISMATCH = 1u << 4,
    APP_POWER_ERR_UNVERIFIED = 1u << 5
} app_power_error_t;

/** 0x31 一致快照。所有时间均为本地毫秒基准。 */
typedef struct {
    float measured_chassis_power_w; /**< 实测底盘功率，W；负值保留 */
    float buffer_energy_j;          /**< 裁判缓冲能量，J；负值保留 */
    app_can_bus_t source_bus;       /**< 实际接收总线 */
    uint32_t frame_arrival_ms;      /**< 有效帧到达时间，不能被读取刷新 */
    uint32_t frame_count;           /**< 有效帧计数 */
    uint32_t invalid_frame_count;   /**< 非法 ID/DLC/解析计数 */
    uint32_t timeout_count;         /**< 在线→超时转换次数 */
    uint32_t error_flags;           /**< app_power_error_t 位组合 */
    bool frame_valid;               /**< 至少收到过一个有效帧 */
    bool source_online;             /**< 当前在超时窗口内 */
    bool config_valid;              /**< 总线/超时配置齐全 */
    bool limits_valid;              /**< 规则功率/能量限制齐全 */
    bool hardware_verified;         /**< 已上板核验 0x31 */
    app_status_t status;
    uint32_t updated_ms;
} app_power_feedback_t;

/** 底盘功率策略状态。 */
typedef enum {
    APP_POWER_POLICY_UNAVAILABLE = 0,
    APP_POWER_POLICY_NORMAL,
    APP_POWER_POLICY_LIMITED,
    APP_POWER_POLICY_LOW_ENERGY
} app_power_policy_state_t;

/** 由反馈计算的底盘策略；不是电机功率模型。 */
typedef struct {
    app_power_policy_state_t state;
    float measured_power_w;
    float buffer_energy_j;
    float chassis_power_limit_w;  /**< 规则允许值，W */
    float buffer_energy_limit_j;  /**< 规则上限，J */
    float allowed_power_w;        /**< 当前允许功率参考，W；不是测量值 */
    float power_margin_w;         /**< limit - measured，可为负 */
    float buffer_margin_j;        /**< energy - 低能量阈值，可为负 */
    float chassis_scale;          /**< 只给底盘速度/电流的保守缩放 [0,1] */
    bool over_power;
    bool low_energy;
    bool source_online;
    bool source_ready;            /**< 配置/超时/上板核验都满足 */
    uint32_t updated_ms;
} app_power_policy_t;

/** 旧代理预算快照，保留给非底盘路径兼容。 */
typedef struct {
    float available_power_w;      /**< 旧字段：恒 0，不要把实测功率塞进来 */
    float max_total_output_norm;  /**< 归一化输出绝对值之和的代理预算 */
    bool source_valid;            /**< 旧字段：0x31 接入后也不代表真实瓦数 */
    bool valid;
    uint32_t updated_ms;
} app_power_budget_t;

/** 初始化；不注册 CAN，不阻塞。 */
app_status_t app_power_init(uint32_t now_ms);

/** ISR 入口：只保存原始帧和到达时间；解析/策略在 update 中完成。 */
void app_power_on_can_frame(uint8_t bus, uint32_t id, const uint8_t *data,
                            uint8_t dlc, uint32_t arrival_ms);

/** 任务上下文：解析最新原始帧、判断超时、计算策略。 */
app_status_t app_power_update(uint32_t now_ms);

/** 读取一致的 0x31 快照。 */
app_status_t app_power_get_feedback(app_power_feedback_t *out);

/** 读取底盘功率策略。 */
app_status_t app_power_get_policy(app_power_policy_t *out);

/** 旧代理预算快照。 */
app_status_t app_power_get_budget(app_power_budget_t *out);

/** 旧代理缩放入口；不用于底盘 0x31 策略。 */
app_status_t app_power_limit_scale(float requested_total_norm, float *scale_out);

/** 模块状态。 */
app_status_t app_power_status(void);

#ifdef __cplusplus
}
#endif

#endif /* APP_POWER_H */
