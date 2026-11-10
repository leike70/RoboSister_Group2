/**
  ******************************************************************************
  * @file    app_protection.h
  * @brief   整车保护与运行许可（两车公共）。
  *
  * 职责：汇总配置、遥控、设备、功率状态，给出唯一“是否允许输出”的判断；
  * 维护运行状态与故障标志。各模块不得自行解除故障。
  *
  * 首轮：配置未确认、遥控离线、无设备反馈，因此永远不会进入允许输出状态。
  ******************************************************************************
  */
#ifndef APP_PROTECTION_H
#define APP_PROTECTION_H

#include <stdbool.h>
#include <stdint.h>

#include "Common/app_types.h"

#ifdef __cplusplus
extern "C" {
#endif

/** 整车运行状态。 */
typedef enum {
    APP_RUN_STATE_DISABLED = 0,  /**< 未使能/显式停用 */
    APP_RUN_STATE_WAIT_CONFIG,   /**< 配置未确认 */
    APP_RUN_STATE_WAIT_INPUT,    /**< 输入（遥控）离线 */
    APP_RUN_STATE_WAIT_DEVICE,   /**< 设备未注册/未就绪 */
    APP_RUN_STATE_WAIT_POWER,    /**< 功率预算/来源不可用 */
    APP_RUN_STATE_ARMED,         /**< 已满足条件、等待模式动作 */
    APP_RUN_STATE_RUNNING,       /**< 允许输出且当前处于动作模式 */
    APP_RUN_STATE_FAULT          /**< 存在未解除故障 */
} app_run_state_t;

/** 故障位标志，可组合。 */
typedef enum {
    APP_FAULT_NONE = 0u,
    APP_FAULT_MANUAL = 1u << 0,         /**< 上层显式置位 */
    APP_FAULT_CYCLE_OVERRUN = 1u << 1,  /**< 控制周期超时 */
    APP_FAULT_DEVICE = 1u << 2,         /**< 设备故障 */
    APP_FAULT_POWER = 1u << 3,          /**< 功率预算异常 */
    APP_FAULT_CONFIG = 1u << 4,         /**< 配置在运行中失效 */
    APP_FAULT_CAN = 1u << 5             /**< CAN 启动/总线/发送持续异常 */
} app_fault_flags_t;

/** 保护模块状态快照。 */
typedef struct {
    app_run_state_t state;
    app_status_t config_status;
    app_status_t remote_status;
    app_status_t device_status;
    app_status_t power_status;
    bool output_allowed;    /**< 唯一输出许可 */
    uint32_t active_faults; /**< 当前故障位 */
    uint32_t latched_faults;/**< 锁存故障位，需显式清除 */
    uint32_t updated_ms;
} app_protection_report_t;

/** 初始化保护模块（未使能、无故障）。 */
app_status_t app_protection_init(uint32_t now_ms);

/**
 * @brief 每个周期汇总判断一次运行许可。
 * @note  只读各模块状态，不触发动作；会锁存当前故障位。
 */
app_status_t app_protection_evaluate(uint32_t now_ms);

/** 逻辑使能请求（来自抽象输入，物理开关/通道映射未定）；未使能时上层传 false。 */
void app_protection_set_arm_request(bool requested);

/** 本周期是否处于动作模式（由周期任务按模式判断）。 */
void app_protection_set_mode_active(bool active);

/** 当前是否允许输出（保护模块唯一结论）。 */
bool app_protection_output_allowed(void);

/** 置位故障；active 与 latched 同时置位。 */
void app_protection_raise(uint32_t fault_flags);

/**
 * @brief 监督逻辑核实故障原因消失后解除指定活动位，保留历史锁存。
 * @note  所有写接口由整车控制/监督任务串行调用；ISR/其他任务须转交故障事件。
 *        resolve 不清锁存、不恢复输出许可。
 */
void app_protection_resolve(uint32_t fault_flags);

/**
 * @brief 清除锁存故障；仅允许上层监督逻辑调用，各模块不得自行调用。
 * @return true 表示已清除；false 表示仍有活动故障，清除被拒绝。
 */
bool app_protection_clear_latched(void);

/** 逻辑急停：置位 APP_FAULT_MANUAL，撤销使能和动作模式；不占用物理按键。 */
void app_protection_emergency_stop(void);

/** 释放逻辑急停的活动位（锁存仍需 clear_latched，且不得绕过其他活动故障）。 */
void app_protection_release_emergency_stop(void);

/** 逻辑急停当前是否保持。 */
bool app_protection_emergency_stop_active(void);

/** 只读状态快照。 */
const app_protection_report_t *app_protection_report(void);

#ifdef __cplusplus
}
#endif

#endif /* APP_PROTECTION_H */
