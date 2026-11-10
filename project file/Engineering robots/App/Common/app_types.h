/**
  ******************************************************************************
  * @file    app_types.h
  * @brief   两车公共应用层基础类型：状态码、单位约定、时间戳语义。
  *
  * 约定（根 README 第 6 节）：
  *  - 应用层角度 rad、角速度 rad/s、长度 m、时间 ms（函数名带单位后缀）。
  *  - 归一化量 [-1,1]：遥控通道、电流/电压指令等，物理意义须另行核验。
  *  - 任何“未配置/未实现/离线”必须显式返回状态，不得用 0 冒充有效测量。
  ******************************************************************************
  */
#ifndef APP_COMMON_APP_TYPES_H
#define APP_COMMON_APP_TYPES_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/** 每车逻辑电机数量（两车均为 9，型号组合不同）。 */
#define APP_MOTOR_COUNT 9u

/** 时间戳“从未更新”的哨兵值，与 FreeRTOS tick 0 区分。 */
#define APP_TIME_NEVER 0u

/**
 * @brief 通用状态码。
 *
 * 用于模块整体状态、单设备状态和接口返回值；至少可区分
 * 未配置、未就绪、正常、离线、未实现、参数非法、故障。
 */
typedef enum {
    APP_STATUS_UNCONFIGURED = 0,   /**< 配置缺失或未确认，禁止使用。 */
    APP_STATUS_NOT_READY,          /**< 已配置但依赖尚未满足（数据/标定/初始化未完成）。 */
    APP_STATUS_OK,                 /**< 正常。 */
    APP_STATUS_OFFLINE,            /**< 设备或链路离线。 */
    APP_STATUS_NOT_IMPLEMENTED,    /**< 接口已定义、实现尚不存在（占位）。 */
    APP_STATUS_INVALID_PARAM,      /**< 入参越界/非法，本次调用被拒绝。 */
    APP_STATUS_FAULT               /**< 已检测到故障，需显式处理。 */
} app_status_t;

/** 把浮点数限幅到 [min_value, max_value]。min_value 必须不大于 max_value。 */
float app_clampf(float value, float min_value, float max_value);

#ifdef __cplusplus
}
#endif

#endif /* APP_COMMON_APP_TYPES_H */
