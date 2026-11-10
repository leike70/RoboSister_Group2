/**
  ******************************************************************************
  * @file    app_time.h
  * @brief   应用层时间基准（毫秒），基于 CMSIS-RTOS V2 内核 tick。
  *
  * 只在调度器启动后调用；本文件不创建任何 RTOS 对象、不阻塞。
  ******************************************************************************
  */
#ifndef APP_COMMON_APP_TIME_H
#define APP_COMMON_APP_TIME_H

#include <stdbool.h>
#include <stdint.h>

#include "Common/app_types.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief 当前时间，单位 ms，来自内核 tick 计数。
 * @note  仅用于周期判断和超时判断，不保证与真实时间严格一致。
 */
uint32_t app_time_now_ms(void);

/**
 * @brief 判断时间戳是否新鲜（用于失联/超时检查）。
 * @param now_ms      当前时间，单位 ms
 * @param stamp_ms    待检查时间戳，单位 ms；APP_TIME_NEVER 视为不新鲜
 * @param timeout_ms  允许的最大时差，单位 ms
 * @return true 表示时差未超过 timeout_ms（已处理 32 位回绕）。
 */
bool app_time_is_fresh(uint32_t now_ms, uint32_t stamp_ms, uint32_t timeout_ms);

/** 两个时间戳之差（now_ms - then_ms），单位 ms，已处理回绕。 */
uint32_t app_time_elapsed_ms(uint32_t now_ms, uint32_t then_ms);

#ifdef __cplusplus
}
#endif

#endif /* APP_COMMON_APP_TIME_H */
