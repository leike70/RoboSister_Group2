/**
  ******************************************************************************
  * @file    app_time.c
  * @brief   应用层毫秒时间基准实现。
  ******************************************************************************
  */
#include "Common/app_time.h"

#include "cmsis_os.h"

uint32_t app_time_now_ms(void)
{
    const uint32_t tick_freq = osKernelGetTickFreq();
    if (tick_freq == 0u) {
        return 0u;
    }

    /* 用 64 位中间量避免 tick 计数乘 1000 后溢出。 */
    const uint64_t ticks = (uint64_t)osKernelGetTickCount();
    return (uint32_t)((ticks * 1000u) / (uint64_t)tick_freq);
}

uint32_t app_time_elapsed_ms(uint32_t now_ms, uint32_t then_ms)
{
    /* 无符号减法天然处理 32 位回绕。 */
    return (uint32_t)(now_ms - then_ms);
}

bool app_time_is_fresh(uint32_t now_ms, uint32_t stamp_ms, uint32_t timeout_ms)
{
    if (stamp_ms == APP_TIME_NEVER) {
        return false;
    }
    return app_time_elapsed_ms(now_ms, stamp_ms) <= timeout_ms;
}
