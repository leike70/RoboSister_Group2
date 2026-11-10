#ifndef APP_COMMON_APP_CRITICAL_H
#define APP_COMMON_APP_CRITICAL_H
#include <stdint.h>
#ifdef __cplusplus
extern "C" {
#endif
/* 短临界区：保存/恢复 PRIMASK，可嵌套，任务和普通 ISR 可调用。
 * 不阻塞；区间内只保护少量数据，禁止等待或 I/O。NMI/HardFault 不使用此接口。 */
uint32_t app_critical_enter(void);
void app_critical_exit(uint32_t saved_mask);
#ifdef __cplusplus
}
#endif
#endif
