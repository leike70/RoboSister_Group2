/**
  ******************************************************************************
  * @file    remote_uart.h
  * @brief   DR16/DT7 接收机串口接收（USART3 + DMA/IDLE，两车公共）。
  *
  * 本文件只负责“收到一帧原始 DBUS 字节并交给上层解析”，不做通道映射、
  * 不做在线判断、不直接驱动电机。协议解析在 `User_Driver/Remote/dbus.*`，
  * 在线/失联判断在 `App/Remote`。
  *
  * 硬件约定：USART3，PC11 RX / PC10 TX，100 kbps、8 数据位、偶校验、1 停止位；
  * 接收 DMA 为 DMA1 Stream1 Channel4，NORMAL 模式 + IDLE 事件结束一帧。
  * DBUS 信号需要板上 DBUS 接口的反相/电平链路，不能当普通 TTL UART 直连。
  ******************************************************************************
  */
#ifndef BSP_REMOTE_UART_H
#define BSP_REMOTE_UART_H

#include <stdbool.h>
#include <stdint.h>

#include "Common/app_types.h"

#ifdef __cplusplus
extern "C" {
#endif

/** 接收统计，供调试器读取；计数均为上电以来累计值。 */
typedef struct {
    uint32_t valid_frames;  /**< 有效且未过期，已发布为逻辑输入 */
    uint32_t invalid_frames;/**< 长度/字段/映射失败，已撤销输入 */
    uint32_t stale_frames;  /**< 到达时刻已超过失联门限，丢弃并撤销输入 */
    uint32_t uart_errors;   /**< HAL UART 错误回调次数 */
    uint32_t restarts;      /**< 接收重启次数（含停收看门狗） */
    uint32_t start_failures;/**< 重启/启动失败次数 */
} remote_uart_stats_t;

/**
 * @brief 启动接收。必须在 RTOS 对象和 USART3/DMA 初始化之后调用。
 * @param now_ms 当前时间，ms
 */
void remote_uart_init(uint32_t now_ms);

/**
 * @brief 周期服务：消费待处理帧（解析/映射/发布）、处理错误与停收恢复。
 *
 * 由整车控制任务每 2 ms 调用；内部不阻塞等待，不使用 HAL 超时循环。
 * @param now_ms 当前时间，ms
 */
void remote_uart_service(uint32_t now_ms);

/** 读取接收统计。 */
app_status_t remote_uart_get_stats(remote_uart_stats_t *out);

#ifdef __cplusplus
}
#endif

#endif /* BSP_REMOTE_UART_H */
