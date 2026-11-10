/**
  ******************************************************************************
  * @file    remote_uart.c
  * @brief   USART3 + DMA/IDLE 接收 DR16/DT7 DBUS 帧。
  *
  * 中断只做“保存一帧字节 + 到达时刻 + 重新挂接收”，解析、映射、发布都在任务
  * 上下文（remote_uart_service）完成，避免在 ISR 里查配置或做耗时处理。
  * 到达时刻使用 app_time_now_ms()，与任务侧同一时间基准；服务端用“到达时刻”
  * 而不是“处理时刻”判断新鲜度，延迟处理不会把过期帧重新判为在线。
  *
  * 恢复策略（不计入“已验证硬件行为”，需上板测量）：
  *  - UART 错误或重启失败：撤销输入，控制任务下一周期重试；
  *  - 1 s 无接收事件：串行关闭 IRQ、有限时间中止接收并重新启动。
  ******************************************************************************
  */
#include "remote_uart.h"

#include <stddef.h>
#include <string.h>

#include "Common/app_critical.h"
#include "Common/app_time.h"
#include "Remote/app_remote.h"
#include "Remote/dbus.h"
#include "app_config.h"
#include "usart.h"

#define RX_BUFFER_SIZE (DBUS_FRAME_SIZE * 2u)
#define RX_STALL_MS 1000u

static uint8_t rx_buffer[RX_BUFFER_SIZE];
static uint8_t pending_frame[DBUS_FRAME_SIZE];
static volatile bool pending_valid;
static volatile uint16_t pending_size;
static volatile uint32_t pending_arrival_ms;
static volatile bool restart_requested;
static volatile bool input_fault;
static uint32_t last_event_ms;
static remote_uart_stats_t stats;

static HAL_StatusTypeDef receive_start(void)
{
    const HAL_StatusTypeDef status =
        HAL_UARTEx_ReceiveToIdle_DMA(&huart3, rx_buffer, RX_BUFFER_SIZE);
    if (status == HAL_OK) {
        /* 只关心整帧/IDLE，不使用半传输事件。 */
        __HAL_DMA_DISABLE_IT(&hdma_usart3_rx, DMA_IT_HT);
    }
    return status;
}

static void stats_add(uint32_t *counter)
{
    const uint32_t key = app_critical_enter();
    (*counter)++;
    app_critical_exit(key);
}

static bool restart_receive(uint32_t now_ms)
{
    /* 串行化任务恢复与两路接收中断；HAL 中止调用有内部超时，不无限等待。 */
    HAL_NVIC_DisableIRQ(USART3_IRQn);
    HAL_NVIC_DisableIRQ(DMA1_Stream1_IRQn);
    (void)HAL_UART_AbortReceive(&huart3);
    __HAL_UART_CLEAR_OREFLAG(&huart3);
    const HAL_StatusTypeDef status = receive_start();
    const uint32_t key = app_critical_enter();
    stats.restarts++;
    restart_requested = status != HAL_OK;
    if (status != HAL_OK) {
        stats.start_failures++;
    }
    last_event_ms = now_ms;
    app_critical_exit(key);
    HAL_NVIC_EnableIRQ(DMA1_Stream1_IRQn);
    HAL_NVIC_EnableIRQ(USART3_IRQn);
    return status == HAL_OK;
}

void remote_uart_init(uint32_t now_ms)
{
    const uint32_t key = app_critical_enter();
    pending_valid = false;
    pending_size = 0u;
    pending_arrival_ms = APP_TIME_NEVER;
    restart_requested = true;
    input_fault = false;
    last_event_ms = now_ms;
    (void)memset(&stats, 0, sizeof(stats));
    app_critical_exit(key);
    (void)remote_uart_service(now_ms);
}

void remote_uart_service(uint32_t now_ms)
{
    uint8_t frame[DBUS_FRAME_SIZE] = {0};
    uint16_t frame_size;
    uint32_t arrival_ms;
    bool have_frame;
    bool fault;
    bool restart;
    bool stale;
    uint32_t key;

    key = app_critical_enter();
    have_frame = pending_valid;
    frame_size = pending_size;
    arrival_ms = pending_arrival_ms;
    if (have_frame) {
        if (frame_size == DBUS_FRAME_SIZE) {
            (void)memcpy(frame, pending_frame, DBUS_FRAME_SIZE);
        }
        pending_valid = false;
    }
    fault = input_fault;
    input_fault = false;
    restart = restart_requested;
    restart_requested = false;
    stale = (uint32_t)(now_ms - last_event_ms) >= RX_STALL_MS;
    if (have_frame) {
        last_event_ms = arrival_ms;
    } else if (fault) {
        last_event_ms = now_ms;
    }
    app_critical_exit(key);

    if (have_frame) {
        const app_remote_config_t *config = &app_config_get()->remote;
        app_dbus_frame_t decoded = {0};
        app_remote_input_t input = {0};
        app_status_t status = (frame_size == DBUS_FRAME_SIZE)
                                  ? dbus_decode(frame, (size_t)frame_size, arrival_ms, &decoded)
                                  : APP_STATUS_INVALID_PARAM;
        if (status == APP_STATUS_OK) {
            status = dbus_to_input(&decoded, config, &input);
        }

        /* 用到达时刻判断新鲜度：任务延迟处理不能让过期帧重新取得在线窗口。 */
        bool frame_stale = false;
        if (status == APP_STATUS_OK && config->timeout_valid &&
            config->link_timeout_ms > 0u) {
            const int32_t age_ms = (int32_t)(now_ms - arrival_ms);
            frame_stale = age_ms > (int32_t)config->link_timeout_ms;
        }

        if (status == APP_STATUS_OK && !frame_stale) {
            app_remote_aux_snapshot_t aux = {
                .valid = true,
                .timestamp_ms = arrival_ms,
                .channel = {decoded.channel[0], decoded.channel[1], decoded.channel[2],
                            decoded.channel[3]},
                .switch_s1 = decoded.switch_s1,
                .switch_s2 = decoded.switch_s2,
                .mouse_x = decoded.mouse_x,
                .mouse_y = decoded.mouse_y,
                .mouse_z = decoded.mouse_z,
                .mouse_press_left = decoded.mouse_press_left,
                .mouse_press_right = decoded.mouse_press_right,
                .keyboard = decoded.keyboard,
            };
            (void)app_remote_publish_frame(&input, &aux);
            stats_add(&stats.valid_frames);
        } else if (status == APP_STATUS_OK) {
            /* 过期帧：直接撤销输入，不发布旧状态。 */
            app_remote_invalidate(now_ms);
            stats_add(&stats.stale_frames);
        } else {
            /* 非法帧/映射未确认：立即撤销输入，避免沿用旧快照。 */
            app_remote_invalidate(now_ms);
            stats_add(&stats.invalid_frames);
        }
    }

    if (fault) {
        app_remote_invalidate(now_ms);
    }

    if (restart || stale) {
        if (!restart_receive(now_ms)) {
            app_remote_invalidate(now_ms);
        }
    }
}

app_status_t remote_uart_get_stats(remote_uart_stats_t *out)
{
    if (out == NULL) {
        return APP_STATUS_INVALID_PARAM;
    }
    const uint32_t key = app_critical_enter();
    *out = stats;
    app_critical_exit(key);
    return APP_STATUS_OK;
}

/* ------------------------------------------------------------------------- */
/* HAL 回调：只保存原始帧 + 到达时刻，并重新挂接收                            */
/* ------------------------------------------------------------------------- */

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *uart, uint16_t size)
{
    if (uart != &huart3) {
        return;
    }

    /* 与应用同基准的到达时刻；在 32 位 tick 配置下只读内核计数，ISR 可调用。 */
    const uint32_t arrival_ms = app_time_now_ms();

    uint32_t key = app_critical_enter();
    pending_size = size;
    pending_arrival_ms = arrival_ms;
    if (size == DBUS_FRAME_SIZE) {
        (void)memcpy(pending_frame, rx_buffer, DBUS_FRAME_SIZE);
    }
    pending_valid = true;
    app_critical_exit(key);

    if (receive_start() != HAL_OK) {
        key = app_critical_enter();
        restart_requested = true;
        input_fault = true;
        stats.start_failures++;
        app_critical_exit(key);
    }
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *uart)
{
    if (uart != &huart3) {
        return;
    }

    const uint32_t key = app_critical_enter();
    input_fault = true;
    restart_requested = true;
    stats.uart_errors++;
    app_critical_exit(key);
}
