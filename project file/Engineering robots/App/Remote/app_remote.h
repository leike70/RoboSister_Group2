#ifndef APP_REMOTE_H
#define APP_REMOTE_H
#include <stdbool.h>
#include <stdint.h>
#include "Common/app_types.h"
#ifdef __cplusplus
extern "C" {
#endif

/* 逻辑输入接口。物理接收链路已接入：USART3 + DMA/IDLE -> DBUS 解析
 * （User_Driver/Remote）-> app_remote_publish_frame。物理开关/通道映射在 Config 中配置，
 * 未确认时映射函数返回 UNCONFIGURED。 */
typedef enum {
    APP_MODE_UNKNOWN = 0,
    APP_MODE_DISABLED,
    APP_MODE_CHASSIS,      /* middle: shared chassis */
    APP_MODE_VEHICLE_UP,   /* up: implemented inside this vehicle */
    APP_MODE_VEHICLE_DOWN  /* down: implemented inside this vehicle */
} app_mode_t;

/** 接收适配层提交的归一化逻辑输入；sample_ms 为该帧的到达时刻（本地 ms 基准）。 */
typedef struct {
    float forward, turn, aux1, aux2;
    bool arm_request, reset_request;
    app_mode_t mode;
    uint32_t sample_ms;
} app_remote_input_t;

/**
 * @brief 原始/协议基础状态一致快照：四通道、S1/S2、鼠标、键盘。
 *
 * 供各车上下档入口自行读取，未绑定任何机构动作。valid=false 时所有字段已清零，
 * 时间戳为 APP_TIME_NEVER，表示当前没有可用帧（失联/非法帧后不残留按下状态）。
 */
typedef struct {
    bool valid;              /**< 来自一帧有效且未过期的 DBUS 帧 */
    uint32_t timestamp_ms;   /**< 该帧到达时刻，与应用同一时间基准 */
    int16_t channel[4];      /**< ch0..ch3 原始值 364..1684 */
    uint8_t switch_s1;       /**< 左开关 S1：1 上 / 2 下 / 3 中 */
    uint8_t switch_s2;       /**< 右开关 S2：1 上 / 2 下 / 3 中 */
    int16_t mouse_x;
    int16_t mouse_y;
    int16_t mouse_z;
    uint8_t mouse_press_left;
    uint8_t mouse_press_right;
    uint16_t keyboard;       /**< 键盘位（W/S/A/D/Q/E/Shift/Ctrl） */
} app_remote_aux_snapshot_t;

typedef struct {
    float forward, turn, aux1, aux2;
    bool arm_request, reset_request;
    app_mode_t mode;
    bool online;
    uint32_t sample_ms, timestamp_ms;
} app_remote_snapshot_t;

app_status_t app_remote_init(uint32_t now_ms);
app_status_t app_remote_update(uint32_t now_ms);
/* Caller keeps input stable during this call. Invalid data revokes the input.
 * Publication/read is protected for normal task/ISR use, not NMI/HardFault. */
app_status_t app_remote_publish(const app_remote_input_t *input);
/* 同一帧的输入与原始字段原子发布；aux 为 NULL 表示该来源没有原始字段。 */
app_status_t app_remote_publish_frame(const app_remote_input_t *input,
                                      const app_remote_aux_snapshot_t *aux);
/* 读取原始/协议基础状态快照；返回模块状态，字段有效性看 out->valid。 */
app_status_t app_remote_get_aux_snapshot(app_remote_aux_snapshot_t *out);
void app_remote_invalidate(uint32_t now_ms);
app_status_t app_remote_get_snapshot(app_remote_snapshot_t *out);
app_status_t app_remote_status(void);
#ifdef __cplusplus
}
#endif
#endif
