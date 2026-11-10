/**
  ******************************************************************************
  * @file    dbus.h
  * @brief   DR16/DT7 接收机 DBUS 协议解析（两车公共）。
  *
  * 协议依据：`资料文件/遥控器/遥控器控制协议V1.4.pdf` 与
  * `DT7&DR16遥控系统用户手册_中文.pdf`。DBUS 为 100 kbps、8 数据位、偶校验、
  * 1 停止位、18 字节/帧（HAL 侧用 9B+EVEN 表示 8 数据位）。
  *
  * 本文件只做协议位域解析与“逻辑输入”映射，不访问 HAL、不创建 RTOS 对象。
  * 物理串口接收见 `Bsp/remote_uart.*`，在线/失联判断见 `App/Remote`。
  ******************************************************************************
  */
#ifndef USER_REMOTE_DBUS_H
#define USER_REMOTE_DBUS_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "Common/app_types.h"
#include "Remote/app_remote.h"
#include "app_config.h"

#ifdef __cplusplus
extern "C" {
#endif

#define DBUS_FRAME_SIZE 18u
#define DBUS_CHANNEL_MIN 364
#define DBUS_CHANNEL_MID 1024
#define DBUS_CHANNEL_MAX 1684
#define DBUS_SWITCH_UP 1u
#define DBUS_SWITCH_DOWN 2u
#define DBUS_SWITCH_MID 3u

/** 解析后的 DBUS 一帧。字段含义见协议 V1.4 第 3 页。 */
typedef struct {
    int16_t channel[4];        /**< ch0..ch3，364..1684，中位 1024 */
    uint8_t switch_s1;         /**< 左开关 S1：1 上 / 2 下 / 3 中 */
    uint8_t switch_s2;         /**< 右开关 S2：1 上 / 2 下 / 3 中 */
    int16_t mouse_x;           /**< 鼠标 X，-32768..32767（本车未使用） */
    int16_t mouse_y;           /**< 鼠标 Y（本车未使用） */
    int16_t mouse_z;           /**< 鼠标 Z（本车未使用） */
    uint8_t mouse_press_left;  /**< 左键 0/1（本车未使用） */
    uint8_t mouse_press_right; /**< 右键 0/1（本车未使用） */
    uint16_t keyboard;         /**< 键盘位：W/S/A/D/Q/E/Shift/Ctrl（保留未映射） */
    uint32_t sample_ms;        /**< 采样时刻，ms */
} app_dbus_frame_t;

/**
 * @brief 解析 18 字节 DBUS 帧并做字段合法性检查。
 * @param frame     原始帧，长度必须恰好为 DBUS_FRAME_SIZE
 * @param size      帧长度
 * @param sample_ms 采样时刻，ms
 * @param out       输出；失败时清零
 * @return OK / INVALID_PARAM（长度、通道、开关或鼠标按键非法）
 */
app_status_t dbus_decode(const uint8_t *frame, size_t size, uint32_t sample_ms,
                         app_dbus_frame_t *out);

/**
 * @brief 按本车配置把 DBUS 帧映射为归一化逻辑输入。
 *
 * 映射内容：四轴通道下标与方向、死区、档位开关（S1/S2 之一）、
 * 使能/复位开关（S1/S2 之一）。遥测/键盘字段当前不映射为动作。
 * @return OK；配置未确认或映射参数非法时返回 UNCONFIGURED/INVALID_PARAM。
 */
app_status_t dbus_to_input(const app_dbus_frame_t *frame,
                           const app_remote_config_t *config,
                           app_remote_input_t *out);

#ifdef __cplusplus
}
#endif

#endif /* USER_REMOTE_DBUS_H */
