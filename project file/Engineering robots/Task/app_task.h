/**
  ******************************************************************************
  * @file    app_task.h
  * @brief   整车应用启动与周期调度入口（C 兼容，供 freertos.c 的 USER CODE 调用）。
  *
  * 只创建**一个**整车周期控制任务；已有 CAN1/CAN2 发送任务与 DJI 服务任务保留。
  * 周期数据流：输入/反馈快照 → 健康/模式判断 → 机构目标 → 功率/保护约束 →
  * 唯一电机输出提交。
  *
  * 禁止在中断中调用本文件的周期入口；本文件不阻塞等待任何硬件。
  ******************************************************************************
  */
#ifndef APP_TASK_H
#define APP_TASK_H

#include <stdbool.h>
#include <stdint.h>

#include "cmsis_os.h"

#include "Common/app_types.h"
#include "Protection/app_protection.h"

#ifdef __cplusplus
extern "C" {
#endif

/** 整车控制周期，单位 ms。首轮取值，未做实时性验证。 */
#define APP_CONTROL_PERIOD_MS 2u

/** 应用控制任务句柄（调试用）。 */
extern osThreadId_t g_app_control_task_handle;

/** 周期任务报告，供调试器读取。 */
typedef struct {
    uint32_t cycle_count;      /**< 已完成周期数 */
    uint32_t overrun_count;    /**< 周期超时次数 */
    uint32_t last_period_ms;   /**< 上周期实际间隔，ms */
    uint32_t updated_ms;       /**< 报告刷新时刻，ms */
    app_run_state_t run_state;
    bool output_allowed;
    uint32_t active_faults;
    app_status_t remote_status;
    bool remote_aux_valid;     /**< 原始通道/开关/按键快照当前是否有效 */
    uint32_t remote_aux_ms;    /**< 原始快照对应帧的到达时刻，ms */
    bool attitude_valid;       /**< 姿态快照当前是否有效 */
    uint32_t attitude_ms;      /**< 姿态快照对应的采样时刻，ms */
    app_status_t imu_status;   /**< IMU 融合状态（IMU_Acquire 报告） */
    uint16_t imu_calibration_samples; /**< 静止校准进度，0..500 */
    uint32_t imu_samples;      /**< 已处理的有效 IMU 样本数 */
    float heater_duty;         /**< IMU 加热实际占空比，0..1 */
    bool heater_stale;         /**< App_Control 看门狗已强制关闭加热 */
    bool can_started;          /**< CAN1/CAN2 已启动 */
    uint32_t can_tx_fail;      /**< 累计发送失败次数 */
    uint32_t can_bus_off;      /**< 累计 bus-off 事件数 */
    uint32_t can_errors;       /**< 累计 CAN 错误回调次数 */
    app_status_t chassis_status;
    app_status_t lift_status;
    app_status_t attitude_status;
    app_status_t power_status;
    float power_measured_w;       /**< 0x31 实测底盘功率，W */
    float power_buffer_j;         /**< 0x31 缓冲能量，J */
    float power_chassis_scale;    /**< 底盘策略缩放，0..1 */
    bool power_source_online;     /**< 0x31 来源在线 */
    bool power_hardware_verified; /**< 0x31 已上板核验 */
    uint32_t power_error_flags;   /**< app_power_error_t 位组合 */
    uint32_t power_frame_ms;      /**< 有效帧到达时间 */
    app_status_t vehicle_status;
    app_status_t motor_status;
} app_task_report_t;

/**
 * @brief 应用控制任务主体（RTOS 线程入口）。
 *
 * 首次进入时依次初始化配置、电机注册、保护、遥控、底盘、抬升、姿态、功率和
 * 本车专有模块；随后按固定周期执行控制流水线。初始化不等待硬件。
 */
void app_control_task(void *argument);

/** 读取周期任务报告。 */
app_status_t app_task_get_report(app_task_report_t *out);

#ifdef __cplusplus
}
#endif

#endif /* APP_TASK_H */
