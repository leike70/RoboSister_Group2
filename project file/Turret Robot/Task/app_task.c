/**
  ******************************************************************************
  * @file    app_task.c
  * @brief   整车应用启动与周期调度（两车公共实现）。
  *
  * 启动顺序（线程首次运行时完成，不等待任何硬件）：
  *   配置校验 → 电机注册 → 保护 → 遥控 → 底盘 → 抬升 → 姿态 → 功率 → 本车模块
  *
  * 周期数据流（每 APP_CONTROL_PERIOD_MS 执行一次）：
  *   遥控/反馈快照 → 故障复位边沿 → 健康与模式判断 → 清空旧请求 →
  *   机构目标生成 → 功率与保护约束 → 唯一电机输出提交
  *
  * 本文件不阻塞等待硬件；未接入的模块只报告状态并继续维护应用健康信息。
  ******************************************************************************
  */
#include "app_task.h"

#include <stddef.h>

#include "Attitude/app_attitude.h"
#include "Chassis/app_chassis.h"
#include "Common/app_time.h"
#include "Lift/app_lift.h"
#include "Motor/app_motor.h"
#include "Power/app_power.h"
#include "Protection/app_protection.h"
#include "Remote/app_remote.h"
#include "Vehicle/app_vehicle.h"
#include "app_config.h"
#include "can_bsp_api.h"
#include "imu_task.h"
#include "imu_heater.h"
#include "remote_uart.h"

/** 周期超时判定余量，单位 ms（仅统计，不直接触发故障）。 */
#define APP_CONTROL_OVERRUN_SLACK_MS 2u
/** 连续超时次数达到该值才置故障，区分启动/单次抖动与持续异常。 */
#define APP_CONTROL_OVERRUN_STREAK_LIMIT 3u

osThreadId_t g_app_control_task_handle = NULL;

static app_task_report_t g_app_task_report;
static uint32_t g_last_cycle_ms = APP_TIME_NEVER;
static uint32_t g_overrun_streak;
static bool g_last_output_allowed = false;
static bool g_last_reset_request = false;
static app_mode_t g_last_mode = APP_MODE_UNKNOWN;
static bool g_last_chassis_power_ready = false;
static float g_last_chassis_scale = 1.0f;

static void app_modules_init(uint32_t now_ms)
{
    /* 配置必须最先校验：后续模块全部依赖配置有效性标记。
     * 保护模块会读取其它模块的状态，因此放在它们之后初始化。 */
    (void)app_config_init();
    (void)app_motor_init(now_ms);
    (void)app_remote_init(now_ms);
    (void)app_attitude_init(now_ms);
    (void)app_power_init(now_ms);
    /* 0x31 receiver is registered on the configured bus only. If the bus is
     * unknown, no observer/filter is installed and the source stays unconfigured. */
    {
        const app_power_config_t *power_cfg = &app_config_get()->power;
        if (power_cfg->source_bus_valid && power_cfg->source_bus != APP_CAN_BUS_NONE) {
            (void)can_bsp_add_std_observer(app_power_on_can_frame);
            (void)can_bsp_register_std((uint8_t)power_cfg->source_bus, APP_POWER_CAN_ID);
        }
    }
    (void)app_chassis_init(now_ms);
    (void)app_lift_init(now_ms);
    (void)app_protection_init(now_ms);
    /* CAN is started only after queues/semaphores exist and motor registration
     * has completed; the CAN send tasks check this flag before using resources. */
    if (!can_bsp_start()) {
        (void)app_protection_raise(APP_FAULT_CAN);
    }
    (void)app_vehicle_init(now_ms);
    /* 串口/DMA 已初始化，最后启动接收；之后每周期由 remote_uart_service 消费。 */
    remote_uart_init(app_time_now_ms());
}

static void app_control_step(void)
{
    const app_config_t *config = app_config_get();
    app_remote_snapshot_t remote = {0};
    app_remote_aux_snapshot_t remote_aux = {0};
    app_attitude_snapshot_t attitude = {0};
    imu_task_report_t imu = {0};
    can_bsp_stats_t can_stats = {0};
    app_power_feedback_t power_feedback = {0};
    app_power_policy_t power_policy = {0};
    bool chassis_power_ready;
    const app_protection_report_t *protection;
    app_chassis_request_t chassis_request = {0};
    app_lift_request_t lift_request = {0};
    uint32_t now_ms = app_time_now_ms();
    uint32_t period_ms;
    bool output_allowed;
    bool remote_verified;

    /* 1) 输入与反馈一致快照：先消费串口收到的 DBUS 帧，再取遥控快照。 */
    remote_uart_service(now_ms);
    (void)app_remote_update(now_ms);
    (void)app_remote_get_snapshot(&remote);
    (void)app_remote_get_aux_snapshot(&remote_aux);
    (void)app_motor_update_feedback(now_ms);
    (void)app_attitude_update(now_ms);
    /* Keep a production caller for the public snapshot getter: it is also the
     * debugger entry for reading roll/pitch/yaw and rates on the board. */
    (void)app_attitude_get_snapshot(&attitude);
    (void)imu_task_get_report(&imu);
    /* Independent watchdog: if the IMU task stops refreshing the heater
     * command, force TIM10 to 0 instead of leaving an old duty latched. */
    imu_heater_supervise(now_ms);
    (void)app_power_update(now_ms);
    (void)app_power_get_feedback(&power_feedback);
    (void)app_power_get_policy(&power_policy);
    chassis_power_ready = power_policy.source_ready && power_policy.chassis_scale > 0.0f;
    if (g_last_chassis_power_ready &&
        (!chassis_power_ready || power_policy.chassis_scale < g_last_chassis_scale)) {
        /* Power-source loss/low-energy/over-limit transition: invalidate old
         * chassis requests and queued CAN commands; recovery starts from the
         * current input, never from an old motion target. */
        app_motor_disable_all(now_ms);
        can_bsp_flush_tx();
    }
    g_last_chassis_power_ready = chassis_power_ready;
    g_last_chassis_scale = power_policy.chassis_scale;

    /* CAN errors are observed here; only registered devices may raise a device
     * fault, so placeholder slots cannot spam protection. Recovery is finite and
     * runs in task context. */
    can_bsp_service(now_ms);
    can_bsp_get_stats(&can_stats);
    if (app_motor_registered_count() > 0 &&
        (!can_stats.started || can_stats.bus_off || can_stats.tx_fail_streak >= 3u)) {
        (void)app_protection_raise(APP_FAULT_CAN);
    }

    /* 2) 故障复位只接受逻辑复位请求的上升沿，且只清锁存故障。 */
    if (remote.reset_request && !g_last_reset_request) {
        (void)app_protection_clear_latched();
    }
    g_last_reset_request = remote.reset_request;

    /* 3) 健康与模式判断：使能请求、动作模式标志，再统一评估输出许可。
     * 在线只说明帧有效；运动许可还必须要求映射已上板核验。 */
    remote_verified = config->remote.hardware_verified;
    (void)app_protection_set_arm_request(remote.online && remote.arm_request &&
                                         remote_verified);
    (void)app_protection_set_mode_active(remote.online &&
                                         remote_verified &&
                                         (remote.mode != APP_MODE_UNKNOWN) &&
                                         (remote.mode != APP_MODE_DISABLED));
    (void)app_protection_evaluate(now_ms);
    protection = app_protection_report();
    output_allowed = app_protection_output_allowed();

    /* 4) 模式切换是显式失效点：清掉旧请求和已排队的旧运动报文；
     *    失联/停用/故障在输出下降沿做同样处理。 */
    if (remote.mode != g_last_mode) {
        app_motor_disable_all(now_ms);
        can_bsp_flush_tx();
        g_last_mode = remote.mode;
    }
    app_motor_clear_requests();

    /* 5) 机构目标生成。模式映射只在本周期允许输出时进行。 */
    if (output_allowed) {
        if (remote.mode == APP_MODE_CHASSIS) {
            chassis_request.forward_mps = remote.forward * config->chassis.max_linear_speed_mps;
            chassis_request.turn_rad_s = remote.turn * config->chassis.max_turn_rate_rad_s;
            chassis_request.valid = true;
        }
        /* 抬升目标映射（遥控通道/自动越障）尚未确认，本周期不生成目标；
         * 待通道确认后在此填充 lift_request。 */
    }

    /* 6) 各机构把目标转成统一电机请求（只有配置齐全时才会真正写入）。 */
    g_app_task_report.chassis_status = app_chassis_update(&chassis_request, now_ms);
    g_app_task_report.lift_status = app_lift_update(&lift_request, now_ms);
    g_app_task_report.vehicle_status = app_vehicle_update(&remote, output_allowed, now_ms);

    /* 7) 唯一输出提交入口，每周期只调用一次。 */
    g_app_task_report.motor_status = app_motor_submit(now_ms);

    /* 8) 输出许可下降沿：使旧命令失效，并让受重力机构进入停止接口。
     * 同时丢弃 CAN 队列中失效前排队的旧运动报文；停止/保护路径不再补发旧动作。 */
    if (g_last_output_allowed && !output_allowed) {
        app_motor_disable_all(now_ms);
        can_bsp_flush_tx();
        (void)app_lift_stop(now_ms);
        (void)app_vehicle_stop(now_ms);
    }
    g_last_output_allowed = output_allowed;

    /* 9) 周期统计与报告。 */
    period_ms = app_time_elapsed_ms(now_ms, g_last_cycle_ms);
    if ((g_last_cycle_ms != APP_TIME_NEVER) &&
        (period_ms > (APP_CONTROL_PERIOD_MS + APP_CONTROL_OVERRUN_SLACK_MS))) {
        g_app_task_report.overrun_count++;
        if (g_overrun_streak < UINT32_MAX) { g_overrun_streak++; }
        if (g_overrun_streak >= APP_CONTROL_OVERRUN_STREAK_LIMIT) {
            /* Sustained overrun: revoke output immediately and latch the fault. */
            app_motor_disable_all(now_ms);
            can_bsp_flush_tx();
            (void)app_protection_raise(APP_FAULT_CYCLE_OVERRUN);
            g_overrun_streak = 0u;
        }
    } else {
        g_overrun_streak = 0u;
    }
    g_last_cycle_ms = now_ms;

    g_app_task_report.cycle_count++;
    g_app_task_report.last_period_ms = period_ms;
    g_app_task_report.updated_ms = now_ms;
    g_app_task_report.run_state = protection->state;
    g_app_task_report.output_allowed = output_allowed;
    g_app_task_report.active_faults = protection->active_faults;
    g_app_task_report.remote_status = app_remote_status();
    g_app_task_report.remote_aux_valid = remote_aux.valid;
    g_app_task_report.remote_aux_ms = remote_aux.timestamp_ms;
    g_app_task_report.attitude_status = app_attitude_status();
    g_app_task_report.attitude_valid = attitude.valid;
    g_app_task_report.attitude_ms = attitude.timestamp_ms;
    g_app_task_report.imu_status = imu.status;
    g_app_task_report.imu_calibration_samples = imu.calibration_samples;
    g_app_task_report.imu_samples = imu.samples;
    g_app_task_report.heater_duty = imu_heater_get_duty();
    g_app_task_report.heater_stale = imu_heater_is_stale();
    g_app_task_report.power_status = app_power_status();
    g_app_task_report.power_measured_w = power_feedback.measured_chassis_power_w;
    g_app_task_report.power_buffer_j = power_feedback.buffer_energy_j;
    g_app_task_report.power_chassis_scale = power_policy.chassis_scale;
    g_app_task_report.power_source_online = power_feedback.source_online;
    g_app_task_report.power_hardware_verified = power_feedback.hardware_verified;
    g_app_task_report.power_error_flags = power_feedback.error_flags;
    g_app_task_report.power_frame_ms = power_feedback.frame_arrival_ms;
    g_app_task_report.can_started = can_stats.started;
    g_app_task_report.can_tx_fail = can_stats.tx_fail;
    g_app_task_report.can_bus_off = can_stats.bus_off;
    g_app_task_report.can_errors = can_stats.errors;
}

void app_control_task(void *argument)
{
    uint32_t period_ticks;
    uint32_t next_tick;

    (void)argument;

    /* 应用启动入口：RTOS 对象已由 freertos.c 创建完毕，这里只做只读初始化。 */
    app_modules_init(app_time_now_ms());
    /* Keep the debug report getter linked even without a functional caller. */
    app_task_report_t boot_report;
    (void)app_task_get_report(&boot_report);

    period_ticks = (osKernelGetTickFreq() * APP_CONTROL_PERIOD_MS) / 1000u;
    if (period_ticks == 0u) {
        period_ticks = 1u;
    }
    next_tick = osKernelGetTickCount();

    for (;;) {
        app_control_step();
        next_tick += period_ticks;
        osDelayUntil(next_tick);
    }
}

app_status_t app_task_get_report(app_task_report_t *out)
{
    if (out == NULL) {
        return APP_STATUS_INVALID_PARAM;
    }
    *out = g_app_task_report;
    return APP_STATUS_OK;
}
