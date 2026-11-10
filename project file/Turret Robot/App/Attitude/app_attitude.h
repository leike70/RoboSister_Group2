/**
  ******************************************************************************
  * @file    app_attitude.h
  * @brief   姿态（IMU）接口（两车公共）。
  *
  * BMI088采集、温度准备与四元数EKF由独立IMU任务发布；初始化/温度准备/校准期间NOT_READY。
  * 只有进入RUNNING才发布有效姿态。IMU任务把解算结果交给 app_attitude_ingest()，本模块检查快照及20ms新鲜度。
  * 姿态单位：角度 rad，角速度 rad/s；默认板上传感器坐标，车体安装方向待标定。
  ******************************************************************************
  */
#ifndef APP_ATTITUDE_H
#define APP_ATTITUDE_H

#include <stdbool.h>
#include <stdint.h>

#include "Common/app_types.h"

#ifdef __cplusplus
extern "C" {
#endif

/** 姿态快照。valid=false 时所有角度/角速度均无意义。 */
typedef struct {
    float roll_rad;
    float pitch_rad;
    float yaw_rad;
    float roll_rate_rad_s;
    float pitch_rate_rad_s;
    float yaw_rate_rad_s;
    bool valid;
    uint32_t timestamp_ms;
} app_attitude_snapshot_t;

/** 初始化；默认无数据。 */
app_status_t app_attitude_init(uint32_t now_ms);

/** 周期刷新；数据超过20ms则清空姿态并返回OFFLINE。 */
app_status_t app_attitude_update(uint32_t now_ms);

/**
 * @brief IMU 适配层入口：提交一帧已解算姿态。
 * @note  可由 IMU 任务或普通 ISR 调用；sample 在调用内必须稳定。
 *        valid=false 发布无效快照并返回 NOT_READY；NaN/Inf 发布无效快照并
 *        返回 INVALID_PARAM。NULL 仅拒绝调用，不改变当前快照。
 *        读写通过短临界区保护；NMI/HardFault 不得调用。采集驱动在 User_Driver/IMU 与 Bsp/imu_bsp。
 */
app_status_t app_attitude_ingest(const app_attitude_snapshot_t *sample);

/** 读取一致快照。 */
app_status_t app_attitude_get_snapshot(app_attitude_snapshot_t *out);

/** 姿态模块状态：NOT_READY / OFFLINE / INVALID_PARAM / OK。 */
app_status_t app_attitude_status(void);

#ifdef __cplusplus
}
#endif

#endif /* APP_ATTITUDE_H */
