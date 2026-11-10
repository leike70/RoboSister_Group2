/**
  ******************************************************************************
  * @file    app_attitude.c
  * @brief   姿态（IMU）快照接口（两车公共）。
  *
  * BMI088 采集与四元数 EKF 在独立 IMU_Acquire 任务中运行，由 imu_task 调用
  * app_attitude_ingest() 发布；本模块只做快照、数值校验和 20 ms 新鲜度检查。
  * 默认状态 NOT_READY，数据超时后置 OFFLINE 并清零角度/角速度。
  *
  * 本文件不创建 RTOS 对象、不阻塞；单位：角度 rad、角速度 rad/s；
  * 默认板上传感器坐标，车体安装方向由 imu_config 的安装矩阵标定。
  ******************************************************************************
  */
#include "Attitude/app_attitude.h"

#include <math.h>
#include <string.h>

#include "Common/app_critical.h"
#include "imu_config.h"

static app_attitude_snapshot_t g_snapshot;
static app_status_t g_status = APP_STATUS_NOT_READY;

/* ------------------------------------------------------------------------- */
/* 内部工具                                                                   */
/* ------------------------------------------------------------------------- */

/** 六个姿态量必须都是有限值；NaN/Inf 视为无效样本。 */
static bool attitude_is_finite(const app_attitude_snapshot_t *sample)
{
    return isfinite(sample->roll_rad) && isfinite(sample->pitch_rad) &&
           isfinite(sample->yaw_rad) && isfinite(sample->roll_rate_rad_s) &&
           isfinite(sample->pitch_rate_rad_s) && isfinite(sample->yaw_rate_rad_s);
}

/* ------------------------------------------------------------------------- */
/* 公开接口                                                                   */
/* ------------------------------------------------------------------------- */

app_status_t app_attitude_init(uint32_t now_ms)
{
    const uint32_t key = app_critical_enter();
    (void)memset(&g_snapshot, 0, sizeof(g_snapshot));
    g_snapshot.valid = false;
    g_snapshot.timestamp_ms = now_ms;
    g_status = APP_STATUS_NOT_READY;
    app_critical_exit(key);
    return APP_STATUS_NOT_READY;
}

app_status_t app_attitude_update(uint32_t now_ms)
{
    const uint32_t key = app_critical_enter();
    const int32_t age_ms = (int32_t)(now_ms - g_snapshot.timestamp_ms);
    if (g_status == APP_STATUS_OK && age_ms > (int32_t)IMU_DATA_TIMEOUT_MS) {
        const uint32_t stamp = g_snapshot.timestamp_ms;
        g_snapshot = (app_attitude_snapshot_t){0};
        g_snapshot.timestamp_ms = stamp;
        g_status = APP_STATUS_OFFLINE;
    }
    const app_status_t status = g_status;
    app_critical_exit(key);
    return status;
}

app_status_t app_attitude_ingest(const app_attitude_snapshot_t *sample)
{
    app_attitude_snapshot_t next;
    app_status_t status;
    if (sample == NULL) {
        return APP_STATUS_INVALID_PARAM;
    }
    /* 调用者保证传入样本在本次调用内稳定。计算在临界区外，整帧发布在区内。 */
    next = *sample;
    if (!next.valid) {
        status = APP_STATUS_NOT_READY;
    } else if (!attitude_is_finite(&next)) {
        status = APP_STATUS_INVALID_PARAM;
    } else {
        status = APP_STATUS_OK;
    }
    if (status != APP_STATUS_OK) {
        const uint32_t sample_ms = next.timestamp_ms;
        next = (app_attitude_snapshot_t){0};
        next.timestamp_ms = sample_ms;
    }

    /* 时间戳由适配层填写（app_time_now_ms()）；本函数不臆造采样时刻。 */
    const uint32_t key = app_critical_enter();
    g_snapshot = next;
    g_status = status;
    app_critical_exit(key);
    return status;
}

app_status_t app_attitude_get_snapshot(app_attitude_snapshot_t *out)
{
    if (out == NULL) {
        return APP_STATUS_INVALID_PARAM;
    }
    const uint32_t key = app_critical_enter();
    *out = g_snapshot;
    const app_status_t status = g_status;
    app_critical_exit(key);
    return status;
}

app_status_t app_attitude_status(void)
{
    const uint32_t key = app_critical_enter();
    const app_status_t status = g_status;
    app_critical_exit(key);
    return status;
}
