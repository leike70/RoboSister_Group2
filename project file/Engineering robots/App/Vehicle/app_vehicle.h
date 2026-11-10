/**
  ******************************************************************************
  * @file    app_vehicle.h
  * @brief   本车专有模块的统一接入层（接口两车一致，实现按车不同）。
  *
  * 周期任务只依赖本接口；工程车实现在 App/Vehicle/app_vehicle.c 中调用机械臂，
  * 炮台车调用云台与发射。两车实现差异集中在 App/Vehicle/ 与 Config/app_config.c。
  ******************************************************************************
  */
#ifndef APP_VEHICLE_H
#define APP_VEHICLE_H

#include <stdbool.h>
#include <stdint.h>

#include "Common/app_types.h"
#include "Remote/app_remote.h"

#ifdef __cplusplus
extern "C" {
#endif

/** 本车专有模块状态汇总。 */
typedef struct {
    app_status_t status;           /**< 本车专有功能整体状态 */
    app_status_t primary_status;   /**< 工程车=机械臂 / 炮台车=云台 */
    app_status_t secondary_status; /**< 工程车=机械臂任务 / 炮台车=发射 */
    app_mode_t mode;               /**< 本周期使用的模式 */
    uint32_t updated_ms;
} app_vehicle_report_t;

/** 初始化本车专有模块；不创建硬件对象、不阻塞。 */
app_status_t app_vehicle_init(uint32_t now_ms);

/**
 * @brief 周期更新本车专有模块。
 * @param remote         只读遥控快照
 * @param output_allowed 保护模块给出的输出许可；false 时必须清空目标
 * @return 汇总状态
 */
app_status_t app_vehicle_update(const app_remote_snapshot_t *remote,
                                bool output_allowed,
                                uint32_t now_ms);

/* Each car implements these separately in its own app_vehicle.c. */
app_status_t app_vehicle_on_up(const app_remote_snapshot_t *input, uint32_t now_ms);
app_status_t app_vehicle_on_down(const app_remote_snapshot_t *input, uint32_t now_ms);

/** 停用：清空本车专有目标，并登记机械自锁/制动待办。 */
app_status_t app_vehicle_stop(uint32_t now_ms);

/** 读取状态快照。 */
app_status_t app_vehicle_get_report(app_vehicle_report_t *out);

#ifdef __cplusplus
}
#endif

#endif /* APP_VEHICLE_H */
