/**
  ******************************************************************************
  * @file    app_unit.h
  * @brief   应用单位（机构侧 rad / rad/s）与现有驱动单位（转子圈数 / 转子 rpm /
  *          归一化量）之间的换算接口。
  *
  * 换算只在本适配层完成，方向、减速比、零位全部来自本车配置；任一缺少有效
  * 标记时返回 APP_STATUS_UNCONFIGURED，不产生“看似有效”的数值。
  ******************************************************************************
  */
#ifndef APP_MOTOR_APP_UNIT_H
#define APP_MOTOR_APP_UNIT_H

#include <stdbool.h>

#include "Common/app_types.h"
#include "app_config.h"

#ifdef __cplusplus
extern "C" {
#endif

/** 机构角度 rad → 驱动侧转子圈数（已应用方向与零位）。 */
app_status_t app_unit_rad_to_rotor_rounds(const app_motor_config_t *config,
                                          float mechanism_rad,
                                          float *rotor_rounds);

/** 机构角速度 rad/s → 驱动侧转子 rpm（已应用方向）。 */
app_status_t app_unit_rad_s_to_rotor_rpm(const app_motor_config_t *config,
                                         float mechanism_rad_s,
                                         float *rotor_rpm);

/** 驱动侧转子圈数 → 机构角度 rad（已应用方向与零位）。 */
app_status_t app_unit_rotor_rounds_to_rad(const app_motor_config_t *config,
                                          float rotor_rounds,
                                          float *mechanism_rad);

/** 驱动侧转子 rpm → 机构角速度 rad/s（已应用方向）。 */
app_status_t app_unit_rotor_rpm_to_rad_s(const app_motor_config_t *config,
                                         float rotor_rpm,
                                         float *mechanism_rad_s);

/** 把机构角度限幅到配置限位；限位无效时返回 INVALID_PARAM，不做猜测。 */
app_status_t app_unit_limit_rad(const app_motor_config_t *config,
                                float mechanism_rad,
                                float *limited_rad);

#ifdef __cplusplus
}
#endif

#endif /* APP_MOTOR_APP_UNIT_H */
