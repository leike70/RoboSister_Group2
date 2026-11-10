/**
  ******************************************************************************
  * @file    app_unit.c
  * @brief   机构单位与驱动单位换算实现。
  *
  * 换算公式（与 Config 中 direction / gear_ratio / zero_offset_rad 的定义一致）：
  *   机构角度 rad  = zero_offset_rad + direction * (2π * 转子圈数 / gear_ratio)
  *   转子圈数      = direction * gear_ratio * (机构角度 rad - zero_offset_rad) / (2π)
  *   机构角速度 rad/s = direction * (2π * 转子 rpm / 60) / gear_ratio
  *   转子 rpm      = direction * gear_ratio * 机构角速度 rad/s * 60 / (2π)
  *
  * 任一相关配置缺少有效标记时返回 UNCONFIGURED，绝不输出猜测值。
  ******************************************************************************
  */
#include "Motor/app_unit.h"

#include <math.h>
#include <stddef.h>

#define APP_PI_F 3.14159265358979f

static bool config_direction_gear_ready(const app_motor_config_t *config)
{
    return (config != NULL) && config->direction_valid && config->gear_ratio_valid &&
           (config->direction == 1 || config->direction == -1) &&
           (config->gear_ratio > 0.0f);
}

static float config_direction(const app_motor_config_t *config)
{
    return (config->direction < 0) ? -1.0f : 1.0f;
}

app_status_t app_unit_rad_to_rotor_rounds(const app_motor_config_t *config,
                                          float mechanism_rad,
                                          float *rotor_rounds)
{
    if ((config == NULL) || (rotor_rounds == NULL)) {
        return APP_STATUS_INVALID_PARAM;
    }
    if (!config_direction_gear_ready(config) || !config->zero_offset_valid) {
        return APP_STATUS_UNCONFIGURED;
    }
    if (!isfinite(mechanism_rad)) {
        return APP_STATUS_INVALID_PARAM;
    }

    const float rotor = config_direction(config) * config->gear_ratio *
                        (mechanism_rad - config->zero_offset_rad) / (2.0f * APP_PI_F);
    *rotor_rounds = rotor;
    return APP_STATUS_OK;
}

app_status_t app_unit_rad_s_to_rotor_rpm(const app_motor_config_t *config,
                                         float mechanism_rad_s,
                                         float *rotor_rpm)
{
    if ((config == NULL) || (rotor_rpm == NULL)) {
        return APP_STATUS_INVALID_PARAM;
    }
    if (!config_direction_gear_ready(config)) {
        return APP_STATUS_UNCONFIGURED;
    }
    if (!isfinite(mechanism_rad_s)) {
        return APP_STATUS_INVALID_PARAM;
    }

    const float rpm = config_direction(config) * config->gear_ratio * mechanism_rad_s *
                      60.0f / (2.0f * APP_PI_F);
    *rotor_rpm = rpm;
    return APP_STATUS_OK;
}

app_status_t app_unit_rotor_rounds_to_rad(const app_motor_config_t *config,
                                          float rotor_rounds,
                                          float *mechanism_rad)
{
    if ((config == NULL) || (mechanism_rad == NULL)) {
        return APP_STATUS_INVALID_PARAM;
    }
    if (!config_direction_gear_ready(config) || !config->zero_offset_valid) {
        return APP_STATUS_UNCONFIGURED;
    }
    if (!isfinite(rotor_rounds)) {
        return APP_STATUS_INVALID_PARAM;
    }

    const float rad = config->zero_offset_rad +
                      config_direction(config) * (2.0f * APP_PI_F * rotor_rounds) /
                          config->gear_ratio;
    *mechanism_rad = rad;
    return APP_STATUS_OK;
}

app_status_t app_unit_rotor_rpm_to_rad_s(const app_motor_config_t *config,
                                         float rotor_rpm,
                                         float *mechanism_rad_s)
{
    if ((config == NULL) || (mechanism_rad_s == NULL)) {
        return APP_STATUS_INVALID_PARAM;
    }
    if (!config_direction_gear_ready(config)) {
        return APP_STATUS_UNCONFIGURED;
    }
    if (!isfinite(rotor_rpm)) {
        return APP_STATUS_INVALID_PARAM;
    }

    const float rad_s = config_direction(config) * (2.0f * APP_PI_F * rotor_rpm) / 60.0f /
                        config->gear_ratio;
    *mechanism_rad_s = rad_s;
    return APP_STATUS_OK;
}

app_status_t app_unit_limit_rad(const app_motor_config_t *config,
                                float mechanism_rad,
                                float *limited_rad)
{
    if ((config == NULL) || (limited_rad == NULL)) {
        return APP_STATUS_INVALID_PARAM;
    }
    if (!config->limit_valid || (config->limit_max_rad <= config->limit_min_rad)) {
        return APP_STATUS_UNCONFIGURED;
    }
    if (!isfinite(mechanism_rad)) {
        return APP_STATUS_INVALID_PARAM;
    }

    *limited_rad = app_clampf(mechanism_rad, config->limit_min_rad, config->limit_max_rad);
    return APP_STATUS_OK;
}
