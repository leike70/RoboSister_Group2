/**
  ******************************************************************************
  * @file    app_shooting.h
  * @brief   炮台车摩擦轮与拨弹接口（含卡弹状态）。
  *
  * 摩擦轮为两只 3508（转速闭环），拨弹为 2006。目标转速、拨弹定量方式、
  * 防卡弹阈值与恢复策略待确定；首轮不自动发射、不填经验值。
  * 发射请求必须经使能条件与设备健康判断后才能生效（由保护模块门控）。
  ******************************************************************************
  */
#ifndef APP_SHOOTING_H
#define APP_SHOOTING_H

#include <stdbool.h>
#include <stdint.h>

#include "Common/app_types.h"

#ifdef __cplusplus
extern "C" {
#endif

/** 卡弹状态。首轮没有卡弹检测实现，保持 UNKNOWN。 */
typedef enum {
    APP_SHOOT_JAM_UNKNOWN = 0,
    APP_SHOOT_JAM_CLEAR,
    APP_SHOOT_JAM_SUSPECTED,
    APP_SHOOT_JAM_CONFIRMED
} app_shoot_jam_state_t;

/** 发射请求（由本车接入层按模式生成）。 */
typedef struct {
    float friction_left_rad_s;  /**< 左摩擦轮目标角速度，rad/s */
    float friction_right_rad_s; /**< 右摩擦轮目标角速度，rad/s */
    bool friction_valid;        /**< 摩擦轮目标是否有效 */
    bool fire;                  /**< 是否请求拨弹 */
    bool jam_reset;             /**< 是否请求卡弹复位 */
    bool valid;                 /**< 整个请求是否有效 */
} app_shooting_request_t;

/** 发射状态快照。 */
typedef struct {
    float friction_left_rad_s;   /**< 左摩擦轮目标，rad/s；friction_valid=false 时无意义 */
    float friction_right_rad_s;
    bool friction_valid;
    bool firing;                /**< 当前是否请求拨弹 */
    app_shoot_jam_state_t jam;
    app_status_t status;
    uint32_t updated_ms;
} app_shooting_state_t;

/** 初始化；不创建电机对象、不阻塞。 */
app_status_t app_shooting_init(uint32_t now_ms);

/**
 * @brief 周期更新：把摩擦轮/拨弹请求转成统一电机请求。
 * @param request 本周期请求；NULL 或 valid=false 时清空目标
 */
app_status_t app_shooting_update(const app_shooting_request_t *request,
                                 uint32_t now_ms);

/** 读取状态快照（摩擦轮目标/卡弹状态）。 */
app_status_t app_shooting_get_state(app_shooting_state_t *out);

/** 停用：清空摩擦轮与拨弹目标。 */
app_status_t app_shooting_stop(uint32_t now_ms);

#ifdef __cplusplus
}
#endif

#endif /* APP_SHOOTING_H */
