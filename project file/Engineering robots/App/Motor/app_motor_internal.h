/**
  ******************************************************************************
  * @file    app_motor_internal.h
  * @brief   Motor 模块内部共享状态（仅 App/Motor 内部使用，不对外暴露）。
  ******************************************************************************
  */
#ifndef APP_MOTOR_INTERNAL_H
#define APP_MOTOR_INTERNAL_H

#include "Motor/app_motor.h"

#ifdef __cplusplus
extern "C" {
#endif

/** 单台电机内部运行状态。 */
typedef struct {
    app_status_t status;
    bool registered;
    bool comms_usable;
    bool motion_usable;
    uint32_t missing_flags;
    uint32_t last_feedback_ms;
    app_motor_feedback_t feedback;
} app_motor_slot_t;

/** 由 app_motor_registry.c 定义、app_motor_output.c 只读。 */
extern app_motor_slot_t g_app_motor_slots[APP_MOTOR_COUNT];
extern int g_app_motor_registered_count;
extern app_status_t g_app_motor_status;

#ifdef __cplusplus
}
#endif

#endif /* APP_MOTOR_INTERNAL_H */
