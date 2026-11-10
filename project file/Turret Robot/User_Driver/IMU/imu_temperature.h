#ifndef IMU_TEMPERATURE_H
#define IMU_TEMPERATURE_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/*
 * BMI088 heater temperature controller.
 *
 * The reference INS uses a 40 C setpoint and a 1 kHz TIM10 PWM channel; those
 * values are only the starting point. Protection thresholds, the duty cap and
 * the state machine below are this project's additions and require board
 * validation before they can be called vehicle-proven.
 *
 * Units: temperature in degC, time in ms, duty normalized to [0, 1].
 * This module is pure control logic: no HAL, RTOS, GPIO or timer access.
 */

typedef struct {
    float target_c;                 /* temperature setpoint, degC */
    float pwm_max_duty;             /* hard duty cap, 0..1 */
    float kp;                       /* duty per degC */
    float ki;                       /* duty per (degC * s) */
    float kd;                       /* duty per (degC / s) */
    float integral_limit_duty;      /* anti-windup integral cap, duty units */
    float deadband_c;               /* no integral update inside this band */
    float ready_tolerance_c;        /* "near target" band for warm-up ready */
    float calibration_tolerance_c;  /* max deviation allowed while calibrating */
    float running_tolerance_c;      /* max deviation allowed while running */
    uint32_t ready_hold_ms;         /* time inside band before ready */
    uint32_t preheat_timeout_ms;    /* max time from first valid temp to ready */
    uint32_t stale_timeout_ms;      /* temp sample age that forces heat off */
    uint32_t control_period_ms;     /* PID update period */
    float overtemp_c;               /* immediate cut-off threshold */
    float min_valid_c;              /* lower plausible sensor range */
    float max_valid_c;              /* upper plausible sensor range */
    float runaway_margin_c;         /* above target + margin starts runaway timer */
    uint32_t runaway_hold_ms;       /* sustained overshoot before runaway fault */
} imu_temperature_config_t;

typedef enum {
    IMU_TEMP_STATE_OFF = 0,
    IMU_TEMP_STATE_WAIT_VALID,
    IMU_TEMP_STATE_HEATING,
    IMU_TEMP_STATE_READY,
    IMU_TEMP_STATE_FAULT
} imu_temperature_state_t;

typedef enum {
    IMU_TEMP_FAULT_NONE = 0,
    IMU_TEMP_FAULT_CONFIG = 1u << 0,
    IMU_TEMP_FAULT_SENSOR = 1u << 1,
    IMU_TEMP_FAULT_STALE = 1u << 2,
    IMU_TEMP_FAULT_RANGE = 1u << 3,
    IMU_TEMP_FAULT_OVERTEMP = 1u << 4,
    IMU_TEMP_FAULT_RUNAWAY = 1u << 5,
    IMU_TEMP_FAULT_PREHEAT_TIMEOUT = 1u << 6
} imu_temperature_fault_t;

typedef struct {
    imu_temperature_state_t state;
    uint32_t faults;                /* imu_temperature_fault_t bit mask */
    bool valid;                     /* last sample accepted as a temperature */
    bool ready;                     /* warm-up/steady-state criterion met */
    bool heater_allowed;            /* duty may be applied to the heater */
    float temperature_c;
    float target_c;
    float duty;                     /* commanded normalized duty, 0..max */
    float integral_duty;            /* exposed for diagnostics/tests */
    uint32_t updated_ms;
} imu_temperature_report_t;

typedef struct {
    imu_temperature_config_t config;
    imu_temperature_state_t state;
    uint32_t faults;
    float integral;
    float last_temperature_c;
    float last_duty;
    float last_error;
    uint32_t last_valid_ms;
    uint32_t last_control_ms;
    uint32_t preheat_start_ms;
    uint32_t ready_since_ms;
    uint32_t runaway_since_ms;
    bool have_valid;
    bool have_control;
    bool have_error;
    bool ready;
    bool ready_timer_active;
    bool runaway_active;
    bool ever_ready;
    bool config_ok;
    bool initialized;
} imu_temperature_t;

/** Initialize with a copied configuration. Invalid config enters FAULT. */
void imu_temperature_init(imu_temperature_t *ctrl,
                          const imu_temperature_config_t *config);

/** Clear faults, integrator, readiness and stored temperature. Duty becomes 0. */
void imu_temperature_reset(imu_temperature_t *ctrl);

/**
 * Feed one temperature sample.
 * valid=false forces duty 0; an aged last sample becomes STALE after timeout.
 * Returns a snapshot; the caller is responsible for applying report.duty only
 * when report.heater_allowed is true.
 */
imu_temperature_report_t imu_temperature_update(imu_temperature_t *ctrl,
                                                float temperature_c,
                                                bool valid,
                                                uint32_t now_ms);

/** Check sample age without feeding a new sample. Returns current snapshot. */
imu_temperature_report_t imu_temperature_supervise(imu_temperature_t *ctrl,
                                                   uint32_t now_ms);

/** Latch faults and immediately clear duty/integrator/readiness. */
void imu_temperature_abort(imu_temperature_t *ctrl, uint32_t faults,
                           uint32_t now_ms);

#ifdef __cplusplus
}
#endif

#endif /* IMU_TEMPERATURE_H */
