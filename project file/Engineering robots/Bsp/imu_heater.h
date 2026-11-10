#ifndef IMU_HEATER_H
#define IMU_HEATER_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/*
 * BMI088 heater output on PF6 / TIM10_CH1.
 *
 * Reference C board: PF6 -> TIM10_CH1 (AF3), PWM mode 1, active high,
 * TIM10 on APB2 timer clock 168 MHz, PSC=84-1, ARR=2000-1 -> 1 kHz.
 * Duty is normalized to [0, 1]; the temperature controller applies its own
 * lower cap. The output defaults to 0 and only starts after init succeeds.
 *
 * Watchdog: every non-zero duty records a timestamp. App_Control calls
 * imu_heater_supervise() every cycle; if the IMU task stops refreshing the
 * command for longer than stale_timeout_ms, the channel is forced to 0.
 */
bool imu_heater_init(uint32_t stale_timeout_ms);
void imu_heater_set_duty(float duty, uint32_t now_ms);
void imu_heater_stop(void);
void imu_heater_supervise(uint32_t now_ms);
float imu_heater_get_duty(void);
bool imu_heater_is_stale(void);
bool imu_heater_is_ready(void);

#ifdef __cplusplus
}
#endif

#endif /* IMU_HEATER_H */
