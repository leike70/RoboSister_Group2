#include "imu_config.h"

/*
 * Reference INS initial values: 40 C setpoint, 1 kHz TIM10 PWM, PID Kp=1000,
 * Ki=20, max output 2000, integral limit 300 (the supplied project expresses
 * these in CCR counts). This project normalizes duty to 0..1 and adds a
 * conservative cap, readiness/protection thresholds and a control period.
 * None of these thermal values has been validated on this board; tune after
 * measuring the real heater, thermal mass and steady-state duty.
 */
static const imu_config_t config = {
    .sensor_to_body = {1,0,0, 0,1,0, 0,0,1},
    .vehicle_mounting_verified = false,
    .calibration_samples = 500,
    .sample_period_ms = 2,
    .stale_timeout_ms = IMU_DATA_TIMEOUT_MS,
    .calibration_timeout_ms = 60000u,
    .max_calibration_rate_rad_s = 0.08f,
    .max_gyro_span_rad_s = 0.02f,
    .max_accel_span_mps2 = 0.30f,
    .ekf_q1 = 10.0f, .ekf_q2 = 0.001f, .ekf_r = 10000000.0f,
    .ekf_fading = 1.0f, .ekf_accel_lpf_s = 0.0f,
    .temperature = {
        /* Reference 40 C setpoint; board-specific target still unverified. */
        .target_c = 40.0f,
        /* Reference uses 100% max output; start conservatively at 80%. */
        .pwm_max_duty = 0.80f,
        /* Reference 1000/2000 and 20/2000 after duty normalization. */
        .kp = 0.50f,
        .ki = 0.01f,
        .kd = 0.0f,
        /* Reference integral limit 300/2000 = 0.15 duty. */
        .integral_limit_duty = 0.15f,
        .deadband_c = 0.10f,
        .ready_tolerance_c = 1.0f,
        .calibration_tolerance_c = 2.0f,
        .running_tolerance_c = 3.0f,
        /* New this round; confirm thermal response and tune on board. */
        .ready_hold_ms = 3000u,
        .preheat_timeout_ms = 120000u,
        .stale_timeout_ms = 300u,
        .control_period_ms = 100u,
        .overtemp_c = 50.0f,
        .min_valid_c = -20.0f,
        .max_valid_c = 60.0f,
        .runaway_margin_c = 3.0f,
        .runaway_hold_ms = 5000u
    }
};
const imu_config_t *imu_config_get(void) { return &config; }
