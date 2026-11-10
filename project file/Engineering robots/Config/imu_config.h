#ifndef IMU_CONFIG_H
#define IMU_CONFIG_H
#include <stdbool.h>
#include <stdint.h>
#include "IMU/imu_temperature.h"
#define IMU_DATA_TIMEOUT_MS 20u
typedef struct {
    float sensor_to_body[9]; /* Proper orthonormal rotation, row-major. */
    bool vehicle_mounting_verified;
    uint16_t calibration_samples;
    uint32_t sample_period_ms, stale_timeout_ms, calibration_timeout_ms;
    float max_calibration_rate_rad_s, max_gyro_span_rad_s, max_accel_span_mps2;
    float ekf_q1, ekf_q2, ekf_r, ekf_fading, ekf_accel_lpf_s;
    imu_temperature_config_t temperature;
} imu_config_t;
const imu_config_t *imu_config_get(void);
#endif
