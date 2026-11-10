#ifndef IMU_FUSION_H
#define IMU_FUSION_H
#include "imu_config.h"
#include "bmi088.h"
#include "Attitude/app_attitude.h"
/* Single-writer filter, owned by IMU task. No shared EKF globals are public API. */
void imu_fusion_reset(void);
app_status_t imu_fusion_push(const bmi088_sample_t *sample,
                             app_attitude_snapshot_t *out);
uint16_t imu_fusion_calibration_count(void);
void imu_fusion_bias(float out[3]);
#endif
