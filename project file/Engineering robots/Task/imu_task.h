#ifndef IMU_TASK_H
#define IMU_TASK_H

#include "cmsis_os.h"
#include "IMU/bmi088.h"
#include "IMU/imu_temperature.h"
#include "Common/app_types.h"

#ifdef __cplusplus
extern "C" {
#endif

void imu_task(void *argument);

/** Non-blocking component state; INIT/TEMP_PREP/CALIBRATING never publish a
 * control-usable attitude. */
typedef enum {
    IMU_TASK_STATE_INIT = 0,
    IMU_TASK_STATE_TEMP_PREP,
    IMU_TASK_STATE_CALIBRATING,
    IMU_TASK_STATE_RUNNING,
    IMU_TASK_STATE_FAULT
} imu_task_state_t;

/** Latching fault bits for the current attempt; cleared on a fresh INIT. */
typedef enum {
    IMU_FAULT_NONE = 0,
    IMU_FAULT_SENSOR = 1u << 0,
    IMU_FAULT_TEMP_INVALID = 1u << 1,
    IMU_FAULT_TEMP_STALE = 1u << 2,
    IMU_FAULT_OVERTEMP = 1u << 3,
    IMU_FAULT_TEMP_RUNAWAY = 1u << 4,
    IMU_FAULT_PREHEAT_TIMEOUT = 1u << 5,
    IMU_FAULT_CALIBRATION_TIMEOUT = 1u << 6,
    IMU_FAULT_FUSION = 1u << 7,
    IMU_FAULT_HEATER = 1u << 8,
    IMU_FAULT_CONFIG = 1u << 9
} imu_fault_t;

typedef struct {
    app_status_t status;       /**< fusion/component status; see state/faults too */
    imu_task_state_t state;    /**< INIT / TEMP_PREP / CALIBRATING / RUNNING / FAULT */
    uint32_t faults;           /**< imu_fault_t bit mask for this attempt */
    bmi088_result_t sensor_result;
    uint32_t samples, errors, initialization_attempts, updated_ms;
    uint16_t calibration_samples;
    bool vehicle_mounting_verified;
    float gyro_bias_rad_s[3];
    bmi088_sample_t last_raw;
    bool temperature_valid;
    float temperature_c;       /**< degC, latest accepted sensor temperature */
    float target_temperature_c;/**< degC */
    float heater_duty;         /**< actual normalized duty applied, 0..1 */
    uint32_t temperature_updated_ms;
    imu_temperature_state_t temperature_state;
    uint32_t temperature_faults; /**< imu_temperature_fault_t bit mask */
} imu_task_report_t;

app_status_t imu_task_get_report(imu_task_report_t *out);

#ifdef __cplusplus
}
#endif

#endif
