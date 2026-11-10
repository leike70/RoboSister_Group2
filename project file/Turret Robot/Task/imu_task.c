/* Independent BMI088 acquisition, temperature preparation, static calibration
 * and EKF. All waits yield here; App_Control/remote/CAN are never blocked. */
#include "imu_task.h"

#include <math.h>
#include <string.h>

#include "Common/app_critical.h"
#include "Common/app_time.h"
#include "Attitude/app_attitude.h"
#include "IMU/imu_fusion.h"
#include "imu_bsp.h"
#include "imu_config.h"
#include "imu_heater.h"

static imu_task_report_t report = {.status = APP_STATUS_NOT_READY,
                                   .state = IMU_TASK_STATE_INIT};
static imu_temperature_t temperature;
static imu_temperature_report_t temperature_report;
static imu_task_state_t state = IMU_TASK_STATE_INIT;
static uint32_t task_faults;
static uint32_t last_sample_ms;
static uint32_t calibration_start_ms;
static bmi088_result_t last_sensor_result = BMI088_OK;

static void invalidate(uint32_t now)
{
    const app_attitude_snapshot_t invalid = {.timestamp_ms = now, .valid = false};
    (void)app_attitude_ingest(&invalid);
}

static void record(bmi088_result_t sensor, app_status_t status,
                   const bmi088_sample_t *sample, bool initializing)
{
    float bias[3];
    imu_fusion_bias(bias);
    const uint32_t key = app_critical_enter();
    report.status = status;
    report.state = state;
    report.faults = task_faults;
    report.sensor_result = sensor;
    report.updated_ms = app_time_now_ms();
    report.calibration_samples = imu_fusion_calibration_count();
    report.vehicle_mounting_verified = imu_config_get()->vehicle_mounting_verified;
    memcpy(report.gyro_bias_rad_s, bias, sizeof(bias));
    report.temperature_valid = temperature_report.valid;
    report.temperature_c = temperature_report.temperature_c;
    report.target_temperature_c = temperature_report.target_c;
    report.heater_duty = imu_heater_get_duty();
    report.temperature_updated_ms = temperature_report.updated_ms;
    report.temperature_state = temperature_report.state;
    report.temperature_faults = temperature_report.faults;
    if (initializing) { report.initialization_attempts++; }
    if (sensor != BMI088_OK && sensor != BMI088_NO_DATA) { report.errors++; }
    if (sample) { report.last_raw = *sample; report.samples++; }
    app_critical_exit(key);
}

app_status_t imu_task_get_report(imu_task_report_t *out)
{
    if (!out) { return APP_STATUS_INVALID_PARAM; }
    const uint32_t key = app_critical_enter();
    *out = report;
    app_critical_exit(key);
    return out->status;
}

static uint32_t ticks_for_ms(uint32_t ms)
{
    uint32_t ticks = (uint32_t)(((uint64_t)ms * osKernelGetTickFreq() + 999u) / 1000u);
    return ticks ? ticks : 1u;
}

static uint32_t map_temperature_faults(uint32_t faults)
{
    uint32_t mapped = 0;
    if (faults & (IMU_TEMP_FAULT_CONFIG | IMU_TEMP_FAULT_SENSOR |
                  IMU_TEMP_FAULT_RANGE)) {
        mapped |= IMU_FAULT_TEMP_INVALID;
    }
    if (faults & IMU_TEMP_FAULT_STALE) { mapped |= IMU_FAULT_TEMP_STALE; }
    if (faults & IMU_TEMP_FAULT_OVERTEMP) { mapped |= IMU_FAULT_OVERTEMP; }
    if (faults & IMU_TEMP_FAULT_RUNAWAY) { mapped |= IMU_FAULT_TEMP_RUNAWAY; }
    if (faults & IMU_TEMP_FAULT_PREHEAT_TIMEOUT) {
        mapped |= IMU_FAULT_PREHEAT_TIMEOUT;
    }
    return mapped;
}

static void apply_heater(void)
{
    if (temperature_report.heater_allowed && temperature_report.duty > 0.0f) {
        imu_heater_set_duty(temperature_report.duty, temperature_report.updated_ms);
    } else {
        imu_heater_stop();
    }
}

static void enter_fault(uint32_t faults, bmi088_result_t sensor,
                        const bmi088_sample_t *sample, uint32_t now)
{
    state = IMU_TASK_STATE_FAULT;
    task_faults |= faults;
    imu_heater_stop();
    imu_temperature_abort(&temperature, 0, now);
    temperature_report.state = IMU_TEMP_STATE_FAULT;
    temperature_report.duty = 0.0f;
    temperature_report.heater_allowed = false;
    invalidate(now);
    record(sensor, APP_STATUS_FAULT, sample, false);
}

static void handle_valid_sample(const bmi088_sample_t *sample, uint32_t now)
{
    const imu_config_t *config = imu_config_get();
    app_status_t status = APP_STATUS_NOT_READY;
    const uint32_t temp_faults = map_temperature_faults(temperature_report.faults);

    if (temp_faults != 0u) {
        enter_fault(temp_faults, BMI088_OK, sample, now);
        return;
    }

    switch (state) {
    case IMU_TASK_STATE_TEMP_PREP:
        if (temperature_report.ready) {
            imu_fusion_reset();
            calibration_start_ms = now;
            state = IMU_TASK_STATE_CALIBRATING;
        }
        status = APP_STATUS_NOT_READY;
        break;

    case IMU_TASK_STATE_CALIBRATING: {
        if (fabsf(sample->temperature_c - config->temperature.target_c) >
            config->temperature.calibration_tolerance_c) {
            /* Temperature moved during calibration: discard partial bias and
             * re-enter preparation instead of silently keeping bad data. */
            imu_fusion_reset();
            state = IMU_TASK_STATE_TEMP_PREP;
            invalidate(now);
            status = APP_STATUS_NOT_READY;
            break;
        }
        app_attitude_snapshot_t attitude;
        const app_status_t fusion_status = imu_fusion_push(sample, &attitude);
        (void)app_attitude_ingest(&attitude);
        if (fusion_status == APP_STATUS_OK) {
            state = IMU_TASK_STATE_RUNNING;
            status = APP_STATUS_OK;
        } else if (fusion_status == APP_STATUS_NOT_READY) {
            status = APP_STATUS_NOT_READY;
        } else {
            enter_fault(IMU_FAULT_FUSION, BMI088_OK, sample, now);
            return;
        }
        if (state == IMU_TASK_STATE_CALIBRATING &&
            (uint32_t)(now - calibration_start_ms) > config->calibration_timeout_ms) {
            enter_fault(IMU_FAULT_CALIBRATION_TIMEOUT, BMI088_OK, sample, now);
            return;
        }
        break;
    }

    case IMU_TASK_STATE_RUNNING: {
        if (fabsf(sample->temperature_c - config->temperature.target_c) >
            config->temperature.running_tolerance_c) {
            /* Temperature left the validated band: stop publishing and let the
             * heater/prep path re-stabilise before calibrating again. */
            imu_fusion_reset();
            state = IMU_TASK_STATE_TEMP_PREP;
            invalidate(now);
            status = APP_STATUS_NOT_READY;
            break;
        }
        app_attitude_snapshot_t attitude;
        const app_status_t fusion_status = imu_fusion_push(sample, &attitude);
        (void)app_attitude_ingest(&attitude);
        if (fusion_status == APP_STATUS_OK) {
            status = APP_STATUS_OK;
        } else {
            enter_fault(IMU_FAULT_FUSION, BMI088_OK, sample, now);
            return;
        }
        break;
    }

    default:
        status = APP_STATUS_NOT_READY;
        break;
    }

    record(BMI088_OK, status, sample, false);
}

static void handle_no_data(uint32_t now)
{
    const imu_config_t *config = imu_config_get();
    temperature_report = imu_temperature_supervise(&temperature, now);
    const uint32_t temp_faults = map_temperature_faults(temperature_report.faults);
    if (temp_faults != 0u) {
        enter_fault(temp_faults, BMI088_NO_DATA, NULL, now);
        return;
    }
    if ((uint32_t)(now - last_sample_ms) > config->temperature.stale_timeout_ms) {
        enter_fault(IMU_FAULT_TEMP_STALE, BMI088_NO_DATA, NULL, now);
        return;
    }
    record(BMI088_NO_DATA,
           state == IMU_TASK_STATE_RUNNING ? APP_STATUS_OK : APP_STATUS_NOT_READY,
           NULL, false);
}

static void task_init_once(void)
{
    const imu_config_t *config = imu_config_get();
    const uint32_t now = app_time_now_ms();
    task_faults = 0;
    last_sensor_result = BMI088_BUS_ERROR;
    imu_heater_stop();
    imu_temperature_reset(&temperature);
    temperature_report = imu_temperature_supervise(&temperature, now);
    imu_fusion_reset();
    invalidate(now);

    if (!imu_heater_is_ready() &&
        !imu_heater_init(config->temperature.stale_timeout_ms)) {
        task_faults |= IMU_FAULT_HEATER;
        state = IMU_TASK_STATE_FAULT;
        record(last_sensor_result, APP_STATUS_FAULT, NULL, true);
        return;
    }

    last_sensor_result = bmi088_initialize(imu_bsp_bus());
    if (last_sensor_result != BMI088_OK) {
        task_faults |= IMU_FAULT_SENSOR;
        state = IMU_TASK_STATE_FAULT;
        record(last_sensor_result, APP_STATUS_FAULT, NULL, true);
        return;
    }
    state = IMU_TASK_STATE_TEMP_PREP;
    last_sample_ms = app_time_now_ms();
    calibration_start_ms = 0;
    record(BMI088_OK, APP_STATUS_NOT_READY, NULL, true);
}

void imu_task(void *argument)
{
    (void)argument;
    const imu_config_t *config = imu_config_get();
    imu_bsp_init();
    imu_temperature_init(&temperature, &config->temperature);
    temperature_report = imu_temperature_supervise(&temperature, app_time_now_ms());
    imu_fusion_reset();
    invalidate(app_time_now_ms());
    uint32_t next_tick = osKernelGetTickCount();

    for (;;) {
        if (state == IMU_TASK_STATE_INIT) {
            task_init_once();
            next_tick = osKernelGetTickCount();
            continue;
        }
        if (state == IMU_TASK_STATE_FAULT) {
            imu_heater_stop();
            invalidate(app_time_now_ms());
            record(last_sensor_result, APP_STATUS_FAULT, NULL, false);
            (void)osDelay(ticks_for_ms(500u));
            state = IMU_TASK_STATE_INIT;
            next_tick = osKernelGetTickCount();
            continue;
        }

        bmi088_sample_t sample;
        const bmi088_result_t result = bmi088_sample(imu_bsp_bus(), &sample);
        const uint32_t now = app_time_now_ms();
        sample.timestamp_ms = now;
        sample.timestamp_us = imu_bsp_now_us();

        if (result == BMI088_OK) {
            last_sample_ms = now;
            last_sensor_result = result;
            temperature_report = imu_temperature_update(
                &temperature, sample.temperature_c, true, now);
            apply_heater();
            handle_valid_sample(&sample, now);
        } else if (result == BMI088_NO_DATA) {
            last_sensor_result = result;
            handle_no_data(now);
        } else {
            last_sensor_result = result;
            enter_fault(IMU_FAULT_SENSOR, result, NULL, now);
        }

        if (state == IMU_TASK_STATE_FAULT) {
            continue; /* fault delay is handled at the top of the loop */
        }
        next_tick += ticks_for_ms(config->sample_period_ms);
        if ((int32_t)(next_tick - osKernelGetTickCount()) <= 0) {
            next_tick = osKernelGetTickCount() + ticks_for_ms(config->sample_period_ms);
        }
        (void)osDelayUntil(next_tick);
    }
}
