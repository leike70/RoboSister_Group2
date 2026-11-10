/* Unit tests for the BMI088 heater temperature controller and PWM watchdog.
 * Pure host logic; no board hardware is claimed by these tests. */
#include <math.h>
#include <stdio.h>
#include <stdlib.h>

#include "IMU/imu_temperature.h"
#include "imu_heater.h"
#include "tim.h"

#define CHECK(x) do { if (!(x)) { fprintf(stderr, "FAIL %s:%d: %s\n", __FILE__, __LINE__, #x); exit(1); } } while (0)

static TIM_TypeDef test_tim = {.ARR = 1999u};
TIM_HandleTypeDef htim10 = {.Instance = &test_tim};
static unsigned pwm_starts;
static bool fail_start;

HAL_StatusTypeDef HAL_TIM_PWM_Start(TIM_HandleTypeDef *handle, uint32_t channel)
{
    CHECK(handle == &htim10 && channel == TIM_CHANNEL_1);
    if (fail_start) { return HAL_ERROR; }
    pwm_starts++;
    return HAL_OK;
}

static imu_temperature_config_t test_config(void)
{
    imu_temperature_config_t c = {
        .target_c = 40.0f,
        .pwm_max_duty = 0.80f,
        .kp = 0.5f,
        .ki = 0.01f,
        .kd = 0.0f,
        .integral_limit_duty = 0.15f,
        .deadband_c = 0.1f,
        .ready_tolerance_c = 1.0f,
        .calibration_tolerance_c = 2.0f,
        .running_tolerance_c = 3.0f,
        .ready_hold_ms = 3000u,
        .preheat_timeout_ms = 120000u,
        .stale_timeout_ms = 300u,
        .control_period_ms = 100u,
        .overtemp_c = 50.0f,
        .min_valid_c = -20.0f,
        .max_valid_c = 60.0f,
        .runaway_margin_c = 3.0f,
        .runaway_hold_ms = 5000u
    };
    return c;
}

static imu_temperature_t ctrl;
static imu_temperature_config_t cfg;
static imu_temperature_report_t report;

static void reset_controller(void)
{
    cfg = test_config();
    imu_temperature_init(&ctrl, &cfg);
    report = imu_temperature_supervise(&ctrl, 0u);
}

static void feed(float temperature, uint32_t now_ms)
{
    report = imu_temperature_update(&ctrl, temperature, true, now_ms);
}

static void feed_invalid(float temperature, uint32_t now_ms)
{
    report = imu_temperature_update(&ctrl, temperature, false, now_ms);
}

static void test_pwm_limits_and_anti_windup(void)
{
    reset_controller();
    feed(25.0f, 100u);
    CHECK(report.valid && report.heater_allowed);
    CHECK(report.duty > 0.0f && report.duty <= cfg.pwm_max_duty);
    CHECK(report.integral_duty <= cfg.integral_limit_duty);

    for (uint32_t t = 200u; t <= 5000u; t += 100u) {
        feed(25.0f, t);
        CHECK(report.duty >= 0.0f && report.duty <= cfg.pwm_max_duty);
        CHECK(report.integral_duty <= cfg.integral_limit_duty);
    }
    CHECK(report.state == IMU_TEMP_STATE_HEATING);

    /* A negative fault command must not drive the heater above the cap. */
    report = imu_temperature_update(&ctrl, 25.0f, true, 5100u);
    CHECK(report.duty <= cfg.pwm_max_duty);
    puts("PASS temperature duty cap, saturation and integral clamp");
}

static void test_ready_hold_and_preheat_timeout(void)
{
    reset_controller();
    feed(40.0f, 100u);
    CHECK(!report.ready && report.state == IMU_TEMP_STATE_HEATING);
    feed(40.0f, 3000u);
    CHECK(!report.ready);
    feed(40.0f, 3100u);
    CHECK(report.ready && report.state == IMU_TEMP_STATE_READY);
    CHECK(report.heater_allowed);

    reset_controller();
    cfg.preheat_timeout_ms = 1000u;
    imu_temperature_init(&ctrl, &cfg);
    feed(25.0f, 100u);
    CHECK(report.faults == 0u);
    feed(25.0f, 1101u);
    CHECK((report.faults & IMU_TEMP_FAULT_PREHEAT_TIMEOUT) != 0u);
    CHECK(report.state == IMU_TEMP_STATE_FAULT && report.duty == 0.0f);
    puts("PASS ready hold and preheat timeout");
}

static void test_invalid_stale_and_supervise(void)
{
    reset_controller();
    feed(25.0f, 100u);
    feed_invalid(NAN, 150u);
    CHECK(!report.valid && report.duty == 0.0f &&
          report.state == IMU_TEMP_STATE_WAIT_VALID);
    feed_invalid(NAN, 401u);
    CHECK((report.faults & IMU_TEMP_FAULT_STALE) != 0u);
    CHECK(report.state == IMU_TEMP_STATE_FAULT && !report.heater_allowed);

    reset_controller();
    feed(25.0f, 100u);
    report = imu_temperature_supervise(&ctrl, 401u);
    CHECK((report.faults & IMU_TEMP_FAULT_STALE) != 0u && report.duty == 0.0f);
    puts("PASS invalid temperature, stale input and supervise timeout");
}

static void test_ready_loss_and_reentry(void)
{
    reset_controller();
    feed(40.0f, 100u);
    feed(40.0f, 3100u);
    CHECK(report.ready);
    feed(35.0f, 3200u);
    CHECK(!report.ready && report.state == IMU_TEMP_STATE_HEATING);
    CHECK(report.faults == 0u);
    feed(40.0f, 3300u);
    CHECK(!report.ready);
    feed(40.0f, 6301u);
    CHECK(report.ready && report.state == IMU_TEMP_STATE_READY);
    puts("PASS ready loss and re-entry after a temperature step");
}

static void test_overtemp_range_and_runaway(void)
{
    reset_controller();
    feed(51.0f, 100u);
    CHECK((report.faults & IMU_TEMP_FAULT_OVERTEMP) != 0u);
    CHECK(report.duty == 0.0f && report.state == IMU_TEMP_STATE_FAULT);

    reset_controller();
    feed(100.0f, 100u);
    CHECK((report.faults & IMU_TEMP_FAULT_RANGE) != 0u);
    CHECK(report.duty == 0.0f);

    reset_controller();
    feed(30.0f, 100u);
    CHECK(report.duty > 0.0f);
    feed(44.0f, 200u);
    CHECK((report.faults & IMU_TEMP_FAULT_RUNAWAY) == 0u);
    feed(44.0f, 5199u);
    CHECK((report.faults & IMU_TEMP_FAULT_RUNAWAY) == 0u);
    feed(44.0f, 5301u);
    CHECK((report.faults & IMU_TEMP_FAULT_RUNAWAY) != 0u);
    CHECK(report.duty == 0.0f);
    puts("PASS over-temperature, range and runaway protection");
}

static void test_abort_reset_and_recovery(void)
{
    reset_controller();
    feed(25.0f, 100u);
    imu_temperature_abort(&ctrl, IMU_TEMP_FAULT_SENSOR, 200u);
    report = imu_temperature_supervise(&ctrl, 200u);
    CHECK((report.faults & IMU_TEMP_FAULT_SENSOR) != 0u);
    CHECK(report.state == IMU_TEMP_STATE_FAULT && report.duty == 0.0f);

    imu_temperature_reset(&ctrl);
    report = imu_temperature_supervise(&ctrl, 300u);
    CHECK(report.faults == 0u && report.state == IMU_TEMP_STATE_WAIT_VALID);
    feed(25.0f, 400u);
    CHECK(report.valid && report.duty > 0.0f && report.state == IMU_TEMP_STATE_HEATING);
    puts("PASS fault latch, reset and re-entry after abort");
}

static void test_config_rejection(void)
{
    imu_temperature_config_t bad = test_config();
    bad.pwm_max_duty = 0.0f;
    imu_temperature_init(&ctrl, &bad);
    report = imu_temperature_supervise(&ctrl, 0u);
    CHECK(report.state == IMU_TEMP_STATE_FAULT);
    CHECK((report.faults & IMU_TEMP_FAULT_CONFIG) != 0u);
    CHECK(report.duty == 0.0f);
    puts("PASS invalid temperature configuration rejection");
}

static void test_heater_pwm_and_watchdog(void)
{
    test_tim.ARR = 1999u;
    test_tim.CCR1 = 0xFFFFFFFFu;
    pwm_starts = 0;
    fail_start = true;
    CHECK(!imu_heater_init(300u));
    CHECK(!imu_heater_is_ready());

    fail_start = false;
    CHECK(imu_heater_init(300u));
    CHECK(imu_heater_is_ready() && pwm_starts == 1u);
    CHECK(test_tim.CCR1 == 0u && imu_heater_get_duty() == 0.0f);

    imu_heater_set_duty(0.5f, 1000u);
    CHECK(test_tim.CCR1 > 0u && test_tim.CCR1 <= test_tim.ARR);
    CHECK(imu_heater_get_duty() > 0.45f && imu_heater_get_duty() < 0.55f);

    imu_heater_set_duty(2.0f, 1100u);
    CHECK(test_tim.CCR1 == test_tim.ARR);
    imu_heater_set_duty(-1.0f, 1200u);
    CHECK(test_tim.CCR1 == 0u && imu_heater_get_duty() == 0.0f);

    imu_heater_set_duty(0.5f, 2000u);
    imu_heater_supervise(2300u);
    CHECK(test_tim.CCR1 > 0u && !imu_heater_is_stale());
    imu_heater_supervise(2301u);
    CHECK(test_tim.CCR1 == 0u && imu_heater_get_duty() == 0.0f);
    CHECK(imu_heater_is_stale());
    imu_heater_set_duty(0.5f, 2400u);
    CHECK(!imu_heater_is_stale() && test_tim.CCR1 > 0u);
    imu_heater_stop();
    CHECK(test_tim.CCR1 == 0u && imu_heater_get_duty() == 0.0f);
    puts("PASS heater duty limits, failed start and stale-command watchdog");
}

int main(void)
{
    test_pwm_limits_and_anti_windup();
    test_ready_hold_and_preheat_timeout();
    test_ready_loss_and_reentry();
    test_invalid_stale_and_supervise();
    test_overtemp_range_and_runaway();
    test_abort_reset_and_recovery();
    test_config_rejection();
    test_heater_pwm_and_watchdog();
    puts("PASS IMU thermal regression suite");
    return 0;
}
