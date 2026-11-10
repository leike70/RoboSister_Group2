#include "imu_heater.h"

#include <math.h>

#include "tim.h"

static bool initialized;
static bool active;
static bool stale;
static uint32_t stale_timeout_ms;
static uint32_t last_command_ms;
static float applied_duty;

static void write_duty(float duty)
{
    if (!initialized) { return; }
    if (!isfinite(duty) || duty < 0.0f) { duty = 0.0f; }
    if (duty > 1.0f) { duty = 1.0f; }
    const uint32_t arr = htim10.Instance->ARR;
    uint32_t compare = (uint32_t)(duty * (float)arr + 0.5f);
    if (compare > arr) { compare = arr; }
    __HAL_TIM_SET_COMPARE(&htim10, TIM_CHANNEL_1, compare);
    applied_duty = (arr == 0u) ? 0.0f : (float)compare / (float)arr;
}

bool imu_heater_init(uint32_t timeout_ms)
{
    if (initialized) { return true; }
    stale_timeout_ms = (timeout_ms == 0u) ? 1u : timeout_ms;
    if (HAL_TIM_PWM_Start(&htim10, TIM_CHANNEL_1) != HAL_OK) { return false; }
    initialized = true;
    active = false;
    stale = false;
    last_command_ms = 0u;
    applied_duty = 0.0f;
    write_duty(0.0f);
    return true;
}

void imu_heater_set_duty(float duty, uint32_t now_ms)
{
    if (!initialized) { return; }
    if (!isfinite(duty) || duty <= 0.0f) {
        imu_heater_stop();
        return;
    }
    write_duty(duty);
    last_command_ms = now_ms;
    active = applied_duty > 0.0f;
    if (active) { stale = false; }
}

void imu_heater_stop(void)
{
    if (!initialized) { return; }
    write_duty(0.0f);
    active = false;
    stale = false;
}

void imu_heater_supervise(uint32_t now_ms)
{
    if (!initialized || !active) { return; }
    if ((uint32_t)(now_ms - last_command_ms) > stale_timeout_ms) {
        write_duty(0.0f);
        active = false;
        stale = true;
    }
}

float imu_heater_get_duty(void) { return applied_duty; }
bool imu_heater_is_stale(void) { return stale; }
bool imu_heater_is_ready(void) { return initialized; }
