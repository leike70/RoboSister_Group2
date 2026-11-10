#include "imu_temperature.h"

#include <math.h>

static float clampf(float value, float low, float high)
{
    if (value < low) { return low; }
    if (value > high) { return high; }
    return value;
}

static imu_temperature_report_t make_report(const imu_temperature_t *ctrl,
                                            float temperature_c, bool valid,
                                            uint32_t now_ms)
{
    imu_temperature_report_t out = {0};
    out.temperature_c = temperature_c;
    out.updated_ms = now_ms;
    if (!ctrl) {
        out.state = IMU_TEMP_STATE_FAULT;
        out.faults = IMU_TEMP_FAULT_CONFIG;
        return out;
    }
    out.state = ctrl->state;
    out.faults = ctrl->faults;
    out.valid = valid;
    out.ready = ctrl->ready;
    out.target_c = ctrl->config.target_c;
    out.duty = ctrl->last_duty;
    out.integral_duty = ctrl->integral;
    out.heater_allowed = valid && ctrl->have_valid &&
                         ctrl->faults == 0u &&
                         ctrl->state != IMU_TEMP_STATE_OFF &&
                         ctrl->state != IMU_TEMP_STATE_WAIT_VALID &&
                         ctrl->state != IMU_TEMP_STATE_FAULT;
    if (out.state == IMU_TEMP_STATE_FAULT) { out.heater_allowed = false; }
    return out;
}

static bool config_valid(const imu_temperature_config_t *c)
{
    if (!c) { return false; }
    if (!isfinite(c->target_c) || !isfinite(c->pwm_max_duty) ||
        !isfinite(c->kp) || !isfinite(c->ki) || !isfinite(c->kd) ||
        !isfinite(c->integral_limit_duty) || !isfinite(c->deadband_c) ||
        !isfinite(c->ready_tolerance_c) || !isfinite(c->calibration_tolerance_c) ||
        !isfinite(c->running_tolerance_c) || !isfinite(c->overtemp_c) ||
        !isfinite(c->min_valid_c) || !isfinite(c->max_valid_c) ||
        !isfinite(c->runaway_margin_c)) {
        return false;
    }
    if (c->pwm_max_duty <= 0.0f || c->pwm_max_duty > 1.0f) { return false; }
    if (c->min_valid_c >= c->max_valid_c) { return false; }
    if (c->target_c <= c->min_valid_c || c->target_c >= c->max_valid_c) {
        return false;
    }
    if (c->overtemp_c <= c->target_c || c->overtemp_c > c->max_valid_c) {
        return false;
    }
    if (c->kp < 0.0f || c->ki < 0.0f || c->kd < 0.0f ||
        c->integral_limit_duty < 0.0f || c->deadband_c < 0.0f ||
        c->ready_tolerance_c < 0.0f || c->calibration_tolerance_c < 0.0f ||
        c->running_tolerance_c < 0.0f || c->runaway_margin_c < 0.0f) {
        return false;
    }
    if (c->control_period_ms == 0u || c->ready_hold_ms == 0u ||
        c->preheat_timeout_ms == 0u || c->stale_timeout_ms == 0u ||
        c->runaway_hold_ms == 0u) {
        return false;
    }
    return true;
}

void imu_temperature_reset(imu_temperature_t *ctrl)
{
    if (!ctrl) { return; }
    ctrl->state = ctrl->config_ok ? IMU_TEMP_STATE_WAIT_VALID : IMU_TEMP_STATE_FAULT;
    ctrl->faults = ctrl->config_ok ? 0u : IMU_TEMP_FAULT_CONFIG;
    ctrl->integral = 0.0f;
    ctrl->last_temperature_c = 0.0f;
    ctrl->last_duty = 0.0f;
    ctrl->last_error = 0.0f;
    ctrl->last_valid_ms = 0;
    ctrl->last_control_ms = 0;
    ctrl->preheat_start_ms = 0;
    ctrl->ready_since_ms = 0;
    ctrl->runaway_since_ms = 0;
    ctrl->have_valid = false;
    ctrl->have_control = false;
    ctrl->have_error = false;
    ctrl->ready = false;
    ctrl->ready_timer_active = false;
    ctrl->runaway_active = false;
    ctrl->ever_ready = false;
}

void imu_temperature_init(imu_temperature_t *ctrl,
                          const imu_temperature_config_t *config)
{
    if (!ctrl) { return; }
    ctrl->initialized = false;
    ctrl->config = (imu_temperature_config_t){0};
    ctrl->config_ok = config_valid(config);
    imu_temperature_reset(ctrl);
    ctrl->initialized = true;
    if (!ctrl->config_ok) {
        ctrl->state = IMU_TEMP_STATE_FAULT;
        ctrl->faults = IMU_TEMP_FAULT_CONFIG;
        return;
    }
    ctrl->config = *config;
    ctrl->state = IMU_TEMP_STATE_WAIT_VALID;
}

void imu_temperature_abort(imu_temperature_t *ctrl, uint32_t faults,
                           uint32_t now_ms)
{
    if (!ctrl) { return; }
    ctrl->faults |= faults;
    ctrl->state = IMU_TEMP_STATE_FAULT;
    ctrl->integral = 0.0f;
    ctrl->last_duty = 0.0f;
    ctrl->last_error = 0.0f;
    ctrl->have_control = false;
    ctrl->have_error = false;
    ctrl->ready = false;
    ctrl->ready_timer_active = false;
    ctrl->runaway_active = false;
    (void)now_ms;
}

static void evaluate_ready(imu_temperature_t *ctrl, float temperature_c,
                           uint32_t now_ms)
{
    const float error = fabsf(temperature_c - ctrl->config.target_c);
    if (error <= ctrl->config.ready_tolerance_c) {
        if (!ctrl->ready_timer_active) {
            ctrl->ready_timer_active = true;
            ctrl->ready_since_ms = now_ms;
        }
        if (ctrl->ready && !ctrl->ever_ready) { ctrl->ever_ready = true; }
        if ((uint32_t)(now_ms - ctrl->ready_since_ms) >=
            ctrl->config.ready_hold_ms) {
            ctrl->ready = true;
            ctrl->ever_ready = true;
            ctrl->state = IMU_TEMP_STATE_READY;
        } else {
            ctrl->state = IMU_TEMP_STATE_HEATING;
        }
    } else {
        ctrl->ready = false;
        ctrl->ready_timer_active = false;
        ctrl->state = IMU_TEMP_STATE_HEATING;
    }
}

static void run_pid(imu_temperature_t *ctrl, float temperature_c,
                    uint32_t now_ms)
{
    const imu_temperature_config_t *c = &ctrl->config;
    const float error = c->target_c - temperature_c;
    float dt;
    if (ctrl->have_control) {
        dt = (float)(uint32_t)(now_ms - ctrl->last_control_ms) * 0.001f;
    } else {
        dt = (float)c->control_period_ms * 0.001f;
        ctrl->have_control = true;
    }
    ctrl->last_control_ms = now_ms;
    if (!isfinite(dt) || dt <= 0.0f) { dt = (float)c->control_period_ms * 0.001f; }
    if (dt > 1.0f) { dt = 1.0f; }

    if (fabsf(error) > c->deadband_c) {
        ctrl->integral += c->ki * error * dt;
    }
    ctrl->integral = clampf(ctrl->integral, 0.0f, c->integral_limit_duty);

    float duty = c->kp * error + ctrl->integral;
    if (c->kd > 0.0f) {
        float derivative = 0.0f;
        if (ctrl->have_error && dt > 0.0f) {
            derivative = (error - ctrl->last_error) / dt;
        }
        duty += c->kd * derivative;
        ctrl->last_error = error;
        ctrl->have_error = true;
    }

    /* Back-calculation anti-windup: do not keep winding the integrator once
     * the actuator is saturated high. */
    if (duty > c->pwm_max_duty) {
        duty = c->pwm_max_duty;
        const float allowed_integral = c->pwm_max_duty - c->kp * error;
        if (ctrl->integral > allowed_integral) {
            ctrl->integral = clampf(allowed_integral, 0.0f,
                                    c->integral_limit_duty);
        }
    } else if (duty < 0.0f) {
        duty = 0.0f;
    }
    ctrl->last_duty = clampf(duty, 0.0f, c->pwm_max_duty);
}

imu_temperature_report_t imu_temperature_update(imu_temperature_t *ctrl,
                                                float temperature_c,
                                                bool valid,
                                                uint32_t now_ms)
{
    if (!ctrl) { return make_report(NULL, temperature_c, false, now_ms); }
    if (!ctrl->initialized) {
        return make_report(ctrl, temperature_c, false, now_ms);
    }
    if (ctrl->faults != 0u) {
        return make_report(ctrl, temperature_c, false, now_ms);
    }

    if (!valid || !isfinite(temperature_c)) {
        ctrl->last_duty = 0.0f;
        ctrl->ready = false;
        ctrl->ready_timer_active = false;
        ctrl->state = IMU_TEMP_STATE_WAIT_VALID;
        if (ctrl->have_valid &&
            (uint32_t)(now_ms - ctrl->last_valid_ms) >
                ctrl->config.stale_timeout_ms) {
            ctrl->faults |= IMU_TEMP_FAULT_STALE;
            ctrl->state = IMU_TEMP_STATE_FAULT;
        }
        return make_report(ctrl, temperature_c, false, now_ms);
    }

    if (temperature_c < ctrl->config.min_valid_c ||
        temperature_c > ctrl->config.max_valid_c) {
        imu_temperature_abort(ctrl, IMU_TEMP_FAULT_RANGE, now_ms);
        return make_report(ctrl, temperature_c, false, now_ms);
    }

    if (!ctrl->have_valid) {
        ctrl->have_valid = true;
        ctrl->preheat_start_ms = now_ms;
        ctrl->last_control_ms = now_ms;
        ctrl->have_control = false;
    }
    ctrl->last_valid_ms = now_ms;
    ctrl->last_temperature_c = temperature_c;
    ctrl->state = IMU_TEMP_STATE_HEATING;

    if (temperature_c > ctrl->config.overtemp_c) {
        imu_temperature_abort(ctrl, IMU_TEMP_FAULT_OVERTEMP, now_ms);
        return make_report(ctrl, temperature_c, false, now_ms);
    }

    if (temperature_c > ctrl->config.target_c + ctrl->config.runaway_margin_c) {
        if (!ctrl->runaway_active && ctrl->last_duty > 0.0f) {
            ctrl->runaway_active = true;
            ctrl->runaway_since_ms = now_ms;
        }
        if (ctrl->runaway_active &&
            (uint32_t)(now_ms - ctrl->runaway_since_ms) >=
                ctrl->config.runaway_hold_ms) {
            imu_temperature_abort(ctrl, IMU_TEMP_FAULT_RUNAWAY, now_ms);
            return make_report(ctrl, temperature_c, false, now_ms);
        }
    } else {
        ctrl->runaway_active = false;
    }

    evaluate_ready(ctrl, temperature_c, now_ms);
    if (!ctrl->ever_ready &&
        (uint32_t)(now_ms - ctrl->preheat_start_ms) >
            ctrl->config.preheat_timeout_ms) {
        imu_temperature_abort(ctrl, IMU_TEMP_FAULT_PREHEAT_TIMEOUT, now_ms);
        return make_report(ctrl, temperature_c, false, now_ms);
    }

    if (!ctrl->have_control ||
        (uint32_t)(now_ms - ctrl->last_control_ms) >=
            ctrl->config.control_period_ms) {
        run_pid(ctrl, temperature_c, now_ms);
    }
    ctrl->state = ctrl->ready ? IMU_TEMP_STATE_READY : IMU_TEMP_STATE_HEATING;
    return make_report(ctrl, temperature_c, true, now_ms);
}

imu_temperature_report_t imu_temperature_supervise(imu_temperature_t *ctrl,
                                                   uint32_t now_ms)
{
    if (!ctrl) { return make_report(NULL, 0.0f, false, now_ms); }
    if (!ctrl->initialized) {
        return make_report(ctrl, ctrl->last_temperature_c, false, now_ms);
    }
    if (ctrl->faults == 0u && ctrl->have_valid &&
        (uint32_t)(now_ms - ctrl->last_valid_ms) >
            ctrl->config.stale_timeout_ms) {
        imu_temperature_abort(ctrl, IMU_TEMP_FAULT_STALE, now_ms);
    }
    return make_report(ctrl, ctrl->last_temperature_c,
                       ctrl->faults == 0u && ctrl->have_valid, now_ms);
}
