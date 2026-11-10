#include <stdio.h>
#include <stdbool.h>

#include "app_config.h"
#include "Motor/app_motor.h"
#include "Power/app_power.h"
#include "Protection/app_protection.h"

static int failures;

#define CHECK(cond) do { \
    if (!(cond)) { printf("FAIL line %d: %s\n", __LINE__, #cond); failures++; } \
} while (0)

/* Test-only time/remote boundary. Production Config/Motor/Power/Protection are
 * linked unchanged; this test deliberately exercises their unconfigured path. */
uint32_t app_time_now_ms(void) { return 1000u; }
app_status_t app_remote_status(void) { return APP_STATUS_OFFLINE; }

int main(void) {
    app_motor_output_t output = {0};
    app_power_budget_t budget = {0};
    float scale = 1.0f;
    const app_config_report_t *config_report;

    CHECK(app_config_init() == APP_STATUS_UNCONFIGURED);
    config_report = app_config_report();
    CHECK(config_report != NULL);
    CHECK(config_report->status == APP_STATUS_UNCONFIGURED);
    CHECK(config_report->motors_configured == 0u);
    CHECK(config_report->motors_motion_usable == 0u);

    CHECK(app_motor_init(1000u) == APP_STATUS_UNCONFIGURED);
    CHECK(app_motor_registered_count() == 0);
    CHECK(!app_motor_adapter_ready());
    CHECK(app_motor_request(0, APP_MOTOR_CMD_CURRENT_NORM, 0.1f) == APP_STATUS_UNCONFIGURED);
    CHECK(app_motor_submit(1000u) == APP_STATUS_NOT_IMPLEMENTED);
    CHECK(app_motor_output_get(0, &output) != APP_STATUS_OK || !output.enabled);

    CHECK(app_power_init(1000u) == APP_STATUS_UNCONFIGURED);
    CHECK(app_power_update(1000u) == APP_STATUS_UNCONFIGURED);
    CHECK(app_power_status() == APP_STATUS_UNCONFIGURED);
    CHECK(app_power_get_budget(&budget) == APP_STATUS_UNCONFIGURED);
    CHECK(!budget.valid);
    CHECK(app_power_limit_scale(1.0f, &scale) == APP_STATUS_UNCONFIGURED);
    CHECK(scale == 0.0f);

    CHECK(app_protection_init(1000u) == APP_STATUS_NOT_READY);
    CHECK(app_protection_evaluate(1000u) == APP_STATUS_UNCONFIGURED);
    CHECK(app_protection_report()->state == APP_RUN_STATE_WAIT_CONFIG);
    CHECK(!app_protection_output_allowed());

    /* Logical e-stop: latches, revokes output, and cannot be cleared while active. */
    CHECK(!app_protection_emergency_stop_active());
    app_protection_emergency_stop();
    CHECK(app_protection_emergency_stop_active());
    CHECK(app_protection_evaluate(1001u) == APP_STATUS_FAULT);
    CHECK(!app_protection_output_allowed());
    CHECK((app_protection_report()->active_faults & APP_FAULT_MANUAL) != 0u);
    CHECK(!app_protection_clear_latched());
    app_protection_release_emergency_stop();
    CHECK(!app_protection_emergency_stop_active());
    CHECK(app_protection_clear_latched());
    CHECK(app_protection_evaluate(1002u) == APP_STATUS_UNCONFIGURED);

    if (failures) {
        printf("%d production-chain checks failed\n", failures);
        return 1;
    }
    puts("PASS real Config/Motor/Power/Protection unconfigured path and logical e-stop");
    return 0;
}
