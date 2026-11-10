#include <stdio.h>
#include <string.h>

#include "app_config.h"
#include "Motor/app_motor.h"
#include "Power/app_power.h"

static int failures;
static app_config_t fixture;

#ifdef APP_POWER_TEST
extern void app_power_test_set_before_publish(void (*hook)(void));
static app_power_feedback_t mid_feedback;
static app_power_policy_t mid_policy;
static app_power_budget_t mid_budget;
static int mid_read_count;
static void mid_update_read_hook(void) {
    app_power_get_feedback(&mid_feedback);
    app_power_get_policy(&mid_policy);
    app_power_get_budget(&mid_budget);
    mid_read_count++;
}
#endif

#define CHECK(cond) do { \
    if (!(cond)) { printf("FAIL line %d: %s\n", __LINE__, #cond); failures++; } \
} while (0)

/* Test input boundary only: production Motor/Power/Unit logic is linked. */
const app_config_t *app_config_get(void) { return &fixture; }
app_status_t app_config_status(void) { return APP_STATUS_OK; }
const app_motor_config_t *app_config_motor(int slot) {
    return (slot >= 0 && slot < (int)APP_MOTOR_COUNT) ? &fixture.motors[slot] : NULL;
}
uint32_t app_config_motor_missing(int slot) {
    return (slot >= 0 && slot < (int)APP_MOTOR_COUNT) ? 0u : 0xFFFFFFFFu;
}
bool app_config_motor_comms_usable(int slot) {
    return (slot == 0 || slot == 1) && fixture.motors[slot].configured;
}
bool app_config_motor_motion_usable(int slot) {
    return (slot == 0 || slot == 1) && fixture.motors[slot].configured;
}
int app_config_find_motor(app_motor_role_t role) {
    for (int i = 0; i < (int)APP_MOTOR_COUNT; ++i) {
        if (fixture.motors[i].role == role) { return i; }
    }
    return -1;
}
bool app_protection_output_allowed(void) { return true; }

static void setup_valid_config(void) {
    memset(&fixture, 0, sizeof(fixture));
    fixture.chassis.valid = true;
    fixture.chassis.max_linear_speed_mps = 1.0f;
    fixture.chassis.max_turn_rate_rad_s = 1.0f;
    fixture.power.max_total_output_norm = 10.0f;
    fixture.power.valid = true;
    fixture.power.source_bus = APP_CAN_BUS_1;
    fixture.power.source_bus_valid = true;
    fixture.power.source_timeout_ms = 100u;
    fixture.power.source_timeout_valid = true;
    fixture.power.hardware_verified = true;
    fixture.power.chassis_power_limit_w = 50.0f;
    fixture.power.chassis_power_limit_valid = true;
    fixture.power.buffer_energy_limit_j = 500.0f;
    fixture.power.buffer_energy_limit_valid = true;
    fixture.power.low_buffer_energy_j = 5.0f;
    fixture.power.low_buffer_energy_valid = true;

    fixture.motors[0].role = APP_MOTOR_ROLE_TRACK_LEFT;
    fixture.motors[0].name = "track_left";
    fixture.motors[0].model = APP_MOTOR_MODEL_DJI_M3508;
    fixture.motors[0].bus = APP_CAN_BUS_1;
    fixture.motors[0].motor_id = 1u;
    fixture.motors[0].tx_msg_id = 0x200u;
    fixture.motors[0].rx_msg_id = 0x201u;
    fixture.motors[0].control_mode = APP_MOTOR_CONTROL_SPEED;
    fixture.motors[0].direction = 1;
    fixture.motors[0].gear_ratio = 19.0f;
    fixture.motors[0].zero_offset_rad = 0.0f;
    fixture.motors[0].limit_min_rad = -10.0f;
    fixture.motors[0].limit_max_rad = 10.0f;
    fixture.motors[0].pid.valid = true;
    fixture.motors[0].configured = true;
    fixture.motors[0].bus_valid = true;
    fixture.motors[0].id_valid = true;
    fixture.motors[0].msg_id_valid = true;
    fixture.motors[0].control_mode_valid = true;
    fixture.motors[0].direction_valid = true;
    fixture.motors[0].gear_ratio_valid = true;
    fixture.motors[0].zero_offset_valid = true;
    fixture.motors[0].limit_valid = true;

    /* Non-chassis slot for the old proxy-budget isolation test. */
    fixture.motors[1].role = APP_MOTOR_ROLE_LIFT_FRONT;
    fixture.motors[1].name = "lift_front";
    fixture.motors[1].model = APP_MOTOR_MODEL_DJI_GM6020;
    fixture.motors[1].bus = APP_CAN_BUS_1;
    fixture.motors[1].motor_id = 5u;
    fixture.motors[1].tx_msg_id = 0x1ffu;
    fixture.motors[1].rx_msg_id = 0x205u;
    fixture.motors[1].control_mode = APP_MOTOR_CONTROL_SPEED;
    fixture.motors[1].direction = 1;
    fixture.motors[1].gear_ratio = 1.0f;
    fixture.motors[1].zero_offset_rad = 0.0f;
    fixture.motors[1].limit_min_rad = -10.0f;
    fixture.motors[1].limit_max_rad = 10.0f;
    fixture.motors[1].pid.valid = true;
    fixture.motors[1].configured = true;
    fixture.motors[1].bus_valid = true;
    fixture.motors[1].id_valid = true;
    fixture.motors[1].msg_id_valid = true;
    fixture.motors[1].control_mode_valid = true;
    fixture.motors[1].direction_valid = true;
    fixture.motors[1].gear_ratio_valid = true;
    fixture.motors[1].zero_offset_valid = true;
    fixture.motors[1].limit_valid = true;
}

static void frame(int16_t power_0_1w, int16_t energy_0_1j, uint32_t arrival_ms) {
    uint8_t data[4];
    data[0] = (uint8_t)((uint16_t)power_0_1w & 0xFFu);
    data[1] = (uint8_t)(((uint16_t)power_0_1w >> 8) & 0xFFu);
    data[2] = (uint8_t)((uint16_t)energy_0_1j & 0xFFu);
    data[3] = (uint8_t)(((uint16_t)energy_0_1j >> 8) & 0xFFu);
    app_power_on_can_frame(1u, APP_POWER_CAN_ID, data, 4u, arrival_ms);
}

int main(void) {
    app_motor_output_t output = {0};
    app_power_feedback_t feedback = {0};
    app_power_policy_t policy = {0};

    setup_valid_config();
    CHECK(app_power_init(0u) == APP_STATUS_UNCONFIGURED);
    CHECK(app_power_update(0u) == APP_STATUS_OFFLINE);
    CHECK(!app_power_get_feedback(&feedback) || feedback.status == APP_STATUS_OFFLINE);

    /* 50 W / 500 J: F4 01 88 13; normal policy permits chassis speed. */
    frame(500, 5000, 0u);
    CHECK(app_power_update(0u) == APP_STATUS_OK);
    CHECK(app_power_get_feedback(&feedback) == APP_STATUS_OK);
    CHECK(feedback.measured_chassis_power_w == 50.0f);
    CHECK(feedback.buffer_energy_j == 500.0f);
    CHECK(feedback.source_online && feedback.frame_valid);
    CHECK(feedback.frame_arrival_ms == 0u);
    CHECK(app_power_get_policy(&policy) == APP_STATUS_OK && policy.source_ready);
    CHECK(policy.chassis_scale == 1.0f);

    CHECK(app_motor_init(0u) == APP_STATUS_NOT_IMPLEMENTED);
    CHECK(app_motor_registered_count() == 2);
    CHECK(!app_motor_adapter_ready());
    app_motor_set_adapter_ready(true);
    CHECK(app_motor_request(0, APP_MOTOR_CMD_SPEED_RAD_S, 1.0f) == APP_STATUS_OK);
    CHECK(app_motor_submit(0u) == APP_STATUS_OK);
    CHECK(app_motor_output_get(0, &output) == APP_STATUS_OK && output.enabled);
    CHECK(output.value == 1.0f && output.current_limit_norm == 1.0f);

#ifdef APP_POWER_TEST
    /* Read while update has computed but not yet published: must return the
     * previous complete snapshot immediately, not spin or return a mix. */
    {
        app_power_feedback_t before = {0};
        app_power_policy_t before_policy = {0};
        app_power_feedback_t after = {0};
        app_power_policy_t after_policy = {0};
        CHECK(app_power_get_feedback(&before) == APP_STATUS_OK);
        CHECK(app_power_get_policy(&before_policy) == APP_STATUS_OK);
        mid_read_count = 0;
        frame(600, 10, 10u);
        app_power_test_set_before_publish(mid_update_read_hook);
        CHECK(app_power_update(10u) == APP_STATUS_OK);
        app_power_test_set_before_publish(NULL);
        CHECK(mid_read_count == 1);
        CHECK(mid_feedback.measured_chassis_power_w == before.measured_chassis_power_w);
        CHECK(mid_feedback.buffer_energy_j == before.buffer_energy_j);
        CHECK(mid_feedback.frame_arrival_ms == before.frame_arrival_ms);
        CHECK(mid_policy.state == before_policy.state);
        CHECK(mid_budget.valid == true);
        CHECK(app_power_get_feedback(&after) == APP_STATUS_OK);
        CHECK(app_power_get_policy(&after_policy) == APP_STATUS_OK);
        CHECK(after.measured_chassis_power_w == 60.0f);
        CHECK(after.buffer_energy_j == 1.0f);
        CHECK(after_policy.state == APP_POWER_POLICY_LOW_ENERGY);
    }
#endif

    /* 60 W: over 50 W limit, conservative ratio and a final current limit. */
    frame(600, 5000, 10u);
    CHECK(app_power_update(10u) == APP_STATUS_OK);
    CHECK(app_power_get_policy(&policy) == APP_STATUS_OK && policy.over_power);
    CHECK(policy.state == APP_POWER_POLICY_LIMITED);
    CHECK(policy.chassis_scale > 0.83f && policy.chassis_scale < 0.84f);
    CHECK(app_motor_request(0, APP_MOTOR_CMD_SPEED_RAD_S, 1.0f) == APP_STATUS_OK);
    CHECK(app_motor_submit(10u) == APP_STATUS_OK);
    CHECK(app_motor_output_get(0, &output) == APP_STATUS_OK && output.enabled);
    CHECK(output.value > 0.83f && output.value < 0.84f);
    CHECK(output.current_limit_norm == output.value || output.current_limit_norm < 1.0f);
    CHECK(output.power_limited);

    /* Position target is never multiplied by the power ratio; blocked instead. */
    CHECK(app_motor_request(0, APP_MOTOR_CMD_POSITION_RAD, 0.5f) == APP_STATUS_OK);
    CHECK(app_motor_submit(10u) == APP_STATUS_NOT_READY);
    CHECK(app_motor_output_get(0, &output) != APP_STATUS_OK || !output.enabled);

    /* Low energy: buffer below configured threshold disables chassis output. */
    frame(100, 10, 20u);
    CHECK(app_power_update(20u) == APP_STATUS_OK);
    CHECK(app_power_get_policy(&policy) == APP_STATUS_OK && policy.low_energy);
    CHECK(policy.state == APP_POWER_POLICY_LOW_ENERGY);
    CHECK(policy.chassis_scale == 0.0f);
    CHECK(app_motor_request(0, APP_MOTOR_CMD_SPEED_RAD_S, 1.0f) == APP_STATUS_OK);
    CHECK(app_motor_submit(20u) == APP_STATUS_NOT_READY);
    CHECK(app_motor_output_get(0, &output) != APP_STATUS_OK || !output.enabled);

    /* Timeout: old valid frame cannot be renewed by reading it. */
    CHECK(app_power_update(200u) == APP_STATUS_OFFLINE);
    CHECK(!feedback.source_online || app_power_get_feedback(&feedback) == APP_STATUS_OFFLINE);
    CHECK(app_motor_submit(200u) == APP_STATUS_NOT_READY);

    /* Recovery uses a new frame; old queued output must stay disabled. */
    frame(500, 5000, 210u);
    CHECK(app_power_update(210u) == APP_STATUS_OK);
    CHECK(app_motor_request(0, APP_MOTOR_CMD_SPEED_RAD_S, 1.0f) == APP_STATUS_OK);
    CHECK(app_motor_submit(210u) == APP_STATUS_OK);

    /* Negative values must be preserved, not clamped to zero. */
    frame((int16_t)-10, (int16_t)-10, 220u);
    CHECK(app_power_update(220u) == APP_STATUS_OK);
    (void)app_power_get_feedback(&feedback);
    CHECK(feedback.measured_chassis_power_w == -1.0f);
    CHECK(feedback.buffer_energy_j == -1.0f);

    /* Wrong bus is counted but must not replace the configured-bus snapshot. */
    {
        uint8_t wrong[4] = {0xF4, 0x01, 0x88, 0x13};
        app_power_on_can_frame(2u, APP_POWER_CAN_ID, wrong, 4u, 225u);
        CHECK(app_power_update(225u) == APP_STATUS_OK);
        (void)app_power_get_feedback(&feedback);
        CHECK(feedback.frame_arrival_ms == 220u);
        CHECK((feedback.error_flags & APP_POWER_ERR_BUS_MISMATCH) != 0u);
    }

    /* DLC=3 must not refresh the valid frame arrival. */
    {
        uint8_t bad[4] = {0xF4, 0x01, 0x88, 0x13};
        app_power_on_can_frame(1u, APP_POWER_CAN_ID, bad, 3u, 230u);
        CHECK(app_power_update(330u) == APP_STATUS_OFFLINE);
        (void)app_power_get_feedback(&feedback);
        CHECK(feedback.frame_arrival_ms == 220u);
        CHECK(feedback.invalid_frame_count >= 1u);
    }

    /* 32-bit time wrap: a recent frame across wrap remains fresh. */
    {
        frame(500, 5000, 0xFFFFFFFAu);
        CHECK(app_power_update(5u) == APP_STATUS_OK);
        CHECK(app_power_update(120u) == APP_STATUS_OFFLINE);
    }

    if (failures) {
        printf("%d power/motor contract checks failed\n", failures);
        return 1;
    }
    puts("PASS 0x31 parsing, timeout, power/energy policy and chassis output gate");
    return 0;
}
