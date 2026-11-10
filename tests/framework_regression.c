/* Native Windows regressions: actual application modules with explicit fixtures
 * for hardware/config/time. The production Cortex-M critical implementation is
 * checked by firmware compilation; host threads model interrupt/task exclusion. */
#include <windows.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "Common/app_critical.h"
#include "Common/app_time.h"
#include "Remote/app_remote.h"
#include "Attitude/app_attitude.h"
#include "Protection/app_protection.h"
#include "Motor/app_motor.h"
#include "app_config.h"
#ifdef TEST_ENGINEER
#include "Arm/app_arm.h"
#include "Arm/app_arm_task.h"
#else
#include "Gimbal/app_gimbal.h"
#endif
#include "Vehicle/app_vehicle.h"
#include "Remote/dbus.h"
#include "remote_uart.h"
#include "usart.h"

#define CHECK(expr) do { if (!(expr)) { \
    fprintf(stderr, "FAIL %s:%d: %s\n", __FILE__, __LINE__, #expr); \
    ExitProcess(1); } } while (0)

static CRITICAL_SECTION test_lock;
static _Thread_local uint32_t test_lock_depth;
static app_config_t fixture_config;
static unsigned int motor_requests;
static uint32_t mock_now_ms = 3000u;
static volatile LONG producer_done;
static volatile LONG controller_done;

/* ---- HAL UART/DMA stubs used by Bsp/remote_uart.c on the host ---------- */
UART_HandleTypeDef huart1, huart3;
DMA_HandleTypeDef hdma_usart3_rx;
static uint8_t *mock_rx_buffer;
static unsigned int mock_start_calls, mock_abort_calls, mock_irq_enabled = 3u;
static bool mock_fail_start;

HAL_StatusTypeDef HAL_UARTEx_ReceiveToIdle_DMA(UART_HandleTypeDef *uart, uint8_t *data,
                                               uint16_t size)
{
    CHECK(uart == &huart3 && size == 36u);
    mock_rx_buffer = data;
    mock_start_calls++;
    if (mock_fail_start) {
        mock_fail_start = false;
        return HAL_ERROR;
    }
    return HAL_OK;
}
HAL_StatusTypeDef HAL_UART_AbortReceive(UART_HandleTypeDef *uart)
{
    CHECK(uart == &huart3 && mock_irq_enabled == 0u);
    mock_abort_calls++;
    return HAL_OK;
}
void HAL_NVIC_DisableIRQ(int irq) { mock_irq_enabled &= ~(unsigned int)irq; }
void HAL_NVIC_EnableIRQ(int irq) { mock_irq_enabled |= (unsigned int)irq; }
void test_dma_disable_it(DMA_HandleTypeDef *dma, unsigned int flags)
{
    CHECK(dma == &hdma_usart3_rx && flags == DMA_IT_HT);
}
void test_uart_clear_error(UART_HandleTypeDef *uart) { CHECK(uart == &huart3); }

uint32_t app_critical_enter(void)
{
    const uint32_t previous = test_lock_depth;
    EnterCriticalSection(&test_lock);
    test_lock_depth++;
    return previous;
}
void app_critical_exit(uint32_t previous)
{
    CHECK(test_lock_depth == previous + 1u);
    test_lock_depth = previous;
    LeaveCriticalSection(&test_lock);
}

const app_config_t *app_config_get(void) { return &fixture_config; }
app_status_t app_config_status(void) { return APP_STATUS_OK; }
bool app_config_motor_motion_usable(int slot) { return slot >= 0 && slot < 9; }
const app_motor_config_t *app_config_motor(int slot)
{
    return (slot >= 0 && slot < 9) ? &fixture_config.motors[slot] : NULL;
}
int app_config_find_motor(app_motor_role_t role)
{
    for (int i = 0; i < 9; i++) {
        if (fixture_config.motors[i].role == role) { return i; }
    }
    return -1;
}
app_status_t app_motor_request(int slot, app_motor_cmd_mode_t mode, float value)
{
    CHECK(slot >= 0 && slot < 9);
    CHECK(mode == APP_MOTOR_CMD_POSITION_RAD);
    CHECK(isfinite(value));
    motor_requests++;
    return APP_STATUS_OK;
}
app_status_t app_motor_status(void) { return APP_STATUS_OK; }
app_status_t app_power_status(void) { return APP_STATUS_OK; }
uint32_t app_time_now_ms(void) { return mock_now_ms; }
uint32_t app_time_elapsed_ms(uint32_t now, uint32_t then) { return now - then; }
bool app_time_is_fresh(uint32_t now, uint32_t stamp, uint32_t timeout)
{
    return stamp != APP_TIME_NEVER && now - stamp <= timeout;
}

static void reset_fixture(void)
{
    memset(&fixture_config, 0, sizeof(fixture_config));
    fixture_config.remote.valid = true;
    fixture_config.remote.mapping_valid = true;
    fixture_config.remote.timeout_valid = true;
    fixture_config.remote.deadband_valid = true;
    fixture_config.remote.channel_map_valid = true;
    fixture_config.remote.switch_map_valid = true;
    fixture_config.remote.link_timeout_ms = 10000u;
    fixture_config.remote.channel_deadband = 0.03f;
    fixture_config.remote.channel_forward = 3u;
    fixture_config.remote.channel_turn = 2u;
    fixture_config.remote.channel_aux1 = 0u;
    fixture_config.remote.channel_aux2 = 1u;
    fixture_config.remote.sign_forward = 1;
    fixture_config.remote.sign_turn = -1;
    fixture_config.remote.sign_aux1 = -1;
    fixture_config.remote.sign_aux2 = 1;
    fixture_config.remote.gear_switch = 0u;
    fixture_config.remote.enable_switch = 1u;
    mock_now_ms = 3000u;
    fixture_config.gimbal.valid = true;
    fixture_config.gimbal.limits_valid = true;
    fixture_config.gimbal.yaw_min_rad = -2.0f;
    fixture_config.gimbal.yaw_max_rad = 2.0f;
    fixture_config.gimbal.pitch_min_rad = -2.0f;
    fixture_config.gimbal.pitch_max_rad = 2.0f;
    fixture_config.arm.valid = true;
    fixture_config.arm.limits_valid = true;
    for (int i = 0; i < 9; i++) {
        fixture_config.motors[i].role = (app_motor_role_t)(i + 1);
        fixture_config.motors[i].limit_valid = true;
        fixture_config.motors[i].limit_min_rad = -2.0f;
        fixture_config.motors[i].limit_max_rad = 2.0f;
    }
    for (int i = 0; i < 5; i++) {
        fixture_config.arm.joint_min_rad[i] = -2.0f;
        fixture_config.arm.joint_max_rad[i] = 2.0f;
    }
    motor_requests = 0;
}

static app_remote_input_t input_frame(bool second)
{
    return (app_remote_input_t){
        .sample_ms = second ? 2222u : 1111u,
        .forward = second ? 1.0f : -1.0f,
        .turn = second ? -1.0f : 0.0f,
        .aux1 = second ? 0.0f : 1.0f,
        .aux2 = second ? 1.0f : -1.0f,
        .mode = second ? APP_MODE_VEHICLE_UP : APP_MODE_CHASSIS
    };
}

static void test_remote_and_fault_recovery(void)
{
    app_remote_snapshot_t remote;
    const app_remote_input_t raw = input_frame(false);
    CHECK(app_remote_init(3000) == APP_STATUS_UNCONFIGURED);
    CHECK(app_remote_update(3000) == APP_STATUS_OFFLINE);
    CHECK(app_remote_publish(NULL) == APP_STATUS_INVALID_PARAM);
    CHECK(app_remote_get_snapshot(NULL) == APP_STATUS_INVALID_PARAM);
    CHECK(app_remote_publish(&raw) == APP_STATUS_OK);
    CHECK(app_remote_update(3000) == APP_STATUS_OK);
    CHECK(app_remote_get_snapshot(&remote) == APP_STATUS_OK && remote.online);
    CHECK(remote.mode == APP_MODE_CHASSIS && !remote.arm_request);

    CHECK(app_protection_init(3000) == APP_STATUS_NOT_READY);
    app_protection_set_arm_request(true);
    app_protection_set_mode_active(true);
    CHECK(app_protection_evaluate(3000) == APP_STATUS_OK);
    CHECK(app_protection_output_allowed());
    app_protection_raise(APP_FAULT_MANUAL | APP_FAULT_DEVICE);
    CHECK(!app_protection_output_allowed());
    CHECK(!app_protection_clear_latched());
    app_protection_resolve(APP_FAULT_MANUAL);
    CHECK(app_protection_report()->active_faults == APP_FAULT_DEVICE);
    CHECK(!app_protection_clear_latched());
    app_protection_resolve(APP_FAULT_DEVICE);
    CHECK(app_protection_report()->active_faults == APP_FAULT_NONE);
    CHECK(app_protection_evaluate(3001) == APP_STATUS_FAULT);
    CHECK(!app_protection_output_allowed());
    CHECK(app_protection_clear_latched());
    CHECK(app_protection_evaluate(3002) == APP_STATUS_NOT_READY);
    CHECK(!app_protection_output_allowed());
    app_protection_set_arm_request(true);
    app_protection_set_mode_active(true);
    CHECK(app_protection_evaluate(3003) == APP_STATUS_OK);

    CHECK(app_remote_update(20000) == APP_STATUS_OFFLINE);
    CHECK(app_remote_get_snapshot(&remote) == APP_STATUS_OFFLINE);
    CHECK(!remote.online && remote.forward == 0.0f);
    CHECK(app_protection_evaluate(20000) == APP_STATUS_NOT_READY);
    CHECK(!app_protection_output_allowed());
    puts("PASS remote validity, timeout and latched-fault recovery");
}

static void test_input_routes(void)
{
    app_remote_input_t input = {.mode = APP_MODE_CHASSIS, .sample_ms = 120};
    app_remote_snapshot_t out;
    const app_mode_t modes[] = {APP_MODE_CHASSIS, APP_MODE_VEHICLE_UP, APP_MODE_VEHICLE_DOWN};
    (void)app_remote_init(100);
    for (unsigned int i = 0; i < 3; i++) {
        input.mode = modes[i];
        input.arm_request = true;
        CHECK(app_remote_publish(&input) == APP_STATUS_OK);
        CHECK(app_remote_update(119) == APP_STATUS_OK);
        CHECK(app_remote_get_snapshot(&out) == APP_STATUS_OK);
        CHECK(out.mode == modes[i] && out.arm_request);
    }
    input.mode = APP_MODE_DISABLED;
    CHECK(app_remote_publish(&input) == APP_STATUS_OK);
    (void)app_remote_update(120);
    (void)app_remote_get_snapshot(&out);
    CHECK(!out.arm_request && out.mode == APP_MODE_DISABLED);
    input.mode = (app_mode_t)99;
    CHECK(app_remote_publish(&input) == APP_STATUS_INVALID_PARAM);
    CHECK(app_remote_get_snapshot(&out) == APP_STATUS_OFFLINE && !out.online);
    input.mode = APP_MODE_CHASSIS;
    input.turn = NAN;
    CHECK(app_remote_publish(&input) == APP_STATUS_INVALID_PARAM);
    input.turn = 2.0f;
    CHECK(app_remote_publish(&input) == APP_STATUS_INVALID_PARAM);
    input.turn = 0.0f;
    fixture_config.remote.mapping_valid = false;
    CHECK(app_remote_publish(&input) == APP_STATUS_UNCONFIGURED);
    CHECK(app_remote_update(120) == APP_STATUS_UNCONFIGURED);
    fixture_config.remote.mapping_valid = true;
    puts("PASS normalized input: middle/up/down routing, explicit enable and invalid data");
}

static void test_attitude_validity(void)
{
    app_attitude_snapshot_t sample = {0}, out;
    sample.timestamp_ms = 1234;
    CHECK(app_attitude_init(1) == APP_STATUS_NOT_READY);
    CHECK(app_attitude_ingest(&sample) == APP_STATUS_NOT_READY);
    CHECK(app_attitude_get_snapshot(&out) == APP_STATUS_NOT_READY && !out.valid);
    CHECK(app_attitude_update(9999) == APP_STATUS_NOT_READY);
    CHECK(app_attitude_get_snapshot(&out) == APP_STATUS_NOT_READY);
    CHECK(out.timestamp_ms == 1234);
    sample.valid = true;
    CHECK(app_attitude_ingest(&sample) == APP_STATUS_OK);
    sample.valid = false;
    CHECK(app_attitude_ingest(&sample) == APP_STATUS_NOT_READY);
    CHECK(app_attitude_get_snapshot(&out) == APP_STATUS_NOT_READY && !out.valid);
    sample.valid = true;
    sample.yaw_rad = NAN;
    CHECK(app_attitude_ingest(&sample) == APP_STATUS_INVALID_PARAM);
    CHECK(app_attitude_get_snapshot(&out) == APP_STATUS_INVALID_PARAM && !out.valid);
    sample.yaw_rad = INFINITY;
    CHECK(app_attitude_ingest(&sample) == APP_STATUS_INVALID_PARAM);
    CHECK(app_attitude_ingest(NULL) == APP_STATUS_INVALID_PARAM);
    puts("PASS invalid IMU samples, non-finite samples and timestamp preservation");
}

static void seed_vehicle_target(void)
{
#ifdef TEST_ENGINEER
    CHECK(app_arm_set_joint_target(APP_ARM_JOINT_YAW, 0.5f) == APP_STATUS_OK);
    (void)app_arm_task_request(APP_ARM_TASK_GRAB_DART);
#else
    CHECK(app_gimbal_set_target(APP_GIMBAL_AXIS_YAW, 0.5f) == APP_STATUS_OK);
#endif
}
static void check_vehicle_target_cleared(void)
{
    float target;
#ifdef TEST_ENGINEER
    app_arm_task_id_t task;
    CHECK(app_arm_get_joint_target(APP_ARM_JOINT_YAW, &target) == APP_STATUS_NOT_READY);
    CHECK(app_arm_task_get_state(&task) == APP_ARM_TASK_STATE_IDLE);
    CHECK(task == APP_ARM_TASK_NONE);
#else
    CHECK(app_gimbal_get_target(APP_GIMBAL_AXIS_YAW, &target) == APP_STATUS_NOT_READY);
#endif
}
static void test_vehicle_branches(void)
{
    app_vehicle_report_t report;
    app_remote_snapshot_t remote = {.online = true, .mode = APP_MODE_VEHICLE_UP};
    const app_mode_t rejected[] = {APP_MODE_CHASSIS, APP_MODE_DISABLED, APP_MODE_UNKNOWN};
    (void)app_vehicle_init(3000);
    CHECK(app_vehicle_update(&remote, true, 3000) == APP_STATUS_NOT_IMPLEMENTED);
    seed_vehicle_target();
    remote.mode = APP_MODE_VEHICLE_DOWN;
    CHECK(app_vehicle_update(&remote, true, 3001) == APP_STATUS_NOT_IMPLEMENTED);
    CHECK(app_vehicle_get_report(&report) == APP_STATUS_OK);
    CHECK(report.mode == APP_MODE_VEHICLE_DOWN);
    check_vehicle_target_cleared();
    seed_vehicle_target();
    remote.mode = APP_MODE_VEHICLE_UP;
    CHECK(app_vehicle_update(&remote, true, 3002) == APP_STATUS_NOT_IMPLEMENTED);
    check_vehicle_target_cleared();
    for (unsigned int i = 0; i < 6; i++) {
        remote.online = true;
        remote.mode = APP_MODE_VEHICLE_UP;
        (void)app_vehicle_update(&remote, true, 3003);
        seed_vehicle_target();
        if (i < 3) { remote.mode = rejected[i]; }
        if (i == 3) { remote.online = false; }
        CHECK(app_vehicle_update(i == 5 ? NULL : &remote, i != 4, 3004) == APP_STATUS_NOT_READY);
        check_vehicle_target_cleared();
    }
    CHECK(motor_requests == 0); /* both extension points remain honest no-motion stubs */
    puts("PASS vehicle-local upper/lower placeholders, transition cleanup and middle-mode isolation");
}

static void check_remote_frame(void)
{
    app_remote_snapshot_t out;
    if (app_remote_get_snapshot(&out) != APP_STATUS_OK) { return; }
    CHECK(out.online);
    if (out.sample_ms == 1111u) {
        CHECK(out.forward == -1 && out.turn == 0 && out.aux1 == 1 && out.aux2 == -1);
        CHECK(out.mode == APP_MODE_CHASSIS);
    } else {
        CHECK(out.sample_ms == 2222u);
        CHECK(out.forward == 1 && out.turn == -1 && out.aux1 == 0 && out.aux2 == 1);
        CHECK(out.mode == APP_MODE_VEHICLE_UP);
    }
}

static void check_attitude_frame(void)
{
    app_attitude_snapshot_t out;
    if (app_attitude_get_snapshot(&out) != APP_STATUS_OK) { return; }
    const float base = out.timestamp_ms == 1111u ? 1.0f : 10.0f;
    CHECK(out.timestamp_ms == 1111u || out.timestamp_ms == 2222u);
    CHECK(out.valid && out.roll_rad == base && out.pitch_rad == base + 1 &&
          out.yaw_rad == base + 2 && out.roll_rate_rad_s == base + 3 &&
          out.pitch_rate_rad_s == base + 4 && out.yaw_rate_rad_s == base + 5);
}

static void check_aux_frame(void)
{
    app_remote_aux_snapshot_t aux;
    if (app_remote_get_aux_snapshot(&aux) != APP_STATUS_OK) { return; }
    CHECK(aux.valid);
    CHECK(aux.timestamp_ms == 1111u || aux.timestamp_ms == 2222u);
    const bool second = aux.timestamp_ms == 2222u;
    CHECK(aux.channel[0] == (second ? 1684 : 364));
    CHECK(aux.channel[1] == (second ? 364 : 1024));
    CHECK(aux.channel[2] == 1024);
    CHECK(aux.channel[3] == (second ? 1024 : 1684));
    CHECK(aux.switch_s1 == (second ? DBUS_SWITCH_UP : DBUS_SWITCH_MID));
    CHECK(aux.switch_s2 == DBUS_SWITCH_MID);
    CHECK(aux.mouse_x == (second ? 0x2222 : 0x1111));
    CHECK(aux.mouse_press_left == (second ? 1u : 0u));
    CHECK(aux.keyboard == (second ? 0x8005u : 0x0500u));
    CHECK(aux.mouse_y == 0 && aux.mouse_z == 0 && aux.mouse_press_right == 0u);
}

static DWORD WINAPI produce_frames(LPVOID unused)
{
    (void)unused;
    for (unsigned int i = 0; i < 40000; i++) {
        const bool second = (i & 1u) != 0;
        const app_remote_input_t raw = input_frame(second);
        const app_remote_aux_snapshot_t aux = {
            .valid = true,
            .timestamp_ms = raw.sample_ms,
            .channel = {second ? 1684 : 364, second ? 364 : 1024, 1024,
                        second ? 1024 : 1684},
            .switch_s1 = second ? DBUS_SWITCH_UP : DBUS_SWITCH_MID,
            .switch_s2 = DBUS_SWITCH_MID,
            .mouse_x = second ? 0x2222 : 0x1111,
            .mouse_press_left = second ? 1u : 0u,
            .keyboard = second ? 0x8005u : 0x0500u,
        };
        const float base = second ? 10.0f : 1.0f;
        const app_attitude_snapshot_t attitude = {
            .roll_rad = base, .pitch_rad = base + 1, .yaw_rad = base + 2,
            .roll_rate_rad_s = base + 3, .pitch_rate_rad_s = base + 4,
            .yaw_rate_rad_s = base + 5, .valid = true, .timestamp_ms = raw.sample_ms};
        CHECK(app_remote_publish_frame(&raw, &aux) == APP_STATUS_OK);
        CHECK(app_attitude_ingest(&attitude) == APP_STATUS_OK);
        if (i % 32u == 0) { SwitchToThread(); }
    }
    InterlockedExchange(&producer_done, 1);
    return 0;
}

static DWORD WINAPI control_frames(LPVOID unused)
{
    (void)unused;
    unsigned int cycles = 0;
    do {
        (void)app_remote_update(3000);
        check_remote_frame();
        check_aux_frame();
        check_attitude_frame();
        cycles++;
    } while (!InterlockedCompareExchange(&producer_done, 0, 0) || cycles < 40000u);
    InterlockedExchange(&controller_done, 1);
    return 0;
}

static void test_concurrent_snapshots(void)
{
    (void)app_remote_init(3000);
    (void)app_attitude_init(3000);
    producer_done = controller_done = 0;
    const HANDLE producer = CreateThread(NULL, 0, produce_frames, NULL, 0, NULL);
    const HANDLE controller = CreateThread(NULL, 0, control_frames, NULL, 0, NULL);
    CHECK(producer != NULL && controller != NULL);
    do {
        check_remote_frame();
        check_aux_frame();
        check_attitude_frame();
    } while (!InterlockedCompareExchange(&controller_done, 0, 0));
    CHECK(WaitForSingleObject(producer, 10000) == WAIT_OBJECT_0);
    CHECK(WaitForSingleObject(controller, 10000) == WAIT_OBJECT_0);
    CloseHandle(producer);
    CloseHandle(controller);
    puts("PASS concurrent producer/controller/reader: 40000 frames, no mixed snapshots");
}

/* ------------------------------------------------------------------------- */
/* Real DBUS frame path: raw 18-byte frame -> decode -> map -> publish ->     */
/* snapshot -> three-gear dispatch. The abstract publisher tests above are    */
/* NOT evidence for the physical receive chain; these are.                    */
/* ------------------------------------------------------------------------- */

static void pack_dbus(uint8_t frame[DBUS_FRAME_SIZE], const int16_t channel[4],
                      uint8_t s1, uint8_t s2)
{
    uint64_t bits = 0;
    memset(frame, 0, DBUS_FRAME_SIZE);
    for (unsigned int i = 0; i < 4u; i++) {
        bits |= (uint64_t)((uint16_t)channel[i] & 0x7FFu) << (11u * i);
    }
    bits |= (uint64_t)(s1 & 0x3u) << 44;
    bits |= (uint64_t)(s2 & 0x3u) << 46;
    for (unsigned int i = 0; i < 6u; i++) {
        frame[i] = (uint8_t)(bits >> (8u * i));
    }
}

static void stage_dbus(const uint8_t frame[DBUS_FRAME_SIZE], uint16_t size,
                       uint32_t arrival_ms)
{
    CHECK(mock_rx_buffer != NULL);
    mock_now_ms = arrival_ms;
    memset(mock_rx_buffer, 0, 36u);
    (void)memcpy(mock_rx_buffer, frame, DBUS_FRAME_SIZE);
    HAL_UARTEx_RxEventCallback(&huart3, size);
}

static void feed_dbus(const uint8_t frame[DBUS_FRAME_SIZE], uint16_t size, uint32_t now_ms)
{
    stage_dbus(frame, size, now_ms);
    remote_uart_service(now_ms);
}

static void test_dbus_protocol(void)
{
    const int16_t channel[4] = {1684, 364, 1024, 1200};
    app_dbus_frame_t decoded;
    uint8_t frame[DBUS_FRAME_SIZE];

    pack_dbus(frame, channel, DBUS_SWITCH_UP, DBUS_SWITCH_DOWN);
    frame[6] = 0x34;
    frame[7] = 0x12;
    frame[12] = 1;
    frame[13] = 0;
    frame[14] = 0x05;
    frame[15] = 0x80;
    CHECK(dbus_decode(frame, DBUS_FRAME_SIZE, 777, &decoded) == APP_STATUS_OK);
    for (int i = 0; i < 4; i++) {
        CHECK(decoded.channel[i] == channel[i]);
    }
    CHECK(decoded.switch_s1 == DBUS_SWITCH_UP && decoded.switch_s2 == DBUS_SWITCH_DOWN);
    CHECK(decoded.mouse_x == 0x1234 && decoded.mouse_press_left == 1u &&
          decoded.mouse_press_right == 0u && decoded.keyboard == 0x8005u);
    CHECK(decoded.sample_ms == 777u);

    CHECK(dbus_decode(frame, 17u, 777, &decoded) == APP_STATUS_INVALID_PARAM);
    CHECK(dbus_decode(frame, 36u, 777, &decoded) == APP_STATUS_INVALID_PARAM);
    CHECK(dbus_decode(NULL, DBUS_FRAME_SIZE, 777, &decoded) == APP_STATUS_INVALID_PARAM);
    CHECK(dbus_decode(frame, DBUS_FRAME_SIZE, 777, NULL) == APP_STATUS_INVALID_PARAM);

    frame[12] = 2u; /* mouse button must be 0/1 */
    CHECK(dbus_decode(frame, DBUS_FRAME_SIZE, 777, &decoded) == APP_STATUS_INVALID_PARAM);
    frame[12] = 0u;
    pack_dbus(frame, channel, 0u, DBUS_SWITCH_UP); /* invalid switch code */
    CHECK(dbus_decode(frame, DBUS_FRAME_SIZE, 777, &decoded) == APP_STATUS_INVALID_PARAM);
    const int16_t bad_channel[4] = {1685, 1024, 1024, 1024};
    pack_dbus(frame, bad_channel, DBUS_SWITCH_MID, DBUS_SWITCH_UP);
    CHECK(dbus_decode(frame, DBUS_FRAME_SIZE, 777, &decoded) == APP_STATUS_INVALID_PARAM);
    memset(frame, 0, DBUS_FRAME_SIZE);
    CHECK(dbus_decode(frame, DBUS_FRAME_SIZE, 777, &decoded) == APP_STATUS_INVALID_PARAM);
    puts("PASS DBUS 18-byte frame: channels, S1/S2, mouse, keyboard and invalid frames");
}

static void test_remote_receiver_chain(void)
{
    const int16_t mid[4] = {DBUS_CHANNEL_MID, DBUS_CHANNEL_MID, DBUS_CHANNEL_MID,
                            DBUS_CHANNEL_MID};
    const int16_t axes[4] = {1684, 364, 1024, 1684};
    const int16_t small[4] = {1024, 1024, 1024, 1030};
    app_remote_snapshot_t snapshot;
    uint8_t frame[DBUS_FRAME_SIZE];

    mock_start_calls = 0;
    mock_abort_calls = 0;
    mock_irq_enabled = 3u;
    mock_fail_start = false;
    fixture_config.remote.link_timeout_ms = 42u;
    (void)app_remote_init(1000);
    remote_uart_init(1000);
    CHECK(mock_start_calls == 1u && mock_abort_calls == 1u && mock_irq_enabled == 3u);

    /* Middle gear -> shared chassis; S2 up -> explicit enable. */
    pack_dbus(frame, mid, DBUS_SWITCH_MID, DBUS_SWITCH_UP);
    feed_dbus(frame, DBUS_FRAME_SIZE, 1000);
    CHECK(app_remote_update(1000) == APP_STATUS_OK);
    CHECK(app_remote_get_snapshot(&snapshot) == APP_STATUS_OK);
    CHECK(snapshot.online && snapshot.mode == APP_MODE_CHASSIS && snapshot.arm_request &&
          !snapshot.reset_request);
    CHECK(app_vehicle_update(&snapshot, true, 1000) == APP_STATUS_NOT_READY);

    /* Up gear -> this vehicle's own up entry. */
    pack_dbus(frame, mid, DBUS_SWITCH_UP, DBUS_SWITCH_UP);
    feed_dbus(frame, DBUS_FRAME_SIZE, 1010);
    CHECK(app_remote_update(1010) == APP_STATUS_OK);
    (void)app_remote_get_snapshot(&snapshot);
    CHECK(snapshot.mode == APP_MODE_VEHICLE_UP && snapshot.arm_request);
    CHECK(app_vehicle_update(&snapshot, true, 1010) == APP_STATUS_NOT_IMPLEMENTED);

    /* Down gear + S2 down: down entry, disabled and reset request. */
    pack_dbus(frame, mid, DBUS_SWITCH_DOWN, DBUS_SWITCH_DOWN);
    feed_dbus(frame, DBUS_FRAME_SIZE, 1020);
    CHECK(app_remote_update(1020) == APP_STATUS_OK);
    (void)app_remote_get_snapshot(&snapshot);
    CHECK(snapshot.mode == APP_MODE_VEHICLE_DOWN && !snapshot.arm_request &&
          snapshot.reset_request);
    CHECK(app_vehicle_update(&snapshot, true, 1020) == APP_STATUS_NOT_IMPLEMENTED);

    /* Axes: forward=ch3, turn=-ch2, aux1=-ch0, aux2=ch1, deadband 0.03. */
    pack_dbus(frame, axes, DBUS_SWITCH_MID, DBUS_SWITCH_UP);
    feed_dbus(frame, DBUS_FRAME_SIZE, 1030);
    CHECK(app_remote_update(1030) == APP_STATUS_OK);
    (void)app_remote_get_snapshot(&snapshot);
    CHECK(fabsf(snapshot.forward - 1.0f) < 0.01f);
    CHECK(fabsf(snapshot.turn) < 0.01f);
    CHECK(fabsf(snapshot.aux1 + 1.0f) < 0.01f);
    CHECK(fabsf(snapshot.aux2 + 1.0f) < 0.01f);

    pack_dbus(frame, small, DBUS_SWITCH_MID, DBUS_SWITCH_UP);
    feed_dbus(frame, DBUS_FRAME_SIZE, 1040);
    CHECK(app_remote_update(1040) == APP_STATUS_OK);
    (void)app_remote_get_snapshot(&snapshot);
    CHECK(snapshot.forward == 0.0f);

    /* Link timeout: last frame at 1040, 42 ms threshold. */
    CHECK(app_remote_update(1082) == APP_STATUS_OK);
    CHECK(app_remote_update(1083) == APP_STATUS_OFFLINE);
    (void)app_remote_get_snapshot(&snapshot);
    CHECK(!snapshot.online && snapshot.mode == APP_MODE_UNKNOWN);

    /* Short/merged frame revokes input immediately. */
    pack_dbus(frame, mid, DBUS_SWITCH_MID, DBUS_SWITCH_UP);
    feed_dbus(frame, 17u, 2000);
    CHECK(app_remote_update(2000) == APP_STATUS_OFFLINE);
    (void)app_remote_get_snapshot(&snapshot);
    CHECK(!snapshot.online);

    /* Invalid switch field revokes input. */
    frame[5] = 0u; /* clear S1/S2 bits -> invalid switch codes */
    feed_dbus(frame, DBUS_FRAME_SIZE, 2010);
    CHECK(app_remote_update(2010) == APP_STATUS_OFFLINE);

    pack_dbus(frame, mid, DBUS_SWITCH_MID, DBUS_SWITCH_UP);
    feed_dbus(frame, DBUS_FRAME_SIZE, 2020);
    CHECK(app_remote_update(2020) == APP_STATUS_OK);
    (void)app_remote_get_snapshot(&snapshot);
    CHECK(snapshot.online && snapshot.mode == APP_MODE_CHASSIS);

    fixture_config.remote.link_timeout_ms = 10000u;
    puts("PASS DBUS receiver chain: online, timeout, invalid frames, axes and three-gear dispatch");
}

static void test_uart_recovery(void)
{
    remote_uart_stats_t stats;
    const int16_t mid[4] = {DBUS_CHANNEL_MID, DBUS_CHANNEL_MID, DBUS_CHANNEL_MID,
                            DBUS_CHANNEL_MID};
    app_remote_snapshot_t snapshot;
    uint8_t frame[DBUS_FRAME_SIZE];

    mock_start_calls = 0;
    mock_abort_calls = 0;
    mock_irq_enabled = 3u;
    mock_fail_start = false;
    fixture_config.remote.link_timeout_ms = 10000u;
    (void)app_remote_init(3000);
    remote_uart_init(3000);
    CHECK(mock_start_calls == 1u && mock_abort_calls == 1u && mock_irq_enabled == 3u);

    pack_dbus(frame, mid, DBUS_SWITCH_MID, DBUS_SWITCH_UP);
    feed_dbus(frame, DBUS_FRAME_SIZE, 3001);
    CHECK(mock_start_calls == 2u);
    CHECK(app_remote_update(3001) == APP_STATUS_OK);

    HAL_UARTEx_RxEventCallback(&huart3, 17u); /* partial frame */
    CHECK(mock_start_calls == 3u);
    remote_uart_service(3002);
    (void)app_remote_get_snapshot(&snapshot);
    CHECK(!snapshot.online);

    HAL_UART_ErrorCallback(&huart3);
    remote_uart_service(3003);
    CHECK(mock_abort_calls == 2u && mock_irq_enabled == 3u);

    mock_fail_start = true;
    HAL_UART_ErrorCallback(&huart3);
    remote_uart_service(3004); /* restart fails, retry flag kept */
    CHECK(mock_irq_enabled == 3u);
    remote_uart_service(3005); /* retry succeeds */
    CHECK(mock_irq_enabled == 3u);
    const unsigned int before_stall = mock_start_calls;
    remote_uart_service(4006); /* one second without receive events */
    CHECK(mock_start_calls == before_stall + 1u);

    CHECK(remote_uart_get_stats(&stats) == APP_STATUS_OK);
    CHECK(stats.valid_frames == 1u && stats.invalid_frames == 1u &&
          stats.stale_frames == 0u && stats.uart_errors == 2u &&
          stats.start_failures == 1u && stats.restarts >= 5u);
    CHECK(remote_uart_get_stats(NULL) == APP_STATUS_INVALID_PARAM);
    HAL_UART_ErrorCallback(&huart1); /* other UART ignored */
    CHECK(remote_uart_get_stats(&stats) == APP_STATUS_OK && stats.uart_errors == 2u);
    puts("PASS UART DMA/IDLE path: partial frame, error, failed restart retry, stall recovery");
}

static void test_receive_timestamp_and_aux(void)
{
    const int16_t channel[4] = {1684, 364, 1024, 1200};
    app_remote_snapshot_t snapshot;
    app_remote_aux_snapshot_t aux;
    remote_uart_stats_t stats;
    uint8_t frame[DBUS_FRAME_SIZE];

    mock_start_calls = 0;
    mock_abort_calls = 0;
    mock_irq_enabled = 3u;
    mock_fail_start = false;
    fixture_config.remote.link_timeout_ms = 42u;
    (void)app_remote_init(1000);
    remote_uart_init(1000);
    CHECK(app_remote_get_aux_snapshot(NULL) == APP_STATUS_INVALID_PARAM);

    pack_dbus(frame, channel, DBUS_SWITCH_DOWN, DBUS_SWITCH_MID);
    frame[6] = 0x34;
    frame[7] = 0x12;
    frame[12] = 1;
    frame[13] = 0;
    frame[14] = 0x05;
    frame[15] = 0x80;

    /* 正常消费：到达时刻与处理时刻相同，原始字段完整可读。 */
    feed_dbus(frame, DBUS_FRAME_SIZE, 1000);
    CHECK(app_remote_update(1000) == APP_STATUS_OK);
    CHECK(app_remote_get_aux_snapshot(&aux) == APP_STATUS_OK);
    CHECK(aux.valid && aux.timestamp_ms == 1000u);
    for (int i = 0; i < 4; i++) {
        CHECK(aux.channel[i] == channel[i]);
    }
    CHECK(aux.switch_s1 == DBUS_SWITCH_DOWN && aux.switch_s2 == DBUS_SWITCH_MID);
    CHECK(aux.mouse_x == 0x1234 && aux.mouse_press_left == 1u &&
          aux.mouse_press_right == 0u && aux.keyboard == 0x8005u);
    CHECK(aux.mouse_y == 0 && aux.mouse_z == 0);

    /* 延迟 30 ms（未超时）：仍在线，sample_ms/timestamp 是到达时刻。 */
    stage_dbus(frame, DBUS_FRAME_SIZE, 2000);
    remote_uart_service(2030);
    CHECK(app_remote_update(2030) == APP_STATUS_OK);
    (void)app_remote_get_snapshot(&snapshot);
    CHECK(snapshot.online && snapshot.sample_ms == 2000u);
    (void)app_remote_get_aux_snapshot(&aux);
    CHECK(aux.valid && aux.timestamp_ms == 2000u);

    /* 延迟 100 ms（>42 ms）：过期帧不得重新判在线，原始状态清零。 */
    stage_dbus(frame, DBUS_FRAME_SIZE, 3000);
    remote_uart_service(3100);
    CHECK(app_remote_update(3100) == APP_STATUS_OFFLINE);
    (void)app_remote_get_snapshot(&snapshot);
    CHECK(!snapshot.online && snapshot.mode == APP_MODE_UNKNOWN);
    (void)app_remote_get_aux_snapshot(&aux);
    CHECK(!aux.valid && aux.timestamp_ms == APP_TIME_NEVER);
    CHECK(aux.channel[0] == 0 && aux.channel[1] == 0 && aux.switch_s1 == 0u &&
          aux.switch_s2 == 0u && aux.keyboard == 0u && aux.mouse_press_left == 0u &&
          aux.mouse_press_right == 0u);
    CHECK(remote_uart_get_stats(&stats) == APP_STATUS_OK && stats.stale_frames == 1u);

    /* 时间回绕：到达 0xFFFFFFF0、处理 0x10，差 32 ms，仍应在线。 */
    stage_dbus(frame, DBUS_FRAME_SIZE, 0xFFFFFFF0u);
    remote_uart_service(0x10u);
    CHECK(app_remote_update(0x10u) == APP_STATUS_OK);
    (void)app_remote_get_snapshot(&snapshot);
    CHECK(snapshot.online && snapshot.sample_ms == 0xFFFFFFF0u);
    (void)app_remote_get_aux_snapshot(&aux);
    CHECK(aux.valid && aux.timestamp_ms == 0xFFFFFFF0u);

    /* 回绕后差 64 ms：丢弃并撤销。 */
    stage_dbus(frame, DBUS_FRAME_SIZE, 0xFFFFFFF0u);
    remote_uart_service(0x30u);
    CHECK(app_remote_update(0x30u) == APP_STATUS_OFFLINE);

    /* 非法帧后原始快照同样失效，不残留按键。 */
    stage_dbus(frame, DBUS_FRAME_SIZE, 4000);
    remote_uart_service(4000);
    CHECK(app_remote_update(4000) == APP_STATUS_OK);
    stage_dbus(frame, 17u, 4010);
    remote_uart_service(4010);
    (void)app_remote_get_aux_snapshot(&aux);
    CHECK(!aux.valid && aux.keyboard == 0u && aux.mouse_press_left == 0u);

    fixture_config.remote.link_timeout_ms = 10000u;
    puts("PASS arrival-time freshness, delayed-frame rejection, wrap-around and aux snapshot");
}

int main(void)
{
    InitializeCriticalSection(&test_lock);
    reset_fixture();
    test_remote_and_fault_recovery();
    test_input_routes();
    test_attitude_validity();
    test_vehicle_branches();
    test_dbus_protocol();
    test_remote_receiver_chain();
    test_uart_recovery();
    test_receive_timestamp_and_aux();
    test_concurrent_snapshots();
    CHECK(test_lock_depth == 0);
    DeleteCriticalSection(&test_lock);
    return 0;
}
