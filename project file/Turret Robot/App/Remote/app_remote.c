/* 逻辑输入快照、原始状态快照与三档模式。生产调用链：
 * USART3 + DMA/IDLE -> Bsp/remote_uart -> dbus_decode -> dbus_to_input ->
 * app_remote_publish_frame -> app_remote_update/get_snapshot/get_aux_snapshot。 */
#include "Remote/app_remote.h"
#include <math.h>
#include <stddef.h>
#include "Common/app_critical.h"
#include "app_config.h"

static app_remote_input_t g_input;
static app_remote_aux_snapshot_t g_aux;
static app_remote_snapshot_t g_snapshot;
static bool g_received;
static app_status_t g_status = APP_STATUS_UNCONFIGURED;

static bool valid_mode(app_mode_t mode)
{
    return mode == APP_MODE_DISABLED || mode == APP_MODE_CHASSIS ||
           mode == APP_MODE_VEHICLE_UP || mode == APP_MODE_VEHICLE_DOWN;
}
static bool valid_axis(float axis)
{
    return isfinite(axis) && axis >= -1.0f && axis <= 1.0f;
}

/* Caller owns the critical section. Invalidating the main input also clears the
 * raw/protocol snapshot so no pressed key/switch state survives a lost link. */
static void clear_aux(void)
{
    g_aux = (app_remote_aux_snapshot_t){0};
    g_aux.timestamp_ms = APP_TIME_NEVER;
}

/* Caller owns the critical section. */
static void unavailable(app_status_t status, uint32_t now_ms)
{
    g_snapshot = (app_remote_snapshot_t){0};
    g_snapshot.mode = APP_MODE_UNKNOWN;
    g_snapshot.sample_ms = g_received ? g_input.sample_ms : APP_TIME_NEVER;
    g_snapshot.timestamp_ms = now_ms;
    clear_aux();
    g_status = status;
}

app_status_t app_remote_init(uint32_t now_ms)
{
    const uint32_t key = app_critical_enter();
    g_input = (app_remote_input_t){0};
    g_received = false;
    unavailable(APP_STATUS_UNCONFIGURED, now_ms);
    app_critical_exit(key);
    return APP_STATUS_UNCONFIGURED;
}

void app_remote_invalidate(uint32_t now_ms)
{
    const uint32_t key = app_critical_enter();
    g_received = false;
    unavailable(APP_STATUS_OFFLINE, now_ms);
    app_critical_exit(key);
}

app_status_t app_remote_publish_frame(const app_remote_input_t *input,
                                      const app_remote_aux_snapshot_t *aux)
{
    if (input == NULL) {
        return APP_STATUS_INVALID_PARAM;
    }
    if (!valid_mode(input->mode) || !valid_axis(input->forward) ||
        !valid_axis(input->turn) || !valid_axis(input->aux1) || !valid_axis(input->aux2)) {
        app_remote_invalidate(input->sample_ms);
        return APP_STATUS_INVALID_PARAM;
    }

    const uint32_t key = app_critical_enter();
    g_input = *input;
    g_received = true;
    if (aux != NULL) {
        g_aux = *aux;
        /* 输入与原始字段必须来自同一帧：统一用输入帧的到达时刻。 */
        g_aux.timestamp_ms = input->sample_ms;
    } else {
        clear_aux();
    }
    app_critical_exit(key);

    const app_remote_config_t *config = &app_config_get()->remote;
    return (config->valid && config->mapping_valid) ? APP_STATUS_OK : APP_STATUS_UNCONFIGURED;
}

app_status_t app_remote_publish(const app_remote_input_t *input)
{
    return app_remote_publish_frame(input, NULL);
}

app_status_t app_remote_update(uint32_t now_ms)
{
    const app_remote_config_t *config = &app_config_get()->remote;
    const uint32_t key = app_critical_enter();
    if (!config->valid || !config->mapping_valid || !config->timeout_valid ||
        config->link_timeout_ms == 0u) {
        unavailable(APP_STATUS_UNCONFIGURED, now_ms);
    } else {
        /* 输入 sample_ms 是接收回调保存的帧到达时刻；ISR 可能比周期起点更新。 */
        const int32_t age = (int32_t)(now_ms - g_input.sample_ms);
        if (!g_received || (age >= 0 && (uint32_t)age > config->link_timeout_ms)) {
            unavailable(APP_STATUS_OFFLINE, now_ms);
        } else {
            g_snapshot.forward = g_input.forward;
            g_snapshot.turn = g_input.turn;
            g_snapshot.aux1 = g_input.aux1;
            g_snapshot.aux2 = g_input.aux2;
            g_snapshot.mode = g_input.mode;
            g_snapshot.arm_request = g_input.arm_request &&
                                     g_input.mode != APP_MODE_DISABLED;
            g_snapshot.reset_request = g_input.reset_request;
            g_snapshot.online = true;
            g_snapshot.sample_ms = g_input.sample_ms;
            g_snapshot.timestamp_ms = now_ms;
            /* 原始快照必须与当前输入属于同一帧，否则视为失效。 */
            if (!g_aux.valid || g_aux.timestamp_ms != g_input.sample_ms) {
                clear_aux();
            }
            g_status = APP_STATUS_OK;
        }
    }
    const app_status_t status = g_status;
    app_critical_exit(key);
    return status;
}

app_status_t app_remote_get_snapshot(app_remote_snapshot_t *out)
{
    if (out == NULL) {
        return APP_STATUS_INVALID_PARAM;
    }
    const uint32_t key = app_critical_enter();
    *out = g_snapshot;
    const app_status_t status = g_status;
    app_critical_exit(key);
    return status;
}

app_status_t app_remote_get_aux_snapshot(app_remote_aux_snapshot_t *out)
{
    if (out == NULL) {
        return APP_STATUS_INVALID_PARAM;
    }
    const uint32_t key = app_critical_enter();
    *out = g_aux;
    const app_status_t status = g_status;
    app_critical_exit(key);
    return status;
}

app_status_t app_remote_status(void)
{
    const uint32_t key = app_critical_enter();
    const app_status_t status = g_status;
    app_critical_exit(key);
    return status;
}
