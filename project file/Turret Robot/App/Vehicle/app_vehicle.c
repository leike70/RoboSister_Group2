/* Turret: edit this vehicle's gimbal/shooting behavior here.
 * Up/down are independent extension points, with no predefined manual/auto action. */
#include "Vehicle/app_vehicle.h"
#include <stddef.h>
#include "Gimbal/app_gimbal.h"
#include "Shooting/app_shooting.h"

static app_vehicle_report_t g_report;
static app_mode_t g_last_mode = APP_MODE_UNKNOWN;

static void clear_targets(uint32_t now_ms)
{
    (void)app_gimbal_stop(now_ms);
    (void)app_shooting_stop(now_ms);
}
static void record(app_status_t status, app_mode_t mode, uint32_t now_ms)
{
    g_report.status = status;
    g_report.primary_status = status;
    g_report.secondary_status = status;
    g_report.mode = mode;
    g_report.updated_ms = now_ms;
}
app_status_t app_vehicle_init(uint32_t now_ms)
{
    (void)app_gimbal_init(now_ms);
    (void)app_shooting_init(now_ms);
    g_last_mode = APP_MODE_UNKNOWN;
    record(APP_STATUS_NOT_READY, APP_MODE_UNKNOWN, now_ms);
    return APP_STATUS_NOT_READY;
}

/* Fill only this car's upper-position behavior; no motion is implemented yet. */
app_status_t app_vehicle_on_up(const app_remote_snapshot_t *input, uint32_t now_ms)
{
    (void)input;
    (void)now_ms;
    return APP_STATUS_NOT_IMPLEMENTED;
}

/* Fill only this car's lower-position behavior, independently of the upper one. */
app_status_t app_vehicle_on_down(const app_remote_snapshot_t *input, uint32_t now_ms)
{
    (void)input;
    (void)now_ms;
    return APP_STATUS_NOT_IMPLEMENTED;
}

app_status_t app_vehicle_update(const app_remote_snapshot_t *remote,
                               bool output_allowed, uint32_t now_ms)
{
    const app_mode_t mode = remote != NULL ? remote->mode : APP_MODE_UNKNOWN;
    if (remote == NULL || !remote->online || !output_allowed ||
        (mode != APP_MODE_VEHICLE_UP && mode != APP_MODE_VEHICLE_DOWN)) {
        clear_targets(now_ms);
        g_last_mode = APP_MODE_UNKNOWN;
        record(APP_STATUS_NOT_READY, mode, now_ms);
        return APP_STATUS_NOT_READY;
    }
    if (mode != g_last_mode) {
        /* Switching branches invalidates the previous branch's saved targets. */
        clear_targets(now_ms);
    }
    g_last_mode = mode;
    const app_status_t status = mode == APP_MODE_VEHICLE_UP
        ? app_vehicle_on_up(remote, now_ms) : app_vehicle_on_down(remote, now_ms);
    record(status, mode, now_ms);
    return status;
}
app_status_t app_vehicle_stop(uint32_t now_ms)
{
    clear_targets(now_ms);
    g_last_mode = APP_MODE_UNKNOWN;
    record(APP_STATUS_NOT_READY, APP_MODE_UNKNOWN, now_ms);
    return APP_STATUS_NOT_READY;
}
app_status_t app_vehicle_get_report(app_vehicle_report_t *out)
{
    if (out == NULL) { return APP_STATUS_INVALID_PARAM; }
    *out = g_report;
    return APP_STATUS_OK;
}
