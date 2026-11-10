#include "imu_fusion.h"
#include "QuaternionEKF.h"
#include <math.h>
#include <string.h>
static uint16_t count;
static float sum[3], bias[3], gyro_min[3], gyro_max[3], acc_min[3], acc_max[3];
static uint32_t last_us;
static bool ready, have_time;
void imu_fusion_reset(void)
{
    count = 0; ready = false; have_time = false;
    memset(sum, 0, sizeof(sum)); memset(bias, 0, sizeof(bias));
}
uint16_t imu_fusion_calibration_count(void) { return count; }
void imu_fusion_bias(float out[3]) { if (out) { memcpy(out, bias, sizeof(bias)); } }
static void rotate(const float r[9], const float in[3], float out[3])
{
    for (unsigned i = 0; i < 3u; ++i)
        out[i] = r[i*3u]*in[0] + r[i*3u+1u]*in[1] + r[i*3u+2u]*in[2];
}
static bool rotation_valid(const float r[9])
{
    for (unsigned row = 0; row < 3u; ++row)
        for (unsigned other = 0; other < 3u; ++other) {
            float dot = 0;
            for (unsigned col = 0; col < 3u; ++col) {
                if (!isfinite(r[row*3u+col])) { return false; }
                dot += r[row*3u+col] * r[other*3u+col];
            }
            if (fabsf(dot - (row == other ? 1.0f : 0.0f)) > 0.001f) { return false; }
        }
    const float det = r[0]*(r[4]*r[8]-r[5]*r[7]) -
                      r[1]*(r[3]*r[8]-r[5]*r[6]) +
                      r[2]*(r[3]*r[7]-r[4]*r[6]);
    return fabsf(det - 1.0f) < 0.001f;
}
static void start_ekf(const float accel[3], const imu_config_t *config)
{
    IMU_QuaternionEKF_Init(config->ekf_q1, config->ekf_q2, config->ekf_r,
                          config->ekf_fading, config->ekf_accel_lpf_s);
    /* Initial roll/pitch from gravity, yaw=0; never start a tilted board as level. */
    const float roll = atan2f(accel[1], accel[2]);
    const float pitch = atan2f(-accel[0], hypotf(accel[1], accel[2]));
    const float cr = cosf(roll*0.5f), sr = sinf(roll*0.5f);
    const float cp = cosf(pitch*0.5f), sp = sinf(pitch*0.5f);
    const float q[4] = {cr*cp, sr*cp, cr*sp, -sr*sp};
    memcpy(QEKF_INS.IMU_QuaternionEKF.xhat_data, q, sizeof(q));
}
app_status_t imu_fusion_push(const bmi088_sample_t *sample, app_attitude_snapshot_t *out)
{
    if (!out) { return APP_STATUS_INVALID_PARAM; }
    memset(out, 0, sizeof(*out));
    if (!sample) { imu_fusion_reset(); return APP_STATUS_INVALID_PARAM; }
    out->timestamp_ms = sample->timestamp_ms;
    const imu_config_t *config = imu_config_get();
    if (!config || !rotation_valid(config->sensor_to_body) ||
        config->calibration_samples < 10u || config->sample_period_ms == 0u ||
        config->stale_timeout_ms < config->sample_period_ms ||
        config->stale_timeout_ms > IMU_DATA_TIMEOUT_MS ||
        !isfinite(config->max_calibration_rate_rad_s) ||
        config->max_calibration_rate_rad_s <= 0.0f ||
        !isfinite(config->max_gyro_span_rad_s) || config->max_gyro_span_rad_s <= 0.0f ||
        !isfinite(config->max_accel_span_mps2) || config->max_accel_span_mps2 <= 0.0f ||
        !isfinite(config->ekf_q1) || config->ekf_q1 <= 0.0f ||
        !isfinite(config->ekf_q2) || config->ekf_q2 < 0.0f ||
        !isfinite(config->ekf_r) || config->ekf_r <= 0.0f ||
        !isfinite(config->ekf_fading) || config->ekf_fading <= 0.0f ||
        config->ekf_fading > 1.0f || !isfinite(config->ekf_accel_lpf_s) ||
        config->ekf_accel_lpf_s < 0.0f) {
        imu_fusion_reset(); return APP_STATUS_UNCONFIGURED;
    }
    float a[3], g[3];
    rotate(config->sensor_to_body, sample->accel_mps2, a);
    rotate(config->sensor_to_body, sample->gyro_rad_s, g);
    for (unsigned i = 0; i < 3u; ++i)
        if (!isfinite(a[i]) || !isfinite(g[i])) {
            imu_fusion_reset(); return APP_STATUS_INVALID_PARAM;
        }
    const float anorm = sqrtf(a[0]*a[0]+a[1]*a[1]+a[2]*a[2]);
    const float gnorm = sqrtf(g[0]*g[0]+g[1]*g[1]+g[2]*g[2]);
    if (!isfinite(anorm) || anorm < 0.1f || anorm > 58.0f ||
        !isfinite(gnorm) || gnorm > 34.9f) {
        imu_fusion_reset(); return APP_STATUS_INVALID_PARAM;
    }
    float dt = 0.0f;
    if (have_time) {
        dt = (float)(uint32_t)(sample->timestamp_us - last_us) * 1e-6f;
        if (dt <= 0.0f || dt > (float)config->stale_timeout_ms * 0.001f) {
            imu_fusion_reset(); return APP_STATUS_NOT_READY;
        }
    }
    last_us = sample->timestamp_us; have_time = true;
    if (!ready) {
        if (anorm < 9.81f-0.5f || anorm > 9.81f+0.5f ||
            gnorm > config->max_calibration_rate_rad_s) {
            count = 0; memset(sum, 0, sizeof(sum)); return APP_STATUS_NOT_READY;
        }
        for (unsigned i = 0; i < 3u; ++i) {
            if (count == 0u) {
                gyro_min[i] = gyro_max[i] = g[i]; acc_min[i] = acc_max[i] = a[i];
            }
            gyro_min[i] = fminf(gyro_min[i], g[i]); gyro_max[i] = fmaxf(gyro_max[i], g[i]);
            acc_min[i] = fminf(acc_min[i], a[i]); acc_max[i] = fmaxf(acc_max[i], a[i]);
            if (gyro_max[i]-gyro_min[i] > config->max_gyro_span_rad_s ||
                acc_max[i]-acc_min[i] > config->max_accel_span_mps2) {
                count = 0; memset(sum, 0, sizeof(sum)); return APP_STATUS_NOT_READY;
            }
        }
        for (unsigned i = 0; i < 3u; ++i) { sum[i] += g[i]; }
        if (++count < config->calibration_samples) { return APP_STATUS_NOT_READY; }
        for (unsigned i = 0; i < 3u; ++i) { bias[i] = sum[i] / (float)count; }
        start_ekf(a, config); ready = true;
        return APP_STATUS_NOT_READY;
    }
    for (unsigned i = 0; i < 3u; ++i) { g[i] -= bias[i]; }
    imu_matrix_clear_error();
    IMU_QuaternionEKF_Update(g[0], g[1], g[2], a[0], a[1], a[2], dt);
    float *q = QEKF_INS.q;
    float norm = sqrtf(q[0]*q[0]+q[1]*q[1]+q[2]*q[2]+q[3]*q[3]);
    if (imu_matrix_failed() || !isfinite(norm) || norm < 0.5f || norm > 1.5f) {
        imu_fusion_reset(); return APP_STATUS_FAULT;
    }
    for (unsigned i = 0; i < 4u; ++i) {
        q[i] /= norm;
        QEKF_INS.IMU_QuaternionEKF.xhat_data[i] = q[i];
    }
    /* Standard ZYX Euler: roll about X, pitch about Y, yaw about Z, radians.
     * The reference's legacy Pitch/Roll labels are deliberately not exported. */
    out->roll_rad = atan2f(2.0f*(q[0]*q[1]+q[2]*q[3]), 1.0f-2.0f*(q[1]*q[1]+q[2]*q[2]));
    out->pitch_rad = asinf(fmaxf(-1.0f, fminf(1.0f, 2.0f*(q[0]*q[2]-q[3]*q[1]))));
    out->yaw_rad = atan2f(2.0f*(q[0]*q[3]+q[1]*q[2]), 1.0f-2.0f*(q[2]*q[2]+q[3]*q[3]));
    out->roll_rate_rad_s = g[0]-QEKF_INS.GyroBias[0];
    out->pitch_rate_rad_s = g[1]-QEKF_INS.GyroBias[1];
    out->yaw_rate_rad_s = g[2]-QEKF_INS.GyroBias[2];
    const float values[6] = {out->roll_rad, out->pitch_rad, out->yaw_rad,
        out->roll_rate_rad_s, out->pitch_rate_rad_s, out->yaw_rate_rad_s};
    for (unsigned i = 0; i < 6u; ++i)
        if (!isfinite(values[i])) { imu_fusion_reset(); return APP_STATUS_FAULT; }
    out->valid = true;
    return APP_STATUS_OK;
}
