/* Actual BMI088 driver, reference EKF port, fusion/calibration and public attitude.
 * Register bus/time are explicit fixtures; no sensor hardware is claimed by these tests. */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include "IMU/bmi088.h"
#include "IMU/BMI088reg.h"
#include "IMU/imu_fusion.h"
#include "IMU/imu_matrix.h"
#include "Attitude/app_attitude.h"
#include "Common/app_critical.h"
#define CHECK(x) do { if (!(x)) { fprintf(stderr,"FAIL %s:%d: %s\n",__FILE__,__LINE__,#x); exit(1); } } while (0)
uint32_t app_critical_enter(void) { return 0; }
void app_critical_exit(uint32_t previous) { (void)previous; }
static uint8_t registers[2][128];
static unsigned reads, writes, delays;
static bool fail_bus, mismatch;
static bool gyro_id_read_seen;
static bool bus_read(bool accel, uint8_t reg, uint8_t *data, size_t size)
{
    reads++;
    if (!accel && reg == BMI088_GYRO_CHIP_ID) { gyro_id_read_seen = true; }
    if (fail_bus || (size_t)reg + size > 128u) { return false; }
    memcpy(data, registers[accel ? 0 : 1]+reg, size);
    return true;
}
static bool bus_write(bool accel, uint8_t reg, uint8_t value)
{
    writes++;
    /* The gyro die must see a chip-select transaction before its soft reset:
     * otherwise the reset write can be ignored in the power-up interface mode. */
    if (!accel && reg == BMI088_GYRO_SOFTRESET) { CHECK(gyro_id_read_seen); }
    if (fail_bus) { return false; }
    if (!mismatch && reg != BMI088_ACC_SOFTRESET && reg != BMI088_GYRO_SOFTRESET)
        registers[accel ? 0 : 1][reg] = value;
    return true;
}
static void bus_delay(uint32_t ms) { CHECK(ms <= 80u); delays++; }
static const bmi088_bus_t bus = {bus_read,bus_write,bus_delay};
static void reset_bus(void)
{
    memset(registers,0,sizeof(registers));
    registers[0][0] = BMI088_ACC_CHIP_ID_VALUE;
    registers[1][0] = BMI088_GYRO_CHIP_ID_VALUE;
    registers[0][BMI088_ACC_STATUS] = BMI088_ACCEL_DRDY;
    registers[1][BMI088_GYRO_INT_STAT_1] = BMI088_GYRO_DYDR;
    reads = writes = delays = 0; fail_bus = mismatch = false;
    gyro_id_read_seen = false;
}
static void put16(uint8_t *out, int16_t value)
{ out[0] = (uint8_t)value; out[1] = (uint8_t)((uint16_t)value >> 8); }
static void test_driver(void)
{
    reset_bus();
    CHECK(bmi088_initialize(&bus) == BMI088_OK);
    CHECK(writes == 10u && delays == 11u && reads < 30u);
    CHECK(registers[0][BMI088_ACC_RANGE] == BMI088_ACC_RANGE_6G);
    CHECK(registers[1][BMI088_GYRO_BANDWIDTH] == 0x81u);
    put16(registers[0]+BMI088_ACCEL_XOUT_L, 1000);
    put16(registers[0]+BMI088_ACCEL_XOUT_L+2, -1000);
    put16(registers[0]+BMI088_ACCEL_XOUT_L+4, 5467);
    put16(registers[1]+BMI088_GYRO_X_L, -123);
    registers[0][BMI088_TEMP_M] = 0xFF;
    registers[0][BMI088_TEMP_L] = 0xE0; /* signed 11-bit -1 -> 22.875 C */
    bmi088_sample_t sample;
    CHECK(bmi088_sample(&bus,&sample) == BMI088_OK);
    CHECK(fabsf(sample.accel_mps2[0]-1.7944336f) < 1e-5f);
    CHECK(fabsf(sample.accel_mps2[1]+1.7944336f) < 1e-5f);
    CHECK(fabsf(sample.gyro_rad_s[0]+0.1310275f) < 1e-5f);
    CHECK(fabsf(sample.temperature_c-22.875f) < 1e-5f);
    registers[0][BMI088_ACC_STATUS] = 0;
    CHECK(bmi088_sample(&bus,&sample) == BMI088_NO_DATA);
    CHECK(sample.accel_mps2[0] == 0);
    registers[0][BMI088_ACC_STATUS] = BMI088_ACCEL_DRDY;
    registers[1][0] = 0;
    CHECK(bmi088_sample(&bus,&sample) == BMI088_BAD_ID);
    fail_bus = true;
    CHECK(bmi088_sample(&bus,&sample) == BMI088_BUS_ERROR);
    CHECK(bmi088_initialize(&bus) == BMI088_BUS_ERROR);
    reset_bus(); mismatch = true;
    CHECK(bmi088_initialize(&bus) == BMI088_CONFIG_ERROR);
    CHECK(writes < 12u); /* Readback failure must not retry forever. */
    reset_bus(); put16(registers[1]+BMI088_GYRO_X_L, 32767);
    CHECK(bmi088_sample(&bus,&sample) == BMI088_INVALID_DATA);
    CHECK(bmi088_sample(NULL,&sample) == BMI088_BUS_ERROR);
    puts("PASS checked BMI088 init, signed/scaled samples, DRDY, bus/ID/readback/saturation errors");
}
static void test_matrix(void)
{
    float a_data[4] = {0,2,1,3}, inverse[4], product[4];
    imu_matrix_t a,b,out;
    imu_matrix_init(&a,2,2,a_data); imu_matrix_init(&b,2,2,inverse);
    imu_matrix_init(&out,2,2,product);
    imu_matrix_clear_error();
    CHECK(imu_matrix_inverse(&a,&b) == 0);
    CHECK(imu_matrix_mult(&a,&b,&out) == 0);
    CHECK(fabsf(product[0]-1) < 1e-6f && fabsf(product[3]-1) < 1e-6f);
    CHECK(fabsf(product[1]) < 1e-6f && fabsf(product[2]) < 1e-6f);
    float singular[4] = {1,2,2,4}; a.pData = singular;
    CHECK(imu_matrix_inverse(&a,&b) != 0 && imu_matrix_failed());
    imu_matrix_clear_error();
    CHECK(!imu_matrix_failed());
    a.numCols = 1;
    CHECK(imu_matrix_mult(&a,&b,&out) != 0 && imu_matrix_failed());
    puts("PASS portable EKF matrix pivoting, singular and shape errors");
}
static uint32_t sample_us;
static bmi088_sample_t stationary(float roll, float pitch)
{
    bmi088_sample_t sample = {0};
    sample.accel_mps2[0] = -9.81f*sinf(pitch);
    sample.accel_mps2[1] = 9.81f*sinf(roll)*cosf(pitch);
    sample.accel_mps2[2] = 9.81f*cosf(roll)*cosf(pitch);
    sample.gyro_rad_s[0] = 0.005f;
    sample.gyro_rad_s[1] = -0.003f;
    sample.gyro_rad_s[2] = 0.002f;
    sample.temperature_c = 23;
    sample_us += 2000u; sample.timestamp_us = sample_us;
    sample.timestamp_ms = sample_us/1000u;
    return sample;
}
static app_attitude_snapshot_t calibrate(float roll,float pitch)
{
    imu_fusion_reset(); sample_us = 0;
    app_attitude_snapshot_t out;
    for (unsigned i=0;i<imu_config_get()->calibration_samples;++i) {
        bmi088_sample_t sample = stationary(roll,pitch);
        CHECK(imu_fusion_push(&sample,&out) == APP_STATUS_NOT_READY && !out.valid);
    }
    float bias[3]; imu_fusion_bias(bias);
    CHECK(fabsf(bias[0]-0.005f) < 1e-6f);
    CHECK(fabsf(bias[1]+0.003f) < 1e-6f);
    CHECK(fabsf(bias[2]-0.002f) < 1e-6f);
    for (unsigned i=0;i<500u;++i) {
        bmi088_sample_t sample = stationary(roll,pitch);
        CHECK(imu_fusion_push(&sample,&out) == APP_STATUS_OK && out.valid);
    }
    CHECK(fabsf(out.roll_rad-roll) < 0.01f && fabsf(out.pitch_rad-pitch) < 0.01f);
    CHECK(fabsf(out.roll_rate_rad_s) < 0.001f && fabsf(out.pitch_rate_rad_s) < 0.001f);
    return out;
}
static void test_fusion(void)
{
    (void)calibrate(0,0);
    (void)calibrate(0.4f,-0.3f);
    (void)calibrate(-0.3f,0.4f);
    app_attitude_snapshot_t out = calibrate(0,0);
    for (unsigned i=0;i<500u;++i) {
        bmi088_sample_t sample = stationary(0,0);
        sample.gyro_rad_s[2] += 0.5f;
        CHECK(imu_fusion_push(&sample,&out) == APP_STATUS_OK && out.valid);
    }
    CHECK(fabsf(out.yaw_rad-0.5f) < 0.015f);
    CHECK(fabsf(out.yaw_rate_rad_s-0.5f) < 0.001f);
    imu_fusion_reset();
    for (unsigned i=0;i<100u;++i) {
        bmi088_sample_t sample = stationary(0,0);
        CHECK(imu_fusion_push(&sample,&out) == APP_STATUS_NOT_READY);
    }
    bmi088_sample_t moving = stationary(0,0); moving.gyro_rad_s[0] = 0.5f;
    CHECK(imu_fusion_push(&moving,&out) == APP_STATUS_NOT_READY);
    CHECK(imu_fusion_calibration_count() == 0);
    out = calibrate(0,0);
    bmi088_sample_t bad = stationary(0,0); bad.accel_mps2[0] = NAN;
    CHECK(imu_fusion_push(&bad,&out) == APP_STATUS_INVALID_PARAM && !out.valid);
    out = calibrate(0,0); bad = stationary(0,0); bad.timestamp_us += 100000u;
    CHECK(imu_fusion_push(&bad,&out) == APP_STATUS_NOT_READY && !out.valid);
    out = calibrate(0,0);
    sample_us = 0xFFFFF000u; /* First jump resets, then calibrate across uint32 wrap. */
    imu_fusion_reset();
    for (unsigned i=0;i<501u;++i) {
        bmi088_sample_t sample = stationary(0,0);
        app_status_t status = imu_fusion_push(&sample,&out);
        CHECK(i < 500u ? status == APP_STATUS_NOT_READY : status == APP_STATUS_OK);
    }
    puts("PASS EKF/calibration, initial tilt, yaw integration, moving/NaN/gap rejection and time wrap");
}
static void test_public_snapshot(void)
{
    app_attitude_snapshot_t sample = {.valid=true,.roll_rad=0.2f,.timestamp_ms=0xFFFFFFF0u};
    app_attitude_snapshot_t out;
    CHECK(app_attitude_init(0) == APP_STATUS_NOT_READY);
    CHECK(app_attitude_ingest(&sample) == APP_STATUS_OK);
    CHECK(app_attitude_update(4u) == APP_STATUS_OK); /* wrap, age20ms */
    CHECK(app_attitude_update(5u) == APP_STATUS_OFFLINE);
    CHECK(app_attitude_get_snapshot(&out) == APP_STATUS_OFFLINE && !out.valid);
    CHECK(out.roll_rad == 0 && out.timestamp_ms == sample.timestamp_ms);
    CHECK(app_attitude_update(6u) == APP_STATUS_OFFLINE);
    sample.timestamp_ms = 7u;
    CHECK(app_attitude_ingest(&sample) == APP_STATUS_OK);
    CHECK(app_attitude_update(7u) == APP_STATUS_OK);
    puts("PASS public attitude freshness, wrap-around invalidation and recovery");
}
int main(void)
{
    test_driver(); test_matrix(); test_fusion(); test_public_snapshot();
    puts("PASS IMU regression suite"); return 0;
}

