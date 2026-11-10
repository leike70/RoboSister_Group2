/* Register setup and scaling ported from the supplied Wang Hongxi BMI088 reference.
 * Adapted for checked bus transfers and finite retries; no foreign-board calibration. */
#include "bmi088.h"
#include "BMI088reg.h"
#include <string.h>
#include <math.h>
static bool bus_valid(const bmi088_bus_t *bus)
{ return bus && bus->read && bus->write && bus->delay_ms; }
static bmi088_result_t identify(const bmi088_bus_t *bus)
{
    uint8_t acc, gyro;
    if (!bus->read(true, BMI088_ACC_CHIP_ID, &acc, 1) ||
        !bus->read(false, BMI088_GYRO_CHIP_ID, &gyro, 1)) { return BMI088_BUS_ERROR; }
    return acc == BMI088_ACC_CHIP_ID_VALUE && gyro == BMI088_GYRO_CHIP_ID_VALUE
        ? BMI088_OK : BMI088_BAD_ID;
}
bmi088_result_t bmi088_initialize(const bmi088_bus_t *bus)
{
    if (!bus_valid(bus)) { return BMI088_BUS_ERROR; }
    uint8_t ignored;
    /* Both dies must see a SPI transaction before configuration/reset: the
     * first chip-select rising edge selects SPI mode (the gyro die was
     * previously left in its power-up interface mode). */
    if (!bus->read(true, BMI088_ACC_CHIP_ID, &ignored, 1) ||
        !bus->read(false, BMI088_GYRO_CHIP_ID, &ignored, 1)) { return BMI088_BUS_ERROR; }
    bus->delay_ms(2);
    if (!bus->write(true, BMI088_ACC_SOFTRESET, BMI088_ACC_SOFTRESET_VALUE) ||
        !bus->write(false, BMI088_GYRO_SOFTRESET, BMI088_GYRO_SOFTRESET_VALUE))
        { return BMI088_BUS_ERROR; }
    bus->delay_ms(80);
    if (!bus->read(true, BMI088_ACC_CHIP_ID, &ignored, 1)) { return BMI088_BUS_ERROR; }
    bus->delay_ms(2);
    bmi088_result_t status = identify(bus);
    if (status != BMI088_OK) { return status; }
    const uint8_t acc[][2] = {
        {BMI088_ACC_PWR_CTRL, BMI088_ACC_ENABLE_ACC_ON},
        {BMI088_ACC_PWR_CONF, BMI088_ACC_PWR_ACTIVE_MODE},
        {BMI088_ACC_CONF, BMI088_ACC_NORMAL | BMI088_ACC_800_HZ | BMI088_ACC_CONF_MUST_Set},
        {BMI088_ACC_RANGE, BMI088_ACC_RANGE_6G}
    };
    const uint8_t gyro[][2] = {
        {BMI088_GYRO_RANGE, BMI088_GYRO_2000},
        {BMI088_GYRO_BANDWIDTH, BMI088_GYRO_2000_230_HZ | BMI088_GYRO_BANDWIDTH_MUST_Set},
        {BMI088_GYRO_LPM1, BMI088_GYRO_NORMAL_MODE},
        {BMI088_GYRO_CTRL, BMI088_DRDY_ON}
    };
    for (unsigned sensor = 0; sensor < 2u; ++sensor) {
        const uint8_t (*settings)[2] = sensor == 0u ? acc : gyro;
        for (unsigned i = 0; i < 4u; ++i) {
            if (!bus->write(sensor == 0u, settings[i][0], settings[i][1]))
                { return BMI088_BUS_ERROR; }
            bus->delay_ms(2);
            uint8_t actual;
            if (!bus->read(sensor == 0u, settings[i][0], &actual, 1))
                { return BMI088_BUS_ERROR; }
            if (actual != settings[i][1]) { return BMI088_CONFIG_ERROR; }
        }
    }
    return BMI088_OK;
}
static int16_t little_i16(const uint8_t *bytes)
{ return (int16_t)((uint16_t)bytes[0] | ((uint16_t)bytes[1] << 8)); }
bmi088_result_t bmi088_sample(const bmi088_bus_t *bus, bmi088_sample_t *out)
{
    if (!bus_valid(bus) || !out) { return BMI088_BUS_ERROR; }
    memset(out, 0, sizeof(*out));
    bmi088_result_t status = identify(bus);
    if (status != BMI088_OK) { return status; }
    uint8_t ready_acc, ready_gyro;
    if (!bus->read(true, BMI088_ACC_STATUS, &ready_acc, 1) ||
        !bus->read(false, BMI088_GYRO_INT_STAT_1, &ready_gyro, 1))
        { return BMI088_BUS_ERROR; }
    if (!(ready_acc & BMI088_ACCEL_DRDY) || !(ready_gyro & BMI088_GYRO_DYDR))
        { return BMI088_NO_DATA; }
    uint8_t accel[6], gyro[6], temp[2], sensor_error;
    if (!bus->read(true, BMI088_ACCEL_XOUT_L, accel, sizeof(accel)) ||
        !bus->read(false, BMI088_GYRO_X_L, gyro, sizeof(gyro)) ||
        !bus->read(true, BMI088_TEMP_M, temp, sizeof(temp)) ||
        !bus->read(true, BMI088_ACC_ERR_REG, &sensor_error, 1))
        { return BMI088_BUS_ERROR; }
    if (sensor_error & (BMI088_ACCEL_CONGIF_ERROR | BMI088_FATAL_ERROR))
        { return BMI088_CONFIG_ERROR; }
    for (unsigned axis = 0; axis < 3u; ++axis) {
        const int16_t a = little_i16(accel + 2u*axis);
        const int16_t g = little_i16(gyro + 2u*axis);
        if (a >= 32760 || a <= -32760 || g >= 32760 || g <= -32760)
            { memset(out, 0, sizeof(*out)); return BMI088_INVALID_DATA; }
        out->accel_mps2[axis] = (float)a * 0.00179443359375f;
        out->gyro_rad_s[axis] = (float)g * 0.0010652644360316953f;
    }
    int16_t raw_temp = (int16_t)(((uint16_t)temp[0] << 3) | (temp[1] >> 5));
    if (raw_temp > 1023) { raw_temp -= 2048; }
    out->temperature_c = (float)raw_temp * 0.125f + 23.0f;
    if (out->temperature_c < -40.0f || out->temperature_c > 85.0f)
        { return BMI088_INVALID_DATA; }
    return BMI088_OK;
}
