#ifndef IMU_BMI088_H
#define IMU_BMI088_H
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
typedef struct {
    bool (*read)(bool accel, uint8_t reg, uint8_t *data, size_t size);
    bool (*write)(bool accel, uint8_t reg, uint8_t value);
    void (*delay_ms)(uint32_t ms);
} bmi088_bus_t;
typedef struct {
    float accel_mps2[3], gyro_rad_s[3], temperature_c;
    uint32_t timestamp_ms, timestamp_us;
} bmi088_sample_t;
typedef enum {
    BMI088_OK = 0, BMI088_NO_DATA, BMI088_BUS_ERROR, BMI088_BAD_ID,
    BMI088_CONFIG_ERROR, BMI088_INVALID_DATA
} bmi088_result_t;
/* Bounded, yielding init. Re-checks IDs/config; never substitutes stored board offsets. */
bmi088_result_t bmi088_initialize(const bmi088_bus_t *bus);
bmi088_result_t bmi088_sample(const bmi088_bus_t *bus, bmi088_sample_t *out);
#endif
