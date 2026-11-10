#ifndef IMU_MATRIX_H
#define IMU_MATRIX_H
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <string.h>
#include <math.h>
typedef struct { uint16_t numRows, numCols; float *pData; } imu_matrix_t;
void imu_matrix_init(imu_matrix_t *m, uint16_t rows, uint16_t cols, float *data);
int8_t imu_matrix_add(const imu_matrix_t *a, const imu_matrix_t *b, imu_matrix_t *out);
int8_t imu_matrix_sub(const imu_matrix_t *a, const imu_matrix_t *b, imu_matrix_t *out);
int8_t imu_matrix_mult(const imu_matrix_t *a, const imu_matrix_t *b, imu_matrix_t *out);
int8_t imu_matrix_trans(const imu_matrix_t *a, imu_matrix_t *out);
int8_t imu_matrix_inverse(const imu_matrix_t *a, imu_matrix_t *out);
void imu_matrix_clear_error(void);
bool imu_matrix_failed(void);
#endif
