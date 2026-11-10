/* Small portable matrix backend for the supplied six-state quaternion EKF.
 * Fixed capacity, alias-safe operations, sticky errors; no heap/DSP ABI dependency. */
#include "imu_matrix.h"
#define CAP 6u
static bool failed;
void imu_matrix_clear_error(void) { failed = false; }
bool imu_matrix_failed(void) { return failed; }
static int8_t error(void) { failed = true; return -1; }
static bool valid(const imu_matrix_t *m)
{
    return m != NULL && m->pData != NULL && m->numRows > 0u &&
           m->numCols > 0u && m->numRows <= CAP && m->numCols <= CAP;
}
void imu_matrix_init(imu_matrix_t *m, uint16_t rows, uint16_t cols, float *data)
{
    m->numRows = rows; m->numCols = cols; m->pData = data;
}
static int8_t elementwise(const imu_matrix_t *a, const imu_matrix_t *b,
                          imu_matrix_t *out, float sign)
{
    if (!valid(a) || !valid(b) || !valid(out) ||
        a->numRows != b->numRows || a->numCols != b->numCols ||
        out->numRows != a->numRows || out->numCols != a->numCols) { return error(); }
    for (unsigned i = 0; i < (unsigned)a->numRows * a->numCols; ++i) {
        out->pData[i] = a->pData[i] + sign * b->pData[i];
        if (!isfinite(out->pData[i])) { return error(); }
    }
    return 0;
}
int8_t imu_matrix_add(const imu_matrix_t *a, const imu_matrix_t *b, imu_matrix_t *out)
{ return elementwise(a, b, out, 1.0f); }
int8_t imu_matrix_sub(const imu_matrix_t *a, const imu_matrix_t *b, imu_matrix_t *out)
{ return elementwise(a, b, out, -1.0f); }
int8_t imu_matrix_mult(const imu_matrix_t *a, const imu_matrix_t *b, imu_matrix_t *out)
{
    float temp[CAP * CAP] = {0};
    if (!valid(a) || !valid(b) || !valid(out) || a->numCols != b->numRows ||
        out->numRows != a->numRows || out->numCols != b->numCols) { return error(); }
    for (unsigned r = 0; r < a->numRows; ++r) {
        for (unsigned c = 0; c < b->numCols; ++c) {
            float value = 0.0f;
            for (unsigned k = 0; k < a->numCols; ++k) {
                value += a->pData[r * a->numCols + k] * b->pData[k * b->numCols + c];
            }
            if (!isfinite(value)) { return error(); }
            temp[r * b->numCols + c] = value;
        }
    }
    memcpy(out->pData, temp, (size_t)out->numRows * out->numCols * sizeof(float));
    return 0;
}
int8_t imu_matrix_trans(const imu_matrix_t *a, imu_matrix_t *out)
{
    float temp[CAP * CAP];
    if (!valid(a) || !valid(out) || out->numRows != a->numCols ||
        out->numCols != a->numRows) { return error(); }
    for (unsigned r = 0; r < a->numRows; ++r)
        for (unsigned c = 0; c < a->numCols; ++c)
            temp[c * a->numRows + r] = a->pData[r * a->numCols + c];
    memcpy(out->pData, temp, (size_t)a->numRows * a->numCols * sizeof(float));
    return 0;
}
int8_t imu_matrix_inverse(const imu_matrix_t *a, imu_matrix_t *out)
{
    float aug[CAP][CAP * 2u] = {{0}};
    if (!valid(a) || !valid(out) || a->numRows != a->numCols ||
        out->numRows != a->numRows || out->numCols != a->numCols) { return error(); }
    const unsigned n = a->numRows;
    for (unsigned r = 0; r < n; ++r) {
        for (unsigned c = 0; c < n; ++c) {
            if (!isfinite(a->pData[r*n+c])) { return error(); }
            aug[r][c] = a->pData[r*n+c];
        }
        aug[r][n+r] = 1.0f;
    }
    for (unsigned k = 0; k < n; ++k) {
        unsigned pivot = k;
        for (unsigned r = k+1u; r < n; ++r)
            if (fabsf(aug[r][k]) > fabsf(aug[pivot][k])) { pivot = r; }
        if (fabsf(aug[pivot][k]) < 1e-20f) { return error(); }
        for (unsigned c = 0; c < 2u*n; ++c) {
            const float value = aug[k][c]; aug[k][c] = aug[pivot][c]; aug[pivot][c] = value;
        }
        const float scale = aug[k][k];
        for (unsigned c = 0; c < 2u*n; ++c) { aug[k][c] /= scale; }
        for (unsigned r = 0; r < n; ++r) {
            if (r == k) { continue; }
            const float factor = aug[r][k];
            for (unsigned c = 0; c < 2u*n; ++c) { aug[r][c] -= factor * aug[k][c]; }
        }
    }
    for (unsigned r = 0; r < n; ++r)
        for (unsigned c = 0; c < n; ++c) {
            if (!isfinite(aug[r][n+c])) { return error(); }
            out->pData[r*n+c] = aug[r][n+c];
        }
    return 0;
}
