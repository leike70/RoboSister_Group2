#ifndef IMU_BSP_H
#define IMU_BSP_H
#include "IMU/bmi088.h"
void imu_bsp_init(void);
/* Single-task-owned monotonic microseconds; call at least once per DWT wrap. */
uint32_t imu_bsp_now_us(void);
const bmi088_bus_t *imu_bsp_bus(void);
#endif
