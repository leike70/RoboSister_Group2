#ifndef TEST_IMU_OS_H
#define TEST_IMU_OS_H
#include <stdint.h>
uint32_t osKernelGetTickFreq(void);
int osDelay(uint32_t ticks);
#endif
