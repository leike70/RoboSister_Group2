#ifndef TEST_CMSIS_STM32F4XX_H
#define TEST_CMSIS_STM32F4XX_H
#include <stdint.h>
uint32_t __get_PRIMASK(void);
void __disable_irq(void);
void __DMB(void);
void __set_PRIMASK(uint32_t value);
#endif
