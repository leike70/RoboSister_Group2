#include "Common/app_critical.h"
#include "stm32f4xx.h"

uint32_t app_critical_enter(void)
{
    const uint32_t saved_mask = __get_PRIMASK();
    __disable_irq();
    __DMB();
    return saved_mask;
}

void app_critical_exit(uint32_t saved_mask)
{
    __DMB();
    __set_PRIMASK(saved_mask);
}
