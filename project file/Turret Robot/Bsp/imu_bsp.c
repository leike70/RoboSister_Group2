#include "imu_bsp.h"
#include "spi.h"
#include "cmsis_os.h"
#include <string.h>
static uint32_t previous_cycles, total_us, remaining_cycles;
static bool transfer(bool accel, uint8_t reg, uint8_t *data, size_t size, bool write)
{
    uint8_t tx[16] = {0}, rx[16] = {0};
    const size_t overhead = write ? 1u : (accel ? 2u : 1u);
    if (!data || size == 0u || size + overhead > sizeof(tx)) { return false; }
    tx[0] = write ? reg : (uint8_t)(reg | 0x80u);
    if (write) { memcpy(tx + 1, data, size); }
    GPIO_TypeDef *port = accel ? GPIOA : GPIOB;
    const uint16_t pin = accel ? GPIO_PIN_4 : GPIO_PIN_0;
    HAL_GPIO_WritePin(port, pin, GPIO_PIN_RESET);
    const HAL_StatusTypeDef status = HAL_SPI_TransmitReceive(
        &hspi1, tx, rx, (uint16_t)(size + overhead), 1u);
    HAL_GPIO_WritePin(port, pin, GPIO_PIN_SET);
    if (status != HAL_OK) { return false; }
    if (!write) { memcpy(data, rx + overhead, size); }
    return true;
}
static bool read_register(bool accel, uint8_t reg, uint8_t *data, size_t size)
{ return transfer(accel, reg, data, size, false); }
static bool write_register(bool accel, uint8_t reg, uint8_t value)
{ return transfer(accel, reg, &value, 1u, true); }
static void delay_ms(uint32_t ms)
{
    uint32_t ticks = (uint32_t)(((uint64_t)ms * osKernelGetTickFreq() + 999u) / 1000u);
    (void)osDelay(ticks ? ticks : 1u);
}
static const bmi088_bus_t bus = {read_register, write_register, delay_ms};
void imu_bsp_init(void)
{
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
    previous_cycles = DWT->CYCCNT;
    total_us = remaining_cycles = 0;
}
uint32_t imu_bsp_now_us(void)
{
    const uint32_t cycles = DWT->CYCCNT;
    const uint32_t delta = cycles - previous_cycles;
    previous_cycles = cycles;
    const uint32_t cycles_per_us = SystemCoreClock / 1000000u;
    const uint64_t accumulated = (uint64_t)delta + remaining_cycles;
    total_us += (uint32_t)(accumulated / cycles_per_us);
    remaining_cycles = (uint32_t)(accumulated % cycles_per_us);
    return total_us;
}
const bmi088_bus_t *imu_bsp_bus(void) { return &bus; }
