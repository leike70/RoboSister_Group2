#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "spi.h"
#include "imu_bsp.h"
#define CHECK(x) do { if (!(x)) { fprintf(stderr,"FAIL line %d: %s\n",__LINE__,#x); exit(1); } } while(0)
GPIO_TypeDef test_gpio_a = {1}, test_gpio_b = {2};
SPI_HandleTypeDef hspi1;
test_dwt_t test_dwt;
test_debug_t test_debug;
uint32_t SystemCoreClock = 168000000u;
static GPIO_TypeDef *selected;
static uint16_t selected_pin, transfer_size;
static uint8_t last_tx[16];
static bool fail_transfer;
static unsigned releases, calls;
static uint32_t delayed;
uint32_t osKernelGetTickFreq(void) { return 1000; }
int osDelay(uint32_t ticks) { delayed = ticks; return 0; }
void HAL_GPIO_WritePin(GPIO_TypeDef *port, uint16_t pin, unsigned state)
{
    if (state == GPIO_PIN_RESET) { CHECK(selected == NULL); selected=port; selected_pin=pin; }
    else { CHECK(selected == port && selected_pin == pin); selected=NULL; releases++; }
}
HAL_StatusTypeDef HAL_SPI_TransmitReceive(SPI_HandleTypeDef *spi, uint8_t *tx,
                                         uint8_t *rx, uint16_t size, uint32_t timeout)
{
    CHECK(spi == &hspi1 && selected && timeout == 1u && size <= 16u);
    calls++; transfer_size=size; memcpy(last_tx,tx,size);
    for (unsigned i=0;i<size;++i) { rx[i]=(uint8_t)(0x30u+i); }
    return fail_transfer ? HAL_ERROR : HAL_OK;
}
int main(void)
{
    const bmi088_bus_t *bus=imu_bsp_bus();
    uint8_t bytes[16]={0};
    CHECK(bus->read(true,0x12,bytes,6));
    CHECK(transfer_size == 8 && last_tx[0] == 0x92 && last_tx[1] == 0);
    CHECK(bytes[0] == 0x32 && bytes[5] == 0x37 && releases == 1 && !selected);
    CHECK(bus->read(false,0x02,bytes,6));
    CHECK(transfer_size == 7 && last_tx[0] == 0x82 && bytes[0] == 0x31);
    CHECK(bus->write(true,0x7D,4));
    CHECK(transfer_size == 2 && last_tx[0] == 0x7D && last_tx[1] == 4);
    fail_transfer=true;
    CHECK(!bus->read(true,0,bytes,1) && !selected && releases == 4);
    const unsigned before=calls;
    CHECK(!bus->read(true,0,bytes,16) && calls == before && !selected);
    bus->delay_ms(2); CHECK(delayed == 2);
    test_dwt.CYCCNT=0xFFFFFF00u; imu_bsp_init();
    test_dwt.CYCCNT += 1680u;
    CHECK(imu_bsp_now_us() == 10u); /* Handles the hardware cycle-counter wrap. */
    test_dwt.CYCCNT += 84u; CHECK(imu_bsp_now_us() == 10u);
    test_dwt.CYCCNT += 84u; CHECK(imu_bsp_now_us() == 11u);
    puts("PASS SPI accel dummy/gyro framing, CS error release, bounded transfer, DWT wrap");
    return 0;
}
