#ifndef TEST_IMU_SPI_H
#define TEST_IMU_SPI_H
#include <stdint.h>
typedef struct { int id; } GPIO_TypeDef;
typedef struct { int unused; } SPI_HandleTypeDef;
typedef struct { uint32_t CTRL, CYCCNT; } test_dwt_t;
typedef struct { uint32_t DEMCR; } test_debug_t;
extern GPIO_TypeDef test_gpio_a, test_gpio_b;
extern SPI_HandleTypeDef hspi1;
extern test_dwt_t test_dwt;
extern test_debug_t test_debug;
extern uint32_t SystemCoreClock;
#define GPIOA (&test_gpio_a)
#define GPIOB (&test_gpio_b)
#define GPIO_PIN_4 16u
#define GPIO_PIN_0 1u
#define GPIO_PIN_RESET 0u
#define GPIO_PIN_SET 1u
#define DWT (&test_dwt)
#define CoreDebug (&test_debug)
#define CoreDebug_DEMCR_TRCENA_Msk 1u
#define DWT_CTRL_CYCCNTENA_Msk 1u
typedef enum { HAL_OK, HAL_ERROR } HAL_StatusTypeDef;
void HAL_GPIO_WritePin(GPIO_TypeDef *port, uint16_t pin, unsigned state);
HAL_StatusTypeDef HAL_SPI_TransmitReceive(SPI_HandleTypeDef *spi, uint8_t *tx,
                                         uint8_t *rx, uint16_t size, uint32_t timeout);
#endif
