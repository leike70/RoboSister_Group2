#ifndef TEST_UART_STUB_H
#define TEST_UART_STUB_H

/* Host-side HAL UART/DMA stub: only the symbols used by Bsp/remote_uart.c.
 * Real firmware uses the STM32 HAL; this header is never on the target build. */
#include <stddef.h>
#include <stdint.h>

typedef struct {
    unsigned int unused;
} UART_HandleTypeDef;
typedef struct {
    unsigned int unused;
} DMA_HandleTypeDef;

typedef enum { HAL_OK = 0, HAL_ERROR = 1, HAL_BUSY = 2, HAL_TIMEOUT = 3 } HAL_StatusTypeDef;

extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart3;
extern DMA_HandleTypeDef hdma_usart3_rx;

#define USART3_IRQn 1
#define DMA1_Stream1_IRQn 2
#define DMA_IT_HT 4

HAL_StatusTypeDef HAL_UARTEx_ReceiveToIdle_DMA(UART_HandleTypeDef *uart, uint8_t *data,
                                               uint16_t size);
HAL_StatusTypeDef HAL_UART_AbortReceive(UART_HandleTypeDef *uart);
void HAL_NVIC_DisableIRQ(int irq);
void HAL_NVIC_EnableIRQ(int irq);
void test_dma_disable_it(DMA_HandleTypeDef *dma, unsigned int flags);
void test_uart_clear_error(UART_HandleTypeDef *uart);

#define __HAL_DMA_DISABLE_IT(dma, flags) test_dma_disable_it((dma), (flags))
#define __HAL_UART_CLEAR_OREFLAG(uart) test_uart_clear_error(uart)

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *uart, uint16_t size);
void HAL_UART_ErrorCallback(UART_HandleTypeDef *uart);

#endif /* TEST_UART_STUB_H */
