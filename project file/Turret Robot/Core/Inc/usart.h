#ifndef PROJECT_USART_H
#define PROJECT_USART_H
#include "main.h"
#ifdef __cplusplus
extern "C" {
#endif
extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart3;
extern DMA_HandleTypeDef hdma_usart3_rx;
void MX_USART1_UART_Init(void);
void MX_USART3_UART_Init(void);
#ifdef __cplusplus
}
#endif
#endif
