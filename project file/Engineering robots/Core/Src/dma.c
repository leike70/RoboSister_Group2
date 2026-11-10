/**
  ******************************************************************************
  * @file    dma.c
  * @brief   DMA 控制器初始化：DMA1 Stream1 用于 USART3 遥控接收。
  ******************************************************************************
  */
#include "dma.h"

void MX_DMA_Init(void)
{
    __HAL_RCC_DMA1_CLK_ENABLE();
    HAL_NVIC_SetPriority(DMA1_Stream1_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(DMA1_Stream1_IRQn);
}
