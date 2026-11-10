/**
  ******************************************************************************
  * @file    dma.h
  * @brief   DMA 初始化（当前只用于 USART3 遥控接收）。
  ******************************************************************************
  */
#ifndef PROJECT_DMA_H
#define PROJECT_DMA_H

#include "main.h"

#ifdef __cplusplus
extern "C" {
#endif

void MX_DMA_Init(void);

#ifdef __cplusplus
}
#endif

#endif /* PROJECT_DMA_H */
