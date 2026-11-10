#ifndef TIM_H
#define TIM_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

extern TIM_HandleTypeDef htim10;

void MX_TIM10_Init(void);
void HAL_TIM_MspPostInit(TIM_HandleTypeDef *timHandle);

#ifdef __cplusplus
}
#endif

#endif /* TIM_H */
