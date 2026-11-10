#ifndef TEST_TIM_H
#define TEST_TIM_H

#include <stdint.h>

typedef struct {
    uint32_t ARR;
    uint32_t CCR1;
} TIM_TypeDef;

typedef struct {
    TIM_TypeDef *Instance;
} TIM_HandleTypeDef;

typedef enum {
    HAL_OK = 0,
    HAL_ERROR = 1
} HAL_StatusTypeDef;

extern TIM_HandleTypeDef htim10;

#define TIM_CHANNEL_1 1u
#define __HAL_TIM_SET_COMPARE(handle, channel, value) \
    do { (void)(channel); (handle)->Instance->CCR1 = (value); } while (0)

HAL_StatusTypeDef HAL_TIM_PWM_Start(TIM_HandleTypeDef *handle, uint32_t channel);

#endif
