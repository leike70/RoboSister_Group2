/* IMU heater PWM: PF6 -> TIM10_CH1 (AF3), 1 kHz, active high.
 * TIM10 is on APB2; with the 168 MHz system clock the timer clock is 168 MHz:
 * 168 MHz / 84 = 2 MHz counter, / 2000 = 1 kHz PWM. */
#include "tim.h"

TIM_HandleTypeDef htim10;

void MX_TIM10_Init(void)
{
    TIM_OC_InitTypeDef sConfigOC = {0};

    htim10.Instance = TIM10;
    htim10.Init.Prescaler = 84 - 1;
    htim10.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim10.Init.Period = 2000 - 1;
    htim10.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    htim10.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
    if (HAL_TIM_Base_Init(&htim10) != HAL_OK) { Error_Handler(); }
    if (HAL_TIM_PWM_Init(&htim10) != HAL_OK) { Error_Handler(); }

    sConfigOC.OCMode = TIM_OCMODE_PWM1;
    sConfigOC.Pulse = 0;
    sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
    sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
    if (HAL_TIM_PWM_ConfigChannel(&htim10, &sConfigOC, TIM_CHANNEL_1) != HAL_OK) {
        Error_Handler();
    }
    HAL_TIM_MspPostInit(&htim10);
}

void HAL_TIM_Base_MspInit(TIM_HandleTypeDef *tim_baseHandle)
{
    if (tim_baseHandle->Instance == TIM10) {
        __HAL_RCC_TIM10_CLK_ENABLE();
    }
}

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *timHandle)
{
    GPIO_InitTypeDef gpio = {0};
    if (timHandle->Instance != TIM10) { return; }
    __HAL_RCC_GPIOF_CLK_ENABLE();
    /* Keep the gate low while switching PF6 to the timer alternate function. */
    HAL_GPIO_WritePin(IMU_HEAT_GPIO_Port, IMU_HEAT_Pin, GPIO_PIN_RESET);
    gpio.Pin = IMU_HEAT_Pin;
    gpio.Mode = GPIO_MODE_AF_PP;
    gpio.Pull = GPIO_PULLUP;
    gpio.Speed = GPIO_SPEED_FREQ_HIGH;
    gpio.Alternate = GPIO_AF3_TIM10;
    HAL_GPIO_Init(IMU_HEAT_GPIO_Port, &gpio);
}

void HAL_TIM_Base_MspDeInit(TIM_HandleTypeDef *tim_baseHandle)
{
    if (tim_baseHandle->Instance == TIM10) {
        __HAL_RCC_TIM10_CLK_DISABLE();
    }
}
