/*
 * ventilador.cpp
 *
 *  Created on: Jun 22, 2025
 *      Author: davim
 */
#include "ventilador.h"
#include "stm32g4xx_hal_tim.h"

#include "stm32g4xx_hal.h"

static void GPIO_PC2_PWM_Init(void);
static void TIM1_PWM_Init(void);

TIM_HandleTypeDef timer1_pwm_handle;

void ventiladorInit(void)
{
    GPIO_PC2_PWM_Init();
    TIM1_PWM_Init();
    HAL_TIM_PWM_Start(&timer1_pwm_handle, TIM_CHANNEL_3);
    ventiladorSetDutyCycle(50.0f); // Default 50% duty
}

void ventiladorSetDutyCycle(float duty_cycle_percent)
{// fazer limitação
    uint32_t auto_reload = __HAL_TIM_GET_AUTORELOAD(&timer1_pwm_handle);
    uint32_t compare_value = (duty_cycle_percent / 100.0f) * auto_reload;
    __HAL_TIM_SET_COMPARE(&timer1_pwm_handle, TIM_CHANNEL_3, compare_value);
}

static void GPIO_PC2_PWM_Init(void)
{
    __HAL_RCC_GPIOC_CLK_ENABLE();

    GPIO_InitTypeDef gpio_pwm_pc2 = {0};
    gpio_pwm_pc2.Pin = GPIO_PIN_2;
    gpio_pwm_pc2.Mode = GPIO_MODE_AF_PP;
    gpio_pwm_pc2.Pull = GPIO_NOPULL;
    gpio_pwm_pc2.Speed = GPIO_SPEED_FREQ_LOW;
    gpio_pwm_pc2.Alternate = GPIO_AF2_TIM1;

    HAL_GPIO_Init(GPIOC, &gpio_pwm_pc2);
}

static void TIM1_PWM_Init(void)
{
    __HAL_RCC_TIM1_CLK_ENABLE();

    timer1_pwm_handle.Instance = TIM1;
    timer1_pwm_handle.Init.Prescaler = 71;
    timer1_pwm_handle.Init.CounterMode = TIM_COUNTERMODE_UP;
    timer1_pwm_handle.Init.Period = 999;
    timer1_pwm_handle.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    timer1_pwm_handle.Init.RepetitionCounter = 0;

    HAL_TIM_PWM_Init(&timer1_pwm_handle);

    TIM_OC_InitTypeDef pwm_channel_config = {0};
    pwm_channel_config.OCMode = TIM_OCMODE_PWM1;
    pwm_channel_config.Pulse = 0;
    pwm_channel_config.OCPolarity = TIM_OCPOLARITY_HIGH;
    pwm_channel_config.OCFastMode = TIM_OCFAST_DISABLE;

    HAL_TIM_PWM_ConfigChannel(&timer1_pwm_handle, &pwm_channel_config, TIM_CHANNEL_3);
}

