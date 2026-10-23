/**
 * @file    drill.c
 * @brief   Drill motor (FIT0441) on TIM3 CH3.
 */
#include "config.h"
#if !HW_SIMULATED

#include "main.h"
#include "drill.h"

#define DRILL_CHANNEL   TIM_CHANNEL_3

extern TIM_HandleTypeDef htim3;

static bool enabled;

bool drill_init(void)
{
    __HAL_TIM_SET_COMPARE(&htim3, DRILL_CHANNEL, 0);
    HAL_GPIO_WritePin(DRILL_DIR_GPIO_Port, DRILL_DIR_Pin, DRILL_DIR_LEVEL ? GPIO_PIN_SET : GPIO_PIN_RESET);

    /* With polarity Low, compare 0 keeps the pin high = FIT0441 stopped.
     * With polarity High it would be low = full speed, so refuse. */
    enabled = (htim3.Instance->CCER & TIM_CCER_CC3P) != 0;
    if (enabled) {
        HAL_TIM_PWM_Start(&htim3, DRILL_CHANNEL);
    } else {
        /* take the pin off the timer and hold it high: drill stopped */
        HAL_GPIO_WritePin(DRILL_PWM_GPIO_Port, DRILL_PWM_Pin, GPIO_PIN_SET);
        GPIO_InitTypeDef pin = { .Pin = DRILL_PWM_Pin, .Mode = GPIO_MODE_OUTPUT_PP };
        HAL_GPIO_Init(DRILL_PWM_GPIO_Port, &pin);
    }
    return enabled;
}

bool drill_available(void)
{
    return enabled;
}

void drill_off(void)
{
    htim3.Instance->CCR3 = 0;
}

void drill_set(bool on, uint8_t speedPercent)
{
    if (!enabled || !on || speedPercent == 0) {
        drill_off();
        return;
    }
    if (speedPercent > 100)
        speedPercent = 100;
    uint32_t period = __HAL_TIM_GET_AUTORELOAD(&htim3) + 1;
    __HAL_TIM_SET_COMPARE(&htim3, DRILL_CHANNEL, period * speedPercent / 100);
}

#endif /* !HW_SIMULATED */
