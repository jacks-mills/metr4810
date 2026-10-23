/**
 * @file    drive_hw.c
 * @brief   Arm drive motor and limit switches, at the pin level.
 */
#include "config.h"
#if !HW_SIMULATED

#include "main.h"
#include "drive_hw.h"

#define DRIVE_CHANNEL   TIM_CHANNEL_2

extern TIM_HandleTypeDef htim3;

/* direction currently being driven; read by the limit switch interrupt */
static volatile enum DriveMotion driven = DRIVE_STOPPED;

static bool pressed(GPIO_TypeDef *port, uint16_t pin)
{
    GPIO_PinState level = HAL_GPIO_ReadPin(port, pin);
    return LIMIT_ACTIVE_HIGH ? (level == GPIO_PIN_SET) : (level == GPIO_PIN_RESET);
}

void drive_hw_init(void)
{
    __HAL_TIM_SET_COMPARE(&htim3, DRIVE_CHANNEL, 0);
    HAL_TIM_PWM_Start(&htim3, DRIVE_CHANNEL);
    driven = DRIVE_STOPPED;
}

void drive_hw_off(void)
{
    htim3.Instance->CCR2 = 0;           /* one register write: safe anywhere */
    driven = DRIVE_STOPPED;
}

void drive_hw_set(enum DriveMotion motion, uint8_t dutyPercent)
{
    if (motion == DRIVE_STOPPED || dutyPercent == 0) {
        drive_hw_off();
        return;
    }
    if (dutyPercent > 100)
        dutyPercent = 100;

    /* don't drive into a switch that's already pressed */
    if ((motion == DRIVE_EXTENDING && drive_hw_at_max()) ||
        (motion == DRIVE_RETRACTING && drive_hw_at_min())) {
        drive_hw_off();
        return;
    }

    bool extend = (motion == DRIVE_EXTENDING);
    GPIO_PinState level = (extend == (DRIVE_DIR_EXTEND != 0)) ? GPIO_PIN_SET : GPIO_PIN_RESET;
    HAL_GPIO_WritePin(DRIVE_DIR_GPIO_Port, DRIVE_DIR_Pin, level);

    uint32_t period = __HAL_TIM_GET_AUTORELOAD(&htim3) + 1;
    driven = motion;
    __HAL_TIM_SET_COMPARE(&htim3, DRIVE_CHANNEL, period * dutyPercent / 100);
}

bool drive_hw_at_min(void)
{
    return pressed(EXT_MIN_GPIO_Port, EXT_MIN_Pin);
}

bool drive_hw_at_max(void)
{
    return pressed(EXT_MAX_GPIO_Port, EXT_MAX_Pin);
}

void drive_hw_limit_isr(uint16_t pin)
{
    if ((pin == EXT_MAX_Pin && driven == DRIVE_EXTENDING && drive_hw_at_max()) ||
        (pin == EXT_MIN_Pin && driven == DRIVE_RETRACTING && drive_hw_at_min()))
        drive_hw_off();
}

#endif /* !HW_SIMULATED */
