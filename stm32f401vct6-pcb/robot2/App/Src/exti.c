/**
 * @file    exti.c
 * @brief   GPIO interrupts: start button and arm limit switches.
 */
#include "main.h"
#include "command.h"
#include "drive_hw.h"

#define BUTTON_DEBOUNCE_MS  250

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    if (GPIO_Pin == EXT_MIN_Pin || GPIO_Pin == EXT_MAX_Pin) {
        /* cut the motor now if it's driving into this switch; the drive task
         * sees the switch on its next tick and reports */
        drive_hw_limit_isr(GPIO_Pin);
        return;
    }

    if (GPIO_Pin == START_BUTTON_Pin) {
        static uint32_t lastPress;
        uint32_t now = HAL_GetTick();

        if (now - lastPress < BUTTON_DEBOUNCE_MS)
            return;                     /* contact bounce */
        lastPress = now;

        struct Command cmd = { .type = CMD_SEQ_START, .source = SOURCE_BUTTON };
        (void)command_send(&cmd);
    }
}
