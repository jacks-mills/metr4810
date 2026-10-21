/**
 * @file    exti.c
 * @brief   GPIO interrupts: the start button (limit switches join in stage 3).
 */
#include "main.h"
#include "command.h"

#define BUTTON_DEBOUNCE_MS  250

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
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
