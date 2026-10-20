#include <stdio.h>
#include <stdbool.h>

#include "main.h"
#include "cmsis_os.h"
#include "usb_device.h"
#include "robot_state.h"
#include "task_default.h"

void TaskDefault(void *argument)
{
    struct Params *params = (struct Params *)argument;
    struct State *state = params->state;
    (void)params;

    for (;;)
    {
        HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_SET);
        state->ledGreenIsOn = true;
        osDelay(2000);

        HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);
        state->ledGreenIsOn = false;
        osDelay(2000);
    }
}
