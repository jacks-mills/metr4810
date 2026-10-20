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
    (void)params;

    for (;;)
    {
        HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_SET);
        params->state->ledGreenIsOn = true;
        (void) osSemaphoreRelease(params->updateUISemHandle);
        osDelay(3000);

        HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);
        params->state->ledGreenIsOn = false;
        (void) osSemaphoreRelease(params->updateUISemHandle);
        osDelay(3000);
    }
}
