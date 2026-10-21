/**
 * @file    fault.c
 * @brief   Last-resort stop for faults the firmware can't recover from.
 */
#include "main.h"
#include "fault.h"

/* Inspect these in the debugger after a halt. */
volatile const char *faultReason;
volatile const char *faultDetail;

void fault_halt(const char *reason, const char *detail)
{
    __disable_irq();
    faultReason = reason;
    faultDetail = detail;

    /* Later stages: switch the drive and drill outputs off here, directly,
     * since no task will run again. */

    for (;;) {
        HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
        for (volatile uint32_t i = 0; i < 1000000; i++) {
            /* busy wait: interrupts are off, so no HAL_Delay */
        }
    }
}
