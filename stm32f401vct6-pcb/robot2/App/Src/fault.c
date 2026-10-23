/**
 * @file    fault.c
 * @brief   Last-resort stop for faults the firmware can't recover from
 *          (stack overflow, heap full, hard fault).
 */
#include "main.h"
#include "drill.h"
#include "drive_hw.h"
#include "fault.h"
#include "status_led.h"

/* Inspect these in the debugger after a halt. */
volatile const char *faultReason;
volatile const char *faultDetail;

void fault_halt(const char *reason, const char *detail)
{
    __disable_irq();

    /* no task will run again, so switch the motors off directly */
    drive_hw_off();
    drill_off();

    faultReason = reason;
    faultDetail = detail;

    for (;;) {
        status_led_fault_toggle();
        for (volatile uint32_t i = 0; i < 250000; i++) {   /* ~20 Hz */
            /* busy wait: interrupts are off, so no HAL_Delay */
        }
    }
}
