/**
 * @file    status_led.h
 * @brief   Status LEDs: status_led_blackpill.c (PC13) in the Blackpill
 *          project, status_led_pcb.c (four LEDs) in the PCB project.
 */
#ifndef STATUS_LED_H
#define STATUS_LED_H

#include <stdbool.h>
#include <stdint.h>
#include "robot_state.h"

/* Called by the control task every CONTROL_PERIOD_MS. startReady: 'start'
 * would be accepted now (control_start_ready()). */
void status_led_show(enum Mode mode, bool startReady, uint32_t now);

/* Toggles the LEDs for fault_halt()'s fast flash. Only touches GPIO, so it's
 * safe with interrupts off. */
void status_led_fault_toggle(void);

#endif /* STATUS_LED_H */
