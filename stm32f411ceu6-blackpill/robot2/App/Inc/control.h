/**
 * @file    control.h
 * @brief   Control logic: modes, command rules, safety interlock, sequence.
 *
 * task_control.c runs these. They make no RTOS calls of their own (only
 * through drive_send(), rotate_send(), drill_set() and log_...), so the
 * whole thing can be tested on a PC.
 */
#ifndef CONTROL_H
#define CONTROL_H

#include <stdbool.h>
#include <stdint.h>
#include "command.h"

#define CONTROL_PERIOD_MS   50

void control_init(uint32_t now);
void control_command(const struct Command *cmd, uint32_t now);   /* commands and events */
void control_tick(uint32_t now);                                /* every CONTROL_PERIOD_MS */

/* True when 'start' (or the start button) would be accepted now: not
 * already running, and nothing blocking it. Shown on the PCB's start LED. */
bool control_start_ready(void);

#endif /* CONTROL_H */
