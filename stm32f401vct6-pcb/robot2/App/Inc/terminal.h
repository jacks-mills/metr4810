/**
 * @file    terminal.h
 * @brief   Terminal logic: line editing, keys, refresh timing.
 *
 * task_terminal.c runs these in a loop. They take the time as an argument and
 * make no RTOS calls, so the whole terminal can be tested on a PC.
 */
#ifndef TERMINAL_H
#define TERMINAL_H

#include <stdint.h>

void terminal_init(void);

/* A terminal program just opened the port. */
void terminal_connected(uint32_t now);

/* The port was closed. */
void terminal_disconnected(void);

/* One byte of input. */
void terminal_input(uint8_t c, uint32_t now);

/* Timed work: Esc timeout, log messages, screen refresh. */
void terminal_poll(uint32_t now);

/* How long the task may wait for input before terminal_poll() has work. */
uint32_t terminal_wait_time(uint32_t now);

#endif /* TERMINAL_H */
