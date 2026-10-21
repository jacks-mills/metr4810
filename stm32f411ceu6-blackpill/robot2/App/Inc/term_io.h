/**
 * @file    term_io.h
 * @brief   Byte transport under the terminal.
 *
 * The terminal code only talks to these functions, so moving from USB CDC
 * (Blackpill, term_io_usb.c) to USART1 + USB-serial adapter (PCB) means
 * writing one new file with the same functions.
 *
 * Only the terminal task calls these (except term_io_init).
 */
#ifndef TERM_IO_H
#define TERM_IO_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

/* Called once from main() before the transport is started. */
void term_io_init(void);

/* Sends bytes. Silently drops them when no terminal is open, or when the PC
 * stops reading, so a missing terminal never holds anything up. */
void term_io_write(const char *data, size_t len);

/* Waits up to timeoutTicks for input; returns how many bytes were read
 * (0 on timeout). */
size_t term_io_read(char *buf, size_t size, uint32_t timeoutTicks);

/* True while a terminal program has the port open. */
bool term_io_connected(void);

/* True once each time a terminal program opens the port (redraw then). */
bool term_io_take_connect_event(void);

#endif /* TERM_IO_H */
