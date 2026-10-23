/**
 * @file    term.h
 * @brief   Buffered output to the terminal (terminal task only).
 *
 * Output is collected in a buffer and sent with term_flush(), once per pass
 * of the terminal task. That way a screen update goes out as one USB (or
 * DMA) transfer instead of dozens of small ones.
 */
#ifndef TERM_H
#define TERM_H

#include <stddef.h>

void term_write(const char *data, size_t len);
void term_puts(const char *s);
void term_printf(const char *fmt, ...) __attribute__((format(printf, 1, 2)));
void term_flush(void);

#endif /* TERM_H */
