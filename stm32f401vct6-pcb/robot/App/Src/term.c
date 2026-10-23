/**
 * @file    term.c
 * @brief   Buffered output to the terminal (terminal task only).
 */
#include <stdarg.h>
#include <stdio.h>
#include <string.h>

#include "term.h"
#include "term_io.h"

#define TERM_BUFFER_SIZE    1024

static char buffer[TERM_BUFFER_SIZE];
static size_t used;

void term_flush(void)
{
    if (used > 0) {
        term_io_write(buffer, used);
        used = 0;
    }
}

void term_write(const char *data, size_t len)
{
    while (len > 0) {
        if (used == sizeof buffer)
            term_flush();
        size_t n = sizeof buffer - used;
        if (n > len)
            n = len;
        memcpy(buffer + used, data, n);
        used += n;
        data += n;
        len -= n;
    }
}

void term_puts(const char *s)
{
    term_write(s, strlen(s));
}

void term_printf(const char *fmt, ...)
{
    char text[160];
    va_list args;

    va_start(args, fmt);
    int n = vsnprintf(text, sizeof text, fmt, args);
    va_end(args);

    if (n < 0)
        return;
    if ((size_t)n >= sizeof text)
        n = sizeof text - 1;
    term_write(text, (size_t)n);
}
