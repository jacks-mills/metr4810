/**
 * @file    vcp.h
 * @brief   The USB serial port (CDC) as a text console.
 */
#ifndef VCP_H
#define VCP_H

#include <stdbool.h>
#include <stddef.h>

bool vcp_open(void);            /* configured, and a terminal has the port open (DTR) */
bool vcp_just_opened(void);     /* true once each time a terminal opens the port */
int vcp_getc(void);             /* next byte typed, or -1 */
void vcp_write(const char *s, size_t len);  /* dropped if no terminal is open */
void vcp_puts(const char *s);
void vcp_poll(void);            /* call often: sends queued text, takes more input */

#endif /* VCP_H */
