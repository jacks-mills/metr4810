/**
 * @file    usb_syscalls.h
 * @brief   stdio (printf / scanf / getchar ...) over USB CDC.
 */
#ifndef USB_SYSCALLS_H
#define USB_SYSCALLS_H

#include <stdint.h>

/* Call once before MX_USB_DEVICE_Init(). Creates the receive buffer. */
void usb_syscalls_init(void);

/* Called from CDC_Receive_FS (USB interrupt) with each received packet. */
void usb_syscalls_rx_isr(const uint8_t *buf, uint32_t len);

#endif /* USB_SYSCALLS_H */
