/**
 * @file    term_io_usb.h
 * @brief   Hooks the USB CDC driver (USB_DEVICE/App/usbd_cdc_if.c) calls.
 *
 * All three run in the USB interrupt.
 */
#ifndef TERM_IO_USB_H
#define TERM_IO_USB_H

#include <stdbool.h>
#include <stdint.h>

/* CDC_Init_FS: the PC has (re)configured the device. */
void term_io_usb_reset_isr(void);

/* CDC_Control_FS, CDC_SET_CONTROL_LINE_STATE: DTR goes high when a terminal
 * program opens the port and low when it closes it. */
void term_io_usb_line_state_isr(bool dtr);

/* CDC_Receive_FS: bytes typed in the terminal. */
void term_io_usb_receive_isr(const uint8_t *data, uint32_t len);

#endif /* TERM_IO_USB_H */
