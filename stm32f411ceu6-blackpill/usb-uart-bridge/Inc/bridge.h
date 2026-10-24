/**
 * @file    bridge.h
 * @brief   Moves bytes between the USB virtual COM port and USART1.
 *
 * The *_isr functions are called from the USB and UART interrupts;
 * bridge_poll() is called over and over from the main loop.
 */
#ifndef BRIDGE_H
#define BRIDGE_H

#include <stdbool.h>
#include <stdint.h>

/* Counters for the debugger (watch bridgeStats). */
struct BridgeStats {
    uint32_t toUartDropped;     /* PC -> target bytes lost (only after a USB reset) */
    uint32_t toUsbDropped;      /* target -> PC bytes lost: the PC stopped reading */
    uint32_t codingRejected;    /* line codings USART1 can't do (kept the old one) */
};
extern volatile struct BridgeStats bridgeStats;

void bridge_init(void);
void bridge_poll(void);

/* ---- USB interrupt (usbd_cdc_if.c) --------------------------------------- */

/* The device was (re)configured or unconfigured by the PC. */
void bridge_usb_reset_isr(void);

/* SET_CONTROL_LINE_STATE: a terminal raises DTR when it opens the port. */
void bridge_usb_line_state_isr(bool dtr);

/* One OUT packet from the PC. True if the OUT endpoint may be re-armed now;
 * false holds the PC off until bridge_poll() re-arms it. */
bool bridge_usb_receive_isr(const uint8_t *data, uint32_t len);

/* SET_LINE_CODING / GET_LINE_CODING: the 7-byte CDC line coding structure
 * (baud rate LSB first, stop bits, parity, data bits). */
void bridge_usb_set_line_coding_isr(const uint8_t *coding);
void bridge_usb_get_line_coding_isr(uint8_t *coding);

/* ---- UART interrupt (uart.c) --------------------------------------------- */

/* Bytes received from the target. */
void bridge_uart_receive_isr(const uint8_t *data, uint32_t len);

#endif /* BRIDGE_H */
