/**
 * @file    bridge_port.h
 * @brief   What bridge.c needs from the board. Kept small so bridge.c can be
 *          tested on a PC with fakes (test/bridge_test.c).
 */
#ifndef BRIDGE_PORT_H
#define BRIDGE_PORT_H

#include <stdbool.h>
#include <stdint.h>

/* ---- main.c -------------------------------------------------------------- */
uint32_t board_millis(void);
void board_led(bool on);

/* ---- usbd_cdc_if.c: the USB virtual COM port ------------------------------ */
bool vcp_configured(void);                              /* enumerated and configured by the PC */
bool vcp_tx_idle(void);                                 /* IN endpoint free for the next transfer */
bool vcp_tx_start(const uint8_t *data, uint32_t len);   /* data must stay put until vcp_tx_idle() */
void vcp_rx_resume(void);                               /* re-arm the OUT endpoint */

/* ---- uart.c: USART1 ------------------------------------------------------ */
void uart_init(void);                                   /* 115200 8N1 */
bool uart_tx_busy(void);                                /* DMA still reading the last block */
bool uart_tx_complete(void);                            /* last stop bit has left the pin */
void uart_tx_start(const uint8_t *data, uint32_t len);  /* data must stay put until !uart_tx_busy() */

/* parity: 0 none, 1 odd, 2 even. stopBits: 0 = 1, 1 = 1.5, 2 = 2 (CDC
 * coding). False, and nothing changed, if USART1 can't do it. */
bool uart_set_format(uint32_t baud, uint8_t dataBits, uint8_t parity, uint8_t stopBits);

#endif /* BRIDGE_PORT_H */
