/**
 * @file    servo_uart.h
 * @brief   Byte transport to the servo: USART2 in single-wire (half-duplex)
 *          mode on PA2. Rotate task only.
 *
 * With HW_SIMULATED, servo_sim.c provides these instead, with a virtual
 * ST3215 on the other end.
 */
#ifndef SERVO_UART_H
#define SERVO_UART_H

#include <stddef.h>
#include <stdint.h>

void servo_uart_init(void);

/* Sends tx, then collects the servo's reply into rx (at most rxSize bytes).
 * Our own bytes, which the receiver also hears on the shared wire, are
 * stripped off. Returns the number of reply bytes received (0 if the servo
 * didn't answer). Takes at most about 1.5 ms. */
size_t servo_uart_transfer(const uint8_t *tx, size_t txLen, uint8_t *rx, size_t rxSize);

/* Waits (the servo needs a moment after EEPROM writes). */
void servo_uart_delay_ms(uint32_t ms);

#endif /* SERVO_UART_H */
