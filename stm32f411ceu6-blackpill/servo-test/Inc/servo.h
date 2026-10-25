/**
 * @file    servo.h
 * @brief   Feetech STS protocol (ST3215 / SER0070) over a half-duplex UART.
 */
#ifndef SERVO_H
#define SERVO_H

#include <stdbool.h>
#include <stdint.h>

#define SERVO_MAX_DATA      58u     /* most bytes one read can return */

enum ServoResult {
    SERVO_OK = 0,
    SERVO_NO_REPLY,                 /* nothing came back */
    SERVO_BAD_REPLY,                /* something came back but it isn't a valid reply */
};

/* Registers (STS series). Two-byte values are little-endian. */
enum {
    REG_MODEL           = 3,        /* 2 bytes */
    REG_ID              = 5,
    REG_BAUD            = 6,
    REG_MIN_ANGLE       = 9,        /* 2 bytes */
    REG_MAX_ANGLE       = 11,       /* 2 bytes */
    REG_OFFSET          = 31,       /* 2 bytes, bit 11 = sign */
    REG_MODE            = 33,       /* 0 position, 1 wheel, 2 PWM, 3 step */
    REG_TORQUE          = 40,       /* 0 off, 1 on, 128 = make here the middle */
    REG_ACC             = 41,       /* x100 steps/s^2, 0 = fastest */
    REG_GOAL_POS        = 42,       /* 2 bytes */
    REG_GOAL_TIME       = 44,       /* 2 bytes */
    REG_GOAL_SPEED      = 46,       /* 2 bytes, steps/s */
    REG_LOCK            = 55,       /* 0 = EEPROM writable, 1 = locked */
    REG_PRESENT_POS     = 56,       /* 2 bytes, bit 15 = sign */
    REG_PRESENT_SPEED   = 58,       /* 2 bytes, bit 15 = sign */
    REG_PRESENT_LOAD    = 60,       /* 2 bytes, bit 10 = sign, 0.1 % */
    REG_VOLTAGE         = 62,       /* 0.1 V */
    REG_TEMPERATURE     = 63,       /* deg C */
    REG_STATUS          = 65,       /* fault bits */
    REG_MOVING          = 66,
    REG_CURRENT         = 69,       /* 2 bytes, bit 15 = sign, 6.5 mA */
};

/* error: the status byte from the servo's reply (0 = no faults). */
enum ServoResult servo_ping(uint8_t id, uint32_t timeoutUs, uint8_t *error);
enum ServoResult servo_read(uint8_t id, uint8_t addr, uint8_t len, uint8_t *out, uint8_t *error);
enum ServoResult servo_write(uint8_t id, uint8_t addr, const uint8_t *data, uint8_t len, uint8_t *error);

/* The raw bytes of the last reply, for showing what came back when it
 * wasn't valid. */
extern uint8_t servoLastRx[64];
extern uint32_t servoLastRxLen;

/* ---- provided by servo_uart.c (or a fake, for testing on a PC) ---------- */

void servo_port_init(void);

/* Sends tx, then collects the reply into rx until the line has been quiet
 * for a moment after it, rxMax bytes arrive, or nothing has started within
 * timeoutUs. Returns the number of bytes received. */
uint32_t servo_port_exchange(const uint8_t *tx, uint32_t txLen, uint8_t *rx, uint32_t rxMax,
                             uint32_t timeoutUs);

#endif /* SERVO_H */
