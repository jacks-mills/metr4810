/**
 * @file    servo.h
 * @brief   ST3215 (Feetech STS) serial bus servo protocol. Rotate task only.
 *
 * Packets: FF FF id length instruction params... checksum, with
 * length = params + 2 and checksum = ~(id + length + instruction + params).
 * The servo answers FF FF id length error data... checksum. Two-byte
 * registers are little-endian.
 *
 * Positions are servo steps: 4096 per turn, SERVO_CENTRE straight ahead.
 */
#ifndef SERVO_H
#define SERVO_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#define SERVO_CENTRE        2048
#define SERVO_STEPS         4096    /* per turn */

/* error flags in every reply */
#define SERVO_ERR_VOLTAGE       0x01
#define SERVO_ERR_SENSOR        0x02
#define SERVO_ERR_TEMPERATURE   0x04
#define SERVO_ERR_CURRENT       0x08
#define SERVO_ERR_OVERLOAD      0x20

struct ServoStatus {
    int16_t position;       /* steps */
    int16_t speed;          /* steps/s */
    int16_t load;           /* tenths of a percent */
    uint8_t voltage;        /* tenths of a volt */
    uint8_t temperature;    /* deg C */
    bool moving;
};

void servo_init(void);

/* Each returns false if the servo didn't answer properly (after retries). */
bool servo_ping(void);
bool servo_read_status(struct ServoStatus *status);
bool servo_read_limits(uint16_t *min, uint16_t *max);
bool servo_read_mode(uint8_t *mode);
bool servo_move(uint16_t position, uint16_t speed, uint8_t accel);
bool servo_set_torque(bool on);

/* Saved in the servo's EEPROM, so they survive power-off. */
bool servo_set_centre(void);                        /* here becomes SERVO_CENTRE */
bool servo_write_limits(uint16_t min, uint16_t max);

/* Error flags from the last reply, and a readable list of them. */
uint8_t servo_error(void);
const char *servo_error_text(uint8_t flags);

/* Packet helpers, exposed for testing. */
size_t servo_packet(uint8_t *buf, uint8_t id, uint8_t instruction,
                    const uint8_t *params, size_t count);
bool servo_reply_ok(const uint8_t *reply, size_t len, uint8_t id, size_t dataLen);

#endif /* SERVO_H */
