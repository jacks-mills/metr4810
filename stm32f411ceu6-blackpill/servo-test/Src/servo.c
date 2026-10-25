/**
 * @file    servo.c
 * @brief   Feetech STS packets: build a request, check the reply.
 *
 * Request: FF FF id len instr params... checksum
 * Reply:   FF FF id len error params... checksum
 * len = number of params + 2. checksum = ~(id + len + instr/error + params).
 *
 * No hardware access here; servo_uart.c moves the bytes.
 */
#include <string.h>

#include "servo.h"

#define INSTR_PING          0x01u
#define INSTR_READ          0x02u
#define INSTR_WRITE         0x03u

#define REPLY_TIMEOUT_US    3000u   /* from the end of the request to the start of the reply */

uint8_t servoLastRx[64];
uint32_t servoLastRxLen;

static enum ServoResult command(uint8_t id, uint8_t instr, const uint8_t *params, uint8_t n,
                                uint8_t *data, uint8_t dataLen, uint8_t *error, uint32_t timeoutUs)
{
    uint8_t tx[6 + SERVO_MAX_DATA];
    if (n > SERVO_MAX_DATA)
        return SERVO_BAD_REPLY;

    uint8_t len = (uint8_t)(n + 2u);
    uint8_t sum = (uint8_t)(id + len + instr);
    tx[0] = 0xFF;
    tx[1] = 0xFF;
    tx[2] = id;
    tx[3] = len;
    tx[4] = instr;
    for (uint8_t i = 0; i < n; i++) {
        tx[5 + i] = params[i];
        sum = (uint8_t)(sum + params[i]);
    }
    tx[5 + n] = (uint8_t)~sum;

    servoLastRxLen = servo_port_exchange(tx, 6u + n, servoLastRx, sizeof servoLastRx, timeoutUs);
    if (servoLastRxLen == 0)
        return SERVO_NO_REPLY;

    /* Find the header (skipping anything before it), then check it all. */
    const uint8_t *r = servoLastRx;
    uint32_t left = servoLastRxLen;
    while (left >= 2 && !(r[0] == 0xFF && r[1] == 0xFF)) {
        r++;
        left--;
    }
    while (left >= 3 && r[2] == 0xFF) {     /* extra FF before the id */
        r++;
        left--;
    }
    if (left < 6)
        return SERVO_BAD_REPLY;
    uint8_t rlen = r[3];
    if (r[2] != id || rlen < 2 || left < 4u + rlen || rlen - 2u != dataLen)
        return SERVO_BAD_REPLY;
    sum = 0;
    for (uint32_t i = 2; i < 3u + rlen; i++)
        sum = (uint8_t)(sum + r[i]);
    if ((uint8_t)~sum != r[3 + rlen])
        return SERVO_BAD_REPLY;

    if (error)
        *error = r[4];
    if (dataLen)
        memcpy(data, &r[5], dataLen);
    return SERVO_OK;
}

enum ServoResult servo_ping(uint8_t id, uint32_t timeoutUs, uint8_t *error)
{
    return command(id, INSTR_PING, NULL, 0, NULL, 0, error, timeoutUs);
}

enum ServoResult servo_read(uint8_t id, uint8_t addr, uint8_t len, uint8_t *out, uint8_t *error)
{
    if (len == 0 || len > SERVO_MAX_DATA)
        return SERVO_BAD_REPLY;
    uint8_t params[2] = {addr, len};
    return command(id, INSTR_READ, params, 2, out, len, error, REPLY_TIMEOUT_US);
}

enum ServoResult servo_write(uint8_t id, uint8_t addr, const uint8_t *data, uint8_t len, uint8_t *error)
{
    uint8_t params[1 + SERVO_MAX_DATA];
    if (len == 0 || len + 1u > sizeof params)
        return SERVO_BAD_REPLY;
    params[0] = addr;
    memcpy(&params[1], data, len);
    return command(id, INSTR_WRITE, params, (uint8_t)(len + 1u), NULL, 0, error, REPLY_TIMEOUT_US);
}
