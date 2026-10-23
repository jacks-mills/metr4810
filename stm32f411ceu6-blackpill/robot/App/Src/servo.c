/**
 * @file    servo.c
 * @brief   ST3215 (Feetech STS) serial bus servo protocol.
 *
 * Register addresses are from Feetech's STS memory table (the same ones the
 * SCServo library uses for the STS series).
 */
#include <string.h>

#include "config.h"
#include "servo.h"
#include "servo_uart.h"

#define INST_PING           0x01
#define INST_READ           0x02
#define INST_WRITE          0x03

#define REG_MIN_LIMIT       9       /* 2 bytes, EEPROM */
#define REG_MAX_LIMIT       11      /* 2 bytes, EEPROM */
#define REG_MODE            33      /* 0 = position mode */
#define REG_TORQUE          40      /* 0 off, 1 on, 128 = make this position the centre */
#define REG_ACCEL           41      /* then goal position (42), time (44), speed (46) */
#define REG_LOCK            55      /* 0 = EEPROM writes are saved, 1 = locked */
#define REG_STATUS          56      /* position 56, speed 58, load 60, voltage 62,
                                       temperature 63, moving 66 */
#define STATUS_LEN          11

#define TORQUE_CENTRE       128
#define RETRIES             3
#define EEPROM_SETTLE_MS    20

static uint8_t lastError;

/* ------------------------------------------------------------------------- */
/* Packets                                                                    */
/* ------------------------------------------------------------------------- */

static uint8_t checksum(const uint8_t *bytes, size_t count)
{
    uint8_t sum = 0;
    for (size_t i = 0; i < count; i++)
        sum += bytes[i];
    return (uint8_t)~sum;
}

size_t servo_packet(uint8_t *buf, uint8_t id, uint8_t instruction,
                    const uint8_t *params, size_t count)
{
    buf[0] = 0xFF;
    buf[1] = 0xFF;
    buf[2] = id;
    buf[3] = (uint8_t)(count + 2);
    buf[4] = instruction;
    if (count > 0)
        memcpy(buf + 5, params, count);
    buf[5 + count] = checksum(buf + 2, count + 3);
    return count + 6;
}

bool servo_reply_ok(const uint8_t *reply, size_t len, uint8_t id, size_t dataLen)
{
    return len == dataLen + 6
        && reply[0] == 0xFF && reply[1] == 0xFF
        && reply[2] == id
        && reply[3] == dataLen + 2
        && reply[len - 1] == checksum(reply + 2, len - 3);
}

/* sign-magnitude, as the servo sends it */
static int16_t decode(const uint8_t *p, int signBit)
{
    uint16_t raw = (uint16_t)(p[0] | (p[1] << 8));
    uint16_t sign = (uint16_t)(1u << signBit);
    return (raw & sign) ? (int16_t)-(int16_t)(raw & ~sign) : (int16_t)raw;
}

/* One instruction and its reply, with retries. */
static bool transaction(uint8_t instruction, const uint8_t *params, size_t count,
                        uint8_t *data, size_t dataLen)
{
    uint8_t tx[24];
    uint8_t rx[24];

    if (count + 6 > sizeof tx || dataLen + 6 > sizeof rx)
        return false;
    size_t txLen = servo_packet(tx, SERVO_ID, instruction, params, count);

    for (int attempt = 0; attempt < RETRIES; attempt++) {
        size_t got = servo_uart_transfer(tx, txLen, rx, dataLen + 6);
        if (servo_reply_ok(rx, got, SERVO_ID, dataLen)) {
            lastError = rx[4];
            if (dataLen > 0)
                memcpy(data, rx + 5, dataLen);
            return true;
        }
    }
    return false;
}

static bool read_regs(uint8_t address, uint8_t *data, uint8_t len)
{
    uint8_t params[2] = { address, len };
    return transaction(INST_READ, params, sizeof params, data, len);
}

static bool write_regs(uint8_t address, const uint8_t *data, size_t len)
{
    uint8_t params[16];
    if (len + 1 > sizeof params)
        return false;
    params[0] = address;
    memcpy(params + 1, data, len);
    return transaction(INST_WRITE, params, len + 1, NULL, 0);
}

static bool write_byte(uint8_t address, uint8_t value)
{
    return write_regs(address, &value, 1);
}

/* ------------------------------------------------------------------------- */
/* Public                                                                     */
/* ------------------------------------------------------------------------- */

void servo_init(void)
{
    servo_uart_init();
}

bool servo_ping(void)
{
    return transaction(INST_PING, NULL, 0, NULL, 0);
}

bool servo_read_status(struct ServoStatus *s)
{
    uint8_t d[STATUS_LEN];
    if (!read_regs(REG_STATUS, d, sizeof d))
        return false;
    s->position = decode(&d[0], 15);
    s->speed = decode(&d[2], 15);
    s->load = decode(&d[4], 10);
    s->voltage = d[6];
    s->temperature = d[7];
    s->moving = d[10] != 0;
    return true;
}

bool servo_read_limits(uint16_t *min, uint16_t *max)
{
    uint8_t d[4];
    if (!read_regs(REG_MIN_LIMIT, d, sizeof d))
        return false;
    *min = (uint16_t)(d[0] | (d[1] << 8));
    *max = (uint16_t)(d[2] | (d[3] << 8));
    return true;
}

bool servo_read_mode(uint8_t *mode)
{
    return read_regs(REG_MODE, mode, 1);
}

bool servo_move(uint16_t position, uint16_t speed, uint8_t accel)
{
    uint8_t d[7] = {
        accel,
        (uint8_t)position, (uint8_t)(position >> 8),
        0, 0,                                   /* time: unused, speed rules */
        (uint8_t)speed, (uint8_t)(speed >> 8),
    };
    return write_regs(REG_ACCEL, d, sizeof d);
}

bool servo_set_torque(bool on)
{
    return write_byte(REG_TORQUE, on ? 1 : 0);
}

bool servo_set_centre(void)
{
    bool ok = write_byte(REG_LOCK, 0)
           && write_byte(REG_TORQUE, TORQUE_CENTRE);
    servo_uart_delay_ms(EEPROM_SETTLE_MS);
    return write_byte(REG_LOCK, 1) && ok;
}

bool servo_write_limits(uint16_t min, uint16_t max)
{
    uint8_t d[4] = { (uint8_t)min, (uint8_t)(min >> 8), (uint8_t)max, (uint8_t)(max >> 8) };
    bool ok = write_byte(REG_LOCK, 0)
           && write_regs(REG_MIN_LIMIT, d, sizeof d);
    servo_uart_delay_ms(EEPROM_SETTLE_MS);
    return write_byte(REG_LOCK, 1) && ok;
}

uint8_t servo_error(void)
{
    return lastError;
}

const char *servo_error_text(uint8_t flags)
{
    static char text[64];
    static const struct { uint8_t bit; const char *name; } names[] = {
        { SERVO_ERR_VOLTAGE, "voltage" },
        { SERVO_ERR_SENSOR, "angle sensor" },
        { SERVO_ERR_TEMPERATURE, "overheated" },
        { SERVO_ERR_CURRENT, "overcurrent" },
        { SERVO_ERR_OVERLOAD, "overload" },
    };

    text[0] = '\0';
    for (size_t i = 0; i < sizeof names / sizeof names[0]; i++) {
        if (flags & names[i].bit) {
            if (text[0] != '\0')
                strncat(text, ", ", sizeof text - strlen(text) - 1);
            strncat(text, names[i].name, sizeof text - strlen(text) - 1);
        }
    }
    if (text[0] == '\0')
        return "none";
    return text;
}
