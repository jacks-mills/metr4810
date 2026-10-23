/**
 * @file    servo_sim.c
 * @brief   A virtual ST3215 on the end of the servo link (HW_SIMULATED builds,
 *          and PC tests).
 *
 * Same functions as servo_uart.c. Packets are checked and answered the way
 * the real servo does, from a register table, and the position moves toward
 * the goal at the goal speed. It starts like a new servo: limits 0-4095
 * (not set yet), slightly off centre.
 */
#include "config.h"
#if HW_SIMULATED

#include <string.h>

#include "cmsis_os.h"
#include "servo_uart.h"

#define REGS            80
#define SIM_START       2063        /* ~ +1.3 deg: as if never centred */

static uint8_t reg[REGS];
static int32_t position;            /* raw encoder position */
static int32_t offset;              /* what "centre" calibration subtracts */
static uint32_t lastUpdate;

static uint16_t get16(int a)            { return (uint16_t)(reg[a] | (reg[a + 1] << 8)); }
static void put16(int a, uint16_t v)    { reg[a] = (uint8_t)v; reg[a + 1] = (uint8_t)(v >> 8); }

static void sim_update(void)
{
    uint32_t now = osKernelGetTickCount();
    uint32_t elapsed = now - lastUpdate;
    lastUpdate = now;

    int32_t present = position - offset;
    int32_t goal = get16(42);
    int32_t lo = get16(9), hi = get16(11);
    if (goal < lo) goal = lo;
    if (goal > hi) goal = hi;

    if (reg[40] == 1) {                         /* torque on: move toward the goal */
        int32_t speed = get16(46) ? get16(46) : 3000;
        int32_t step = speed * (int32_t)elapsed / 1000;
        if (step < 1 && elapsed > 0)
            step = 1;
        int32_t error = goal - present;
        if (error > step) error = step;
        if (error < -step) error = -step;
        position += error;
        present = position - offset;
    }
    put16(56, (uint16_t)present);
    reg[66] = (reg[40] == 1 && present != goal) ? 1 : 0;
    reg[60] = reg[66] ? 120 : 0;                /* 12.0 % load while moving */
    reg[61] = 0;
}

void servo_uart_init(void)
{
    memset(reg, 0, sizeof reg);
    reg[5] = SERVO_ID;
    put16(9, 0);
    put16(11, 4095);
    reg[33] = 0;                                /* position mode */
    reg[40] = 0;                                /* torque off at power-up */
    reg[55] = 1;
    reg[62] = 121;                              /* 12.1 V */
    reg[63] = 31;                               /* deg C */
    position = SIM_START;
    offset = 0;
    put16(42, SIM_START);
    lastUpdate = osKernelGetTickCount();
    sim_update();
}

static uint8_t checksum(const uint8_t *b, size_t n)
{
    uint8_t sum = 0;
    for (size_t i = 0; i < n; i++)
        sum += b[i];
    return (uint8_t)~sum;
}

/* Builds a reply in rx. Returns its length. */
static size_t reply(uint8_t *rx, size_t rxSize, const uint8_t *data, size_t n)
{
    uint8_t buf[40];
    buf[0] = 0xFF;
    buf[1] = 0xFF;
    buf[2] = SERVO_ID;
    buf[3] = (uint8_t)(n + 2);
    buf[4] = 0;                                 /* no error */
    if (n > 0)
        memcpy(buf + 5, data, n);
    buf[5 + n] = checksum(buf + 2, n + 3);
    size_t len = n + 6;
    if (len > rxSize)
        len = rxSize;
    memcpy(rx, buf, len);
    return len;
}

size_t servo_uart_transfer(const uint8_t *tx, size_t txLen, uint8_t *rx, size_t rxSize)
{
    if (txLen < 6 || tx[0] != 0xFF || tx[1] != 0xFF || tx[2] != SERVO_ID ||
        tx[3] != txLen - 4 || tx[txLen - 1] != checksum(tx + 2, txLen - 3))
        return 0;                               /* the real servo ignores bad packets */

    sim_update();
    uint8_t instruction = tx[4];
    const uint8_t *p = tx + 5;
    size_t n = txLen - 6;

    switch (instruction) {
    case 0x01:                                  /* ping */
        return reply(rx, rxSize, NULL, 0);

    case 0x02:                                  /* read */
        if (n != 2 || p[0] + p[1] > REGS)
            return 0;
        return reply(rx, rxSize, &reg[p[0]], p[1]);

    case 0x03: {                                /* write */
        if (n < 2 || p[0] + n - 1 > REGS)
            return 0;
        uint8_t address = p[0];
        if (address == 40 && p[1] == 128) {     /* make this position the centre */
            offset = position - 2048;
            put16(42, 2048);
        } else {
            memcpy(&reg[address], p + 1, n - 1);
        }
        sim_update();
        return reply(rx, rxSize, NULL, 0);
    }

    default:
        return 0;
    }
}

void servo_uart_delay_ms(uint32_t ms)
{
    osDelay(ms);
}

#endif /* HW_SIMULATED */
