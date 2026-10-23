/**
 * @file    sim.c
 * @brief   Simulated arm drive and drill (HW_SIMULATED builds only).
 *
 * Same functions as drive_hw.c and drill.c. The arm position is worked out
 * from how long it has been driven at what speed; the limit switches read
 * "pressed" at the ends of travel. The servo is simulated separately, at the
 * byte level, in servo_sim.c.
 */
#include "config.h"
#if HW_SIMULATED

#include "cmsis_os.h"
#include "drill.h"
#include "drive_hw.h"

#define SIM_FULL    1000000             /* arm position at EXT_MAX */

static enum DriveMotion simMotion = DRIVE_STOPPED;
static uint8_t simDuty;
static int32_t simPosition;             /* 0 = at EXT_MIN */
static uint32_t simLastUpdate;

/* Moves the simulated arm on to now. */
static void sim_update(void)
{
    uint32_t now = osKernelGetTickCount();
    uint32_t elapsed = now - simLastUpdate;
    simLastUpdate = now;

    int32_t step = (int32_t)((int64_t)SIM_FULL * elapsed * simDuty / 100 / SIM_ARM_TRAVEL_MS);
    if (simMotion == DRIVE_EXTENDING)
        simPosition += step;
    else if (simMotion == DRIVE_RETRACTING)
        simPosition -= step;

    if (simPosition >= SIM_FULL) {
        simPosition = SIM_FULL;
        if (simMotion == DRIVE_EXTENDING)
            simMotion = DRIVE_STOPPED;  /* what the limit switch interrupt does */
    }
    if (simPosition <= 0) {
        simPosition = 0;
        if (simMotion == DRIVE_RETRACTING)
            simMotion = DRIVE_STOPPED;
    }
}

void drive_hw_init(void)
{
    simLastUpdate = osKernelGetTickCount();
    simMotion = DRIVE_STOPPED;
}

void drive_hw_off(void)
{
    sim_update();
    simMotion = DRIVE_STOPPED;
}

void drive_hw_set(enum DriveMotion motion, uint8_t dutyPercent)
{
    sim_update();
    simMotion = (dutyPercent == 0) ? DRIVE_STOPPED : motion;
    simDuty = dutyPercent > 100 ? 100 : dutyPercent;
}

bool drive_hw_at_min(void)
{
    sim_update();
    return simPosition <= 0;
}

bool drive_hw_at_max(void)
{
    sim_update();
    return simPosition >= SIM_FULL;
}

void drive_hw_limit_isr(uint16_t pin)
{
    (void)pin;
}

bool drill_init(void)
{
    return true;
}

bool drill_available(void)
{
    return true;
}

void drill_set(bool on, uint8_t speedPercent)
{
    (void)on;
    (void)speedPercent;
}

void drill_off(void)
{
}

#endif /* HW_SIMULATED */
