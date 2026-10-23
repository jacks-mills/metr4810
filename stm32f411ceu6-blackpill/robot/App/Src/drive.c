/**
 * @file    drive.c
 * @brief   Arm drive logic: run to a limit switch, soft start, timeout.
 *
 * A goal goes through:
 *   PAUSING   motor off for DRIVE_REVERSE_PAUSE_MS (only when reversing)
 *   RUNNING   ramping up to state.driveSpeed over DRIVE_RAMP_MS, then holding
 * and ends when the target switch is pressed, a STOP or new goal arrives, or
 * DRIVE_TIMEOUT_MS passes. The limit switch interrupt (drive_hw_limit_isr)
 * cuts the motor the instant the switch closes; this code then notices on
 * its next tick and reports the result.
 */
#include "config.h"
#include "command.h"
#include "drive.h"
#include "drive_hw.h"
#include "log.h"
#include "robot_state.h"

enum Phase {
    PHASE_IDLE,
    PHASE_PAUSING,
    PHASE_RUNNING
};

static enum Phase phase;
static enum DriveMotion target;     /* DRIVE_EXTENDING or DRIVE_RETRACTING */
static uint32_t goalId;
static uint32_t goalStart;          /* for the timeout */
static uint32_t phaseStart;         /* for the pause and the ramp */
static bool switchErrorShown;

static void report(enum DriveResult result, uint32_t id)
{
    struct Command event = {
        .type = EVT_DRIVE_RESULT, .source = SOURCE_TASK,
        .value = result, .value2 = (int32_t)id
    };
    if (!command_send(&event))
        log_error("drive: command queue full, result lost");
}

static void read_switches(void)
{
    state.atExtMin = drive_hw_at_min();
    state.atExtMax = drive_hw_at_max();
}

static void finish(enum DriveResult result)
{
    drive_hw_off();
    phase = PHASE_IDLE;
    state.driveMotion = DRIVE_STOPPED;
    report(result, goalId);
}

void drive_logic_init(uint32_t now)
{
    (void)now;
    phase = PHASE_IDLE;
    state.driveMotion = DRIVE_STOPPED;
    read_switches();
}

void drive_logic_goal(enum DriveGoal goal, uint32_t id, uint32_t now)
{
    enum DriveMotion newTarget = (goal == DRIVE_GOAL_EXTEND) ? DRIVE_EXTENDING
                               : (goal == DRIVE_GOAL_RETRACT) ? DRIVE_RETRACTING
                               : DRIVE_STOPPED;
    bool wasActive = (phase != PHASE_IDLE);
    /* if the motor was on (or only just switched off), pause before reversing */
    uint32_t pauseFrom = (phase == PHASE_PAUSING) ? phaseStart : now;

    read_switches();

    if (wasActive) {
        if (newTarget == target) {
            /* same direction: carry on, under the new goal */
            report(DRIVE_RESULT_STOPPED, goalId);
            goalId = id;
            goalStart = now;
            return;
        }
        drive_hw_off();
        phase = PHASE_IDLE;
        state.driveMotion = DRIVE_STOPPED;
        report(DRIVE_RESULT_STOPPED, goalId);
    }

    if (newTarget == DRIVE_STOPPED)
        return;

    goalId = id;
    if (state.atExtMin && state.atExtMax) {
        report(DRIVE_RESULT_SWITCH_ERROR, id);
        return;
    }
    if ((newTarget == DRIVE_EXTENDING && state.atExtMax) ||
        (newTarget == DRIVE_RETRACTING && state.atExtMin)) {
        report(DRIVE_RESULT_REACHED, id);       /* already there */
        return;
    }

    target = newTarget;
    goalStart = now;
    phaseStart = wasActive ? pauseFrom : now;
    phase = wasActive ? PHASE_PAUSING : PHASE_RUNNING;    /* reversing: stop first */
    state.driveMotion = target;
    log_debug("drive: %s", target == DRIVE_EXTENDING ? "extending" : "retracting");
}

void drive_logic_tick(uint32_t now)
{
    read_switches();

    if (state.atExtMin && state.atExtMax) {
        if (phase != PHASE_IDLE)
            finish(DRIVE_RESULT_SWITCH_ERROR);
        if (!switchErrorShown)
            log_error("Both arm limit switches read pressed: check the wiring");
        switchErrorShown = true;
        return;
    }
    switchErrorShown = false;

    if (phase == PHASE_IDLE)
        return;

    if ((target == DRIVE_EXTENDING && state.atExtMax) ||
        (target == DRIVE_RETRACTING && state.atExtMin)) {
        finish(DRIVE_RESULT_REACHED);
        return;
    }
    if (now - goalStart >= DRIVE_TIMEOUT_MS) {
        finish(DRIVE_RESULT_TIMEOUT);
        return;
    }

    if (phase == PHASE_PAUSING) {
        if (now - phaseStart < DRIVE_REVERSE_PAUSE_MS)
            return;
        phase = PHASE_RUNNING;
        phaseStart = now;
    }

    /* soft start, then whatever speed is set (it can change while moving) */
    uint32_t duty = state.driveSpeed;
    uint32_t sinceStart = now - phaseStart;
    if (sinceStart < DRIVE_RAMP_MS)
        duty = duty * sinceStart / DRIVE_RAMP_MS;
    drive_hw_set(target, (uint8_t)duty);
}
