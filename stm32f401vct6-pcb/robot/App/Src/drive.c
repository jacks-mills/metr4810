/**
 * @file    drive.c
 * @brief   Arm drive logic: run to a limit switch, soft start, timeout, and
 *          timed full-power pulses.
 *
 * A goal goes through:
 *   PAUSING   motor off for DRIVE_REVERSE_PAUSE_MS (only when reversing)
 *   RUNNING   ramping up to state.driveSpeed over DRIVE_RAMP_MS, then holding
 * and ends when the target switch is pressed, a STOP or new goal arrives, or
 * the timeout passes (state.driveTimeoutMs, set with 'drive timeout'; a
 * change applies at once, counted from the goal's start). The limit switch
 * interrupt (drive_hw_limit_isr) cuts the motor the instant the switch
 * closes; this code then notices on its next tick and reports the result.
 *
 * A pulse goal ('pulse extend <ms>') skips the soft start: the motor goes to
 * 100 % (E held high, no PWM) as soon as it's RUNNING, and stops pulseMs
 * later. The task ticks at that exact moment (drive_logic_pulse_end), so the
 * length is right to the millisecond. The limit switches and STOP still end
 * it early; the timeout doesn't apply. A new pulse in the same direction
 * while one is running restarts the time from now, with no gap.
 */
#include "config.h"
#include "command.h"
#include "drive.h"
#include "drive_hw.h"
#include "log.h"
#include "robot_state.h"

#define PULSE_DUTY      100     /* full power: E stays high */

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
static bool pulse;                  /* the current goal is a pulse */
static uint32_t pulseMs;
static uint32_t pulseEnd;           /* set when the pulse starts RUNNING */
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

static void set_pulse(bool on)
{
    pulse = on;
    state.drivePulsing = on;
}

static void finish(enum DriveResult result)
{
    drive_hw_off();
    phase = PHASE_IDLE;
    state.driveMotion = DRIVE_STOPPED;
    set_pulse(false);
    report(result, goalId);
}

/* Motor on. A pulse starts at full power straight away; a normal move ramps
 * up from the next tick. */
static void start_running(uint32_t now)
{
    phase = PHASE_RUNNING;
    phaseStart = now;
    if (pulse) {
        pulseEnd = now + pulseMs;
        drive_hw_set(target, PULSE_DUTY);
    }
}

void drive_logic_init(uint32_t now)
{
    (void)now;
    phase = PHASE_IDLE;
    state.driveMotion = DRIVE_STOPPED;
    set_pulse(false);
    read_switches();
}

void drive_logic_goal(enum DriveGoal goal, uint32_t ms, uint32_t id, uint32_t now)
{
    enum DriveMotion newTarget =
        (goal == DRIVE_GOAL_EXTEND || goal == DRIVE_GOAL_PULSE_EXTEND) ? DRIVE_EXTENDING
      : (goal == DRIVE_GOAL_RETRACT || goal == DRIVE_GOAL_PULSE_RETRACT) ? DRIVE_RETRACTING
      : DRIVE_STOPPED;
    bool newPulse = (goal == DRIVE_GOAL_PULSE_EXTEND || goal == DRIVE_GOAL_PULSE_RETRACT);
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
            bool wasPulse = pulse;
            set_pulse(newPulse);
            pulseMs = ms;
            if (phase == PHASE_RUNNING) {
                if (newPulse)
                    start_running(now);             /* full power from now for ms */
                else if (wasPulse)
                    phaseStart = now - DRIVE_RAMP_MS;   /* already moving: no soft start */
            }
            return;
        }
        drive_hw_off();
        phase = PHASE_IDLE;
        state.driveMotion = DRIVE_STOPPED;
        set_pulse(false);
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
    set_pulse(newPulse);
    pulseMs = ms;
    state.driveMotion = target;
    if (wasActive) {
        phase = PHASE_PAUSING;                  /* reversing: stop first */
        phaseStart = pauseFrom;
    } else {
        start_running(now);
    }
    log_debug("drive: %s%s", pulse ? "pulse " : "", target == DRIVE_EXTENDING ? "extending" : "retracting");
}

bool drive_logic_pulse_end(uint32_t *when)
{
    if (phase != PHASE_RUNNING || !pulse)
        return false;
    *when = pulseEnd;
    return true;
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
    if (!pulse && now - goalStart >= state.driveTimeoutMs) {
        finish(DRIVE_RESULT_TIMEOUT);
        return;
    }

    if (phase == PHASE_PAUSING) {
        if (now - phaseStart < DRIVE_REVERSE_PAUSE_MS)
            return;
        start_running(now);
    }

    if (pulse) {
        if ((int32_t)(now - pulseEnd) >= 0)
            finish(DRIVE_RESULT_PULSE_DONE);
        else
            drive_hw_set(target, PULSE_DUTY);
        return;
    }

    /* soft start, then whatever speed is set (it can change while moving) */
    uint32_t duty = state.driveSpeed;
    uint32_t sinceStart = now - phaseStart;
    if (sinceStart < DRIVE_RAMP_MS)
        duty = duty * sinceStart / DRIVE_RAMP_MS;
    drive_hw_set(target, (uint8_t)duty);
}
