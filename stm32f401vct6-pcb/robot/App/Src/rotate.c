/**
 * @file    rotate.c
 * @brief   Rotate logic: servo moves, status polling, calibration.
 *
 * The servo does its own speed and acceleration control, so a move is: send
 * the goal, then poll until the servo says it has stopped within
 * SERVO_TOLERANCE steps of it (or the expected time plus a margin passes).
 *
 * If the servo stops answering, the task keeps trying every
 * OFFLINE_RETRY_MS; when it answers again it is told to hold where it is.
 */
#include <stdio.h>
#include <stdlib.h>

#include "config.h"
#include "command.h"
#include "log.h"
#include "robot_state.h"
#include "rotate.h"
#include "servo.h"

#define OFFLINE_RETRY_MS        500
#define MISSES_BEFORE_OFFLINE   3       /* polls in a row with no answer */
#define LIMIT_SLACK             10      /* tenths: servo limits this far past ROTATE_LIMIT are "not set" */
#define MOVE_MARGIN_MS          1000    /* on top of distance / speed */

static bool online;
static int misses;
static uint32_t nextContact;

static bool busy;                       /* a move is in progress */
static uint32_t goalId;
static int32_t goalSteps;
static uint32_t deadline;

static uint8_t shownError;

/* ------------------------------------------------------------------------- */
/* Units                                                                      */
/* ------------------------------------------------------------------------- */

int32_t rotate_angle_to_steps(int32_t tenths)
{
    int32_t steps = (tenths * SERVO_STEPS + (tenths >= 0 ? 1800 : -1800)) / 3600;
    return SERVO_CENTRE + (SERVO_REVERSED ? -steps : steps);
}

int32_t rotate_steps_to_angle(int32_t steps)
{
    int32_t d = steps - SERVO_CENTRE;
    if (SERVO_REVERSED)
        d = -d;
    return (d * 3600 + (d >= 0 ? SERVO_STEPS / 2 : -SERVO_STEPS / 2)) / SERVO_STEPS;
}

/* "+2.5" */
static const char *deg(char *buf, size_t size, int32_t tenths)
{
    int32_t a = tenths < 0 ? -tenths : tenths;
    snprintf(buf, size, "%c%ld.%ld", tenths < 0 ? '-' : '+', (long)(a / 10), (long)(a % 10));
    return buf;
}

/* ------------------------------------------------------------------------- */
/* Helpers                                                                    */
/* ------------------------------------------------------------------------- */

static void report(enum RotateResult result, uint32_t id)
{
    struct Command event = {
        .type = EVT_ROTATE_RESULT, .source = SOURCE_TASK,
        .value = result, .value2 = (int32_t)id
    };
    if (!command_send(&event))
        log_error("rotate: command queue full, result lost");
}

static void finish(enum RotateResult result)
{
    busy = false;
    state.rotating = false;
    report(result, goalId);
}

static void go_offline(uint32_t now)
{
    if (online)
        log_warn("Servo stopped answering");
    online = false;
    misses = 0;
    nextContact = now + OFFLINE_RETRY_MS;
    state.servoOnline = false;
    state.rotating = false;
    if (busy)
        finish(ROTATE_RESULT_OFFLINE);
}

static void read_limits(void)
{
    uint16_t lo, hi;
    if (!servo_read_limits(&lo, &hi))
        return;
    int32_t a = rotate_steps_to_angle(lo);
    int32_t b = rotate_steps_to_angle(hi);
    state.servoLimitMin = (int16_t)(a < b ? a : b);
    state.servoLimitMax = (int16_t)(a < b ? b : a);
}

static void show_status(const struct ServoStatus *s)
{
    state.rotateAngle = (int16_t)rotate_steps_to_angle(s->position);
    state.rotating = busy || s->moving;
    state.servoVoltage = s->voltage;
    state.servoTemperature = (int8_t)s->temperature;
    state.servoLoad = s->load;

    uint8_t error = servo_error();
    state.servoError = error;
    if (error != shownError) {
        if (error)
            log_warn("Servo reports: %s", servo_error_text(error));
        else
            log_info("Servo error cleared");
        shownError = error;
    }
}

/* Makes the servo hold where it is now. */
static bool hold_here(void)
{
    struct ServoStatus s;
    return servo_read_status(&s)
        && servo_move((uint16_t)s.position, SERVO_MAX_SPEED, SERVO_ACCEL);
}

/* First contact (and reconnects): check the setup, then hold position. */
static bool contact(void)
{
    struct ServoStatus s;
    uint8_t mode;
    char a[12], lo[12], hi[12];

    if (!servo_ping())
        return false;
    if (servo_read_mode(&mode) && mode != 0)
        log_warn("Servo is in mode %u, not position mode (0)", mode);
    read_limits();
    if (!servo_read_status(&s) ||
        !servo_move((uint16_t)s.position, SERVO_MAX_SPEED, SERVO_ACCEL) ||
        !servo_set_torque(true))
        return false;

    online = true;
    misses = 0;
    state.servoOnline = true;
    state.servoFree = false;
    show_status(&s);

    log_info("Servo found at %s deg (its limits: %s to %s deg)",
             deg(a, sizeof a, state.rotateAngle),
             deg(lo, sizeof lo, state.servoLimitMin), deg(hi, sizeof hi, state.servoLimitMax));
    if (state.servoLimitMin < -ROTATE_LIMIT - LIMIT_SLACK || state.servoLimitMax > ROTATE_LIMIT + LIMIT_SLACK)
        log_warn("Servo limits not set yet: see 'servo limits' (debug mode)");
    if (state.rotateAngle < -ROTATE_LIMIT - LIMIT_SLACK || state.rotateAngle > ROTATE_LIMIT + LIMIT_SLACK)
        log_warn("Base is outside the rotation range: centre the servo (debug mode)");
    return true;
}

/* ------------------------------------------------------------------------- */
/* Goals                                                                      */
/* ------------------------------------------------------------------------- */

static void start_move(const struct RotateGoal *g, uint32_t now)
{
    int32_t steps = rotate_angle_to_steps(g->angle);
    if (steps < 0)
        steps = 0;
    if (steps > SERVO_STEPS - 1)
        steps = SERVO_STEPS - 1;
    uint32_t speed = (uint32_t)SERVO_MAX_SPEED * g->speed / 100;
    if (speed < 1)
        speed = 1;

    if ((state.servoFree && !servo_set_torque(true)) ||
        !servo_move((uint16_t)steps, (uint16_t)speed, SERVO_ACCEL)) {
        report(ROTATE_RESULT_SERVO_ERROR, g->id);
        return;
    }
    state.servoFree = false;

    if (busy)
        report(ROTATE_RESULT_STOPPED, goalId);      /* replaced */

    int32_t distance = abs(steps - rotate_angle_to_steps(state.rotateAngle));
    busy = true;
    goalId = g->id;
    goalSteps = steps;
    deadline = now + (uint32_t)distance * 1000 / speed + MOVE_MARGIN_MS;
    state.rotating = true;
}

void rotate_logic_goal(const struct RotateGoal *g, uint32_t now)
{
    char lo[12], hi[12];
    bool ok;

    if (!online) {
        report(ROTATE_RESULT_OFFLINE, g->id);
        return;
    }

    switch (g->type) {
    case ROTATE_GOAL_TO:
        start_move(g, now);
        return;

    case ROTATE_GOAL_STOP:
        (void)hold_here();
        if (busy)
            finish(ROTATE_RESULT_STOPPED);
        return;

    case ROTATE_GOAL_FREE:
        ok = servo_set_torque(false);
        if (busy)
            finish(ROTATE_RESULT_STOPPED);
        if (ok) {
            state.servoFree = true;
            log_info("Servo free: turn the plate by hand ('servo hold' locks it)");
        }
        break;

    case ROTATE_GOAL_HOLD:
        ok = hold_here() && servo_set_torque(true);
        if (ok) {
            state.servoFree = false;
            log_info("Servo holding");
        }
        break;

    case ROTATE_GOAL_CENTRE:
        if (busy)
            finish(ROTATE_RESULT_STOPPED);
        ok = servo_set_centre();
        if (ok) {
            struct ServoStatus s;
            if (servo_read_status(&s))
                show_status(&s);
            log_info("Centre saved: this position is now 0.0 deg");
        }
        break;

    case ROTATE_GOAL_LIMITS: {
        int32_t a = rotate_angle_to_steps(g->angle);
        int32_t b = rotate_angle_to_steps(g->angle2);
        ok = servo_write_limits((uint16_t)(a < b ? a : b), (uint16_t)(a < b ? b : a));
        read_limits();
        if (ok)
            log_info("Servo limits saved: %s to %s deg",
                     deg(lo, sizeof lo, state.servoLimitMin), deg(hi, sizeof hi, state.servoLimitMax));
        break;
    }

    default:
        return;
    }

    if (!ok)
        log_error("Servo didn't accept the command");
    report(ok ? ROTATE_RESULT_DONE : ROTATE_RESULT_SERVO_ERROR, g->id);
}

/* ------------------------------------------------------------------------- */
/* Polling                                                                    */
/* ------------------------------------------------------------------------- */

void rotate_logic_init(uint32_t now)
{
    servo_init();
    online = false;
    busy = false;
    shownError = 0;
    state.servoOnline = false;
    if (!contact()) {
        log_warn("Servo not answering: check its 12 V supply and signal wire");
        nextContact = now + OFFLINE_RETRY_MS;
    }
}

void rotate_logic_tick(uint32_t now)
{
    if (!online) {
        if ((int32_t)(now - nextContact) >= 0 && !contact())
            nextContact = now + OFFLINE_RETRY_MS;
        return;
    }

    struct ServoStatus s;
    if (!servo_read_status(&s)) {
        if (++misses >= MISSES_BEFORE_OFFLINE)
            go_offline(now);
        return;
    }
    misses = 0;
    show_status(&s);

    if (!busy)
        return;

    if (servo_error() & (SERVO_ERR_OVERLOAD | SERVO_ERR_CURRENT | SERVO_ERR_TEMPERATURE)) {
        (void)hold_here();
        finish(ROTATE_RESULT_SERVO_ERROR);
    } else if (!s.moving && abs(s.position - goalSteps) <= SERVO_TOLERANCE) {
        finish(ROTATE_RESULT_DONE);
    } else if ((int32_t)(now - deadline) >= 0) {
        (void)hold_here();
        finish(ROTATE_RESULT_TIMEOUT);
    }
}
