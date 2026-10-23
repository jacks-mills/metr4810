/**
 * @file    control.c
 * @brief   Control logic: modes, command rules, safety interlock, sequence.
 *
 * The only place that decides what moves. It sends goals to the drive and
 * rotate tasks (each with a fresh id) and switches the drill directly. The
 * tasks answer with EVT_DRIVE_RESULT / EVT_ROTATE_RESULT; a result whose id
 * isn't the latest goal's is stale (that goal was replaced) and is ignored.
 *
 * Sequence: a table of steps with fixed targets ("retract to EXT_MIN",
 * "extend to EXT_MAX"). Continuing re-runs the current step, so it works from
 * wherever things were left in manual mode. A step that needs the drill
 * spins it up first if it isn't already running.
 *
 * Rules (from the design):
 *   - stop always gets through;
 *   - while the sequence runs, only pause and debug are accepted;
 *   - servo calibration only in debug mode;
 *   - safety interlock: rotate only with the arm stopped at EXT_MIN, and
 *     don't extend while the base is rotating.
 */
#include <stdio.h>

#include "config.h"
#include "console.h"
#include "control.h"
#include "drill.h"
#include "drive.h"
#include "log.h"
#include "robot_state.h"
#include "rotate.h"

#define ANGLE_SLACK     10      /* tenths: refuse to rotate if the base is further than this outside the range */

struct Step {
    enum StepAction action;
    bool needsDrill;            /* spin the drill up first if it's off */
};

static const struct Step sequence[] = {
    { STEP_RETRACT,   false },  /* home: arm in */
    { STEP_DRILL_ON,  false },  /* and wait DRILL_SPINUP_MS */
    { STEP_EXTEND,    true  },  /* drill into the wall, to EXT_MAX */
    { STEP_RETRACT,   true  },  /* back out with the drill still turning */
    { STEP_DRILL_OFF, false },
};
#define SEQUENCE_STEPS  ((uint8_t)(sizeof sequence / sizeof sequence[0]))

static uint32_t nextGoalId = 1;
static uint32_t driveGoalId;
static uint32_t rotateGoalId;
static enum DriveGoal driveGoal;
static bool driveBusy;
static bool rotateBusy;
static bool waitingForDrill;    /* drill spinning up before the step's action */
static uint32_t drillReadyAt;

/* ------------------------------------------------------------------------- */
/* Helpers                                                                    */
/* ------------------------------------------------------------------------- */

static bool reached(uint32_t now, uint32_t when)
{
    return (int32_t)(now - when) >= 0;
}

static const char *deg(char *buf, size_t size, int32_t tenths)
{
    int32_t a = tenths < 0 ? -tenths : tenths;
    snprintf(buf, size, "%c%ld.%ld", tenths < 0 ? '-' : '+', (long)(a / 10), (long)(a % 10));
    return buf;
}

static const char *source_text(const struct Command *cmd)
{
    switch (cmd->source) {
    case SOURCE_STOP_KEY: return " (Esc)";
    case SOURCE_BUTTON:   return " (button)";
    default:              return "";
    }
}

static const char *step_text(enum StepAction a)
{
    switch (a) {
    case STEP_RETRACT:   return "retract to EXT_MIN";
    case STEP_DRILL_ON:  return "drill on";
    case STEP_EXTEND:    return "extend to EXT_MAX";
    case STEP_DRILL_OFF: return "drill off";
    default:             return "-";
    }
}

/* ------------------------------------------------------------------------- */
/* Actuators                                                                  */
/* ------------------------------------------------------------------------- */

static void drill(bool on)
{
    state.drillOn = on;
    drill_set(on, state.drillSpeed);
}

static void drive(enum DriveGoal goal)
{
    uint32_t id = nextGoalId++;
    if (goal != DRIVE_GOAL_STOP) {
        driveGoalId = id;
        driveGoal = goal;
    }
    driveBusy = (goal != DRIVE_GOAL_STOP);
    if (!drive_send(goal, id)) {
        driveBusy = false;
        log_error("Drive task not responding");
    }
}

static void rotate(enum RotateGoalType type, int32_t angle, int32_t angle2)
{
    struct RotateGoal goal = {
        .type = type, .angle = (int16_t)angle, .angle2 = (int16_t)angle2,
        .speed = state.rotateSpeed, .id = nextGoalId++
    };
    if (type != ROTATE_GOAL_STOP)
        rotateGoalId = goal.id;
    rotateBusy = (type != ROTATE_GOAL_STOP);
    if (!rotate_send(&goal)) {
        rotateBusy = false;
        log_error("Rotate task not responding");
    }
}

static void stop_motion(void)
{
    drive(DRIVE_GOAL_STOP);
    if (rotateBusy)
        rotate(ROTATE_GOAL_STOP, 0, 0);
    waitingForDrill = false;
}

static void stop_all(void)
{
    stop_motion();
    drill(false);
}

static void fault(const char *text)
{
    stop_all();
    state.mode = MODE_FAULT;
    log_error("%s", text);
    log_info("Sort it out, then 'start' to carry on or 'restart'");
}

/* ------------------------------------------------------------------------- */
/* Sequence                                                                   */
/* ------------------------------------------------------------------------- */

static void set_step(uint8_t step)
{
    state.seqStep = step;
    state.seqAction = sequence[step].action;
}

static void spin_up(uint32_t now)
{
    drill(true);
    waitingForDrill = true;
    drillReadyAt = now + DRILL_SPINUP_MS;
}

static void advance(uint32_t now);

/* Starts (or restarts) the current step. */
static void run_step(uint32_t now)
{
    const struct Step *s = &sequence[state.seqStep];

    if (s->needsDrill && !state.drillOn) {
        log_info("Drill spinning up");
        spin_up(now);                   /* run_step again once it's up to speed */
        return;
    }

    switch (s->action) {
    case STEP_RETRACT:
        drive(DRIVE_GOAL_RETRACT);
        break;
    case STEP_EXTEND:
        drive(DRIVE_GOAL_EXTEND);
        break;
    case STEP_DRILL_ON:
        spin_up(now);                   /* advances once it's up to speed */
        break;
    case STEP_DRILL_OFF:
        drill(false);
        advance(now);
        break;
    default:
        advance(now);
        break;
    }
}

static void advance(uint32_t now)
{
    if (state.seqStep + 1 >= SEQUENCE_STEPS) {
        set_step(0);
        state.mode = MODE_IDLE;
        log_info("Sequence complete");
        return;
    }
    set_step(state.seqStep + 1);
    log_info("Step %u/%u: %s", state.seqStep + 1u, SEQUENCE_STEPS, step_text(state.seqAction));
    run_step(now);
}

/* Why the sequence can't start right now, or NULL if it can. */
static const char *start_blocker(void)
{
    if (rotateBusy)
        return "Wait for the base to finish rotating";
    if (state.servoFree)
        return "The servo is free: 'servo hold' first";
    if (!drill_available())
        return "Drill disabled (TIM3 CH3 polarity), so the sequence can't run";
    return NULL;
}

static void start_sequence(bool fromTheTop, const struct Command *cmd, uint32_t now)
{
    const char *blocker = start_blocker();
    if (blocker != NULL) {
        log_warn("%s", blocker);
        return;
    }
    bool fresh = fromTheTop || state.mode == MODE_IDLE;
    if (fresh)
        set_step(0);
    state.mode = MODE_AUTO;
    log_info("Sequence %s%s, step %u/%u: %s", fresh ? "started" : "continuing",
             source_text(cmd), state.seqStep + 1u, SEQUENCE_STEPS, step_text(state.seqAction));
    run_step(now);
}

/* ------------------------------------------------------------------------- */
/* Manual moves                                                               */
/* ------------------------------------------------------------------------- */

static bool may_extend(void)
{
    if (state.driveSpeed == 0) {
        log_warn("Drive speed is 0");
        return false;
    }
    if (state.safetyEnabled && rotateBusy) {
        log_warn("Safety: can't extend while the base is rotating");
        return false;
    }
    return true;
}

static bool may_rotate(void)
{
    char a[12], r[12];

    if (!state.servoOnline) {
        log_warn("Servo isn't answering");
        return false;
    }
    if (state.rotateSpeed == 0) {
        log_warn("Rotate speed is 0");
        return false;
    }
    if (state.rotateAngle < -ROTATE_LIMIT - ANGLE_SLACK || state.rotateAngle > ROTATE_LIMIT + ANGLE_SLACK) {
        log_warn("Base is at %s deg, outside %s deg: centre the servo first",
                 deg(a, sizeof a, state.rotateAngle), deg(r, sizeof r, ROTATE_LIMIT));
        return false;
    }
    if (state.safetyEnabled && (driveBusy || !state.atExtMin)) {
        log_warn("Safety: rotate only with the arm stopped at EXT_MIN");
        return false;
    }
    return true;
}

static void rotate_to(int32_t target)
{
    char a[12];
    int32_t lo = state.servoLimitMin > -ROTATE_LIMIT ? state.servoLimitMin : -ROTATE_LIMIT;
    int32_t hi = state.servoLimitMax < ROTATE_LIMIT ? state.servoLimitMax : ROTATE_LIMIT;
    int32_t clamped = target < lo ? lo : target > hi ? hi : target;

    if (clamped != target)
        log_warn("Angle limited to %s deg", deg(a, sizeof a, clamped));
    state.rotateTarget = (int16_t)clamped;
    rotate(ROTATE_GOAL_TO, clamped, 0);
}

/* ------------------------------------------------------------------------- */
/* Events from the drive and rotate tasks                                     */
/* ------------------------------------------------------------------------- */

static void drive_result(enum DriveResult result, uint32_t id, uint32_t now)
{
    if (!driveBusy || id != driveGoalId)
        return;                         /* a goal that was since replaced */
    driveBusy = false;

    bool extending = (driveGoal == DRIVE_GOAL_EXTEND);
    switch (result) {
    case DRIVE_RESULT_REACHED:
        log_info("Arm at %s", extending ? "EXT_MAX" : "EXT_MIN");
        if (state.mode == MODE_AUTO &&
            (state.seqAction == STEP_EXTEND || state.seqAction == STEP_RETRACT))
            advance(now);
        break;
    case DRIVE_RESULT_TIMEOUT:
        if (state.mode == MODE_AUTO)
            fault(extending ? "Arm didn't reach EXT_MAX in time (DRIVE_TIMEOUT_MS)"
                            : "Arm didn't reach EXT_MIN in time (DRIVE_TIMEOUT_MS)");
        else
            log_warn("Arm stopped: no limit switch within DRIVE_TIMEOUT_MS");
        break;
    case DRIVE_RESULT_SWITCH_ERROR:
        fault("Both limit switches read pressed: check the wiring");
        break;
    case DRIVE_RESULT_STOPPED:
        break;
    }
}

static void rotate_result(enum RotateResult result, uint32_t id)
{
    if (!rotateBusy || id != rotateGoalId)
        return;
    rotateBusy = false;

    switch (result) {
    case ROTATE_RESULT_TIMEOUT:
        log_warn("Base didn't reach its target in time");
        break;
    case ROTATE_RESULT_SERVO_ERROR:
        log_warn("Servo error: rotation stopped");
        break;
    case ROTATE_RESULT_OFFLINE:
        log_warn("Servo isn't answering");
        break;
    case ROTATE_RESULT_DONE:
    case ROTATE_RESULT_STOPPED:
        break;
    }
}

/* ------------------------------------------------------------------------- */
/* Public                                                                     */
/* ------------------------------------------------------------------------- */

void control_init(uint32_t now)
{
    (void)now;
    state.mode = MODE_IDLE;
    state.seqStepCount = SEQUENCE_STEPS;
    set_step(0);
    state.safetyEnabled = true;
    state.simulated = HW_SIMULATED;
    state.drillSpeed = DEFAULT_DRILL_SPEED;
    state.driveSpeed = DEFAULT_DRIVE_SPEED;
    state.rotateSpeed = DEFAULT_ROTATE_SPEED;

    log_info("Boring robot ready. Type 'help' for commands, Esc to stop.");
    if (HW_SIMULATED)
        log_warn("Simulated hardware (HW_SIMULATED in config.h)");
    if (!drill_available())
        log_error("Drill disabled: set TIM3 CH3 output polarity to Low in CubeMX");
}

void control_command(const struct Command *cmd, uint32_t now)
{
    const char *name = console_command_name(cmd->type);
    char a[12];

    /* events from the other tasks */
    switch (cmd->type) {
    case EVT_DRIVE_RESULT:
        drive_result((enum DriveResult)cmd->value, (uint32_t)cmd->value2, now);
        return;
    case EVT_ROTATE_RESULT:
        rotate_result((enum RotateResult)cmd->value, (uint32_t)cmd->value2);
        return;
    case CMD_TERMINAL_LOST:
        if (state.mode != MODE_AUTO && (driveBusy || rotateBusy || state.drillOn)) {
            stop_all();
            state.mode = MODE_STOPPED;
            log_warn("Terminal closed: everything stopped");
        }
        return;
    default:
        break;
    }

    /* stop always gets through */
    if (cmd->type == CMD_SEQ_STOP) {
        stop_all();
        state.mode = MODE_STOPPED;
        log_info("Stopped%s", source_text(cmd));
        return;
    }

    /* while the sequence runs, only pause and debug get through */
    if (state.mode == MODE_AUTO && cmd->type != CMD_SEQ_PAUSE && cmd->type != CMD_DEBUG_SET) {
        log_warn("'%s' rejected: sequence running (pause or stop first)", name);
        return;
    }

    switch (cmd->type) {
    case CMD_SERVO_FREE:
    case CMD_SERVO_HOLD:
    case CMD_SERVO_CENTRE:
    case CMD_SERVO_LIMITS:
        if (!state.debugEnabled) {
            log_warn("'%s' is only available in debug mode", name);
            return;
        }
        if (rotateBusy) {
            log_warn("Wait for the base to stop first");
            return;
        }
        if (!state.servoOnline) {
            log_warn("Servo isn't answering");
            return;
        }
        break;
    default:
        break;
    }

    switch (cmd->type) {
    case CMD_SEQ_START:
        start_sequence(false, cmd, now);
        break;

    case CMD_SEQ_RESTART:
        start_sequence(true, cmd, now);
        break;

    case CMD_SEQ_PAUSE:
        if (state.mode != MODE_AUTO) {
            log_info("Nothing to pause");
            break;
        }
        stop_motion();
        state.mode = MODE_MANUAL;
        log_info("Paused at step %u: manual control ('start' carries on)", state.seqStep + 1u);
        break;

    case CMD_SAFETY_SET:
        state.safetyEnabled = cmd->value != 0;
        if (state.safetyEnabled)
            log_info("Safety interlock on");
        else
            log_warn("Safety interlock OFF: rotation allowed while extended");
        break;

    case CMD_DEBUG_SET:
        state.debugEnabled = cmd->value != 0;
        log_info("Debug %s", state.debugEnabled ? "on" : "off");
        break;

    case CMD_DRILL_START:
        if (!drill_available()) {
            log_warn("Drill disabled (TIM3 CH3 polarity)");
            break;
        }
        drill(true);
        log_info("Drill on at %u%%", state.drillSpeed);
        break;

    case CMD_DRILL_STOP:
        drill(false);
        log_info("Drill off");
        break;

    case CMD_DRILL_SET_SPEED:
        state.drillSpeed = (uint8_t)cmd->value;
        if (state.drillOn)
            drill(true);
        break;

    case CMD_DRIVE_EXTEND:
        if (may_extend())
            drive(DRIVE_GOAL_EXTEND);
        break;

    case CMD_DRIVE_RETRACT:
        if (state.driveSpeed == 0)
            log_warn("Drive speed is 0");
        else
            drive(DRIVE_GOAL_RETRACT);
        break;

    case CMD_DRIVE_STOP:
        drive(DRIVE_GOAL_STOP);
        break;

    case CMD_DRIVE_SET_SPEED:
        state.driveSpeed = (uint8_t)cmd->value;     /* the drive task picks it up */
        break;

    case CMD_ROTATE_LEFT:
        if (may_rotate())
            rotate_to(-ROTATE_LIMIT);
        break;

    case CMD_ROTATE_RIGHT:
        if (may_rotate())
            rotate_to(ROTATE_LIMIT);
        break;

    case CMD_ROTATE_TO:
        if (may_rotate())
            rotate_to(cmd->value);
        break;

    case CMD_ROTATE_STOP:
        rotate(ROTATE_GOAL_STOP, 0, 0);
        break;

    case CMD_ROTATE_SET_SPEED:
        state.rotateSpeed = (uint8_t)cmd->value;
        break;

    case CMD_SERVO_FREE:
        rotate(ROTATE_GOAL_FREE, 0, 0);
        break;

    case CMD_SERVO_HOLD:
        rotate(ROTATE_GOAL_HOLD, 0, 0);
        break;

    case CMD_SERVO_CENTRE:
        rotate(ROTATE_GOAL_CENTRE, 0, 0);
        break;

    case CMD_SERVO_LIMITS:
        if (cmd->value < -ROTATE_LIMIT || cmd->value2 > ROTATE_LIMIT)
            log_warn("Wider than the firmware's own limit (%s deg), which still applies",
                     deg(a, sizeof a, ROTATE_LIMIT));
        rotate(ROTATE_GOAL_LIMITS, cmd->value, cmd->value2);
        break;

    default:
        break;
    }
}

bool control_start_ready(void)
{
    return state.mode != MODE_AUTO && start_blocker() == NULL;
}

void control_tick(uint32_t now)
{
    if (state.mode == MODE_AUTO && waitingForDrill && reached(now, drillReadyAt)) {
        waitingForDrill = false;
        if (state.seqAction == STEP_DRILL_ON)
            advance(now);
        else
            run_step(now);              /* the drill was off; now do the step */
    }
}
