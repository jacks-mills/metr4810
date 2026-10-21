/**
 * @file    task_control.c
 * @brief   control task -- STAGE 1 STAND-IN.
 *
 * Just enough control to drive the terminal UI:
 *   - applies commands to the shared state, with the agreed rules: only
 *     stop / pause / debug while the sequence runs, servo calibration only in
 *     debug mode, and the safety interlock;
 *   - simulates the arm and base plate in place of the drive and rotate
 *     tasks, so the panel has something to show;
 *   - blinks the LED as a heartbeat.
 * No motor, drill or servo hardware is touched. Stage 2 replaces this with
 * the real control task and the sequence engine.
 */
#include "main.h"
#include "cmsis_os.h"
#include "app.h"
#include "command.h"
#include "config.h"
#include "console.h"
#include "log.h"
#include "robot_state.h"

#define CONTROL_PERIOD_MS   50
#define HEARTBEAT_MS        500

/* simulation */
#define SIM_ARM_TRAVEL_MS   3000    /* full extension at 100 % speed */
#define SIM_ARM_FULL        10000   /* arm position units */
#define SIM_ROTATE_RATE     100     /* tenths of a degree per second at 100 % */

#define SEQUENCE_STEPS      5

static const enum StepAction sequence[SEQUENCE_STEPS] = {
    STEP_RETRACT, STEP_DRILL_ON, STEP_EXTEND, STEP_RETRACT, STEP_DRILL_OFF
};

static int32_t simArmPosition;      /* 0 = at EXT_MIN, SIM_ARM_FULL = at EXT_MAX */
static int32_t simRotateRemainder;

/* ------------------------------------------------------------------------- */
/* Helpers                                                                    */
/* ------------------------------------------------------------------------- */

static const char *source_text(const struct Command *cmd)
{
    switch (cmd->source) {
    case SOURCE_STOP_KEY: return " (Esc)";
    case SOURCE_BUTTON:   return " (button)";
    case SOURCE_CONSOLE:  break;
    }
    return "";
}

static int32_t clamp(int32_t v, int32_t lo, int32_t hi)
{
    return v < lo ? lo : v > hi ? hi : v;
}

static void stop_motion(void)
{
    state.driveMotion = DRIVE_STOPPED;
    state.rotateTarget = state.rotateAngle;
    state.rotating = false;
}

static void set_step(uint8_t step)
{
    state.seqStep = step;
    state.seqAction = sequence[step];
}

/* Safety interlock. Returns true if the move may go ahead; warns if not. */
static bool may_rotate(void)
{
    if (!state.safetyEnabled)
        return true;
    if (state.driveMotion != DRIVE_STOPPED || !state.atExtMin) {
        log_warn("Safety: rotate only with the arm stopped at EXT_MIN");
        return false;
    }
    return true;
}

static bool may_extend(void)
{
    if (state.safetyEnabled && state.rotating) {
        log_warn("Safety: can't extend while the base is rotating");
        return false;
    }
    return true;
}

static void rotate_to(int32_t target)
{
    int32_t clamped = clamp(target, -ROTATE_LIMIT, ROTATE_LIMIT);
    if (clamped != target)
        log_warn("Angle limited to %+d.%d deg", (int)(clamped / 10), (int)(clamped < 0 ? -clamped : clamped) % 10);
    state.rotateTarget = (int16_t)clamped;
    state.rotating = (state.rotateTarget != state.rotateAngle);
}

/* ------------------------------------------------------------------------- */
/* Commands                                                                   */
/* ------------------------------------------------------------------------- */

static bool is_servo_calibration(enum CommandType type)
{
    return type == CMD_SERVO_FREE || type == CMD_SERVO_HOLD ||
           type == CMD_SERVO_CENTRE || type == CMD_SERVO_LIMITS;
}

static void handle_command(const struct Command *cmd)
{
    const char *name = console_command_name(cmd->type);

    /* stop always gets through */
    if (cmd->type == CMD_SEQ_STOP) {
        stop_motion();
        state.drillOn = false;
        state.mode = MODE_STOPPED;
        log_info("Stopped%s", source_text(cmd));
        return;
    }

    /* while the sequence runs, only pause and debug get through */
    if (state.mode == MODE_AUTO && cmd->type != CMD_SEQ_PAUSE && cmd->type != CMD_DEBUG_SET) {
        log_warn("'%s' rejected: sequence running (pause or stop first)", name);
        return;
    }

    if (is_servo_calibration(cmd->type) && !state.debugEnabled) {
        log_warn("'%s' is only available in debug mode", name);
        return;
    }

    switch (cmd->type) {
    case CMD_SEQ_START:
        if (state.mode == MODE_IDLE)
            set_step(0);
        state.mode = MODE_AUTO;
        log_info("Sequence running from step %u%s (stage 1: steps don't run yet)",
                 state.seqStep + 1u, source_text(cmd));
        break;

    case CMD_SEQ_RESTART:
        set_step(0);
        state.mode = MODE_AUTO;
        log_info("Sequence restarted");
        break;

    case CMD_SEQ_PAUSE:
        if (state.mode != MODE_AUTO) {
            log_info("Nothing to pause");
            break;
        }
        stop_motion();
        state.mode = MODE_MANUAL;
        log_info("Paused at step %u: manual control", state.seqStep + 1u);
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
        state.drillOn = true;
        log_info("Drill on at %u%%", state.drillSpeed);
        break;

    case CMD_DRILL_STOP:
        state.drillOn = false;
        log_info("Drill off");
        break;

    case CMD_DRILL_SET_SPEED:
        state.drillSpeed = (uint8_t)cmd->value;
        break;

    case CMD_DRIVE_EXTEND:
        if (state.atExtMax)
            log_info("Arm is already at EXT_MAX");
        else if (may_extend())
            state.driveMotion = DRIVE_EXTENDING;
        break;

    case CMD_DRIVE_RETRACT:
        if (state.atExtMin)
            log_info("Arm is already at EXT_MIN");
        else
            state.driveMotion = DRIVE_RETRACTING;
        break;

    case CMD_DRIVE_STOP:
        state.driveMotion = DRIVE_STOPPED;
        break;

    case CMD_DRIVE_SET_SPEED:
        state.driveSpeed = (uint8_t)cmd->value;
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
        state.rotateTarget = state.rotateAngle;
        state.rotating = false;
        break;

    case CMD_ROTATE_SET_SPEED:
        state.rotateSpeed = (uint8_t)cmd->value;
        break;

    case CMD_SERVO_FREE:
    case CMD_SERVO_HOLD:
    case CMD_SERVO_CENTRE:
    case CMD_SERVO_LIMITS:
        log_info("'%s': servo calibration arrives in stage 4", name);
        break;

    default:
        break;
    }
}

/* ------------------------------------------------------------------------- */
/* Simulation of the arm and the base plate (stage 1 only)                   */
/* ------------------------------------------------------------------------- */

static void simulate(void)
{
    int32_t armStep = SIM_ARM_FULL * CONTROL_PERIOD_MS / SIM_ARM_TRAVEL_MS * state.driveSpeed / 100;
    if (armStep < 1)
        armStep = 1;

    if (state.driveMotion == DRIVE_EXTENDING)
        simArmPosition += armStep;
    else if (state.driveMotion == DRIVE_RETRACTING)
        simArmPosition -= armStep;
    simArmPosition = clamp(simArmPosition, 0, SIM_ARM_FULL);

    bool wasAtMax = state.atExtMax;
    bool wasAtMin = state.atExtMin;
    state.atExtMax = (simArmPosition == SIM_ARM_FULL);
    state.atExtMin = (simArmPosition == 0);

    if (state.driveMotion == DRIVE_EXTENDING && state.atExtMax) {
        state.driveMotion = DRIVE_STOPPED;
        if (!wasAtMax)
            log_info("Arm reached EXT_MAX");
    }
    if (state.driveMotion == DRIVE_RETRACTING && state.atExtMin) {
        state.driveMotion = DRIVE_STOPPED;
        if (!wasAtMin)
            log_info("Arm reached EXT_MIN");
    }

    if (state.rotating) {
        /* whole tenths per period at this speed, carrying the remainder */
        simRotateRemainder += SIM_ROTATE_RATE * state.rotateSpeed * CONTROL_PERIOD_MS / 100;
        int32_t step = simRotateRemainder / 1000;
        simRotateRemainder %= 1000;

        int32_t error = state.rotateTarget - state.rotateAngle;
        if (step > (error < 0 ? -error : error))
            step = error < 0 ? -error : error;
        state.rotateAngle += (int16_t)(error < 0 ? -step : step);
        if (state.rotateAngle == state.rotateTarget)
            state.rotating = false;
    }
}

/* ------------------------------------------------------------------------- */
/* Task                                                                       */
/* ------------------------------------------------------------------------- */

void TaskControl(void *argument)
{
    (void)argument;

    state.mode = MODE_IDLE;
    state.seqStepCount = SEQUENCE_STEPS;
    set_step(0);
    state.safetyEnabled = true;
    state.simulated = true;
    state.drillSpeed = DEFAULT_DRILL_SPEED;
    state.driveSpeed = DEFAULT_DRIVE_SPEED;
    state.rotateSpeed = DEFAULT_ROTATE_SPEED;

    /* simulated hardware: arm home, servo centred and answering */
    state.atExtMin = true;
    state.servoOnline = true;
    state.servoLimitMin = -ROTATE_LIMIT;
    state.servoLimitMax = ROTATE_LIMIT;
    state.servoVoltage = 120;
    state.servoTemperature = 30;

    log_info("Boring robot ready. Type 'help' for commands, Esc to stop.");
    log_warn("Stage 1: drive and rotation are simulated");

    uint32_t nextTick = osKernelGetTickCount();
    uint32_t nextBlink = nextTick;

    for (;;) {
        struct Command cmd;
        int32_t wait = (int32_t)(nextTick - osKernelGetTickCount());

        if (command_receive(&cmd, wait > 0 ? (uint32_t)wait : 0)) {
            handle_command(&cmd);
            continue;                   /* handle any other queued commands first */
        }

        uint32_t now = osKernelGetTickCount();
        if ((int32_t)(now - nextTick) >= 0) {
            nextTick += CONTROL_PERIOD_MS;
            simulate();
        }
        if ((int32_t)(now - nextBlink) >= 0) {
            nextBlink += HEARTBEAT_MS;
            HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
        }
    }
}
