/**
 * @file    command.h
 * @brief   Commands and events for the control task.
 *
 * Anything that wants the robot to do something (the terminal, the start
 * button) builds a struct Command and calls command_send(). The drive and
 * rotate tasks report how their goals ended on the same queue (EVT_...).
 * Only the control task receives them, so it is the one place that decides
 * what moves, and it waits on just this one queue.
 */
#ifndef COMMAND_H
#define COMMAND_H

#include <stdbool.h>
#include <stdint.h>

enum CommandType {
    CMD_NONE,

    /* sequence */
    CMD_SEQ_START,          /* start, or continue from the current step */
    CMD_SEQ_RESTART,        /* go back to the first step and start */
    CMD_SEQ_PAUSE,          /* stop drive and rotation, keep the drill -> manual */
    CMD_SEQ_STOP,           /* stop drive, rotation and drill */

    /* settings; value: 0 = off, 1 = on */
    CMD_SAFETY_SET,
    CMD_DEBUG_SET,

    /* drill; value: speed 0-100 % */
    CMD_DRILL_START,
    CMD_DRILL_STOP,
    CMD_DRILL_SET_SPEED,

    /* arm drive; value: speed 0-100 % */
    CMD_DRIVE_EXTEND,
    CMD_DRIVE_RETRACT,
    CMD_DRIVE_STOP,
    CMD_DRIVE_SET_SPEED,

    /* base rotation; value: angle in tenths of a degree, or speed 0-100 % */
    CMD_ROTATE_LEFT,
    CMD_ROTATE_RIGHT,
    CMD_ROTATE_STOP,
    CMD_ROTATE_TO,
    CMD_ROTATE_SET_SPEED,

    /* servo calibration, debug mode only; value/value2: min/max angle */
    CMD_SERVO_FREE,
    CMD_SERVO_HOLD,
    CMD_SERVO_CENTRE,
    CMD_SERVO_LIMITS,

    /* not typed: sent by other tasks */
    CMD_TERMINAL_LOST,      /* the terminal program closed the port */
    EVT_DRIVE_RESULT,       /* value: enum DriveResult, value2: goal id */
    EVT_ROTATE_RESULT,      /* value: enum RotateResult, value2: goal id */

    CMD_COUNT
};

enum CommandSource {
    SOURCE_CONSOLE,         /* typed command */
    SOURCE_STOP_KEY,        /* Esc in the terminal */
    SOURCE_BUTTON,          /* start button */
    SOURCE_TASK             /* the drive or rotate task, or the terminal itself */
};

struct Command {
    enum CommandType type;
    enum CommandSource source;
    int32_t value;
    int32_t value2;
};

/* Creates the queue. Called from main() after osKernelInitialize(). */
void command_init(void);

/* Queues a command for the control task. Never blocks, so it can be called
 * from any task or interrupt. Returns false if the queue is full. */
bool command_send(const struct Command *cmd);

/* Control task only: waits up to timeoutTicks for the next command. */
bool command_receive(struct Command *cmd, uint32_t timeoutTicks);

#endif /* COMMAND_H */
