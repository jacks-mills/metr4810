/**
 * @file    drive.h
 * @brief   Arm drive task: extends or retracts the arm to a limit switch.
 *
 * The control task sends goals with drive_send(). Each goal carries an id;
 * when it ends, the drive task sends EVT_DRIVE_RESULT (value: enum
 * DriveResult, value2: the id) back on the command queue. A new goal ends the
 * previous one (reported as DRIVE_RESULT_STOPPED).
 *
 * The drive task owns the motor and switches, and writes state.driveMotion,
 * state.atExtMin and state.atExtMax.
 */
#ifndef DRIVE_H
#define DRIVE_H

#include <stdbool.h>
#include <stdint.h>

enum DriveGoal {
    DRIVE_GOAL_STOP,
    DRIVE_GOAL_EXTEND,          /* until EXT_MAX */
    DRIVE_GOAL_RETRACT          /* until EXT_MIN */
};

enum DriveResult {
    DRIVE_RESULT_REACHED,       /* at the limit switch */
    DRIVE_RESULT_STOPPED,       /* stopped or replaced before getting there */
    DRIVE_RESULT_TIMEOUT,       /* took longer than DRIVE_TIMEOUT_MS */
    DRIVE_RESULT_SWITCH_ERROR   /* both limit switches pressed at once */
};

/* task_drive.c */
void drive_init(void);                              /* creates the goal queue */
bool drive_send(enum DriveGoal goal, uint32_t id);  /* never blocks */
void TaskDrive(void *argument);

/* drive.c: the logic, run by TaskDrive. No RTOS calls, so it can be tested
 * on a PC. */
void drive_logic_init(uint32_t now);
void drive_logic_goal(enum DriveGoal goal, uint32_t id, uint32_t now);
void drive_logic_tick(uint32_t now);                /* every DRIVE_PERIOD_MS */

#define DRIVE_PERIOD_MS     10

#endif /* DRIVE_H */
