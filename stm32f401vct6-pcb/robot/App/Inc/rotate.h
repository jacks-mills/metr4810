/**
 * @file    rotate.h
 * @brief   Rotate task: turns the base plate with the ST3215 servo.
 *
 * The control task sends goals with rotate_send(). Each goal carries an id;
 * when it ends, the rotate task sends EVT_ROTATE_RESULT (value: enum
 * RotateResult, value2: the id) back on the command queue. A new goal ends a
 * move in progress (reported as ROTATE_RESULT_STOPPED).
 *
 * The rotate task is the only one that talks to the servo, and it writes the
 * servo fields of the state (servoOnline, rotating, rotateAngle, ...).
 */
#ifndef ROTATE_H
#define ROTATE_H

#include <stdbool.h>
#include <stdint.h>

enum RotateGoalType {
    ROTATE_GOAL_TO,             /* angle, at speed % */
    ROTATE_GOAL_STOP,           /* hold where it is now */
    ROTATE_GOAL_FREE,           /* torque off: turn the plate by hand */
    ROTATE_GOAL_HOLD,           /* torque on, holding where it is */
    ROTATE_GOAL_CENTRE,         /* where it is now becomes 0 deg (saved in the servo) */
    ROTATE_GOAL_LIMITS          /* angle..angle2 become the servo's limits (saved) */
};

enum RotateResult {
    ROTATE_RESULT_DONE,         /* got there / calibration step done */
    ROTATE_RESULT_STOPPED,      /* stopped or replaced before getting there */
    ROTATE_RESULT_TIMEOUT,      /* didn't get there in time */
    ROTATE_RESULT_SERVO_ERROR,  /* the servo reported overload etc., or refused */
    ROTATE_RESULT_OFFLINE       /* the servo isn't answering */
};

struct RotateGoal {
    enum RotateGoalType type;
    int16_t angle;              /* tenths of a degree */
    int16_t angle2;
    uint8_t speed;              /* 0-100 % of SERVO_MAX_SPEED */
    uint32_t id;
};

/* task_rotate.c */
void rotate_init(void);                             /* creates the goal queue */
bool rotate_send(const struct RotateGoal *goal);    /* never blocks */
void TaskRotate(void *argument);

/* rotate.c: the logic, run by TaskRotate. No RTOS calls of its own. */
void rotate_logic_init(uint32_t now);
void rotate_logic_goal(const struct RotateGoal *goal, uint32_t now);
void rotate_logic_tick(uint32_t now);               /* every SERVO_POLL_MS */

/* tenths of a degree <-> servo steps (SERVO_CENTRE = 0 deg) */
int32_t rotate_angle_to_steps(int32_t tenths);
int32_t rotate_steps_to_angle(int32_t steps);

#endif /* ROTATE_H */
