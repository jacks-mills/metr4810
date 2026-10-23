/**
 * @file    robot_state.c
 * @brief   Shared robot state.
 */
#include <string.h>

#include "FreeRTOS.h"
#include "task.h"
#include "robot_state.h"

struct State state;

void state_init(void)
{
    memset(&state, 0, sizeof state);
}

void state_snapshot(struct State *copy)
{
    /* The struct is small, so copying it with interrupts masked is quick. */
    taskENTER_CRITICAL();
    *copy = state;
    taskEXIT_CRITICAL();
}
