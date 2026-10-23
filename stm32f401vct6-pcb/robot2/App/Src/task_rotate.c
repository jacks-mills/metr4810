/**
 * @file    task_rotate.c
 * @brief   rotate task: runs the logic in rotate.c every SERVO_POLL_MS, and
 *          straight away when a goal arrives.
 *
 * Created by CubeMX ("As external").
 */
#include "cmsis_os.h"
#include "app.h"
#include "config.h"
#include "rotate.h"

#define ROTATE_QUEUE_LENGTH 4

static osMessageQueueId_t queue;

void rotate_init(void)
{
    queue = osMessageQueueNew(ROTATE_QUEUE_LENGTH, sizeof(struct RotateGoal),
                              &(osMessageQueueAttr_t){ .name = "rotate" });
}

bool rotate_send(const struct RotateGoal *goal)
{
    return queue != NULL && osMessageQueuePut(queue, goal, 0, 0) == osOK;
}

void TaskRotate(void *argument)
{
    (void)argument;

    rotate_logic_init(osKernelGetTickCount());
    uint32_t next = osKernelGetTickCount();

    for (;;) {
        struct RotateGoal goal;
        int32_t wait = (int32_t)(next - osKernelGetTickCount());

        if (osMessageQueueGet(queue, &goal, NULL, wait > 0 ? (uint32_t)wait : 0) == osOK) {
            rotate_logic_goal(&goal, osKernelGetTickCount());
            continue;
        }

        uint32_t now = osKernelGetTickCount();
        if ((int32_t)(now - next) >= 0) {
            next = now + SERVO_POLL_MS;
            rotate_logic_tick(now);
        }
    }
}
