/**
 * @file    task_drive.c
 * @brief   drive task: runs the logic in drive.c every DRIVE_PERIOD_MS,
 *          straight away when a goal arrives, and at the exact moment a
 *          pulse ends.
 *
 * Created by CubeMX ("As external").
 */
#include "cmsis_os.h"
#include "app.h"
#include "drive.h"

#define DRIVE_QUEUE_LENGTH  4

struct DriveMsg {
    enum DriveGoal goal;
    uint32_t pulseMs;
    uint32_t id;
};

static osMessageQueueId_t queue;

void drive_init(void)
{
    queue = osMessageQueueNew(DRIVE_QUEUE_LENGTH, sizeof(struct DriveMsg),
                              &(osMessageQueueAttr_t){ .name = "drive" });
}

bool drive_send(enum DriveGoal goal, uint32_t pulseMs, uint32_t id)
{
    struct DriveMsg msg = { goal, pulseMs, id };
    return queue != NULL && osMessageQueuePut(queue, &msg, 0, 0) == osOK;
}

void TaskDrive(void *argument)
{
    (void)argument;
    uint32_t next = osKernelGetTickCount();

    drive_logic_init(next);

    for (;;) {
        struct DriveMsg msg;

        /* wake for the next tick, or earlier if a pulse ends first */
        uint32_t due = next;
        uint32_t pulseEnd;
        if (drive_logic_pulse_end(&pulseEnd) && (int32_t)(pulseEnd - due) < 0)
            due = pulseEnd;
        int32_t wait = (int32_t)(due - osKernelGetTickCount());

        if (osMessageQueueGet(queue, &msg, NULL, wait > 0 ? (uint32_t)wait : 0) == osOK) {
            drive_logic_goal(msg.goal, msg.pulseMs, msg.id, osKernelGetTickCount());
            continue;
        }

        uint32_t now = osKernelGetTickCount();
        if ((int32_t)(now - due) >= 0) {
            if ((int32_t)(now - next) >= 0)
                next = now + DRIVE_PERIOD_MS;
            drive_logic_tick(now);
        }
    }
}
