/**
 * @file    command.c
 * @brief   Command queue for the control task.
 */
#include "cmsis_os.h"
#include "command.h"

#define COMMAND_QUEUE_LENGTH    16

static osMessageQueueId_t queue;

void command_init(void)
{
    queue = osMessageQueueNew(COMMAND_QUEUE_LENGTH, sizeof(struct Command),
                              &(osMessageQueueAttr_t){ .name = "commands" });
}

bool command_send(const struct Command *cmd)
{
    /* timeout 0: never blocks, and is allowed from an interrupt */
    return queue != NULL && osMessageQueuePut(queue, cmd, 0, 0) == osOK;
}

bool command_receive(struct Command *cmd, uint32_t timeoutTicks)
{
    return osMessageQueueGet(queue, cmd, NULL, timeoutTicks) == osOK;
}
