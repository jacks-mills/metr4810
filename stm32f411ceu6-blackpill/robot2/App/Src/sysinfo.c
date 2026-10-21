/**
 * @file    sysinfo.c
 * @brief   Heap and stack figures for the debug panel.
 */
#include <stdio.h>
#include <string.h>

#include "FreeRTOS.h"
#include "task.h"
#include "log.h"
#include "sysinfo.h"

void sysinfo_read(struct SysInfo *info)
{
    static TaskStatus_t status[SYSINFO_MAX_TASKS];  /* static: keeps it off the caller's stack */

    info->heapFree = xPortGetFreeHeapSize();
    info->heapMinFree = xPortGetMinimumEverFreeHeapSize();
    info->logDropped = log_dropped();

    /* Walks every task's stack to find its high-water mark, so call it
     * rarely (the terminal does once a second, in debug mode only). */
    UBaseType_t count = uxTaskGetSystemState(status, SYSINFO_MAX_TASKS, NULL);

    /* sort by creation order so the list doesn't jump around */
    for (UBaseType_t i = 1; i < count; i++) {
        TaskStatus_t t = status[i];
        UBaseType_t j = i;
        while (j > 0 && status[j - 1].xTaskNumber > t.xTaskNumber) {
            status[j] = status[j - 1];
            j--;
        }
        status[j] = t;
    }

    info->taskCount = (uint8_t)count;
    for (UBaseType_t i = 0; i < count; i++) {
        snprintf(info->tasks[i].name, sizeof info->tasks[i].name, "%s", status[i].pcTaskName);
        info->tasks[i].stackFree = status[i].usStackHighWaterMark * sizeof(StackType_t);
    }
}
